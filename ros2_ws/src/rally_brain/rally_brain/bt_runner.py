"""rally_brain/rally_brain/bt_runner.py — py_trees 세션 트리 실행 (모드 1 → (3) → 5 → 7)
담당: 조정묵

  /rally/session/run (RunSession action 서버)  대시보드·음성이 세션 시작 요청
     goal.modes 가 비면 [1, 3, 5, 7] (skills.yaml mode3_challenge.enabled 가 false 면 3 제외)
     goal.trials_per_mode 가 0 이면 파라미터 trials_per_mode (기본 5)
  /rally/brain/context      구독 — 트리의 조건(안전·손)은 모두 이 상황판을 본다 (context_builder 가 발행)
  /rally/brain/trial_result 발행 — 시도마다 (Reliable)
  /rally/brain/state        발행 — 지금 실행 중인 트리 블록 이름, 2 Hz (대시보드용)

세션 시작 조건: readiness.yaml 의 session 그룹(안전·로봇·인식·음성)이 준비돼야 goal 을 받는다 (NFR-08).
  가짜 스킬 서버로 흐름만 볼 때는 check_readiness:=false.
세션 중단: 안전 STOP (상황판 기준) 또는 상황판이 context_timeout_s(기본 1 s) 넘게 안 옴 → abort / 취소 요청 → canceled.
  모두 진행 중인 스킬 goal 도 취소한다. (실제 정지는 로봇 쪽 safety_monitor 가 따로 한다)
트리는 execute 콜백 안에서 tick_hz(기본 10 Hz)로 돈다. 스킬 action 은 비동기라 tick 이 막히지 않는다.
시도가 기록될 때마다 rule_layer 가 다음 행동을 고른다 (/rally/voice/intent 의 tired·harder·easier 도 여기서 소비).
  harder/easier → 모드별 난이도 단계만 바꾼다 (값 반영은 각 모드 서브트리). suggest_rest → 지금은 로그만 (TTS 노드 전).
  ask_llm → llm_advisor 가 생기기 전까지 continue 로 대체.
모드 전환: /rally/brain/set_mode (SetMode, 대시보드) · 음성 next_mode (+ 슬롯 mode, INTERFACES 2.2).
  요청은 바로 답하고, 실제 전환은 로봇 스킬이 돌고 있지 않을 때 한다 (건네는 도중에 끊지 않음).
  고른 모드부터 원래 순서대로 이어 간다 (session_tree.modes_after_switch). 슬롯 없는 next_mode = 다음 모드.
TODO: 준비가 도중에 깨지면 중단 + 관측 자세 대기, 음성 start_session(+모드), suggest_rest → TTS·일시정지
"""
from __future__ import annotations

import threading
import time
import uuid

import py_trees
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

from rally_brain import rule_layer
from rally_brain.config import load_skills
from rally_brain.behaviours import SkillCall
from rally_brain.trees.session_tree import MODE_NAMES, Session, build_session_tree, modes_after_switch
from rally_bringup.readiness import ReadinessChecker
from rally_interfaces.action import RunSession
from rally_interfaces.msg import SafetyStatus, SceneContext, TrialResult, VoiceIntent
from rally_interfaces.srv import SetMode

CHECK_PERIOD_S = 1.0
STATE_PERIOD_S = 0.5
ALL_MODES = [TrialResult.MODE_PUSH_BALL, TrialResult.MODE_CATCH_PAD, TrialResult.MODE_HANDOVER, TrialResult.MODE_RESIST]


RULE_INTENTS = ("tired", "harder", "easier")   # 규칙 레이어가 소비하는 음성 intent


def mode3_enabled(skills_cfg: dict) -> bool:
    """skills.yaml mode3_challenge.enabled. 없으면 True (기획 기본값)."""
    return bool((skills_cfg.get("mode3_challenge") or {}).get("enabled", True))


def default_modes(include_mode3: bool) -> list[int]:
    return [m for m in ALL_MODES if include_mode3 or m != TrialResult.MODE_CATCH_PAD]


class BtRunner(Node):
    def __init__(self):
        super().__init__("rally_bt_runner")
        self.declare_parameter("check_readiness", True)
        self.declare_parameter("trials_per_mode", 5)
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("context_timeout_s", 1.0)
        self.check_readiness = bool(self.get_parameter("check_readiness").value)
        self.tick_period = 1.0 / float(self.get_parameter("tick_hz").value)
        self.context_timeout_s = float(self.get_parameter("context_timeout_s").value)

        self.readiness = ReadinessChecker(self, ["session"]) if self.check_readiness else None
        self.session_ready = not self.check_readiness
        self._last_missing: list[str] | None = None
        self._context: SceneContext | None = None
        self._context_rx = 0.0          # 마지막 상황판 수신 시각 (monotonic)
        self._state_name = "idle"       # tick 스레드에서만 갱신, 타이머는 읽기만
        self._busy = threading.Lock()   # 세션은 한 번에 하나
        self._skill_clients: dict[str, ActionClient] = {}
        self._client_group = ReentrantCallbackGroup()
        self._skills_cfg = load_skills()
        self._rule_cfg = rule_layer.rule_config(self._skills_cfg)
        self._pending_intent: str | None = None
        self._session: Session | None = None
        self._pending_switch: tuple[int | None, str] | None = None   # (목표 모드 또는 None=다음, 요청한 곳)

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.create_subscription(SceneContext, "/rally/brain/context", self._on_context, qos_profile_sensor_data)
        self.create_subscription(VoiceIntent, "/rally/voice/intent", self._on_intent, reliable)
        self._trial_pub = self.create_publisher(TrialResult, "/rally/brain/trial_result", reliable)
        self._state_pub = self.create_publisher(String, "/rally/brain/state", reliable)
        self.create_service(SetMode, "/rally/brain/set_mode", self._on_set_mode,
                            callback_group=MutuallyExclusiveCallbackGroup())
        if self.readiness is not None:
            self.create_timer(CHECK_PERIOD_S, self._check_ready)
        self.create_timer(STATE_PERIOD_S, self._publish_state)
        self._server = ActionServer(
            self, RunSession, "/rally/session/run", self._execute,
            goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(
            "bt_runner 대기 — /rally/session/run" + ("" if self.check_readiness else " (준비 확인 끔)"))

    # ---------- 상태 ----------
    def _on_context(self, msg: SceneContext) -> None:
        self._context = msg
        self._context_rx = time.monotonic()

    def _context_stale(self, since: float) -> bool:
        """상황판이 context_timeout_s 넘게 안 왔으면 True. since = 세션 시작 (그 전 수신은 인정)."""
        return time.monotonic() - max(self._context_rx, since) > self.context_timeout_s

    def _on_intent(self, msg: VoiceIntent) -> None:
        # 상황판의 last_intent 는 3 s 만 남아 시도 끝까지 못 버틴다 → 규칙용 intent 는 다음 판단까지 들고 있는다
        if msg.intent in RULE_INTENTS:
            self._pending_intent = msg.intent
        elif msg.intent == "next_mode":
            slots = dict(zip(msg.slot_keys, msg.slot_values))
            try:
                target = int(slots["mode"]) if "mode" in slots else None
            except ValueError:
                self.get_logger().warn(f"음성 next_mode 슬롯 mode={slots['mode']!r} 를 모드 번호로 못 읽음 — 무시")
                return
            ok, message = self._request_switch(target, "음성")
            (self.get_logger().info if ok else self.get_logger().warn)(f"[모드 전환] 음성 — {message}")

    # ---------- 모드 전환 ----------
    def _on_set_mode(self, request: SetMode.Request, response: SetMode.Response) -> SetMode.Response:
        response.success, response.message = self._request_switch(request.mode, "set_mode")
        (self.get_logger().info if response.success else self.get_logger().warn)(
            f"[모드 전환] set_mode — {response.message}")
        return response

    def _request_switch(self, target: int | None, source: str) -> tuple[bool, str]:
        """요청 확인만 하고 접수. 실제 전환은 tick 루프의 _apply_switch."""
        session = self._session
        if session is None:
            return False, "진행 중인 세션 없음"
        if target is not None:
            if target not in MODE_NAMES:
                return False, f"없는 모드 {target} (1·3·5·7)"
            if target == TrialResult.MODE_CATCH_PAD and not mode3_enabled(self._skills_cfg):
                return False, "모드 3 캐치볼은 꺼져 있음 (skills.yaml mode3_challenge.enabled)"
            if target == session.current_mode:
                return False, f"이미 모드 {target} {MODE_NAMES[target]} 진행 중"
        self._pending_switch = (target, source)
        what = "다음 모드" if target is None else f"모드 {target} {MODE_NAMES[target]}"
        return True, f"{what}(으)로 전환 접수 — 로봇 동작이 끝나면 바뀜"

    @staticmethod
    def _skill_running(tree: py_trees.trees.BehaviourTree) -> bool:
        return any(isinstance(b, SkillCall) and b.status == py_trees.common.Status.RUNNING
                   for b in tree.root.iterate())

    def _apply_switch(self, tree: py_trees.trees.BehaviourTree, session: Session,
                      n: int) -> py_trees.trees.BehaviourTree:
        """접수된 전환이 있고 로봇 스킬이 돌고 있지 않으면 남은 순서로 트리를 새로 만든다."""
        if self._pending_switch is None or self._skill_running(tree):
            return tree
        (target, source), self._pending_switch = self._pending_switch, None
        modes = modes_after_switch(session.modes, session.current_mode, target)
        tree.root.stop(py_trees.common.Status.INVALID)   # 손 대기 등 진행 중 블록 정리 (스킬은 돌고 있지 않음)
        self.get_logger().info(f"[모드 전환] {source}: 모드 {session.current_mode} → 남은 순서 {modes}")
        session.current_mode = None
        return py_trees.trees.BehaviourTree(build_session_tree(session, modes, n))

    def _after_trial(self, session: Session, trial: TrialResult) -> None:
        intent, self._pending_intent = self._pending_intent, None
        d = rule_layer.decide(self._context, session.trials_for_rules(trial.mode), intent, self._rule_cfg)
        log = self.get_logger()
        if d.action in ("harder", "easier"):
            old = session.difficulty.get(trial.mode, 0)
            new = rule_layer.apply_difficulty(old, d.action, self._rule_cfg)
            session.set_difficulty(trial.mode, new)
            log.info(f"[규칙] {d.reason} — 모드 {trial.mode} 난이도 {old} → {new}"
                     + ("" if new != old else " (한계)"))
        elif d.action == "ask_llm":
            log.info(f"[규칙] {d.reason} — llm_advisor 전이라 continue")
        elif d.action == "suggest_rest":
            log.warn(f"[규칙] {d.reason} — TTS 노드 전이라 로그만")
        else:
            log.info(f"[규칙] {d.reason}")

    def _check_ready(self) -> None:
        missing = self.readiness.missing()
        self.session_ready = not missing
        if missing != self._last_missing:
            if missing:
                self.get_logger().warn(f"세션 시작 불가 — 준비 안 됨: {'; '.join(missing)}")
            else:
                self.get_logger().info("세션 시작 조건 충족")
            self._last_missing = missing

    def _publish_state(self) -> None:
        self._state_pub.publish(String(data=self._state_name))

    def _action_client(self, action_type: type, name: str) -> ActionClient:
        if name not in self._skill_clients:
            self._skill_clients[name] = ActionClient(self, action_type, name, callback_group=self._client_group)
        return self._skill_clients[name]

    # ---------- RunSession ----------
    def _on_goal(self, goal: RunSession.Goal) -> GoalResponse:
        if self._busy.locked():
            self.get_logger().warn("세션 요청 거절 — 이미 진행 중")
            return GoalResponse.REJECT
        if not self.session_ready:
            self.get_logger().warn(f"세션 요청 거절 — 준비 안 됨: {'; '.join(self._last_missing or [])}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle) -> RunSession.Result:
        with self._busy:
            return self._run_session(goal_handle)

    def _run_session(self, goal_handle) -> RunSession.Result:
        goal: RunSession.Goal = goal_handle.request
        modes = list(goal.modes) or default_modes(mode3_enabled(self._skills_cfg))
        n = goal.trials_per_mode or int(self.get_parameter("trials_per_mode").value)
        session = Session(node=self, session_id=str(uuid.uuid4()), get_context=lambda: self._context,
                          publish=self._trial_pub.publish, action_client=self._action_client,
                          after_trial=self._after_trial)
        tree = py_trees.trees.BehaviourTree(build_session_tree(session, modes, n))
        self._session, self._pending_switch = session, None
        started = time.monotonic()
        self.get_logger().info(f"세션 시작 {session.session_id[:8]} — 모드 {modes}, 모드마다 {n}회")

        outcome = "aborted"
        last_feedback = 0.0
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    tree.root.stop(py_trees.common.Status.INVALID)   # 실행 중 블록 terminate → 스킬 goal 취소
                    outcome = "canceled"
                    break
                ctx = self._context
                if self._context_stale(started):
                    tree.root.stop(py_trees.common.Status.INVALID)   # 실행 중 블록 terminate → 스킬 goal 취소
                    self.get_logger().error(f"/rally/brain/context 가 {self.context_timeout_s} s 넘게 안 옴 — 세션 중단")
                    break
                if ctx is not None and ctx.safety.level == SafetyStatus.LEVEL_STOP:
                    tree.root.stop(py_trees.common.Status.INVALID)   # 실행 중 블록 terminate → 스킬 goal 취소
                    self.get_logger().warn(f"안전 정지({ctx.safety.reason}) — 세션 중단")
                    break
                tree = self._apply_switch(tree, session, n)
                tree.tick()
                tip = tree.tip()
                self._state_name = tip.name if tip is not None else "idle"
                status = tree.root.status
                if status == py_trees.common.Status.SUCCESS:
                    outcome = "succeeded"
                    break
                if status == py_trees.common.Status.FAILURE:
                    self.get_logger().error(f"세션 실패 — {tree.tip().name if tree.tip() else '?'}")
                    break
                if ctx is not None and time.monotonic() - last_feedback >= 1.0:
                    goal_handle.publish_feedback(RunSession.Feedback(context=ctx))
                    last_feedback = time.monotonic()
                time.sleep(self.tick_period)
        finally:
            self._state_name = "idle"
            self._session = self._pending_switch = None

        getattr(goal_handle, {"succeeded": "succeed", "canceled": "canceled", "aborted": "abort"}[outcome])()
        result = RunSession.Result(session_id=session.session_id, total_trials=len(session.trials),
                                   overall_success_rate=session.success_rate())
        self.get_logger().info(
            f"세션 종료 {session.session_id[:8]} ({outcome}) — 시도 {result.total_trials}회, "
            f"성공률 {result.overall_success_rate:.0%}")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = BtRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
