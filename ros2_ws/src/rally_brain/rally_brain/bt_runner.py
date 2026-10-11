"""rally_brain/rally_brain/bt_runner.py — py_trees 세션 트리 실행 (모드 1 → (3) → 5 → 7)
담당: 조정묵

  /rally/session/run (RunSession action 서버)  대시보드·음성이 세션 시작 요청
     goal.modes 가 비면 [1, 3, 5, 7] (skills.yaml mode3_challenge.enabled 가 false 면 3 제외)
     goal.trials_per_mode 가 0 이면 파라미터 trials_per_mode (기본 5) — 모드 1 은 거치대 공 개수(mode1.balls_per_cycle)로 고정 (session_tree.TRIAL_COUNTS)
  /rally/brain/context      구독 — 트리의 조건(안전·손)은 모두 이 상황판을 본다 (context_builder 가 발행)
  /rally/brain/trial_result 발행 — 시도마다 (Reliable)
  /rally/brain/state        발행 — 지금 실행 중인 트리 블록 이름, 2 Hz (대시보드용)

세션 시작 조건: readiness.yaml 의 session 그룹(안전·로봇·인식·음성)이 준비돼야 goal 을 받는다 (NFR-08).
  가짜 스킬 서버로 흐름만 볼 때는 check_readiness:=false.
세션 중단: 안전 STOP (상황판 기준) 또는 상황판이 context_timeout_s(기본 1 s) 넘게 안 옴 → abort / 취소 요청 → canceled.
  모두 진행 중인 스킬 goal 도 취소한다. (실제 정지는 로봇 쪽 safety_monitor 가 따로 한다)
  예외 — 모드 1 손 근접 STOP 대기 (#98, hand_stop.py): 로봇이 서 있을 때 걸린 손 근접 STOP(비래치)은 세션을 끝내지 않고
  트리를 멈춘 채 기다리다가, NORMAL 이 hand_stop_resume_hold_s 이어지면 같은 시도를 이어 간다. 로봇이 움직이는 중이면 기존대로 종료.
트리는 execute 콜백 안에서 tick_hz(기본 10 Hz)로 돈다. 스킬 action 은 비동기라 tick 이 막히지 않는다.
시도가 기록될 때마다 rule_layer 가 다음 행동을 고른다 (/rally/voice/intent 의 tired·harder·easier 도 여기서 소비).
  harder/easier → 구현된 모드 레시피의 난이도 단계만 바꾼다. suggest_rest → 현재 시도 후 휴식.
  ask_llm → llm_advisor 가 생기기 전까지 continue 로 대체.
모드 전환: /rally/brain/set_mode (SetMode, 대시보드) · 음성 next_mode (+ 슬롯 mode, INTERFACES 2.2).
  요청은 바로 답하고, 실제 전환은 시도와 시도 사이(또는 손 대기 중)에만 한다 — 로봇이 물체를 건네는 도중에 끊지 않음.
  고른 모드부터 원래 순서대로 이어 간다 (session_tree.modes_after_switch). 슬롯 없는 next_mode = 다음 모드.
  같은 발화(header.stamp 같음)는 한 번만 처리. 안전 STOP 중 요청은 거절. 대기 중 요청은 새 요청이 덮어쓴다.
  요청 저장·소비는 _switch_lock 으로 (서비스·구독 콜백 스레드 ↔ tick 스레드).
TODO: 준비가 도중에 깨지면 중단 + 관측 자세 대기, 음성 start_session(+모드)
"""
from __future__ import annotations

import threading
import time
from typing import Any
import uuid
import os

import py_trees
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

from rally_brain import hand_stop, rule_layer
from rally_brain.config import load_skills
from rally_brain.behaviours import SkillCall
from rally_brain.knowledge import ontology
from rally_brain.speech import phrase_problem
from rally_brain.session_control import SessionControl
from rally_brain.proposal_control import ProposalControl, REST_PROMPT
from rally_brain.trees.session_tree import MODE_NAMES, Session, build_session_tree, modes_after_switch
from rally_bringup.readiness import ReadinessChecker
from rally_interfaces.action import RunSession
from rally_interfaces.msg import SafetyStatus, SceneContext, TrialResult, VoiceIntent
from rally_interfaces.srv import SetMode, Speak

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
        self._srv_clients: dict[str, Any] = {}
        self._client_group = ReentrantCallbackGroup()
        self._skills_cfg = load_skills()
        self._rule_cfg = rule_layer.rule_config(self._skills_cfg)
        if problems := rule_layer.check_graph(self._rule_cfg):   # 파일 읽기 실패도 여기서 문제로 나온다
            # 세션 도중 after_trial 에서 터지지 않게, 켤 때 막는다 (#60 리뷰 3)
            for p in problems:
                self.get_logger().fatal(f"ontology 문제: {p}")
            raise RuntimeError(f"ontology 검사 실패 {len(problems)}건 — knowledge/ontology.jsonld 확인")
        self._graph = ontology.load()
        self._pending_intent: str | None = None
        self._session: Session | None = None
        self._pending_switch: tuple[int | None, str] | None = None   # (목표 모드 또는 None=다음, 요청한 곳)
        self.control = SessionControl()
        self.proposals = ProposalControl()
        self._proposal_cancelled = False
        self._switch_lock = self.control.lock                         # 세션·전환·휴식의 공통 lock
        self._safety_abort = False                                    # STOP 한 번 수신하면 세션 종료까지 유지
        self._hand_cfg = hand_stop.config(self._skills_cfg)
        self._hand_stop_seen: str | None = None                      # 기다릴 수 있는 손 STOP 을 받음 (tick 이 소비)
        self._hand_wait: hand_stop.HandStopWait | None = None        # 손 근접 STOP 대기 중 (tick 스레드만)
        self._last_intent_stamp: tuple[int, int] = (0, 0)             # 마지막으로 처리한 발화

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
        self._speak = self.create_client(Speak, "/rally/voice/speak", callback_group=self._client_group)
        self._server = ActionServer(
            self, RunSession, "/rally/session/run", self._execute,
            goal_callback=self._on_goal, cancel_callback=self._cancel_session,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.declare_parameter("test_rest_proposals", False)
        if self.get_parameter("test_rest_proposals").value:
            from std_srvs.srv import Trigger
            self.create_service(Trigger, "/rally/brain/test/propose_rest", self._test_proposal,
                                callback_group=MutuallyExclusiveCallbackGroup())
        self.declare_parameter("dashboard_control", bool(os.environ.get("REDIS_URL")))
        self._dashboard_bridge = None
        if self.get_parameter("dashboard_control").value:
            from rally_brain.dashboard_bridge import DashboardBridge
            self._dashboard_bridge = DashboardBridge(self)
        self.get_logger().info(
            "bt_runner 대기 — /rally/session/run" + ("" if self.check_readiness else " (준비 확인 끔)"))

    # ---------- 상태 ----------
    def _on_context(self, msg: SceneContext) -> None:
        with self._switch_lock:
            self._context = msg
            self._context_rx = time.monotonic()
            if self._session is not None and msg.safety.level == SafetyStatus.LEVEL_STOP:
                if hand_stop.waitable(msg.safety.reason, self._session.current_mode, self._hand_cfg):
                    self._hand_stop_seen = msg.safety.reason   # 로봇이 움직이는지는 tick 이 보고 대기·종료를 정한다
                else:
                    self._safety_abort = True
                self.proposals.invalidate()

    def _context_stale(self, since: float) -> bool:
        """상황판이 context_timeout_s 넘게 안 왔으면 True. since = 세션 시작 (그 전 수신은 인정)."""
        return time.monotonic() - max(self._context_rx, since) > self.context_timeout_s

    def _on_intent(self, msg: VoiceIntent) -> None:
        # 같은 발화가 다시 오면(header.stamp 같음) 무시 — "다음" 한 번에 모드 두 칸 건너뛰지 않게 (#51 리뷰 1)
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp != (0, 0):
            if stamp == self._last_intent_stamp:
                return
            self._last_intent_stamp = stamp
        # 상황판의 last_intent 는 3 s 만 남아 시도 끝까지 못 버틴다 → 규칙용 intent 는 다음 판단까지 들고 있는다
        if msg.intent in ("pause", "tired", "resume"):
            if msg.is_emergency:
                return   # 비상어 처리는 기존 safety 경로. resume으로 사용하지 않음
            if msg.intent == "resume":
                with self._switch_lock:
                    stamp_ns = stamp[0] * 1_000_000_000 + stamp[1]
                    if stamp_ns <= self.control.paused_stamp_ns:
                        ok, message = False, "휴식 시작 이후의 새 재개 요청이 필요합니다"
                    else:
                        ok, message = self._resume(self.control.session_id, self.control.pause_id)
            else:
                ok, message = self._pause(f"음성 {msg.intent}")
            if ok:
                self.get_logger().info(f"[휴식] {message}")
            else:
                self.get_logger().warn(f"[휴식] {message}")
            if not ok:
                self._say("지금은 이어갈 수 없어요. 화면을 확인해 주세요")
        elif msg.intent in RULE_INTENTS:
            with self._switch_lock:
                if self.control.phase in ("paused", "pause_pending") or self.control.hand_wait:
                    self._say("쉬는 동안에는 난이도를 그대로 유지할게요")
                elif self._session is not None:
                    self.proposals.invalidate()
                    self._pending_intent = msg.intent
        elif msg.intent == "next_mode":
            slots = dict(zip(msg.slot_keys, msg.slot_values))
            try:
                target = int(slots["mode"]) if "mode" in slots else None
            except ValueError:
                self.get_logger().warn(f"음성 next_mode 슬롯 mode={slots['mode']!r} 를 모드 번호로 못 읽음 — 무시")
                return
            ok, message = self._request_switch(target, "음성")
            if ok:
                self.get_logger().info(f"[모드 전환] 음성 — {message}")
            else:
                self.get_logger().warn(f"[모드 전환] 음성 — {message}")

    # ---------- 모드 전환 ----------
    def _on_set_mode(self, request: SetMode.Request, response: SetMode.Response) -> SetMode.Response:
        response.success, response.message = self._request_switch(request.mode, "set_mode")
        if response.success:
            self.get_logger().info(f"[모드 전환] set_mode — {response.message}")
        else:
            self.get_logger().warn(f"[모드 전환] set_mode — {response.message}")
        return response

    def _request_switch(self, target: int | None, source: str) -> tuple[bool, str]:
        """요청 확인만 하고 접수. 실제 전환은 tick 루프의 _apply_switch. 확인+저장을 한 lock 안에서 (#51 리뷰 2)."""
        with self._switch_lock:
            session = self._session
            if session is None:
                return False, "진행 중인 세션 없음"
            if self.control.phase in ("pause_pending", "paused"):
                return False, "휴식 요청 중에는 모드를 유지합니다 — 재개 후 요청해 주세요"
            ctx = self._context
            if self.control.hand_wait:
                return False, "손 근접 정지 대기 중 — 손이 빠지고 이어진 뒤 요청해 주세요"
            if ctx is not None and ctx.safety.level == SafetyStatus.LEVEL_STOP:
                return False, "안전 정지 중 — 세션이 끝나므로 전환하지 않음"
            if target is not None:
                if target not in MODE_NAMES:
                    return False, f"없는 모드 {target} (1·3·5·7)"
                if target == TrialResult.MODE_CATCH_PAD and not mode3_enabled(self._skills_cfg):
                    return False, "모드 3 캐치볼은 꺼져 있음 (skills.yaml mode3_challenge.enabled)"
                if target == session.current_mode:
                    return False, f"이미 모드 {target} {MODE_NAMES[target]} 진행 중"
            replaced = self._pending_switch is not None
            self.proposals.invalidate()
            self._pending_switch = (target, source)
        if target is None and not modes_after_switch(session.modes, session.current_mode, None):
            what = "다음 모드 없음(마지막 모드) — 전환하면 세션이 끝남"
        else:
            what = ("다음 모드" if target is None else f"모드 {target} {MODE_NAMES[target]}") + "(으)로 전환"
        return True, f"{what} 접수 — 지금 시도가 끝나면 바뀜" + (" (앞선 요청은 취소)" if replaced else "")

    @staticmethod
    def _skill_running(tree: py_trees.trees.BehaviourTree) -> bool:
        return any(isinstance(b, SkillCall) and b.status == py_trees.common.Status.RUNNING
                   for b in tree.root.iterate())

    def _apply_switch(self, tree: py_trees.trees.BehaviourTree, session: Session,
                      n: int) -> py_trees.trees.BehaviourTree:
        """접수된 전환이 있고 시도 경계(시도 사이·손 대기)이면 남은 순서로 트리를 새로 만든다.
        스킬 하나가 끝났어도 시도가 안 끝났으면(예: GIVE 뒤 RECEIVE 전) 기다린다 (#51 리뷰 3)."""
        with self._switch_lock:
            if (self.control.phase in ("pause_pending", "paused") or self._pending_switch is None
                    or session.trial_active or self._skill_running(tree)):
                return tree
            (target, source), self._pending_switch = self._pending_switch, None
        modes = modes_after_switch(session.modes, session.current_mode, target)
        tree.root.stop(py_trees.common.Status.INVALID)   # 손 대기 등 진행 중 블록 정리 (스킬은 돌고 있지 않음)
        self.get_logger().info(f"[모드 전환] {source}: 모드 {session.current_mode} → 남은 순서 {modes}")
        session.current_mode = None
        return py_trees.trees.BehaviourTree(build_session_tree(session, modes, n))

    def _after_trial(self, session: Session, trial: TrialResult) -> None:
        with self._switch_lock:
            if self.control.phase in ("pause_pending", "paused", "ended") or self._safety_abort:
                self._pending_intent = None
                return   # 휴식 전후 난이도 보존. 중단 경로의 기록으로 난이도를 바꾸지 않음
            self._apply_trial_rule(session, trial)

    def _apply_trial_rule(self, session: Session, trial: TrialResult) -> None:
        intent, self._pending_intent = self._pending_intent, None
        d = rule_layer.decide(self._context, session.trials_for_rules(trial.mode), intent, self._rule_cfg, self._graph)
        log = self.get_logger()
        if d.action in ("harder", "easier"):
            if trial.mode not in (TrialResult.MODE_PUSH_BALL, TrialResult.MODE_RESIST):
                log.info(f"[규칙] 모드 {trial.mode} 난이도 레시피 없음 — 기존 값 유지")
                if intent in ("harder", "easier"):
                    self._say("이 훈련은 아직 난이도를 바꿀 수 없어요")
                return
            old = session.difficulty.get(trial.mode, 0)
            new = rule_layer.apply_difficulty(old, d.action, self._rule_cfg)
            if trial.mode == TrialResult.MODE_PUSH_BALL:
                new = max(0, new)  # 모드 1의 가장 쉬운 개발 단계는 0
            if old != new:
                self.proposals.invalidate()
            session.set_difficulty(trial.mode, new)
            log.info(f"[규칙] {d.reason} — 모드 {trial.mode} 난이도 {old} → {new}"
                     + ("" if new != old else " (한계)"))
        elif d.action == "ask_llm":
            log.info(f"[규칙] {d.reason} — llm_advisor 전이라 continue")
        elif d.action == "suggest_rest":
            self._pause(d.reason)
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
        with self._switch_lock:
            self._proposal_snapshot()
            phase = self.control.phase
            name = ("hand_stop_wait" if self.control.hand_wait
                    else phase if phase in ("pause_pending", "paused") else self._state_name)
        self._state_pub.publish(String(data=name))

    # ---------- 휴식·재개: 음성/대시보드 공통 ----------
    def _pause(self, reason: str) -> tuple[bool, str]:
        with self._switch_lock:
            if self._session is None or self._safety_abort:
                return False, "진행 중인 세션 없음 또는 안전 중단 중"
            ok, message = self.control.pause(reason)
            if ok:
                self.proposals.invalidate()
                self._pending_switch = self._pending_intent = None
            return ok, message

    def _allow_trial(self) -> bool:
        with self._switch_lock:
            if self._safety_abort or self._context_stale(0.0):
                return False
            was_pending = self.control.phase == "pause_pending"
            allowed = self.control.at_boundary(self.get_clock().now().nanoseconds)
            if was_pending:
                self.get_logger().info("[휴식] 현재 시도 완료 — 휴식 중, 난이도 유지")
                self._say("편하게 쉬세요. 준비되면 다시 하자고 말씀해 주세요")
            return allowed

    def _resume_blocked(self) -> str:
        if self._session is None or self._safety_abort:
            return "세션이 없거나 안전 중단 중입니다"
        if self._context is None or self._context_stale(0.0):
            return "안전 상황판 수신이 오래됐습니다"
        if self.control.hand_wait:
            return "손 근접 정지 대기 중입니다 — 손이 빠지면 이어집니다"
        if self._context.safety.level == SafetyStatus.LEVEL_STOP:
            return "안전 정지 중입니다 — 운영자 확인이 필요합니다"
        missing = self.readiness.missing() if self.readiness is not None else []
        return "준비 안 됨: " + "; ".join(missing) if missing else ""

    def _resume(self, session_id: str, pause_id: str) -> tuple[bool, str]:
        with self._switch_lock:
            ok, message = self.control.resume(session_id, pause_id, self._resume_blocked())
            if ok:
                self.get_logger().info(f"[휴식] {message}")
                self._say("같은 난이도로 이어갈게요")
            return ok, message

    def _cancel_session(self, _):
        with self._switch_lock:
            self._proposal_cancelled = True
            self.proposals.invalidate()
        return CancelResponse.ACCEPT

    def _proposal_context(self):
        session = self._session
        return (self.control.session_id, session.current_mode,
                session.difficulty.get(session.current_mode, 0)) if session else None

    def _proposal_blocked(self):
        return (self._resume_blocked() or self.control.phase != "running"
                or self._proposal_cancelled or self._pending_switch is not None
                or self._pending_intent is not None)

    def _proposal_snapshot(self):
        return self.proposals.snapshot(self._proposal_context(), bool(self._proposal_blocked()))

    def _test_proposal(self, request, response):
        with self._switch_lock:
            if self._proposal_blocked():
                response.success, response.message = False, "제안할 수 있는 훈련 상태가 아닙니다"
            else:
                response.success, response.message = self.proposals.propose(self._proposal_context())
                if response.success:
                    self._say(REST_PROMPT)
            return response

    def _respond_proposal(self, session_id, proposal_id, decision):
        with self._switch_lock:
            blocked = bool(self._proposal_blocked())
            if session_id != self.control.session_id:
                return False, "이전 세션의 요청입니다"
            ok, message = self.proposals.respond(self._proposal_context(), proposal_id, decision, blocked)
            if ok and decision == "accept":
                ok, message = self._pause("휴식 제안 수락")
            return ok, message

    def control_snapshot(self) -> dict:
        with self._switch_lock:
            session = self._session
            blocked = self._resume_blocked()
            return {
                "session_id": self.control.session_id, "phase": self.control.phase,
                "pause_id": self.control.pause_id, "reason": self.control.reason,
                "mode": session.current_mode if session else None,
                "difficulty": session.difficulty.get(session.current_mode, 0) if session else None,
                "trial_index": len(session.trials) if session else 0,
                "can_resume": self.control.phase == "paused" and not blocked,
                "blocked_reason": blocked, "updated_at": time.time(),
                "hand_wait": self.control.hand_wait,
                "bt_node": self._state_name,
                "mode3_enabled": mode3_enabled(self._skills_cfg),
                "proposal": self._proposal_snapshot(),
            }

    def _action_client(self, action_type: type, name: str) -> ActionClient:
        if name not in self._skill_clients:
            self._skill_clients[name] = ActionClient(self, action_type, name, callback_group=self._client_group)
        return self._skill_clients[name]

    def _service_client(self, srv_type: type, name: str):
        if name not in self._srv_clients:
            self._srv_clients[name] = self.create_client(srv_type, name, callback_group=self._client_group)
        return self._srv_clients[name]

    def _say(self, text: str) -> None:
        """TTS 안내 (기다리지 않음). tts_node 가 없거나 거절해도 로그만 — 안내가 없어도 세션은 계속된다.
        말하면 안 되는 문구(비상어)는 보내지 않는다 — tts_node 도 같은 기준으로 거절한다 (#53)."""
        if (why := phrase_problem(text)) is not None:
            self.get_logger().error(f"[안내] 보내지 않음 ({why}): {text}")
            return
        if not self._speak.service_is_ready():
            self.get_logger().warn(f"[안내] tts 없음 — 못 함: {text}")
            return
        self.get_logger().info(f"[안내] {text}")
        future = self._speak.call_async(Speak.Request(text=text, block_mic=True))
        future.add_done_callback(lambda f: self._on_spoken(f, text))

    def _on_spoken(self, future, text: str) -> None:
        """tts_node 가 말하지 않았으면(done=false) 로그 (#53 요청)."""
        try:
            res = future.result()
        except Exception as e:  # noqa: BLE001 — 서비스가 사라지는 등
            self.get_logger().warn(f"[안내] tts 응답 실패 ({type(e).__name__}): {text}")
            return
        if res is not None and not res.done:
            self.get_logger().warn(f"[안내] tts 가 말하지 않음(done=false): {text}")

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
                          skills_cfg=self._skills_cfg, say=self._say, after_trial=self._after_trial,
                          service_client=self._service_client, allow_trial=self._allow_trial)
        tree = py_trees.trees.BehaviourTree(build_session_tree(session, modes, n))
        with self._switch_lock:
            self._session, self._pending_switch = session, None
            self._pending_intent = None
            self._safety_abort = False
            self._hand_stop_seen = self._hand_wait = None
            self._proposal_cancelled = False
            self.proposals = ProposalControl()
            self.control.start(session.session_id)
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
                if not self._handle_hand_stop(tree, session, ctx):
                    tree.root.stop(py_trees.common.Status.INVALID)   # 실행 중 블록 terminate → 스킬 goal 취소
                    self.get_logger().warn("안전 정지 — 세션 중단")
                    break
                if self._hand_wait is not None:                      # 손 근접 정지 대기: 트리를 멈춘 채 기다림
                    if ctx is not None and time.monotonic() - last_feedback >= 1.0:
                        goal_handle.publish_feedback(RunSession.Feedback(context=ctx))
                        last_feedback = time.monotonic()
                    time.sleep(self.tick_period)
                    continue
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
            self._hand_wait = None
            with self._switch_lock:
                self.proposals.invalidate()
                self.control.finish(outcome)
                self._session = self._pending_switch = None
                self._pending_intent = None

        getattr(goal_handle, {"succeeded": "succeed", "canceled": "canceled", "aborted": "abort"}[outcome])()
        result = RunSession.Result(session_id=session.session_id, total_trials=len(session.trials),
                                   overall_success_rate=session.success_rate())
        self.get_logger().info(
            f"세션 종료 {session.session_id[:8]} ({outcome}) — 시도 {result.total_trials}회, "
            f"성공률 {result.overall_success_rate:.0%}")
        return result

    # ---------- 손 근접 STOP 대기 (#98) ----------
    def _handle_hand_stop(self, tree: py_trees.trees.BehaviourTree, session: Session, ctx: SceneContext | None) -> bool:
        """tick 마다. False = 세션 종료해야 함. 기다릴 수 있는 손 STOP 이면 대기에 들어가거나(로봇이 서 있을 때) 대기를 갱신."""
        with self._switch_lock:
            if self._safety_abort:
                return False
            seen, self._hand_stop_seen = self._hand_stop_seen, None
        stop = ctx is not None and ctx.safety.level == SafetyStatus.LEVEL_STOP
        if stop and seen is None:
            # 세션 전에 이미 STOP 이었거나 _on_context 가 아직 못 본 상황판 — 같은 기준으로 분류
            if not hand_stop.waitable(ctx.safety.reason, session.current_mode, self._hand_cfg):
                return False
            seen = ctx.safety.reason
        now = time.monotonic()
        if seen is not None:
            if self._hand_wait is None:
                if hand_stop.robot_busy(tree.root.iterate()):
                    self.get_logger().warn(f"[손 근접 정지] 로봇 동작 중 {seen} — 공·그리퍼 상태를 몰라 이어 가지 않음")
                    return False
                self._hand_wait = hand_stop.HandStopWait(seen, now, self._hand_cfg)
                with self._switch_lock:
                    self.control.hand_wait = seen
                self.get_logger().warn(f"[손 근접 정지] {seen} — 세션 유지, 손이 빠지면 같은 시도를 이어 감")
                self._say(self._hand_cfg["hand_stop_prompt"])
            else:
                self._hand_wait.stop_seen(seen)
        if self._hand_wait is None:
            return True
        level = ctx.safety.level if ctx is not None else SafetyStatus.LEVEL_STOP
        if self._hand_wait.update(level, now):
            waited = now - self._hand_wait.started
            hand_stop.shift_timers(tree.root.iterate(), waited)
            self.get_logger().info(f"[손 근접 정지] 해제 — {waited:.1f} s 기다림, 같은 시도를 이어 감")
            self._hand_wait = None
            with self._switch_lock:
                self.control.hand_wait = ""
        elif self._hand_wait.prompt_due(now):
            self._say(self._hand_cfg["hand_stop_prompt"])
        return True

    def destroy_node(self):
        if self._dashboard_bridge is not None:
            self._dashboard_bridge.close()
        return super().destroy_node()


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
