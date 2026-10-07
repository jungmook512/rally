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
세션 중단: 안전 STOP (상황판 기준) → abort / 취소 요청 → canceled. 둘 다 진행 중인 스킬 goal 도 취소한다.
트리는 execute 콜백 안에서 tick_hz(기본 10 Hz)로 돈다. 스킬 action 은 비동기라 tick 이 막히지 않는다.
TODO(Day 3): 준비가 도중에 깨지면 중단 + 관측 자세 대기, rule_layer(난이도·휴식), SetMode·음성 intent
"""
from __future__ import annotations

import os
import threading
import time
import uuid

import py_trees
import rclpy
import yaml
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

from rally_bringup.readiness import ReadinessChecker
from rally_brain.trees.session_tree import Session, build_session_tree
from rally_interfaces.action import RunSession
from rally_interfaces.msg import SafetyStatus, SceneContext, TrialResult

CHECK_PERIOD_S = 1.0
STATE_PERIOD_S = 0.5
ALL_MODES = [TrialResult.MODE_PUSH_BALL, TrialResult.MODE_CATCH_PAD, TrialResult.MODE_HANDOVER, TrialResult.MODE_RESIST]


def mode3_enabled() -> bool:
    """skills.yaml mode3_challenge.enabled. 파일을 못 읽으면 True (기획 기본값)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("rally_bringup"), "config", "skills.yaml")
        with open(path, encoding="utf-8") as f:
            return bool(((yaml.safe_load(f) or {}).get("mode3_challenge") or {}).get("enabled", True))
    except Exception:  # noqa: BLE001 — 설정 못 읽어도 세션은 돈다
        return True


def default_modes(include_mode3: bool) -> list[int]:
    return [m for m in ALL_MODES if include_mode3 or m != TrialResult.MODE_CATCH_PAD]


class BtRunner(Node):
    def __init__(self):
        super().__init__("rally_bt_runner")
        self.declare_parameter("check_readiness", True)
        self.declare_parameter("trials_per_mode", 5)
        self.declare_parameter("tick_hz", 10.0)
        self.check_readiness = bool(self.get_parameter("check_readiness").value)
        self.tick_period = 1.0 / float(self.get_parameter("tick_hz").value)

        self.readiness = ReadinessChecker(self, ["session"]) if self.check_readiness else None
        self.session_ready = not self.check_readiness
        self._last_missing: list[str] | None = None
        self._context: SceneContext | None = None
        self._tree: py_trees.trees.BehaviourTree | None = None
        self._busy = threading.Lock()   # 세션은 한 번에 하나
        self._skill_clients: dict[str, ActionClient] = {}
        self._client_group = ReentrantCallbackGroup()

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.create_subscription(SceneContext, "/rally/brain/context", self._on_context, qos_profile_sensor_data)
        self._trial_pub = self.create_publisher(TrialResult, "/rally/brain/trial_result", reliable)
        self._state_pub = self.create_publisher(String, "/rally/brain/state", reliable)
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
        tree = self._tree
        tip = tree.tip() if tree is not None else None
        self._state_pub.publish(String(data=tip.name if tip is not None else "idle"))

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
        modes = list(goal.modes) or default_modes(mode3_enabled())
        n = goal.trials_per_mode or int(self.get_parameter("trials_per_mode").value)
        session = Session(node=self, session_id=str(uuid.uuid4()), get_context=lambda: self._context,
                          publish=self._trial_pub.publish, action_client=self._action_client)
        tree = py_trees.trees.BehaviourTree(build_session_tree(session, modes, n))
        self._tree = tree
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
                if ctx is not None and ctx.safety.level == SafetyStatus.LEVEL_STOP:
                    tree.root.stop(py_trees.common.Status.INVALID)   # 실행 중 블록 terminate → 스킬 goal 취소
                    self.get_logger().warn(f"안전 정지({ctx.safety.reason}) — 세션 중단")
                    break
                tree.tick()
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
            self._tree = None

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
