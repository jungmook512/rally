"""rally_robot/skill_server.py — RALLY 로봇 스킬 action 서버 7종 (PushBall · PickCompliant · Handover · ResistPush · CatchBall · DetachBall · MoveTarget)
담당: 정정인

설계 메모
- 선생님 코드는 import 시점에 rclpy.init()+DR_init 을 하고 movel/mwait 가 블로킹이다.
  여기서는 main()에서 init_dsr() 한 번 호출하고, 스킬 콜백은 ReentrantCallbackGroup 에서 돌려
  safety_monitor 구독(별도 그룹)이 막히지 않게 한다. Executor 는 MultiThreadedExecutor.
- 모든 스킬은 실행 중 SafetyStatus.level == STOP 이면 즉시 abort 하고 순응 해제 → 후퇴 (dsr_helpers.move_checked / compliance).
- DRL 함수(movel, task_compliance_ctrl, ...)는 init_dsr() 이후 import.
- 접촉이 예정된 구간(집기 하강·핸드오버 당김 대기·저항 밀기)에서는 /rally/safety/contact_expected 를 True 로 올린다(5 Hz 하트비트).
  safety_monitor 는 이 동안 nudge(힘 급변) 판정만 끄고 절대 힘 한계는 그대로 둔다. skill_server 가 죽으면 0.5 s 뒤 자동으로 꺼진다.
- 설정: rally_bringup/config/skills.yaml + table.yaml (share 에 설치된 것 → 없으면 소스 트리 폴백).
- 로봇 없이: `ros2 run rally_robot skill_server --fake` 또는 RALLY_FAKE=1 → rally_robot.fake_dsr 를 DSR_ROBOT2 자리에 끼운다. 실제 모드 경로는 그대로.
"""
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from rally_interfaces.action import CatchBall, DetachBall, Handover, MoveTarget, PickCompliant, PushBall, ResistPush
from rally_interfaces.msg import HandState, SafetyStatus
from rally_interfaces.srv import GripperCommand
from rally_robot.dsr_helpers import ACC, OBSERVE_POS, ROBOT_ID, VELOCITY, SkillAbort, init_dsr

CONTACT_HZ = 5.0
CONFIG_FILES = ("skills.yaml", "table.yaml")


def load_config(logger):
    """skills.yaml + table.yaml 을 한 dict 로. share 에 설치된 파일 → 소스 트리(ros2_ws/src/rally_bringup/config) 순."""
    cfg = {}
    here = os.path.dirname(os.path.abspath(__file__))
    for name in CONFIG_FILES:
        candidates = []
        try:
            candidates.append(os.path.join(get_package_share_directory("rally_bringup"), "config", name))
        except PackageNotFoundError:
            pass
        candidates.append(os.path.normpath(os.path.join(here, "..", "..", "rally_bringup", "config", name)))
        path = next((c for c in candidates if os.path.exists(c)), None)
        if path is None:
            logger.warning(f"{name} 을 찾지 못함 — 스킬 기본값 사용")
            continue
        with open(path, encoding="utf-8") as f:
            cfg.update(yaml.safe_load(f) or {})
        logger.info(f"config 로드: {path}")
    return cfg


class SkillServer(Node):
    def __init__(self, fake=False):
        super().__init__("rally_skill_server")
        self.fake = fake
        self.dsr = None                      # main() 에서 init_dsr() 노드를 넣어준다 (fake 면 None)
        self.cfg = load_config(self.get_logger())
        self.cb_safety = MutuallyExclusiveCallbackGroup()
        self.cb_state = MutuallyExclusiveCallbackGroup()
        self.cb_skill = ReentrantCallbackGroup()
        self.safety = SafetyStatus()
        self.last_hand = None
        self._contact = False
        self._cm_client = None

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        reliable1 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(SafetyStatus, "/rally/safety/status", self._on_safety, reliable, callback_group=self.cb_safety)
        self.create_subscription(HandState, "/rally/perception/hand", self._on_hand, qos_profile_sensor_data, callback_group=self.cb_state)
        self.pub_contact = self.create_publisher(Bool, "/rally/safety/contact_expected", reliable1)
        self.create_timer(1.0 / CONTACT_HZ, self._contact_tick, callback_group=self.cb_state)
        self.gripper_cli = self.create_client(GripperCommand, "/rally/gripper/command", callback_group=self.cb_state)
        self.create_service(Trigger, "/rally/robot/go_observe_pose", self._go_observe, callback_group=self.cb_skill)

        from rally_robot.skills import catch_ball, detach_ball, handover, move_target, pick_compliant, push_ball, resist_push
        self._servers = [
            ActionServer(self, PushBall, "/rally/skill/push_ball", push_ball.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, CatchBall, "/rally/skill/catch_ball", catch_ball.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, MoveTarget, "/rally/skill/move_target", move_target.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, DetachBall, "/rally/skill/detach_ball", detach_ball.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, Handover, "/rally/skill/handover", handover.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, ResistPush, "/rally/skill/resist_push", resist_push.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, PickCompliant, "/rally/skill/pick_compliant", pick_compliant.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
        ]
        self.get_logger().info(f"skill_server ready ({'FAKE' if fake else 'real'} DSR)")

    # ----- 안전 -----
    def _on_safety(self, msg):
        self.safety = msg

    def _on_hand(self, msg):
        self.last_hand = msg

    def stop_requested(self):
        return self.safety.level == SafetyStatus.LEVEL_STOP

    def safety_reason(self):
        return self.safety.reason or "safety_stop"

    def speed_scale(self):
        return self.safety.speed_override if self.safety.level == SafetyStatus.LEVEL_SLOW else 1.0

    def check_stop(self, goal_handle=None):
        """단계 사이에 호출. STOP 이면 SkillAbort, 취소 요청이면 SkillAbort('canceled')."""
        if self.stop_requested():
            raise SkillAbort(self.safety_reason(), stopped=True)
        if goal_handle is not None and goal_handle.is_cancel_requested:
            raise SkillAbort("canceled", stopped=True)

    def check_controller(self, timeout_s=2.0):
        """두산 motion 서비스가 응답하는지 (fake 면 통과). 무응답이면 SkillAbort('controller_unresponsive') — bringup 재시작 안내."""
        if self.fake:
            return
        import DSR_ROBOT2
        from dsr_msgs2.srv import CheckMotion
        prefix = getattr(DSR_ROBOT2, "_srv_name_prefix", "dsr_controller2/")
        if self._cm_client is None:
            self._cm_client = self.create_client(CheckMotion, f"/{ROBOT_ID}/{prefix}motion/check_motion", callback_group=self.cb_state)
        if not self._cm_client.wait_for_service(timeout_sec=timeout_s):
            raise SkillAbort("controller_unresponsive", stopped=True)
        fut = self._cm_client.call_async(CheckMotion.Request())
        t0 = time.monotonic()
        while not fut.done():
            time.sleep(0.02)
            if time.monotonic() - t0 > timeout_s:
                self.get_logger().error("두산 컨트롤러 motion 서비스 무응답 — bringup 재시작 필요")
                raise SkillAbort("controller_unresponsive", stopped=True)

    # ----- 접촉 예정 플래그 -----
    def set_contact(self, on: bool):
        self._contact = bool(on)
        self.pub_contact.publish(Bool(data=self._contact))

    def _contact_tick(self):
        if self._contact:
            self.pub_contact.publish(Bool(data=True))

    # ----- 그리퍼 -----
    def gripper_cfg(self, object_class: str):
        """skills.yaml gripper.<class> → (열림 폭 mm, 파지 힘 N). 없으면 기본 (110, 20)."""
        d = (self.cfg.get("gripper") or {}).get(object_class) or {}
        return float(d.get("open_mm", 110.0)), float(d.get("force_n", 20.0))

    def gripper(self, width_mm: float, force_n: float, wait: bool = True, timeout_s: float = 4.0):
        """/rally/gripper/command 호출 → (success, actual_width_mm, object_detected). fake 면 닫을 때 물체 있음으로 흉내."""
        if self.fake:
            time.sleep(0.3)
            closing = width_mm < 50.0
            return True, (45.0 if closing else float(width_mm)), closing
        if not self.gripper_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/rally/gripper/command 서비스 없음 — gripper_node 를 먼저 띄우세요")
            return False, 0.0, False
        req = GripperCommand.Request()
        req.width_mm, req.force_n, req.wait_until_done = float(width_mm), float(force_n), bool(wait)
        fut = self.gripper_cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            time.sleep(0.02)
            if time.monotonic() - t0 > timeout_s:
                self.get_logger().error("그리퍼 서비스 응답 타임아웃")
                return False, 0.0, False
        r = fut.result()
        return bool(r.success), float(r.actual_width_mm), bool(r.object_detected)

    # ----- action plumbing -----
    def _accept(self, goal):
        return GoalResponse.REJECT if self.stop_requested() else GoalResponse.ACCEPT

    def _cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _go_observe(self, req, res):
        from DSR_ROBOT2 import movej, mwait
        if self.stop_requested():
            res.success, res.message = False, f"safety STOP ({self.safety_reason()})"
            return res
        posj = (self.cfg.get("observe_pose") or {}).get("posj") or OBSERVE_POS   # skills.yaml 티칭 값 우선
        movej([float(v) for v in posj], vel=VELOCITY * self.speed_scale(), acc=ACC * self.speed_scale())
        mwait()
        res.success, res.message = True, f"observe pose {posj}"
        return res


def main(args=None):
    argv = list(sys.argv[1:] if args is None else args)
    fake = "--fake" in argv or os.environ.get("RALLY_FAKE") == "1"
    rclpy.init(args=[a for a in argv if a != "--fake"])
    dsr_node = None
    if fake:
        from rally_robot import fake_dsr
        sys.modules["DSR_ROBOT2"] = fake_dsr        # 스킬의 `from DSR_ROBOT2 import …` 가 가짜를 받는다
    else:
        dsr_node = init_dsr()                        # DR_init 먼저
    node = SkillServer(fake=fake)
    node.dsr = dsr_node
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    if dsr_node is not None:
        ex.add_node(dsr_node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        if dsr_node is not None:
            dsr_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
