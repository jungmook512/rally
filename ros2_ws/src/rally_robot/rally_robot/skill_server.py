"""rally_robot/skill_server.py — RALLY 로봇 스킬 action 서버 (PushBall · MoveTarget · DetachBall · Handover · PickCompliant)
담당: 정정인

설계 메모
- 선생님 코드는 import 시점에 rclpy.init()+DR_init 을 하고 movel/mwait 가 블로킹이다.
  여기서는 main()에서 init_dsr() 한 번 호출하고, 스킬 콜백은 ReentrantCallbackGroup 에서 돌려
  safety_monitor 구독(별도 그룹)이 막히지 않게 한다. Executor 는 MultiThreadedExecutor.
- 모든 스킬은 실행 중 SafetyStatus.level == STOP 이면 즉시 abort 하고 순응 해제 → 후퇴.
- DRL 함수(movel, task_compliance_ctrl, check_force_condition, servol ...)는 init_dsr() 이후 import.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger

from rally_interfaces.action import PushBall, MoveTarget, CatchBall, DetachBall, Handover, PickCompliant, ResistPush
from rally_interfaces.msg import SafetyStatus
from rally_robot.dsr_helpers import init_dsr, OBSERVE_POS, VELOCITY, ACC


class SkillServer(Node):
    def __init__(self):
        super().__init__("rally_skill_server")
        self.cb_safety = MutuallyExclusiveCallbackGroup()
        self.cb_skill = ReentrantCallbackGroup()
        self.safety = SafetyStatus()
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(SafetyStatus, "/rally/safety/status", self._on_safety, reliable, callback_group=self.cb_safety)
        self.create_service(Trigger, "/rally/robot/go_observe_pose", self._go_observe, callback_group=self.cb_skill)

        from rally_robot.skills import push_ball, move_target, catch_ball, detach_ball, handover, pick_compliant, resist_push
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
        self.get_logger().info("skill_server ready")

    # ----- safety -----
    def _on_safety(self, msg):
        self.safety = msg

    def stop_requested(self):
        return self.safety.level == SafetyStatus.LEVEL_STOP

    def speed_scale(self):
        return self.safety.speed_override if self.safety.level == SafetyStatus.LEVEL_SLOW else 1.0

    # ----- action plumbing -----
    def _accept(self, goal):
        return GoalResponse.REJECT if self.stop_requested() else GoalResponse.ACCEPT

    def _cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _go_observe(self, req, res):
        from DSR_ROBOT2 import movej, mwait
        movej(OBSERVE_POS, vel=VELOCITY, acc=ACC)
        mwait()
        res.success = True
        res.message = "observe pose"
        return res


def main(args=None):
    rclpy.init(args=args)
    dsr_node = init_dsr()                      # DR_init 먼저
    node = SkillServer()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.add_node(dsr_node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
