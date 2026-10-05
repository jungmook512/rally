"""rally_robot/skills/detach_ball.py — 패드에서 벨크로 공 떼어내기 (DetachBall)
담당: 정정인 · 모드 3

단계
1. ball_pose 위로 접근 → 하강 → grip_force_n 으로 파지
2. 패드 면 수직 방향으로 힘 제어 당김 (set_desired_force 또는 저속 movel + 힘 감시)
3. pull_force 가 상승하다 급락(detach_drop_n)하면 떼어짐 판정 → peak_pull_force_n 기록
4. pull_force_limit_n 초과 시 중단 → 실패 (brain이 각도 바꿔 재시도)
공통 규칙
- 매 단계 사이에 node.stop_requested() 확인 → True 면 release_compliance_ctrl() 후 후퇴, goal_handle.abort()
- 속도는 VELOCITY * node.speed_scale()
- 선생님 pick_and_place_target() 의 movel → mwait → gripper 패턴을 기본으로 하고, 접촉 구간만 순응·힘 조건으로 교체
"""
from rally_interfaces.action import DetachBall


def execute(node):
    def _execute(goal_handle):
        from DSR_ROBOT2 import movel, movej, mwait, get_current_posx  # noqa: F401  (init_dsr 이후 import 가능)
        goal = goal_handle.request
        feedback = DetachBall.Feedback()
        result = DetachBall.Result()
        node.get_logger().info(f"[detach_ball] goal: {goal}")
        # TODO(정정인): 위 단계 구현
        goal_handle.abort()
        return result
    return _execute
