"""rally_robot/skills/push_ball.py — 공 밀기 (PushBall)
담당: 정정인 · 모드 1

단계
1. 공 뒤쪽(목표 반대편)으로 접근 (관측 자세 → 공 위 → 뒤쪽 하강)
2. push_dir_deg 방향으로 push_speed 속도 movel, check_force_condition 으로 접촉 감지 → feedback.phase='contact'
3. 정해진 거리만큼 밀고 정지 → 관측 자세 복귀
4. 비전(/rally/perception/objects)이 is_moving=false 가 될 때까지 대기 → stop_position, error_mm 채워 result
공통 규칙
- 매 단계 사이에 node.stop_requested() 확인 → True 면 release_compliance_ctrl() 후 후퇴, goal_handle.abort()
- 속도는 VELOCITY * node.speed_scale()
- 선생님 pick_and_place_target() 의 movel → mwait → gripper 패턴을 기본으로 하고, 접촉 구간만 순응·힘 조건으로 교체
"""
from rally_interfaces.action import PushBall


def execute(node):
    def _execute(goal_handle):
        from DSR_ROBOT2 import movel, movej, mwait, get_current_posx  # noqa: F401  (init_dsr 이후 import 가능)
        goal = goal_handle.request
        feedback = PushBall.Feedback()
        result = PushBall.Result()
        node.get_logger().info(f"[push_ball] goal: {goal}")
        # TODO(정정인): 위 단계 구현
        goal_handle.abort()
        return result
    return _execute
