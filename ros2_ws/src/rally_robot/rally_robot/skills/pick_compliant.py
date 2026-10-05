"""rally_robot/skills/pick_compliant.py — 순응 제어 집기 (PickCompliant)
담당: 정정인 · 모드 공통

단계
1. target 위 접근 (yaw 만큼 손목 회전)
2. task_compliance_ctrl + set_stiffnessx(z=z_stiffness) 로 하강
3. check_force_condition(z, max=contact_force) 로 테이블 접촉 감지 → 정지
4. grip_force_n 으로 파지 → release_compliance_ctrl → 들기 → grip_width_mm 로 성공 판정
공통 규칙
- 매 단계 사이에 node.stop_requested() 확인 → True 면 release_compliance_ctrl() 후 후퇴, goal_handle.abort()
- 속도는 VELOCITY * node.speed_scale()
- 선생님 pick_and_place_target() 의 movel → mwait → gripper 패턴을 기본으로 하고, 접촉 구간만 순응·힘 조건으로 교체
"""
from rally_interfaces.action import PickCompliant


def execute(node):
    def _execute(goal_handle):
        from DSR_ROBOT2 import movel, movej, mwait, get_current_posx  # noqa: F401  (init_dsr 이후 import 가능)
        goal = goal_handle.request
        feedback = PickCompliant.Feedback()
        result = PickCompliant.Result()
        node.get_logger().info(f"[pick_compliant] goal: {goal}")
        # TODO(정정인): 위 단계 구현
        goal_handle.abort()
        return result
    return _execute
