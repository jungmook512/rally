"""rally_robot/skills/move_target.py — 벨크로 패드 궤적 이동 (MoveTarget)
담당: 정정인 · 모드 3

단계
1. 패드 손잡이 파지 자세로 이동, 파지
2. pattern 별 궤적: HOLD / LINE(movel 왕복) / CIRCLE(movec) / FIGURE8(movesx 스플라인) / WAYPOINTS(moveb)
3. 이동 중 /rally/robot/wrench 피크 카운트 → impacts_detected (보조 판정)
4. duration_s 경과 또는 취소 시 패드 거치대에 걸기 → 그리퍼 해제
공통 규칙
- 매 단계 사이에 node.stop_requested() 확인 → True 면 release_compliance_ctrl() 후 후퇴, goal_handle.abort()
- 속도는 VELOCITY * node.speed_scale()
- 선생님 pick_and_place_target() 의 movel → mwait → gripper 패턴을 기본으로 하고, 접촉 구간만 순응·힘 조건으로 교체
"""
from rally_interfaces.action import MoveTarget


def execute(node):
    def _execute(goal_handle):
        from DSR_ROBOT2 import movel, movej, mwait, get_current_posx  # noqa: F401  (init_dsr 이후 import 가능)
        goal = goal_handle.request
        feedback = MoveTarget.Feedback()
        result = MoveTarget.Result()
        node.get_logger().info(f"[move_target] goal: {goal}")
        # TODO(정정인): 위 단계 구현
        goal_handle.abort()
        return result
    return _execute
