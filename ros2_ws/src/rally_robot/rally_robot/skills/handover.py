"""rally_robot/skills/handover.py — 건네기 / 받기 (Handover)
담당: 정정인 · 모드 5

단계
GIVE: 1. 물체를 tilt_deg 로 기울여 중앙선 앞 제시 자세로 movel
      2. wrench 로 당김 힘 감시 → release_force_threshold_n 초과 시 그리퍼 해제 → 후퇴
RECEIVE: 1. follow_hand 면 /rally/perception/hand 의 palm_pose 를 servol 로 추종 (One-Euro 필터, 중앙선 앞 구역 제한)
         2. 손 정지 + 접근 완료 시 task_compliance_ctrl 상태로 그리퍼 닫기 → 폭으로 파지 판정
         3. timeout_s 초과 시 실패
공통 규칙
- 매 단계 사이에 node.stop_requested() 확인 → True 면 release_compliance_ctrl() 후 후퇴, goal_handle.abort()
- 속도는 VELOCITY * node.speed_scale()
- 선생님 pick_and_place_target() 의 movel → mwait → gripper 패턴을 기본으로 하고, 접촉 구간만 순응·힘 조건으로 교체
"""
from rally_interfaces.action import Handover


def execute(node):
    def _execute(goal_handle):
        from DSR_ROBOT2 import movel, movej, mwait, get_current_posx  # noqa: F401  (init_dsr 이후 import 가능)
        goal = goal_handle.request
        feedback = Handover.Feedback()
        result = Handover.Result()
        node.get_logger().info(f"[handover] goal: {goal}")
        # TODO(정정인): 위 단계 구현
        goal_handle.abort()
        return result
    return _execute
