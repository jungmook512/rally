"""rally_robot/skills/resist_push.py — 저항 밀기 (ResistPush) · 팔씨름형 등척성 훈련
담당: 정정인 · 모드 7

원칙: 로봇은 버티거나(순응 스프링) 져주기(밀려남)만 한다. 환자 쪽으로 능동적으로 미는 코드는 이 파일에 없어야 한다.
단계
1. 손잡이 막대 파지(거치대) → handle_pose 로 movel 제시. feedback.phase='present'
2. task_compliance_ctrl + set_stiffnessx(밀리는 축만 goal.stiffness, 나머지 축 높게) → 'resist'
   20 Hz 로 /rally/robot/wrench 의 밀기 축 힘 읽어 force_profile 에 누적, feedback.current_force_n
3. 힘 >= target_force_n 인 시간이 hold_time_s 를 넘으면 → 'yielding': 밀리는 축으로 yield_distance_m 만큼
   저속 movel (환자가 미는 방향, 즉 환자에게서 멀어지는 방향으로만) → success=True
4. 힘이 0 근처로 떨어지면(환자가 손 놓음) → 'return': 저속·저힘으로 handle_pose 복귀, 복귀 중 접촉 감지 시 즉시 정지
5. 종료 시 반드시 release_compliance_ctrl (try/finally)
6. 2~4(환자가 손잡이를 쥔 구간) 동안 node.set_patient_contact(True), finally 에서 False — 손 거리 STOP 이 SLOW 로 낮아지므로(#59)
   이 구간엔 환자 쪽(+x) 명령 이동을 절대 넣지 않는다 (져주기·복귀만)
안전
- force_limit_n(기본 15 N) 초과 → 순응 해제·후퇴·abort, failure_reason='force_limit'
- SafetyStatus STOP → abort. 환자 손이 손잡이 외 로봇 링크에 접근(HandState)하면 정지는 safety_monitor 가 담당
- 힘 임계값은 Day 2 핸드오버 당김 힘 실험 데이터로 결정
"""
from rally_interfaces.action import ResistPush


def execute(node):
    def _execute(goal_handle):
        from DSR_ROBOT2 import movel, mwait, task_compliance_ctrl, set_stiffnessx, release_compliance_ctrl  # noqa: F401
        goal = goal_handle.request
        feedback = ResistPush.Feedback()
        result = ResistPush.Result()
        node.get_logger().info(f"[resist_push] stiffness={goal.stiffness} target={goal.target_force_n}N hold={goal.hold_time_s}s")
        try:
            # TODO(정정인): 위 단계 구현
            goal_handle.abort()
        finally:
            try:
                release_compliance_ctrl()
            except Exception:
                pass
        return result
    return _execute
