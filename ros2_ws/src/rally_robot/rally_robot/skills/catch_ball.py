"""rally_robot/skills/catch_ball.py — 던진 벨크로 공을 패드로 예측 요격 (CatchBall)
담당: 정정인 · 모드 3 (Phase 1)

단계
1. 패드 손잡이 파지(이미 들고 있으면 생략) → ready_pose 로 이동, 패드 면을 환자 쪽으로
2. /rally/perception/objects 에서 velcro_ball.in_flight == True 감지 → feedback.phase='tracking', reaction_time 측정 시작
3. predicted_landing 이 catch_radius 안이면: lead_time_s 앞선 예측점으로 amovel (비동기) — 이동 중에도 예측 계속 수신
   밖이면 failure_reason='out_of_radius' 로 포기 (초기 단계는 radius 를 넓게 주므로 거의 안 생김)
4. time_to_impact < 0.25 s 부터 servol 로 패드 중심을 최신 예측점에 맞춤 (One-Euro 필터, 속도·가속도 클램핑)
5. 착지 후 0.3 s 대기 → 비전 pad_hit_judge 결과로 caught 판정, landing_actual·prediction_error_mm 기록
6. 결과와 무관하게 ready_pose 복귀. caught 면 brain 이 이어서 패드 거치 → DetachBall → Handover GIVE
안전
- 요격 이동도 작업 공간 제한(중앙선) 안에서만. 예측점이 중앙선 너머면 가장 가까운 경계점으로 클램핑
- 속도는 VELOCITY * node.speed_scale(), STOP 시 즉시 abort
"""
from rally_interfaces.action import CatchBall


def execute(node):
    def _execute(goal_handle):
        from DSR_ROBOT2 import amovel, servol, movel, mwait, get_current_posx  # noqa: F401
        goal = goal_handle.request
        feedback = CatchBall.Feedback()
        result = CatchBall.Result()
        node.get_logger().info(f"[catch_ball] radius={goal.catch_radius} lead={goal.lead_time_s}")
        # TODO(정정인): 위 단계 구현. 예측 토픽 구독은 node 에 미리 만들어 두고(콜백 그룹 분리) 최신값을 읽는다.
        goal_handle.abort()
        return result
    return _execute
