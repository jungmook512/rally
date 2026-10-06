# TODO — 로봇 제어 (PC 1) · 정정인

패키지: `ros2_ws/src/rally_robot` · 책임 모드: **모드 1 (공 굴려주기)**
참고: `docs/INTERFACES.md` 2.1 / 3 / 4, `rally_bringup/config/safety.yaml`, `skills.yaml`

## Day 0 (10/5)
- [ ] `rally_robot/README.md` 읽기 — 선생님 코드가 어디로 옮겨졌는지, `dsr_helpers.init_dsr()` 패턴
- [ ] dsr_bringup2 빌드, M0609 연결, `movel` 기본 이동 확인
- [ ] RG2 Modbus TCP 연결, 폭·힘 레지스터 읽기/쓰기 확인
- [ ] `get_tool_force()` 값 읽기, 노이즈 수준 메모
- [ ] 관측 자세 후보 2~3개 티칭 (카메라가 테이블 전체를 보는 자세)
- [ ] (박성현과) 패드 들고 환자 쪽 향할 때 RealSense 시야 확인

## Day 1 — 기반
- [ ] **기준선**: `ros2 launch rally_bringup legacy_baseline.launch.py` 로 선생님 픽앤플레이스가 그대로 동작하는지 확인 (박성현·정동휘와)
- [ ] `force_publisher.py`: `/rally/robot/wrench` (WrenchStamped, 100 Hz), `/rally/robot/tcp_pose`
- [ ] `gripper_node.py`: Pymodbus **비동기** 클라이언트, `GripperCommand` srv, `/rally/gripper/state`, 폭 피드백으로 `object_detected`
- [ ] `/rally/robot/go_observe_pose` (std_srvs/Trigger)
- [ ] 캘리브레이션용 로봇 자세 15개 순회 스크립트 (박성현 `RunCalibration`에서 호출)
- [ ] 그리퍼 닫힘 지연 측정 (명령 → 실제 폭 변화까지 ms) → `skills.yaml`에 기록

## Day 2 — 모드 5 스킬
- [ ] `skill_server.py` 골격: MultiThreadedExecutor, 콜백 그룹(안전 / 스킬 / 상태) 분리
- [ ] `pick_compliant.py`: `task_compliance_ctrl` Z 저강성 → 하강 → `check_force_condition` 접촉 → 파지 → 들기
- [ ] `handover.py` GIVE: 물체를 `tilt_deg`로 기울여 중앙선 앞 제시 → 당김 힘 임계 감지 → RG2 해제 → 후퇴
- [ ] 당김 힘 임계값 실험 (사람이 당길 때 힘 곡선 10회 기록)

## Day 3 — 모드 5 완성 + 안전 ✅
- [ ] `handover.py` RECEIVE: `HandState.palm_pose`를 `servol`로 추종(30 Hz, One-Euro 필터) → 순응 상태로 받기 → 파지 판정
- [ ] (정동휘와) 손 토픽 → 로봇 반응 지연 측정, 추종 게인 튜닝
- [ ] `safety_monitor.py`: 작업 공간 제한(중앙선), 손 근접 감속(0.25 m → 50 %, 0.15 m → 정지), 힘 한계 25 N → 해제·후퇴, nudge(힘 급변) 정지, `/rally/safety/estop` 구독
- [ ] `SafetyStatus` Reliable 발행, `LEVEL_STOP` 시 실행 중 action abort 확인
- [ ] `/rally/robot/reset_safety`
- [ ] 3중 정지 전수 테스트 10회

## Day 4 — 모드 7 완성 ✅ (오전) + 모드 3 착수
- [ ] `resist_push.py`: 손잡이 제시 → `task_compliance_ctrl` + `set_stiffnessx`(밀리는 축만 저강성) → 20 Hz 힘 누적 → 목표 유지 2 s → 져주기 저속 movel(환자 힘 벡터 방향) → 손 놓으면 저속 복귀(접촉 감지 정지) → `finally` 해제
- [ ] 힘 한계 15 N 전수 테스트, 로그로 "이동 방향이 항상 환자에게서 멀어짐" 확인
- [ ] 강성 3단계(200/700/1500 N/m) 실제 느낌 확인, Day 2 당김 힘 데이터로 target_force 초기값 결정
- [ ] 패드 손잡이 어댑터 파지 자세 티칭, 충격 시 회전 여부 확인
- [ ] 대기 자세(ready_pose) 티칭: 테이블 중앙, 패드 면이 환자 쪽, 어느 방향이든 이동량 최소
- [ ] `catch_ball.py`: `/rally/perception/landing` 구독(별도 콜백 그룹) → in_flight 감지 → 예측점으로 `amovel` → time_to_impact < 0.25 s 부터 `servol` 보정 → 착지 후 복귀
- [ ] 예측점 중앙선 클램핑, 속도 클램핑, STOP 시 abort
- [ ] (박성현·조정묵과) `lead_time_s` 결정: measure_latency 결과 + 로봇 가속 시간. 살살 토스 20회 요격 성공률 기록 → **CP13 결정** (≥70 % 목표)
- [ ] [Phase 2 대비, 여유 시] `move_target.py` 궤적 3종
- [ ] 패드 거치대에 걸기 동작 (고정 시퀀스)
- [ ] `detach_ball.py`: 공 위치로 접근 → 파지 → 패드 면 수직 방향으로 힘 제어 당김 → **힘 급락** 감지 → 떼어짐 판정 → 후퇴
- [ ] 떼어내기 힘 곡선 10회 기록 → 급락 임계 결정 (조정묵 DB 저장)
- [ ] 떼어낸 공 → `Handover GIVE`로 환자에게

## Day 5 — 모드 1 스킬
- [ ] `push_ball.py`: `lane` 으로 가이드 길 선택(skills.yaml) → 공을 길 입구에 놓고 뒤로 접근 → `push_speed` 속도 프로파일 movel → 접촉 힘 감지 → 정해진 거리 밀고 정지 → 관측 자세 복귀
- [ ] 가이드 길 3개 위치 실측 → `skills.yaml mode1.lanes` 기입, 그리퍼가 길 벽에 안 걸리는지 확인
- [ ] 밀기 파라미터 ↔ 정지 거리 사전 실험 20회 (박성현 정지 위치 측정) → 초기 룩업 테이블
- [ ] 돌아온 공 `PickCompliant`로 집기 (테이블 접촉 감지)

## Day 6 — 모드 1 학습 루프 ✅
- [ ] `SuggestParams` 결과를 `PushBall` goal에 반영하는 경로 확인 (brain → skill)
- [ ] 30회 반복 시도로 학습 곡선 데이터 축적 (박성현·조정묵과)
- [ ] 실패 사례 분류: 접촉 못함 / 과도하게 밀림 / 방향 이탈 → `failure_reason`

## Day 7 — 통합·안정화
- [ ] 세션 전체(1→3→5) 로봇 측 연속 실행, action 취소·재시도 시 상태 복구 확인
- [ ] 안전 기능 전수 재점검, 속도 오버라이드 반영 확인
- [ ] 관측 자세·거치대·파지 자세 `skills.yaml` 최종값 커밋
- [ ] 오후부터 버그 수정만

## Day 8 — 시연
- [ ] 시연 직전 루틴: 기준점 터치 검증 보조, 3중 정지 확인
- [ ] 리허설 3회 로봇 운영, 비상정지 담당
- [ ] 발표: 힘·순응 제어 파트 (힘 곡선 그래프 준비)

## 메모 / 막힌 것
- 
