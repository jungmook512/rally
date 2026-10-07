# TODO — 로봇 제어 (로봇·인식 PC, 네이티브) · 정정인

패키지: `ros2_ws/src/rally_robot` · 책임 모드: **모드 1 (공 굴려주기)**
참고: `docs/INTERFACES.md` 2.1 / 3 / 4, `rally_bringup/config/safety.yaml`, `skills.yaml`

## Day 0 (10/5)
- [x] `rally_robot/README.md` 읽기 — 선생님 코드가 어디로 옮겨졌는지, `dsr_helpers.init_dsr()` 패턴
- [x] dsr_bringup2 빌드, M0609 연결, `movel` 기본 이동 확인
- [x] RG2 Modbus TCP 연결, 폭·힘 레지스터 읽기/쓰기 확인 (`scripts/check_gripper.py`)
- [x] `get_tool_force()` 값 읽기, 노이즈 수준 메모 — 10/6 정지 노이즈 한 주기 |ΔF| ≤ 0.25 N, |F| ≤ 2.5 N → `nudge_dfdt_n` 5 N (#25)
- [x] 관측 자세 후보 2~3개 티칭 (카메라가 테이블 전체를 보는 자세) — 10/6 1개 확정, `skills.yaml observe_pose` (#25)
- [ ] (박성현과) 패드 들고 환자 쪽 향할 때 RealSense 시야 확인

## Day 1 — 기반
- [ ] **기준선**: `ros2 launch rally_bringup legacy_baseline.launch.py` 로 선생님 픽앤플레이스가 그대로 동작하는지 확인 (박성현·정동휘와)
- [x] `force_publisher.py`: `/rally/robot/wrench` (WrenchStamped, 100 Hz), `/rally/robot/tcp_pose` — 코드 완료, PC1 에서 `ros2 topic hz` 확인 필요
- [x] `gripper_node.py`: `GripperCommand` srv, `/rally/gripper/state`, grip 비트 + 폭으로 `object_detected` — 동기 클라이언트 + Lock. 비동기 교체는 TODO. PC1 확인 필요
- [x] `/rally/robot/go_observe_pose` (std_srvs/Trigger) — skill_server 에 있음, `skills.yaml observe_pose.posj` 사용 (#25)
- [x] 캘리브레이션용 로봇 자세 15개 순회 스크립트 (`ros2 run rally_robot calib_sweep --center … --dry-run` 먼저) — 10/6 저녁 실로봇 1차 순회, 정지 서비스 이름·이동 타임아웃 수정 (#28). 10/7 실로봇 이동 중 estop → 출발 1.2 s 만에 정지 확인 (순회가 이동 중 안전 상태를 못 받던 버그 수정). 컨트롤러 J5 한계(≈135°) 넘는 자세는 IK 로 미리 제외 — 관측 자세 J5 131° 라 15개 중 9개만 통과 → 관측 자세 재교시 필요
- [ ] 그리퍼 닫힘 지연 측정 (`scripts/check_gripper.py` 가 ms 출력) → `skills.yaml gripper.command_to_motion_ms`

## Day 2 — 모드 5 스킬
- [x] `skill_server.py` 골격: MultiThreadedExecutor, 콜백 그룹(안전 / 스킬 / 상태) 분리
- [x] `pick_compliant.py`: `task_compliance_ctrl` Z 저강성 → 하강 → Z 힘 변화로 접촉 감지 → 파지 → 들기 — 코드 완료(가짜 모드 통과), 로봇 확인 필요. Z 는 table.yaml 평면 우선
- [x] `handover.py` GIVE: 물체를 `tilt_deg`로 기울여 중앙선 앞 제시 → 당김 힘 임계 감지 → RG2 해제 → 후퇴 — 코드 완료(가짜 모드 통과). `skills.yaml handover.present_pose` 티칭 값 필요
- [ ] 당김 힘 임계값 실험 (사람이 당길 때 힘 곡선 10회 기록)

## Day 3 — 모드 5 완성 + 안전 ✅
- [ ] `handover.py` RECEIVE: `HandState.palm_pose`를 `servol`로 추종(30 Hz, One-Euro 필터) → 순응 상태로 받기 → 파지 판정
- [ ] (정동휘와) 손 토픽 → 로봇 반응 지연 측정, 추종 게인 튜닝
- [x] `safety_monitor.py`: 작업 공간 제한, 손 근접 감속, 힘 한계 25 N, nudge(정지 중·접촉 예정 아닐 때만), `/rally/safety/estop` 구독 — 코드 완료. 접촉 예정 플래그 `/rally/safety/contact_expected` 는 skill_server 가 발행. 임계값 실측 후 확정
- [x] `SafetyStatus` Reliable 발행, `LEVEL_STOP` 시 실행 중 action abort — 가짜 모드에서 확인(estop → 이동 중 abort, STOP 중 goal 거부). 로봇에서 재확인
- [x] `/rally/robot/reset_safety`
- [ ] 3중 정지 전수 테스트 10회

## Day 4 — 모드 7 완성 ✅ (오전) + 모드 3 착수
- [ ] `resist_push.py`: 손잡이 제시 → `task_compliance_ctrl` + `set_stiffnessx`(밀리는 축만 저강성) → 20 Hz 힘 누적 → 목표 유지 2 s → 져주기 저속 movel(환자 힘 벡터 방향) → 손 놓으면 저속 복귀(접촉 감지 정지) → `finally` 해제
- [ ] 힘 한계 15 N 전수 테스트, 로그로 "이동 방향이 항상 환자에게서 멀어짐" 확인
- [ ] 강성 3단계(200/700/1500 N/m) 실제 느낌 확인, Day 2 당김 힘 데이터로 target_force 초기값 결정
- [ ] 패드 손잡이 어댑터 파지 자세 티칭, 충격 시 회전 여부 확인 — 10/7 충격 측정: 로봇이 느끼는 힘은 노이즈(≈2 N) 수준, 센서가 0.1 s 단위라 순간 피크 측정 불가. 3D 프린팅 설계 하중은 계산값(약 100 N) 전달
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
- [x] `push_ball.py`: `lane` 으로 가이드 길 선택(skills.yaml) → 공을 길 입구에 놓고 뒤로 접근 → `push_speed` 속도 프로파일 movel → 접촉 힘 감지 → 정해진 거리 밀고 정지 → 관측 자세 복귀 — 10/7 코드 완료(집기 후 레인 입구에 놓기 → 닫은 손가락으로 위치 기반 밀기, 툴 힘 0 이라 접촉 감지 없음). 가짜 모드·두산 에뮬레이터(scripts/sim_robot.sh) 통과, 실로봇 미확인. 레인 입구·방향·공 지름 실측 필요
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

- [!] 10/7 `get_tool_force` 가 전부 0 (관절 외력 토크는 정상). 재부팅·툴 재적용으로 안 풀림. 우회: `scripts/ejt_wrench.py`(관절 외력 토크 → 자코비안). 선생님·다른 조 로봇과 비교 필요. 접촉 검출·당김 해제·nudge 는 이 문제가 풀리거나 우회 경로를 force_publisher 에 넣어야 실로봇 가능
- 소프트 비상정지는 `scripts/estop.sh` (QoS TransientLocal 맞춤), 해제는 `scripts/estop.sh reset`
- 10/6: 로봇 없이 스킬 흐름 테스트: `RALLY_FAKE=1 ros2 run rally_robot skill_server --fake` (가짜 DSR, 테이블 Z 250 mm). 모드 7 nudge 오탐 대비로 접촉 예정 플래그 도입.
- 10/6: 그리퍼·로봇이 모드 3 검토에 쓰이는 동안 코드 먼저 진행. 물체별 파지 폭은 안 재기로 (폭 0 + 물체별 힘으로 닫음, `skills.yaml gripper`). safety.yaml 값 채움(INTERFACES 6절 기준), nudge·작업 공간 값은 실측 후 확정.
- 10/6 실측: 테이블 Z 14.4 mm(`table.yaml`), 작업 공간 x_min 0.20 m, 관측 자세 티칭 (#25). 측정 CSV 는 `data/measure/`(git 제외, #31).
- 10/6 남은 것: #14 skill_server 안전 상태 타임아웃(담당 정해야 함) · #30 네이티브↔root 컨테이너 공유 메모리 양방향 차단(`env_robot_pc.sh` 에 UDPv4, 수정 후 pc2 로 재확인) · 이동 중 estop 정지 확인 · 그리퍼 닫힘 지연 · 당김 힘 10회
