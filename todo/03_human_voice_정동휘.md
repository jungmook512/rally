# TODO — 사람 인식 · 음성 · 대시보드 (PC 2 hand_pose/voice, PC 3 dashboard) · 정동휘

패키지: `ros2_ws/src/rally_human`, `rally_voice`, `dashboard/` · 책임 모드: **모드 5 (건네고 받기)**
참고: `docs/INTERFACES.md` 2.2 / 3

## Day 0 (10/5)
- [ ] 웹캠 60 fps 확인(`v4l2-ctl`), 환자 측면 위치 가설정
- [ ] MediaPipe Hands·Pose 단독 실행, RealSense 컬러에서 손 검출 확인
- [ ] USB 마이크·스피커 장치 확인(`arecord -l`, `aplay -l`), 컨테이너 `/dev/snd` 패스스루 테스트
- [ ] openWakeWord·faster-whisper·Piper 설치, 한국어 Piper 음성 모델 다운로드 → `models/`
- [ ] 웨이크워드 "랠리야" 녹음 샘플 수집 시작 (팀원 4명 × 20회)

## Day 1 — 손 관절 · STT 기반
- [ ] `hand_pose_node.py`: MediaPipe 2D 21점 + 정렬 깊이 → 3D (결측 픽셀 주변 보간) → tf2로 base_link 변환 → `HandState` 30 Hz
- [ ] 손바닥 좌표계(손목·검지 기저·새끼 기저), `palm_pose`
- [ ] `distance_to_centerline` 계산 (table.yaml 중앙선 기준)
- [ ] **기준선**: `ros2 run rally_voice get_keyword` (선생님 코드, `resource/.env` 에 API 키) 동작 확인
- [ ] `intent_node.py`: 선생님 `WakeupWord`·`STT` 그대로 쓰고 LLM 프롬프트만 RALLY 의도 목록으로 → `VoiceIntent` 토픽, "그만/멈춰" 즉시 `is_emergency`

## Day 2 — 제스처 · 의도 · TTS
- [ ] `gesture.py`: 손가락 굽힘 각도 → `grip_closure`, 주먹/펼침/가리키기 분류, 주먹 → `/rally/safety/estop` 직결
- [ ] `wrist_angle_deg`: 전완 방향 대비 손바닥 회전각
- [ ] `intent_node.py`: LLM function calling → `VoiceIntent` (intent 목록은 msg 주석), "그만/멈춰" 키워드 스팟팅은 STT 전에 처리 → `is_emergency`
- [ ] `tts_node.py`: `Speak` srv, Piper TTS, 발화 중 마이크 차단 플래그
- [ ] 한국어 웨이크워드 모델 학습, 오탐 측정 (목표 ≤ 1회/10분)

## Day 3 — 모드 5 ✅
- [ ] (정정인과) `HandState` → servol 추종 통합: 지연 측정, 필터 파라미터 공유
- [ ] 잡는 순간 손목 각도 기록 (`Handover.result.wrist_angle_at_grasp_deg` 입력)
- [ ] 음성 시나리오: "랠리야, 훈련 시작" → "건네기 시작할게요" → … 문구 초안, 되묻기 문구
- [ ] 모드 5 통합 시연 참여

## Day 4 — 모드 3 + 대시보드 골격
- [ ] `body_pose_node.py`: 웹캠 MediaPipe Pose → 몸통 기울임·어깨 들림 → `BodyPose`, 보상 동작 플래그(임계 15°)
- [ ] (박성현과) 릴리스 순간 검출: 손 마스크·공 마스크 분리 + 손 속도 피크 교차 확인
- [ ] 대시보드 골격: FastAPI + WebSocket(Redis `rally:events` 구독), 페이지 레이아웃(힘 그래프 / 예측 vs 실제 / 학습 곡선 / 세션 리포트 / 안전 상태)

## Day 5 — 모드 1 지원 + 대시보드
- [ ] 환자 손 3D 위치 → 모드 1 목표 지점 제공 (brain이 `PushBall.target`으로 사용)
- [ ] 대시보드: 예측 vs 실제 정지 위치 산점도, 시도별 오차 추이(학습 곡선)
- [ ] 대시보드: `/rally/robot/wrench` 실시간 힘 그래프 (Timescale 최근 10 s)

## Day 6 — 리포트 화면 ✅
- [ ] 세션 리포트 뷰 (조정묵 LLM 리포트 markdown 렌더링)
- [ ] 음성 피드백 문구 세트: 난이도 상승/하향, 휴식 제안, 자세 교정("몸을 덜 기울여 보세요")
- [ ] 안전 상태 배너 (SLOW/STOP 사유 표시), 대시보드 비상정지 버튼 → `/rally/safety/estop`

## Day 7 — 통합·안정화
- [ ] 시연장 소음에서 호출어·STT 재측정, 임계 조정
- [ ] 손 검출률 측정(정면 ≥ 95 %), 결측 시 추종 중단 동작 확인
- [ ] 대시보드 1 s 이내 갱신 확인

## Day 8 — 시연
- [ ] 대시보드 모니터 세팅, 리허설 중 화면 운영
- [ ] 발표: 손 관절·음성 파트, 대시보드 시연

## 메모 / 막힌 것
- 
