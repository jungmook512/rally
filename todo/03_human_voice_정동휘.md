# TODO — 사람 인식 · 음성 (로봇·인식 PC hand_pose/voice 컨테이너) · 정동휘

패키지: `ros2_ws/src/rally_human`, `rally_voice` · 책임 모드: **모드 5 (건네고 받기)**
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
- [x] **기준선**: `ros2 run rally_voice get_keyword` (선생님 코드, `resource/.env` 에 API 키) 동작 확인 — 키 확인됨. 팀 키는 gpt-4o·whisper-1 만 허용
- [x] `intent_node.py`: 선생님 `WakeupWord`·`STT` 그대로 쓰고 LLM 프롬프트만 RALLY 의도 목록으로 → `VoiceIntent` 토픽, "그만/멈춰" 즉시 `is_emergency` — 규칙 먼저, 애매한 말만 gpt-4o (0.8~2 s)

## Day 2 — 제스처 · 의도 · TTS
- [ ] `gesture.py`: 손가락 굽힘 각도 → `grip_closure`, 주먹/펼침/가리키기 분류, 주먹 → `/rally/safety/estop` 직결
- [ ] `wrist_angle_deg`: 전완 방향 대비 손바닥 회전각
- [ ] `intent_node.py`: LLM function calling → `VoiceIntent` (intent 목록은 msg 주석), "그만/멈춰" 키워드 스팟팅은 STT 전에 처리 → `is_emergency` — function calling 완료. STT 전 비상어(호출어 없이 항상 듣기)는 별도 모델 필요로 미룸
- [ ] `tts_node.py`: `Speak` srv, Piper TTS, 발화 중 마이크 차단 플래그
- [ ] 한국어 웨이크워드 모델 학습, 오탐 측정 (목표 ≤ 1회/10분)

## Day 3 — 모드 5 ✅
- [ ] (정정인과) `HandState` → servol 추종 통합: 지연 측정, 필터 파라미터 공유
- [ ] 잡는 순간 손목 각도 기록 (`Handover.result.wrist_angle_at_grasp_deg` 입력)
- [ ] 음성 시나리오: "랠리야, 훈련 시작" → "건네기 시작할게요" → … 문구 초안, 되묻기 문구
- [ ] 모드 5 통합 시연 참여

## Day 4 — 모드 3
- [ ] `body_pose_node.py`: 웹캠 MediaPipe Pose → 몸통 기울임·어깨 들림 → `BodyPose`, 보상 동작 플래그(임계 15°)
- [ ] (박성현과) 릴리스 순간 검출: 손 마스크·공 마스크 분리 + 손 속도 피크 교차 확인 — 요격 타이머의 시작점이라 지연 최소화
- [ ] 릴리스 속도·각도(손목 속도 벡터) 계산 → TrialResult metrics (환자 던지기 지표)

## Day 5 — 모드 1 · 7 지원
- [ ] 모드 7: 손잡이 파지 여부(grip_closure), 손잡이 외 로봇 링크 접근 감지 → safety, 음성 문구("밀어보세요", "2초 버텼어요")
- [ ] 환자 손 3D 위치 → 모드 1 목표 지점 제공 (brain이 `PushBall.target`으로 사용)

## Day 6 — 음성 피드백 ✅
- [ ] 음성 피드백 문구 세트: 난이도 상승/하향, 휴식 제안, 자세 교정("몸을 덜 기울여 보세요")
- [ ] 음성 의도 고도화 (10/7 인터뷰로 정한 요구사항, 조정묵 Day 3 피로도 지표 정의 후):
  - 슬롯 추가: `fatigue`(말투로 본 피로 0~1), `mood`(기분, 정동휘 신규), `body_part`(통증 부위), `pain_level`(약함/심함). 말에 나왔을 때만. DB `events.payload.slots` 에 자동 저장되므로 스키마 변경 없음. 슬롯 이름은 조정묵과 맞춘 뒤 INTERFACES 에
  - 되묻기: 훈련 바꾸기·세션 끝내기만, GPT 확신이 낮을 때 "○○ 할까요?" → 네/아니요 (tts_node 필요). 난이도·휴식은 바로, 비상 정지는 절대 되묻지 않음
  - 화자: 어르신 환자 + 시연 팀원. 정확도 우선 (반응 2~3 s 허용)
  - (선택) 지금 훈련 상황을 GPT 에 같이 주기 (brain 상태 필요)

## Day 7 — 통합·안정화
- [ ] 시연장 소음에서 호출어·STT 재측정, 임계 조정
- [ ] 손 검출률 측정(정면 ≥ 95 %), 결측 시 추종 중단 동작 확인

## Day 8 — 시연
- [ ] 발표: 손 관절·음성 파트

## 메모 / 막힌 것
- (10/6) HMI 대시보드는 조정묵이 맡음 → 대시보드 항목을 모두 `todo/04`로 옮김. 손·몸 인식과 음성에 집중
- (조정묵, 10/6) `rally_human/setup.py` 에 `hand_pose`, `body_pose` 실행 등록(console_scripts)이 없어 `pc2_perception.launch.py part:=human` 이 두 노드를 건너뛴다. 노드 이름을 이 두 개로 등록하면 launch 가 바로 띄운다. `intent_node`·`tts_node` 는 등록돼 있지만 아직 `main()` 이 없어 켜자마자 종료됨
- (조정묵, 10/7) 음성으로 훈련 이름 고르기 요청: "공 굴리기 하자" → `intent: next_mode` + 슬롯 `mode=1` (1 공 굴리기 · 3 캐치볼 · 5 건네고 받기 · 7 팔씨름). 약속은 `docs/INTERFACES.md` 2.2 표 아래. `intent_node` 의 function calling 스키마에 `mode` 인자만 추가하면 됨, 메시지 변경 없음
