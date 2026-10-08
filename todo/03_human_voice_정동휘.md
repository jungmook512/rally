# TODO — 사람 인식 · 음성 (로봇·인식 PC hand_pose/voice 컨테이너) · 정동휘

패키지: `ros2_ws/src/rally_human`, `rally_voice` · 책임 모드: **모드 5 (건네고 받기)**
참고: `docs/INTERFACES.md` 2.2 / 3

## 10/8 회의 결정 — 모드 1 에서 손 인식이 하는 일 (정정인·조정묵 요청)
brain 모드 1 이 `/rally/perception/hand`(HandState)를 이렇게 쓴다 (`rally_brain/trees/mode1_push.py`, 로봇 손 보기 자세에서 RealSense):
- **레인 고르기**: `palm_pose.position.y`(**base_link**) 에 가장 가까운 레인 → header.frame_id 가 base_link 가 아니면 무작위로 대체
- **잘 받음 판정**: `grip_closure` ≥ 0.6 을 0.3 s 유지(쥠) → ≤ 0.3 (폄)
- **손 빼기**: `distance_to_centerline` ≥ 0.25 m 가 되면 로봇이 공을 회수
- [ ] 로봇·인식 PC 에서 `hand_pose` 를 RealSense 토픽에 맞게 실행 — `vision_local.sh` 카메라는 `/camera/color/image_raw`·`/camera/aligned_depth_to_color/image_raw`·`/camera/color/camera_info` (노드 기본값 `/camera/camera/...` 과 다름)
- [ ] 카메라 → base_link tf 가 이미지 시각 기준으로 나오는지 (팔이 손 보기 자세로 움직임) — 안 되면 distance NaN → 안전감시 SLOW
- [ ] **테니스공을 쥔 손이 주먹(GESTURE_FIST → 비상정지)으로 잡히는지** 확인 — 잡히면 매 시도마다 로봇이 멈춤. 안전 규칙이라 임의로 끄지 말고 팀과 결정
- [ ] 공을 쥐었다 폈을 때 `grip_closure` 값 범위 기록 → 임계값(0.6 / 0.3) 확정

## 지금 상태 (10/8 기준) — 한눈에
**한 것 (머지)**: 손 인식 노드 #44 · 음성 의도 노드 #46 · TTS 노드 #53 · 모드 1 칼만 예측 코드 #67(모드 1 은 보류, #69·#70) · 손목 각도 버그 수정 + 확인 창 3개 #74
**한 것 (PR·브랜치)**: 비상어 상시 감지 연결 + 학습 도구 #50 (Draft, push 됨) · 손 인식 체크리스트 1~7 통과(웹캠·RealSense)

**할 것 (순서)**
1. RealSense 비면: 손목 각도 안정화 확인 → PR (`feat/rally_human/wrist-filter`) · 손 따라가기 지연·떨림 측정 (`feat/rally_human/hand-follow-measure`) → 결과 정정인에게
2. #77 머지되면: `fix/rally_human/camera-tf-compat` PR (depth_scale·tf 대체)
3. 정정인 NaN→SLOW + #77 들어오면: 체크리스트 8~10 → 주먹 정지 제거 PR (#59 3단계)
4. 정정인 follow_hand 준비되면: 모드 5 손 따라가기 통합·통합 시연
5. 조정묵 #50 결과 오면: 판정·임계 결정 → #50 마무리 (Draft 해제·리뷰)
6. Day 4: 3D 출력물 마감, `body_pose`, 릴리스 검출(박성현과)

**기다리는 것**
| 무엇 | 누구 | 어디 |
|---|---|---|
| NaN→SLOW · 손 데이터 끊김 감지 · 접촉 예정 중 거리 STOP→SLOW | 정정인 | #59 |
| 잡는 순간 손목 각도: 안 보이면 0 → NaN, 오래된 값 확인 | 정정인 | #79 |
| #77 리뷰 답(10 Hz tf·켜는 순서·높이 검증) · 머지 · 관절 토픽 확인 | 박성현·정정인 | #77 |
| 손목 각도 범위 [-90,90] → [0,180] (또는 −90 변환), NaN 처리 | 조정묵 | #68 |
| GPU PC 비상어 녹음·학습·실측 (시연 화자 = 조정묵) | 조정묵 | #50 |

## Day 0 (10/5)
- [x] 웹캠 60 fps 확인(`v4l2-ctl`), 환자 측면 위치 가설정 — C270 640×480 은 약 20 fps (이슈 #38)
- [x] MediaPipe Hands·Pose 단독 실행, RealSense 컬러에서 손 검출 확인 — `scripts/check_mediapipe.py`. 손은 잘 잡힘. 전신은 카메라에 바짝 붙으면 약하고, **1 m 이상 떨어지면 잘 잡힘** (10/8 노트북 웹캠: 1~1.5 m, 책상 뒤에 앉아 골반이 가려져도 어깨·팔꿈치 안정, 몸통을 기울이면 어깨선이 같이 기울어짐)
- [!] USB 마이크·스피커 장치 확인(`arecord -l`, `aplay -l`), 컨테이너 `/dev/snd` 패스스루 테스트 — USB 마이크 없음 → 시연은 로봇·인식 PC 내장 마이크로 결정 (10/7). 컨테이너 `/dev/snd` 는 #50 녹음 때 그 PC 에서 확인
- [x] openWakeWord·faster-whisper·Piper 설치, 한국어 Piper 음성 모델 다운로드 → `models/` — openWakeWord·Piper(`models/piper/ko_KR-kss-medium`) 완료. faster-whisper(로컬 STT)는 팀 결정 전까지 보류
- [ ] 웨이크워드 "랠리야" 녹음 샘플 수집 시작 (팀원 4명 × 20회) — 비상어 녹음(#50)에 합침: 1인 랠리야 30회. 로봇·인식 PC 녹음은 조정묵 진행 (10/8)

## Day 1 — 손 관절 · STT 기반
- [x] `hand_pose_node.py`: MediaPipe 2D 21점 + 정렬 깊이 → 3D (결측 픽셀 주변 보간) → tf2로 base_link 변환 → `HandState` 30 Hz — bag 재생 19~26 Hz(노트북 CPU), base_link 변환은 로봇·인식 PC에서 확인 필요
- [x] 손바닥 좌표계(손목·검지 기저·새끼 기저), `palm_pose`
- [x] `distance_to_centerline` 계산 (table.yaml 중앙선 기준) — tf 없으면 NaN
- [x] **기준선**: `ros2 run rally_voice get_keyword` (선생님 코드, `resource/.env` 에 API 키) 동작 확인 — 키 확인됨. 팀 키는 gpt-4o·whisper-1 만 허용
- [x] `intent_node.py`: 선생님 `WakeupWord`·`STT` 그대로 쓰고 LLM 프롬프트만 RALLY 의도 목록으로 → `VoiceIntent` 토픽, "그만/멈춰" 즉시 `is_emergency` — 규칙 먼저, 애매한 말만 gpt-4o (0.8~2 s)

## Day 2 — 제스처 · 의도 · TTS
- [x] `gesture.py`: 손가락 굽힘 각도 → `grip_closure`, 주먹/펼침/가리키기 분류, 주먹 → `/rally/safety/estop` 직결 — 5 프레임일 때만 확정 (손 놓친 프레임은 건너뜀, 3 프레임 넘게 놓치면 초기화)
- [x] `wrist_angle_deg`: 전완 방향 대비 손바닥 회전각 — 팔꿈치가 없어 손 방향을 아래팔 축으로 근사 (0 손바닥 아래 · 90 옆 · 180 위)
- [x] `intent_node.py`: LLM function calling → `VoiceIntent` (intent 목록은 msg 주석), "그만/멈춰" 키워드 스팟팅은 STT 전에 처리 → `is_emergency` — function calling 완료(#46). 호출어 없이 항상 듣는 비상어 감지 연결 완료 (#50, 10/8): 녹음·STT 중에도 듣고, 같은 말로 정지 2번 안 나감
- [x] `tts_node.py`: `Speak` srv, Piper TTS, 발화 중 마이크 차단 플래그 — #53. 4 s 문장 합성 0.37 s, speaking false→true→false 확인. 로봇·인식 PC 스피커는 확인 필요
- [x] 손목 각도 버그 수정 — 깊이가 없으면 0° 로 나가던 것 + 확인 창 3개 (`scripts/check_hand_live.py`·`check_gesture_fingers.py`·`check_hand_depth.py`) — #74 (10/8). #58(주먹 정지 실행 중 끄기)은 #59 결정으로 닫음
- [ ] 손 인식 실기 확인 (체크리스트 https://claude.ai/artifact/84vs1ciQAmjyMbz8Pu9G2X) — **1~7 통과** (웹캠 6/6, RealSense 3번 손 거리 차이 중간값 5.0 mm, 주먹·펼침 100 %, 검지 24 % 는 쓰는 곳 없음). **8~10 대기**: #77(tf) + 정정인 NaN→SLOW. 11번(물건 쥔 손 = 주먹)은 주먹 정지 제거로 불필요
- [ ] 한국어 웨이크워드 모델 학습, 오탐 측정 (목표 ≤ 1회/10분) — **비상어 "그만"·"멈춰" 키워드 모델도 같이** (기획 CP9·FR-07: STT 없이 키워드 스팟팅, 0.3 s 이내, 네트워크 안 탐). #50 (Draft)
  - [x] 녹음·학습 도구, "랠리야" 같은 모델로, 오탐 구간 재학습 (노트북 1~3차: 오탐 시간당 200 → 25회)
  - [x] intent_node 상시 감지 연결 (10/8)
  - [ ] 로봇·인식 PC 녹음·학습·실측 — **조정묵 진행** (순서 https://claude.ai/artifact/BjYeoXqdNCxuM3Pw4Rwtva, 시연 화자 = 조정묵)
  - [ ] 정동휘: 결과 판정·임계 결정 → #50 Draft 해제·리뷰, "랠리야" 호출어 교체 여부

## Day 3 — 모드 5 ✅
- [ ] (정정인과) `HandState` → servol 추종 통합: 지연 측정, 필터 파라미터 공유 — 측정 스크립트 준비(`scripts/measure_hand_jitter.py`, 브랜치 `feat/rally_human/hand-follow-measure` push 전). RealSense 로 30·45·60 cm 측정 → 정정인에게. 정정인 follow_hand 는 아직 (#57)
- [ ] 잡는 순간 손목 각도 기록 (`Handover.result.wrist_angle_at_grasp_deg` 입력)
  - [x] 깊이 없을 때 0° 버그 수정 (#74)
  - [ ] 손목 각도 안정화: 너무 가까운 프레임(손바닥 < 0.30 m·깊이 끊김) 빼고 0.5 s 중간값, 가까우면 직전 값 유지, 1 s 놓치면 NaN — 브랜치 `feat/rally_human/wrist-filter` (push 전). 웹캠으로 동작 확인, **RealSense 로 30 cm 기준 확인 후 PR**. 이유: 잡는 순간 손–카메라 10~30 cm 에서 ±20° 흔들림 (40~60 cm 는 ±10°)
  - [ ] 기록 쪽(정정인 `handover.py` 106행): 안 보이면 0 → NaN, 오래된 값 확인 → #79. 범위(조정묵 `embeddings.yaml`) → #68 댓글
- [ ] 음성 시나리오: "랠리야, 훈련 시작" → "건네기 시작할게요" → … 문구 초안, 되묻기 문구 — 팀원에게 맡김 (10/8). 받으면 `phrase_problem()` 으로 비상어 검사 (#72)
- [ ] #77(카메라 tf) 대응: 손 깊이에 `depth_scale`(0.971) 곱하기 + 영상 시각 tf 없으면 0.15 s 안 최신 tf 로 대체 — 브랜치 `fix/rally_human/camera-tf-compat` (커밋됨, push 전) → **#77 머지 후 PR**
- [ ] 주먹 정지 제거 (#59 3단계, 정정인 동의): 정정인 NaN→SLOW → tf(#77) → 체크리스트 8~10 → 손 노드 주먹 → `/rally/safety/estop` 발행 제거 PR + 기획서 "3중 정지"·FR-07 문구
- [ ] 모드 5 통합 시연 참여

## Day 4 — 모드 3
- [ ] `body_pose_node.py`: 웹캠 MediaPipe Pose → 몸통 기울임·어깨 들림 → `BodyPose`, 보상 동작 플래그(임계 15°)
  - 10/8 확인 기반 설계 방향: 몸통 기울임은 **어깨선 기울기** 위주 (골반이 책상에 가려져도 됨), 골반이 보이면 어깨 중점–골반 중점 선도 같이. 어깨 들림은 어깨–귀 거리 줄어듦
  - 측면 웹캠은 환자에게서 **1 m 이상** (위치는 정동휘가 정함). 모드 3 공 추적(#38)에도 같이 쓰면 공 비행 구간과 환자 상체가 같이 보이게
- [ ] (박성현과) 릴리스 순간 검출: 손 마스크·공 마스크 분리 + 손 속도 피크 교차 확인 — 요격 타이머의 시작점이라 지연 최소화
- [ ] 릴리스 속도·각도(손목 속도 벡터) 계산 → TrialResult metrics (환자 던지기 지표)
- [ ] **3D 출력: 패드 손잡이** (마감 Day 4, 10/8 출력 걸어 둠) — 벨크로 패드(지름 15 cm) 뒤에 붙여 RG2 가 잡는 손잡이. 지금은 육각렌치를 테이프로 붙여 씀. RealSense 시야를 가리지 않게 패드에서 튀어나오는 구조, 충격 시 회전 방지 (기획서 CP12)
- [ ] **3D 출력: 패드 거치대** (마감 Day 4, 10/8 출력 걸어 둠) — 로봇이 받은 패드를 걸어두는 곳. 정정인 "패드 거치대에 걸기 동작" · `DetachBall`(공 떼기) 이 이 위에서 이뤄지므로 위치·높이·패드 면 방향을 정정인과 맞춤

## Day 5 — 모드 1 · 7 지원
- [ ] **모드 1 `rally_vision/predictor.py`** (박성현 항목을 맡음, 10/7): 칼만(위치·속도, 등속) + 마찰 감속 → `velocity`·`predicted_stop`·`predicted_stop_time`·`prediction_confidence`. 브랜치 `feat/rally_vision/kalman-predictor`
  - **보류 (#69, 10/7)**: 모드 1 은 밀기 전 레시피 예측으로 한다 (길이 일직선 → 맞출 값이 정지 거리 하나, 밀고 나면 못 고침, 굴러가는 공을 볼 카메라 없음). 칼만은 모드 3 에 먼저 쓰고, 그때 모드 1 에 붙일지 정함. 코드는 유지
  - [x] 구현: numpy 순수 클래스 (`filterpy` 는 vision requirements 에 이미 있음), `header.stamp` 기준 dt, 미검출 프레임은 predict 만
  - [x] 검증 1 (로봇 없이): 합성 궤적(마찰 감속 + 잡음 1 cm) 단위 테스트 → 정지점 오차 — confidence≥0.5 시점 평균 6~21 mm, 굴러간 거리 절반 지점 13~34 mm (`python3 -m rally_vision.rolling_sim`)
  - [ ] 검증 2 (로봇·인식 PC): 공 굴리기 bag 재생 → 예측 vs 실제 정지점 (목표: 0.3 m/s 이하 ≤ 30 mm) — 보류 (#69)
  - [x] 마찰 계수 온라인 추정 (매 시도 실제 궤적으로 갱신) — 한 시도 추정은 ±30~50 % 흔들려 여러 시도 평균. 실측 잡음이 작으면 나아짐
  - [x] PR (리뷰어 박성현) → `tracker_node` 연결은 박성현 — #67 머지. tracker 연결은 보류 (#69)
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
- (10/7) 3D 출력물(패드 손잡이·거치대)은 정동휘 담당, Day 4 마감. 릴리스 순간 검출은 박성현과 공동 그대로 (넘긴 항목 없음)
- (10/7) 모드 3 실측값이 계획과 다른 점 → 이슈 #38 (웹캠 20 fps, 가속 1000 mm/s² 로 0.5 s 에 6~12 cm, 비행 0.40~0.55 s, 색 오검출, servol 연속 추종 위험)
- (조정묵, 10/6) `rally_human/setup.py` 에 `hand_pose`, `body_pose` 실행 등록(console_scripts)이 없어 `pc2_perception.launch.py part:=human` 이 두 노드를 건너뛴다. 노드 이름을 이 두 개로 등록하면 launch 가 바로 띄운다. `intent_node`·`tts_node` 는 등록돼 있지만 아직 `main()` 이 없어 켜자마자 종료됨 → 해결: hand_pose 등록(#44), intent_node(#46)·tts_node(#53) main 있음. body_pose 는 Day 4
- (조정묵, 10/7) 음성으로 훈련 이름 고르기 요청: "공 굴리기 하자" → `intent: next_mode` + 슬롯 `mode=1` (1 공 굴리기 · 3 캐치볼 · 5 건네고 받기 · 7 팔씨름). 약속은 `docs/INTERFACES.md` 2.2 표 아래. `intent_node` 의 function calling 스키마에 `mode` 인자만 추가하면 됨, 메시지 변경 없음
- (10/8) #35 완료: `.dockerignore`(`**/.env`) 로 이미지에 키 안 들어감, `rally_voice/setup.py` 는 그대로 (네이티브 STT 유지). 로봇·인식 PC 예전 voice 이미지 확인만 남음 (학습 순서 5단계)
- (10/8) #59: 정정인 주먹 정지 제거 동의 — 순서 NaN→SLOW·끊김 감지 → tf → 체크리스트 8~10 → 제거. #57 제시 위치가 중앙선 앞 6 cm 라 손 거리 STOP 이 건네기를 중단시킴 → "접촉 예정 중 STOP→SLOW" 요청
- (10/8) #78 (LLM 은 판단·설명에만) 동의 댓글. 음성 의도는 LLM 유지, 비상어는 키워드
- (10/8) 로컬 브랜치 (push 전): `feat/rally_human/wrist-filter` · `fix/rally_human/camera-tf-compat` · `feat/rally_human/hand-follow-measure` — 셋 다 `hand_pose_node.py` 를 고쳐 나중 것은 충돌 해결 필요
- (10/7) 모드 1 칼만 예측(`rally_vision/predictor.py`)을 박성현 대신 맡음. 칼만 검증 → 완성 순서. 모드 3 착지 예측에 칼만 전처리 넣는 건 그 다음에 이슈 #38 에서 제안 → (10/7 #69) 순서 바뀜: 모드 1 은 레시피, 칼만은 모드 3 에 먼저
