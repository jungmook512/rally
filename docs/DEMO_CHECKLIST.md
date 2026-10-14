# 시연 기동 체크리스트 (리허설 10/12 · 시연 10/13)

위에서부터 체크하며 따라간다. 리허설 때 실제로 해 보고 틀린 곳은 이 파일을 바로 고친다.
두 PC를 같이 쓰는 순서다 (`full_system.launch.py` 와 같은 순서). `scripts/robot.sh` 는 모든 명령이 **혼자 시험용 로컬 모드**(서버 PC 없이)로 바꿔 실행하므로 시연에는 쓰지 않는다 — 두 PC 설정에서는 서로 안 닿는다. 비상정지는 `scripts/estop.sh`(지금 터미널 설정 그대로 씀)를 쓴다.

- 서버 PC = brain·DB·대시보드 (compose `pc3`) · 로봇·인식 PC = 두산·로봇 노드(네이티브 `pc1`) + 추론 컨테이너(compose `pc2`)
- 로봇·인식 PC의 **네이티브 터미널은 매번 `source scripts/env_robot_pc.sh` 먼저** (없으면 컨테이너·서버 PC와 서로 못 찾는다)
- ⏳ 표시는 아직 머지 전인 PR이 필요한 항목. 리허설 전에 머지됐는지 확인 (10/14: #136·#139·#140·#143 머지돼 표시 뺌. 남은 ⏳ 없음)

## 0. 전날·당일 아침 준비

- [ ] 두 PC 모두 `main` 최신: `cd ~/rally && git checkout main && git pull`
- [ ] 로봇·인식 PC 네이티브 빌드: `cd ros2_ws && colcon build --symlink-install --packages-up-to rally_bringup rally_robot && cd ..`
- [ ] 서버 PC `docker/.env`
  - `PC3_IP` = 서버 PC IP, `ROS_DOMAIN_ID` 두 PC 같게
  - `OPENAI_API_KEY` 한도 남아 있는지 (없어도 리포트는 수치만으로 나옴)
  - `ROBOT_VIDEO_URL=http://<로봇·인식 PC IP>:8080` (대시보드 카메라 칸, #140)
- [ ] 운영 DB에 그래프 표 있음: `docker exec docker-db-1 psql -U rally -d rally -c "\dt kg_*"` → `kg_edge`·`kg_node`
- [ ] 소품: 공(모드 1 거치대 4개), 벨크로 공·패드(모드 3), 막대·컵(모드 5, 흰 배경 위), (모드 7 은 소품 없음 — 로봇 손목을 민다), 바닥 중앙선
- [ ] 카메라 2대 연결: 손목 RealSense(시리얼 `140122076335`), 측면 RealSense(`147122075430`)
- [ ] 로봇·인식 PC `~/rally/models/` 에 음성 모델 두 개 (git 제외, 노션 전달): 비상어 `kws/emergency_kws.joblib` — `python3 -c "import joblib,os; m=joblib.load(os.path.expanduser('~/rally/models/kws/emergency_kws.joblib')); print(m.get('consec'), m['threshold'], m.get('wake'))"` 가 `3 0.9 ['rally']` · Piper `piper/ko_KR-kss-medium.onnx`(+`.json`)
- [ ] 같은 폴더에 비전 모델: 공 `yolov8n-seg_ball.pt` · 막대·컵 `yolov8n-seg_cupstick.pt`(**10/12 손에 든 막대 학습본 cupstick_v2_scratch 를 이 이름으로** — 10/8 파일과 이름이 같으니 날짜·`md5sum` 으로 확인) · `T_gripper2camera.npy` · 모드 3 `velcro_ball_yolo11n-seg_1024_rot_final.(engine|pt)` · `mode3_side_calib.json`.
  확인: 손에 막대를 들고 `ros2 topic echo /rally/perception/objects --once` 에 `rod` 가 나와야 모드 5 받기 기울기 맞춤·다가가기가 돈다 (안 나오면 힘 4 N 으로만 받음)
- [ ] 음성 장치 (#183 뒤 컨테이너 소리·마이크는 데스크톱 오디오(PipeWire) 경유): **데스크톱에 로그인한 사용자로** 진행, 설정 > 소리에서 **입력 = 맥스틸 C타입 이어셋, 출력 = 방 스피커** (이어셋을 꽂으면 출력이 이어셋으로 바뀌는 경우가 많으니 꽂은 뒤 다시 확인). `docker/.env` 의 `VOICE_MIC_DEVICE`·`VOICE_SPEAKER_DEVICE` 는 **비움**
- [ ] 하드웨어 비상정지 버튼 위치를 시연자·운영자 모두 확인

## 1. 서버 PC — 기반 (먼저)

```bash
cd ~/rally/docker && docker compose --profile pc3 up -d discovery db redis
docker compose --profile pc3 ps          # 셋 다 running, db·redis healthy
```
- [ ] discovery·db·redis 실행 중

## 2. 로봇·인식 PC — 로봇 (터미널 3개)

**터미널 1 — 두산 bringup**
```bash
cd ~/rally && source scripts/env_robot_pc.sh && ros2 daemon stop
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100
```
- [ ] `Successfully switched controllers` 로그

**터미널 2 — 안전 감시·힘·그리퍼·스킬 서버**
```bash
cd ~/rally && source scripts/env_robot_pc.sh
ros2 launch rally_bringup pc1_robot.launch.py hand_stop:=slow      # ★ 리허설 결과로 slow / stop 확정 (off 금지)
```
- [ ] safety_monitor 로그 `★ hand_stop_mode=slow …` (stop 이면 `hand_stop_mode=stop (손 근접 STOP 켜짐)`) — **off 가 보이면 끄고 다시**
- [ ] `skill_server ready`

**터미널 3 — 추론 컨테이너 (비전·손·음성)**
```bash
cd ~/rally/docker && docker compose --profile pc2 up -d --build
docker compose --profile pc2 ps          # vision · hand_pose · voice running
docker compose --profile pc2 logs -f vision   # 아래 로그 확인 후 Ctrl+C
```
- [ ] vision 로그 `/side/side 노출 고정: … exposure=20, … gain=128`
- [ ] voice 로그 (`docker compose --profile pc2 logs voice`): `[voice] 소리: 호스트 PipeWire (펄스 소켓)` · `마이크: pulse …` · `호출어 대기 (랠리야, 임계 0.90, 연속 3 프레임)` · `Piper 음성 ko_KR-kss-medium.onnx`. `펄스 소켓 없음`·`hello_rokey`·`상시 감지 꺼짐` 이 보이면 0절 음성 장치·모델부터
- [ ] 카메라 주기 (네이티브 터미널, `source scripts/env_robot_pc.sh` 후)
  - `ros2 topic hz /camera/color/image_raw` ≈ 60 (손목)
  - `ros2 topic hz /side/side/color/image_raw` ≈ 60 (측면, 모드 3)

## 3. 서버 PC — brain·대시보드

```bash
cd ~/rally/docker && docker compose --profile pc3 up -d --build brain dashboard
docker compose --profile pc3 logs --tail 30 brain    # 오류 없는지
```
- [ ] 대시보드 열기: `http://<서버 PC IP>:8000` → 오른쪽 위 "실시간 연결됨"

## 4. 준비 확인 (세션 시작 전)

```bash
# 로봇·인식 PC 네이티브 터미널 (source scripts/env_robot_pc.sh 후) 또는 서버 PC brain 컨테이너 안에서
ros2 run rally_bringup wait_ready --group session --once
```
- [ ] 전부 준비 (관측 자세·물체 보기 서비스 포함). 안 된 항목이 있으면 그 노드부터
- [ ] 대시보드 안전 띠 **NORMAL** (STOP 이면 아래 7절)
- [ ] 대시보드 카메라 칸: 손목·측면 두 화면 (안 되면 `No Image` — 영상만 안 보이는 것, 훈련은 됨)
- [ ] brain 로그 `세션 시작 조건 충족`

## 5. 시연 직전 안전 점검 (3중 정지)

- [ ] **하드웨어 비상정지**: 누르면 멈추는지 → 해제 (두산 펜던트 절차)
- [ ] **소프트 비상정지**: 네이티브 터미널에서 `scripts/estop.sh` → 대시보드 STOP(`estop`) → `scripts/estop.sh reset` → NORMAL
- [ ] **음성**: "그만" → STOP(`voice_stop`) → `scripts/estop.sh reset`
- [ ] **음성 비상어 상시 감지 켜짐**: 터미널 3 intent_node 로그에 `비상어 상시 감지 켬 (… 연속 3 프레임)` 이 있고 `상시 감지 꺼짐` 경고(`kws_enabled=false` 또는 모델 없음)가 없을 것 (#172)
- [ ] 손 STOP 모드 로그를 다시 한 번 (터미널 2 맨 위 `hand_stop_mode=…`)

**시연자(환자 역할)에게 미리 알려 줄 것**
- 서서, **왼팔**로 (측면 웹캠이 왼쪽)
- 모드 3: 로봇 정면 투구 자리에서 패드를 향해 던지기. 공은 운영자가 패드에서 떼어 건넨다
- 모드 5: 막대를 **끝을 잡고** 그리퍼 사이에 넣기 — 가운데를 잡으면 손가락이 끼일 수 있다(닫는 힘 20 N). 받기 대기 중에는 손목이 천천히 돈다
- 모드 7: 로봇 손목(그리퍼 위 J5·J6 몸체)을 밀기 — 그리퍼 손가락·카메라는 밀지 않기. 로봇은 버티다가 물러난다. "이제 손을 떼 주세요" 가 나오면 손을 뗀다
- 로봇을 부를 때는 **"랠리야" → 1초 쉬고 → 명령** (붙여 말하면 명령 앞부분이 잘림 — 감지가 말 끝 +0.4~0.6 s). TTS 가 끝난 뒤에 말하기
- 언제든 "그만"·"멈춰"라고 말하면 멈춘다 (호출어 없이, 2 번 연속 말할 필요 없음)

## 6. 세션 진행

**시작** (로봇·인식 PC 네이티브 터미널, `source scripts/env_robot_pc.sh` 후)
```bash
# 시연 (10/14 부터, skills.yaml brain.mode_select.enabled): modes 를 비우면 '모드 선택 대기' — 10 s 마다 "어떤 모드를 진행할지 말씀해 주세요",
#   음성("랠리야 → 공 굴리기 하자"·"모드 5")이나 대시보드 모드 버튼으로 고른 모드만 N회 → "○○ 모드가 끝났습니다. 다음으로 진행할 모드를 말씀해 주세요" → 다시 대기
ros2 action send_goal /rally/session/run rally_interfaces/action/RunSession "{patient_id: demo, modes: [], trials_per_mode: 0}" --feedback
# 예전처럼 순서대로 돌리려면 modes 를 적는다 (모드 3 은 mode3_challenge.enabled 일 때만)
ros2 action send_goal /rally/session/run rally_interfaces/action/RunSession "{patient_id: demo, modes: [1, 5], trials_per_mode: 0}" --feedback
```
- 시도 수: 모드 1 = 2회(#200), 나머지 모드당 5회 (`trials_per_mode: 0` 일 때)
- 모드 고르기·바꾸기: 음성 "공 굴리기 하자"·"캐치볼"·"건네고 받기 하자" / 대시보드 모드 버튼. 대기 중 "다음"은 안 받음 (모드 이름을 말해야 함)
- 세션 끝내기: 음성 "여기까지 하자"·"세션 끝내자" / 대시보드 **세션 종료** 버튼 — 시도 중이면 그 시도를 마친 뒤 끝남. 비상어(그만·멈춰)는 종료가 아니라 안전 정지

**모드별로 볼 것**

| 모드 | 정상 흐름 | 볼 것 |
|---|---|---|
| 1 공 굴리기 | 관측 자세 → 손 보기 → 공 집기 → 레인 입구 → 치기 → 받기 관측 → 회수 | 손이 가까우면 감속(slow) / stop 이면 "손을 조금 떼어 주세요" 뒤 이어 감 |
| 3 캐치볼 | "공을 던져 주세요" → 투구 → 패드 이동 → 명중 판정 | 두 번 연속 안 던지면 남은 시도 건너뜀, 받기 스킬이 없으면 모드 3 통째로 건너뜀 (세션은 계속) |
| 5 건네고 받기 | 물체 보기 자세 → 집기 → 기울여 건네기 → 받기 → 제자리 | 손가락 끼임 안내, 받기 대기 중 손목 회전 |
| 7 저항 밀기 | 손목 제시 → 버팀 → 목표 힘 유지 → 져주기 → "손을 떼 주세요" → 복귀 | 로봇이 환자 쪽으로 밀지 않는지. 손을 안 떼면 복귀 없이 그 자리에서 끝남(정상). `hand_stop_mode=stop` 이면 제시 이동이 손 근접 STOP 으로 끊기므로 모드 7 은 slow 로 |

**끝**
- [ ] 대시보드 '세션 리포트' 칸에 리포트 (두 번째 세션부터 '지식 그래프 근거' 숫자)
- [ ] brain 로그 `[리포트] 세션 … 발행`

## 7. 문제가 생기면

| 증상 | 할 일 |
|---|---|
| 대시보드 STOP — `force_limit`·`nudge`·`workspace_limit`·`estop`·`voice_stop` (래치) | 원인 확인(사람·물체 접촉?) → 안전하면 `scripts/estop.sh reset`. 세션은 끝나 있으므로 다시 시작 |
| STOP — `hand_near_*`·`hand_lost_near` (stop 모드, 모드 1·5) | 손을 빼면 1 s 뒤 같은 동작을 이어 감. 계속 멈추면 손이 로봇 20 cm 안에 남아 있는 것 |
| STOP — `wrench_missing`·`tcp_missing`·`wrench_lost` | 터미널 1·2 로그 확인 (두산 연결·force_publisher). 특이점 근처면 관측 자세로: `ros2 service call /rally/robot/go_observe_pose std_srvs/srv/Trigger` |
| 세션이 시작 안 됨 (`세션 시작 불가 — 준비 안 됨: …`) | 4절 `wait_ready` 로 빠진 노드 찾기 → 그 터미널부터 다시 |
| 비전이 물체를 못 찾음 | 대시보드 카메라 칸 / `ros2 topic hz /rally/perception/objects`. vision 컨테이너 `docker compose --profile pc2 restart vision` |
| 모드 3 이 계속 실패 | 세션 취소(터미널 Ctrl+C) → `modes: [5, 7]` 로 이어서, 모드 3 은 따로 시연 |
| 꼬여서 처음부터 | 로봇·인식 PC: 터미널 2 → 1 순서로 Ctrl+C, `docker compose --profile pc2 down` → 2절부터. 서버 PC는 그대로 둬도 됨 |

## 8. 끝낸 뒤

- [ ] 로봇을 관측 자세나 홈으로 → 터미널 2 → 1 Ctrl+C
- [ ] 로봇·인식 PC `docker compose --profile pc2 down`, 서버 PC `docker compose --profile pc3 stop brain dashboard` (db·redis 는 기록 보관용으로 둬도 됨)
- [ ] **시연 뒤 개발로 돌아갈 때** 손 STOP 값을 다시 정할 것 (개발 기본은 off)
