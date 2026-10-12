# 실로봇 실행 가이드 (로봇·인식 PC, 혼자 시험할 때)

터미널 **3개**로 나눈다. 1·2 는 띄워 둔 채로 두고, 끌 때는 그 터미널에서 **Ctrl+C**.
서버 PC brain·대시보드와 붙이는 전체 실행은 `README.md` 순서를 따른다 (이 문서는 로컬 모드 `RALLY_LOCAL=1`).

## 0. 시작 전

- 컨트롤러 전원 ON, 펜던트 **자동 모드**, 서보 ON
- 코드를 고쳤으면 빌드: `cd ~/rally/ros2_ws && colcon build --packages-up-to rally_bringup`

## 1. 터미널 1 — 두산 bringup

```bash
cd ~/rally && scripts/robot.sh bringup
```
`Successfully switched controllers` 가 나오면 터미널 2 로. 이미 떠 있으면 거부한다 (두 번 띄우면 컨트롤러 연결이 꼬임).

## 2. 터미널 2 — 안전감시 · 힘 · 그리퍼 · 스킬서버 (+ 비전)

```bash
cd ~/rally && scripts/robot.sh nodes             # 비전 컨테이너도 같이 (Ctrl+C 하면 비전도 꺼짐)
cd ~/rally && scripts/robot.sh nodes novision    # 비전 없이 (모드 5 수동 좌표, 레시피 --manual)
```
`skill_server ready` 가 나오면 터미널 3 에서 명령. 이 터미널에 로봇 노드 로그가 그대로 나온다.

## 2-1. 손 인식 (모드 1·5, 손 거리 안전 감시)

```bash
cd ~/rally && scripts/hand_local.sh          # 손 + (C270 이 꽂혀 있으면) 몸 자세 — 자동. 비전(RealSense)이 떠 있어야 함
cd ~/rally && scripts/hand_local.sh hand     # 손만 (C270 이 있어도 몸 자세 끔)
cd ~/rally && scripts/hand_local.sh stop     # 끄기
```
- 컨테이너라 터미널을 잡지 않는다. 확인: `ros2 topic echo /rally/perception/hand --once` (손을 RealSense 에 보이기)
- 이미지가 없거나 #112 전 빌드면: `docker build -f docker/hand_pose/Dockerfile -t rally/hand_pose:local .` → 로그(`scripts/hand_local.sh logs`)에 `중앙선 x=0.568 m, 깊이 배율 0.971`
- 몸 자세: C270 을 이름으로 찾아 자동으로 켬. 마지막 줄 `[hand] 몸 자세: 켬/끔 (이유)` 로 확인. 기준은 **바르게 선 뒤** `ros2 service call /rally/perception/body/reset_baseline std_srvs/srv/Trigger`

## 3-1. 터미널 4 — brain (모드를 "세션"으로 돌릴 때만)

```bash
cd ~/rally && scripts/robot.sh brain             # context_builder + bt_runner (준비 확인 끔, DB·LLM·음성 없이)
```
그다음 터미널 3 에서 `scripts/robot.sh session 1` → 모드 1 세션 (10/8 회의 흐름, 공 4번):
손 보기 자세 → 환자 손에 가까운 레인 → 레인별 고정 목표(개발용 입구 +35 cm, 실측·안전·속도 상한 0.37 범위 제한) → 손 시작 거리(reach 초기값·단계 = 레인 RMSE × 2) 및 NORMAL 확인 → 거치대 공 집어 치기 →
타격 후 새 표본으로 펼침→쥠 유지→펼침을 관측하면 받기 후보(실물 검증 전 자동 난이도 차단) → 손 이탈을 최신 관측으로 확인할 때까지 세션을 유지하며 대기한 뒤 로봇이 레인 공을 집어 거치대 맨 윗부분에 넣기.
필요한 것: 손 인식(`/rally/perception/hand`, 정동휘 hand_pose) · 비전 · `skills.yaml mode1.rack_drop_posx`(아래 직접교시).
모드 5 는 `session 5`. 처음 한 번만 brain 의존성 설치: `python3 -m pip install --target ~/.local/rally_brain_py py_trees "rdflib>=6.1,<8"`

## 3. 터미널 3 — 명령

```bash
cd ~/rally && source scripts/robot.sh env        # 터미널 열 때마다 한 번
```

필요할 때 하나씩 (차례로 치는 목록 아님):

```bash
scripts/robot.sh status          # 뭐가 떠 있는지 + 컨트롤러 점검 + 비전 공 한 번
scripts/robot.sh observe         # 관측 자세로 (특이점이라 힘이 0 일 때)
scripts/robot.sh estop           # 멈출 때만: 소프트 비상정지 → 이후 동작 거부. 풀려면 scripts/robot.sh reset
scripts/robot.sh down            # 꼬였을 때만: 남은 bringup·노드·비전 전부 정리 → 다시 터미널 1, 2
```

### 모드 1 — 공 굴리기 (거치대 공 비전 집기 → 레인 입구 → 치기 → 환자 손 보기)

```bash
ros2 action send_goal /rally/skill/push_ball rally_interfaces/action/PushBall "{lane: 1, push_speed: 0.3}" --feedback
```
- `lane`: 0 왼쪽 · 1 가운데 · 2 오른쪽. `push_speed`: 0.10 ~ 0.60 m/s
- 거치대에 공이 여러 개면 x 가 가장 작은 공(로봇 쪽)을 집는다

**레시피 실측** (속도 → 굴러간 거리)
```bash
python3 scripts/push_recipe.py --lane 1 --speeds 0.2 0.3 0.4 0.5 --repeats 3          # 정지 위치 비전 (못 재면 cm 입력)
python3 scripts/push_recipe.py --lane 1 --speeds 0.2 0.3 0.4 0.5 --repeats 3 --manual # 줄자만
```
끝에 나오는 `recipe: {decel_mps2: …, d0_m: …}` 를 `skills.yaml mode1.lanes[레인]` 에 넣는다.

**거치대 맨 윗부분 직접교시 (공 회수용, 한 번)**: 수동 모드 → 공을 쥐고 손끝을 거치대 레일 위쪽 끝 위(공이 굴러 내려갈 자리)에 →
`python3 scripts/print_pose.py` → 나온 `[x, y, z, a, b, c]` 를 `skills.yaml mode1.rack_drop_posx` 에 → 터미널 2 재시작.

**공 회수만 따로**:
```bash
ros2 service call /rally/robot/go_hand_view std_srvs/srv/Trigger                                       # 손 보기 자세
ros2 action send_goal /rally/skill/return_ball rally_interfaces/action/ReturnBall "{lane: -1, timeout_s: 20.0}" --feedback   # 레인 공 → 거치대
```

### 모드 5 — 건네고 받기

**세션 (비전, 10/11~)**: 터미널 2 를 모드 5 모델로 띄우고 `session 5`.
```bash
MODEL=yolov8n-seg_864.pt IMGSZ=864 scripts/robot.sh nodes     # 터미널 2 — 막대·컵 분할 모델
scripts/robot.sh session 5                                       # 터미널 3
```
흐름: 관측 자세 → 물체 보기 자세(`skills.yaml mode5.pick_view_posx`, 흰 배경이 화면에 다 들어오게) → 보이는 막대·컵 중 무작위 →
마스크 주축 방향으로 그리퍼를 돌려 집기(높이 = 테이블 + `mode5.grasp_height_m`) → 기울여 건네기(당기면 놓음) → 물러난 자리에서 그리퍼 열고 대기 →
**환자가 물체를 그리퍼 사이로 넣어 준다**(4 N, 로봇은 손을 따라가지 않음 #134) → 집었던 자리에 내려놓기.
- 막대·컵은 흰 배경 위에 아무 방향으로 둔다(그리퍼가 마스크 방향에 맞춰 돈다). 막대 굵기 14.5 mm.
- 속도는 `skills.yaml mode5.speed_scale` 하나로 조절(기본 0.5). 손 근접 STOP 이 풀리면 하던 단계를 이어 간다(#115, `brain.hand_stop_wait_modes`).
- 물체 보기 자세를 다시 잡을 때: `python3 scripts/jog.py --dy -0.10` (NORMAL 일 때만, 도착 posx 를 `pick_view_posx` 에 옮겨 적기).

**수동 좌표 (비전 없이)**: 수동 모드에서 손끝을 막대 가운데에 대고 `python3 scripts/print_pose.py` → x, y 기록 → 자동 모드.
`yaw: 0.0` 이면 그리퍼를 돌리지 않으니 막대를 **y 방향(보라색 한계선과 평행)** 으로 눕힌다.

```bash
# 단계별 (X, Y 는 잰 값, m)
ros2 action send_goal /rally/skill/pick_compliant rally_interfaces/action/PickCompliant "{target: {header: {frame_id: base_link}, pose: {position: {x: X, y: Y}}}, yaw: 0.0, grip_force_n: 20.0, z_stiffness: 300.0}" --feedback
ros2 action send_goal /rally/skill/handover rally_interfaces/action/Handover "{direction: 0, object_class: rod, tilt_deg: 0.0, timeout_s: 30.0}" --feedback   # 건네기
ros2 action send_goal /rally/skill/handover rally_interfaces/action/Handover "{direction: 1, object_class: rod, tilt_deg: 0.0, timeout_s: 30.0}" --feedback   # 받기(환자가 넣어 줌) → 제자리

# 한 사이클 자동 (집기 → 기울여 건네기 → 받기 → 제자리)
python3 scripts/mode5_cycle.py --cycles 1 --manual rod:X,Y,0 --tilt 30 30
python3 scripts/mode5_cycle.py --cycles 2 --manual rod:X,Y,0 --tilt 45 90
```

### 자주 쓰는 확인

```bash
python3 scripts/check_controller.py                         # 컨트롤러·힘·특이점 점검
python3 scripts/print_pose.py                               # 지금 손끝 좌표 (수동 모드에서도)
ros2 topic echo /rally/perception/objects --once            # 비전 (frame_id: base_link 여야 함)
ros2 service call /rally/gripper/command rally_interfaces/srv/GripperCommand "{width_mm: 90.0, force_n: 20.0, wait_until_done: true}"   # 그리퍼 열기 (닫기: width_mm 0.0)
```

## 4. 실로봇 없이 — 시뮬 (RViz)

```bash
scripts/sim_robot.sh up          # 에뮬레이터 + RViz + 가상 그리퍼·비전·막대·환자 손 (도메인 77, 실로봇과 안 섞임)
SIM_GRIPPER_ARGS="--refill --mode1-patient" scripts/sim_robot.sh up   # 모드 1 세션용: 가짜 손 인식(레인 앞 손·쥐었다 폄·공 굴려 줌)
source scripts/sim_robot.sh env  # 명령 터미널 — 위 3절 명령을 그대로 (모드 5 막대는 rod:0.30,-0.25,0)
scripts/sim_robot.sh down
```

## 문제가 생기면

| 증상 | 확인 |
|---|---|
| 힘이 0, 접촉 감지 안 됨 | 특이점 자세 → `scripts/robot.sh observe` |
| goal 거부 | 안전 STOP → 원인 해소 후 `scripts/robot.sh reset` |
| `not_arrived` · `amovel_rejected` | 터미널 1 bringup 로그의 알람 (관절 한계·속도 한계·특이점) |
| 비전 공 없음 | `scripts/vision_local.sh logs`, 거치대가 카메라에 보이는지 |
| 그리퍼 서비스 대기만 | 시뮬 도메인(77) 터미널인지 확인 → `source scripts/robot.sh env` |
| "more than one action server" · goal 거부 | 로봇 노드 중복 → `scripts/robot.sh down` 후 터미널 1, 2 다시 |
| 로그 보기 | 터미널 1(bringup)·2(노드)에 그대로 나온다. 비전은 `scripts/vision_local.sh logs` |
