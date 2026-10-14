# 모드 3 — 대기 자세 후보(Y0·성현 자세) 실로봇 확인 가이드 (로봇·인식 PC)

정동휘 10/14. 노트북 최적화·시뮬 결과를 **실로봇에서 쓸 수 있는지** 확인하는 순서. **시연까지는 기존 자세 유지 + amovej(박성현 10/14 결정, #195)** 이므로 이 확인은 시연 뒤 또는 로봇 PC 가 비는 시간에. 결과는 이 파일 아래 "기록"에 적고 `MODE3_RESULTS.md` 에 한 줄 링크. 결정은 정정인(안전값·자세)·박성현(예측 평면·학습 bag).

근거: 최적화 `scripts/mode3_ready_pose_time_opt.py`(설명 https://claude.ai/artifact/FxCy7MxEhP3MhcPPkkcoJ6 · 결과 https://claude.ai/artifact/SzpPiSfN84JeYqoxEeRDdt), 시뮬 `data/measure/mode3_reach_1014_*.csv`, 박성현 #195.

## 0. 전제 — 받기 이동은 amovej (#184, 10/13)

`skills.yaml mode3.catch_move: amovej`, 관절 속도 120 °/s · 가속 400 °/s² (500 은 힘 94~106 N). 실측(기존 자세) 20/30 cm 도착 0.40/0.51 s — amovel(0.52/0.71) 보다 빠르다. 최적화 식(movel 관절 속도 상한)은 **특이점·관절 한계·높이·도달 제약**에는 그대로 쓰이지만, **시간은 amovej 로 다시 계산**했다 (가장 많이 도는 관절의 사다리꼴, 120/400).

## 1. 후보 비교 (노트북 모델, 10/14)

| | 기존 [89.8, 30.0, −121.7, 90.2, −80.2, 88.3] | **Y0** [−58.7, 7.0, 100.2, 98.8, 63.6, −100.3] | **성현 자세** [99.5, −3.8, −93.0, 92.3, −70.6, 82.9] (#195) |
|---|---|---|---|
| 패드 중심 (base, m) | (0.345, −0.332, 0.496) | (0.595, −0.423, 0.433) | (0.41, −0.56, 0.52) |
| 24방향 중 되는 방향 (15 cm) | **12** — 왼쪽·아래 반원(105~270°) J3 한계 | **24** | **24** |
| 되는 방향 (25 cm) | 12 | 24 | 22 (150·165° 끝에서 손목-베이스) |
| amovej 15 cm 최악 / 중앙 | 0.60 / 0.48 s (J1) | **0.48 / 0.45 s** (J3) | 0.53 / 0.44 s (J3) |
| amovej 25 cm 최악 / 중앙 | 0.73 / 0.62 s | **0.67 / 0.59 s** | 0.79 / 0.59 s |
| 요격 중 TCP 가 safety 작업 공간 안인가 | 예 | **아니오 — x 최대 0.73 (x_max 0.57)** | x 는 안 (0.40~0.58), **y 가 −0.56 이라 오른쪽 5 cm 부터 y_min −0.60 밖** |
| 팔 모양 | 팔꿈치 아래(J3 −122°) | **팔꿈치 반대로 접힘(J3 +100°)** — 관측 자세에서 가는 경로 확인 필요 | 기존과 같은 쪽(J3 −93°) |
| 팔뚝 ↔ 베이스 기둥 | — | 최소 7 cm (왼쪽·위-왼쪽) | 미계산 |
| 박성현 시뮬 규칙 정책 성공률 (#195) | 28 % | — | 47 % |

- 기존 자세의 문제는 속도가 아니라 **왼쪽·아래 반원이 아예 안 되는 것**. 두 후보 모두 그걸 연다.
- Y0 가 15 cm 에서 20 %, 25 cm 에서 9 % 빠르지만 **safety x 완화 + 팔 모양 반전**이라는 대가. 성현 자세는 기존과 팔 모양이 같아 위험이 작고 safety x 는 그대로 — 대신 y 한계(−0.60)를 오른쪽 범위만큼 풀거나 오른쪽을 12.5 cm 로 자른다(#195 의 "오른 12.5").
- 모델의 amovej 시간은 실측보다 보수적(기존 자세 20 cm: 모델 ~0.53, 실측 0.40). 비율 비교로만.
- **먼저 성현 자세를 확인하고, 더 빠른 게 필요하면 Y0** 를 추천.

## 2. 시간 예산

`MODE3_PLAN.md` 8절: 명령(공 놓은 뒤 0.16 s) → 공 도착 0.745 s = **예산 ≈ 0.58 s**. amovej 실측(20 cm 0.40 s)이면 기존 자세로도 20 cm 는 예산 안이고, 후보 자세는 모델 비율로 15 cm 0.45~0.48 · 25 cm 0.59~0.67 → 25 cm 는 경계. 직선 가속(amovel 2000 → 3000, −14 %)은 amovej 로 바뀐 지금은 받기 이동과 무관 — IK 실패 때 amovel 로 떨어지는 경우에만 의미.

## 3. 자세를 바꾸면 같이 해야 하는 것 (#195 박성현)

| 항목 | 다시 하나 | 방법 |
|---|---|---|
| **로봇 이동 측정 bag** (공 없이, 영상 없음, 10~15 분) | **필수** | 학습 환경의 로봇 응답표가 출발 관절값에 묶여 있음. 아래 4절 2·3단계를 `IMAGES=0 DURATION=900 OUT=~/bags/motion_<자세> scripts/record_mode3_bag.sh` 로 녹화하면서. CSV 의 `t_cmd_wall`(실제 시각, 이 PR 에서 추가)과 `/rosout` 의 "amovej 명령 …" 줄로 bag 과 맞춘다 |
| 투구 bag (영상 포함) | 학습 시작엔 불필요, 확정엔 **20~30 회 권장** | 패드 위치가 바뀌면 조준 편향(10/12 +19 cm 위)이 달라질 수 있음. 4절 6단계 받기 시험을 겸해서 |
| 측면 카메라 캘리브레이션 | 불필요 | 카메라는 안 움직임. 새 패드가 측면 화면 안에 들어오는지만 |
| 예측 오차표 | 불필요 | 자세와 무관 |
| `skills.yaml mode3` | 필수 | `ready_posj`, `pad_center_base_m`(flight_predictor 가 읽음), `pad_normal`, `right_max_m`·`updown_max_m`·`down_max_m`, 왼쪽 허용(새 키 필요) |

## 4. 안전 (꼭)

- 로봇이 움직인다 ⚠. 비상정지 손 닿는 곳, 반경 1 m 안 사람 없음, 패드 쥔 상태(38 N).
- **Y0 는 팔꿈치가 반대로 접힌다**. 관측 자세 → Y0 의 movej 경로가 테이블·거치대에 닿는지 **처음은 펜던트 수동/저속**으로. 성현 자세는 기존과 같은 쪽이라 이 걱정은 작음.
- 시험 동안만 `ros2 param set`, 끝나면 원복 확인. `safety.yaml` 파일은 고치지 않는다.
  - Y0: `workspace_x_max_m` 0.57 → **0.78** (요격 경로 TCP x 최대 0.734, 투구자는 J1 에서 3 m 밖) — **정정인 동의 뒤**
  - 성현 자세: `workspace_y_min_m` −0.60 → **−0.75** (오른쪽 15 cm 까지) — 또는 오른쪽 12.5 cm 로 자르고 안 바꿈
  - 힘: 이동 시험 스크립트가 받기 창(130 N)을 켜고 움직이므로 `force_limit_n` 은 그대로 25 N. amovej 400 °/s² 는 ≤ 88 N 실측(#185)

## 5. 준비 (터미널 4개, 모두 `cd ~/rally && source scripts/robot.sh env`)

```bash
git checkout main && git pull          # --posj/--angles 가 있는 mode3_movej_test.py·mode3_reach_test.py 가 main 에 있어야 함 (#196)
# 1
scripts/robot.sh bringup
# 2  안전감시·힘·그리퍼·스킬서버 (safety 로그가 여기)
scripts/robot.sh nodes novision
# 3  시험값 (정정인 확인 후) — 끝나면 반드시 원복.  Y0 면 x, 성현 자세면 y
ros2 param set /rally_safety_monitor workspace_x_max_m 0.78      # Y0
ros2 param set /rally_safety_monitor workspace_y_min_m -0.75     # 성현 자세
ros2 param get /rally_safety_monitor workspace_x_max_m; ros2 param get /rally_safety_monitor workspace_y_min_m
# 4  이동 측정 bag (#195 형식, 영상 없음, 15 분 자동 종료)
IMAGES=0 DURATION=900 OUT=~/bags/motion_y0 scripts/record_mode3_bag.sh
```

아래는 Y0 기준. 성현 자세는 `--posj 99.5 -3.8 -93.0 92.3 -70.6 82.9`, `--ws-x-max` 대신 y 한계만 param set.

## 6. 순서 (터미널 3)

**0. 계획만 (로봇 안 움직임)** — 관절 변화·예상 시간·작업 공간 판정이 모델과 같은지
```bash
python3 scripts/mode3_movej_test.py --posj -58.7 7.0 100.2 98.8 63.6 -100.3 --angles $(seq 0 30 330) --dist 15 25 --cases amovej --jvel 120 --jacc 400 --ws-x-max 0.78
```

**1. 후보 자세로 이동 + 저속 확인 (Y0 만)** — `--posj` 가 먼저 movej(20 °/s). 왼쪽·위-왼쪽(135~180°) 25 cm 를 amovel 100 mm/s 로 — 팔뚝이 베이스 기둥에 7 cm 까지 가는 경로. 두산 자기충돌 정지·펜던트 알람 여부, `print_pose.py` 로 TCP (0.56, −0.32, 0.43) 확인.
```bash
python3 scripts/mode3_reach_test.py --run --posj -58.7 7.0 100.2 98.8 63.6 -100.3 --angles 135 150 165 180 --dist 25 --vel 100 --acc 500 --reps 1 --ws-x-max 0.78
```

**2. amovej 15 cm 12방향** — 핵심. 받기와 같은 120/400. 방향별 도착(`t_arrive_s`)·5 cm 안 진입(`t_near5_s`)·힘 최대. 이동 측정 bag 이 돌아가는 동안.
```bash
python3 scripts/mode3_movej_test.py --run --posj -58.7 7.0 100.2 98.8 63.6 -100.3 --angles $(seq 0 30 330) --dist 15 --cases amovej --jvel 120 --jacc 400 --reps 2 --ws-x-max 0.78
```

**3. amovej 25 cm 12방향** — 범위 상한 확인 (왼쪽 180° 가 되는지가 핵심).
```bash
python3 scripts/mode3_movej_test.py --run --posj -58.7 7.0 100.2 98.8 63.6 -100.3 --angles $(seq 0 30 330) --dist 25 --cases amovej --jvel 120 --jacc 400 --reps 2 --ws-x-max 0.78
```

**4. (선택) amovel 비교** — IK 실패 때 폴백 경로. 가속 2000 → 3000 차이(시뮬 −14 %)도 여기서. 힘 80 N 시험값은 정정인 확인 후.
```bash
python3 scripts/mode3_reach_test.py --run --posj -58.7 7.0 100.2 98.8 63.6 -100.3 --angles $(seq 0 30 330) --dist 15 --vel 540 --acc 2000 3000 --reps 1 --ws-x-max 0.78
```

**5. 카메라** — 손목 카메라(Y0 는 광축 정면 3°)에 투구 위치가 보이는지, 측면 RealSense 에 새 패드 위치가 보이는지 `rqt_image_view`.

**6. 받기 시험** (1~5 통과 뒤, 값 적용 후) — `mode3_catch_loop.py --count 20~30` + 영상 포함 bag (#195 "투구 20~30 회"). 결과·힘 최대 기록 (#173 형식).

끝: bag Ctrl+C → `ros2 param set` 원복(x 0.57 / y −0.60) → `ros2 param get` 으로 확인.

## 7. 통과 기준과 적용할 값

| 확인 | 기준 | 안 되면 |
|---|---|---|
| 1 | 자기충돌 정지·safety STOP 없음 | 왼쪽 범위를 15 cm 로 줄이거나 성현 자세로 |
| 2 | 12방향 모두 완주, 도착 ≤ 0.5 s, 힘 ≤ 88 N | 기존 자세 유지 (amovej 만) |
| 3 | 왼쪽 포함 완주 | 범위 0.25 → 0.15 |
| 5 | 두 카메라에 투구·패드 보임 | 요(±10°) 쪽으로 재탐색 |

적용(별도 PR, 정정인·박성현): `skills.yaml mode3` — `ready_posj`, `pad_center_base_m`·`pad_normal`(Y0: (0.595, −0.423, 0.433)·(0.714, −0.066, 0.697)), `right_max_m`·`updown_max_m`(지금 0.35)·`down_max_m` 를 확인된 범위로 + 왼쪽 허용 키, `safety.yaml` 작업 공간은 **모드 3 세션 동안만** param set(코드에 모드별 작업 공간이 없음 — 필요하면 brain 모드 전환에서 set/원복). flight_predictor 평면은 같은 파일을 읽는다(#146). 학습 환경은 새 이동 측정 bag 으로 응답표 갱신(#195).

## 8. 모델이 안 보는 것 (실로봇에서만)

두산 관절 가속 한계·자기충돌(기둥 7 cm)·케이블·툴 힘 추정의 자세 의존 오차·명령 지연(시뮬 0.11~0.33 s, 실로봇 0.1 s)·amovej 중 패드 기울기 변화(movej_test 가 `tilt_deg` 로 계산해 줌).

## 9. 적용본 검증 — 금요일(10/16) 시연을 Y0 로 (10/14 정동휘 결정, PR 적용본)

적용 PR(`feat/rally_robot/mode3-ready-y0`)에 들어간 것: `skills.yaml mode3` Y0 값(`ready_posj`·`ready_via_posj`·`pad_center_base_m`·`pad_normal`·범위 0.25·`left_max_m`·`reach_max_m`·`catch_radius_m` 0.30), `catch_plan.intercept` 왼쪽 허용, `pad_rack.ready_route` 경유 자세, `safety_monitor` 받기 창 동안 x 상한 `workspace_catch_x_max_m`(0.78, `safety.yaml`, **정정인 승인**). 코드 바꾼 데 없이 실로봇에서 아래 순서로 — 모두 `source scripts/robot.sh env`, bringup·nodes 떠 있는 상태, 비상정지 손 닿는 곳.

| # | 무엇 | 명령 | 통과 |
|---|---|---|---|
| 1 | 빌드·값 확인 | `cd ros2_ws && colcon build --packages-select rally_robot rally_bringup && cd ..` → `ros2 param get /rally_safety_monitor workspace_catch_x_max_m` = 0.78 | 값이 나옴 (−1 이면 빌드·재시작 안 됨) |
| 2 | **경유 경로** (가장 먼저, 저속) | 관측 자세에서 `python3 scripts/mode3_go_ready.py --vel 10` 가 Y0 로 가는지. 그다음 패드 집기 단독: `ros2 service call /rally/robot/pad_pick std_srvs/srv/Trigger` → U → 팔 세움(경유, 최고 0.97 m) → Y0 | 작업 공간 STOP 없음, 천장·거치대 안 닿음, 끝 TCP ≈ (0.56, −0.32, 0.43) |
| 3 | **꽂기 경로** | `python3 scripts/pad_rack_loop.py` 1회 (Y0 → 경유 → U → 꽂기) | 작업 공간 STOP 없음, 꽂힘 |
| 4 | 요격 이동 + x 완화 | `python3 scripts/mode3_movej_test.py --run --angles $(seq 0 30 330) --dist 15 25 --cases amovej --jvel 120 --jacc 400 --reps 1` (스크립트가 받기 창을 켜므로 x 0.73 도 STOP 아님) | 12방향 완주, 터미널 2 에 `작업 공간 x 상한 0.78 m` ↔ `0.57 m` 로그가 한 쌍씩, 힘 ≤ 88 N |
| 5 | 받기 창 뒤 복귀 | 4번 로그에서 창이 꺼진 뒤(`힘 한계 25 N`) 복귀 중 STOP 이 없는지 | `workspace_limit` 없음 |
| 6 | 카메라 | `rqt_image_view` 손목·측면 — Y0 에서 투구 위치·패드가 보이는지 | 보임 |
| 7 | 받기 | `python3 scripts/mode3_catch_loop.py --count 10` + `scripts/record_mode3_bag.sh` (영상 포함, #195 투구 20~30회는 시연 뒤) | 왼쪽·아래 투구도 움직임, STOP 없음 |

막히면: 2·3 에서 STOP → `ready_via_posj` 값을 펜던트에서 손으로 찾아 교체(팔 수직이면 됨). 4 에서 힘 > 88 N → `catch_joint_acc_deg_s2` 400 → 300. 어느 단계든 안 되면 `skills.yaml` 을 기존 값으로 되돌리면 전부 10/13 동작 (키 `left_max_m` 0, `ready_via_posj` 지움, 범위 0.35/0.08, `catch_radius_m` 0.35, `safety.yaml workspace_catch_x_max_m` −1).

## 기록

(실로봇 결과를 여기에 — 날짜 · 누가 · 자세 · 단계 · 방향별 도착 s · 힘 최대 N · 알람/STOP · bag 경로)
