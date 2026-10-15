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
| **로봇 이동 측정 bag** (공 없이, 영상 없음, 10~15 분) | **필수** | 학습 환경의 로봇 응답표가 출발 관절값에 묶여 있음. 아래 6절 2·3단계(**약 84회, 15 분 — 박성현 #196 코멘트 구성**)를 `IMAGES=0 DURATION=900 OUT=~/bags/motion_<자세> scripts/record_mode3_bag.sh` 로 녹화하면서. 실제 요격 대부분이 15 cm 안이라 5·10 cm 짧은 거리 점이 꼭 필요(학습표가 거리별 보간). CSV 의 `t_cmd_wall`(실제 시각, 이 PR 에서 추가)과 `/rosout` 의 "amovej 명령 …" 줄로 bag 과 맞춘다 |
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

**2. 이동 측정 — amovej 72회 (40 + 32)** (박성현 #196 코멘트 구성). 핵심. 받기와 같은 120/400. 방향별 도착(`t_arrive_s`)·5 cm 안 진입(`t_near5_s`)·힘 최대. **5절 4번 이동 측정 bag 을 켠 채로.** 왼쪽 180° 25 cm 가 되는지가 범위 확인의 핵심. (`--posj` 는 처음 한 번만 — 이미 Y0 면 빼도 같음)
```bash
# a) 4축(0·90·180·270°) × 5·10·15·20·25 cm × 2회 = 40
python3 scripts/mode3_movej_test.py --run --posj -58.7 7.0 100.2 98.8 63.6 -100.3 --angles 0 90 180 270 --dist 5 10 15 20 25 --cases amovej --jvel 120 --jacc 400 --reps 2 --ws-x-max 0.78
# b) 8방향(45° 간격) × 15·25 cm × 2회 = 32
python3 scripts/mode3_movej_test.py --run --angles $(seq 0 45 315) --dist 15 25 --cases amovej --jvel 120 --jacc 400 --reps 2 --ws-x-max 0.78
```
스크립트는 방향 순서대로 돈다 (섞는 옵션 없음 — 박성현 제안의 "순서 섞기"는 아직 미반영).

**3. 이동 중 재전송 — amovej 12회** (박성현 구성: 3쌍 × 4회). 받기 스킬의 비행 중 갱신(0.3 s 간격)과 같은 조건. Y0 에서 시작(2단계 끝 자세 그대로).
```bash
python3 scripts/mode3_amovel_resend_test.py --run --cases jdup --dir -y --first 10 --second 15 --delay 0.3 --jvel 120 --jacc 400 --reps 4
python3 scripts/mode3_amovel_resend_test.py --run --cases jdup --dir -y --first 15 --second 25 --delay 0.3 --jvel 120 --jacc 400 --reps 4
python3 scripts/mode3_amovel_resend_test.py --run --cases jdup --dir +u --first 10 --second 20 --delay 0.3 --jvel 120 --jacc 400 --reps 4
```
재전송 스크립트의 `-y`·`+u` 는 45° 고정 면 기준 방향(Y0 실제 법선과 4° 차이). 계획 모드(`--run` 빼고)로 먼저 판정이 OK 인지.

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

## 9. 적용본 검증 — 어깨 반대 자세 (10/15, 로봇 시간이 적을 때 이 순서만)

**10/14 저녁 실로봇에서 Y0 로 가다 J1 −97° 에서 J2 모터 몸통이 패드 거치대에 충돌(#208).** 이 브랜치(`feat/rally_robot/mode3-ready-flip`)는 같은 TCP·패드 자리의 **어깨 반대 IK 해** `[123.1, -7.0, -100.2, -80.5, 61.9, -100.6]` 로 바꾼 것. 두산 충돌 메시 모델(`scripts/m0609_collision.py`, 실로봇 충돌을 J1 −90~−120° 겹침으로 재현함)로 계산한 결과 — 실로봇 미확인:

| 경로 | 거치대 여유 (계산) | 비고 |
|---|---|---|
| 관측 자세 → 대기 (직접 관절 이동) | 11.0 cm | TCP x 최대 0.560, **경유 자세 불필요** |
| 받기 이동 25 cm 24방향 (왼쪽 23) | **6.1 cm** | amovej 15 cm 0.47 s · 25 cm 0.67 s, 관절 변화 ≤ 44°, \|sin J5\| ≥ 0.52 |
| 대기 → 거치대 A (패드 집기) | **−5.9 cm** (기존 자세는 −2.6) | 손목을 먼저 A 로 돌릴 때 그리퍼가 거치대 상자를 쓸고 들어감(#208 "위에서 내려옴"). **이번 시험에서는 집기·꽂기를 안 한다** |
| U ↔ 대기 (꽂기·집기 끝) | 끝점 U 에서 −2.3 (기존과 같음) | U 자체가 거치대 바로 위라 상자 모델의 한계. 중간 쓸림 없음 |

**대기 자세에서 TCP x 0.560 — safety x 상한 0.57 까지 9.6 mm.** 컨트롤러 fkin 과 URDF 계산이 x 로 2.7 mm 차이(print_pose.py) → 멈춰 있을 때 `workspace_limit` 이 날 수 있으므로 시험 동안 x 상한을 0.60 으로.

### 준비 (터미널 4개, 모두 `cd ~/rally && source scripts/robot.sh env`)
```bash
git fetch && git checkout feat/rally_robot/mode3-ready-flip && git pull
cd ros2_ws && colcon build --packages-select rally_robot rally_bringup && cd ..
# 1
scripts/robot.sh bringup
# 2  안전감시·힘·그리퍼·스킬서버 (safety 로그가 여기)
scripts/robot.sh nodes novision
# 3  시험값 — 끝나면 원복
ros2 param get /rally_safety_monitor workspace_catch_x_max_m     # 0.78 (−1 이면 빌드·재시작 안 됨)
ros2 param set /rally_safety_monitor workspace_x_max_m 0.60       # 대기 자세 x 0.560 의 9.6 mm 여유 때문 (시험 동안만)
# 4  이동 측정 bag (#195, 영상 없음)
IMAGES=0 DURATION=900 OUT=~/bags/motion_flip scripts/record_mode3_bag.sh
```
**패드는 미리 손으로 그리퍼에 쥐여 둔다** (그리퍼 닫기 `ros2 service call /rally/gripper/command rally_interfaces/srv/GripperCommand "{width_mm: 0.0, force_n: 38.0, wait_until_done: true}"`) — 집기·꽂기 경로(표 3행)는 이번에 안 돌린다.

### 순서 (터미널 3, 비상정지 손에)
| # | 무엇 | 명령 | 통과 |
|---|---|---|---|
| 0 | 계획만 | `python3 scripts/mode3_movej_test.py --posj 123.1 -7.0 -100.2 -80.5 61.9 -100.6 --angles $(seq 0 30 330) --dist 15 23 --cases amovej --jvel 120 --jacc 400 --ws-x-max 0.78` | 24개 모두 OK (관절 변화 ≤ 44°) |
| 1 | **대기 자세로 (저속)** | `python3 scripts/mode3_go_ready.py --vel 10` — 관측 자세에서 직접. J1 이 0 → 123° 로 크게 도는 게 정상 | 거치대·천장 안 닿음, STOP 없음. `print_pose.py` TCP ≈ (0.56, −0.32, 0.43). **눈으로: J2 모터가 거치대 반대쪽(로봇 왼쪽 뒤)** |
| 2 | 받기 이동 15 cm 12방향 (bag 켠 채) | `python3 scripts/mode3_movej_test.py --run --angles $(seq 0 30 330) --dist 15 --cases amovej --jvel 120 --jacc 400 --reps 2 --ws-x-max 0.78` | 완주, 힘 ≤ 88 N, 터미널 2 에 `작업 공간 x 상한 0.78` ↔ `0.60` 로그 쌍 |
| 3 | 받기 이동 23 cm 12방향 | 같은 명령 `--dist 23` | 완주 (왼쪽 180° 포함), 거치대 가까운 방향 240~270° 눈으로 |
| 4 | (시간 있으면) 5·10·20·25 cm 4축 + 재전송 — #196 가이드 6절 2·3단계 | | 이동 측정 bag 완성 |
| 5 | 받기 10회 | `python3 scripts/mode3_catch_loop.py --count 10` — **`skills.yaml mode3.pad_place_after_catch: false` 로 바꾼 뒤** (꽂기 경로 미확인) | 왼쪽·아래 투구도 움직임, STOP 없음 |

끝: bag Ctrl+C → `ros2 param set /rally_safety_monitor workspace_x_max_m 0.57` → `git checkout main`.

### 안 되면
- 1 에서 거치대 쪽으로 가까워 보이면 즉시 정지 → 관절값·`print_pose.py` 를 #208 에.
- 2·3 에서 `workspace_limit` STOP: 터미널 2 로그의 TCP 와 그때 x 상한(0.78 이었는지). 받기 창이 안 켜졌으면 `movej_test` 가 내는 `catch_window` 가 safety 에 안 닿은 것.
- 되돌리기: `git checkout main` (10/13 자세). 이 브랜치는 main 에 코드 변경(왼쪽 허용·x 완화)을 다시 넣은 것이라 main 으로 가면 전부 꺼진다.

## 기록

(실로봇 결과를 여기에 — 날짜 · 누가 · 자세 · 단계 · 방향별 도착 s · 힘 최대 N · 알람/STOP · bag 경로)
