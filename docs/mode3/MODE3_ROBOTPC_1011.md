# 모드 3 — 로봇 PC 할 일 (10/11 박성현 → 정정인 노트북)

로봇 PC 의 Claude 와 사람이 같이 보는 실행 목록. 앞 문서 `MODE3_BRIEF.md`·`MODE3_TODO.md`·`MODE3_RESULTS.md`(PR #107) 다음 단계다.
결과는 `MODE3_RESULTS.md` 에 적고, 각 작업 끝의 "보낼 것"을 박성현에게 보낸다.

## 0. 먼저 읽기 — 10/11 바뀐 결정

| 항목 | 결정 |
|---|---|
| YOLO | 비교 실험 중(15시쯤 끝). 시험용 `velcro_ball_yolo11s-seg_1024_test.pt` (최종 후보 중 느린 쪽 → 지연 상한). 실시간은 **TensorRT** |
| 패드 실측 | 벨크로 지름 15 cm(공이 닿으면 성공, 반지름 7.5 cm), 전체 18 cm, 손잡이 포함 199 g, **TCP → 패드 중심 10.5 cm**(이전 가정 17.5), 패드 면이 그리퍼보다 1.87 cm 나옴 |
| 이동 규칙 | TCP 자세 고정, **왼쪽(로봇 기준 +y)으로는 잡으러 가지 않음**. 오른쪽(−y)·위·아래 약 15 cm. 현재 `safety.yaml` 작업 공간 그대로 |
| 성공률 | 패드 왼쪽 끝 밖으로 온 공 = 투구 실패 → 로봇 성공률 분모에서 뺌. 판정은 측면 카메라 실제 궤적으로 사후. 보고는 유효 투구 성공률 · 전체 성공률 · 왼쪽 투구 비율 세 개 |
| 조준 | 패드 중심보다 **로봇 기준 오른쪽(−y) 5 cm** (투구자가 보면 왼쪽). 목표 표시를 거기에 |
| 투구 | 지금 투구는 로봇 앞에서 너무 높다(x 0.4 m 에서 base z 약 1.0 m). **짧고 낮게** — 새 대기 자세 패드 높이(아래 2절)로 오게 |
| 새 대기 자세 | 아래 2절. 이전 채택 자세 [90, 30, −122, 90, −80, 88] 는 왼쪽 10~15 cm 에서 어깨 특이점 |
| 안전 | 모드 3 전용 작업 공간·힘 한계는 **이슈로 정정인·팀 합의 후** 따로 (지금 바꾸지 않음) |

## 1. 파일 배치 (박성현 PC 에서 USB 로 복사)

| 파일 | 정인 PC 위치 |
|---|---|
| `robotpc_yolo_bench.sh` | `~/rally/scripts/catch_lab/robotpc_yolo_bench.sh` |
| `velcro_ball_yolo11s-seg_1024_test.pt` (20.6 MB) | `~/rally/models/velcro_ball_yolo11s-seg_1024_test.pt` (`models/` 는 git 제외) |
| 이 문서 | `~/rally/docs/mode3/MODE3_ROBOTPC_1011.md` |

```bash
mkdir -p ~/rally/models ~/rally/scripts/catch_lab ~/rally/docs/mode3
# USB 경로 예: /media/$USER/<USB이름>/
cp /media/$USER/*/robotpc_yolo_bench.sh ~/rally/scripts/catch_lab/
cp /media/$USER/*/velcro_ball_yolo11s-seg_1024_test.pt ~/rally/models/
cp /media/$USER/*/MODE3_ROBOTPC_1011.md ~/rally/docs/mode3/
ls -la ~/rally/models/velcro_ball_yolo11s-seg_1024_test.pt     # 20553060 바이트면 정상
```

## 안전 (모든 작업 공통)
- 로봇이 움직이는 작업(⚠)은 **사람이 확인한 뒤** 실행. 비상정지 버튼 손 닿는 곳, 로봇 반경 안에 사람 없음.
- `safety.yaml` 은 바꾸지 않는다. 10/10 처럼 시험용으로 힘 한계를 올려야 하면 정정인 확인 → 시험 끝나면 원래 값으로 → RESULTS 에 기록.
- 새 자세로 처음 갈 때는 펜던트 저속(10 % 이하), 테이블·케이블·카메라 간섭을 눈으로 보면서.

---

## 작업 1 — YOLO 추론 지연 (로봇 안 움직임, 20~30분)

```bash
cd ~/rally
bash scripts/catch_lab/robotpc_yolo_bench.sh                  # .pt 1장·2장, TensorRT 변환(수 분)·속도
# ROS realsense 노드를 끈 뒤 (카메라를 pyrealsense2 가 직접 연다)
LIVE=1 bash scripts/catch_lab/robotpc_yolo_bench.sh           # + 카메라 2대 실영상 15 s. 공을 두 카메라 앞에서 흔들기
```
- venv 생성 실패 → `sudo apt install python3.12-venv` 후 다시.
- 판정: TensorRT 2장 묶음 p95 < **16.7 ms**(60 fps 예산). 실영상 촬영 → 결과 지연(이전 측정: 촬영 → 수신만 20 ms).
- **보낼 것**: 끝에 나오는 「요약」 몇 줄 + `~/rally/data/mode3_bench_*/det_*.jpg` 2~3장.

## 작업 2 — 새 대기 자세 확인 ⚠ (R, 30분)

박성현 PC 순기구학(두산 URDF)으로 찾은 후보. 조건: 왼쪽 이동 없음, 오른쪽·위·아래 15 cm 경로 전체가 작업 공간 안, 손목 중심이 J1 축에서 ≥ 0.15 m, |sin J5| ≥ 0.5, 관절 원점이 테이블(z 0)에서 ≥ 0.13 m.

| | 1순위 (패드 45°) | 2순위 (패드 20°) |
|---|---|---|
| 관절 [deg] | **[116.7, −14.1, −128.1, 137.3, −111.4, 35.9]** | [119.1, −10.5, −125.1, 124.5, −85.7, 37.8] |
| TCP (base, m) | (0.387, −0.295, 0.487) | (0.482, −0.295, 0.394) |
| 패드 중심 (base, m) | (0.40, −0.40, 0.50) → 바닥에서 약 1.46 m | (0.50, −0.40, 0.40) → 약 1.36 m |
| 1 m/s 에 필요한 관절 속도 (최악) | 107 % (≈ 0.93 m/s 까지 한계 안) | 111 % |
| 손목 중심 ↔ J1 축 | 0.296 m | 0.312 m |
| 관절 최저 높이 | 0.136 m | 0.135 m |

가정: 공구 z = 패드 법선(투구자 쪽, 위로 45°/20°), 패드 중심 = TCP − 0.105 m × 공구 y + 0.0187 m × 공구 z. **실제 손잡이 방향이 다르면 박성현에게 알리기.**

순서
1. 지금 자세에서 1순위 관절값으로 **펜던트 movej 저속(10 %)** ⚠ — J2·J4·J5 가 크게 바뀌니 중간 자세에서 팔꿈치·손목이 테이블·케이블에 닿지 않는지 계속 보기. 위험하면 바로 멈추고 중간 경유점을 둔다.
2. 도착 후 `python3 scripts/print_pose.py` 로 TCP 확인 → 위 표와 1 cm 안인지.
3. 패드 중심 위치를 줄자로: J1 축 기준 x·y, 바닥 높이.
4. 계획 모드(로봇 안 움직임):
   ```bash
   cd ~/rally && source scripts/robot.sh env
   python3 scripts/mode3_reach_test.py --dist 5 10 15 --dirs -y +u -u --acc 1000 2000
   ```
   필요 관절 속도·작업 공간 경고 출력을 저장.
- **보낼 것**: print_pose 출력, 줄자 값, 계획 모드 출력, 자세 사진 2장(정면·옆).

## 작업 3 — 새 자세 도달 시험 ⚠ (R, 40분)

작업 2 계획 모드에 경고가 없을 때만. 대기 자세 = 실행 시점의 로봇 자세.
```bash
python3 scripts/mode3_reach_test.py --run --dist 5 10 15 --dirs -y +u -u --acc 1000 --vel 450 --reps 3
python3 scripts/mode3_reach_test.py --run --dist 5 10 15 --dirs -y +u -u --acc 2000 --vel 730 --reps 3
```
- `+y`(왼쪽)는 하지 않는다. `+u` = 패드 면 안 위·뒤, `−u` = 아래·앞.
- acc 2000 은 10/10 에 힘 STOP 이 났던 조건 — STOP 나면 그 줄 기록하고 다음으로(한계 올리지 말 것).
- **보낼 것**: `data/measure/mode3_reach_MMDD_HHMM.csv` 2개.

## 작업 4 — 새 조준점 투구 녹화 + 붙음/튕김 (C, 로봇은 대기 자세 정지, 40분)

1. 로봇이 패드 쥐고 새 대기 자세에서 정지(안전감시·힘 노드 실행: `scripts/robot.sh nodes novision`).
2. 목표 표시: 패드 중심에서 **로봇 기준 오른쪽(−y) 5 cm**. 투구자는 J1 에서 3 m, **짧고 낮게** 패드 높이로.
3. 카메라 2대 실행·노출 고정(BRIEF 7절 명령), 10회씩 bag(BRIEF 7절 `ros2 bag record`), 3개 = 30회. 이름은 `throw_1011_HHMM` 그대로.
4. 던질 때마다 `data/measure/throws_1011.csv` 에 한 줄: `bag,번호,결과(붙음/튕김/빗나감),빗나간 쪽(왼/오/위/아래),STOP(없음/nudge/force),메모`.
- 공이 패드에 맞는 순간 **nudge STOP**(힘 변화 8 N) 이 나는지 꼭 기록 — 나면 모드 3 스킬이 받는 동안 `/rally/safety/contact_expected` 를 켜야 한다(코드 작업은 박성현·정정인).
- **보낼 것**: bag 이름 목록, `throws_1011.csv`. bag 은 외장 디스크로(이전처럼).

## 작업 5 — 공기 저항 확인용 녹화 (C, 로봇 불필요, 15분)

측정한 k = 0.0775 /m 가 공 크기로 계산한 값의 약 5배라 따로 확인.
- 측면 카메라만 켜고 bag 녹화(측면 color·camera_info·depth).
- ① 낙하 10회: 측면 카메라 화면 가운데, 카메라에서 약 2.5 m(투구 궤적 위치) 거리에서 높이 약 2 m 손을 놓아 떨어뜨림(던지지 말고 놓기).
- ② 수평 투구 10회: 카메라 시야를 가로지르게 빠르게(5 m/s 이상).
- 놓는 지점 바닥에 테이프 표시, 카메라까지 거리 줄자 기록.
- **보낼 것**: bag 이름, 카메라까지 거리.

## 하지 말 것 / 나중
- servol 응답 시험: 스크립트 준비 중(박성현). 아직 하지 않는다.
- `safety.yaml` 수정, 왼쪽(+y) 이동 시험.
- `scripts/catch_lab/config.yaml` `ball.diameter_m` 0.065 → 0.0634 (TODO 남은 것, 시간 나면).

## 결과 보내는 곳
박성현에게: 작업 1 요약 · 작업 2 사진·값 · 작업 3 CSV · 작업 4 CSV·bag 목록 · 작업 5 bag·거리. `MODE3_RESULTS.md` 에도 같은 내용.
