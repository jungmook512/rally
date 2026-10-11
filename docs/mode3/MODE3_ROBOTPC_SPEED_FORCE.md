# 모드 3 — 빠른 요격(가속 2000)과 힘 한계 측정 (정정인 노트북)

박성현 → 정정인 PC (로봇 PC Claude 와 사람이 같이 봄). 결과는 `MODE3_RESULTS.md` 「10/12 속도·힘」에 적고 아래 "보낼 것"을 박성현에게.

## 왜 재나
모드 3 을 빠르게(가속 2000 mm/s²) 받으려면 안전 설정을 모드 3 에서만 다르게 해야 할 수 있다 (제안: 힘 한계 50 N, 작업 공간 해제). 그런데 이미 잰 값이 걸린다:
- 10/10: 가속 2000 · 730 mm/s 로 −y 20 cm → **도착 직후 추정 툴 힘이 50 N 도 넘어 STOP** (감속 관성. 툴 힘은 관절 토크 추정이라 패드에 닿지 않아도 커짐)
- 10/11 계산: 기존 대기 자세에서 위아래(±u) 730 mm/s 는 관절 속도 한계의 121 % → 불가, 450 은 75 %
→ **가속·속도별로 실제 관성 힘이 얼마나 나오는지** 재야 힘 한계(50 N 이 맞는지)와 쓸 수 있는 가속을 정할 수 있다.

## 안전 (꼭)
- 로봇이 움직인다 ⚠. 비상정지 손 닿는 곳, 로봇 반경 1 m 안에 사람 없음, 패드 쥔 상태(손잡이 포함 199 g).
- 시험 동안만 힘 한계를 **80 N** 으로 올린다 — **정정인 확인 후**. 끝나면 25 N 원복 확인. 작업 공간 검사는 켠 채로 둔다(이 시험 이동은 모두 안쪽).
- `safety.yaml` 파일은 고치지 않는다 (실행 중 `ros2 param set` 만). 시험 중 STOP 이 나면 그 줄을 기록하고 다음으로.

## 준비 (터미널 4개, 모두 `cd ~/rally && source scripts/robot.sh env`)
```bash
# 1
scripts/robot.sh bringup
# 2  안전감시·힘·그리퍼·스킬서버
scripts/robot.sh nodes novision
# 3  펜던트 또는 이걸로 대기 자세 [89.8, 30.0, -121.7, 90.2, -80.2, 88.3] 확인, 패드 쥐기 (gripper 38 N)
python3 scripts/print_pose.py
# 3  힘 한계 시험값 (정정인 확인 후) — 끝나면 반드시 25.0 으로
ros2 param set /rally_safety_monitor force_limit_n 80.0
ros2 param get /rally_safety_monitor force_limit_n
# 4  힘·관절 기록 (시험 내내 켜 둠)
ros2 bag record -o data/bags/mode3_speedforce_$(date +%m%d_%H%M) /rally/robot/wrench /dsr01/joint_states /rally/robot/tcp_pose /rally/safety/status
```
safety_monitor 노드 이름이 다르면 `ros2 node list | grep safety` 로 확인.

## 측정 (터미널 3, 대기 자세에서 시작 — 매 이동 뒤 스크립트가 천천히 복귀)
먼저 계획 모드(로봇 안 움직임)로 관절 속도 % 와 작업 공간 경고를 본다:
```bash
python3 scripts/mode3_reach_test.py --dist 5 10 15 --dirs -y +u -u --acc 1000 1500 2000 --vel 600
```
그다음 실제 이동 (위에서 경고 없는 것만):

| # | 가속 mm/s² · 속도 mm/s | 방향 · 거리 | 명령 | 이동 수 |
|---|---|---|---|---|
| T1 | 1000 · 450 | −y, +u, −u · 5/10/15 cm | `python3 scripts/mode3_reach_test.py --run --dist 5 10 15 --dirs -y +u -u --acc 1000 --vel 450 --reps 2` | 18 |
| T2 | 1500 · 600 | 같음 | `... --acc 1500 --vel 600 --reps 2` | 18 |
| T3 | 2000 · 600 | 같음 | `... --acc 2000 --vel 600 --reps 2` | 18 |
| T4 | 2000 · 730 | −y 만 · 10/15 cm | `python3 scripts/mode3_reach_test.py --run --dist 10 15 --dirs -y --acc 2000 --vel 730 --reps 2` | 4 |

- 순서대로. 앞 단계에서 80 N STOP 이 나면 그 가속 이상은 하지 않는다.
- **−u 는 5 cm 까지만** — 10/11 T1 에서 −u 약 9.4 cm 에 두산 자기충돌 정지 (`MODE3_RESULTS.md`). T1 을 다시 할 때는 `--dirs -y +u` 와 `--dirs -u --dist 5` 로 나눠서. T1 은 10/11 에 됨 → **남은 것 T2~T4**
- `+y`(왼쪽)는 하지 않는다 (어깨 특이점, 모드 3 은 왼쪽으로 안 감).
- 끝나면: bag 기록 Ctrl+C → `ros2 param set /rally_safety_monitor force_limit_n 25.0` → `ros2 param get` 으로 25.0 확인.

## T5 servol 추종 (강화학습 2단계용, 약 20분) ⚠
`scripts/mode3_servol_test.py` (박성현, USB 로 전달 — `~/rally/scripts/` 에 넣기). 대기 자세에서 작은 폭(3 cm)으로:
- **step**: 목표를 3 cm 한 번에 옮기고 1 s 동안 같은 목표를 계속 보냄 → 출발·도착·지나침 (amovel 과 비교)
- **sine**: 목표를 0 ~ 3 cm 로 0.5·1·2 Hz 오감, 30·60 Hz 로 보냄 → 지연(ms)·진폭비·추종 오차
- servol 은 두산 속도 슬라이더·Safety Zone 감속을 안 따르므로, 스크립트가 매 명령 전 안전 상태를 보고 NORMAL 이 아니면 바로 정지·종료한다.
```bash
python3 scripts/mode3_servol_test.py                       # 계획만 — 판정이 전부 OK 인지 확인
python3 scripts/mode3_servol_test.py --run --reps 2        # 실제 (step 3방향 + sine 3방향 × 3 Hz, 각 30·60 Hz → 48번)
```
- 힘 한계는 T1~T4 와 같은 시험값(80 N) 그대로 두고 bag 기록도 켠 채로. 처음 몇 번은 비상정지에 손 대고 보기.
- 알람(servol 반환값 오류)이나 덜컹거림이 나면 그 줄 기록하고 멈춘 뒤 박성현에게.
- 결과: `data/measure/mode3_servol_MMDD_HHMM.csv`

## (선택, 10분) 움직이는 중 amovel 다시 보내기
나중 단계(예측이 바뀌면 목표를 고치기)에 필요. `scripts/mode3_amovel_resend_test.py` — 대기 자세에서 −y 10 cm 로 amovel(1000 · 450) → 0.15 s 뒤 −y 15 cm 로 amovel 한 번 더.
`single`(15 cm 한 번, 비교 기준) · `duplicate`(두산 기본 `ra`) · `override`(`DR_MV_RA_OVERRIDE`) 세 경우 × 3회. 반환값(알람·거부), 사이에 멈췄는지, 첫 목표에 머문 시간, 15 cm 도착 시각을 CSV 로.
```bash
python3 scripts/mode3_amovel_resend_test.py                  # 계획만
python3 scripts/mode3_amovel_resend_test.py --run --reps 3   # 9번. 다른 간격도 보려면 --delay 0.1 0.15 0.25
```
결과: `data/measure/mode3_resend_MMDD_HHMM.csv`

## 보낼 것
- `data/measure/mode3_reach_*.csv` (T1~T4), `data/measure/mode3_servol_*.csv` (T5)
- `data/bags/mode3_speedforce_*` 폴더 (힘·관절, 작음). 이동마다 힘 최대는 CSV `force_peak_n`·`force_dpeak_n` 에도 있음
- STOP 이 난 이동 목록, 힘 한계를 25.0 으로 되돌렸는지
- (선택) `data/measure/mode3_resend_*.csv`

## 이걸로 정할 것 (박성현이 계산해서 팀에)
1. 가속·속도별 이동 중 최대 추정 힘 → **모드 3 힘 한계** 값(최대 관성 + 공 충격 15 N + 여유)과 쓸 가속
2. 방향·거리별 도착 시간 → 성공률 상한 다시 계산, `skills.yaml mode3.catch_vel/acc`
3. servol 지연·추종 → Isaac 시뮬 로봇 반응 모델(강화학습 2단계)
4. 모드 3 전용 안전 설정을 safety_monitor 에 어떻게 넣을지 (모드별 힘 한계·작업 공간) — 코드 변경이라 정정인 리뷰
