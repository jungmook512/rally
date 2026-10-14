# 모드 3 정책 모드 — 10/14 학습 정책 적용 절차 (A/B)

담당 박성현. 로봇 조작은 정정인과 같이. 이 문서는 `skills.yaml mode3.decide_mode: policy` 로 바꿔 Isaac 1단계 PPO 정책을 실로봇에서 시험하는 순서와, 그 전에 확인한 것·위험을 적는다. 기본값은 A/B 통과 전까지 `rule` 그대로다.

## 1. 정책이 하는 일 (바뀌지 않는 것)

- 정책이 정하는 것은 **언제 출발할지**와 **예측점에 더할 보정(u·w, ±10 cm)** 뿐이다. 공 놓은 뒤 0.10~0.40 s 동안 50 Hz 로 최신 예측을 넣어 `action[0] > 0` 이 나오는 순간 한 번 출발한다 (`catch_policy.decide_with_policy`).
- 이동 한계 클램핑(`catch_plan.intercept`: 오른쪽 35·위 35·아래 8 cm·두 축 합 35·지나침), 품질 게이트(std ≤ 0.13 m·관측 ≥ 4), 받기 창(130 N), SLOW 중단, 비전 판정, safety_monitor 는 규칙 모드와 똑같다 (#78: 학습은 결정만, 안전 범위 제한은 그대로).
- 0.40 s 까지 출발이 안 나오거나 정책 실행 오류(파일·onnxruntime·NaN)면 그 투구는 규칙 결정(`decide_after_s`·품질 기다림)으로 넘어간다. 정책으로 출발한 투구는 재전송하지 않는다 (#175 리뷰).

## 2. 10/14 정책 파일

| 항목 | 값 |
|---|---|
| 파일 | `models/policy_stage1.onnx` (74 KB) + 옆 `models/policy_stage1.onnx.json` — 둘 다 git 제외, 박성현 구글 드라이브 |
| md5 | `cae0ac79ec7b36cc7831b28572ef8a7e` |
| 학습 | `~/rally_isaac/rl` stage1 run `stage1_2026-10-14_18-27-37_watch1`, model_300.pt (PPO 300 반복, 4096판) |
| 학습 조건 | 로봇 amovej(jvel 120·jacc 400) 10/14 실측 2D 시간표 · 출발 지연 U(85, 120 ms) · 한계 오른쪽 0~35, 위 35, 아래 8 cm · 두 축 합 35 · 지나침 0 · 투구 10/12 겨냥 27회 · 예측 오차 표 10/12 오프라인 EKF |
| 시뮬 결과 | 같은 투구 1만 회: 규칙 0.16 s 68.8 % → 정책 71.4 % (+2.6 %p, 정책만 받음 198 vs 규칙만 32, p 2×10⁻³⁰). 출발 시각 0.21 ± 0.06 s |
| 옛 정책 | 10/13 파일은 amovel·한계 15/15/8 학습. 섞이지 않게 md5 로 구분 |

`policy_stage1.onnx.json` 내용 (드라이브에 json 이 없으면 이 내용으로 만들어 ONNX 옆에 둔다):

```json
{"run": "stage1_2026-10-14_18-27-37_watch1", "checkpoint": "model_300.pt", "exported": "2026-10-14T20:16:20",
 "md5": "cae0ac79ec7b36cc7831b28572ef8a7e", "bytes": 73990,
 "env": {"accel": "movej", "limits": [0.0, 0.35, -0.35, 0.08], "reach_max": 0.35, "overshoot": 0.0,
         "center_bonus": 0.5, "force_pen": 1.0, "delay": [0.085, 0.12], "profile": "measured"},
 "obs": "stage1_env_torch._obs_tensor 순서 10칸 (catch_policy.policy_obs 와 같음)", "action": "[go>0, corr_u×0.05, corr_w×0.05] (±0.10 m)"}
```

catch_ball 은 정책을 읽을 때 이 json 의 학습 조건과 skills.yaml(right_max_m·updown_max_m·down_max_m·reach_max_m·catch_move·catch_overshoot_m)을 대조해 다르면 경고 로그를 남긴다 (동작은 yaml 대로).

## 3. 박성현 PC 에서 확인한 것 (10/14)

- 로봇 코드 `catch_policy.policy_obs` 가 만드는 입력 10칸과 시뮬 `stage1_env_torch._obs_tensor` 가 같은 상황에서 최대 차이 0.0004 (float32 오차).
- 로봇 코드의 `SIM_STD_T/M` 표 = 학습 환경의 std(t) 표 (동일).
- 로봇식 입력으로 새 정책을 돌리면 출발 0.12~0.24 s (오른쪽 30 cm·비행 0.55 s → 0.12 s, 오른쪽 5·위 10 cm·비행 0.80 s → 0.24 s), 보정 ±2.5 cm 안.
- ONNX 구조 검사(onnx.checker) 통과: 입력 `obs[1,10]`, 출력 `action[1,3]`. onnxruntime 수치 검증은 박성현 PC 에 onnxruntime 이 없어 로봇 PC 에서 (아래 4-②).
- 단위 테스트 `test_catch_policy.py` 10개 통과 (로봇·onnxruntime 없이).

## 4. 로봇 PC 절차

1. **파일**: 드라이브의 `policy_stage1.onnx`·`policy_stage1.onnx.json` → `~/rally/models/`. `md5sum ~/rally/models/policy_stage1.onnx` 가 `cae0ac79…` 인지.
2. **실행기**: rally_robot 을 돌리는 파이썬에서 `pip install onnxruntime`. 확인:
   ```bash
   cd ~/rally/ros2_ws/src/rally_robot && python3 -m pytest test/test_catch_policy.py -q      # real_policy 가 skip 이 아니라 pass
   ```
3. **설정**: `ros2_ws/src/rally_bringup/config/skills.yaml` mode3 `decide_mode: rule` → `policy` (시험 세션 동안만. 한계·catch_move·지나침·`policy_std_source: sim_table` 은 그대로). 빌드 뒤 재시작.
4. **공 없이 1회**: catch_ball 로그에 `정책 모드 — …/policy_stage1.onnx` · `정책 학습 조건 — run stage1_2026-10-14_18-27-37_watch1 …` 이 찍히고, **`정책 학습 조건과 skills.yaml 이 다름` 경고가 없어야** 한다. 투구 감지 뒤 `정책 출발 — 놓은 뒤 0.2x s (n회 호출), 보정 u … w …` 가 찍히는지. `규칙 결정으로` 만 찍히면 파일·onnxruntime 문제 (에러 로그에 이유).
5. **A/B**: 같은 투구자·3 m·같은 대기 자세. 규칙 10회 → 정책 10회 블록을 3번 (각 30회). 매 블록 `decide_mode` 만 바꿈. bag(landing, landing_info, catch_ball 로그, ball_on_pad, joint_states, 힘)을 보존.
   - 기록: 투구 번호 · 모드 · 결과(caught/missed/out_of_radius/too_late/low_confidence) · 출발 시각(놓은 뒤 s) · 보정 · 반응 시간 · 예측점과 실제 교차점.
   - 30회씩으로는 15 %p 넘는 차이만 구분된다. "정책이 규칙보다 나빠지지 않는가" 를 보는 시험이다.
6. **재생 검증**: bag 의 landing_info 를 시뮬 정책에 넣어 실제 출발 시각과 비교 — 일치율 70 % 미만이면 시뮬 수치를 실로봇 근거로 쓰지 않는다 (10/14 전략 5단계 관문).
7. **되돌리기**: `decide_mode: rule`. 정책 파일은 그대로 두어도 된다 (rule 이면 읽지 않음).

## 5. 위험·주의

- **일찍 출발**: 정책은 학습 표(오프라인 EKF 오차)를 믿고 0.12~0.24 s 에 출발한다. 10/13 실측에서 0.25 s 시점 std 가 8~10 cm 였으므로, 실제 예측이 그때 아직 흔들리면 규칙(0.25 s 결정·교차 0.45 s 전)보다 못할 수 있다. 판단은 A/B 결과로만. 여유가 있으면 `policy_std_source: landing_info` 블록도 하나 (10/13 옛 정책은 이 설정에서 0.36~0.38 s 로 늦게 출발했음).
- **품질 게이트**: 정책이 출발을 내도 그 시점 std > 0.13 m 또는 관측 < 4 면 `low_confidence` 로 움직이지 않는다 (시뮬엔 없는 규칙). 로그에서 이 사유가 몇 번인지 센다.
- **지나침 5 cm**: 학습에 없던 값. 규칙·정책에 같은 값을 두고 비교한다 (시뮬에선 정책이 보정으로 5 cm 되돌려 70.4~71.0 %).
- **출발 지연**: 시뮬에서 지연이 150 ms 를 넘으면 정책 이득이 사라진다 (85~120 ms: +2.6 %p, 150~200 ms: −0.1). bag 의 명령 시각 → joint_states 움직임 시작으로 실제 지연을 다시 확인한다.
- **안전**: safety.yaml·힘 한계·STOP 흐름은 건드리지 않는다. 정책 모드는 결정 시각과 ±10 cm 보정만 바꾸고, 한계 밖은 intercept 가 자른다.

## 6. 근거 자료 (박성현 PC, git 제외)

`~/rally_isaac/notes/data/mode3_rl_report_2026-10-14.html` (결과 보고서), `04_성능지표.md` §13, `mode3_QA.md` Q116~Q119. 학습 코드 `~/rally_isaac/rl/` (`train.sh`, `export_policy.py --copy_to`, `view.sh`).
