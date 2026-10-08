# rally_brain

담당: **조정묵**

상황 요약, 규칙 레이어, LLM advisor, py_trees 행동 트리

인터페이스는 `docs/INTERFACES.md`, 할 일은 `todo/` 참고.

## 모드 1 — 손 기준 목표와 레시피 (#81 · #84 · #85 · #87)

`손 보기 자세 → 신선한 손(base_link) → 손에 가까운 레인 → 손 기준 목표·속도 → PushBall → 받기 후보 관측 → 손 이탈 대기 → ReturnBall → TrialResult`.
중앙선은 `table.yaml`의 `table.centerline_x_m`만 읽는다. 환자 쪽은 +x이다. 목표 하한은 중앙선 + `max(patient_margin_m, safety.stop_distance_m + 0.05)`로 계산한다.
현재 설정은 중앙선 0.568 + 여유 0.20 = x 0.768 m. 이 여유는 계획값이며 실제 손끝 거리·공 정지 오차를 보장하지 않는다.

- `skills.yaml mode1.lanes[].recipe`: PR #57 `cd92d1a`에서 보고한 10/8 실측 a·d0·RMSE와 레인 입구·방향을 사용.
  a는 타격 속도를 기준으로 맞춘 계수로, 물리적 마찰 감속도와 동일하지 않다.
- 개발용 `reach_requested = (level + 1) × reach_rmse_multiplier × RMSE`, level은 0 이상.
  사용자 합의값 `reach_rmse_multiplier=2`에 따라 초기 뻗기·단계 간격은 레인별 4.2/4.8/5.8 cm.
  임상적으로 검증된 난이도 값이 아니며 실물 확인 후 설정을 조정한다.
- 손을 레인 축에 투영한 거리에서 reach를 빼고, 실측 거리 `[0.15, 0.45] m`, 실측 속도 `[0.2, 0.4] m/s`,
  운용 속도 제한, 중앙선 여유, 손보다 앞의 구간을 모두 만족하도록 목표를 제한한다.
  교집합이 없으면 `unreachable_target`으로 기록하고 공을 치지 않는다.
- 로봇 허용 범위는 최신 main의 `[0.10, 0.60]`을 유지하되, Brain의 `brain_speed_range_mps`로 **0.35 m/s 상한**을 추가 적용한다.
  속도만 잘라서 목표와 불일치시키지 않고 가능한 목표 구간을 먼저 줄인다.
  따라서 손이 너무 멀면 실제 뻗기 거리가 요청값보다 커질 수 있다. `reach_requested_m`과 `reach_actual_m`을 따로 기록한다.
- `PushBall.target`과 계산한 **양수 `push_speed`를 함께 전송**한다. 0을 보내 로봇에 계산을 맡기지 않는다.
  로봇의 최종 범위 제한·안전 감시는 그대로다. 로봇 PC와 서버 PC의 레인 좌표·레시피 설정을 맞춰야 한다.
- 손 미검출/NaN/다른 좌표계/1 s 이상 오래된 stamp/미래·zero stamp는 10 s씩 2회 대기 후 `no_hand` 기록, goal 0개.
  goal 전송 직전에도 안전·손·목표 계산에 사용한 stamp를 재검사한다. 두 PC 시계 동기화가 필요하다.
- 레시피 누락·잘못된 계수는 고정 속도로 대신 치지 않는다. `no_recipe`/`invalid_recipe` 등 사유를 기록한다.
- `success`는 타격 뒤 새 손 표본의 **펼침 → 쥠 0.3 s 유지 → 펼침**을 관측한 받기 후보 판정이다.
  같은 stamp 반복·타격 전 표본은 인정하지 않으며 손 미검출·오래된 관측은 연속 파지를 끊는다.
  공을 실제로 잡았다는 증거는 아니므로 실물 검증 전 자동 난이도 변경은 차단한다. 명시적 `harder`/`easier`만 시도 경계에서 반영한다.
- 받기 후보는 20 s까지 관측하고, 성공 여부와 별개로 **손 이탈을 확인할 때까지 세션을 유지한다**.
  `hand_clear_wait_s`(8 s)는 안내까지의 시간이다. 이후에는 시간 제한 없이 대기하며, 손을 늦게 빼도 회수가 이어진다.
  신선하고 유효한 손 거리 ≥ `safety.slow_distance_m`(0.25 m)일 때만 회수한다. 손 미검출은 이탈로 간주하지 않는다.
  대기 중 회수·다음 시도·모드 전환을 시작하지 않는다. 세션 취소·안전 STOP은 기존대로 중단하며 자동 재개하지 않는다.
- 회수 goal 전송 직전(재시도 포함)에도 손 이탈을 검사하며 다시 가까워졌으면 기다린다.
  회수 실패나 손 보기 자세 서비스 실패는 다음 시도로 진행하지 않는다. 회수 여부는 `ball_returned`, 손동작은 `catch_gesture_detected`로 따로 기록한다.
- 측정되지 않은 정지 위치·오차는 NaN으로 기록한다. 실측 결과를 이용한 온라인 레시피 갱신은 아직 하지 않는다.

### 로봇 없이 눈으로 확인

서버 패키지 빌드 후 각 터미널에서 다음 공통 환경을 먼저 적용한다.

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/rally-mode1-build/install/setup.bash  # 본인 빌드 환경의 install 경로
unset ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/rally-mode1-build/roslog
```

저장소 루트에서 터미널별로 실행한다. 실제 로봇 launch나 context_builder는 함께 실행하지 않는다.

```bash
# 1: 가짜 손(.86 m, base_link)·스킬·안내·손 보기 서비스
python3 scripts/fake_skills.py --duration 3
# 2: brain — 준비 확인 해제는 이 가짜 환경 전용
ros2 run rally_brain bt_runner --ros-args -p check_readiness:=false
# 3: 공 4회 (balls_per_cycle)
ros2 action send_goal /rally/session/run rally_interfaces/action/RunSession '{modes: [1]}'
# 4: 결과 및 목표/속도 기록
ros2 topic echo /rally/brain/trial_result
# 5: 명시적 난이도 변경 (별도 세션을 시작해 첫 시도 도중 입력)
ros2 topic pub --once /rally/voice/intent rally_interfaces/msg/VoiceIntent '{header: auto, intent: harder}'
```

기본 가짜 손은 펼친 상태라 20 s 뒤 `not_caught`를 기록하고 회수한다. 받기 후보 4회 성공만으로도 level이 증가하면 안 된다. `harder` 입력 후에는 다음 시도에서 level이 1 증가하고,
범위 안에서 목표가 환자에게서 멀어지며 속도가 낮아진다. 제한에 걸리면 단계가 달라도 목표가 같을 수 있다.
현재 안전 여유와 속도 상한에서 레인별 가능한 목표 구간이 좁으므로 실물 범위를 확인해야 한다.
가짜 노드에서 `ros2 param set /rally_fake_skills hand false` 후 새 세션을 시작하면 약 20 s 대기 후
`no_hand` 기록이 나오고 새 `push_ball` 시작 로그가 없어야 한다. `hand true`로 다시 켤 수 있다.
`stop true`는 세션을 중단하며 손 대기 중에도 처리돼야 한다. 종료는 각 터미널 Ctrl+C.

손 이탈 대기를 눈으로 확인하려면 세션 시작 전에 `ros2 param set /rally_fake_skills hand_distance 0.20`을 실행한다.
타격·받기 관측 후 8 s가 지나 안내가 나와도 `return_ball` 시작과 시도 기록이 없어야 한다.
`hand_distance 0.30`으로 바꾸면 회수가 시작되고 기록이 한 번 나온다. 대기 중 `hand false`로 바꿔도 회수하지 않아야 한다.
받기 후보 확인은 타격 완료 뒤 `grip 0.8`로 바꾸고 0.5 s 이상 유지 후 `grip 0.0`으로 바꾼다(명령 형식은 위와 같은 `ros2 param set`).

**로봇·인식 PC에서 확인 필요:** #42/#57의 실제 PushBall 스킬, 카메라에서 밀기 전 손 관측 가능 여부,
레인별 실제 정지 오차·범위·손동작과 실제 공 잡기의 일치, `rack_drop_posx` 티칭 및 ReturnBall 동작.
손 보기 Trigger는 action 취소 ACK가 없으므로 일반 세션 취소 중 실제 자세 이동의 중단 연동은 로봇 담당자 확인이 필요하다. #42/#57은 통합 도중 main에 머지되어 반영됐으며 이 PR이 직접 머지한 것은 아니다.
