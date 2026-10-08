# rally_brain

담당: **조정묵**

상황 요약, 규칙 레이어, LLM advisor, py_trees 행동 트리

인터페이스는 `docs/INTERFACES.md`, 할 일은 `todo/` 참고.

## 모드 1 — 손 기준 목표와 레시피 (#81 · #84)

`신선한 손(base_link) → 무작위 레인 → 목표 위치 → brain 속도 계산 → PushBall → TrialResult`.
중앙선은 `table.yaml`의 `table.centerline_x_m`만 읽는다. 환자 쪽은 +x이며 목표는 중앙선+5 cm 이상이다.

- `skills.yaml mode1.lanes[].recipe`: PR #57 `cd92d1a`에서 보고한 10/8 실측 a·d0·RMSE와 레인 입구·방향을 사용.
  a는 타격 속도를 기준으로 맞춘 계수로, 물리적 마찰 감속도와 동일하지 않다.
- 개발용 `reach_requested = (level + 1) × reach_rmse_multiplier × RMSE`, level은 0 이상.
  사용자 합의값 `reach_rmse_multiplier=2`에 따라 초기 뻗기·단계 간격은 레인별 4.2/4.8/5.8 cm.
  임상적으로 검증된 난이도 값이 아니며 실물 확인 후 설정을 조정한다.
- 손을 레인 축에 투영한 거리에서 reach를 빼고, 실측 거리 `[0.15, 0.45] m`, 실측 속도 `[0.2, 0.4] m/s`,
  운용 속도 제한, 중앙선 여유, 손보다 앞의 구간을 모두 만족하도록 목표를 제한한다.
  교집합이 없으면 `unreachable_target`으로 기록하고 공을 치지 않는다.
- 현재 운용 속도 상한 **0.35 m/s를 유지**한다. #57의 0.60 m/s로 올리지 않는다.
  속도만 잘라서 목표와 불일치시키지 않고 가능한 목표 구간을 먼저 줄인다.
  따라서 손이 너무 멀면 실제 뻗기 거리가 요청값보다 커질 수 있다. `reach_requested_m`과 `reach_actual_m`을 따로 기록한다.
- `PushBall.target`과 계산한 **양수 `push_speed`를 함께 전송**한다. 0을 보내 로봇에 계산을 맡기지 않는다.
  로봇의 최종 범위 제한·안전 감시는 그대로다. 로봇 PC와 서버 PC의 레인 좌표·레시피 설정을 맞춰야 한다.
- 손 미검출/NaN/다른 좌표계/1 s 이상 오래된 stamp/미래·zero stamp는 10 s씩 2회 대기 후 `no_hand` 기록, goal 0개.
  goal 전송 직전에도 안전·손·목표 계산에 사용한 stamp를 재검사한다. 두 PC 시계 동기화가 필요하다.
- 레시피 누락·잘못된 계수는 고정 속도로 대신 치지 않는다. `no_recipe`/`invalid_recipe` 등 사유를 기록한다.
- 현재 `success`는 **로봇 타격 성공**이다. 모드 1은 이를 규칙의 성공/실패 이력에서 제외해 자동 난이도 변경을 막는다.
  명시적 `harder`/`easier`만 시도 경계에서 반영한다. 환자 잡기 성공 판정은 별도 신호 연결이 필요하다.
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
# 1: 가짜 손(.78 m, base_link)·스킬·안내
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

기본 4회 성공만으로 level이 증가하면 안 된다. `harder` 입력 후에는 다음 시도에서 level이 1 증가하고,
범위 안에서 목표가 환자에게서 멀어지며 속도가 낮아진다. 랜덤 레인이 달라지면 같은 레인끼리 비교한다.
가짜 노드에서 `ros2 param set /rally_fake_skills hand false` 후 새 세션을 시작하면 약 20 s 대기 후
`no_hand` 기록이 나오고 새 `push_ball` 시작 로그가 없어야 한다. `hand true`로 다시 켤 수 있다.
`stop true`는 세션을 중단하며 손 대기 중에도 처리돼야 한다. 종료는 각 터미널 Ctrl+C.

**로봇·인식 PC에서 확인 필요:** #42/#57의 실제 PushBall 스킬, 카메라에서 밀기 전 손 관측 가능 여부,
레인별 실제 정지 오차·범위·환자 잡기 판정. 이번 PR은 로봇 스킬 PR을 병합하지 않는다.
