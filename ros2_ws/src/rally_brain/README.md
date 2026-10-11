# rally_brain

담당: **조정묵**

상황 요약, 규칙 레이어, LLM advisor, py_trees 행동 트리

인터페이스는 `docs/INTERFACES.md`, 할 일은 `todo/` 참고.

## 모드 1 — 고정 공 목표·손 시작 위치와 레시피 (#81 · #84 · #85 · #87)

`손 보기 자세 → 신선한 손(base_link) → 손에 가까운 레인 → 고정 공 목표·속도 → 손 시작 위치·NORMAL 대기 → PushBall → 받기 후보 관측 → 손 이탈 대기 → ReturnBall → TrialResult`.
중앙선은 `table.yaml`의 `table.centerline_x_m`만 읽는다. 환자 쪽은 +x이다. 목표 하한은 중앙선 + `max(patient_margin_m, safety.stop_distance_m + 0.05)`로 계산한다.
현재 설정은 중앙선 0.568 + 여유 0.20 = x 0.768 m. 이 여유는 계획값이며 실제 손끝 거리·공 정지 오차를 보장하지 않는다.

- `skills.yaml mode1.lanes[].recipe`: PR #57 `cd92d1a`에서 보고한 10/8 실측 a·d0·RMSE와 레인 입구·방향을 사용.
  a는 타격 속도를 기준으로 맞춘 계수로, 물리적 마찰 감속도와 동일하지 않다.
- 개발용 `reach_requested = (level + 1) × reach_rmse_multiplier × RMSE`, level은 0 이상.
  사용자 합의값 `reach_rmse_multiplier=2`에 따라 초기 뻗기·단계 간격은 레인별 4.2/4.8/5.8 cm.
  임상적으로 검증된 난이도 값이 아니며 실물 확인 후 설정을 조정한다.
- 공 목표는 `lanes[].target_d_m`(레인 입구 기준)으로 고정한다. 개발용 초기값은 세 레인 모두 0.35 m이며 실물 확인 전 확정값이 아니다.
  실측 거리 `[0.15, 0.45] m`, 실측 속도 `[0.2, 0.4] m/s`, 로봇 제한과 **Brain 상한 0.37 m/s**, 중앙선 여유의 교집합으로 제한한다.
  교집합이 없으면 `unreachable_target`으로 기록하고 공을 치지 않는다. 0.40 m/s는 테이블 끝에 근접했다는 PM 정정에 따라 운용하지 않는다.
- 난이도는 공 목표를 움직이지 않고 필요한 손 시작 위치를 바꾼다. 손을 레인 축에 투영한 거리 ≥ `목표 거리 + reach_requested`이고 안전 NORMAL일 때만 보낸다.
  손 위치가 부족하면 몸 쪽으로 더 두도록 안내하고 계속 기다린다. 대기 뒤 및 전송 직전 최신 표본으로 레인·거리·NORMAL을 재검사한다.
  손이 더 멀면 실제 뻗기는 요청보다 커진다. `reach_requested_m`, `reach_actual_m`, `hand_start_d_m`을 기록하며 처음부터 쉬운 거리라고 보장하지 않는다.
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
  신선하고 유효한 손 거리 ≥ `skills.yaml mode1.hand_clear_m`(0.20 m)일 때만 회수한다. 기다리는 이유(손 안 보임·거리 등)는 2 s마다 로그. 손 미검출은 이탈로 간주하지 않는다.
  대기 중 회수·다음 시도·모드 전환을 시작하지 않는다. 세션 취소·안전 STOP은 기존대로 중단하며 자동 재개하지 않는다.
- 회수 goal 전송 직전(재시도 포함)에도 손 이탈을 검사하며 다시 가까워졌으면 기다린다.
  회수 실패나 손 보기 자세 서비스 실패는 다음 시도로 진행하지 않는다. 회수 여부는 `ball_returned`, 손동작은 `catch_gesture_detected`로 따로 기록한다.
- 측정되지 않은 정지 위치·오차는 NaN으로 기록한다. 실측 결과를 이용한 온라인 레시피 갱신은 아직 하지 않는다.

### 로봇 없이 눈으로 확인

서버 패키지 빌드 후 각 터미널에서 다음 공통 환경을 먼저 적용한다.

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/rally-pause-build/install/setup.bash  # 본인 빌드 환경의 install 경로
unset ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/rally-pause-build/roslog
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
공 목표·속도는 유지되고 필요한 손 시작 거리가 증가한다. 손 위치가 부족하면 안내 후 대기한다.
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

## 휴식·재개 직접 확인 (로봇 없이)

현재 시도 전체가 끝나면 쉬고, 음성 또는 대시보드로 같은 난이도에서 다음 시도를 시작한다.
모드 5에서는 집기 → 건네기 → 받기까지 끝낸다. 마지막 시도는 세션 종료로 처리한다.
`harder`/`easier`는 모드 1의 명시적 요청 및 모드 7 레시피를 지원한다. 모드 1 자동 난이도 변경은 차단한다. 휴식 요청 후 재개까지 모드·난이도는 유지한다.

먼저 ROS Jazzy에서 서버 패키지를 빌드한다 (`AGENTS.md` 검증 절차).
Python 환경에는 ROS 외에 brain 의존성 및 `redis`, `fastapi`, `uvicorn`, `psycopg[binary]`가 필요하다.
이번 작업환경에서는 `/tmp/rally-pause-build/install/setup.bash`와 `/tmp/rally-control-venv/bin/python`으로 검증했다.

**아래 공통 설정을 테스트 터미널마다 적용한다.** 실제 로봇 launch·context_builder는 실행하지 않는다.

```bash
cd /tmp/rally-pause-integration
source /opt/ros/jazzy/setup.bash
source /tmp/rally-pause-build/install/setup.bash  # 다른 환경에서는 본인 install/setup.bash
unset ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export REDIS_URL=redis://127.0.0.1:16379/0
export ROS_LOG_DIR=/tmp/rally-review-ros-logs
source /tmp/rally-control-venv/bin/activate  # 다른 환경에서는 의존성 설치한 venv
```

터미널별로 하나씩 실행한다. Redis는 기존 6379와 분리된 임시 컨테이너다.

```bash
# 터미널 1: 테스트 Redis (종료는 Ctrl+C)
docker run --rm --name rally-control-test -p 127.0.0.1:16379:6379 redis:7-alpine redis-server --save '' --appendonly no

# 터미널 2: 가짜 스킬 + 가짜 상황판 + 안내 로그
python scripts/fake_skills.py --duration 3

# 터미널 3: brain — 준비 확인 해제는 이 가짜 환경에서만 사용
python -m rally_brain.bt_runner --ros-args -p check_readiness:=false

# 터미널 4: 대시보드 (DB가 없어도 휴식·재개 패널은 동작)
python -m uvicorn dashboard.app.main:app --host 127.0.0.1 --port 18000

# 터미널 5: 모드 5 세션 시작 (스킬당 3 s, 한 시도는 약 9 s)
ros2 action send_goal /rally/session/run rally_interfaces/action/RunSession '{modes: [5], trials_per_mode: 5}'

# 터미널 6: 시도 진행 중 휴식 요청
ros2 topic pub --once /rally/voice/intent rally_interfaces/msg/VoiceIntent '{header: auto, intent: pause}'
```

브라우저에서 `http://127.0.0.1:18000`을 연다.

1. **휴식 대기**가 표시돼도 터미널 2의 GIVE·RECEIVE는 끝까지 진행한다.
2. **휴식 중**이 되면 새 `pick_compliant`가 시작되지 않는지 확인한다. 난이도를 기억한다.
3. **훈련 재개**를 누른다. brain 승인 메시지와 함께 다음 시도가 한 번 시작하고 난이도는 같다.
4. 다시 `pause` 후 아래 음성 의도 명령으로도 재개되는지 확인한다.

```bash
ros2 topic pub --once /rally/voice/intent rally_interfaces/msg/VoiceIntent '{header: auto, intent: resume}'
# 휴식 중 안전 STOP 흉내: 세션이 종료돼야 하며 재개 버튼으로 살아나면 안 됨
ros2 param set /rally_fake_skills stop true
ros2 param set /rally_fake_skills stop false
```

Redis를 종료하면 패널은 연결 끊김으로 바뀌고 재개 버튼이 꺼진다. 복구해도 이전 재개 요청은 실행하지 않는다.
DB/recorder를 실행하지 않은 이 테스트에서는 기록 표가 비어 있고 상단 안전 표시가 '모름'인 것이 정상이다.
실제 마이크 인식·2 PC 통신·실물 인수인계 및 실패 시 물체 보유 상태·순응 제어 해제는 **로봇·인식 PC에서 확인 필요**.
끝나면 각 테스트 프로세스를 Ctrl+C로 종료한다. 이 테스트는 실제 로봇 안전 reset을 호출하지 않는다.

### PR #87 모드 1 통합 확인

위 휴식·재개 공통 환경과 터미널 1~4를 그대로 사용한다. 터미널 5에서는 모드 5 대신 아래 명령으로 시작한다.
손 거리 변경은 가짜 노드의 입력만 바꾸며 실제 로봇에는 사용하지 않는다.

```bash
# 시작 전: 손이 아직 회수 구역에 남아 있는 상황
ros2 param set /rally_fake_skills hand_distance 0.20
ros2 action send_goal /rally/session/run rally_interfaces/action/RunSession '{modes: [1]}'

# 별도 터미널: 타격 진행 중 또는 손 이탈 대기 중 휴식 요청
ros2 topic pub --once /rally/voice/intent rally_interfaces/msg/VoiceIntent '{header: auto, intent: pause}'
```

1. 화면은 **현재 시도 완료 후 휴식 예정**을 표시한다. 기본 가짜 손은 계속 펼친 상태이므로 받기 관측은 20 s 뒤 끝난다.
2. 손 이탈 안내(8 s) 뒤에도 새 `return_ball`이나 다음 `push_ball`이 시작되지 않아야 한다. 아직 **휴식 중**이 아니므로 재개 버튼도 비활성이다.
3. 아래처럼 손 거리를 바꾸면 `return_ball`이 한 번 실행되고 시도가 기록된 다음 **휴식 중**이 된다.

```bash
ros2 param set /rally_fake_skills hand_distance 0.30
ros2 topic echo /rally/brain/trial_result
```

4. 화면의 **훈련 재개**를 누른다. 다음 `push_ball`이 시작하고 모드·난이도가 유지돼야 한다.
5. 다음 시도 중 `pause`를 보내 다시 휴식한 뒤 `resume` 음성 의도로도 같은 결과를 확인한다.
6. 별도 세션에서 손 이탈 대기/휴식 중 `stop true`를 보낸다. 세션이 종료돼야 하며 `stop false`와 `resume`으로 살아나면 안 된다.

명시적 난이도는 휴식 요청이 없는 시도 도중 `intent: harder`/`intent: easier`를 보내 확인한다.
다음 시도부터 모드 1 level이 바뀌고 0 아래로 내려가지 않는다. 받기 후보 연속 성공만으로 자동 변경되지 않는다.
손 이탈 대기 중 휴식 요청은 회수까지 끝낸 뒤 적용한다. 마지막 시도라면 휴식 대신 세션이 끝난다.

검증 기록: 서버 4패키지 격리 빌드, brain/db/dashboard pytest 결과는 담당 TODO에 기록.
localhost ROS + 가짜 스킬 + 임시 Redis + 대시보드 API로 지연된 손 이탈→회수→휴식,
HTTP 재개·중복 거절, 음성 재개·난이도 유지, 휴식 요청 중 STOP 및 재개 거절을 확인했다.
이 검증은 실제 마이크·로봇·2 PC 통신과 브라우저 버튼의 사용자 확인을 대신하지 않는다.

### 고정 목표·손 시작 위치 확인 (PM 정정 반영)

가짜 세션 시작 전 `ros2 param set /rally_fake_skills hand_x 0.80`으로 둔다.
손 위치 안내가 나오고 PushBall이 시작되지 않아야 한다. `hand_x 0.90`으로 바꾸면 시작한다.
별도 세션에서는 시작 전 `slow true`로 두면 기다리고, `slow false` 뒤 NORMAL에서 시작한다.
명령 형식은 모두 `ros2 param set /rally_fake_skills <파라미터> <값>`이다.
`harder` 후 공 목표·속도는 같고 `hand_start_d_m`만 증가하는지 확인한다. 높은 단계는 `hand_x 1.00` 등으로 뒤로 이동시켜 가짜 흐름을 확인한다.
실제 손 위치·도달 가능 범위는 실물 검증이 필요하며 가짜 좌표를 환자에게 요구하는 기준으로 쓰지 않는다.
PushBall 실패 시 공/그리퍼 상태가 불명확하므로 자동 회수나 다음 시도 없이 실패 기록 후 세션을 중단한다.

### 모드별 대시보드 화면

대시보드의 전체 현황과 모드 1·3·5·7 탭은 조회용이다. 탭 전환으로 훈련 모드가 바뀌지는 않는다. 각 페이지는 동작 설명 그림, 현재 실행 단계, 난이도·세션 상태와 해당 모드의 최근 시도를 표시한다. 그림의 좌표와 공 궤적은 실측값이 아니다.

현재 단계는 `rally:session_control.bt_node`를 이용하며, 2초 이상 지난 상태나 휴식·종료 상태에서는 실행 단계를 강조하지 않는다. 모드 3은 챌린지 설정을 표시하되 세부 단계·궤적은 연결 대기, 모드 7의 실시간 힘도 연결 대기로 표시한다. 안전 배너와 기존 휴식 재개 기능은 모든 페이지에서 확인할 수 있다.

로봇 없이 확인: `node dashboard/test/test_mode_pages.cjs`. 대시보드 실행 후 `/#mode1`, `/#mode3`, `/#mode5`, `/#mode7`을 열어 그림·단계·기록 필터를 확인한다. Brain의 새 필드를 받으려면 수정된 Brain을 재시작해야 한다.

### 휴식 제안 확인 테스트 (테스트 생성 경로)

작업본: `/tmp/rally-dashboard-pr`. 기존 격리 가짜 스킬 실행 절차를 사용하되 **모든 터미널을 이 작업본에서** 시작하고 `PYTHONPATH=/tmp/rally-dashboard-pr/ros2_ws/src/rally_brain:$PYTHONPATH`로 수정된 Brain 소스를 선택한다. 실제 로봇 launch와 context_builder는 실행하지 않는다.

Brain 시작에 `-p test_rest_proposals:=true`를 추가한다(기본 false). 가짜 모드 5 세션이 running일 때 다음 명령으로 휴식 제안 한 개를 만든다.

```bash
ros2 service call /rally/brain/test/propose_rest std_srvs/srv/Trigger '{}'
```

화면에 “혹시 피곤하진 않으세요? 조금 쉬었다 할까요?”와 20초 카운트다운이 나타난다. 가짜 Speak 서버는 발화를 로그로만 표시한다.

1. 아무 버튼도 누르지 않으면 훈련이 계속되고 20초 뒤 만료된다.
2. 새 제안에서 “아니요, 계속할게요”를 누르면 즉시 거절되고 훈련은 계속된다.
3. 새 제안에서 “네, 쉬겠습니다”를 누르면 현재 시도의 집기·GIVE·RECEIVE가 끝난 뒤 휴식한다. 마지막 시도라면 세션이 종료된다. 기존 훈련 재개 버튼으로 이어간다.
4. 제안 중 가짜 STOP을 켜면 세션이 종료되고 제안을 수락해 되살릴 수 없다.

이 테스트 서비스는 rest 제안 UI 검증용이다. 실제 LLM 경로는 아래 「LLM 휴식 판단 연결」 절을 따른다. 음성 수락/거절 및 tired 분류 변경, 난이도 제안은 후속 작업이다. 제안에 `source: test`를 표시하며 실제 피로 판단으로 기록하지 않는다.

#### 휴식 제안 안정화 검증 (10/8)

- Brain 249개(격리 Redis 왕복 1개 포함) + 대시보드 API 17개, 합계 266개 통과. Node 화면 검사는 클릭·중복 클릭·504·네트워크 오류·상태 갱신·재접속·새 제안·늦은 응답을 확인한다.
- 수락 후 원래 20초가 지나도 휴식 예약을 유지한다. 모드 1 회수·기록과 모드 5 RECEIVE·기록 전에는 쉬지 않는 회귀 검사를 추가했다. 마지막 시도는 휴식 대신 정상 종료한다.
- localhost 전용 ROS domain 222 + 별도 임시 Redis에서 거절·중복 거절·20초 만료 중 훈련 지속·수락 후 경계 휴식·STOP·취소·마지막 시도 종료 왕복을 확인했다. 실제 로봇·DB에는 연결하지 않았다.
- 브라우저에서 모드 1·3·5·7 탭 표시, 휴식 제안 카운트다운, 거절 결과 안내 보존, 수락·재개 버튼, 연결 종료 표시를 확인했다. 오류 응답 안내는 같은 세션·제안의 갱신·재접속 뒤에도 남으며, 새 세션에서는 이전 안내를 지운다. 최신 상태 없이는 응답할 수 없다.

위 가짜 환경에서 `http://127.0.0.1:18000/#mode5`를 열어 확인한다(이번 격리 검증은 별도 포트 18092 사용 후 종료).
Brain 소스는 반드시 이 작업본의 `PYTHONPATH`를 우선한다. 다른 가짜 세션과도 ROS 도메인·Redis 포트를 분리하고 `ROS_DISCOVERY_SERVER`, `FASTRTPS_DEFAULT_PROFILES_FILE`, `FASTDDS_DEFAULT_PROFILES_FILE`, `ROS_STATIC_PEERS`를 해제한 뒤 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`로 제한한다.
실제 LLM·음성 수락/거절·2 PC 통신·실제 로봇은 미검증이며 **로봇·인식 PC에서 확인 필요**. #98 STOP 예외는 이번 변경에 포함하지 않는다.


### LLM 휴식 판단 연결

규칙의 `ask_llm` → 비동기 휴식 판단 → 최신 상태 재검증 → 20초 제안 → 대시보드 응답 → 기존 시도 경계 휴식 순서다. 대기 중 BT는 계속 실행한다. 외부 호출 결과만으로 쉬거나 재개하지 않는다.

OpenAI 공식 API의 gpt-4o에 현재 모드, 같은 모드·난이도 구간의 최근 유효 시도 최대 5개(번호·성공 여부·유한 오차·소요 시간), 보상 동작/시간 증가 등의 불리언 신호만 전송한다. 모드1의 받기 후보 성공 여부는 제외한다. 키는 기존 `docker/.env`의 `OPENAI_API_KEY`로 제공하고 출력하거나 저장소에 넣지 않는다. 승인된 목적지 외의 API 주소로 우회하지 않는다.

시도 소요 시간에는 로봇 실행과 시도 중 대기가 포함된다. 시간이 길다는 것만으로 환자 피로를 단정하지 않는다. `source: llm`은 화면에서 **AI 제안**으로 보이고 테스트 서비스의 `source: test`는 **테스트**로 구분한다. 거절·만료 시 훈련 유지, 수락 시 현재 시도 전체를 마친 뒤 휴식하는 동작은 같다.

최대 3초 시간 초과·인증/네트워크 오류·허용 목록 밖 응답은 제안 없이 훈련을 유지한다. 이전 세션·모드·난이도·시도, STOP/취소/휴식/상황판 상실 전의 응답은 폐기한다. 숫자 제어용 SuggestParams·로봇 임계값·STOP/reset은 변경하지 않는다. 음성 수락/거절과 KG/Graph RAG 검색은 후속이다.


설정은 설치된 `rally_bringup/config/llm.yaml`의 `rally_bt_runner.ros__parameters`를 시작 시 직접 읽는다. `rest_enabled: true`가 기본이며 `OPENAI_API_KEY`가 없으면 외부 호출 없이 계속 훈련한다. `rest_enabled: false`로 변경하고 재시작하면 끌 수 있다. ROS 동적 파라미터가 아니므로 `ros2 param set`으로 켜지 않는다. 시작 로그 `휴식 LLM: ready/missing_api_key/disabled/invalid_config/adapter_unavailable`로 설정 상태를 구분한다.

검증 실행(실제 로봇 없이, 저장소 루트):

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/rally-llm-rest-check/install/setup.bash  # 본인의 최신 빌드 경로로 교체
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_STATIC_PEERS
unset OPENAI_API_KEY REDIS_URL
export ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest ros2_ws/src/rally_brain/test/test_rest_advisor.py -q
node dashboard/test/test_mode_pages.cjs
```

첫 검사는 실제 BtRunner에 가짜 LLM을 주입해 규칙→제안→수락→휴식 및 늦은 응답 폐기를 재현한다. 눈으로 확인할 때는 위 격리 가짜 세션의 대시보드에서 `source: llm` 제안이 **AI 제안**으로 표시되는지, 대기 중 훈련 지속·거절/만료 후 지속·수락 후 현재 시도 완료와 휴식을 확인한다. `/rally/brain/test/propose_rest`는 **테스트** 표시만 검증하므로 실제 API 연결 증거로 삼지 않는다.

실제 API 별도 확인: 환자 기록 없이 합성 모드5 이력 3개와 허용 신호만 공식 gpt-4o로 전송해 `suggest_rest`, 1.832초를 확인했다. 이는 API 연결 확인이며 임상 정확도·실제 장비 통합·항상 3초 내 응답을 보장하는 검증은 아니다.

최종 로컬 검증: 4패키지 격리 build 성공, colcon 311 tests/0 errors/0 failures/1 skip(Brain284+DB25+bringup1 통과, 전용 Redis1 생략), 대시보드 API26·Node 회귀·ontology export·Compose pc3 config 검사 통과. 실제 로봇·인식 PC에서 확인 필요.
