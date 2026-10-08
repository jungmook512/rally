# RALLY ROS 2 인터페이스 명세

ROS 2 Jazzy 기준. 표준 메시지를 쓸 수 있는 곳은 표준을 쓰고, 프로젝트 고유 데이터만 `rally_interfaces`에 정의한다. 모든 커스텀 메시지는 `std_msgs/Header`를 가지며, 3D 좌표는 특별한 언급이 없으면 `base_link`(로봇 베이스) 기준이다.

## 1. 좌표계 (tf2)

```
base_link            로봇 베이스 (world 역할)
 └─ link_6 / tool0   플랜지
     ├─ gripper_tcp  RG2 TCP (set_tcp 값과 일치)
     └─ camera_link  RealSense (eye-in-hand 캘리브레이션 결과, static)
         └─ camera_color_optical_frame
table_link           테이블 원점 (base_link 기준 static, 중앙선 y=0)
webcam_link          측면 대각 웹캠, 높이 약 2 m (table_link 기준 static) — 모드 3 공 비행 + 몸 자세(BodyPose)
```

- `camera_link`는 `rally_vision/camera_tf`(`ros2 run rally_vision camera_tf`)가 `hand_eye.yaml`을 읽어 올린다. 기본 `mode: static`은 두산 `base_link → link_6`(robot_state_publisher, 10/8 실로봇 약 56 Hz) 아래에 `link_6 → camera_link`(플랜지→TCP 0.228 m + hand_eye)를 `republish_hz`로 발행한다 — 영상 시각으로 보간되는 정확한 tf. **`/dsr01/joint_states`가 1 s 넘게 끊기거나 팔 6축이 전부 0이면(ros2_control 죽음) 발행을 멈추고 error 로그** — 그래야 받는 쪽이 NaN/폴백으로 가고 '그럴듯하지만 틀린 좌표'(손 z 1.4 m)가 안 나간다. `mode: tcp_pose`는 `/rally/robot/tcp_pose`(10 Hz)로 `base_link → camera_link`(두산 tf 없는 노트북·에뮬레이터용, segmenter `tcp_npy`와 같은 값). **`camera_link` 에 다른 부모(URDF `camera_bottom_screw_frame`)가 /tf_static 에 있으면 부모가 둘이 되므로 error 를 내고 발행하지 않는다** — bringup URDF 에서 카메라 체인(bracket·camera_*)을 빼야 static 모드가 켜진다(@정정인). tcp_pose 모드는 마지막 변환을 `republish_hz`(30 Hz)로 현재 시각에 다시 발행(hold-last)해 영상 stamp 조회가 되게 한다 — 값은 최근 tcp_pose(≤ 0.1 s 전)라 로봇 정지 중에만 정확 (static 모드는 두산 tf 가 보간하므로 이동 중에도 정확). 기본은 RealSense의 `camera_link → … → optical` static tf를 기다렸다가 `camera_link`를 자식으로 발행한다. RealSense가 tf를 안 내는 환경(`publish_tf:=false`)에서만 `optical_fallback: true`로 optical 프레임을 직접 발행한다(기본 false — 폴백 중 optical을 받은 노드는 RealSense가 늦게 켜져도 조회가 계속 실패하므로). `RunCalibration` 서비스는 아직 없음(Day 7 여유 시).
- **깊이 보정**: 이 RealSense는 정렬 깊이가 일관되게 약 3 % 길게 나온다(10/7·10/8 ArUco 실측). `hand_eye.yaml depth_scale`(0.971)을 segmenter가 깊이에 곱한다. 깊이를 직접 쓰는 다른 노드(손 인식)도 같은 값을 곱해야 base 좌표가 맞는다.
- 비전 노드는 `camera_color_optical_frame` → `base_link` 변환을 tf2로 매 프레임 조회한다 (카메라가 팔과 함께 움직이므로 반드시 timestamp 일치).

## 2. 토픽

### 2.1 센서 (표준 메시지)

| 토픽 | 타입 | 발행 | 주기 | QoS |
|---|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | realsense2_camera | 30 Hz | SensorData |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | realsense2_camera | 30 Hz | SensorData |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | realsense2_camera | 30 Hz | SensorData |
| `/webcam/image_raw` | `sensor_msgs/Image` | usb_cam (측면) | 60 Hz | SensorData |
| `/rally/robot/wrench` | `geometry_msgs/WrenchStamped` | rally_robot/force_publisher | 50 Hz (두산 서비스 폴링 — 100 Hz 면 컨트롤러 서비스가 막힘) | SensorData |
| `/rally/robot/tcp_pose` | `geometry_msgs/PoseStamped` | rally_robot/force_publisher | 10 Hz | SensorData |
| `/rally/gripper/state` | `sensor_msgs/JointState` — name `[rg2_width_mm, rg2_grip, rg2_target_mm]`, position `[폭 mm, grip 감지 0/1, 마지막 명령 폭 mm(-1 = 없음)]`, effort 미사용. 쥠 판단 = grip 비트 **또는** 명령 폭보다 5 mm 이상 벌어져 멈춤(공을 손가락 끝에 걸치면 grip 비트 0) | rally_robot/gripper_node | 20 Hz | SensorData |
| `/joint_states` | `sensor_msgs/JointState` | dsr_bringup2 | 100 Hz | SensorData |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 여러 노드 | - | 표준 |

### 2.2 인식 (커스텀)

| 토픽 | 타입 | 발행 | 구독 | 주기 | QoS |
|---|---|---|---|---|---|
| `/rally/perception/objects` | `rally_interfaces/TrackedObjectArray` | rally_vision/segmenter (ByteTrack tracker 전까지) | brain, robot | 30 Hz (웹캠 비행 추적은 60 Hz) | BestEffort, depth 1 |
| `/rally/perception/landing` | `geometry_msgs/PointStamped` (+ confidence는 objects에) | rally_vision/flight_predictor | robot(catch_ball) | 60 Hz, 비행 중만 | BestEffort, depth 1 |
| `/rally/perception/objects/debug_image` | `sensor_msgs/Image` | rally_vision/segmenter (ByteTrack tracker 전까지) | dashboard | 10 Hz | BestEffort |
| `/rally/perception/hand` | `rally_interfaces/HandState` | rally_human/hand_pose_node | brain, robot(safety), robot(handover) | 30 Hz | BestEffort, depth 1 |
| `/rally/perception/body` | `rally_interfaces/BodyPose` | rally_human/body_pose_node | brain | 15 Hz | BestEffort |
| `/rally/voice/intent` | `rally_interfaces/VoiceIntent` | rally_voice/intent_node | brain, safety | 이벤트 | Reliable |
| `/rally/voice/transcript` | `std_msgs/String` | rally_voice/intent_node | dashboard | 이벤트 | Reliable |
| `/rally/voice/speaking` | `std_msgs/Bool` | rally_voice/tts_node | rally_voice/intent_node (dashboard 선택) | 발화 시작·끝 이벤트 | **Reliable, TransientLocal**, depth 1 |

**objects 좌표 프레임** (#66): `/rally/perception/objects` 의 `header.frame_id` 가 `base_link` 가 아니면(카메라 프레임 `camera_color_optical_frame`) 카메라→base 변환을 못 구한 것이다 — 그 좌표로 로봇을 움직이지 않는다(abort 또는 대기). segmenter 는 파라미터 `transform_source` 로 변환을 고른다: `tcp_npy`(기본, `/rally/robot/tcp_pose` × hand_eye) · `tf` · `auto`(tf 우선). **로봇 PC bringup URDF(`m0609_with_rg2_camera`)가 `tool0 → bracket_link → camera_bottom_screw_frame → camera_link → … → camera_color_optical_frame` 을 설계값으로 발행**(10/8 정정인 view_frames)하므로, 그 체인이 있는 동안 `tf`/`auto` 는 설계값을 집어 1 cm·2.5° 치우친다(PR #42 원인). URDF 에서 카메라 체인을 빼고 `camera_tf` 가 유일한 `camera_link` 부모가 되면 기본을 `auto` 로 바꾼다. `tcp_npy` 는 영상 시각이 아니라 마지막 `tcp_pose`(10 Hz) 를 쓰므로 **로봇이 멈춰 있을 때의 좌표만 정확**하다 — 이동 중 좌표로 로봇을 보내지 않는다. `camera_tf` 가 떠 있으면 `auto`(tf 는 영상 시각으로 보간) 가 낫다.

**HandState 거리 NaN** (#44): `detected=true` 인데 손 위치를 못 구하면(정렬 깊이 없음·컬러와 시각이 맞는 깊이 없음·`base_link` tf 없음) `distance_to_centerline = NaN` = "손은 보이는데 거리 모름". 가짜 거리를 만들지 않는다. safety_monitor 는 `detected && NaN` 이면 SLOW. tf 가 없을 때는 `header.frame_id` 가 카메라 프레임.

**safety_monitor 손 거리·입력 끊김** (#59, #14): 손 없음 → NORMAL / NaN → SLOW(`hand_distance_unknown`) / `slow_distance_m` 안 → SLOW / `stop_distance_m` 안 → STOP(`hand_near_centerline`). 단 `/rally/safety/patient_contact` 동안은 SLOW(`hand_near_patient_contact`) — 건네기·저항 밀기에서 손이 물체·손잡이를 잡는 순간. 손이 `hand_deep_stop_m` 보다 깊이 들어오거나 손 메시지가 끊겼으면 STOP. 치기·집기·놓기의 `contact_expected` 는 완화 대상이 아니다. 손·wrench·tcp_pose 를 받다가 `*_timeout_s` 넘게 끊기면 SLOW(`hand_stale`·`wrench_stale`·`tcp_stale`), 한 번도 못 받은 입력은 경고 로그만. `SafetyStatus.min_hand_distance` 는 손 없음 9.9, 거리 모름 NaN(rally_db 에는 null). 값은 모두 `safety.yaml`.

**HandState 손목 각도** (`wrist_angle_deg`): 0° 손바닥 아래 · 90° 옆(엄지 위) · 180° 위. **안정화된 값** — 최근 0.5 s 의 좋은 프레임(카메라에서 너무 가깝지 않음: 깊이가 있으면 손바닥 거리 ≥ 0.30 m, 손은 보이는데 손바닥 깊이가 비면 가까움으로 봄, 깊이가 아예 없으면 화면 속 손 크기로 대신) 중간값. 손이 너무 가까우면 직전 값을 유지(잡는 순간에도 다가올 때의 값), 손이 1 s 넘게 안 보이면 NaN. 0 은 "모름" 이 아니다 — 모르면 NaN. 중간값이라 손목을 돌리면 약 0.25 s 늦게 따라온다 (잡는 순간 기록용. 실시간 제어에 쓰려면 이 지연을 감안). 파라미터 `wrist_window_s`·`wrist_lost_s`·`wrist_near_m`·`wrist_max_hand_frac` (hand_pose).

**TTS 발화 중 마이크** (#45): `/rally/voice/speaking` 은 `Speak.block_mic=true` 발화가 시작되면 `true`, 끝나고 잔향 여유(약 0.3 s) 뒤 `false`. intent_node 는 `true` 동안 **일반 명령만 무시**하고 비상 키워드("그만/멈춰" → `is_emergency`)는 계속 듣는다 — 로봇이 말하는 동안에도 음성 정지가 살아 있어야 한다. 그래서 TTS 문구에는 "그만"·"멈춰"를 쓰지 않는다 (에코로 정지 오인식 방지).

**음성으로 훈련 고르기** (`VoiceIntent` 슬롯 약속, 메시지 변경 없음): "공 굴리기 하자"처럼 훈련 이름을 말하면 `intent: next_mode` + `slot_keys: [mode]`, `slot_values: ["1"]` (1 공 굴리기 · 3 캐치볼 · 5 건네고 받기 · 7 팔씨름). 세션 시작과 같이 말하면 `start_session` 에 같은 슬롯. 슬롯이 없으면 순서상 다음 모드. 모드 3 이 꺼져 있으면(`mode3_challenge.enabled: false`) brain 이 거절하고 로그·`SetMode` 응답 message 로 알린다 (TTS 안내는 Speak 연결 뒤). 같은 발화(`header.stamp` 같음)는 한 번만 처리한다.

### 2.3 판단·안전·기록 (커스텀)

| 토픽 | 타입 | 발행 | 구독 | 주기 | QoS |
|---|---|---|---|---|---|
| `/rally/safety/status` | `rally_interfaces/SafetyStatus` | rally_robot/safety_monitor | brain, dashboard, skill_server | 100 Hz (변화 시 즉시) | **Reliable, depth 10** |
| `/rally/safety/estop` | `std_msgs/Bool` | voice, 대시보드 버튼 (hand_pose 주먹 정지는 #59 에서 뺌) | safety_monitor | 이벤트 | **Reliable, TransientLocal** |
| `/rally/safety/contact_expected` | `std_msgs/Bool` | rally_robot/skill_server | safety_monitor | 접촉 예정 구간 동안 5 Hz 하트비트 (집기 하강 · 핸드오버 당김 대기 · 저항 밀기) | Reliable, depth 1 — 0.5 s 안에 갱신 없으면 꺼진 것으로 봄 |
| `/rally/safety/patient_contact` | `std_msgs/Bool` | rally_robot/skill_server | safety_monitor | 환자가 물체·손잡이를 잡는 구간 동안 5 Hz 하트비트 (건네기 당김 대기 → 후퇴, 저항 밀기). 이 동안 스킬은 환자 쪽(+x) 명령 이동 금지 | Reliable, depth 1 — 0.5 s 안에 갱신 없으면 꺼진 것으로 봄 |
| `/rally/brain/context` | `rally_interfaces/SceneContext` | rally_brain/context_builder | llm_advisor, dashboard | 5 Hz | BestEffort |
| `/rally/brain/trial_result` | `rally_interfaces/TrialResult` | rally_brain/bt_runner | rally_db/recorder, dashboard | 이벤트 | Reliable |
| `/rally/brain/state` | `std_msgs/String` (BT 현재 노드) | rally_brain/bt_runner | dashboard | 2 Hz | Reliable |

QoS 규칙: 영상·관절·힘 같은 **스트림은 BestEffort + depth 1**(최신 것만), **정지·안전·결과는 Reliable**. TransientLocal은 "늦게 켜진 노드도 마지막 상태를 알아야 하는" 상태 토픽에만 쓴다: `/rally/safety/estop`, `/rally/voice/speaking` (#45).

## 3. 서비스

| 서비스 | 타입 | 서버 | 설명 |
|---|---|---|---|
| `/rally/brain/set_mode` | `rally_interfaces/SetMode` | rally_brain | 모드 전환 요청 (대시보드·음성). 바로 답하고, 실제 전환은 지금 시도가 끝난 뒤(시도 사이·손 대기 중). 안전 STOP 중엔 거절. 대기 중 요청은 새 요청이 덮어씀. 마지막 모드에서 "다음" = 세션 종료 (message 에 표시) |
| `/rally/vision/run_calibration` | `rally_interfaces/RunCalibration` | rally_vision/calibration | eye-in-hand 캘리브레이션 (로봇 자세 순회 포함) |
| `/rally/llm/suggest_params` | `rally_interfaces/SuggestParams` | rally_brain/llm_advisor | 다음 시도 파라미터 제안. 응답은 최대 timeout_s(3 s) 뒤. 조절할 수 있는 값과 범위는 `skills.yaml llm_tunable` 이 정한다 (요청 범위는 그 안으로 좁히기만). `success=true` = LLM 값, `success=false` + `param_values` 있음 = LLM 실패로 대체값(마지막 값) — 둘 다 범위 안이라 그대로 써도 된다. `success=false` + `param_values` 비어 있음 = 요청 오류(llm_tunable 에 없는 값·범위 안 겹침) → 호출자가 규칙 값을 쓴다. `rationale` 은 80자 이내, 비상어가 들어 있으면 빈 문자열 |
| `/rally/voice/speak` | `rally_interfaces/Speak` | rally_voice/tts_node | TTS 발화, 발화 중 마이크 차단 |
| `/rally/gripper/command` | `rally_interfaces/GripperCommand` | rally_robot/gripper_node | RG2 폭·힘 명령 |
| `/rally/robot/go_observe_pose` | `std_srvs/Trigger` | rally_robot/skill_server | 관측 자세로 이동 (카메라가 테이블 전체를 보는 자세) |
| `/rally/robot/go_hand_view` | `std_srvs/Trigger` | rally_robot/skill_server | 손 보기 자세로 이동 — 툴을 환자 쪽으로 기울여 그리퍼 위 RealSense 가 환자 손을 본다 (모드 1 시도 전 레인 고르기). `skills.yaml mode1.hand_view_*`. 이미 그 자세면 바로 성공 |
| `/rally/robot/reset_safety` | `std_srvs/Trigger` | rally_robot/safety_monitor | 정지 해제 (운영자만) |
| `/rally/db/start_session` | `std_srvs/Trigger` → 추후 커스텀 | rally_db | 세션 레코드 생성 |

## 4. 액션 (스킬)

모든 스킬은 `rally_robot/skill_server`가 제공하고 `rally_brain/bt_runner`가 호출한다. 진행률 피드백과 취소를 지원하며, `SafetyStatus.level == STOP`이면 서버가 즉시 abort 한다. 안전 상태를 한 번도 못 받았거나 `status_timeout_s`(0.5 s, `safety.yaml rally_skill_server`) 넘게 끊겨도 STOP 으로 보고 goal 거부·실행 중 abort(`safety_status_stale`, #14) — safety_monitor 없이 `skill_server --fake` 만 띄우면 goal 이 거부된다.

| 액션 | 타입 | 모드 | 핵심 로봇 기능 |
|---|---|---|---|
| `/rally/skill/push_ball` | `rally_interfaces/PushBall` | 1 | 거치대 공 비전 집기 → 레인 입구 → 퍼터식 치기 → **손 보기 자세에서 끝남**(`mode1.end_at_hand_view`, 환자가 받는 모습). 훈련 Brain은 `target` + 계산한 양수 `push_speed` 전달. 속도 0의 로봇 계산 경로는 단독 시험용 |
| `/rally/skill/return_ball` | `rally_interfaces/ReturnBall` | 1 | 시도 뒤 레인 안 공을 비전으로 찾아 집고 거치대 맨 윗부분(`mode1.rack_drop_posx`, 직접교시)에 넣기 → 손 보기 자세. 로봇이 닿는 곳(중앙선 앞)에 올 때까지 기다림 (feedback `out_of_reach` → brain 이 안내) |
| `/rally/skill/pick_compliant` | `rally_interfaces/PickCompliant` | 1·3·5 공통 | task_compliance_ctrl(Z 저강성), 접촉 감지 후 파지 |
| `/rally/skill/catch_ball` | `rally_interfaces/CatchBall` | 3 (챌린지) | 예측 착지점으로 amovel 선행 이동 + servol 보정, 패드 면 정렬 |
| `/rally/skill/move_target` | `rally_interfaces/MoveTarget` | 3 (Phase 2) | movec / movesx / moveb 궤적, 환자가 맞혀야 하는 후기 단계 |
| `/rally/skill/detach_ball` | `rally_interfaces/DetachBall` | 3 | 힘 제어 당김, 힘 급락으로 떼어짐 판정 |
| `/rally/skill/handover` | `rally_interfaces/Handover` | 5 | 기울여 제시, 당김 힘 감지 해제, servol 손 추종 |
| `/rally/skill/resist_push` | `rally_interfaces/ResistPush` | 7 | 순응 강성으로 버티기, 목표 힘 유지 시 져주기(밀려남), 저속 복귀. 로봇은 절대 밀지 않음 |
| `/rally/session/run` | `rally_interfaces/RunSession` | 전체 | brain 최상위 action |

## 4.1 모드 3 요격 타이밍

릴리스 검출(손·공 마스크 분리, 웹캠 60 Hz) → 3~5 프레임 후 첫 착지 예측 발행 → `catch_ball`이 `lead_time_s`만큼 앞선 시각의 예측점으로 amovel → 매 프레임 갱신되는 예측으로 servol 보정 → 착지 → 패드 위 공 판정. 공 비행 시간이 보통 0.5~0.8 s(살살 토스)이므로 카메라→로봇 전체 지연을 100 ms 안쪽으로 맞추는 것이 핵심(`scripts/measure_latency.py`).

## 5. 기동 순서 (준비 확인 방식)

노드를 ROS 2 managed(Lifecycle) 노드로 바꾸지 않고, **앞 단계가 실제로 준비됐는지 확인한 뒤 다음 단계를 띄운다.** "준비"의 정의는 `rally_bringup/config/readiness.yaml` 한 곳에 그룹으로 둔다 (토픽: `max_age_s` 안에 메시지 수신 / 서비스·action: 서버 응답 가능).

```
0. 서버 PC      compose pc3: discovery · db · redis (healthcheck)        ← 모든 노드가 discovery 에 의존
1. 로봇·인식 PC source scripts/env_robot_pc.sh → dsr_bringup2 → pc1_robot.launch
                  safety_monitor → [wait_ready --group safety] → force_publisher · gripper_node · skill_server
                  확인 실패 시 로봇 노드를 켜지 않고 launch 종료
2. 로봇·인식 PC compose pc2: vision · hand_pose · voice (pc2_perception.launch part:=...)
3. 서버 PC      compose pc3: brain (pc3_brain.launch) · dashboard
                  bt_runner 가 session 그룹(robot + perception + voice)을 1 s 마다 확인 → 준비 안 되면 세션을 시작하지 않음
```

| 도구 | 쓰임 |
|---|---|
| `rally_bringup/readiness.py` `ReadinessChecker` | 노드 안에서 그룹 준비 여부 확인 (bt_runner) |
| `ros2 run rally_bringup wait_ready --group <g> [--timeout s]` | launch 단계 관문. 준비되면 0, 시간 초과면 1 |
| `ros2 run rally_bringup wait_ready --group session --once` / `ros2 launch rally_bringup full_system.launch.py` | 시연 전 점검 |

세션 도중 준비가 깨지면(토픽 끊김·서버 사라짐) brain 은 `RunSession` 을 abort 하고 robot 은 관측 자세에서 대기한다 (Day 2 bt_runner 구현).

## 6. 파라미터 파일 (rally_bringup/config)

- `hand_eye.yaml` — `tcp_T_camera_optical`(TCP ← camera_color_optical_frame, xyz m + 쿼터니언), `depth_scale`, `tcp_offset_from_flange_m`. 10/8 ArUco 터치 재맞춤 값(`scripts/fit_hand_eye.py`, 평균 6 mm·최대 9 mm). 재검증은 `scripts/aruco_detect.py --json` → `scripts/verify_calibration.py --depth-scale 0.971`
- `table.yaml` — table_link 원점, 중앙선 위치, 작업 공간 한계(x,y,z min/max)
- `safety.yaml` — 손 근접 감속 거리(기본 0.25 m → 50 %, 0.15 m → 정지), 힘 한계(기본 25 N, 항상 적용), nudge 임계(한 주기 |Δ|F|| 8 N — 로봇 정지 중이고 `/rally/safety/contact_expected` 가 아닐 때만 판정), 작업 공간(x·y·z min/max → 밖이면 STOP)
- `skills.yaml` — 관측 자세, 모드별 기본 파라미터와 허용 범위(LLM 클램핑 범위와 동일), `gripper.<class>` 열림 폭·힘, `pick.*`, `handover.*`(제시 자세·당김 임계)
- `table.yaml` — `table.z_m` 테이블 면 높이: 집기 Z 는 비전 깊이가 아니라 이 평면 + 물체 반지름으로 계산
- `llm.yaml` — 모델, 타임아웃(3 s), 프롬프트 템플릿 경로, 전송 필드 화이트리스트(익명화)

## 모드 1 Brain 목표·속도 계약 (#81 · #84 · #85 · #87)

- 입력: 최신 `SceneContext.hand`의 `HandState` (검출됨, `base_link`, 유한한 손바닥 좌표,
  원본 stamp가 현재 ROS 시각 기준 0~1 s 이내). 미검출·zero/미래/오래된 stamp·좌표 오류는 제한 시간 대기 후 `no_hand`로 기록하고 스킬을 보내지 않는다.
- 중앙선은 `table.yaml table.centerline_x_m` 한 곳. 레인별 입구·방향·실측 `recipe`는 `skills.yaml mode1.lanes`.
  recipe에 `decel_mps2`, `d0_m`, `rmse_m`, `distance_range_m`, `speed_range_mps`를 사용한다.
- 난이도는 개발용 `reach_m = (max(level, 0) + 1) × mode1.reach_rmse_multiplier × rmse_m`.
  목표는 레인별 target_d_m(개발용 0.35 m)으로 고정하며 중앙선 + max(mode1.patient_margin_m, safety.stop_distance_m + 0.05), 실측 거리·속도 범위와 로봇·Brain 속도 제한의 교집합으로 제한한다.
  Brain 속도 상한은 0.37 m/s. 단일 목표를 정한 뒤 `v = sqrt(2a(d-d0))`를 계산한다. 교집합이 없거나 레시피가 유효하지 않으면 goal 없이 실패 사유 기록.
- `/rally/skill/push_ball`에 **target(base_link) + 양수 push_speed**를 함께 보낸다. Brain이 속도를 계산하고 로봇은 최종 클램프·안전 감시.
  `push_speed=0`으로 로봇이 계산하는 경로는 단독 시험용이며 본 훈련 트리에서 사용하지 않는다.
- `TrialResult.param_keys`는 기존 `lane`, `push_speed` 뒤에 `target_d_m`, `level`, `reach_requested_m`,
  `reach_actual_m`, `target_x_m`, `target_y_m`, `hand_start_d_m` 추가. push_speed는 실제 전달한 명령값이며 실측 공 속도가 아니다.
  측정되지 않은 `error_mm` 및 `stop_x_m`/`stop_y_m`은 NaN. DB는 기존 유한값 정리 경로를 사용.
- `TrialResult.success`는 타격 후 새 손 표본의 펼침→쥠 유지→펼침을 관측한 받기 후보 판정. 실제 공 접촉 증거는 아니며 실물 검증 전 자동 난이도 변경에서 제외한다.
  `catch_gesture_detected`, `ball_returned`, `lane_by_hand`를 metric에 추가한다. 명시적 harder/easier는 시도 경계에서 반영하며 단계 0 아래로 낮추지 않는다. 온라인 레시피 갱신은 후속 연결 사항.
- 손에 가까운 레인을 선택한다. 받기 관측 후 손 이탈은 최신 유효 손 거리 ≥ safety.slow_distance_m일 때만 인정한다.
  `hand_clear_wait_s` 이후 안내하고 계속 대기한다. 미검출·시간 초과만으로 회수하거나 세션을 종료하지 않는다.
  뒤늦게 이탈이 확인되면 ReturnBall을 보낸다. 대기 중 취소·STOP은 기존대로 중단하며 자동 재개하지 않는다.
- 기존 ROS 메시지·토픽·서비스 이름과 STOP/reset 계약은 변경하지 않는다. 두 PC의 시계·레인·레시피 설정을 일치시켜야 한다.

## 7. 세션 휴식·재개와 대시보드 제어 (#78)

- 기존 `/rally/voice/intent`의 `pause` 또는 `tired` → 현재 시도를 마친 뒤 휴식. `resume` → 휴식 중인 세션만 재개. 음성 계약 필드 변경 없음.
- 시도 경계는 모드 1의 타격·받기 관측·손 이탈 대기·회수·기록, 모드 5의 집기·GIVE·RECEIVE를 모두 포함한다. 손 이탈 대기는 시간 초과로 끝내지 않으며 이때 휴식 요청도 유지한다. 휴식 대기 중에는 현재 시도를 계속 처리하고, 다음 시도 앞의 BT 관문에서 대기한다. 마지막 시도면 휴식 대신 세션을 종료한다.
- 휴식 중 모드 전환·난이도 변경은 거절한다. 휴식 요청 시 대기하던 모드/난이도 요청은 비운다. 재개는 같은 난이도로 다음 시도부터 진행한다. 레시피 없는 모드의 난이도 변경은 안내만 한다 (모드 1 명시적 변경·모드 7 지원).
- STOP·context 끊김·세션 취소는 휴식 중에도 세션을 종료한다. STOP 해제 서비스는 기존 그대로이며 재개로 해제하지 않는다. 준비 조건 미충족이면 재개를 거절한다.
- `/rally/brain/state`는 기존 BT 노드 이름을 유지하되 휴식 요청 후 `pause_pending`, 실제 휴식 중 `paused`를 발행한다. 상세 상태는 아래 Redis 상태에 제공한다.

대시보드는 ROS를 import하지 않는다. brain의 별도 I/O 콜백이 기존 Redis로 일시적 요청·응답을 주고받는다. `REDIS_URL`이 있으면 활성화되며 `dashboard_control` 노드 파라미터로 끌 수 있다. Redis 장애는 음성·안전 감시를 막지 않는다.

| 경로 | 계약 |
|---|---|
| Redis `rally:session_control` | brain이 0.2 s마다 갱신, TTL 2 s. `session_id`, `phase`(idle/running/pause_pending/paused/ended), `pause_id`, `reason`, `mode`, `difficulty`, `trial_index`, `can_resume`, `blocked_reason`, `updated_at`(Unix s), `bt_node`(현재 BT 실행 노드명), `mode3_enabled`(챌린지 설정) |
| Redis pub/sub `rally:session_commands` | 재개 요청 JSON: `request_id`, `session_id`, `pause_id`(모두 UUID 문자열), `command: resume`, `expires_at`(Unix s, 최대 3 s). 저장·재전송하지 않음 |
| Redis `rally:session_reply:<request_id>` | brain 응답 `success`, `message`, `request_id`. TTL 5 s. 세션·휴식 식별값을 확인한 뒤 현재 안전·준비 조건으로 재검증. 중복 요청은 이미 소비된 휴식 ID이므로 재실행 불가 |
| HTTP `GET /api/session` | 최신 Redis 제어 상태 또는 null (미수신·만료). Redis 연결 실패 503 |
| HTTP `POST /api/session/resume` | JSON `session_id`, `pause_id`. 서버가 요청 UUID·만료를 생성한다. brain 승인 시 200, 거절 409, 상태 없음/Redis/수신자 없음 503, 응답 미확인 504. timeout은 결과 미확인이므로 상태 새로고침 후 판단하고 자동 재시도하지 않음 |
| WebSocket `/ws` | 기존 메시지에 `type: session`, `data: 위 상태 또는 null` 추가. 0.5 s마다 전달. 브라우저도 2 s 이상 갱신이 없으면 버튼 비활성화 |

음성 재개는 발화 timestamp가 현재 휴식 시작 이후인 경우만 허용한다. zero stamp·오래된/중복 발화는 재개하지 않는다. 대시보드 요청은 화면에 표시된 세션·휴식 ID를 보내므로 이전 세션/휴식에서 누른 버튼은 거절된다. 안전 여부는 화면의 표시가 아니라 brain이 결정한다.

### 모드 1 손 시작 위치 대기 (PR #87 PM 정정·사용자 합의)

공 목표는 레인별 고정값을 실측·안전·속도 구간으로 제한한다. 손 위치나 난이도로 목표를 이동시키지 않는다.
난이도별 reach는 기존 RMSE×2 간격이며 손의 레인 축 투영 거리 ≥ 목표 거리 + reach일 때만 타격한다.
부족하거나 유효 손/NORMAL이 없으면 안내하며 대기한다. 대기 중에는 손 표본을 갱신하고 전송 직전에 조건을 재검증한다.
STOP·취소는 기존 세션 중단 처리. 타격 action 실패 시에도 다음 타격을 시작하지 않고 기록 후 종료하여 운영자가 공·그리퍼 상태를 확인한다.

## 테스트 휴식 제안 확인 — LLM·음성 응답 연결 전

- Brain의 `test_rest_proposals` 파라미터는 기본 false. true로 시작할 때만 `/rally/brain/test/propose_rest` (`std_srvs/Trigger`)를 제공한다. 격리 가짜 스킬 환경에서만 켠다. 최신 상황판·안전·준비 조건과 running 상태, 대기 중인 모드/난이도 요청 없음을 검사한다. 이미 pending인 제안을 덮어쓰지 않는다.
- 발화/화면 문구: “혹시 피곤하진 않으세요? 조금 쉬었다 할까요?”. 생성 시점부터 monotonic 20초이며 TTS 재생 완료를 기다리지 않는다. 응답 대기 중 기존 훈련은 계속한다. 현재 구현은 rest만 지원한다.
- Redis `rally:session_control.proposal`: null 또는 `proposal_id`(UUID), `action: rest`, `source: test`, `prompt`, `status`(pending/accepted/rejected/expired/invalidated), `remaining_s`. 기존 GET `/api/session`과 WS `type: session`에 포함한다.
- POST `/api/session/proposal/respond`: `{session_id, proposal_id, decision: accept|reject}`. ID는 UUID, 임의의 행동명 불허. API는 최신 상태를 확인하고 Redis에 `command: proposal_accept|proposal_reject`, `request_id`, `session_id`, `proposal_id`, `expires_at`을 보낸다. 요청 기한·reply 키·TTL·응답 대기는 resume과 동일하다. HTTP 200은 Brain 응답 후에만, 거절은 409, 연결/상태 없음 503, 결과 미확인 504. 자동 재전송하지 않는다.
- Brain은 같은 lock에서 현재 세션·모드·난이도·상태·제안 ID·monotonic 만료·안전·준비 상태를 다시 검사한다. 수락은 한 번 소비하고 기존 `_pause`로 적용 예약하여 현재 시도 전체 완료 후 휴식한다. 수락 후 응답 타이머로 예약을 취소하지 않는다. 마지막 시도라면 기존대로 세션 종료다.
- 거절은 즉시 rejected, 무응답은 20초 뒤 expired. 둘 다 훈련을 바꾸지 않는다. STOP/취소/상황판 상실/세션 종료/모드·난이도 변경/직접 휴식 요청은 pending을 무효화한다. 종료된 제안은 다시 수락할 수 없다. #98 손 근접 STOP 예외는 구현하지 않는다.
- 화면은 응답 결과(성공·거절·결과 미확인)를 같은 제안의 상태 갱신·재접속 뒤에도 보존한다. 새 세션 또는 새 제안 ID를 받으면 이전 안내를 초기화하며, 이전 세션·제안의 늦은 응답은 새 안내를 덮지 않는다. 최신 상태가 없으면 응답 버튼을 비활성화하고 자동 재전송하지 않는다.
- 테스트 제안 생성은 안전 판단이나 LLM 호출이 아니다. `tired` 직접 휴식의 기존 동작은 아직 변경하지 않으며, 실제 음성 분류·제안 응답 연결 전 상태다. 임의의 “네/아니요”나 resume은 제안 응답으로 사용하지 않는다.
