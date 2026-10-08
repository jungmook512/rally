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
webcam_link          측면 웹캠 (table_link 기준 static)
```

- `camera_link`는 `rally_vision/camera_tf`(`ros2 run rally_vision camera_tf`)가 `hand_eye.yaml`을 읽어 올린다. 기본 `mode: tcp_pose`는 `/rally/robot/tcp_pose`(10 Hz)를 받아 `base_link → camera_link`를 발행(두산 URDF tf 불필요, segmenter의 `tcp_npy` 경로와 같은 값). `mode: static`은 `link_6 → camera_link`를 static으로 한 번(플랜지→TCP 0.228 m 포함, 두산 robot_state_publisher가 떠 있을 때). tcp_pose 모드는 마지막 변환을 `republish_hz`(30 Hz)로 현재 시각에 다시 발행(hold-last)해 영상 stamp 조회가 되게 한다 — 값은 최근 tcp_pose(≤ 0.1 s 전)라 로봇 정지 중에만 정확. 기본은 RealSense의 `camera_link → … → optical` static tf를 기다렸다가 `camera_link`를 자식으로 발행한다. RealSense가 tf를 안 내는 환경(`publish_tf:=false`)에서만 `optical_fallback: true`로 optical 프레임을 직접 발행한다(기본 false — 폴백 중 optical을 받은 노드는 RealSense가 늦게 켜져도 조회가 계속 실패하므로). `RunCalibration` 서비스는 아직 없음(Day 7 여유 시).
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

**objects 좌표 프레임** (#66): `/rally/perception/objects` 의 `header.frame_id` 가 `base_link` 가 아니면(카메라 프레임 `camera_color_optical_frame`) 카메라→base 변환을 못 구한 것이다 — 그 좌표로 로봇을 움직이지 않는다(abort 또는 대기). segmenter 는 파라미터 `transform_source` 로 변환을 고른다: `tcp_npy`(기본, `/rally/robot/tcp_pose` × `T_gripper2camera.npy`) · `tf`(tf2 만) · `auto`(tf 우선). 검증된 `camera_link` static tf 를 올리기 전에는 URDF 설계값 tf 가 캘리브레이션값을 이기므로 기본을 `tcp_npy` 로 둔다 (10/7 실측 치우침 약 1 cm, PR #42). `tcp_npy` 는 영상 시각이 아니라 마지막 `tcp_pose`(10 Hz) 를 쓰므로 **로봇이 멈춰 있을 때의 좌표만 정확**하다 — 이동 중 좌표로 로봇을 보내지 않는다. `camera_tf` 가 떠 있으면 `auto`(tf 는 영상 시각으로 보간) 가 낫다.

**HandState 거리 NaN** (#44): `detected=true` 인데 손 위치를 못 구하면(정렬 깊이 없음·컬러와 시각이 맞는 깊이 없음·`base_link` tf 없음) `distance_to_centerline = NaN` = "손은 보이는데 거리 모름". 가짜 거리를 만들지 않는다. safety_monitor 는 `detected && NaN` 이면 SLOW. tf 가 없을 때는 `header.frame_id` 가 카메라 프레임.

**TTS 발화 중 마이크** (#45): `/rally/voice/speaking` 은 `Speak.block_mic=true` 발화가 시작되면 `true`, 끝나고 잔향 여유(약 0.3 s) 뒤 `false`. intent_node 는 `true` 동안 **일반 명령만 무시**하고 비상 키워드("그만/멈춰" → `is_emergency`)는 계속 듣는다 — 로봇이 말하는 동안에도 음성 정지가 살아 있어야 한다. 그래서 TTS 문구에는 "그만"·"멈춰"를 쓰지 않는다 (에코로 정지 오인식 방지).

**음성으로 훈련 고르기** (`VoiceIntent` 슬롯 약속, 메시지 변경 없음): "공 굴리기 하자"처럼 훈련 이름을 말하면 `intent: next_mode` + `slot_keys: [mode]`, `slot_values: ["1"]` (1 공 굴리기 · 3 캐치볼 · 5 건네고 받기 · 7 팔씨름). 세션 시작과 같이 말하면 `start_session` 에 같은 슬롯. 슬롯이 없으면 순서상 다음 모드. 모드 3 이 꺼져 있으면(`mode3_challenge.enabled: false`) brain 이 거절하고 로그·`SetMode` 응답 message 로 알린다 (TTS 안내는 Speak 연결 뒤). 같은 발화(`header.stamp` 같음)는 한 번만 처리한다.

### 2.3 판단·안전·기록 (커스텀)

| 토픽 | 타입 | 발행 | 구독 | 주기 | QoS |
|---|---|---|---|---|---|
| `/rally/safety/status` | `rally_interfaces/SafetyStatus` | rally_robot/safety_monitor | brain, dashboard, skill_server | 100 Hz (변화 시 즉시) | **Reliable, depth 10** |
| `/rally/safety/estop` | `std_msgs/Bool` | voice, hand_pose, 대시보드 버튼 | safety_monitor | 이벤트 | **Reliable, TransientLocal** |
| `/rally/safety/contact_expected` | `std_msgs/Bool` | rally_robot/skill_server | safety_monitor | 접촉 예정 구간 동안 5 Hz 하트비트 (집기 하강 · 핸드오버 당김 대기 · 저항 밀기) | Reliable, depth 1 — 0.5 s 안에 갱신 없으면 꺼진 것으로 봄 |
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
| `/rally/robot/reset_safety` | `std_srvs/Trigger` | rally_robot/safety_monitor | 정지 해제 (운영자만) |
| `/rally/db/start_session` | `std_srvs/Trigger` → 추후 커스텀 | rally_db | 세션 레코드 생성 |

## 4. 액션 (스킬)

모든 스킬은 `rally_robot/skill_server`가 제공하고 `rally_brain/bt_runner`가 호출한다. 진행률 피드백과 취소를 지원하며, `SafetyStatus.level == STOP`이면 서버가 즉시 abort 한다.

| 액션 | 타입 | 모드 | 핵심 로봇 기능 |
|---|---|---|---|
| `/rally/skill/push_ball` | `rally_interfaces/PushBall` | 1 | movel 속도 프로파일, check_force_condition(접촉) |
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
