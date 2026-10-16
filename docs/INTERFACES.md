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
| `/camera/color/image_raw` | `sensor_msgs/Image` | realsense2_camera (손목, 시리얼 `cameras.yaml`) | 60 Hz (10/12~, 848×480) | SensorData |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | realsense2_camera (손목, 시리얼 `cameras.yaml`) | 60 Hz (10/12~, 848×480) | SensorData |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | realsense2_camera (손목, 시리얼 `cameras.yaml`) | 60 Hz (10/12~, 848×480) | SensorData |
| `/webcam/image_raw` | `sensor_msgs/Image` | usb_cam (측면) | 60 Hz | SensorData |
| `/side/side/color/image_raw`, `/side/side/color/camera_info` | `sensor_msgs/Image`, `CameraInfo` | realsense2_camera (모드 3 측면 RealSense D435i, `camera_namespace:=side camera_name:=side`, 컬러 848×480 @60, 노출 20 · gain 128) | 60 Hz | SensorData |
| `/rally/robot/wrench` | `geometry_msgs/WrenchStamped` | rally_robot/force_publisher | 50 Hz (두산 서비스 폴링 — 100 Hz 면 컨트롤러 서비스가 막힘) | SensorData |
| `/rally/robot/tcp_pose` | `geometry_msgs/PoseStamped` | rally_robot/force_publisher | 10 Hz | SensorData |
| `/rally/gripper/state` | `sensor_msgs/JointState` — name `[rg2_width_mm, rg2_grip, rg2_target_mm]`, position `[폭 mm, grip 감지 0/1, 마지막 명령 폭 mm(-1 = 없음)]`, effort 미사용. 쥠 판단 = grip 비트 **또는** 명령 폭보다 5 mm 이상 벌어져 멈춤(공을 손가락 끝에 걸치면 grip 비트 0) | rally_robot/gripper_node | 20 Hz | SensorData |
| `/joint_states` | `sensor_msgs/JointState` | dsr_bringup2 | 100 Hz | SensorData |
| `/dsr01/joint_states` | `sensor_msgs/JointState` (팔 6축 rad — 두산 컨트롤러가 직접 냄; `/joint_states` 는 joint_state_publisher 가 0 으로 채울 수 있음) | dsr_bringup2 | 100 Hz | SensorData — safety_monitor(움직임)·camera_tf(건강)·**brain bt_runner**(모드 3 투구 안내: `ready_posj` ± `throw_prompt_pose_tol_deg` 안에 `throw_prompt_settle_s` 머물면, 10/16. 파라미터 `joint_states_topic`) |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 여러 노드 | - | 표준 |

- **손목 인식 모델은 모드마다 (10/15 정정인)**: segmenter 파라미터 `mode_models`(기본 `1:yolov8n-seg_ball.pt;3:velcro_ball_yolo11n-seg_1024_rot_v2.pt;5:yolov8n-seg_cupstick.pt;7:`) 중 `active_mode` 에 해당하는 모델만 돌린다 (시작 때 모두 읽어 둠). brain(`segmenter_mode.py`)이 모드에 들어갈 때 `/rally_segmenter` 의 `active_mode` 를 바꾸고 세션이 끝나면 0. 0 = 표 밖 → `model_path`/기본 찾기(공 + 컵·막대). 7 = 모델 없음(빈 objects). 공 떼기 시험 도구 `scripts/detach_loop.py` 는 brain 없이 3 으로 바꿨다 되돌린다.
- **RealSense 두 대 (10/12, `rally_bringup/config/cameras.yaml`)**: `pc2_perception.launch.py part:=vision` 이 시리얼로 구분해 둘 다 848×480 @60 으로 띄운다 (`side_camera:=false` 면 측면 안 띄움). 노출: 측면은 항상 고정(20 · gain 128, `set_camera_params` 가 뜬 뒤 넣음), **손목은 평소 자동이고 모드 3 동안만 고정** — brain(`camera_exposure.py`)이 모드 3 에 들어갈 때·나올 때·세션이 끝날 때 손목 드라이버 노드(`/camera`)의 `rgb_camera.enable_auto_exposure`·`exposure`·`gain` 파라미터를 바꾼다. brain 이 켜질 때도 자동 노출을 한 번 보낸다(모드 3 도중 재시작 대비). 모드 3 은 고정이 적용된 뒤(최대 `skills.yaml mode3.exposure_wait_s` 3 s) 투구를 안내한다. 모드 3 동안은 노출이 짧아 손 검출이 약해질 수 있다. 노출·gain 은 드라이버 launch 인자로 받지 않아(4.58.4) 파라미터로만 넣는다.

### 2.2 인식 (커스텀)

| 토픽 | 타입 | 발행 | 구독 | 주기 | QoS |
|---|---|---|---|---|---|
| `/rally/perception/objects` | `rally_interfaces/TrackedObjectArray` | rally_vision/segmenter (ByteTrack tracker 전까지) | brain, robot | 30 Hz (웹캠 비행 추적은 60 Hz) | BestEffort, depth 1 |
| `/rally/perception/landing` | `geometry_msgs/PointStamped` — 패드 평면 교차 예측점 (`base_link`). `header.stamp` = 마지막 관측의 센서 시각(손목 카메라 시계) | rally_vision/flight_predictor | robot(catch_ball) | 60 Hz, 비행 중만 | BestEffort, depth 1 |
| `/rally/perception/landing_info` | `std_msgs/Float64MultiArray` — `[교차 시각 s(ROS 시각), 교차까지 남은 시간 s(landing stamp 기준), 평면 위 표준편차 u·w m, 관측 수, 릴리스 이후 s]`. landing 과 같은 순서로 하나씩 발행 (전용 msg 는 팀 합의 후) | rally_vision/flight_predictor | robot(catch_ball) | landing 과 같음 | BestEffort, depth 1 |
| `/rally/perception/ball_on_pad` | `geometry_msgs/PointStamped` — 측면 카메라가 **패드 중심(실시간 TCP + `pad_offset_tool_m`) 주변 10 cm 안**에서 본 공의 3D (`base_link`, 패드 평면과 교차). 보일 때만 측면 프레임마다. `header.stamp` = 측면 시각 + dt(손목 시계) | rally_vision/flight_predictor | robot(catch_ball: `pad_hold_s` 1 s 연속이면 잡음) | ≤ 60 Hz, 보일 때만 | BestEffort, depth 1 |
| `/rally/perception/objects/debug_image` | `sensor_msgs/Image` | rally_vision/segmenter (ByteTrack tracker 전까지) | dashboard | 10 Hz | BestEffort |
| `/rally/perception/side/debug_image` | `sensor_msgs/Image` (bgr8) — 측면 프레임에 YOLO 후보 마스크(파란색)·상자·신뢰도, 고른(움직이는) 후보는 굵게 + `moving`, 패드 위 공 판정 원(판정되면 초록 + `ball on pad`) | rally_vision/flight_predictor | dashboard (모드 3 측면 칸, 안 오면 원본 `/side/side/color/image_raw`) | ≤ `debug_image_hz`(10 Hz), 구독자 있을 때만 (10/14) | BestEffort |
| `/rally/perception/objects/image_axis` | `geometry_msgs/PoseArray` | rally_vision/segmenter | robot(handover 받기) | objects 와 같이 (같은 header·순서) | BestEffort, depth 1 |
| `/rally/perception/hand` | `rally_interfaces/HandState` | rally_human/hand_pose_node | brain, robot(safety), robot(handover) | 30 Hz | BestEffort, depth 1 |
| `/rally/perception/body` | `rally_interfaces/BodyPose` | rally_human/body_pose_node | brain | 15 Hz | BestEffort |
| `/rally/perception/body/debug_image` | `sensor_msgs/Image` (bgr8) | rally_human/body_pose | dashboard('몸 자세' 칸, web_video_server 경유) | 관절 화면 — 입력 영상 + 뼈대 + 몸통 기울기·어깨 들림·팔꿈치 펴짐·보상 판정 (10/15). 구독자가 있을 때만 `debug_hz` 10 Hz | BestEffort |
| `/rally/voice/intent` | `rally_interfaces/VoiceIntent` | rally_voice/intent_node | brain, safety | 이벤트 | Reliable |
| `/rally/voice/transcript` | `std_msgs/String` | rally_voice/intent_node | dashboard | 이벤트 | Reliable |
| `/rally/voice/speaking` | `std_msgs/Bool` | rally_voice/tts_node | rally_voice/intent_node, rally_brain/bt_runner (말하는 중 자세 교정 안내 건너뜀, #119), dashboard 선택 | 발화 시작·끝 이벤트 | **Reliable, TransientLocal**, depth 1 |
| `/rally/voice/listening` | `std_msgs/Bool` | rally_voice/intent_node | rally_voice/tts_node (true 동안 발화를 미룸, 최대 `listen_hold_max_s` 15 s), rally_brain/bt_runner (모드 선택 안내 건너뜀) | 호출어 감지 → 녹음 → STT → 의도 발행 끝 (10/15) | Reliable, depth 1 (TransientLocal 아님 — 시작 때 false 한 번) |

**objects/image_axis** (10/12, 모드 5 받기 기울기): `poses[i]` 는 `objects[i]` 와 짝 (header.stamp 같을 때만 짝지음). `position` = 그 물체 중심(objects 와 같음), `orientation` = 화면 속 마스크 주축 각도를 z 축 회전으로 — 화면 위쪽 0°, 오른쪽으로 기울면 +, −90~90°. 둥근 물체(장단축 비 ≤ 1.3)는 (0,0,0,0). `TrackedObject.yaw` 는 base 평면에 투영한 값이라 카메라가 앞을 보는 받기 자세에서는 늘 ±90° 로 뭉개져서 따로 낸다. 메시지 형식을 안 바꾸려고 TrackedObject 필드 대신 별도 토픽 (정정인 결정, 비전 컨테이너 재빌드 없이).

**objects 좌표 프레임** (#66): `/rally/perception/objects` 의 `header.frame_id` 가 `base_link` 가 아니면(카메라 프레임 `camera_color_optical_frame`) 카메라→base 변환을 못 구한 것이다 — 그 좌표로 로봇을 움직이지 않는다(abort 또는 대기). segmenter 는 파라미터 `transform_source` 로 변환을 고른다: `tcp_npy`(기본, `/rally/robot/tcp_pose` × hand_eye) · `tf` · `auto`(tf 우선). **로봇 PC bringup URDF(`m0609_with_rg2_camera`)가 `tool0 → bracket_link → camera_bottom_screw_frame → camera_link → … → camera_color_optical_frame` 을 설계값으로 발행**(10/8 정정인 view_frames)하므로, 그 체인이 있는 동안 `tf`/`auto` 는 설계값을 집어 1 cm·2.5° 치우친다(PR #42 원인). URDF 에서 카메라 체인을 빼고 `camera_tf` 가 유일한 `camera_link` 부모가 되면 기본을 `auto` 로 바꾼다. `tcp_npy` 는 영상 시각이 아니라 마지막 `tcp_pose`(10 Hz) 를 쓰므로 **로봇이 멈춰 있을 때의 좌표만 정확**하다 — 이동 중 좌표로 로봇을 보내지 않는다. `camera_tf` 가 떠 있으면 `auto`(tf 는 영상 시각으로 보간) 가 낫다.

**BodyPose** (`rally_human/body_pose`, 측면 대각 웹캠(높이 약 2 m) MediaPipe Pose, 기획 FR-05 — 모드 1·5·7, 선 자세): 팔을 뻗었을 때의 보상 동작만 잰다. `elbow_extension_deg` 는 뻗는 팔(`arm`, auto = 카메라 쪽) 어깨–팔꿈치–손목 각이고 **140° 이상이면 '뻗음'**. `trunk_lean_deg`·`shoulder_elevation_deg` 는 **뻗었고 기준이 준비됐을 때만 숫자**(0.5 s 중간값), 아니면 NaN — 숫자 = 기준 준비 + 이번 프레임 유효 관측. `reset_baseline` 을 부른 뒤 숫자가 다시 나오기 시작하면 준비 완료(서비스 응답은 접수일 뿐). 어깨 들림은 그 팔의 기준이 있어야 숫자 — 기준은 기준 잡는 동안 표본이 절반(15 프레임) 이상 모인 팔만 생기고, 그 뒤엔 채우지 않으므로 **훈련 팔을 바꾸면 `reset_baseline` 을 다시 부른다**. 뻗는 팔이 바뀌면 중간값 표본을 새로 모은다. 기준 다시 잡기 직후 `compensation_type` 은 바로 `none` (굽힌 팔이 어깨·몸통을 가려 관절이 틀어지므로, 10/8 실측). 정확한 팔꿈치 각도는 보장하지 않는다(뻗음 판단용). `trunk_lean_deg` = 평소 자세 대비 몸통 방향이 바뀐 각(엉덩이→어깨 방향, 부호 없음 — 앞으로 숙이기가 주이지만 옆·뒤 기울임도 같이 잡힘, 카메라 각도 무관), `shoulder_elevation_deg` = 평소 대비 뻗는 쪽 어깨 으쓱(양 귀 가운데 기준, 고개 기울임 무관). 평소 자세 = 노드 시작 후, 또는 `/rally/perception/body/reset_baseline` 뒤 약 2 s (바르게 선 상태). 기준 전엔 NaN. `compensation_type` 은 **뻗은 동안** 같은 종류가 5 프레임 연속일 때만: `trunk_lean`(≥ 20°) > `shoulder_hike`(≥ 20°) (10/8 선 자세 실측으로 15·10 → 20·20) > `none`, 안 뻗었거나 사람을 놓치면 바로 `none`. 사람 = 뻗는 쪽 어깨가 보임 (먼 쪽은 가려져도 됨).

**HandState 거리 NaN** (#44): `detected=true` 인데 손 위치를 못 구하면(정렬 깊이 없음·컬러와 시각이 맞는 깊이 없음·`base_link` tf 없음) `distance_to_centerline = NaN` = "손은 보이는데 거리 모름". 가짜 거리를 만들지 않는다. safety_monitor 는 `detected && NaN` 이면 SLOW. tf 가 없을 때는 `header.frame_id` 가 카메라 프레임.

**safety_monitor 손 거리·입력 끊김** (#59, #14, #98, #95): 손 없음 → NORMAL / NaN → SLOW(`hand_distance_unknown`, 직전 판정이 STOP 이었으면 놓침과 같이 `hand_lost_stop_s` STOP(`hand_lost_near`) 유지 — 깊이는 카메라 30 cm 안에서 빠진다) / 중앙선 너머 `hand_deep_stop_m`(−0.15) 보다 깊이 → STOP (접촉 중이어도. 사유는 `hand_deep_end_m`(−0.30) 보다 얕으면 `hand_near_centerline` — 손 빼면 이어 감, 더 깊으면 `hand_deep_intrusion` — 세션 종료, 10/12) / **손 관절 21점 ↔ 로봇 끝 선분(TCP → 플랜지 `robot_segment_m`)** 최소 거리(선분 중심축까지 — 그리퍼·카메라 표면까지 실제 여유는 더 작다)가 `hand_stop_robot_m`(0.20) 안 → STOP(`hand_near_robot`), `hand_slow_robot_m`(0.35) 안 → SLOW(`hand_near_robot`) / 중앙선에서 `slow_distance_m`(0.10) 안 → SLOW(`hand_near_centerline`). 로봇 위치(tf·`tcp_pose` 둘 다)를 모르거나 손 좌표가 base_link 가 아니면 로봇 거리를 모르므로 예전 선 기준 `stop_distance_m`(0.15) 안 → STOP(`hand_near_centerline`). 보이던 손을 놓치면(`detected=false`) 직전 판정이 STOP 이었으면 `hand_lost_stop_s` STOP → `hand_lost_slow_s` 까지 SLOW, SLOW 였거나 환자 접촉 중이면 SLOW(`hand_lost_near`). 손 거리 STOP 사유(`hand_near_centerline`·`hand_near_robot`·`hand_lost_near`)는 래치가 아니다. 손 관절이 전부 테이블 y 범위(`table_y_min_m`~`table_y_max_m`) 밖이면(옆 사람 손) 선 기준 판정 없이 로봇 거리로만 (로봇 위치를 모르면 선 기준). 단 `/rally/safety/patient_contact` 동안은 근접 STOP 대신 SLOW(`hand_near_patient_contact`) — 건네기·저항 밀기에서 손이 물체·손잡이를 잡는 순간. 손이 `hand_deep_stop_m` 보다 깊이 들어오거나 손 메시지가 끊겼으면 STOP. 치기·집기·놓기의 `contact_expected` 는 완화 대상이 아니다. 손·wrench·tcp_pose 를 받다가 `*_timeout_s` 넘게 끊기면 SLOW(`hand_stale`·`wrench_stale`·`tcp_stale`). 힘·tcp_pose(`required_inputs`)를 시작 후 `required_grace_s`(5 s) 넘게 한 번도 못 받으면 STOP(`wrench_missing`·`tcp_missing`, 받으면 풀림, 시뮬 `RALLY_SIM=1` 은 제외), 손은 한 번도 못 받아도 경고 로그만. 힘(`stale_stop_inputs`)은 받다가 `stale_stop_s`(5 s, 10/11 2 → 5 s: 특이점 근처를 지나가기만 해도 1~3 s 끊김) 넘게 끊겨도 STOP(`wrench_lost`, 받으면 풀림). 로봇 위치는 tf(`base_link → link_6`)로도 보며, tf 가 살아 있으면 손 ↔ 로봇 거리는 tf 로 재고 `tcp_pose` 짧은 끊김(`robot_tf_tcp_cover_s` 2 s 안)은 SLOW 가 아니다 (`robot_tf_*`). tf 신선도는 `header.stamp` 로 본다(얼어붙은 tf 는 `tcp_timeout_s` 뒤 안 씀). tf TCP 로도 작업 공간을 검사한다(`workspace_limit`). **작업 공간 x 상한은 받기 창 동안 `workspace_catch_x_max_m`(0.78, 10/14 Y0 대기 자세 — 요격 이동 TCP x 최대 0.73)** — `catch_window` 신호가 살아 있는 동안 + 꺼진 뒤 TCP 가 평소 0.57 안으로 돌아올 때까지(최대 `workspace_catch_hold_s` 10 s). 음수면 기능 없음. `SafetyStatus.min_hand_distance` 는 손 없음 9.9, 거리 모름 NaN(rally_db 에는 null). 값은 모두 `safety.yaml`.

**HandState 손목 각도** (`wrist_angle_deg`): 0° 손바닥 아래 · 90° 옆(엄지 위) · 180° 위. **안정화된 값** — 최근 0.5 s 의 좋은 프레임(카메라에서 너무 가깝지 않음: 깊이가 있으면 손바닥 거리 ≥ 0.30 m, 손은 보이는데 손바닥 깊이가 비면 가까움으로 봄, 깊이가 아예 없으면 화면 속 손 크기로 대신) 중간값. 손이 너무 가까우면 직전 값을 유지(잡는 순간에도 다가올 때의 값), 손이 1 s 넘게 안 보이면 NaN. 0 은 "모름" 이 아니다 — 모르면 NaN. 중간값이라 손목을 돌리면 약 0.25 s 늦게 따라온다 (잡는 순간 기록용. 실시간 제어에 쓰려면 이 지연을 감안). 파라미터 `wrist_window_s`·`wrist_lost_s`·`wrist_near_m`·`wrist_max_hand_frac` (hand_pose).

**TTS 발화 중 마이크** (#45): `/rally/voice/speaking` 은 `Speak.block_mic=true` 발화가 시작되면 `true`, 끝나고 잔향 여유(약 0.3 s) 뒤 `false`. intent_node 는 `true` 동안 **일반 명령만 무시**하고 비상 키워드("그만/멈춰" → `is_emergency`)는 계속 듣는다 — 로봇이 말하는 동안에도 음성 정지가 살아 있어야 한다. 그래서 TTS 문구에는 "그만"·"멈춰"를 쓰지 않는다 (에코로 정지 오인식 방지).

**사람이 말하는 중 TTS** (10/15 조정묵): `/rally/voice/listening` 은 호출어("랠리야")를 감지한 순간 `true`, 녹음(5 s)·STT·의도 판단이 끝나 `/rally/voice/intent` 를 냈거나 버렸을 때 `false` (겹치면 마지막 판단이 끝날 때). tts_node 는 `true` 동안 들어온 `Speak` 요청을 끝날 때까지 미뤘다가 순서대로 말하고(15 s 넘게 이어지면 그냥 말함), brain 은 모드 선택 대기 안내("어떤 모드를 진행할지…")를 그 동안 건너뛴다 — 사람이 명령을 말하는 중에 로봇이 끼어들지 않게.

**비상어 상시 감지** (#50, 기획 CP9·FR-07): intent_node 는 호출어 없이 마이크를 80 ms 마다 키워드 모델(`models/kws/emergency_kws.joblib`)로 판정해 "그만"·"멈춰"가 들리면 **바로** `/rally/safety/estop`(true) 와 `VoiceIntent`(`intent=stop`, `is_emergency=true`, `text`=들은 단어, `confidence`=점수, `header.stamp`=감지 시각)를 낸다. STT·네트워크를 안 거치고, 로봇 발화 중·호출어 뒤 녹음 중·STT 처리 중에도 계속 듣는다. 비상 점수(P(그만)+P(멈춰))가 모델에 저장된 임계를 연속 프레임 수(`consec`, 지금 0.7·3 프레임 = 0.24 s)만큼 넘어야 울린다 (순간 소음 오탐 억제, 말 끝 → 감지 약 0.4~0.6 s). 한 번 울린 뒤 `kws_refractory_s`(1.5 s) 동안은 다시 안 울림. 모델 파일이 없으면 이 감지만 꺼지고 호출어 뒤 STT 비상어 규칙은 그대로. **호출어는 "랠리야"** (10/13 정동휘 결정, FR-06): 같은 모델의 P(랠리야)가 같은 임계·연속 프레임을 넘으면 녹음 시작 (`wake_source:=kws` 기본). 모델이 없거나 랠리야 라벨이 없으면 선생님 "hello rokey"(`wake_source:=openwakeword`)로 돌아간다.

**음성으로 훈련 고르기** (`VoiceIntent` 슬롯 약속, 메시지 변경 없음): "공 굴리기 하자"처럼 훈련 이름을 말하면 `intent: next_mode` + `slot_keys: [mode]`, `slot_values: ["1"]` (1 공 굴리기 · 3 캐치볼 · 5 건네고 받기 · 7 팔씨름). 세션 시작과 같이 말하면 `start_session` 에 같은 슬롯. 슬롯이 없으면 순서상 다음 모드. 모드 3 이 꺼져 있으면(`mode3_challenge.enabled: false`) brain 이 거절하고 로그·`SetMode` 응답 message 로 알린다 (TTS 안내는 Speak 연결 뒤). 같은 발화(`header.stamp` 같음)는 한 번만 처리한다.

### 2.3 판단·안전·기록 (커스텀)

| 토픽 | 타입 | 발행 | 구독 | 주기 | QoS |
|---|---|---|---|---|---|
| `/rally/safety/status` | `rally_interfaces/SafetyStatus` | rally_robot/safety_monitor | brain, dashboard, skill_server | 100 Hz (변화 시 즉시) | **Reliable, depth 10** |
| `/rally/safety/estop` | `std_msgs/Bool` | voice, 대시보드 버튼 (hand_pose 주먹 정지는 #59 에서 뺌) | safety_monitor | 이벤트 | **Reliable, TransientLocal** |
| `/rally/safety/contact_expected` | `std_msgs/Bool` | rally_robot/skill_server | safety_monitor | 접촉 예정 구간 동안 5 Hz 하트비트 (집기 하강 · 핸드오버 당김 대기 · 저항 밀기) | Reliable, depth 1 — 0.5 s 안에 갱신 없으면 꺼진 것으로 봄 |
| `/rally/safety/catch_window` | `std_msgs/Bool` | rally_robot/skill_server (catch_ball 만) | safety_monitor | 모드 3 받기 창(투구 감지 ~ 교차 + 0.5 s) 5 Hz 하트비트. `contact_expected` 와 둘 다 켜진 동안만 힘 한계 `catch_window_force_limit_n`(130 N, #144) | Reliable, depth 1 — 0.5 s 안에 갱신 없으면 꺼진 것(25 N). 창이 열린 뒤 `catch_window_max_s`(2 s) 넘으면 신호가 와도 25 N |
| `/rally/safety/detach_window` | `std_msgs/Bool` | rally_robot/skill_server (detach_ball 만) | safety_monitor | 모드 3 공 떼기 벗기는 구간 5 Hz 하트비트. `contact_expected` 와 둘 다 켜진 동안만 힘 한계 `detach_window_force_limit_n`(50 N, 10/15) | Reliable, depth 1 — 0.5 s 안에 갱신 없으면 꺼진 것(25 N). 켠 뒤 `detach_window_max_s`(40 s) 넘으면 신호가 와도 25 N |
| `/rally/safety/workspace_extend` | `std_msgs/Bool` | rally_robot/skill_server (detach_ball 만) | safety_monitor | 모드 3 공 떼기 → 박스 놓기 구간 5 Hz 하트비트. 켜진 동안만 작업 공간 x 상한 `workspace_x_max_extended_m`(0.65 m, 10/14) — y·z·손 근접·힘 한계는 그대로 | Reliable, depth 1 — 0.5 s 안에 갱신 없으면 꺼진 것(0.57 m). 켠 뒤 `workspace_extend_max_s`(15 s) 넘으면 신호가 와도 0.57 m |
| `/rally/safety/patient_contact` | `std_msgs/Bool` | rally_robot/skill_server | safety_monitor | 환자가 물체·손잡이를 잡는 구간 동안 5 Hz 하트비트 (건네기 당김 대기 → 후퇴, 저항 밀기). 이 동안 스킬은 환자 쪽(+x) 명령 이동 금지 | Reliable, depth 1 — 0.5 s 안에 갱신 없으면 꺼진 것으로 봄 |
| `/rally/brain/context` | `rally_interfaces/SceneContext` | rally_brain/context_builder | llm_advisor, dashboard | 5 Hz | BestEffort |
| `/rally/brain/trial_result` | `rally_interfaces/TrialResult` | rally_brain/bt_runner | rally_db/recorder, dashboard | 이벤트 | Reliable |
| `/rally/brain/state` | `std_msgs/String` (BT 현재 노드) | rally_brain/bt_runner | dashboard | 2 Hz | Reliable |
| `/rally/brain/session_report` | `std_msgs/String` (JSON `session_id`·`report_md`·`source` = `llm:<s>` 또는 `numeric:<이유>`) | rally_brain/bt_runner (세션 종료 뒤 별도 스레드) | rally_db/recorder → `sessions.report_md`·`ended_at`, Redis `rally:events` kind=`session_report` → dashboard `GET /api/report/latest` | 세션당 1회 | Reliable |

QoS 규칙: 영상·관절·힘 같은 **스트림은 BestEffort + depth 1**(최신 것만), **정지·안전·결과는 Reliable**. TransientLocal은 "늦게 켜진 노드도 마지막 상태를 알아야 하는" 상태 토픽에만 쓴다: `/rally/safety/estop`, `/rally/voice/speaking` (#45).

## 3. 서비스

| 서비스 | 타입 | 서버 | 설명 |
|---|---|---|---|
| `/rally/brain/set_mode` | `rally_interfaces/SetMode` | rally_brain | 모드 전환 요청 (대시보드·음성). 바로 답하고, 실제 전환은 지금 시도가 끝난 뒤(시도 사이·손 대기 중). 안전 STOP 중엔 거절. 대기 중 요청은 새 요청이 덮어씀. 마지막 모드에서 "다음" = 세션 종료 (message 에 표시) |
| `/rally/vision/run_calibration` | `rally_interfaces/RunCalibration` | rally_vision/calibration | eye-in-hand 캘리브레이션 (로봇 자세 순회 포함) |
| `/rally/llm/suggest_params` | `rally_interfaces/SuggestParams` | rally_brain/llm_advisor | 다음 시도 파라미터 제안. 응답은 최대 timeout_s(3 s) 뒤. 조절할 수 있는 값과 범위는 `skills.yaml llm_tunable` 이 정한다 (요청 범위는 그 안으로 좁히기만). `success=true` = LLM 값, `success=false` + `param_values` 있음 = LLM 실패로 대체값(마지막 값) — 둘 다 범위 안이라 그대로 써도 된다. `success=false` + `param_values` 비어 있음 = 요청 오류(llm_tunable 에 없는 값·범위 안 겹침) → 호출자가 규칙 값을 쓴다. `rationale` 은 80자 이내, 비상어가 들어 있으면 빈 문자열 |
| `/rally/voice/speak` | `rally_interfaces/Speak` | rally_voice/tts_node | TTS 발화, 발화 중 마이크 차단. 부르는 쪽: brain · skill_server(모드 3 패드 꽂은 뒤 "공을 떼 주세요", `mode3.manual_removal_prompt`, 10/15 — 응답을 기다리지 않음) |
| `/rally/gripper/command` | `rally_interfaces/GripperCommand` | rally_robot/gripper_node | RG2 폭·힘 명령 |
| `/rally/robot/go_observe_pose` | `std_srvs/Trigger` | rally_robot/skill_server | 관측 자세로 이동 (카메라가 테이블 전체를 보는 자세) |
| `/rally/robot/go_hand_view` | `std_srvs/Trigger` | rally_robot/skill_server | 손 보기 자세로 이동 — 툴을 환자 쪽으로 기울여 그리퍼 위 RealSense 가 환자 손을 본다 (모드 1 시도 전 레인 고르기). `skills.yaml mode1.hand_view_*`. 이미 그 자세면 바로 성공 |
| `/rally/robot/go_pick_view` | `std_srvs/Trigger` | rally_robot/skill_server | 모드 5 물체 보기 자세로 이동 — 관측 자세(관절 이동) → `skills.yaml mode5.pick_view_posx`(직선). 흰 배경(물체 놓는 자리)이 화면에 다 들어오는 자세. 세션 5 가 집기 전에 부른다 (10/11) |
| `/rally/robot/pad_pick` | `std_srvs/Trigger` | rally_robot/skill_server | 모드 3 패드 거치대에서 패드 집기 → 거치대 위 20 cm(U)에서 끝 — A(관절) → 열기 → G(직선) → 쥐기(폭 24~38 mm 확인) → 홈 따라 순응으로 들어 올림. 교시 `skills.yaml mode3.pad_rack_*_posj` (10/12) |
| `/rally/robot/pad_place` | `std_srvs/Trigger` | rally_robot/skill_server | 모드 3 쥔 패드를 거치대에 꽂기 — U(관절) → 홈 입구 위 P(`pad_insert_pre_m`, 고정 옆 보정 `pad_insert_shift_m`) → 옆·회전을 무르게 한 순응 하강(G 위 2 cm 부터 10 mm/s, 힘 변화 `pad_insert_contact_n` 연속 3 샘플 또는 합력 `pad_insert_abs_stop_n` 한 샘플에서 멈춤) → G 위 2 cm 보다 높으면 걸림: 홈 밖으로 올라가 `pad_insert_search_pattern_m`(옆 이동·base z 회전) 순서로 다시 시도, 다 걸리면 쥔 채 빠지고 실패(`pad_insert_blocked`) → 열기 → 손잡이 축 따라 빠짐 (10/14) |
| `/rally/robot/reset_safety` | `std_srvs/Trigger` | rally_robot/safety_monitor | 정지 해제 (운영자만) |
| `/rally/perception/body/reset_baseline` | `std_srvs/Trigger` | rally_human/body_pose | 몸 자세 기준(평소 자세) 다시 잡기 — 부른 뒤 약 2 s 바르게. 환자가 자리를 잡은 뒤 운영자(확인 창 `b`)·brain(세션 시작)이 부름 |
| `/rally/db/start_session` | `std_srvs/Trigger` → 추후 커스텀 | rally_db | 세션 레코드 생성 |

## 4. 액션 (스킬)

모든 스킬은 `rally_robot/skill_server`가 제공하고 `rally_brain/bt_runner`가 호출한다. 진행률 피드백과 취소를 지원하며, `SafetyStatus.level == STOP`이면 서버가 즉시 abort 한다. 안전 상태를 한 번도 못 받았거나 `status_timeout_s`(0.5 s, `safety.yaml rally_skill_server`) 넘게 끊겨도 STOP 으로 보고 goal 거부·실행 중 abort(`safety_status_stale`, #14) — safety_monitor 없이 `skill_server --fake` 만 띄우면 goal 이 거부된다.

| 액션 | 타입 | 모드 | 핵심 로봇 기능 |
|---|---|---|---|
| `/rally/skill/push_ball` | `rally_interfaces/PushBall` | 1 | 거치대 공 비전 집기 → 레인 입구 → 퍼터식 치기 → **손 보기 자세에서 끝남**(`mode1.end_at_hand_view`, 환자가 받는 모습). 훈련 Brain은 `target` + 계산한 양수 `push_speed` 전달. 속도 0의 로봇 계산 경로는 단독 시험용. 결과 `stop_position`·`error_mm` 은 `mode1.stop_measure: true`(단독 시험)일 때만 채움 — 세션에선 brain 이 받기 관측과 동시에 손 보기 자세 카메라(`SceneContext.objects`)로 재서 `TrialResult.error_mm`·`stop_d_m` 에 기록 |
| `/rally/skill/return_ball` | `rally_interfaces/ReturnBall` | 1 | 시도 뒤 레인 안 공을 비전으로 찾아 집고 거치대 맨 윗부분(`mode1.rack_drop_posx`, 직접교시)에 넣기 → 손 보기 자세. 로봇이 닿는 곳(중앙선 앞)에 올 때까지 기다림 (feedback `out_of_reach` → brain 이 안내) |
| `/rally/skill/pick_compliant` | `rally_interfaces/PickCompliant` | 1·3·5 공통 | task_compliance_ctrl(Z 저강성), 접촉 감지 후 파지 |
| `/rally/skill/catch_ball` | `rally_interfaces/CatchBall` | 3 (챌린지) | 패드 집기(`catch_ball.pickup_pad()`, 정정인 자리) → 대기 자세 → 공 놓은 뒤 `decide_after_s` 0.25 s 예측점으로 amovel 한 번(가속 2000 mm/s², #144)(TCP 자세 고정, 범위 `mode3.left/right/updown/down_max_m` — 10/14 Y0 대기 자세는 전 방향 25 cm(`left_max_m` 신설, 기존 자세는 왼쪽 0·아래 8 cm 자기충돌); `mode3.resend_enabled` 면 놓은 뒤 0.45 s 까지 예측이 3 cm 넘게 바뀔 때 한 번 더, 기본 꺼짐). 예측 std > 10 cm·관측 < 4 면 안 움직임(`low_confidence`), 받기 창(130 N)이 닫히기 전에 멈출 수 있는 거리만 이동(출발·재전송 둘 다, 2 cm 도 못 가면 `too_late`), 안전 SLOW 면 `slow_at_catch` 중단 → **비전 판정**: `/rally/perception/ball_on_pad` 가 `pad_hold_s` 1 s 연속이면 `caught` (힘 판정 없음, 10/12) → 판정 뒤 로직(`catch_ball.after_catch()`, 정정인 자리) → 복귀. `contact_expected`(받기 창)는 투구 감지부터 **교차 시각 + 0.5 s**(`skills.yaml mode3.contact_window_after_cross_s`)까지만(연 뒤 최대 `contact_window_max_s` 1.5 s, 결정 시점 타이머) — 판정은 창이 닫힌 뒤에도 계속. 같은 창 동안 `/rally/safety/catch_window` 도 켜서 힘 한계 130 N (#144, 창 밖 25 N. 반경 밖·저신뢰로 안 움직이면 그 자리에서 끔). servol 보정은 나중 출발 결정은 `mode3.decide_mode`: `rule`(기본, 0.25 s·품질 기다림) / `policy`(학습한 정책 ONNX `models/policy_stage1.onnx` 가 출발 시점·보정을 정함, 한계·게이트·창·판정은 같음, 못 읽으면 rule 로 — `catch_policy.py`) |
| `/rally/skill/move_target` | `rally_interfaces/MoveTarget` | 3 (Phase 2) | movec / movesx / moveb 궤적, 환자가 맞혀야 하는 후기 단계 |
| `/rally/skill/detach_ball` | `rally_interfaces/DetachBall` | 3 | 거치대에 꽂힌 패드에서 벨크로 공 떼기 (10/14, `rally_robot/detach.py`): 공 보기 자세(`mode3.detach_ball_view_posj`) → 비전 `velcro_ball` 1 s 중앙값 → 패드 면(+x) 정면·손가락 세로로 쥠 → 접촉점 아래를 중심으로 5° 씩 돌려 위부터 벗김(순응, 스킬 힘 한계 15 N, 합력 급락 5 N = 떼어짐) → 후퇴 → 박스(`detach_box_drop_posj`)에 놓고 공 보기 자세로. 박스 구간만 `/rally/safety/workspace_extend`. `ball_pose` 가 0 이면 비전. catch_ball.after_catch 가 같은 함수(`detach_ball.run`)를 action 없이 부른다 |
| `/rally/skill/handover` | `rally_interfaces/Handover` | 5 | 기울여 제시, 당김 힘 감지 해제 / 받기: 보라색 선 − 20 cm 에서 열고 대기(막대 화면 기울기에 손목만 맞춤) → 막대 3D 위치로 다가가기(막대 앞에서 옆·위아래 맞춘 뒤 툴 z 로 ≤ 20 cm, 보라색 선 3 cm 앞까지, 40 mm/s, phase `approach`) 또는 밀어 넣는 힘 → 잡기 → 제자리. **다가가기 조건 (#164, 10/13 정정인 — #134 를 이 범위로 좁혀 바꿈)**: 손 인식 입력이 있고 `hand_stop_mode` ≠ off 일 때만(아니면 대기만), 툴 힘 12 N 으로 멈추면 닫지 않고 연 채 물러남, 그리퍼 사이가 깊이로 확인될 때만 닫음. 대기는 순응, 회전·다가가기 때만 해제. `follow_hand`(손 추종 servol)는 쓰지 않음 (#134) |
| `/rally/skill/resist_push` | `rally_interfaces/ResistPush` | 7 | 환자가 로봇 손목(J5·J6)을 직접 민다(10/12, 손잡이 없음). 제시 x 고정·y 만 손에 맞춤 → 순응 강성으로 버티기, 목표 힘 유지 시 −x 로 져주기, 손 뗀 뒤 저속 복귀(release_wait_s 5 s 안에 안 떼면 복귀 없이 그 자리에서 끝남, 10/13). 접촉 예정(nudge 끔)은 손을 뗄 때까지 유지. brain 은 feedback phase `return` 에 `mode7.release_prompt` 를 한 번 말함. 로봇은 절대 밀지 않음. `handle_pose` 는 쓰지 않음 |
| `/rally/session/run` | `rally_interfaces/RunSession` | 전체 | brain 최상위 action |

## 4.1 모드 3 요격 타이밍

**10/11 구현 (`flight_predictor`)**: 손목·측면 RealSense 컬러 60 Hz → velcro_ball YOLO(두 장 묶음) → 측면 시각에 손목 픽셀 보간 + 삼각측량(측면 자세·시각 차는 `mode3_side_calib.json`) → 투구 시작 판정(로봇 쪽 1.5 m/s 이상 3점, 시작점 x ≥ 1.2 m) → EKF(중력 + 공기 저항) → 패드 평면 교차점이 패드 중심 0.6 m 안일 때만 발행. 10/10 bag 재생 오차 0.16 s 중앙 약 3 cm. 아래는 처음 계획(웹캠 기준):

릴리스 검출(손·공 마스크 분리, 웹캠 60 Hz) → 3~5 프레임 후 첫 착지 예측 발행 → `catch_ball`이 `lead_time_s`만큼 앞선 시각의 예측점으로 amovel → 매 프레임 갱신되는 예측으로 servol 보정 → 착지 → 패드 위 공 판정. 공 비행 시간이 보통 0.5~0.8 s(살살 토스)이므로 카메라→로봇 전체 지연을 100 ms 안쪽으로 맞추는 것이 핵심(`scripts/measure_latency.py`).

## 5. 기동 순서 (준비 확인 방식)

노드를 ROS 2 managed(Lifecycle) 노드로 바꾸지 않고, **앞 단계가 실제로 준비됐는지 확인한 뒤 다음 단계를 띄운다.** "준비"의 정의는 `rally_bringup/config/readiness.yaml` 한 곳에 그룹으로 둔다 (토픽: `max_age_s` 안에 메시지 수신 / 서비스·action: 서버 응답 가능).

```
0. 서버 PC      compose pc3: discovery · db · redis (healthcheck)        ← 모든 노드가 discovery 에 의존
1. 로봇·인식 PC source scripts/env_robot_pc.sh → dsr_bringup2 → pc1_robot.launch
                  safety_monitor → [wait_ready --group safety] → force_publisher · gripper_node · skill_server
                  확인 실패 시 로봇 노드를 켜지 않고 launch 종료
2. 로봇·인식 PC compose pc2: vision · hand_pose (pc2_perception.launch part:=...)
3. 서버 PC      compose pc3: brain (pc3_brain.launch) · dashboard · voice (pc2_perception.launch part:=voice — 10/14 부터 서버 PC, 스피커·마이크도 거기)
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
- `safety.yaml` — 손 근접(손 ↔ 로봇 끝 선분 `hand_slow_robot_m` 0.35 m 안 → SLOW(`slow_override` 50 %), `hand_stop_robot_m` 0.20 m 안 → STOP, 중앙선 `slow_distance_m` 0.10 m 안 → SLOW. `hand_stop_mode` stop·slow·off — #118, 기본 slow #158. slow 는 근접 STOP 을 SLOW 로 낮추되 `hand_deep_intrusion` 은 STOP 유지 #160, off 는 개발 시험 전용), 힘 한계(기본 25 N, 항상 적용. 모드 3 받기 창 `/rally/safety/catch_window` 동안만 `catch_window_force_limit_n` 130 N, 창 한 번 최대 `catch_window_max_s` 2 s — #144·#165), nudge 임계(한 샘플 |Δ|F|| `nudge_dfdt_n` 5 N — 로봇 정지 중이고 `/rally/safety/contact_expected` 가 아닐 때만 판정), 작업 공간(x·y·z min/max → 밖이면 STOP. TCP 한 점 기준. 10/11: 환자 쪽 x_max 0.57 m(보라색 선)가 주된 제한, y ±0.60 은 테이블 옆으로 크게 나가지 않게 남김, x_min −2.0 · z 0~2.0 은 자세 전환 경로를 위해 사실상 열어 둠)
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
- 손에 가까운 레인을 선택한다. 받기 관측 후 손 이탈은 최신 유효 손 거리 ≥ `skills.yaml mode1.hand_clear_m`(0.20 m, safety.slow_distance_m 과 별개)일 때만 인정한다.
  `hand_clear_wait_s` 이후 안내하고 계속 대기한다. 미검출·시간 초과만으로 회수하거나 세션을 종료하지 않는다.
  뒤늦게 이탈이 확인되면 ReturnBall을 보낸다. 대기 중 취소·STOP은 기존대로 중단하며 자동 재개하지 않는다.
- 기존 ROS 메시지·토픽·서비스 이름과 STOP/reset 계약은 변경하지 않는다. 두 PC의 시계·레인·레시피 설정을 일치시켜야 한다.

## 7. 세션 휴식·재개와 대시보드 제어 (#78)

- 기존 `/rally/voice/intent`의 `pause` 또는 `tired` → 현재 시도를 마친 뒤 휴식. `resume` → 휴식 중인 세션만 재개. 음성 계약 필드 변경 없음.
- 시도 경계는 모드 1의 타격·받기 관측·손 이탈 대기·회수·기록, 모드 5의 집기·GIVE·RECEIVE를 모두 포함한다. 손 이탈 대기는 시간 초과로 끝내지 않으며 이때 휴식 요청도 유지한다. 휴식 대기 중에는 현재 시도를 계속 처리하고, 다음 시도 앞의 BT 관문에서 대기한다. 마지막 시도면 휴식 대신 세션을 종료한다.
- 휴식 중 모드 전환·난이도 변경은 거절한다. 휴식 요청 시 대기하던 모드/난이도 요청은 비운다. 재개는 같은 난이도로 다음 시도부터 진행한다. 레시피 없는 모드의 난이도 변경은 안내만 한다 (모드 1 명시적 변경·모드 7 지원).
- 세션 취소는 휴식 중에도 세션을 종료한다. STOP·context 끊김은 10/15 부터 바로 끝내지 않고 9절의 손 근접 대기·복구 대기로 간다 (시간을 넘기면 종료). STOP 해제 서비스는 기존 그대로이며 재개로 해제하지 않는다. 준비 조건 미충족이면 재개를 거절한다.
- `/rally/brain/state`는 기존 BT 노드 이름을 유지하되 휴식 요청 후 `pause_pending`, 실제 휴식 중 `paused`, 손 근접 STOP 대기 중 `hand_stop_wait`를 발행한다. 상세 상태는 아래 Redis 상태에 제공한다.

대시보드는 ROS를 import하지 않는다. brain의 별도 I/O 콜백이 기존 Redis로 일시적 요청·응답을 주고받는다. `REDIS_URL`이 있으면 활성화되며 `dashboard_control` 노드 파라미터로 끌 수 있다. Redis 장애는 음성·안전 감시를 막지 않는다.

| 경로 | 계약 |
|---|---|
| Redis `rally:session_control` | brain이 0.2 s마다 갱신, TTL 2 s. `session_id`, `phase`(idle/running/pause_pending/paused/ended), `pause_id`, `reason`, `mode`, `difficulty`, `trial_index`, `can_resume`, `blocked_reason`, `updated_at`(Unix s), `bt_node`(현재 BT 실행 노드명), `mode3_enabled`(챌린지 설정), `hand_wait`(손 근접 STOP 대기 중이면 그 사유, 아니면 빈 문자열 — 뒤에 추가), `proposal`(10절), `pending_switch`(접수된 모드 전환: null 또는 `{mode: 1·3·5·7 | null(다음 모드)}` — 뒤에 추가), `lane`(모드 1 에서 손으로 고른 레인 0·1, 시도 밖·다른 모드는 null — 대시보드 그림이 그 레인으로 공을 굴림, 뒤에 추가), `mode_select`(모드 선택 대기 중이면 true — 음성·버튼으로 모드를 고르면 시작, 뒤에 추가), `rest`(휴식 제안 조건 — 대시보드 「휴식 제안 조건」 패널·알림용. 세션·모드 없으면 null, 아니면 `{mode, trials(같은 모드 유효 시도 수), need(llm_signal_count), count, on[신호 키], signals{compensation{on,count,need,window,trials} — 보상 동작은 세션 전체·모드 무관(10/15) · success_drop{on,applies,rate,max_rate,trials,min_trials} · slowing{on,last_s,mean_s,ratio,need_ratio,trials,min_trials}}, current_comp[이번 시도에 이미 보인 보상 동작 종류], llm{state: idle·asked·answered·skipped·dropped, result: continue·suggest_rest·null, at, signals}}` — `rule_layer.fatigue_breakdown` 과 같은 기준(규칙과 같은 입력)이며 표시용, 제어에 쓰지 않음. 뒤에 추가), `comp_log`(보상 동작 기록 — 대시보드 훈련 탭 「보상 동작 기록」 패널용(10/15). `{log[{trial, mode, success, types[trunk_lean·shoulder_hike]}] 최근 50개(세션 순서, 모드 무관), current[지금 시도에서 이미 보인 종류], window, need, count, trials}` — count/trials 는 규칙 '보상 동작 반복' 창(window) 안 횟수. 세션이 없어도(시작 전·끝난 뒤) 마지막 세션 기록을 그대로 싣고 current 만 빈 목록, 다음 세션 시작 때 비움. 표시용, 제어에 쓰지 않음. 뒤에 추가), `difficulty_log`(난이도 변화 기록 — 대시보드 모드 1 화면 「난이도 변화」 패널용(10/16). `{log[{trial(적용된 시도 회차), mode, from, to, action(harder·easier), source(voice·rule), at(Unix s)}] 최근 20개, range{1:[0,3], 7:[-3,3]}(모드별 단계 범위)}` — 한계라 그대로여도(from == to) 남긴다. 세션 없어도 마지막 세션 기록 유지, 다음 세션 시작 때 비움. 대시보드는 새 줄을 훈련 기록(kind `difficulty_change`, 화면에서만·DB 없음)과 알림으로도 보여 줌. 표시용, 제어에 쓰지 않음. 뒤에 추가) |
| Redis pub/sub `rally:session_commands` | 재개 요청 JSON: `request_id`, `session_id`, `pause_id`(모두 UUID 문자열), `command: resume`, `expires_at`(Unix s, 최대 3 s). 저장·재전송하지 않음. 모드 전환은 `command: set_mode`, `mode`(정수 1·3·5 또는 null = 다음 모드 — 7 은 10/14 시연 제외로 대시보드에서 뺌, brain 은 받으면 그대로 처리), `pause_id` 없음 — brain 은 세션 ID 를 확인한 뒤 `/rally/brain/set_mode`·음성과 같은 확인(휴식·손 대기·안전 정지·모드 3 꺼짐·같은 모드)을 거쳐 접수하고, 실제 전환은 시도 경계에서. 세션 종료는 `command: end_session`(`session_id` 만) — brain 이 접수하고 모드 선택 대기 중이면 바로, 시도 중이면 그 시도가 끝난 뒤 끝냄 (음성 "여기까지 하자"와 같은 경로) |
| Redis `rally:session_reply:<request_id>` | brain 응답 `success`, `message`, `request_id`. TTL 5 s. 세션·휴식 식별값을 확인한 뒤 현재 안전·준비 조건으로 재검증. 중복 요청은 이미 소비된 휴식 ID이므로 재실행 불가 |
| HTTP `GET /api/session` | 최신 Redis 제어 상태 또는 null (미수신·만료). Redis 연결 실패 503 |
| HTTP `POST /api/session/resume` | JSON `session_id`, `pause_id`. 서버가 요청 UUID·만료를 생성한다. brain 승인 시 200, 거절 409, 상태 없음/Redis/수신자 없음 503, 응답 미확인 504. timeout은 결과 미확인이므로 상태 새로고침 후 판단하고 자동 재시도하지 않음 |
| HTTP `POST /api/session/mode` | JSON `session_id`, `mode`(1·3·5 또는 null). `phase: running` 이고 세션이 같을 때만 Redis 로 보냄. brain 접수 200, 거절·상태 바뀜 409, 잘못된 모드 422, 상태 없음/Redis 503, 응답 미확인 504. 자동 재시도 없음 |
| HTTP `POST /api/session/end` | JSON `session_id`. `phase` 가 running·pause_pending·paused 이고 세션이 같을 때만 Redis `end_session`. 응답 코드는 `/mode` 와 같음 |
| WebSocket `/ws` | 기존 메시지에 `type: session`, `data: 위 상태 또는 null` 추가. 0.5 s마다 전달. 브라우저도 2 s 이상 갱신이 없으면 버튼 비활성화 |

음성 재개는 발화 timestamp가 현재 휴식 시작 이후인 경우만 허용한다. zero stamp·오래된/중복 발화는 재개하지 않는다. 대시보드 요청은 화면에 표시된 세션·휴식 ID를 보내므로 이전 세션/휴식에서 누른 버튼은 거절된다. 안전 여부는 화면의 표시가 아니라 brain이 결정한다.

### 모드 1 손 근접 STOP 대기

모드 1·5 에서 손 근접 STOP 이 걸리면 세션을 끝내지 않고 기다렸다가 이어 간다 (#115). 대상·조건·단계별 처리는 **9절**. 이 절의 상태 표시는 `/rally/brain/state` 의 `hand_stop_wait`, Redis `rally:session_control.hand_wait`(대기 사유) 두 가지다.

### 모드 1 손 시작 위치 대기 (PR #87 PM 정정·사용자 합의)

공 목표는 레인별 고정값을 실측·안전·속도 구간으로 제한한다. 손 위치나 난이도로 목표를 이동시키지 않는다.
난이도별 reach는 기존 RMSE×2 간격이며 손의 레인 축 투영 거리 ≥ 목표 거리 + reach일 때만 타격한다.
부족하거나 유효 손/NORMAL이 없으면 안내하며 대기한다. 대기 중에는 손 표본을 갱신하고 전송 직전에 조건을 재검증한다.
STOP·취소는 기존 세션 중단 처리. 타격 action 실패 시에도 다음 타격을 시작하지 않고 기록 후 종료하여 운영자가 공·그리퍼 상태를 확인한다.

## 8. LLM 선제 제안 확인 정책 — 확정 정책 (2026-10-08, 휴식 제안은 #103 에서 구현 — 아래 「휴식 제안」 절)

사용자 결정: **직접 요청은 처리하고, LLM의 선제 제안은 확인받는다.** 이 절은 목표 동작이며 7절의 기존 구현에 확인 기능이 이미 연결됐다는 뜻이 아니다. 새 서비스/토픽/HTTP 경로와 메시지 필드는 후속 구현 계획에서 정하고 명세와 함께 반영한다.

| 입력 | 목표 동작 |
|---|---|
| 명시적 휴식 요청 | 재확인 없이 현재 시도·회수 완료 후 휴식 |
| 명시적 쉽게/어렵게 요청 | 지원하는 모드의 허용 레시피를 다음 시도에 적용. 휴식 중 변경 거절 및 기존 범위 제한 유지 |
| 명시적 재개 요청 | 기존 휴식 상태·안전·준비 검사 후 같은 모드·난이도로 재개 |
| LLM의 휴식/쉽게 제안 | 환자 음성 또는 운영자 대시보드의 수락을 기다림. 수락 전에는 제안으로 제어 상태를 변경하지 않음 |
| 거절·무응답·만료 | 제안으로 상태를 바꾸지 않고 기존 규칙을 따름. 무응답을 수락으로 간주하지 않음 |
| 안전 STOP·세션 취소 | 확인을 기다리지 않고 기존 중단 처리. 대기 제안 무효화, 수락으로 STOP 해제·세션 부활 금지 |

- LLM은 허용 행동과 짧은 이유만 제안한다. 속도·힘·좌표는 출력하지 않는다. `continue`는 상태 변경 제안이 아니므로 수락을 요구하지 않는다.
- 수락 시에도 현재 세션·모드·난이도·안전·준비 상태와 제안의 유효성을 다시 확인한다. 적용은 현재 시도·회수 완료 후 경계에서만 한다. 모드 1 자동 난이도 차단을 우회하지 않으며 수락된 명시적 변경 경로만 검토한다.
- 세션별 대기 제안은 한 개로 제한한다. 제안 ID는 brain이 생성하고 한 번만 소비한다. 세션 종료·모드/난이도 변경·명시적 제어 요청으로 충돌하는 제안은 무효화한다. 제안 유효 시간은 20초다. 무응답이면 만료시키고 적용하지 않는다.
- 오래된/중복 음성, 다른 세션의 버튼, 이미 소비·거절·만료된 제안은 적용하지 않는다. 화면 클릭만으로 성공 처리하지 않고 brain의 적용 예약/거절 응답을 표시한다.
- LLM 호출은 비동기·3 s 제한·스키마/허용 목록 검증·규칙 폴백을 유지한다. 수락 대기는 LLM 호출 시간 제한과 별개이며 BT·안전 감시를 막지 않는다.

### 제안 안내·대기 동작 — 사용자 확정 (2026-10-08)

- 휴식 제안 발화는 **“혹시 피곤하진 않으세요? 조금 쉬었다 할까요?”**로 고정한다. LLM이 생성한 이유와 실제 환자에게 말하는 확인 문구를 구분하며, 이 발화는 피로를 확정하는 표현으로 바꾸지 않는다.
- 제안 후 **20초** 동안 응답을 기다린다. 기다리는 동안에는 기존 훈련을 계속하며, 제안 때문에 다음 시도를 막거나 휴식 상태로 바꾸지 않는다. 기존 안전 감시·직접 제어 요청은 그대로 우선한다.
- 유효한 제안에 대한 “아니요” 등 명시적 거절은 **즉시 제안을 종료**한다. 남은 시간을 기다리지 않고 이후 수락도 거절한다. 적용 효과는 무응답 만료와 같지만 기록에서는 거절과 시간 만료를 구분한다.
- 유효 시간 안에 수락되면 제안을 한 번 소비하여 적용을 예약한다. 현재 진행 중인 시도·회수 완료 후 휴식하며, 수락 후 시도 완료를 기다리는 시간에는 응답 만료 타이머를 다시 적용하지 않는다. 예약 뒤에도 STOP·취소·세션 종료 등 무효화 조건은 유지한다.
- 무응답 만료 또는 거절 뒤에는 기존 훈련을 계속한다. 다른 대화의 “아니요”를 무조건 제안 응답으로 취급하지 않고, 현재 유효한 제안과 음성 응답의 연결은 아래 음성 계약에서 확정한다.

### 음성 계약에서 정리할 부분

현재 `intent_rules.py`의 tired는 상태 표현(힘들다/피곤하다)과 휴식 요청(쉬자)을 함께 포함하고, `intent_llm.py`의 easier도 불편 표현을 포함한다. 기존 brain은 tired를 바로 휴식 요청으로 처리한다. 따라서 현 구현만으로 위 정책을 충족했다고 볼 수 없다.

후속 구현에서는 명시적 휴식 요청을 pause, 명시적 난이도 요청을 harder/easier로 분류하고 상태 표현은 직접 제어와 분리한다. 상태 표현은 규칙에 따라 의사를 확인하거나 애매한 경우 LLM 제안의 근거로 사용할 수 있다. VoiceIntent에는 현재 수락/거절 의도가 없으므로 정동휘 담당 음성 분류와 제안 ID·발화 시각 연계 계약을 맞춰야 한다. 임의의 모든 “네” 또는 resume을 제안 수락으로 해석하지 않는다.

## 9. 손 근접 STOP 대기·재진행 — #98, 1단계 구현 (#115, 2026-10-11) + 복구 대기 (2026-10-15)

**비래치 손 근접 STOP** 은 세션을 유지하고 기다렸다가 같은 시도를 이어 간다 (10/15 부터 모든 모드). 그 밖의 STOP 은 10/15 부터 세션 종료 대신 아래 「복구 대기」. 안전 감시의 거리 계산·임계값·정지 실행은 로봇 파트(safety_monitor)가 맡고, **Brain 은 STOP 을 낮추거나 reset 하지 않는다.** 코드: `rally_brain/hand_stop.py`, `bt_runner._handle_hand_stop`, `trees/mode1_push.py`.

### SafetyStatus.reason 별 Brain 처리 (#104 최종 · 정정인 확정 계약)

| `reason` | 래치 | Brain 처리 |
|---|---|---|
| `hand_near_robot` · `hand_near_centerline`(얕은 근접) · `hand_lost_near` | 아님 | **손 근접 대기** (모드 1·5, 아래 조건) |
| `hand_deep_intrusion` (중앙선 너머 `hand_deep_end_m` 30 cm 보다 깊이, 10/12 분리. 15~30 cm 는 STOP 이지만 사유 `hand_near_centerline` → 대기. `hand_stop_mode=slow` 에서도 STOP — #160 10/13) | 아님 | **자동 재진행 대상 아님** — 대기 목록에 넣지 않는다 (세션 종료) |
| `wrench_missing` · `tcp_missing` · `wrench_lost` | 아님 | 손 대기 아님 — 세션 종료 (운영자 확인) |
| `force_limit` · `nudge` · `workspace_limit` · `estop` · `voice_stop` · `fist_gesture` | 래치 | 세션 종료 (`reset_safety` 수동) |
| SLOW 사유 (`hand_distance_unknown` · `hand_near_patient_contact` · `*_stale` 등) | — | STOP 아님, 대기 대상 아님 |

- 대기 목록은 `skills.yaml brain.hand_stop_wait_reasons` 의 **정확한 문자열**만 (`hand_*` 접두사 허용 없음), 대상 모드는 `hand_stop_wait_modes: [1, 5]` (모드 5 는 10/11 추가).
- 복수 원인이면 safety_monitor 는 가장 심한 레벨의 첫 사유 하나만 낸다. 래치 STOP 중에는 손 판정을 하지 않으므로 손 사유가 래치 STOP 을 가리지 않는다.
- 10/12 분리 완료 (정정인 결정): 중앙선 너머 15 cm(`hand_deep_stop_m`)부터 접촉 중이어도 STOP 하는 건 그대로. 그 STOP 의 사유가 30 cm(`hand_deep_end_m`)보다 얕으면 `hand_near_centerline`(손 빼면 이어 감), 더 깊으면 `hand_deep_intrusion`(대기 목록에 없는 이름 → 세션 종료).

### 대기·재진행 조건 (구현된 동작)

- 대기 중에는 BT tick 을 멈춘다(새 명령 없음). 안내 TTS `hand_stop_prompt` 를 `hand_stop_prompt_s`(10 s) 간격으로.
- **재진행 = 안전 레벨 NORMAL 이 `hand_stop_resume_hold_s`(1 s) 이어짐.** 손 좌표·손이 보이는지는 조건이 아니다 — 카메라가 그리퍼 위(eye-in-hand)라 자세에 따라 손이 영원히 안 보일 수 있고, 손이 보이는 위치 자체가 안전거리 안일 수 있기 때문이다 (조정묵 결정, #115 리뷰). SLOW 는 재진행 조건이 아니다.
- 기다린 시간은 대기 블록의 timeout 계산에서 뺀다 (멈춘 시간 때문에 바로 timeout 나지 않게).
- **후속 이슈** (#115 리뷰, 정정인): `hand_stop_mode=stop` 에서 `hand_lost_near` 로 들어간 대기만 재진행 조건 하나 추가 (손이 다시 보이거나 운영자 확인). 지금은 `hand_lost_near` STOP 1 s → SLOW 3 s → NORMAL 1 s 로 최소 약 5 s 뒤 재진행.

### 중단 단계별 처리

로봇이 서 있을 때(스킬 goal 전·받기 관측·손 이탈 대기·시도 사이)는 그대로 기다린다. **로봇이 움직이는 중**이면 그 명령이 받은 진행 단계(`feedback.phase`)로 정한다.

| 중단 시점 | 해제 뒤 처리 |
|---|---|
| 관측 자세 이동 `go_hand_view` | 손이 빠지면 다시 요청 |
| PushBall `place` 전 (관측 자세·집기) | 같은 goal 다시 보냄 (쥐고 있으면 스킬이 집기를 건너뜀) |
| ReturnBall `find` · `out_of_reach` | 다시 보냄 |
| ReturnBall `hand_view` | 회수 끝난 것으로 |
| 모드 시작 관측 자세 `go_observe_pose` · 모드 5 물체 보기 자세 `go_pick_view` | 다시 요청 |
| 모드 5 PickCompliant `approach` 까지 | 다시 보냄 (그 뒤 하강·파지 중이면 세션 종료) |
| 모드 5 Handover GIVE | `release` 를 지났으면 건네기 끝난 것으로, `present` 까지면 다시 보냄, 그 사이(당김 대기)는 세션 종료 |
| 모드 5 Handover RECEIVE | `present` 까지면 다시 보냄, 그 뒤는 세션 종료 |
| PushBall `place` 이후 (놓기~치기~복귀) · ReturnBall `pickup`~`drop` | 공·그리퍼 상태가 애매해 같은 시도는 못 이어 감 → **복구 대기** 뒤 다음 시도 (10/15. 전엔 세션 종료. 로봇 스킬 옵션 #117 은 보류) |
| 모드 3 CatchBall `ready` (패드를 쥐고 공을 기다림) | 다시 보냄 (집기는 '이미 쥠'으로 통과). 비행·요격 중이면 복구 대기 뒤 다음 시도 |

재전송은 NORMAL 1 s 유지 뒤, 명령당 최대 5회. 재전송한 goal 이 그 순간 다시 STOP 으로 **거절**되면(손이 또 다가옴) 실패로 끝내지 않고 같은 대기로 돌아간다 (10/11 실로봇: 받기 goal 거절 → 세션 종료가 났었음). 공 회수 전 손 이탈 확인은 별도 조건이다: 최신 검출 손의 중앙선 거리 ≥ `skills.yaml mode1.hand_clear_m`(0.20 m, `safety.yaml slow_distance_m` 과 별개 — #113). 이 확인은 **처음 보낼 때만** 하고, 손 STOP 뒤 재전송은 NORMAL 1 s 만 본다 (10/12 실로봇: 회수하러 가다 선 자세에선 손이 카메라에 안 보여 '손 안 보임' 으로 영영 대기).

### 복구 대기 — 세션 종료 대신 (2026-10-15 조정묵, rally_brain/hand_stop.py `Recovery`)

위 표에서 이어 갈 수 없는 단계의 손 STOP, 대기 목록에 없는 모든 STOP 사유(래치 STOP·`*_missing`·`wrench_lost`·`hand_deep_intrusion`·모르는 사유), 대기 중 겹친 다른 STOP 은 **세션을 끝내지 않는다**.
Brain 은 트리를 멈춰 돌던 스킬을 취소하고(그 시도는 `interrupted` 기록) → 안전이 NORMAL 로 `hand_stop_resume_hold_s` 이어질 때까지 기다리고(래치 STOP 은 운영자 reset 이 있어야 풀린다 — `recovery_prompt` 로 부탁, `recovery_prompt_s` 마다) → 모드 1·5 는 `/rally/gripper/state` 로 그리퍼에 물건이 있으면 사람이 뺄 때까지(`clear_gripper_prompt`. 다음 스킬이 시작하자마자 그리퍼를 열어 물건이 떨어지므로) → **같은 모드의 남은 시도부터** 트리를 다시 만든다 (모드 시작 관측 자세 이동이 자세를 정리, intro 안내 없음). `recovery_max_s`(300 s)·`clear_gripper_max_s`(120 s)를 넘기면 그때만 세션 종료.
`no_safety_status`(안전 상태 3 s 끊김, 래치 아님)는 어느 모드든 스킬을 취소하지 않고 손 근접처럼 기다린다 (안내 없음). `/rally/brain/context` 끊김은 3 s 부터 기다리고(새 goal 없음) 10 s 넘으면 종료 (`context_timeout_s`·`context_lost_end_s`).
Brain 은 STOP 을 낮추거나 reset 하지 않는다. 표시는 손 근접 대기와 같은 `hand_wait`(사유) — 대시보드가 사유로 「손 근접 정지 대기」·「정지 뒤 복구 대기」·「안전 상태 수신 대기」를 구분한다. 값은 모두 `skills.yaml brain`.

같은 날 시도 단위로도 세션 종료를 줄였다: 모드 시작 관측 자세·손 보기·물체 보기 자세 실패는 `pose_retry_wait_s` 쉬고 `pose_tries` 번까지 다시, 끝내 못 가면 그 시도만 실패(손·물체 보기) 또는 그 모드만 건너뜀(관측 자세, 스킬 서버 없음 — `mode_skip_prompt`). 모드 1 밀기 실패는 `push_fail_prompt` + `push_fail_wait_s`(그리퍼가 빌 때까지), 모드 5 받기 실패는 `receive_fail_prompt` + 비전에 물체가 보이거나 `receive_fail_wait_s` 뒤 다음 시도 (steps.py `OperatorWait`). 세션 루프의 코드 예외는 기록·로그 뒤 종료.

### 휴식·표시·기록

- 대기 중 모드 전환·난이도 변경은 거절, LLM 제안은 무효화, 휴식 요청은 보존한다. 음성/대시보드 `resume` 은 손 근접 대기를 우회하지 않는다.
- 표시: `/rally/brain/state` = `hand_stop_wait`, Redis `rally:session_control.hand_wait` = 대기 사유 (7절 표).
- 대기 진입·해제는 로그로 남기고, 대기 때문에 TrialResult 를 따로 만들지 않는다 (논리적 시도당 결과 한 번).

### 남은 것

1. ~~로봇 쪽: `hand_deep_intrusion` 분리~~ (10/12 완료), action 취소·정지 확인 계약 (9절 계약 2번, 정정인 별도 정리)
2. ~~2단계: 위 '세션 종료' 단계의 이어 가기~~ → 10/15 복구 대기로 대체 (로봇 스킬 옵션 #117 은 보류)
4. 로봇·인식 PC 실기 확인 (10/15 복구 대기): 래치 STOP → reset → 같은 모드 남은 시도 / 공을 쥔 채 STOP → "물건을 빼 주세요" → 그리퍼 비면 진행
3. 로봇·인식 PC 실기 확인: 움직이는 중 손 STOP → 손 빼기 → 같은 시도 이어 감. 문서 갱신을 실물 검증 완료로 보지 않는다.

## 휴식 제안 — LLM 판단·대시보드 응답

- Brain의 `test_rest_proposals` 파라미터는 기본 false. true로 시작할 때만 `/rally/brain/test/propose_rest` (`std_srvs/Trigger`)를 제공한다. 격리 가짜 스킬 환경에서만 켠다. 최신 상황판·안전·준비 조건과 running 상태, 대기 중인 모드/난이도 요청 없음을 검사한다. 이미 pending인 제안을 덮어쓰지 않는다.
- 발화/화면 문구: “혹시 피곤하진 않으세요? 조금 쉬었다 할까요?”. 생성 시점부터 monotonic 20초이며 TTS 재생 완료를 기다리지 않는다. 응답 대기 중 기존 훈련은 계속한다. 현재 구현은 rest만 지원한다.
- Redis `rally:session_control.proposal`: null 또는 `proposal_id`(UUID), `action: rest`, `source: test|llm`, `prompt`, `status`(pending/accepted/rejected/expired/invalidated), `remaining_s`. 기존 GET `/api/session`과 WS `type: session`에 포함한다.
- POST `/api/session/proposal/respond`: `{session_id, proposal_id, decision: accept|reject}`. ID는 UUID, 임의의 행동명 불허. API는 최신 상태를 확인하고 Redis에 `command: proposal_accept|proposal_reject`, `request_id`, `session_id`, `proposal_id`, `expires_at`을 보낸다. 요청 기한·reply 키·TTL·응답 대기는 resume과 동일하다. HTTP 200은 Brain 응답 후에만, 거절은 409, 연결/상태 없음 503, 결과 미확인 504. 자동 재전송하지 않는다.
- Brain은 같은 lock에서 현재 세션·모드·난이도·상태·제안 ID·monotonic 만료·안전·준비 상태를 다시 검사한다. 수락은 한 번 소비하고 기존 `_pause`로 적용 예약하여 현재 시도 전체 완료 후 휴식한다. 수락 후 응답 타이머로 예약을 취소하지 않는다. 마지막 시도라면 기존대로 세션 종료다.
- 거절은 즉시 rejected, 무응답은 20초 뒤 expired. 둘 다 훈련을 바꾸지 않는다. STOP/취소/상황판 상실/세션 종료/모드·난이도 변경/직접 휴식 요청은 pending을 무효화한다. 종료된 제안은 다시 수락할 수 없다. #98 손 근접 STOP 예외는 구현하지 않는다.
- 화면은 응답 결과(성공·거절·결과 미확인)를 같은 제안의 상태 갱신·재접속 뒤에도 보존한다. 새 세션 또는 새 제안 ID를 받으면 이전 안내를 초기화하며, 이전 세션·제안의 늦은 응답은 새 안내를 덮지 않는다. 최신 상태가 없으면 응답 버튼을 비활성화하고 자동 재전송하지 않는다.
- 테스트 제안 생성은 안전 판단이나 LLM 호출이 아니다. `tired` 직접 휴식의 기존 동작은 아직 변경하지 않으며, 실제 음성 분류·제안 응답 연결 전 상태다. 임의의 “네/아니요”나 resume은 제안 응답으로 사용하지 않는다.

### 실제 LLM 휴식 판단

외부 전송 목적지·필드는 사용자 승인 완료. OpenAI 공식 `https://api.openai.com/v1`의 `gpt-4o`를 사용한다.

- `rule_layer`가 애매한 신호를 `ask_llm`으로 분류한 경우에만 BtRunner 내부 비동기 작업자로 호출한다. 새 ROS 서비스는 추가하지 않는다. 기존 숫자 파라미터용 `SuggestParams`와 휴식 판단의 프롬프트·출력을 분리한다.
- 허용 출력은 `continue` 또는 `suggest_rest`뿐이다. pydantic 검증을 통과해도 현재 세션·모드·난이도·시도 세대와 안전·준비 상태를 다시 확인한다. 상태가 바뀌었다가 돌아와도 이전 응답을 적용하지 않는다.
- 동시에 한 요청만 처리하고 대기열을 쌓지 않는다. 최대 3초 후 실패로 처리하며, 늦게 끝난 응답은 버린다. 오류·키 없음·형식 오류·시간 초과는 기존 훈련 유지로 폴백한다. 네트워크 대기는 BT tick과 안전 콜백을 막지 않는다.
- `suggest_rest`는 `source: llm`의 20초 제안만 생성한다. LLM이 휴식을 즉시 적용하거나 로봇 제어값·STOP/reset을 변경하지 않는다. 화면은 **AI 제안**으로 표시하고 고정 안내 문구를 사용한다. 실제 휴식 적용에는 기존 대시보드 수락이 필요하다.
- 외부 전송은 `llm.yaml`의 휴식 판단 전용 허용 필드만 사용한다. 같은 모드의 유효 이력 최대 5개와 유한한 수치·불리언만 사용하며 세션 식별자·자유 메모·영상·원시 좌표·임의 params/metrics 키를 보내지 않는다.
- 모드 1의 받기 후보는 실제 공 받기의 성공률로 사용하지 않고 자동 난이도 변경도 계속 차단한다. 휴식 판단은 불확실한 신호에 대한 제안이며 피로 진단이나 치료 효과 판정이 아니다.
- 음성 `tired`의 직접 휴식은 기존대로다. 음성 수락/거절, KG/Graph RAG 검색 연결은 별도 후속 범위다.

설정은 BtRunner가 시작할 때 설치된 `llm.yaml`의 `rally_bt_runner.ros__parameters`를 직접 읽는다. `rest_enabled`, `rest_model`, `rest_temperature`, `rest_timeout_s`, `rest_prompt`, `rest_history_n`, `rest_send_fields`, `rest_send_signals`가 휴식 전용 설정이다. 변경 후 Brain 재시작이 필요하며 ROS 동적 파라미터로 제공하지 않는다. 외부 주소 환경변수로 목적지를 바꾸지 않는다.
