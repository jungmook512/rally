# 3D 손 관절 상태 (MediaPipe + RealSense 깊이). 좌표는 header.frame_id (기본 base_link).
uint8 GESTURE_NONE=0
uint8 GESTURE_FIST=1      # 즉시 정지
uint8 GESTURE_OPEN=2      # 재개 / 준비
uint8 GESTURE_POINT=3

std_msgs/Header header
bool detected
string handedness                    # left | right
geometry_msgs/Point[21] landmarks    # MediaPipe 21점, [m]
geometry_msgs/Pose palm_pose         # 손바닥 좌표계
float32 wrist_angle_deg              # 손목 회전(전완 대비) [deg]
float32 grip_closure                 # 0 = 펼침, 1 = 주먹
uint8 gesture
geometry_msgs/Vector3 pointing_dir
float32 distance_to_centerline       # 중앙선까지 거리 [m], 음수 = 로봇 구역 침범
