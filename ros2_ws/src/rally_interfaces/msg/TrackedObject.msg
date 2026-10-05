# 분할 마스크 기반으로 추적 중인 물체 하나. 좌표는 header.frame_id (기본 base_link).
std_msgs/Header header
uint32 track_id                      # ByteTrack ID (프레임 간 유지)
string class_name                    # ball | velcro_ball | pad | rod | cup | hand
float32 confidence
geometry_msgs/Point position         # 마스크 픽셀 깊이로 계산한 3D 중심 [m]
geometry_msgs/Vector3 velocity       # 칼만 추정 속도 [m/s]
float32 yaw                          # 마스크 주축 방향 [rad] (파지 각도용)
float32 mask_area                    # [px^2]
sensor_msgs/RegionOfInterest bbox
bool is_moving
geometry_msgs/Point predicted_stop   # 정지 예측 위치 [m] (is_moving일 때)
builtin_interfaces/Time predicted_stop_time
