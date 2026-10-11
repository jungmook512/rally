uint8 LEVEL_NORMAL=0
uint8 LEVEL_SLOW=1
uint8 LEVEL_STOP=2

std_msgs/Header header
uint8 level
string reason                        # hand_near_centerline | hand_near_robot | hand_lost_near | hand_near_patient_contact | hand_distance_unknown | hand_stale | wrench_stale | tcp_stale | wrench_missing | tcp_missing | voice_stop | estop | nudge | force_limit | workspace_limit
float32 speed_override               # 0.0 ~ 1.0
float32 min_hand_distance            # [m] 손 없음 9.9, 손은 보이는데 거리 모름 NaN
