uint8 LEVEL_NORMAL=0
uint8 LEVEL_SLOW=1
uint8 LEVEL_STOP=2

std_msgs/Header header
uint8 level
string reason                        # hand_near_centerline | fist_gesture | voice_stop | nudge | force_limit | workspace_limit
float32 speed_override               # 0.0 ~ 1.0
float32 min_hand_distance            # [m]
