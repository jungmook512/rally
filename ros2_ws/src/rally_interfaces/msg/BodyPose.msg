# 전신 자세 요약 (웹캠, MediaPipe Pose). 재활 보상 동작 감지용.
std_msgs/Header header
bool detected
float32 trunk_lean_deg               # 몸통 기울기 [deg]
float32 shoulder_elevation_deg       # 어깨 들림 [deg]
float32 elbow_extension_deg          # 팔꿈치 신전 [deg]
bool compensation_detected           # 보상 동작 플래그
string compensation_type             # trunk_lean | shoulder_hike | none
