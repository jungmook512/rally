# 한 번의 시도 결과. DB 기록 + LLM 학습 루프 입력.
uint8 MODE_PUSH_BALL=1
uint8 MODE_CATCH_PAD=3
uint8 MODE_HANDOVER=5

std_msgs/Header header
string session_id
uint8 mode
uint16 trial_index
bool success
float32 error_mm                     # 모드1: 정지 위치 오차, 모드3: 패드 중심 오차, 모드5: 미사용(0)
float32 duration_s
string[] param_keys                  # 이 시도에 사용된 파라미터 (예: push_speed, push_dir_deg)
float32[] param_values
string[] metric_keys                 # 추가 측정값 (예: wrist_angle_deg, release_speed)
float32[] metric_values
string notes
