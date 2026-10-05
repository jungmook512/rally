# brain이 매 판단 주기마다 만드는 상황 요약. 규칙 레이어·LLM 공통 입력.
uint8 ROBOT_IDLE=0
uint8 ROBOT_MOVING=1
uint8 ROBOT_HOLDING=2
uint8 ROBOT_STOPPED=3

std_msgs/Header header
string session_id
uint8 mode
uint16 trial_index
uint8 robot_state
SafetyStatus safety
HandState hand
BodyPose body
TrackedObjectArray objects
VoiceIntent last_intent
TrialResult[] recent_trials          # 최근 N회 (기본 5)
float32 recent_success_rate
float32 fatigue_score                # 0~1, 규칙 레이어 추정
