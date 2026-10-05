# STT + LLM function calling 결과
std_msgs/Header header
string text                          # STT 원문
string intent                        # start_session | stop | pause | resume | next_mode | harder | easier | tired | repeat | unknown
string[] slot_keys
string[] slot_values
float32 confidence
bool is_emergency                    # "그만", "멈춰" → safety로 직결
