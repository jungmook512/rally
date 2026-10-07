# rally_voice — 담당 정동휘

| 파일 | 출처 | 역할 |
|---|---|---|
| `wakeup_word.py` `stt.py` `audio_device.py` `MicController.py` | 선생님 코드 그대로 | openWakeWord("hello rokey"), OpenAI Whisper STT |
| `get_keyword.py` | 선생님 코드 (import만 변경) | **Day 1~2 기준선** `/get_keyword` Trigger 서비스 |
| `intent_node.py` | 신규 | 같은 부품으로 VoiceIntent 토픽 발행, 긴급어 직결 |
| `tts_node.py` | 신규 | `/rally/voice/speak` → Piper 한국어 TTS, 발화 중 `/rally/voice/speaking` |
| `resource/hello_rokey_8332_32.tflite` | 선생님 모델 | "랠리야" 모델로 교체 예정 |
| `resource/.env` | 직접 생성 (.env.example 참고) | OPENAI_API_KEY |

## TTS 음성 모델 (git 제외, 직접 받기)

```bash
mkdir -p ~/rally/models/piper && cd ~/rally/models/piper
base=https://huggingface.co/rhasspy/piper-voices/resolve/main/ko/ko_KR/kss/medium
wget $base/ko_KR-kss-medium.onnx $base/ko_KR-kss-medium.onnx.json
```

컨테이너는 `../models` 를 `/models` 로 마운트하므로 같은 파일을 쓴다. 확인:

```bash
ros2 run rally_voice tts_node
ros2 service call /rally/voice/speak rally_interfaces/srv/Speak "{text: '안녕하세요', block_mic: true}"
```
