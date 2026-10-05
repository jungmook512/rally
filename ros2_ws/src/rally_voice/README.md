# rally_voice — 담당 정동휘

| 파일 | 출처 | 역할 |
|---|---|---|
| `wakeup_word.py` `stt.py` `audio_device.py` `MicController.py` | 선생님 코드 그대로 | openWakeWord("hello rokey"), OpenAI Whisper STT |
| `get_keyword.py` | 선생님 코드 (import만 변경) | **Day 1~2 기준선** `/get_keyword` Trigger 서비스 |
| `intent_node.py` | 신규 | 같은 부품으로 VoiceIntent 토픽 발행, 긴급어 직결 |
| `tts_node.py` | 신규 | Piper TTS |
| `resource/hello_rokey_8332_32.tflite` | 선생님 모델 | "랠리야" 모델로 교체 예정 |
| `resource/.env` | 직접 생성 (.env.example 참고) | OPENAI_API_KEY |
