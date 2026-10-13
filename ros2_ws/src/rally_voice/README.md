# rally_voice — 담당 정동휘

| 파일 | 출처 | 역할 |
|---|---|---|
| `wakeup_word.py` `stt.py` `audio_device.py` `MicController.py` | 선생님 코드 그대로 | openWakeWord("hello rokey"), OpenAI Whisper STT |
| `get_keyword.py` | 선생님 코드 (import만 변경) | **Day 1~2 기준선** `/get_keyword` Trigger 서비스 |
| `intent_node.py` | 신규 | 같은 부품으로 VoiceIntent 토픽 발행, 긴급어 직결 |
| `tts_node.py` | 신규 | `/rally/voice/speak` → Piper 한국어 TTS, 발화 중 `/rally/voice/speaking` |
| `resource/hello_rokey_8332_32.tflite` | 선생님 모델 | 예비 — 10/13 부터 호출어는 "랠리야"(`models/kws/emergency_kws.joblib` 의 P(랠리야), `wake_source:=kws`). 키워드 모델이 없거나 `wake_source:=openwakeword` 면 이것 |
| `resource/.env` | 직접 생성 (.env.example 참고) | OPENAI_API_KEY |

## TTS 음성 모델 (git 제외, 직접 받기)

```bash
mkdir -p ~/rally/models/piper && cd ~/rally/models/piper
# 버전 고정 (rhasspy/piper-voices 커밋 c10ece1) + 해시 확인
base=https://huggingface.co/rhasspy/piper-voices/resolve/c10ece1aade47bb51c153c893d14e5bf8e5b7117/ko/ko_KR/kss/medium
wget $base/ko_KR-kss-medium.onnx $base/ko_KR-kss-medium.onnx.json
sha256sum -c <<'SUM'
624fd774e26895f24bebae1bd9a3379e3394baeade4b584924f83e414096e2c9  ko_KR-kss-medium.onnx
153b5619d0580f824a59108d83ca19434de410eb8e4fe80325b58684fdc8a1df  ko_KR-kss-medium.onnx.json
SUM
```

TTS 문구에 비상어(`intent_rules.EMERGENCY_WORDS` — 그만·멈춰·아프·위험 등)가 있으면 말하지 않고 `done=false`. brain 문구는 `rally_voice.tts_node.phrase_problem()` 으로 미리 검사.

컨테이너는 `../models` 를 `/models` 로 마운트하므로 같은 파일을 쓴다. 확인:

```bash
ros2 run rally_voice tts_node
ros2 service call /rally/voice/speak rally_interfaces/srv/Speak "{text: '안녕하세요', block_mic: true}"
```
