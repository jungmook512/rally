"""rally_voice/intent_node.py — 웨이크워드 → STT → LLM function calling → /rally/voice/intent (VoiceIntent)
담당: 정동휘
재사용: rally_voice.wakeup_word.WakeupWord, rally_voice.stt.STT (선생님 코드). 프롬프트는 get_keyword.py 의 것을 RALLY 의도 목록으로 교체.
추가: "그만/멈춰" 키워드 스팟팅은 STT 결과 받자마자 is_emergency=True 로 즉시 발행. TTS 발화 중(/rally/voice/speaking=true)에는 일반 명령만 무시 — 비상 키워드는 계속 듣는다 (INTERFACES 2.2, #45).
확장: 웨이크워드 "랠리야" 모델(openWakeWord 학습) 로 교체 → resource/rally_ya.tflite, 스트리밍 STT(faster-whisper)로 5초 고정 녹음 대체.
"""
