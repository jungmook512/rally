"""rally_brain/rally_brain/speech.py — TTS 로 말해도 되는 문구인지 (brain 쪽 사본)
담당: 조정묵

tts_node 는 비상어가 든 문구를 말하지 않고 done=false 를 돌려준다 (#53). 스피커 소리가 마이크로 들어가
intent_node 가 "정지" 로 알아들을 수 있기 때문이다 (INTERFACES 2.2, #45).
brain 은 말하기 전에 같은 기준으로 걸러 낸다 — 거절당해 아무 말도 안 하는 일을 미리 막고, 이유를 로그에 남기려고.

EMERGENCY_WORDS·normalize·is_emergency 는 rally_voice/intent_rules.py 와 **똑같아야** 한다.
brain 컨테이너엔 rally_voice 가 없어 import 대신 사본을 두고, test_speech.py 가 원본과 대조한다 (다르면 실패).
"""
from __future__ import annotations

import re

EMERGENCY_WORDS = ("그만", "멈춰", "멈춰요", "멈춰줘", "정지", "스톱", "스탑", "stop", "아파", "아파요", "아프", "위험",
                   "살려", "놔", "놓아")


def normalize(text: str) -> str:
    """띄어쓰기·문장부호를 없애고 소문자로 (intent_rules.normalize 와 같음)."""
    return re.sub(r"[\s\.,!?~·…'\"]+", "", text).lower()


def is_emergency(text: str) -> bool:
    """intent_rules.is_emergency 와 같음. "놔"·"놓아" 는 짧아서 "놔줘/놔요/놔라" 형태만."""
    t = normalize(text)
    for w in EMERGENCY_WORDS:
        if w in ("놔", "놓아"):
            if re.search(w + r"(줘|요|라)", t):
                return True
        elif w in t:
            return True
    return False


def phrase_problem(text: str) -> str | None:
    """TTS 로 말하면 안 되는 문구면 이유, 괜찮으면 None (rally_voice.tts_node.phrase_problem 과 같은 기준)."""
    if not text.strip():
        return "빈 문구"
    if is_emergency(text):
        return "비상어가 들어 있음 (스피커 소리가 마이크로 들어가 정지가 걸릴 수 있음)"
    return None
