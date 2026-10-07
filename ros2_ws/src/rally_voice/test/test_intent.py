"""intent_rules · intent_llm 출력 검사 단위 테스트 (마이크·API 없이)."""
import json

import pytest

from rally_voice.intent_llm import parse_tool_arguments
from rally_voice.intent_rules import classify_rules, is_emergency, normalize


@pytest.mark.parametrize("text", ["그만", "그만해!", "멈춰 멈춰", "아파요", "정지", "스톱", "Stop", "놔줘", "위험해"])
def test_emergency_words(text):
    r = classify_rules(text)
    assert r.intent == "stop" and r.is_emergency and r.confidence == 1.0


@pytest.mark.parametrize("text", ["훈련 시작하자", "공 굴리기 할게", "놀아요", "다음 거 하자"])
def test_not_emergency(text):
    assert not is_emergency(text)


@pytest.mark.parametrize("text, intent", [
    ("랠리야 훈련 시작하자", "start_session"),
    ("잠깐만 기다려", "pause"),
    ("너무 힘들어요", "tired"),
    ("다시 시작하자", "resume"),
    ("좀 더 어렵게 해줘", "harder"),
    ("천천히 해줘", "easier"),
    ("뭐라고? 다시 말해줘", "repeat"),
    ("다음 거 하자", "next_mode"),
])
def test_intents(text, intent):
    r = classify_rules(text)
    assert r is not None and r.intent == intent and not r.is_emergency


@pytest.mark.parametrize("text, intent, mode", [
    ("공 굴리기 하자", "next_mode", "1"),
    ("캐치볼 하고 싶어", "next_mode", "3"),
    ("팔씨름으로 바꿔줘", "next_mode", "7"),
    ("건네고 받기로 시작하자", "start_session", "5"),
])
def test_mode_slot(text, intent, mode):
    r = classify_rules(text)
    assert r.intent == intent and r.slots == {"mode": mode}


def test_emergency_beats_everything():
    r = classify_rules("훈련 시작하지 말고 그만")
    assert r.intent == "stop" and r.is_emergency


def test_ambiguous_goes_to_llm():
    assert classify_rules("오늘 날씨 좋네") is None


def test_empty_is_unknown():
    assert classify_rules("   ").intent == "unknown"


def test_normalize():
    assert normalize("훈련 시작!  하자~") == "훈련시작하자"


def test_llm_valid_output():
    r = parse_tool_arguments(json.dumps({"intent": "next_mode", "mode": "3", "confidence": 0.8}))
    assert (r.intent, r.slots, r.confidence, r.is_emergency, r.source) == ("next_mode", {"mode": "3"}, 0.8, False, "llm")


@pytest.mark.parametrize("bad", [
    "not json",
    json.dumps({"intent": "fly_away", "mode": "", "confidence": 0.9}),      # 목록 밖 의도
    json.dumps({"intent": "next_mode", "mode": "9", "confidence": 0.9}),    # 없는 모드
    json.dumps({"mode": "1"}),                                              # 의도 빠짐
])
def test_llm_invalid_output_is_unknown(bad):
    assert parse_tool_arguments(bad).intent == "unknown"


def test_llm_stop_is_not_emergency_and_mode_dropped():
    r = parse_tool_arguments(json.dumps({"intent": "stop", "mode": "1", "confidence": 0.9}))
    assert r.intent == "stop" and not r.is_emergency and r.slots == {}
    assert parse_tool_arguments(json.dumps({"intent": "harder", "mode": "", "confidence": 3.0})).confidence == 1.0
