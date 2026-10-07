"""llm_advisor 단위 테스트 — 가짜 LLM 으로 네 단계(시간 제한·스키마 검증·범위 클램핑·대체값)와 전송 화이트리스트."""
import json
import time

import pytest

from rally_brain.llm_advisor import Advisor, ParamSpec, load_prompt, tool_schema
from rally_interfaces.msg import TrialResult

SPECS = [ParamSpec("push_speed", 0.05, 0.35), ParamSpec("lane", 0.0, 2.0)]


def trial(speed=0.2, lane=1.0, success=False, error=80.0, idx=1):
    return TrialResult(session_id="6f1c8a52-secret", notes="환자 이름 메모", mode=1, trial_index=idx,
                       success=success, error_mm=error, duration_s=4.0,
                       param_keys=["push_speed", "lane"], param_values=[speed, lane],
                       metric_keys=["release_speed"], metric_values=[1.25])


HISTORY = [trial(0.20, error=80.0, idx=1), trial(0.18, error=40.0, idx=2)]


def advisor(reply=None, delay=0.0, raises=None, timeout=0.3):
    sent = {}

    def llm(system, request, schema):
        sent.update(system=system, request=request, schema=schema)
        if delay:
            time.sleep(delay)
        if raises:
            raise raises
        return reply

    return Advisor(llm, "시스템 프롬프트", timeout_s=timeout), sent


def ok_reply(**values):
    return {"values": {"push_speed": 0.17, "lane": 1.0, **values}, "rationale": "40 mm 더 갔으니 조금 줄임", "confidence": 0.7}


def test_good_reply_is_used():
    a, _ = advisor(ok_reply())
    s = a.suggest(1, HISTORY, SPECS)
    assert s.source == "llm" and s.values == {"push_speed": 0.17, "lane": 1.0}
    assert s.rationale == "40 mm 더 갔으니 조금 줄임" and s.confidence == 0.7 and s.note == ""


def test_out_of_range_is_clamped():
    a, _ = advisor(ok_reply(push_speed=0.9, lane=-3))
    s = a.suggest(1, HISTORY, SPECS)
    assert s.source == "llm" and s.values == {"push_speed": 0.35, "lane": 0.0}
    assert "push_speed 0.9→0.35" in s.note


def test_timeout_falls_back_to_last_values():
    a, _ = advisor(ok_reply(), delay=1.0, timeout=0.2)
    t0 = time.monotonic()
    s = a.suggest(1, HISTORY, SPECS)
    assert time.monotonic() - t0 < 0.6                      # 기다리지 않는다
    assert s.source == "fallback" and "초과" in s.note
    assert s.values == {"push_speed": pytest.approx(0.18), "lane": 1.0}    # 마지막 시도 값 (float32)


@pytest.mark.parametrize("bad, why", [
    ({"values": {"push_speed": "빠르게", "lane": 1}, "rationale": "x", "confidence": 0.5}, "스키마"),
    ({"values": {"push_speed": 0.2}, "rationale": "x", "confidence": 0.5}, "이름 불일치"),
    ({"values": {"push_speed": 0.2, "lane": 1, "stiffness": 9}, "rationale": "x", "confidence": 0.5}, "이름 불일치"),
    ({"values": {"push_speed": 0.2, "lane": 1}, "rationale": "x", "confidence": 3}, "스키마"),
    ({"values": {"push_speed": float("nan"), "lane": 1}, "rationale": "x", "confidence": 0.5}, "NaN"),
    ("그냥 글", "스키마"),
])
def test_bad_reply_falls_back(bad, why):
    a, _ = advisor(bad)
    s = a.suggest(1, HISTORY, SPECS)
    assert s.source == "fallback" and why in s.note
    assert all(sp.lo <= s.values[sp.key] <= sp.hi for sp in SPECS)


def test_errors_fall_back():
    for exc, why in ((json.JSONDecodeError("x", "", 0), "형식 오류"), (ConnectionError("down"), "호출 실패")):
        a, _ = advisor(raises=exc)
        s = a.suggest(1, HISTORY, SPECS)
        assert s.source == "fallback" and why in s.note
    s = Advisor(None, "p").suggest(1, HISTORY, SPECS)
    assert s.source == "fallback" and s.note == "LLM 없음"


def test_fallback_without_history_is_mid_range_and_last_value_is_clamped():
    s = Advisor(None, "p").suggest(1, [], SPECS)
    assert s.values == {"push_speed": pytest.approx(0.2), "lane": 1.0}
    s = Advisor(None, "p").suggest(1, [trial(speed=0.99)], SPECS)
    assert s.values["push_speed"] == 0.35


def test_only_whitelisted_fields_are_sent():
    a, sent = advisor(ok_reply())
    a.suggest(1, HISTORY * 4, SPECS)
    text = json.dumps(sent["request"], ensure_ascii=False)
    assert "secret" not in text and "환자 이름" not in text and "session_id" not in text and "notes" not in text
    assert len(sent["request"]["history"]) == 5                          # history_n
    assert set(sent["request"]["history"][0]) == {"mode", "trial_index", "success", "error_mm", "duration_s",
                                                   "params", "metrics"}
    assert sent["request"]["params"] == {"push_speed": [0.05, 0.35], "lane": [0.0, 2.0]}
    assert sent["system"] == "시스템 프롬프트"


def test_bad_specs_are_rejected():
    with pytest.raises(ValueError):
        Advisor(None, "p").suggest(1, [], [ParamSpec("a", 1.0, 0.0)])
    with pytest.raises(ValueError):
        Advisor(None, "p").suggest(1, [], [ParamSpec("a", 0, 1), ParamSpec("a", 0, 1)])


def test_schema_and_prompt():
    schema = tool_schema(SPECS)
    assert schema["properties"]["values"]["required"] == ["push_speed", "lane"]
    assert schema["properties"]["values"]["properties"]["push_speed"]["maximum"] == 0.35
    assert "suggest_params" in load_prompt("prompts/suggest_params.md")
