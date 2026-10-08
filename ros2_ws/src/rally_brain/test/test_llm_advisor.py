"""llm_advisor 단위 테스트 — 가짜 LLM 으로 네 단계(시간 제한·스키마 검증·범위 클램핑·대체값)와 전송 화이트리스트."""
import json
import time

import pytest

from rally_brain.config import load_skills
from rally_brain.llm_advisor import Advisor, Limit, ParamSpec, load_prompt, tool_schema, tunable_limits
from rally_interfaces.msg import TrialResult

SPECS = [ParamSpec("push_speed", 0.05, 0.35), ParamSpec("lane", 0.0, 2.0)]
LIMITS = {1: {"push_speed": Limit(0.05, 0.35), "lane": Limit(0.0, 2.0)}}   # 테스트용 허용 목록 (실제는 skills.yaml)


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

    return Advisor(llm, "시스템 프롬프트", LIMITS, timeout_s=timeout), sent


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
    ({"values": {"push_speed": "빠르게", "lane": 1}, "rationale": "x", "confidence": 0.5}, "숫자가 아님"),
    ({"values": {"push_speed": "0.2", "lane": 1}, "rationale": "x", "confidence": 0.5}, "숫자가 아님"),
    ({"values": {"push_speed": True, "lane": 1}, "rationale": "x", "confidence": 0.5}, "숫자가 아님"),
    ({"values": {"push_speed": 0.2, "lane": 1}, "rationale": "x", "confidence": "0.5"}, "숫자가 아님"),
    ({"values": {"push_speed": 0.2, "lane": 1}, "rationale": "x", "confidence": True}, "숫자가 아님"),
    ({"values": {"push_speed": 0.2, "lane": 1}, "rationale": "x", "confidence": float("inf")}, "숫자가 아님"),
    ({"values": {"push_speed": 0.2, "lane": 1}, "rationale": "가" * 81, "confidence": 0.5}, "스키마"),
    ({"values": {"push_speed": 0.2}, "rationale": "x", "confidence": 0.5}, "이름 불일치"),
    ({"values": {"push_speed": 0.2, "lane": 1, "stiffness": 9}, "rationale": "x", "confidence": 0.5}, "이름 불일치"),
    ({"values": {"push_speed": 0.2, "lane": 1}, "rationale": "x", "confidence": 3}, "스키마"),
    ({"values": {"push_speed": float("nan"), "lane": 1}, "rationale": "x", "confidence": 0.5}, "숫자가 아님"),
    ("그냥 글", "객체가 아님"),
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
    s = Advisor(None, "p", LIMITS).suggest(1, HISTORY, SPECS)
    assert s.source == "fallback" and s.note == "LLM 없음"


def test_fallback_without_history_is_mid_range_and_last_value_is_clamped():
    s = Advisor(None, "p", LIMITS).suggest(1, [], SPECS)
    assert s.values == {"push_speed": pytest.approx(0.2), "lane": 1.0}
    s = Advisor(None, "p", LIMITS).suggest(1, [trial(speed=0.99)], SPECS)
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
        Advisor(None, "p", LIMITS).suggest(1, [], [ParamSpec("a", 1.0, 0.0)])
    with pytest.raises(ValueError):
        Advisor(None, "p", LIMITS).suggest(1, [], [ParamSpec("a", 0, 1), ParamSpec("a", 0, 1)])


def test_schema_and_prompt():
    schema = tool_schema(SPECS)
    assert schema["properties"]["values"]["required"] == ["push_speed", "lane"]
    assert schema["properties"]["values"]["properties"]["push_speed"]["maximum"] == 0.35
    assert "suggest_params" in load_prompt("prompts/suggest_params.md")


def test_real_tunable_list_from_skills_yaml():
    limits = tunable_limits(load_skills())
    assert limits[1]["push_speed"] == Limit(0.10, 0.60)                # skills.yaml mode1.push_speed_range (10/7 실측: 테니스공은 0.15 로 거의 안 굴러감 → 0.10~0.60, #42)
    assert limits[7]["target_force_n"].default == 5.0          # mode7.target_force_start_n
    assert 3 not in limits                                      # 모드 3 은 LLM 조절 금지


@pytest.mark.parametrize("mode, spec", [
    (3, ParamSpec("catch_radius", 0.0, 1.0)),        # 조절 금지 값 (#62 리뷰 1 재현)
    (1, ParamSpec("stiffness", 0.0, 9999.0)),        # 다른 모드의 값
])
def test_not_tunable_is_rejected_before_asking(mode, spec):
    a, sent = advisor(ok_reply())
    with pytest.raises(ValueError, match="조절할 수 없음"):
        a.suggest(mode, [], [spec])
    assert sent == {}                                 # LLM 에 묻지도 않는다


def test_request_range_is_narrowed_to_skills_yaml():
    # 호출자가 범위를 넓혀 보내도 skills.yaml 범위로 잘린다
    a, sent = advisor({"values": {"push_speed": 0.9}, "rationale": "빠르게", "confidence": 0.5})
    s = a.suggest(1, [], [ParamSpec("push_speed", 0.0, 5.0)])
    assert sent["request"]["params"] == {"push_speed": [0.05, 0.35]}
    assert s.values == {"push_speed": 0.35}
    a, _ = advisor(ok_reply())
    with pytest.raises(ValueError, match="안 겹침"):
        a.suggest(1, [], [ParamSpec("push_speed", 0.5, 0.9)])


def test_fallback_uses_default_ref_without_history():
    limits = tunable_limits(load_skills())
    s = Advisor(None, "p", limits).suggest(7, [], [ParamSpec("target_force_n", 0.0, 100.0)])
    assert s.values == {"target_force_n": 5.0}


@pytest.mark.parametrize("word", ["그만", "멈춰", "아프", "위험", "정지", "아파", "스톱"])
def test_rationale_with_stop_word_is_dropped(word):
    a, _ = advisor({"values": {"push_speed": 0.2, "lane": 1}, "rationale": f"{word} 하면 좋겠어요", "confidence": 0.5})
    s = a.suggest(1, HISTORY, SPECS)
    assert s.source == "llm" and s.values["push_speed"] == 0.2      # 값은 쓰고
    assert s.rationale == "" and "비상어" in s.note                   # 문장만 버림 (speech.phrase_problem 기준)


def test_prompt_and_code_agree_on_length():
    assert "80자" in load_prompt("prompts/suggest_params.md")
