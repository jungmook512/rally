"""세션 리포트 — 수치 요약·LLM 폴백·보내는 필드 (가짜 LLM, 실제 호출 없음)."""
import json
import math
import time
from datetime import datetime, timedelta, timezone

from rally_brain import session_report as r
from rally_interfaces.msg import TrialResult

T0 = datetime(2026, 10, 11, 10, 0, tzinfo=timezone.utc)
T1 = T0 + timedelta(minutes=12)


def trial(mode, success, error_mm=math.nan, notes="", duration=5.0, metrics=None, idx=1):
    t = TrialResult(mode=mode, success=success, error_mm=float(error_mm), notes=notes, duration_s=duration,
                    trial_index=idx, session_id="11111111-2222-3333-4444-555555555555")
    if metrics:
        t.metric_keys, t.metric_values = list(metrics), [float(v) for v in metrics.values()]
    return t


TRIALS = ([trial(1, True, 40.0 - i, idx=i) for i in range(10)]
          + [trial(1, False, notes="no_hand"), trial(1, False, 30.0)]
          + [trial(5, True, metrics={"handover_time_s": 3.0}), trial(5, False, notes="interrupted")])


def good_llm(calls):
    def call(prompt, request, schema):
        calls.append((prompt, request, schema))
        return {"summary": "공 굴리기 성공률이 높았습니다.", "observations": ["오차 평균 32.5 mm"], "next_session": "같은 난이도 유지"}
    return call


def test_summary_counts_and_excludes():
    rows = {s.mode: s for s in r.summarize(TRIALS, {1: 2})}
    m1, m5 = rows[1], rows[5]
    assert (m1.attempts, m1.counted, m1.successes) == (12, 11, 10)
    assert m1.excluded == {"no_hand": 1}
    assert m1.key_measured == 11 and m1.level == 2
    assert m1.improvement is not None and m1.improvement > 0        # 오차가 줄어 나아짐
    assert (m5.attempts, m5.counted, m5.excluded) == (2, 1, {"interrupted": 1})
    assert m5.key_mean == 3.0 and m5.improvement is None             # 성공 10회 미만


def test_numeric_markdown_table():
    md = r.numeric_markdown(r.summarize(TRIALS, {}), T0, T1, "succeeded")
    assert "12분" in md and "완료" in md
    assert "| 1 공 굴리기 | 12 | 91% (10/11) |" in md
    assert "no_hand 1회" in md and "interrupted 1회" in md
    assert "아직" not in md


def test_llm_request_has_only_aggregates():
    calls = []
    md, source = r.build_report(TRIALS, {1: 1}, T0, T1, "succeeded", good_llm(calls), "SYS", 5.0)
    assert source.startswith("llm:") and "## 해석 (LLM, 참고용)" in md and "오차 평균 32.5 mm" in md
    prompt, request, schema = calls[0]
    text = json.dumps(request, ensure_ascii=False)
    assert "11111111" not in text and "no_hand" not in text and "notes" not in text   # 세션 ID·메모 원문 없음
    assert set(request) == {"session_minutes", "outcome", "modes"}
    assert schema["required"] == ["summary", "observations", "next_session"]


def test_fallbacks_keep_numeric_summary():
    slow = lambda *_: time.sleep(1.0) or {}
    def boom(*_):
        raise RuntimeError("api down")
    bad = lambda *_: {"summary": "", "observations": []}             # 빈 요약 = 형식 오류
    long = lambda *_: {"summary": "x" * 500, "observations": [], "next_session": ""}
    for llm, why in ((None, "LLM 키 없음"), (slow, "시간 초과"), (boom, "호출 실패 RuntimeError"),
                     (bad, "형식 오류"), (long, "형식 오류")):
        md, source = r.build_report(TRIALS, {}, T0, T1, "aborted", llm, "SYS", 0.2)
        assert source.startswith("numeric:" + why), (why, source)
        assert "| 1 공 굴리기 |" in md and "해석 없음" in md and "## 해석" not in md and "중단" in md


def test_no_trials_skips_llm():
    calls = []
    md, source = r.build_report([], {}, T0, T1, "canceled", good_llm(calls), "SYS", 5.0)
    assert calls == [] and source == "numeric:시도 없음" and "기록된 시도가 없습니다" in md


def test_prompt_file_has_rules():
    from rally_brain.llm_advisor import load_prompt
    text = load_prompt("prompts/session_report.md")
    assert "단정하지 않는다" in text and "write_session_report" in text and r.REPORT_TOOL == "write_session_report"
