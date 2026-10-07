"""rule_layer 단위 테스트 — 규칙 7개를 위에서부터 하나씩, 실제 skills.yaml·ontology.yaml 로."""
import pytest

from rally_brain import rule_layer
from rally_brain.config import load_skills
from rally_interfaces.msg import SafetyStatus, SceneContext, TrialResult

CFG = rule_layer.rule_config(load_skills())


def ctx(stop=False, compensation=False):
    c = SceneContext()
    c.safety.level = SafetyStatus.LEVEL_STOP if stop else SafetyStatus.LEVEL_NORMAL
    c.safety.reason = "nudge" if stop else ""
    c.body.compensation_detected = compensation
    return c


def trials(*results, mode=1, durations=None):
    durations = durations or [10.0] * len(results)
    return [TrialResult(mode=mode, success=r, duration_s=d) for r, d in zip(results, durations)]


def decide(c=None, ts=(), intent=None):
    return rule_layer.decide(c if c is not None else ctx(), list(ts), intent, CFG)


def test_config_comes_from_skills_yaml():
    assert CFG["success_streak"] == 3 and "ask_llm" in CFG["actions"]


def test_1_stop_waits_even_with_other_signals():
    assert decide(ctx(stop=True), trials(True, True, True), "tired").action == "wait"


def test_2_voice_tired():
    d = decide(ts=trials(True, True, True), intent="tired")
    assert d.action == "suggest_rest" and d.signals == ("voice_tired",)


@pytest.mark.parametrize("intent", ["harder", "easier"])
def test_3_voice_difficulty(intent):
    assert decide(intent=intent).action == intent


def test_4_success_streak():
    assert decide(ts=trials(True, True)).action == "continue"
    d = decide(ts=trials(False, True, True, True))
    assert d.action == "harder" and "3연속 성공" in d.reason


def test_5_failure_streak():
    assert decide(ts=trials(True, False)).action == "continue"
    assert decide(ts=trials(True, False, False)).action == "easier"


def test_streak_counts_only_last_mode():
    ts = trials(True, True, mode=5) + trials(True, mode=1)
    assert rule_layer.tail_streak(ts, True) == 1


def test_6_two_weak_signals_ask_llm():
    # 성공률 1/3 ≤ 0.4 (success_drop) + 마지막 시도가 앞선 평균의 1.5배 이상 (slowing). 연속 실패는 아님
    ts = trials(False, True, False, durations=[10.0, 10.0, 20.0])
    d = decide(ts=ts)
    assert d.action == "ask_llm" and set(d.signals) == {"success_drop", "slowing"}
    ts = trials(False, False, True, False, True, durations=[10, 10, 10, 10, 20])
    d = decide(ctx(compensation=True), ts)
    assert d.action == "ask_llm"
    assert set(d.signals) == {"compensation_repeated", "success_drop", "slowing"}
    assert "성공률 하락" in d.reason


def test_6_one_signal_is_not_enough():
    d = decide(ctx(compensation=True), trials(True, False, True))
    assert d.action == "continue" and d.signals == ("compensation_repeated",)


def test_7_continue():
    assert decide().action == "continue"


def test_action_outside_list_is_rejected():
    cfg = {**CFG, "actions": ["continue"]}
    with pytest.raises(ValueError):
        rule_layer.decide(ctx(), trials(True, True, True), None, cfg)


def test_apply_difficulty_clamped():
    assert rule_layer.apply_difficulty(0, "harder", CFG) == 1
    assert rule_layer.apply_difficulty(3, "harder", CFG) == 3
    assert rule_layer.apply_difficulty(-3, "easier", CFG) == -3
    assert rule_layer.apply_difficulty(2, "continue", CFG) == 2
