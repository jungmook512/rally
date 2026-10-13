"""모드 7 시도 — 난이도 → 목표 힘, ResistPush goal, 기록·TTS, 손 대기. 가짜 스킬로 로봇 없이 확인한다."""
import math

import pytest

from fakes import FakeClient, Status, make_session, metric, param, run
from rally_brain import speech
from rally_brain.trees import mode7_resist
from rally_interfaces.action import ResistPush
from rally_interfaces.msg import TrialResult

FAST = {"mode7": {"hand_wait_s": 0.05, "hand_wait_tries": 2}}
OK = (True, {"success": True, "peak_force_n": 7.5, "hold_time_achieved_s": 2.1, "force_smoothness": 0.12})


def arrange(outcomes, cfg=FAST, **kw):
    client = FakeClient(ResistPush, outcomes)
    session, published, said, ctx = make_session({ResistPush: client}, cfg, **kw)
    return client, session, published, said


@pytest.mark.parametrize("level,expected", [(0, 5.0), (1, 5.5), (2, 6.05), (-1, 4.55), (-3, 3.75)])
def test_target_force_by_level(level, expected):
    assert mode7_resist.target_force(level, mode7_resist.DEFAULTS) == pytest.approx(expected, abs=0.01)


def test_target_force_clamped():
    cfg = {**mode7_resist.DEFAULTS, "target_force_start_n": 11.0}
    assert mode7_resist.target_force(3, cfg) == 12.0
    cfg = {**mode7_resist.DEFAULTS, "target_force_start_n": 3.0}
    assert mode7_resist.target_force(-3, cfg) == 3.0


def test_goal_uses_difficulty_and_config():
    client, session, published, said = arrange([OK])
    session.difficulty[TrialResult.MODE_RESIST] = 1
    assert run(mode7_resist.build_trial(session)) == Status.SUCCESS
    [g] = client.goals
    assert g.target_force_n == pytest.approx(5.5)
    assert (g.stiffness, g.hold_time_s, g.yield_distance_m, g.force_limit_n) == (700.0, 2.0, pytest.approx(0.05), 15.0)
    [t] = published
    assert t.success and t.mode == TrialResult.MODE_RESIST
    assert param(t, "target_force_n") == pytest.approx(5.5)
    assert metric(t, "peak_force_n") == pytest.approx(7.5)
    assert said == ["좋아요, 2초 버텼어요"]


@pytest.mark.parametrize("reason,prompt", [("timeout", "timeout"), ("released", "released"),
                                           ("force_limit", "force_limit"), ("safety_stop", None)])
def test_failure_recorded_with_prompt(reason, prompt):
    client, session, published, said = arrange([(False, {"failure_reason": reason, "peak_force_n": 3.0})])
    assert run(mode7_resist.build_trial(session)) == Status.SUCCESS
    assert client.sent == 1                                     # 재시도 없음
    assert [(t.success, t.notes) for t in published] == [(False, reason)]
    assert metric(published[0], "peak_force_n") == pytest.approx(3.0)
    assert said == ([mode7_resist.PROMPTS[prompt]] if prompt else [])


def test_no_hand_skips_without_presenting():
    client, session, published, said = arrange([], hand=False)
    assert run(mode7_resist.build_trial(session)) == Status.SUCCESS
    assert client.sent == 0
    assert [(t.mode, t.notes) for t in published] == [(TrialResult.MODE_RESIST, "no_hand")]


def test_stop_before_result_records_interrupted():
    client, session, published, _ = arrange([OK])
    tree = mode7_resist.build_trial(session)
    while client.sent < 1:
        tree.tick_once()
    tree.stop(Status.INVALID)
    assert [t.success for t in published] == [True]             # 결과가 이미 와 있었음 → 실제 결과로
    tree.stop(Status.INVALID)
    assert len(published) == 1


def test_interrupted_without_result_has_nan_metrics():
    t = mode7_resist.trial_from_result(mode7_resist.resist_goal(5.0, mode7_resist.DEFAULTS), None, "interrupted")
    assert not t.success and all(math.isnan(v) for v in t.metric_values)


@pytest.mark.parametrize("text", [p.format(hold=2.0) for p in mode7_resist.PROMPTS.values()]
                         + [mode7_resist.DEFAULTS["hand_prompt"]])
def test_prompts_are_speakable(text):
    assert speech.phrase_problem(text) is None


def test_release_prompt_once_on_return_feedback():
    """져준 뒤 손 뗌 대기(phase return)에 release_prompt 한 번 — 결과 TTS 전에 환자가 손을 떼게 (10/13)."""
    cfg = {"mode7": {**FAST["mode7"], "release_prompt": "이제 손을 떼 주세요"}}
    feedbacks = [[{"phase": "present"}, {"phase": "resist", "current_force_n": 6.0}, {"phase": "yielding"},
                  {"phase": "return"}, {"phase": "return"}]]
    client = FakeClient(ResistPush, [OK], feedbacks=feedbacks)
    session, published, said, ctx = make_session({ResistPush: client}, cfg)
    assert run(mode7_resist.build_trial(session)) == Status.SUCCESS
    assert said == ["이제 손을 떼 주세요", "좋아요, 2초 버텼어요"]          # return 두 번 와도 한 번만, 결과 문구보다 먼저
    assert speech.phrase_problem("이제 손을 떼 주세요") is None


def test_no_release_prompt_without_config():
    client = FakeClient(ResistPush, [OK], feedbacks=[[{"phase": "return"}]])
    session, published, said, ctx = make_session({ResistPush: client}, FAST)
    assert run(mode7_resist.build_trial(session)) == Status.SUCCESS
    assert said == ["좋아요, 2초 버텼어요"]

