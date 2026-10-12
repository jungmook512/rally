"""모드 3 시도 — CatchBall goal, 결과 기록·TTS, 연속 안 던짐·서버 없음 건너뛰기. 가짜 스킬로 로봇 없이 확인한다."""
import math

import pytest

from fakes import FakeClient, Status, make_session, metric, param, run
from rally_brain.trees import mode3_catch, session_tree
from rally_interfaces.action import CatchBall
from rally_interfaces.msg import TrialResult

HIT = (True, {"caught": True, "robot_travel_mm": 82.0, "reaction_time_s": 0.17, "prediction_error_mm": math.nan})


def arrange(outcomes, cfg=None, ready=True):
    client = FakeClient(CatchBall, outcomes, ready=ready)
    session, published, said, _ = make_session({CatchBall: client}, cfg)
    return client, session, published, said


def test_goal_and_success_record():
    client, session, published, said = arrange([HIT], {"mode3": {"catch_radius_m": 0.30, "trial_timeout_s": 25.0}})
    assert run(mode3_catch.build_trial(session)) == Status.SUCCESS
    [g] = client.goals
    assert (g.catch_radius, g.timeout_s, g.lead_time_s) == (pytest.approx(0.30), 25.0, 0.0)
    [t] = published
    assert t.mode == TrialResult.MODE_CATCH_PAD and t.success and t.notes == ""
    assert math.isnan(t.error_mm)                                   # 로봇이 아직 예측 오차를 안 잼
    assert param(t, "catch_radius") == pytest.approx(0.30)
    assert metric(t, "robot_travel_mm") == 82.0 and metric(t, "reaction_time_s") == pytest.approx(0.17)
    assert said == [mode3_catch.DEFAULTS["throw_prompt"], mode3_catch.PROMPTS["success"]]


@pytest.mark.parametrize("succeeded,reason,prompt", [
    (True, "missed", "missed"),                # 끝까지 돌았지만 안 맞음
    (True, "out_of_radius", "out_of_radius"),  # 반경 밖이라 안 움직임
    (False, "timeout", None),                  # 공을 안 던짐 (abort)
    (False, "safety_stop", None),
])
def test_failures_recorded_without_retry(succeeded, reason, prompt):
    client, session, published, said = arrange([(succeeded, {"caught": False, "failure_reason": reason})])
    assert run(mode3_catch.build_trial(session)) == Status.SUCCESS
    assert client.sent == 1
    assert [(t.success, t.notes) for t in published] == [(False, reason)]
    assert said[1:] == ([mode3_catch.PROMPTS[prompt]] if prompt else [])


def test_caught_even_if_reason_set_counts_as_success():
    """반경 밖이어도 판정은 돌아 명중이 나올 수 있음 (MODE3_PLAN 4.1-6) — 명중이면 성공, notes 비움."""
    _, session, published, _ = arrange([(True, {"caught": True, "failure_reason": "out_of_radius"})])
    run(mode3_catch.build_trial(session))
    assert [(t.success, t.notes) for t in published] == [(True, "")]


def test_consecutive_timeouts_skip_rest():
    client, session, published, _ = arrange([(False, {"failure_reason": "timeout"})] * 2)
    tree = mode3_catch.build_trial(session)
    for _ in range(2):
        run(tree)
    assert client.sent == 2
    assert run(tree) == Status.SUCCESS                              # 세 번째는 보내지 않고 기록도 없이 넘어감
    assert client.sent == 2 and len(published) == 2


def test_missing_server_skips_without_failing_session():
    client, session, published, said = arrange([], ready=False)
    assert run(mode3_catch.build_trial(session)) == Status.SUCCESS
    assert client.sent == 0 and published == [] and said == []


def test_stop_before_result_records_interrupted():
    client, session, published, _ = arrange([HIT])
    tree = mode3_catch.build_trial(session)
    while client.sent < 1:
        tree.tick_once()
    tree.stop(Status.INVALID)
    tree.stop(Status.INVALID)
    assert len(published) == 1


def test_interrupted_without_result_has_nan_metrics():
    t = mode3_catch.trial_from_result(mode3_catch.catch_goal(mode3_catch.DEFAULTS), None, "interrupted")
    assert not t.success and t.notes == "interrupted" and all(math.isnan(v) for v in t.metric_values)


def test_session_tree_runs_mode3():
    assert session_tree.TRIAL_BUILDERS[TrialResult.MODE_CATCH_PAD] is mode3_catch.build_trial


def test_skills_yaml_mode3_keys():
    from rally_brain.config import load_skills
    cfg = mode3_catch.mode3_config(load_skills())
    assert cfg["trial_timeout_s"] > 0 and cfg["timeout_end_after"] >= 1 and cfg["throw_prompt"]
    assert load_skills()["mode3"]["intro_prompt"] and load_skills()["mode3"]["done_prompt"]
