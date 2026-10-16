"""모드 3 시도 — CatchBall goal, 결과 기록·TTS, 연속 안 던짐·서버 없음 건너뛰기. 가짜 스킬로 로봇 없이 확인한다."""
import math
import time
from types import SimpleNamespace as NS

import pytest
from action_msgs.msg import GoalStatus

from fakes import Done, FakeClient, Status, make_session, metric, param, run
from rally_brain.trees import mode3_catch, session_tree
from rally_interfaces.action import CatchBall
from rally_interfaces.msg import TrialResult

HIT = (True, {"caught": True, "robot_travel_mm": 82.0, "reaction_time_s": 0.17, "prediction_error_mm": math.nan})
D = mode3_catch.DEFAULTS


def arrange(outcomes, cfg=None, ready=True, client_type=FakeClient, feedbacks=None, **mode3):
    """기본은 투구 안내 지연 0 (가짜 스킬은 바로 끝나므로 지연이 있으면 안내가 안 나옴 — 그리퍼 상태는 모름(None) 이라 delay 경로).
    mode3 키워드로 mode3 설정을 덮어씀. feedbacks: goal 마다 보낼 feedback 목록."""
    cfg = dict(cfg or {})
    cfg["mode3"] = {"throw_prompt_delay_s": 0.0, **(cfg.get("mode3") or {}), **mode3}
    client = client_type(CatchBall, outcomes, ready=ready, feedbacks=feedbacks)
    session, published, said, _ = make_session({CatchBall: client}, cfg)
    return client, session, published, said


class Pending:
    """release() 전까지 안 끝나는 결과 future."""
    def __init__(self, value):
        self.value, self.ready = value, False

    def done(self):
        return self.ready

    def result(self):
        return self.value


class SlowClient(FakeClient):
    """goal 은 바로 받지만 결과는 release() 뒤에 — 스킬이 도는 동안 트리를 틱해 보려고. feedbacks 는 goal 직후 차례로 (FakeClient 와 같음)."""
    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        if feedback_callback is not None:
            for fields in (self.feedbacks.pop(0) if self.feedbacks else []):
                feedback_callback(NS(feedback=self.result_type.Feedback(**fields)))
        succeeded, fields = self.outcomes.pop(0)
        status = GoalStatus.STATUS_SUCCEEDED if succeeded else GoalStatus.STATUS_ABORTED
        self.pending = Pending(NS(status=status, result=self.result_type.Result(**fields)))
        return Done(NS(accepted=True, get_result_async=lambda: self.pending, cancel_goal_async=lambda: None))

    def release(self):
        self.pending.ready = True


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
    assert said == [D["throw_prompt"], D["success_prompt"]]


@pytest.mark.parametrize("succeeded,reason,prompt", [
    (True, "missed", "missed_prompt"),                # 끝까지 돌았지만 안 맞음
    (True, "out_of_radius", "out_of_radius_prompt"),  # 반경 밖이라 안 움직임
    (False, "timeout", None),                         # 공을 안 던짐 (abort)
    (False, "safety_stop", None),
])
def test_failures_recorded_without_retry(succeeded, reason, prompt):
    client, session, published, said = arrange([(succeeded, {"caught": False, "failure_reason": reason})])
    assert run(mode3_catch.build_trial(session)) == Status.SUCCESS
    assert client.sent == 1
    assert [(t.success, t.notes) for t in published] == [(False, reason)]
    assert said == [D["throw_prompt"]] + ([D[prompt]] if prompt else [])


def test_throw_prompt_after_delay_while_skill_runs():
    # 10/15: 로봇이 패드를 집고 대기 자세에 설 즈음(throw_prompt_delay_s) 투구 안내 — goal 을 보내자마자가 아니라
    client, session, published, said = arrange([HIT], client_type=SlowClient, throw_prompt_delay_s=0.1)
    tree = mode3_catch.build_trial(session)
    for _ in range(5):
        tree.tick_once()
    assert client.sent == 1 and said == []                       # 스킬은 돌고 있는데 아직 안내 전
    time.sleep(0.12)
    tree.tick_once()
    assert said == [D["throw_prompt"]]
    tree.tick_once()
    assert said == [D["throw_prompt"]] and tree.status == Status.RUNNING   # 한 번만, 스킬은 계속
    client.release()
    assert run(tree) == Status.SUCCESS and said == [D["throw_prompt"], D["success_prompt"]] and len(published) == 1


@pytest.mark.parametrize("outcome,said_after", [
    ((False, {"failure_reason": "timeout"}), []),                       # 집기 실패·abort 가 안내 전에 끝남 → 안내 없음
    ((True, {"caught": False, "failure_reason": "missed"}), [D["missed_prompt"]]),
])
def test_no_throw_prompt_when_skill_ends_before_delay(outcome, said_after):
    client, session, published, said = arrange([outcome], throw_prompt_delay_s=30.0)
    assert run(mode3_catch.build_trial(session)) == Status.SUCCESS
    assert client.sent == 1 and len(published) == 1 and said == said_after


def test_empty_throw_prompt_is_silent():
    _, session, published, said = arrange([HIT], throw_prompt="")
    assert run(mode3_catch.build_trial(session)) == Status.SUCCESS and said == [D["success_prompt"]]


@pytest.mark.parametrize("reason", ["missed", "out_of_radius"])
def test_last_trial_fail_says_short_phrase(reason):
    # 10/15: 마지막 시도에서 못 받으면 '다시 던져 볼까요' 없이 last_fail_prompt — 바로 done_prompt 가 이어짐
    fail = (True, {"caught": False, "failure_reason": reason})
    client, session, published, said = arrange([fail, fail])
    session.trials_planned[TrialResult.MODE_CATCH_PAD] = 2
    tree = mode3_catch.build_trial(session)
    assert run(tree) == Status.SUCCESS and run(tree) == Status.SUCCESS
    assert said == [D["throw_prompt"], D[f"{reason}_prompt"], D["throw_prompt"], D["last_fail_prompt"]]
    assert [t.notes for t in published] == [reason, reason]      # 기록은 그대로


def test_last_trial_success_prompt_unchanged():
    _, session, _, said = arrange([HIT])
    session.trials_planned[TrialResult.MODE_CATCH_PAD] = 1
    run(mode3_catch.build_trial(session))
    assert said == [D["throw_prompt"], D["success_prompt"]]


def test_last_trial_counts_skipped_turns():
    # 계획 3회 중 1·2회가 연속 timeout 이면 3회째는 건너뛰므로(기록·안내 없음) 마지막 안내가 엉뚱한 회차에 가지 않는다
    client, session, published, said = arrange([(False, {"failure_reason": "timeout"})] * 2)
    session.trials_planned[TrialResult.MODE_CATCH_PAD] = 3
    tree = mode3_catch.build_trial(session)
    for _ in range(3):
        run(tree)
    assert client.sent == 2 and len(published) == 2 and said == [D["throw_prompt"]] * 2


def test_without_planned_count_never_last():
    fail = (True, {"caught": False, "failure_reason": "missed"})
    _, session, _, said = arrange([fail])
    run(mode3_catch.build_trial(session))                        # trials_planned 없음 (예: 직접 만든 트리) → 보통 문구
    assert said == [D["throw_prompt"], D["missed_prompt"]]


def test_session_tree_records_planned_count():
    _, session, _, _ = arrange([HIT] * 4)
    session_tree.build_session_tree(session, [TrialResult.MODE_CATCH_PAD], 4)
    assert session.trials_planned[TrialResult.MODE_CATCH_PAD] == 4


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
    assert 0 <= float(cfg["throw_prompt_delay_s"]) < float(cfg["throw_prompt_max_s"])
    assert 0 < float(cfg["throw_prompt_after_grip_s"]) < float(cfg["throw_prompt_max_s"])
    assert 0 < float(cfg["throw_prompt_pose_tol_deg"]) <= 10 and 0 <= float(cfg["throw_prompt_settle_s"]) < 5
    assert len(cfg["ready_posj"]) == 6                            # 관절값 비교 기준 (로봇 skills.yaml 과 같은 파일)
    yaml_mode3 = load_skills()["mode3"]
    assert yaml_mode3["intro_prompt"] and yaml_mode3["done_prompt"]
    for key in ("success_prompt", "missed_prompt", "out_of_radius_prompt", "last_fail_prompt"):
        assert yaml_mode3[key], key
    assert "다시" not in yaml_mode3["last_fail_prompt"]           # 마지막 시도 문구에는 '다시 던져' 가 없어야 함


def test_waits_for_exposure_before_throw_prompt():
    # 손목 노출 고정이 적용된 뒤 투구 안내 (#139 박성현) — 그 전엔 안내도 goal 도 안 보냄
    client, session, published, said = arrange([HIT])
    fixed = {"ok": False}
    session.exposure_fixed = lambda: fixed["ok"]
    tree = mode3_catch.build_trial(session)
    for _ in range(5):
        tree.tick_once()
    assert said == [] and client.sent == 0
    fixed["ok"] = True
    assert run(tree) == Status.SUCCESS and client.sent == 1 and said[0] == D["throw_prompt"]


def test_exposure_wait_times_out_and_continues():
    client, session, published, _ = arrange([HIT], {"mode3": {"exposure_wait_s": 0.05}})
    session.exposure_fixed = lambda: False                         # 카메라가 거절 — 기다린 뒤 그냥 진행
    assert run(mode3_catch.build_trial(session)) == Status.SUCCESS
    assert client.sent == 1 and len(published) == 1


# ── 10/16 조정묵: 결과 안내는 판정 피드백(caught/missed) 순간, 투구 안내는 대기 자세에 선 뒤(그리퍼로 패드를 쥔 뒤) ──
JUDGE_CAUGHT = [{"phase": "ready"}, {"phase": "tracking"}, {"phase": "intercept"}, {"phase": "caught"}]
JUDGE_MISSED = [{"phase": "ready"}, {"phase": "tracking"}, {"phase": "intercept"}, {"phase": "missed"}]
NO_MOVE_MISSED = [{"phase": "ready"}, {"phase": "tracking"}, {"phase": "missed"}]      # 반경 밖·저신뢰: 요격 이동 없이 놓침


def start_slow(outcomes, feedbacks, **mode3):
    client, session, published, said = arrange(outcomes, client_type=SlowClient, feedbacks=feedbacks, **mode3)
    tree = mode3_catch.build_trial(session)
    while client.sent < 1:
        tree.tick_once()
    return client, session, published, said, tree


@pytest.mark.parametrize("feedbacks,outcome,at_judgment", [
    (JUDGE_CAUGHT, HIT, "success_prompt"),
    (JUDGE_MISSED, (True, {"caught": False, "failure_reason": "missed"}), "missed_prompt"),
])
def test_result_prompt_at_judgment_feedback_not_repeated_at_result(feedbacks, outcome, at_judgment):
    client, _, published, said, tree = start_slow([outcome], [feedbacks])
    tree.tick_once()
    assert said == [D["throw_prompt"], D[at_judgment]] and tree.status == Status.RUNNING   # 판정 피드백 순간 — 결과(20 s 뒤)를 안 기다림
    tree.tick_once()
    assert said == [D["throw_prompt"], D[at_judgment]]
    client.release()
    assert run(tree) == Status.SUCCESS and len(published) == 1
    assert said == [D["throw_prompt"], D[at_judgment]]                                   # 결과 때 되풀이하지 않음


def test_missed_without_intercept_waits_for_result_reason():
    # 로봇이 안 움직인 놓침은 이유(반경 밖)를 결과가 알려 줌 — 판정 때는 말하지 않고 결과 때 out_of_radius_prompt
    client, _, published, said, tree = start_slow([(True, {"caught": False, "failure_reason": "out_of_radius"})], [NO_MOVE_MISSED])
    for _ in range(3):
        tree.tick_once()
    assert said == [D["throw_prompt"]]
    client.release()
    assert run(tree) == Status.SUCCESS and said == [D["throw_prompt"], D["out_of_radius_prompt"]] and len(published) == 1


def test_last_trial_judgment_feedback_says_short_phrase():
    client, session, _, said, tree_unused = start_slow([(True, {"caught": False, "failure_reason": "missed"})], [JUDGE_MISSED])
    # 위 트리는 trials_planned 없이 만들어짐 — 계획 1회짜리 트리를 새로 만들어 마지막 시도 문구를 본다
    client2, session2, _, said2 = arrange([(True, {"caught": False, "failure_reason": "missed"})], client_type=SlowClient, feedbacks=[JUDGE_MISSED])
    session2.trials_planned[TrialResult.MODE_CATCH_PAD] = 1
    tree = mode3_catch.build_trial(session2)
    while client2.sent < 1:
        tree.tick_once()
    tree.tick_once()
    assert said2 == [D["throw_prompt"], D["last_fail_prompt"]]
    client2.release()
    assert run(tree) == Status.SUCCESS and said2 == [D["throw_prompt"], D["last_fail_prompt"]]


def test_no_feedback_still_prompts_at_result():
    # 옛 로봇(판정 피드백 없음)이면 결과 때 — 기존 동작 그대로
    client, _, _, said, tree = start_slow([HIT], [])
    for _ in range(3):
        tree.tick_once()
    assert said == [D["throw_prompt"]]
    client.release()
    assert run(tree) == Status.SUCCESS and said == [D["throw_prompt"], D["success_prompt"]]


def test_throw_prompt_after_pad_grip_seen():
    # 거치대에서 집는 경우: goal 때 빈손 → 쥔 것이 보인 뒤 throw_prompt_after_grip_s 지나서
    client, session, _, said = arrange([HIT], client_type=SlowClient, throw_prompt_delay_s=5.0, throw_prompt_after_grip_s=0.1, throw_prompt_max_s=10.0)
    grip = {"holding": False}
    session.gripper_holding = lambda: grip["holding"]
    tree = mode3_catch.build_trial(session)
    for _ in range(4):
        tree.tick_once()
    assert client.sent == 1 and said == []
    grip["holding"] = True
    tree.tick_once()
    assert said == []                                             # 쥔 직후 — 아직 대기 자세로 가는 중
    time.sleep(0.12)
    tree.tick_once()
    assert said == [D["throw_prompt"]]
    client.release()
    assert run(tree) == Status.SUCCESS and said == [D["throw_prompt"], D["success_prompt"]]


def test_throw_prompt_uses_delay_when_pad_already_held():
    # 직전 놓침으로 패드를 쥔 채 시작: 대기 자세로 바로 가므로 goal 뒤 throw_prompt_delay_s
    client, session, _, said = arrange([HIT], client_type=SlowClient, throw_prompt_delay_s=0.1, throw_prompt_after_grip_s=5.0, throw_prompt_max_s=10.0)
    session.gripper_holding = lambda: True
    tree = mode3_catch.build_trial(session)
    for _ in range(3):
        tree.tick_once()
    assert client.sent == 1 and said == []
    time.sleep(0.12)
    tree.tick_once()
    assert said == [D["throw_prompt"]]


def test_throw_prompt_at_most_max_s_when_grip_never_seen():
    client, session, _, said = arrange([HIT], client_type=SlowClient, throw_prompt_delay_s=5.0, throw_prompt_after_grip_s=5.0, throw_prompt_max_s=0.1)
    session.gripper_holding = lambda: False                       # 그리퍼 상태 이상 (쥐었는데 안 보임)
    tree = mode3_catch.build_trial(session)
    for _ in range(3):
        tree.tick_once()
    assert said == []
    time.sleep(0.12)
    tree.tick_once()
    assert said == [D["throw_prompt"]]


# ── 10/16 사용자 결정: 투구 안내는 관절값이 ready_posj 허용 오차 안에 들어와 잠깐 머물면 (일정한 값 비교가 아니라 범위) ──
READY = [89.8, 30.0, -121.7, 90.2, -80.2, 88.3]
POSE_CFG = dict(ready_posj=READY, throw_prompt_pose_tol_deg=3.0, throw_prompt_settle_s=0.1,
                throw_prompt_delay_s=5.0, throw_prompt_after_grip_s=5.0, throw_prompt_max_s=10.0)


def start_pose(q, holding=None, **over):
    client, session, _, said = arrange([HIT], client_type=SlowClient, **{**POSE_CFG, **over})
    joints = {"q": q}
    session.joint_deg = lambda: joints["q"]
    session.gripper_holding = lambda: holding
    tree = mode3_catch.build_trial(session)
    for _ in range(3):
        tree.tick_once()
    assert client.sent == 1
    return joints, said, tree


def test_throw_prompt_when_joints_within_tolerance_and_settled():
    joints, said, tree = start_pose([q + d for q, d in zip(READY, (1.0, -2.5, 0.4, 2.9, -1.0, 0.0))])   # 교시값과 조금 다른 도착값
    assert said == []                                             # 들어왔지만 아직 settle 전
    time.sleep(0.12)
    tree.tick_once()
    assert said == [D["throw_prompt"]]


def test_throw_prompt_waits_until_joints_arrive():
    joints, said, tree = start_pose([q + d for q, d in zip(READY, (0.0, 40.0, 0.0, 0.0, 0.0, 0.0))])   # 아직 거치대 쪽
    time.sleep(0.12)
    tree.tick_once()
    assert said == []                                             # 허용 오차 밖 — 시간이 지나도 안내 없음
    joints["q"] = [q + 0.5 for q in READY]
    tree.tick_once()
    time.sleep(0.12)
    tree.tick_once()
    assert said == [D["throw_prompt"]]


def test_throw_prompt_pose_requires_continuous_stay():
    joints, said, tree = start_pose(list(READY))
    joints["q"] = [q + 10.0 for q in READY]                       # 지나가는 중에 잠깐 들어왔다 나감
    time.sleep(0.12)
    tree.tick_once()
    assert said == []
    joints["q"] = list(READY)
    tree.tick_once()
    time.sleep(0.12)
    tree.tick_once()
    assert said == [D["throw_prompt"]]                            # 다시 들어와 머문 뒤


def test_throw_prompt_pose_waits_if_gripper_known_empty():
    joints, said, tree = start_pose(list(READY), holding=False)   # 자세는 맞지만 패드가 없음(집기 전·놓침)
    time.sleep(0.12)
    tree.tick_once()
    assert said == []


def test_throw_prompt_falls_back_to_gripper_when_no_joints():
    joints, said, tree = start_pose(None, holding=True, throw_prompt_delay_s=0.1)   # 관절값 없음 → 이미 쥔 채 시작 → delay_s
    time.sleep(0.12)
    tree.tick_once()
    assert said == [D["throw_prompt"]]
