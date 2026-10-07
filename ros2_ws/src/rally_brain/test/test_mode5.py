"""모드 5 실패 분기 — 손 대기 시간 제한·안내, 파지 실패 재시도 1회. 가짜 스킬 클라이언트로 로봇 없이 확인한다."""
import time
from types import SimpleNamespace as NS

import py_trees
import pytest
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time

from rally_brain import behaviours
from rally_brain.trees import mode5_handover
from rally_brain.trees.session_tree import Session
from rally_interfaces.action import Handover
from rally_interfaces.msg import SafetyStatus, SceneContext

Status = py_trees.common.Status
FAST = {"mode5": {"hand_wait_s": 0.05, "hand_wait_tries": 2}}


class Done:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value

    def add_done_callback(self, cb):
        cb(self)


class FakeClient:
    """outcomes: (succeeded, success, failure_reason) 를 goal 마다 하나씩."""
    def __init__(self, outcomes, ready=True):
        self.outcomes = list(outcomes)
        self.ready = ready
        self.sent = 0

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.sent += 1
        succeeded, success, reason = self.outcomes.pop(0)
        status = GoalStatus.STATUS_SUCCEEDED if succeeded else GoalStatus.STATUS_ABORTED
        result = Handover.Result(success=success, failure_reason=reason, handover_time_s=1.0)
        handle = NS(accepted=True, get_result_async=lambda: Done(NS(status=status, result=result)),
                    cancel_goal_async=lambda: None)
        return Done(handle)


def make_session(client, hand=True, cfg=FAST):
    log = NS(info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None)
    node = NS(get_logger=lambda: log, get_clock=lambda: NS(now=lambda: NS(to_msg=Time)))
    ctx = SceneContext()
    ctx.safety.level = SafetyStatus.LEVEL_NORMAL
    ctx.hand.detected = hand
    published, said = [], []
    session = Session(node=node, session_id="s", get_context=lambda: ctx, publish=published.append,
                      action_client=lambda *_: client, skills_cfg=cfg, say=said.append)
    return session, published, said


def run(session, max_s=2.0):
    tree = mode5_handover.build_trial(session)
    t0 = time.monotonic()
    while time.monotonic() - t0 < max_s:
        tree.tick_once()
        if tree.status != Status.RUNNING:
            return tree.status
        time.sleep(0.005)
    return tree.status


def metric(trial, key):
    return trial.metric_values[list(trial.metric_keys).index(key)]


def test_success_first_try():
    client = FakeClient([(True, True, "")])
    session, published, said = make_session(client)
    assert run(session) == Status.SUCCESS
    assert [t.success for t in published] == [True]
    assert metric(published[0], "attempts") == 1
    assert said == []


def test_timeout_then_success_is_one_trial():
    # 실제 스킬은 실패하면 abort — 결과 메시지의 failure_reason 으로 재시도 판단
    client = FakeClient([(False, False, "timeout"), (True, True, "")])
    session, published, said = make_session(client)
    assert run(session) == Status.SUCCESS
    assert client.sent == 2
    assert len(published) == 1 and published[0].success
    assert metric(published[0], "attempts") == 2
    assert said == [mode5_handover.DEFAULTS["retry_prompt"]]


def test_retry_only_once():
    client = FakeClient([(False, False, "no_object"), (False, False, "no_object")])
    session, published, _ = make_session(client)
    assert run(session) == Status.SUCCESS          # 시도는 끝남 (실패로 기록)
    assert client.sent == 2
    assert len(published) == 1
    assert not published[0].success and published[0].notes == "no_object"
    assert metric(published[0], "attempts") == 2


def test_safety_stop_is_not_retried():
    client = FakeClient([(False, False, "safety_stop")])
    session, published, said = make_session(client)
    assert run(session) == Status.SUCCESS
    assert client.sent == 1
    assert published[0].notes == "safety_stop"
    assert said == []


def test_no_hand_prompts_then_skips():
    client = FakeClient([])
    session, published, said = make_session(client, hand=False)
    assert run(session) == Status.SUCCESS
    assert client.sent == 0
    assert said == [mode5_handover.DEFAULTS["hand_prompt"]]   # 첫 대기 뒤 한 번 안내
    assert [(t.success, t.notes) for t in published] == [(False, "no_hand")]


def test_no_hand_not_counted_by_rules():
    client = FakeClient([])
    session, _, _ = make_session(client, hand=False)
    run(session)
    assert session.trials and session.trials_for_rules(5) == []


def test_server_missing_fails_session(monkeypatch):
    # 서버가 없으면 재시도하지 않고 세션 실패 (시도 기록 없음). 3 s 기다리지 않게 시계를 빨리 돌린다
    clock = [0.0]

    def fast():
        clock[0] += 1.0
        return clock[0]

    monkeypatch.setattr(behaviours, "time", NS(monotonic=fast))
    client = FakeClient([], ready=False)
    session, published, _ = make_session(client)
    assert run(session) == Status.FAILURE
    assert client.sent == 0 and published == []


def test_config_defaults_and_override():
    cfg = mode5_handover.mode5_config({"mode5": {"max_retries": 0}})
    assert cfg["max_retries"] == 0 and cfg["hand_wait_tries"] == 2
    assert not mode5_handover.should_retry("timeout", 1, cfg)
    assert mode5_handover.should_retry("timeout", 1, mode5_handover.DEFAULTS)
    assert not mode5_handover.should_retry("controller_unresponsive", 1, mode5_handover.DEFAULTS)


@pytest.mark.parametrize("text", [mode5_handover.DEFAULTS["hand_prompt"], mode5_handover.DEFAULTS["retry_prompt"]])
def test_prompts_avoid_stop_words(text):
    assert "그만" not in text and "멈춰" not in text   # INTERFACES #45
