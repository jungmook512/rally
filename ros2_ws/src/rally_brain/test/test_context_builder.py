"""context_builder 단위 테스트 — 순수 함수는 SimpleNamespace, build() 는 노드를 띄워 직접 호출 (spin 없음)."""
from types import SimpleNamespace as NS

import pytest
import rclpy

from rally_brain.context_builder import NO_SAFETY_REASON, ContextBuilder, is_fresh, success_rate
from rally_interfaces.msg import HandState, SafetyStatus, SceneContext, TrialResult, VoiceIntent


def test_is_fresh():
    assert is_fresh(1_000_000_000, 1_500_000_000, 1.0)
    assert not is_fresh(1_000_000_000, 2_500_000_000, 1.0)
    assert not is_fresh(None, 1_000_000_000, 1.0)


def test_success_rate():
    assert success_rate([]) == 0.0
    assert success_rate([NS(success=True), NS(success=False), NS(success=True), NS(success=True)]) == 0.75


@pytest.fixture
def node():
    rclpy.init()
    n = ContextBuilder()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def trial(session="s1", index=1, success=True, mode=TrialResult.MODE_HANDOVER):
    return TrialResult(session_id=session, trial_index=index, success=success, mode=mode)


def test_nothing_received_is_stop_and_empty(node):
    ctx = node.build()
    assert ctx.safety.level == SafetyStatus.LEVEL_STOP
    assert ctx.safety.reason == NO_SAFETY_REASON
    assert ctx.robot_state == SceneContext.ROBOT_STOPPED
    assert not ctx.hand.detected
    assert ctx.recent_trials == []
    assert ctx.session_id == ""


def test_fresh_messages_are_carried(node):
    now = node._now_ns()
    node._safety = (SafetyStatus(level=SafetyStatus.LEVEL_SLOW, reason="nudge"), now)
    node._hand = (HandState(detected=True, handedness="right"), now)
    node._intent = (VoiceIntent(intent="tired"), now)
    ctx = node.build()
    assert ctx.safety.level == SafetyStatus.LEVEL_SLOW
    assert ctx.robot_state == SceneContext.ROBOT_IDLE
    assert ctx.hand.detected and ctx.hand.handedness == "right"
    assert ctx.last_intent.intent == "tired"


def test_stale_messages_are_dropped(node):
    old = node._now_ns() - 10_000_000_000
    node._safety = (SafetyStatus(level=SafetyStatus.LEVEL_NORMAL), old)
    node._hand = (HandState(detected=True), old)
    node._intent = (VoiceIntent(intent="tired"), old)
    ctx = node.build()
    assert ctx.safety.reason == NO_SAFETY_REASON
    assert not ctx.hand.detected
    assert ctx.last_intent.intent == ""


def test_recent_trials_keep_last_n(node):
    for i in range(1, 8):
        node._on_trial(trial(index=i, success=i % 2 == 0))
    ctx = node.build()
    assert [t.trial_index for t in ctx.recent_trials] == [3, 4, 5, 6, 7]
    assert ctx.recent_success_rate == pytest.approx(0.4)
    assert (ctx.session_id, ctx.trial_index, ctx.mode) == ("s1", 7, TrialResult.MODE_HANDOVER)


def test_new_session_clears_trials(node):
    node._on_trial(trial(session="s1", index=1))
    node._on_trial(trial(session="s2", index=1, success=False))
    ctx = node.build()
    assert [t.session_id for t in ctx.recent_trials] == ["s2"]
    assert ctx.recent_success_rate == 0.0
