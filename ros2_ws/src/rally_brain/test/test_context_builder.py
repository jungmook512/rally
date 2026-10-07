"""context_builder 단위 테스트 — 순수 함수는 SimpleNamespace, build() 는 노드를 띄워 직접 호출 (spin 없음)."""

import pytest
import rclpy

from rally_brain.context_builder import NO_SAFETY_REASON, ContextBuilder, is_fresh, same_utterance
from rally_interfaces.msg import HandState, SafetyStatus, SceneContext, TrialResult, VoiceIntent


def test_is_fresh():
    assert is_fresh(1_000_000_000, 1_500_000_000, 1.0)
    assert not is_fresh(1_000_000_000, 2_500_000_000, 1.0)
    assert not is_fresh(None, 1_000_000_000, 1.0)


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


def test_success_rate_uses_metrics_definition(node):
    # 마지막 시도와 같은 모드만, no_hand·interrupted 는 빼고 (metrics.yaml — 규칙 레이어와 같은 정의)
    node._on_trial(trial(index=1, success=True))
    node._on_trial(TrialResult(session_id="s1", mode=TrialResult.MODE_PUSH_BALL, trial_index=2, success=False))
    node._on_trial(trial(index=3, success=False))
    m = trial(index=4, success=False)
    m.notes = "no_hand"
    node._on_trial(m)
    ctx = node.build()
    assert ctx.recent_success_rate == pytest.approx(0.5)   # 모드 5 의 1·3 번만


def test_new_session_clears_trials(node):
    node._on_trial(trial(session="s1", index=1))
    node._on_trial(trial(session="s2", index=1, success=False))
    ctx = node.build()
    assert [t.session_id for t in ctx.recent_trials] == ["s2"]
    assert ctx.recent_success_rate == 0.0


def intent(text="tired", sec=100, nanosec=0):
    m = VoiceIntent(intent=text)
    m.header.stamp.sec, m.header.stamp.nanosec = sec, nanosec
    return m


def test_same_utterance():
    assert same_utterance(intent(sec=100), intent(sec=100))
    assert not same_utterance(intent(sec=100), intent(sec=101))
    assert not same_utterance(intent(sec=0), intent(sec=0))      # stamp 없으면 판별 안 함


def test_repeated_utterance_does_not_extend_hold(node):
    node._on_intent(intent(sec=100))
    first_received = node._intent[1]
    node._intent = (node._intent[0], first_received - 10_000_000_000)   # 10 s 전에 받은 것으로
    node._on_intent(intent(sec=100))                                    # 같은 발화 재수신
    assert node.build().last_intent.intent == ""                        # 유지 시간이 다시 늘지 않음
    node._on_intent(intent(text="stop_session", sec=105))               # 새 발화
    assert node.build().last_intent.intent == "stop_session"


def test_fatigue_score_connected(node):
    # 같은 모드 최근 시도 + 음성 tired → metrics.fatigue_score (#52 리뷰 3)
    assert node.build().fatigue_score == 0.0
    for i in range(1, 4):
        node._on_trial(trial(index=i, success=False))     # 성공률 0 → success_drop (0.25)
    assert node.build().fatigue_score == pytest.approx(0.25)
    node._intent = (intent("tired", sec=5), node._now_ns())
    assert node.build().fatigue_score == 1.0
