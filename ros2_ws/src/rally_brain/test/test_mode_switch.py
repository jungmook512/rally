"""bt_runner 모드 전환 — #51 리뷰: 같은 발화 한 번만, STOP 중 거절, 시도 도중엔 미룸, 마지막 모드 안내."""
import py_trees
import pytest
import rclpy

from rally_brain.bt_runner import BtRunner
from rally_brain.trees.session_tree import Session, build_session_tree
from rally_interfaces.msg import SafetyStatus, SceneContext, VoiceIntent


@pytest.fixture
def node():
    rclpy.init()
    n = BtRunner()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def start(node, modes=(1, 5, 7), current=5):
    ctx = SceneContext()
    ctx.safety.level = SafetyStatus.LEVEL_NORMAL
    node._context = ctx
    s = Session(node=node, session_id="s", get_context=lambda: node._context, publish=lambda t: None,
                action_client=node._action_client)
    s.modes, s.current_mode = list(modes), current
    node._session = s
    return s


def next_mode(sec, slot=None):
    m = VoiceIntent(intent="next_mode")
    if slot is not None:
        m.slot_keys, m.slot_values = ["mode"], [str(slot)]
    m.header.stamp.sec = sec
    return m


def test_same_utterance_is_handled_once(node):
    start(node)
    node._on_intent(next_mode(100))
    assert node._pending_switch == (None, "음성")
    node._pending_switch = None
    node._on_intent(next_mode(100))           # 같은 발화가 또 옴 → 무시 (모드 두 칸 건너뛰기 방지)
    assert node._pending_switch is None
    node._on_intent(next_mode(101))           # 새 발화는 처리
    assert node._pending_switch == (None, "음성")


def test_stop_rejects_switch(node):
    start(node)
    node._context.safety.level = SafetyStatus.LEVEL_STOP
    ok, message = node._request_switch(7, "set_mode")
    assert not ok and "안전 정지" in message and node._pending_switch is None


def test_last_mode_next_says_session_ends(node):
    start(node, current=7)
    ok, message = node._request_switch(None, "set_mode")
    assert ok and "세션이 끝남" in message


def test_new_request_replaces_pending(node):
    start(node)
    node._request_switch(7, "set_mode")
    ok, message = node._request_switch(1, "set_mode")
    assert ok and "앞선 요청은 취소" in message and node._pending_switch == (1, "set_mode")


def test_switch_waits_for_trial_end(node):
    s = start(node)
    tree = py_trees.trees.BehaviourTree(build_session_tree(s, [5, 7], 2))
    node._request_switch(7, "set_mode")
    s.begin_trial()                                         # 로봇이 시도 중 (스킬 사이일 수도)
    assert node._apply_switch(tree, s, 2) is tree and node._pending_switch is not None
    s.trial_active = False                                  # record() 가 하는 것과 같음
    new = node._apply_switch(tree, s, 2)
    assert new is not tree and node._pending_switch is None
    assert s.modes == [7]


def test_mode1_only_explicit_intent_changes_difficulty(node):
    from rally_interfaces.msg import TrialResult, VoiceIntent
    s = start(node, modes=(1,), current=1)
    s.trials = [TrialResult(mode=1, success=True)] * 4
    node._after_trial(s, s.trials[-1])
    assert s.difficulty.get(1, 0) == 0
    node._on_intent(VoiceIntent(intent='harder'))
    node._after_trial(s, s.trials[-1])
    assert s.difficulty[1] == 1
    node._on_intent(VoiceIntent(intent='easier'))
    node._after_trial(s, s.trials[-1])
    assert s.difficulty[1] == 0
    node._on_intent(VoiceIntent(intent='easier'))
    node._after_trial(s, s.trials[-1])
    assert s.difficulty[1] == 0
