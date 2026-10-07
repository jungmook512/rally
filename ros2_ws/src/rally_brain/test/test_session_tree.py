"""세션 트리 단위 테스트 — 스킬 서버 없이 트리 구조·기록 규칙만 본다."""
import time
from types import SimpleNamespace as NS

import py_trees

from rally_brain.behaviours import Do, WaitUntil
from rally_brain.bt_runner import default_modes
from rally_brain.trees.session_tree import Session, build_session_tree
from rally_interfaces.msg import SafetyStatus, SceneContext, TrialResult

Status = py_trees.common.Status


class FakeNode:
    def get_logger(self):
        return NS(info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None)

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: NS(sec=0, nanosec=0)))


def make_session(ctx=None):
    published = []
    s = Session(node=FakeNode(), session_id="s1", get_context=lambda: ctx, publish=published.append,
                action_client=lambda *_: None)
    return s, published


def test_default_modes():
    assert default_modes(True) == [1, 3, 5, 7]
    assert default_modes(False) == [1, 5, 7]


def test_record_fills_index_and_session():
    s, published = make_session()
    s.record(TrialResult(mode=5, success=True))
    s.record(TrialResult(mode=5, success=False))
    assert [t.trial_index for t in published] == [1, 2]
    assert all(t.session_id == "s1" for t in published)
    assert s.success_rate() == 0.5


def test_safe_and_hand():
    s, _ = make_session(None)
    assert not s.safe() and not s.hand_visible()
    ctx = SceneContext()
    ctx.safety.level = SafetyStatus.LEVEL_STOP
    ctx.hand.detected = True
    s, _ = make_session(ctx)
    assert not s.safe() and s.hand_visible()


def test_unknown_modes_are_skipped():
    s, _ = make_session()
    tree = build_session_tree(s, [1, 7], 3)
    tree.tick_once()
    assert tree.status == Status.SUCCESS
    assert [c.name for c in tree.children] == ["모드 1 공 굴리기 건너뜀", "모드 7 팔씨름 건너뜀"]


def test_wait_until_timeout():
    w = WaitUntil("w", lambda: False, timeout_s=0.01)
    w.tick_once()
    assert w.status == Status.RUNNING
    time.sleep(0.02)
    w.tick_once()
    assert w.status == Status.FAILURE


def test_do_false_is_failure():
    d = Do("d", lambda: False)
    d.tick_once()
    assert d.status == Status.FAILURE
