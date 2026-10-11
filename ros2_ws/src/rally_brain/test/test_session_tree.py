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
    tree = build_session_tree(s, [3], 3)
    tree.tick_once()
    assert tree.status == Status.SUCCESS
    assert [c.name for c in tree.children] == ["모드 3 캐치볼 건너뜀"]


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


def test_modes_after_switch():
    from rally_brain.trees.session_tree import modes_after_switch as after
    assert after([1, 5, 7], 5, None) == [7]           # "다음 거"
    assert after([1, 5, 7], 7, None) == []            # 마지막 모드에서 다음 → 세션 끝
    assert after([1, 5, 7], 5, 7) == [7]              # 앞으로 건너뛰기
    assert after([1, 5, 7], 5, 1) == [1, 5, 7]        # 뒤로 돌아가면 거기서부터 다시
    assert after([5, 7], 5, 1) == [1, 7]              # 순서에 없던 모드 → 그 뒤에 원래 남은 순서
    assert after([1, 5, 7], None, None) == [1, 5, 7]  # 아직 시작 전


def test_mode_block_sets_current_mode():
    s, _ = make_session()
    tree = build_session_tree(s, [5], 1)
    assert s.modes == [5] and s.current_mode is None
    tree.tick_once()                                   # "모드 5 시작" → 손 대기(컨텍스트 없음, RUNNING)
    assert s.current_mode == 5
    assert tree.status == Status.RUNNING


# ---------- 자세 기준 잡기 (#83) ----------
class Done:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value


def baseline_session(success=True):
    s, _ = make_session()
    said, calls = [], []
    s.say = said.append
    s.skills_cfg = {"brain": {"body_baseline_prompt_wait_s": 0.0, "body_baseline_hold_s": 0.0}}

    def call_async(req):
        calls.append(req)
        return Done(NS(success=success, message="" if success else "no body"))
    s.service_client = lambda _type, name: NS(name=name, service_is_ready=lambda: True, call_async=call_async)
    return s, said, calls


def tick_until(tree, cond, n=20):
    for _ in range(n):
        tree.tick_once()
        if cond():
            return True
    return False


def test_body_baseline_runs_once_before_first_mode():
    s, said, calls = baseline_session()
    tree = build_session_tree(s, [5], 1)
    assert tree.children[0].name == "자세 기준 잡기"
    assert tick_until(tree, lambda: s.current_mode == 5)
    assert said[0] == "기준 자세를 잡을게요. 바르게 서 주세요."
    assert len(calls) == 1 and s.body_baseline_done
    # 모드 전환으로 트리를 새로 만들어도 다시 잡지 않는다
    assert build_session_tree(s, [7], 1).children[0].name != "자세 기준 잡기"


def test_body_baseline_failure_does_not_block_session():
    s, _, calls = baseline_session(success=False)
    tree = build_session_tree(s, [5], 1)
    assert tick_until(tree, lambda: s.current_mode == 5)
    assert len(calls) == 1 and s.body_baseline_done


def test_body_baseline_skipped_without_service_client():
    s, _ = make_session()
    assert build_session_tree(s, [5], 1).children[0].name != "자세 기준 잡기"
