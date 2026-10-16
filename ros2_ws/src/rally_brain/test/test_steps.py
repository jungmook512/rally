"""trees/steps.py 공용 단계 (10/15) — 사람에게 부탁하고 기다리기(OperatorWait)·지연(Delay)·재시도(retrying). 가짜 시계로."""
from types import SimpleNamespace as NS

import py_trees

from fakes import make_session
from rally_brain.behaviours import Do
from rally_brain.trees import steps

Status = py_trees.common.Status


def clock(monkeypatch):
    t = [0.0]
    monkeypatch.setattr(steps, "time", NS(monotonic=lambda: t[0]))
    return t


def tree_of(b):
    """행동을 그대로 틱한다 (b.tick_once / b.status) — 다른 트리 테스트와 같은 방식."""
    return NS(tick_once=b.tick_once, root=b)


def test_operator_wait_says_prompt_repeats_and_ends_after_wait(monkeypatch):
    t = clock(monkeypatch)
    session, _, said, _ = make_session({})
    w = steps.OperatorWait("정리 대기", session, "치워 주세요", 10.0, prompt_every_s=4.0)
    tree = tree_of(w)
    tree.tick_once()
    assert tree.root.status == Status.RUNNING and said == ["치워 주세요"]
    t[0] = 4.5
    tree.tick_once()
    assert said == ["치워 주세요"] * 2                      # prompt_every_s 마다 다시
    t[0] = 10.0
    tree.tick_once()
    assert tree.root.status == Status.SUCCESS and len(said) == 2


def test_operator_wait_until_condition_ends_early_and_gripper_blocks(monkeypatch):
    t = clock(monkeypatch)
    session, _, said, _ = make_session({})
    flag = {"ok": False}
    w = steps.OperatorWait("물건 대기", session, "", 30.0, until=lambda: flag["ok"])
    tree = tree_of(w)
    tree.tick_once()
    assert tree.root.status == Status.RUNNING and said == []    # 빈 문구면 말하지 않음
    flag["ok"] = True
    tree.tick_once()
    assert tree.root.status == Status.SUCCESS
    session.gripper_empty = lambda: False
    g = steps.OperatorWait("그리퍼 대기", session, "빼 주세요", 1.0, gripper_empty=True, max_s=5.0)
    tree = tree_of(g)
    t[0] = 0.0
    tree.tick_once()
    t[0] = 2.0
    tree.tick_once()
    assert tree.root.status == Status.RUNNING                   # 시간이 지나도 그리퍼에 물건이 있으면 더 기다림
    session.gripper_empty = lambda: True
    tree.tick_once()
    assert tree.root.status == Status.SUCCESS
    session.gripper_empty = lambda: False
    tree = tree_of(steps.OperatorWait("그리퍼 대기", session, "빼 주세요", 1.0, gripper_empty=True, max_s=5.0))
    t[0] = 10.0
    tree.tick_once()
    t[0] = 16.0
    tree.tick_once()
    assert tree.root.status == Status.SUCCESS                   # max_s 넘기면 경고만 남기고 진행 (세션을 끝내지 않음)


def test_delay_restarts_each_time_and_retrying_tries_in_order(monkeypatch):
    t = clock(monkeypatch)
    d = steps.Delay("쉼", 2.0)
    tree = tree_of(d)
    tree.tick_once(); assert tree.root.status == Status.RUNNING
    t[0] = 2.0
    tree.tick_once(); assert tree.root.status == Status.SUCCESS
    t[0] = 2.5
    tree.tick_once(); assert tree.root.status == Status.RUNNING   # 다시 시작하면 처음부터 잼
    calls = []
    outcomes = {1: False, 2: False, 3: True}
    r = steps.retrying("자세", lambda k: Do(f"시도 {k}", lambda k=k: calls.append(k) or outcomes[k]), 3, 1.0)
    tree = tree_of(r)
    t[0] = 10.0
    tree.tick_once()
    assert calls == [1] and tree.root.status == Status.RUNNING   # 실패 → 1 s 쉼
    t[0] = 11.0
    tree.tick_once()
    assert calls == [1, 2] and tree.root.status == Status.RUNNING
    t[0] = 12.0
    tree.tick_once()
    assert calls == [1, 2, 3] and tree.root.status == Status.SUCCESS
    r2 = steps.retrying("자세", lambda k: Do(f"시도 {k}", lambda: False), 2, 0.0)
    tree = tree_of(r2)
    tree.tick_once()
    assert tree.root.status == Status.FAILURE                   # 모두 실패


def test_pose_config_defaults_and_override():
    assert steps.pose_config({}) == (2, 3.0)
    assert steps.pose_config({"brain": {"pose_tries": 0, "pose_retry_wait_s": 1}}) == (1, 1.0)
