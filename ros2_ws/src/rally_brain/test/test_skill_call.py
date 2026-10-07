"""SkillCall 중단(terminate) 경로 — PR #41 리뷰 1·2번 재현. 가짜 future 로 타이밍과 무관하게 확인한다."""
from types import SimpleNamespace as NS

import py_trees
from action_msgs.msg import GoalStatus

from rally_brain.behaviours import SkillCall

Status = py_trees.common.Status


class Future:
    def __init__(self, value=None, done=False):
        self._value, self._done, self._callbacks = value, done, []

    def done(self):
        return self._done

    def result(self):
        return self._value

    def add_done_callback(self, cb):
        self._callbacks.append(cb)
        if self._done:
            cb(self)

    def finish(self, value):
        self._value, self._done = value, True
        for cb in self._callbacks:
            cb(self)


class GoalHandle:
    def __init__(self, result_future, accepted=True):
        self.accepted = accepted
        self.result_future = result_future
        self.cancelled = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancelled += 1


class Client:
    def __init__(self, send_future):
        self.send_future = send_future

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        return self.send_future


def node():
    log = NS(info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None)
    return NS(get_logger=lambda: log)


def make(send_future):
    calls = {"result": [], "interrupt": []}
    skill = SkillCall("s", node(), Client(send_future), lambda: None,
                      on_result=calls["result"].append, on_interrupt=calls["interrupt"].append)
    return skill, calls


def run_until_sent(skill, ticks=1):
    for _ in range(ticks):
        skill.tick_once()


def test_stop_after_server_already_aborted_is_recorded():
    # 리뷰 1: skill_server 가 STOP 을 먼저 보고 abort → bt_runner 가 중단할 땐 결과가 이미 와 있음
    result = Future()
    handle = GoalHandle(result)
    skill, calls = make(Future(handle, done=True))
    run_until_sent(skill, 2)                      # 보내기 → 수락 확인
    result.finish(NS(status=GoalStatus.STATUS_ABORTED, result=None))
    skill.stop(Status.INVALID)                    # update 가 결과를 읽기 전에 중단
    assert calls["interrupt"] == ["aborted"]
    assert handle.cancelled == 0                  # 이미 끝난 goal 은 취소 안 함


def test_stop_after_unread_success_records_real_result():
    result = Future()
    skill, calls = make(Future(GoalHandle(result), done=True))
    run_until_sent(skill, 2)
    result.finish(NS(status=GoalStatus.STATUS_SUCCEEDED, result="ok"))
    skill.stop(Status.INVALID)
    assert calls["result"] == ["ok"] and calls["interrupt"] == []


def test_stop_while_running_cancels_and_records():
    handle = GoalHandle(Future())
    skill, calls = make(Future(handle, done=True))
    run_until_sent(skill, 2)
    skill.stop(Status.INVALID)
    assert handle.cancelled == 1 and calls["interrupt"] == ["interrupted"]


def test_stop_before_accept_cancels_once_accepted():
    # 리뷰 2: 보낸 직후 수락 전에 중단 → 수락되자마자 취소
    send = Future()
    skill, calls = make(send)
    run_until_sent(skill, 1)
    skill.stop(Status.INVALID)
    assert calls["interrupt"] == ["interrupted"]
    handle = GoalHandle(Future())
    send.finish(handle)
    assert handle.cancelled == 1


def test_handled_result_is_not_recorded_twice():
    result = Future(NS(status=GoalStatus.STATUS_ABORTED, result=None), done=True)
    skill, calls = make(Future(GoalHandle(result), done=True))
    run_until_sent(skill, 2)                      # update 가 abort 를 읽고 FAILURE (기록은 Selector 의 '실패 기록')
    assert skill.status == Status.FAILURE
    skill.stop(Status.INVALID)
    assert calls["interrupt"] == [] and calls["result"] == []


def test_not_sent_is_not_recorded():
    skill, calls = make(Future())
    skill.stop(Status.INVALID)
    assert calls == {"result": [], "interrupt": []}
