"""모드 1 손 근접 STOP 대기 (#98 4번) — 순수 함수 + bt_runner 실행 루프를 가짜 상황판으로."""
import threading
import time
import uuid
from types import SimpleNamespace as NS

import py_trees
import pytest
import rclpy

from fakes import Done
from rally_brain import bt_runner, hand_stop
from rally_brain.behaviours import Do, ServiceCall, SkillCall, WaitUntil
from rally_brain.bt_runner import BtRunner
from rally_brain.speech import phrase_problem
from rally_interfaces.action import PushBall
from rally_interfaces.msg import SafetyStatus, SceneContext

Status = py_trees.common.Status
NORMAL, SLOW, STOP = SafetyStatus.LEVEL_NORMAL, SafetyStatus.LEVEL_SLOW, SafetyStatus.LEVEL_STOP
CFG = hand_stop.config({})


# ---------- 순수 ----------
def test_waitable_only_listed_hand_reasons_in_mode1():
    for reason in ("hand_near_robot", "hand_near_centerline", "hand_lost_near"):
        assert hand_stop.waitable(reason, 1, CFG)
    for reason in ("estop", "voice_stop", "force_limit", "nudge", "workspace_limit",
                   "wrench_missing", "tcp_missing", "wrench_lost", "hand_something_new", ""):
        assert not hand_stop.waitable(reason, 1, CFG)
    assert not hand_stop.waitable("hand_near_robot", 5, CFG)
    assert not hand_stop.waitable("hand_near_robot", None, CFG)


def test_config_from_skills_yaml_and_prompt_is_speakable():
    cfg = hand_stop.config({"brain": {"hand_stop_resume_hold_s": 2.0}})
    assert cfg["hand_stop_resume_hold_s"] == 2.0 and cfg["hand_stop_wait_modes"] == [1]
    assert phrase_problem(CFG["hand_stop_prompt"]) is None


class PendingClient:
    """goal 은 수락되지만 결과가 오지 않는 스킬 (로봇이 움직이는 중)."""
    def __init__(self):
        self.sent = 0

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent += 1
        never = NS(done=lambda: False)
        return Done(NS(accepted=True, get_result_async=lambda: never, cancel_goal_async=lambda: None))


def fake_node():
    log = NS(info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None)
    return NS(get_logger=lambda: log)


def test_robot_busy_only_after_goal_or_request_sent():
    waiting = SkillCall("밀기", fake_node(), PendingClient(), PushBall.Goal, on_result=lambda _: None,
                        can_send=lambda: False, wait_for_precondition=True)
    waiting.tick_once()
    assert waiting.status == Status.RUNNING and not hand_stop.robot_busy([waiting])   # precondition 대기 = 로봇 서 있음
    moving = SkillCall("밀기", fake_node(), PendingClient(), PushBall.Goal, on_result=lambda _: None)
    moving.tick_once()
    assert moving.status == Status.RUNNING and hand_stop.robot_busy([moving])
    srv = ServiceCall("손 보기 자세로", NS(service_is_ready=lambda: True, call_async=lambda _: NS(done=lambda: False)), object)
    srv.tick_once()
    assert srv.status == Status.RUNNING and hand_stop.robot_busy([srv])


def test_wait_needs_normal_held_and_restarts_on_new_stop():
    w = hand_stop.HandStopWait("hand_near_robot", 0.0, {**CFG, "hand_stop_resume_hold_s": 1.0})
    assert not w.update(STOP, 0.5)
    assert not w.update(NORMAL, 1.0)
    assert not w.update(SLOW, 1.5)            # SLOW 는 재진행 조건 아님 (10/11 결정) → 다시 처음부터
    assert not w.update(NORMAL, 2.0)
    w.stop_seen("hand_lost_near")             # tick 사이 짧은 STOP
    assert not w.update(NORMAL, 2.9)
    assert w.update(NORMAL, 3.9)


def test_shift_timers_moves_running_waits_only():
    w = WaitUntil("받기 대기", lambda: False, timeout_s=1.0)
    w.tick_once()
    t0 = w._t0
    done = WaitUntil("끝남", lambda: True)
    done.tick_once()
    hand_stop.shift_timers([w, done], 5.0)
    assert w._t0 == pytest.approx(t0 + 5.0)


# ---------- bt_runner 실행 루프 ----------
@pytest.fixture
def node():
    rclpy.init(args=["--ros-args", "-p", "check_readiness:=false", "-p", "dashboard_control:=false",
                     "-p", "tick_hz:=50.0"])
    node = BtRunner()
    node.said = []
    node._say = node.said.append
    node._hand_cfg = {**node._hand_cfg, "hand_stop_resume_hold_s": 0.2, "hand_stop_prompt_s": 100.0}
    yield node
    node.destroy_node()
    rclpy.shutdown()


class Feed:
    """상황판을 20 ms 마다 보내는 스레드. set(level, reason) 으로 바꾼다."""
    def __init__(self, node):
        self.node, self.level, self.reason, self.alive = node, NORMAL, "", True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def set(self, level, reason=""):
        self.level, self.reason = level, reason

    def run(self):
        while self.alive:
            ctx = SceneContext()
            ctx.safety.level, ctx.safety.reason = self.level, self.reason
            self.node._on_context(ctx)
            time.sleep(0.02)

    def close(self):
        self.alive = False
        self.thread.join()


def run_session(node, monkeypatch, build_children, mode=1):
    """build_children(session) → 트리 자식들. 세션 결과(succeed/abort/canceled)를 돌려준다."""
    outcomes = []

    def build(session, modes, n):
        session.current_mode = mode
        return py_trees.composites.Sequence("test", memory=True, children=build_children(session))

    monkeypatch.setattr(bt_runner, "build_session_tree", build)
    goal = NS(request=NS(modes=[mode], trials_per_mode=1), is_cancel_requested=False,
              publish_feedback=lambda _: None, abort=lambda: outcomes.append("abort"),
              canceled=lambda: outcomes.append("cancel"), succeed=lambda: outcomes.append("success"))
    node.goal = goal
    node._run_session(goal)
    return outcomes


def script(feed, steps):
    """steps: [(지연 s, level, reason)] 를 별도 스레드에서 차례로."""
    def run():
        for delay, level, reason in steps:
            time.sleep(delay)
            feed.set(level, reason)
    threading.Thread(target=run, daemon=True).start()


def test_idle_hand_stop_waits_then_continues_same_tree(node, monkeypatch):
    feed = Feed(node)
    flags = {"seen_wait": False}

    def children(session):
        def gate():
            flags["seen_wait"] |= bool(node.control.hand_wait)
            return flags.get("open", False)
        script(feed, [(0.1, STOP, "hand_near_robot"), (0.3, NORMAL, "")])
        threading.Timer(0.8, lambda: flags.update(open=True)).start()   # 재진행(약 0.6 s) 뒤에 조건이 참
        return [WaitUntil("손 시작 위치 대기", gate, timeout_s=0.4), Do("끝", lambda: None)]

    try:
        out = run_session(node, monkeypatch, children)
    finally:
        feed.close()
    # timeout 0.4 s 인데 0.8 s 에 참 — 기다린 시간(약 0.5 s)을 빼 주지 않으면 0.6 s 재진행 때 바로 timeout
    assert out == ["success"]
    assert node.said == [node._hand_cfg["hand_stop_prompt"]]
    assert node.control.hand_wait == ""


def test_hand_stop_while_robot_moving_ends_session(node, monkeypatch):
    feed = Feed(node)
    client = PendingClient()

    def children(session):
        script(feed, [(0.15, STOP, "hand_near_robot")])
        return [SkillCall("밀기", node, client, PushBall.Goal, on_result=lambda _: None)]

    try:
        out = run_session(node, monkeypatch, children)
    finally:
        feed.close()
    assert out == ["abort"] and client.sent == 1


@pytest.mark.parametrize("reason", ["force_limit", "estop", "wrench_lost", "tcp_missing"])
def test_other_stop_ends_session(node, monkeypatch, reason):
    feed = Feed(node)

    def children(session):
        script(feed, [(0.1, STOP, reason)])
        return [WaitUntil("대기", lambda: False)]

    try:
        out = run_session(node, monkeypatch, children)
    finally:
        feed.close()
    assert out == ["abort"]


def test_hand_stop_outside_mode1_ends_session(node, monkeypatch):
    feed = Feed(node)

    def children(session):
        script(feed, [(0.1, STOP, "hand_near_robot")])
        return [WaitUntil("대기", lambda: False)]

    try:
        out = run_session(node, monkeypatch, children, mode=7)   # 10/11: 모드 5 는 손 STOP 대기 대상에 들어감 (skills.yaml)
    finally:
        feed.close()
    assert out == ["abort"]


def test_latched_stop_during_wait_ends_session(node, monkeypatch):
    feed = Feed(node)

    def children(session):
        script(feed, [(0.1, STOP, "hand_near_robot"), (0.2, STOP, "estop"), (0.1, NORMAL, "")])
        return [WaitUntil("대기", lambda: False)]

    try:
        out = run_session(node, monkeypatch, children)
    finally:
        feed.close()
    assert out == ["abort"]


def test_slow_keeps_waiting_and_blocks_switch_keeps_rest_until_cancel(node, monkeypatch):
    feed = Feed(node)
    checks = {}

    def children(session):
        script(feed, [(0.1, STOP, "hand_near_robot"), (0.2, SLOW, "hand_near_centerline")])

        def probe():
            time.sleep(0.6)
            checks["waiting"] = node.control.hand_wait
            checks["switch"] = node._request_switch(5, "test")[0]
            checks["rest"] = node._pause("음성 tired")[0]
            checks["phase"] = node.control.phase
            node.goal.is_cancel_requested = True
        threading.Thread(target=probe, daemon=True).start()
        return [WaitUntil("대기", lambda: False)]

    try:
        out = run_session(node, monkeypatch, children)
    finally:
        feed.close()
    assert checks == {"waiting": "hand_near_robot", "switch": False, "rest": True, "phase": "pause_pending"}
    assert out == ["cancel"]


# ---------- 로봇 동작 중 손 STOP → 단계 보고 다시 보냄 / 끝난 것으로 / 종료 ----------
from action_msgs.msg import GoalStatus   # noqa: E402
from fakes import FakeClient   # noqa: E402
from rally_brain.trees import mode1_push   # noqa: E402
from rally_interfaces.action import ReturnBall   # noqa: E402

REASONS = CFG["hand_stop_wait_reasons"]


def test_mode1_phase_decisions():
    push = mode1_push.push_hand_resume(REASONS)
    assert push("hand_near_robot", []) == "resend"
    assert push("hand_near_robot", ["approach", "pickup"]) == "resend"          # 쥐었어도 스킬이 집기를 건너뜀
    for after in (["approach", "pickup", "place"], ["place", "approach", "contact"], ["push"], ["observe"]):
        assert push("hand_near_robot", after) is None                            # 놓은 뒤 ~ 치기·복귀: 공 위치 애매
    assert push("force_limit", ["pickup"]) is None and push("pickup_failed", []) is None
    ret = mode1_push.return_hand_resume(REASONS)
    assert ret("hand_lost_near", ["find"]) == "resend" and ret("hand_near_robot", ["find", "out_of_reach"]) == "resend"
    assert ret("hand_near_robot", ["find", "pickup"]) is None and ret("hand_near_robot", ["pickup", "to_rack"]) is None
    assert ret("hand_near_robot", ["find", "pickup", "to_rack", "drop", "hand_view"]) == "done"
    assert ret("estop", ["find"]) is None


def skill(client, decide, gate):
    results = []
    b = SkillCall("밀기", fake_node(), client, PushBall.Goal, on_result=results.append,
                  hand_resume=decide, resume_ok=lambda: gate["ok"])
    return b, results


def test_skillcall_resends_same_goal_only_after_resume_ok():
    client = FakeClient(PushBall, [(False, {"failure_reason": "hand_near_robot"}), (True, {"success": True})],
                        feedbacks=[[{"phase": "approach"}, {"phase": "pickup"}]])
    gate = {"ok": False}
    b, results = skill(client, mode1_push.push_hand_resume(REASONS), gate)
    b.tick_once(); b.tick_once()                    # goal 1 → abort(손 STOP) → 다시 보낼 준비
    assert b.status == Status.RUNNING and client.sent == 1
    for _ in range(3):
        b.tick_once()
    assert client.sent == 1                         # 손이 빠지기 전엔 안 보냄
    gate["ok"] = True
    b.tick_once(); b.tick_once()
    assert client.sent == 2 and b.status == Status.SUCCESS and results[0].success


def test_skillcall_after_place_fails_and_return_hand_view_is_done():
    client = FakeClient(PushBall, [(False, {"failure_reason": "hand_near_robot"})], feedbacks=[[{"phase": "place"}]])
    b, results = skill(client, mode1_push.push_hand_resume(REASONS), {"ok": True})
    b.tick_once(); b.tick_once()
    assert b.status == Status.FAILURE and client.sent == 1 and results == []
    client = FakeClient(ReturnBall, [(False, {"failure_reason": "hand_near_robot"})],
                        feedbacks=[[{"phase": "find"}, {"phase": "pickup"}, {"phase": "drop"}, {"phase": "hand_view"}]])
    results = []
    b = SkillCall("공 회수", fake_node(), client, ReturnBall.Goal, on_result=results.append,
                  hand_resume=mode1_push.return_hand_resume(REASONS))
    b.tick_once(); b.tick_once()
    assert b.status == Status.SUCCESS and client.sent == 1 and results[0].success


def test_skillcall_resend_limit():
    client = FakeClient(PushBall, [(False, {"failure_reason": "hand_near_robot"})] * 3)
    b = SkillCall("밀기", fake_node(), client, PushBall.Goal, on_result=lambda _: None,
                  hand_resume=mode1_push.push_hand_resume(REASONS), max_resends=2)
    for _ in range(6):
        b.tick_once()
        if b.status != Status.RUNNING:
            break
    assert client.sent == 3 and b.status == Status.FAILURE


def test_servicecall_resends_on_hand_stop_only():
    responses = [NS(success=False, message="hand_near_robot"), NS(success=True, message="ok")]
    calls = []
    client = NS(service_is_ready=lambda: True, call_async=lambda r: calls.append(r) or Done(responses[len(calls) - 1]))
    gate = {"ok": False}
    srv = ServiceCall("손 보기 자세로", client, object, resend_reasons=tuple(REASONS), resume_ok=lambda: gate["ok"])
    srv.tick_once(); srv.tick_once()
    assert srv.status == Status.RUNNING and len(calls) == 1
    gate["ok"] = True
    srv.tick_once(); srv.tick_once()
    assert srv.status == Status.SUCCESS and len(calls) == 2
    other = ServiceCall("손 보기 자세로", NS(service_is_ready=lambda: True,
                        call_async=lambda r: Done(NS(success=False, message="ik_failed"))), object,
                        resend_reasons=tuple(REASONS))
    other.tick_once(); other.tick_once()
    assert other.status == Status.FAILURE


class StopAbortClient:
    """첫 goal 은 feedback(pickup) 뒤 결과를 붙잡고 있다가, 테스트가 release() 하면 손 STOP abort 로 끝남. 다음 goal 은 성공."""
    def __init__(self):
        self.sent, self.released = 0, threading.Event()

    def server_is_ready(self):
        return True

    def release(self):
        self.released.set()

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent += 1
        if self.sent == 1:
            feedback_callback(NS(feedback=PushBall.Feedback(phase="pickup")))
            fut = NS(done=lambda: self.released.is_set(),
                     result=lambda: NS(status=GoalStatus.STATUS_ABORTED, result=PushBall.Result(failure_reason="hand_near_robot")))
        else:
            fut = Done(NS(status=GoalStatus.STATUS_SUCCEEDED, result=PushBall.Result(success=True)))
        return Done(NS(accepted=True, get_result_async=lambda: fut, cancel_goal_async=lambda: None))


def test_moving_hand_stop_waits_and_resends_in_session_loop(node, monkeypatch):
    feed = Feed(node)
    client = StopAbortClient()
    results = []

    def children(session):
        # 0.1 s: 손 STOP → 스킬이 abort(결과는 STOP 뒤 도착) → 0.4 s: NORMAL → hold 뒤 같은 goal 재전송 → 성공
        script(feed, [(0.1, STOP, "hand_near_robot"), (0.3, NORMAL, "")])
        threading.Timer(0.2, client.release).start()
        return [SkillCall("밀기", node, client, PushBall.Goal, on_result=results.append,
                          hand_resume=mode1_push.push_hand_resume(REASONS), resume_ok=session.robot_resume_ok)]

    try:
        out = run_session(node, monkeypatch, children)
    finally:
        feed.close()
    assert out == ["success"] and client.sent == 2 and results[0].success
