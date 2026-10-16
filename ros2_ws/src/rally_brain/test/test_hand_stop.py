"""손 근접 STOP 대기 (#98 4번) + 복구 대기 (10/15) — 순수 함수 + bt_runner 실행 루프를 가짜 상황판으로."""
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
def test_waitable_only_listed_hand_reasons_in_wait_modes():
    for reason in ("hand_near_robot", "hand_near_centerline", "hand_lost_near"):
        for mode in (1, 3, 5, 7):                                   # 10/15: 모든 모드
            assert hand_stop.waitable(reason, mode, CFG)
    for reason in ("estop", "voice_stop", "force_limit", "nudge", "workspace_limit",
                   "wrench_missing", "tcp_missing", "wrench_lost", "hand_something_new", ""):
        assert not hand_stop.waitable(reason, 1, CFG)               # → 복구 대기 (세션 종료 아님)
    assert not hand_stop.waitable("hand_near_robot", None, CFG)
    # 안전 상태 끊김은 어느 모드든 기다리고(모드 없음 포함) 스킬도 취소하지 않는다
    assert hand_stop.waitable("no_safety_status", None, CFG) and hand_stop.no_cancel("no_safety_status", CFG)
    assert not hand_stop.no_cancel("hand_near_robot", CFG)


def test_config_from_skills_yaml_and_prompts_are_speakable():
    cfg = hand_stop.config({"brain": {"hand_stop_resume_hold_s": 2.0}})
    assert cfg["hand_stop_resume_hold_s"] == 2.0 and cfg["hand_stop_wait_modes"] == [1, 3, 5, 7]
    for key in ("hand_stop_prompt", "recovery_prompt", "clear_gripper_prompt"):
        assert phrase_problem(CFG[key]) is None, key
    assert hand_stop.stop_prompt("hand_near_robot", CFG) == CFG["hand_stop_prompt"]
    assert hand_stop.stop_prompt("force_limit", CFG) == CFG["recovery_prompt"]
    assert hand_stop.stop_prompt("no_safety_status", CFG) == ""                   # 끊김은 말하지 않음


def test_recovery_stages_safety_then_gripper_then_done():
    cfg = {**CFG, "hand_stop_resume_hold_s": 1.0, "recovery_prompt_s": 5.0}
    r = hand_stop.Recovery("force_limit", 0.0, cfg, check_gripper=True)
    assert r.prompt(0.0) == CFG["recovery_prompt"] and r.prompt(1.0) is None      # 바로 한 번, 간격 안엔 다시 안 함
    assert r.update(STOP, 1.0, None) == "wait" and r.update(NORMAL, 2.0, None) == "wait"   # NORMAL 유지 전
    assert r.update(NORMAL, 3.0, False) == "wait" and r.stage == "gripper"          # NORMAL 1 s → 그리퍼 단계, 물건 있음
    assert r.prompt(3.0) == CFG["clear_gripper_prompt"]
    assert r.update(STOP, 3.5, False) == "wait" and r.stage == "safety"              # 비우다 손이 가까워 STOP → 안전부터
    assert r.update(NORMAL, 4.0, False) == "wait" and r.update(NORMAL, 5.5, True) == "done"   # 비면 끝
    r2 = hand_stop.Recovery("hand_near_robot", 0.0, cfg, check_gripper=False)
    assert r2.prompt(0.0) == CFG["hand_stop_prompt"]
    assert r2.update(NORMAL, 1.5, None) == "wait" and r2.update(NORMAL, 2.6, None) == "done"   # NORMAL 1 s 뒤, 그리퍼 확인 없음 (모드 3·7)
    r3 = hand_stop.Recovery("force_limit", 0.0, {**cfg, "recovery_max_s": 10.0}, check_gripper=False)
    assert r3.update(STOP, 11.0, None) == "timeout"                                  # 이때만 세션 종료
    r4 = hand_stop.Recovery("estop", 0.0, {**cfg, "clear_gripper_max_s": 3.0}, check_gripper=True)
    assert r4.update(NORMAL, 1.0, False) == "wait" and r4.update(NORMAL, 2.5, False) == "wait" and r4.stage == "gripper"
    assert r4.update(NORMAL, 6.0, False) == "timeout"                                # 그리퍼 단계 3 s 넘게 안 비움


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
    node._hand_cfg = {**node._hand_cfg, "hand_stop_resume_hold_s": 0.2, "hand_stop_prompt_s": 100.0, "recovery_prompt_s": 100.0}
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

    def build(session, modes, n, first_remaining=None):
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


def recovering_children(node, feed, first, steps, seen):
    """첫 트리는 first(session) (복구로 멈춤), 복구 뒤 다시 만든 트리는 바로 끝. seen 에 대기 상태를 기록."""
    builds = {"n": 0}

    def children(session):
        builds["n"] += 1
        if builds["n"] == 1:
            script(feed, steps)
            return first(session)
        seen["rebuilt"] = builds["n"]
        seen["wait_at_rebuild"] = node.control.hand_wait
        return [Do("끝", lambda: None)]
    return children


def test_hand_stop_while_robot_moving_cancels_trial_then_next_trial(node, monkeypatch):
    # 10/15: 이어 갈 수 없는 단계의 손 STOP → 세션 종료 대신 스킬 취소 → 손 빠짐 → 같은 모드 다음 시도 (트리 다시)
    feed = Feed(node)
    client = PendingClient()
    seen = {}

    def first(session):
        return [SkillCall("밀기", node, client, PushBall.Goal, on_result=lambda _: None)]

    try:
        out = run_session(node, monkeypatch, recovering_children(node, feed, first, [(0.15, STOP, "hand_near_robot"), (0.3, NORMAL, "")], seen))
    finally:
        feed.close()
    assert out == ["success"] and client.sent == 1 and seen["rebuilt"] == 2
    assert node.said == [CFG["hand_stop_prompt"]] and node.control.hand_wait == "" and seen["wait_at_rebuild"] == ""


@pytest.mark.parametrize("reason", ["force_limit", "estop", "wrench_lost", "tcp_missing"])
def test_other_stop_waits_for_reset_then_next_trial(node, monkeypatch, reason):
    # 10/15: 래치 STOP 도 세션을 끝내지 않는다 — reset 부탁 → NORMAL 유지 → 같은 모드 남은 시도
    feed = Feed(node)
    seen = {}

    def first(session):
        def probe():
            time.sleep(0.3)
            seen["waiting"] = node.control.hand_wait
            seen["switch_blocked"] = not node._request_switch(5, "test")[0]
        threading.Thread(target=probe, daemon=True).start()
        return [WaitUntil("대기", lambda: False)]

    try:
        out = run_session(node, monkeypatch, recovering_children(node, feed, first, [(0.1, STOP, reason), (0.5, NORMAL, "")], seen))
    finally:
        feed.close()
    assert out == ["success"] and seen["rebuilt"] == 2
    assert seen["waiting"] == reason and seen["switch_blocked"]
    assert node.said == [CFG["recovery_prompt"]]


def test_other_stop_times_out_then_session_ends(node, monkeypatch):
    feed = Feed(node)
    node._hand_cfg = {**node._hand_cfg, "recovery_max_s": 0.3}

    def children(session):
        script(feed, [(0.1, STOP, "force_limit")])
        return [WaitUntil("대기", lambda: False)]

    try:
        out = run_session(node, monkeypatch, children)
    finally:
        feed.close()
    assert out == ["abort"]


def test_hand_stop_in_mode7_waits_then_continues_same_tree(node, monkeypatch):
    # 10/15: 손 STOP 대기는 모든 모드 (전엔 모드 1·5 만 — 그 밖은 세션 종료)
    feed = Feed(node)

    def children(session):
        script(feed, [(0.1, STOP, "hand_near_robot"), (0.3, NORMAL, "")])
        return [WaitUntil("대기", lambda: time.monotonic() > t0 + 0.8), Do("끝", lambda: None)]

    t0 = time.monotonic()
    try:
        out = run_session(node, monkeypatch, children, mode=7)
    finally:
        feed.close()
    assert out == ["success"] and node.said == [CFG["hand_stop_prompt"]]


def test_latched_stop_during_hand_wait_goes_to_recovery(node, monkeypatch):
    feed = Feed(node)
    seen = {}
    first = lambda session: [WaitUntil("대기", lambda: False)]   # noqa: E731
    try:
        out = run_session(node, monkeypatch, recovering_children(
            node, feed, first, [(0.1, STOP, "hand_near_robot"), (0.2, STOP, "estop"), (0.2, NORMAL, "")], seen))
    finally:
        feed.close()
    assert out == ["success"] and seen["rebuilt"] == 2
    assert node.said == [CFG["hand_stop_prompt"], CFG["recovery_prompt"]]


def test_recovery_waits_until_gripper_is_emptied(node, monkeypatch):
    # 모드 1 복구: 그리퍼에 물건이 있으면 사람이 뺄 때까지 (clear_gripper_prompt) — 다음 스킬이 그리퍼를 열어 떨어뜨리지 않게
    from sensor_msgs.msg import JointState
    feed = Feed(node)
    seen = {}

    def grip(holding: bool):
        node._on_gripper(JointState(name=["rg2_width_mm", "rg2_grip", "rg2_target_mm"],
                                    position=[20.0 if holding else 0.5, 1.0 if holding else 0.0, 0.0]))

    def first(session):
        grip(True)
        def keep():
            for _ in range(45):            # 0.9 s 동안 '쥠' 유지한 뒤 비움 (복구는 0.1 STOP + 0.2 NORMAL + 0.2 유지 ≈ 0.5 s 에 그리퍼 단계)
                grip(True); time.sleep(0.02)
            seen["said_before_clear"] = list(node.said)
            grip(False)
        threading.Thread(target=keep, daemon=True).start()
        return [WaitUntil("대기", lambda: False)]

    try:
        out = run_session(node, monkeypatch, recovering_children(node, feed, first, [(0.1, STOP, "nudge"), (0.2, NORMAL, "")], seen))
    finally:
        feed.close()
    assert out == ["success"] and seen["rebuilt"] == 2
    assert seen["said_before_clear"] == [CFG["recovery_prompt"], CFG["clear_gripper_prompt"]]


def test_no_safety_status_waits_without_cancelling_skill(node, monkeypatch):
    # 안전 상태 끊김(와이파이): 스킬은 그대로 두고 기다렸다가 상태가 돌아오면 결과를 받는다 — 안내 없음
    feed = Feed(node)

    class HoldResultClient:
        """goal 수락 뒤 succeed() 전까지 결과를 붙잡는 스킬 (상태 끊김 동안에도 로봇은 돌고 있음)."""
        def __init__(self):
            self.sent, self.ok = 0, threading.Event()

        def server_is_ready(self):
            return True

        def succeed(self):
            self.ok.set()

        def send_goal_async(self, goal, feedback_callback=None):
            self.sent += 1
            fut = NS(done=lambda: self.ok.is_set(),
                     result=lambda: NS(status=GoalStatus.STATUS_SUCCEEDED, result=PushBall.Result(success=True)))
            return Done(NS(accepted=True, get_result_async=lambda: fut, cancel_goal_async=lambda: None))

    client = HoldResultClient()
    results = []

    def children(session):
        script(feed, [(0.1, STOP, "no_safety_status"), (0.4, NORMAL, "")])
        threading.Timer(0.3, client.succeed).start()
        return [SkillCall("밀기", node, client, PushBall.Goal, on_result=results.append)]

    try:
        out = run_session(node, monkeypatch, children, mode=7)
    finally:
        feed.close()
    assert out == ["success"] and client.sent == 1 and len(results) == 1 and node.said == []


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


def test_return_resend_skips_hand_clear_precondition():
    # 10/12 실로봇: 회수하러 가다 손 STOP → 손 뺌 → NORMAL. 선 자세에선 손이 안 보여 can_send(손 이탈 확인)가 영영 거짓 → 재전송 안 됨.
    # 재전송은 resume_ok 만 본다 (#115). 처음 보낼 때는 can_send 그대로
    client = FakeClient(ReturnBall, [(False, {"failure_reason": "hand_near_robot"}), (True, {"success": True})],
                        feedbacks=[[{"phase": "find"}]])
    gate, clear = {"ok": False}, {"ok": True}
    results = []
    b = SkillCall("공 회수", fake_node(), client, ReturnBall.Goal, on_result=results.append,
                  can_send=lambda: clear["ok"], wait_for_precondition=True,
                  hand_resume=mode1_push.return_hand_resume(REASONS), resume_ok=lambda: gate["ok"])
    b.tick_once(); b.tick_once()                    # 처음: 손 이탈 확인됨 → 보냄 → 손 STOP abort
    assert client.sent == 1 and b.status == Status.RUNNING
    clear["ok"] = False                             # 이제 손이 안 보임
    b.tick_once()
    assert client.sent == 1                         # NORMAL 전엔 안 보냄
    gate["ok"] = True
    b.tick_once(); b.tick_once()
    assert client.sent == 2 and b.status == Status.SUCCESS and results[-1].success
    b2 = SkillCall("공 회수", fake_node(), FakeClient(ReturnBall, [(True, {"success": True})]), ReturnBall.Goal,
                   on_result=lambda _: None, can_send=lambda: False, wait_for_precondition=True)
    for _ in range(3):
        b2.tick_once()
    assert b2.client.sent == 0 and b2.status == Status.RUNNING   # 처음 보낼 때는 손 이탈 확인을 기다림


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


def test_user_speaking_skips_until_listening_ends_or_goes_stale(node):
    # 10/15: intent_node 가 듣는 중(/rally/voice/listening)이면 모드 선택 안내를 건너뛴다. 20 s 넘게 true 면 끝을 못 받은 것으로
    assert not node._user_speaking()
    node._listening = (True, time.monotonic())
    assert node._user_speaking()
    node._listening = (True, time.monotonic() - 30.0)
    assert not node._user_speaking()
    node._listening = (False, time.monotonic())
    assert not node._user_speaking()


def test_service_stop_reason():
    from rally_brain.behaviours import service_stop_reason
    assert service_stop_reason("hand_near_robot") == "hand_near_robot"                 # 이동 중 STOP (SkillAbort 사유)
    assert service_stop_reason("safety STOP (hand_lost_near)") == "hand_lost_near"     # 요청 받을 때 이미 STOP
    assert service_stop_reason("safety STOP (force_limit,hand_stop_dev_off)") == "force_limit"
    assert service_stop_reason("observe pose [0, 0]") == "observe"
    assert service_stop_reason("") == ""
