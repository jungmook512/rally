"""휴식 경계·재개·중복·안전 검사. ROS action 대신 실제 모드 트리에 가짜 클라이언트를 넣는다."""
import time
import uuid
from types import SimpleNamespace as NS

import py_trees
import pytest
import rclpy
from geometry_msgs.msg import Point

from fakes import FakeClient, make_session
from rally_brain.bt_runner import BtRunner
from rally_brain.session_control import SessionControl
from rally_brain.trees.session_tree import build_session_tree
from rally_interfaces.action import Handover, PickCompliant, ResistPush
from rally_interfaces.msg import SafetyStatus, SceneContext, TrackedObject, TrialResult, VoiceIntent


def test_pause_finishes_whole_handover_and_preserves_difficulty():
    ok = (True, {"success": True})
    clients = {PickCompliant: FakeClient(PickCompliant, [ok, ok]),
               Handover: FakeClient(Handover, [ok] * 4)}
    rod = TrackedObject(class_name="rod", confidence=0.9, position=Point(x=0.3, y=0.2))
    session, trials, _, _ = make_session(clients, objects=[rod])
    control = SessionControl()
    control.start("session")
    session.difficulty[5] = 2
    session.allow_trial = lambda: control.at_boundary(10)
    tree = build_session_tree(session, [5], 2)
    tree.tick_once()  # 집기 시작
    assert clients[PickCompliant].sent == 1
    control.pause("음성")
    for _ in range(20):
        tree.tick_once()
    assert control.phase == "paused"
    assert len(trials) == 1
    assert [g.direction for g in clients[Handover].goals] == [Handover.Goal.DIR_GIVE, Handover.Goal.DIR_RECEIVE]
    assert session.difficulty == {5: 2}
    pause_id = control.pause_id
    assert control.resume("session", pause_id, "")[0]
    assert not control.resume("session", pause_id, "")[0]
    for _ in range(20):
        tree.tick_once()
        if tree.status == py_trees.common.Status.SUCCESS:
            break
    assert len(trials) == 2 and clients[PickCompliant].sent == 2
    assert session.difficulty == {5: 2}


def test_pause_blocks_next_mode_in_same_tick_and_last_trial_ends():
    ok = (True, {"success": True})
    clients = {PickCompliant: FakeClient(PickCompliant, [ok]),
               Handover: FakeClient(Handover, [ok, ok]), ResistPush: FakeClient(ResistPush, [ok])}
    rod = TrackedObject(class_name="rod", confidence=0.9, position=Point(x=0.3, y=0.2))
    session, trials, _, _ = make_session(clients, objects=[rod])
    control = SessionControl()
    control.start("session")
    session.allow_trial = lambda: control.at_boundary(10)
    session.after_trial = lambda *_: control.pause("시도 끝의 휴식 요청")
    tree = build_session_tree(session, [5, 7], 1)
    for _ in range(20):
        tree.tick_once()
    assert len(trials) == 1 and session.current_mode == 5
    assert control.phase == "paused" and clients[ResistPush].sent == 0
    control.resume("session", control.pause_id, "")
    for _ in range(20):
        tree.tick_once()
        if tree.status == py_trees.common.Status.SUCCESS:
            break
    assert len(trials) == 2 and tree.status == py_trees.common.Status.SUCCESS


@pytest.fixture
def node():
    rclpy.init(args=["--ros-args", "-p", "check_readiness:=false", "-p", "dashboard_control:=false"])
    node = BtRunner()
    node._say = lambda _: None
    session, _, _, ctx = make_session({})
    session.session_id = str(uuid.uuid4())
    session.current_mode = 7
    session.difficulty[7] = 2
    node._session = session
    node.control.start(session.session_id)
    node._on_context(ctx)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def paused(node):
    assert node._pause("test")[0]
    assert not node._allow_trial()
    return node.control.session_id, node.control.pause_id


def test_resume_rejects_pending_stale_and_missing_readiness(node):
    assert node._pause("test")[0]
    assert not node._resume(node.control.session_id, "")[0]  # 진행 중 시도를 끊지 않는다
    sid, pid = paused(node)
    node._context_rx = time.monotonic() - 10
    assert not node._resume(sid, pid)[0]
    node._context_rx = time.monotonic()
    node.readiness = NS(missing=lambda: ["service missing"])
    assert not node._resume(sid, pid)[0]
    node.readiness = None
    assert not node._resume(str(uuid.uuid4()), pid)[0]
    assert not node._resume(sid, str(uuid.uuid4()))[0]
    assert node._resume(sid, pid)[0]
    assert node._session.difficulty == {7: 2}
    assert not node._resume(sid, pid)[0]


def test_stop_latches_even_if_context_recovers(node):
    sid, pid = paused(node)
    ctx = SceneContext()
    ctx.safety.level = SafetyStatus.LEVEL_STOP
    node._on_context(ctx)
    ctx = SceneContext()
    ctx.safety.level = SafetyStatus.LEVEL_NORMAL
    node._on_context(ctx)
    assert not node._resume(sid, pid)[0]
    assert not node._allow_trial()


def test_voice_resume_needs_new_stamp_and_pause_keeps_level(node):
    old = VoiceIntent(intent="resume")
    old.header.stamp = node.get_clock().now().to_msg()
    paused(node)
    node._on_intent(old)
    assert node.control.phase == "paused"
    node._on_intent(VoiceIntent(intent="resume"))  # stamp 미설정
    assert node.control.phase == "paused"
    assert not node._request_switch(5, "dashboard")[0]
    node._on_intent(VoiceIntent(intent="easier"))
    node._after_trial(node._session, TrialResult(mode=7, success=True))
    assert node._session.difficulty == {7: 2} and node._pending_intent is None
    new = VoiceIntent(intent="resume")
    new.header.stamp = node.get_clock().now().to_msg()
    node._on_intent(new)
    assert node.control.phase == "running"
    node._on_intent(new)
    assert node._session.difficulty == {7: 2}


@pytest.mark.parametrize("cancel", [False, True])
def test_actual_session_loop_stops_while_paused(node, monkeypatch, cancel):
    """휴식 중에도 실제 실행 루프의 STOP/취소 검사가 살아 있다."""
    from rally_brain import bt_runner
    from rally_brain.behaviours import Do, WaitUntil
    outcomes = []

    def build(session, modes, n):
        session.current_mode = 7
        node._pause("test")

        def while_paused():
            if node.control.phase == "paused":
                if cancel:
                    goal.is_cancel_requested = True
                else:
                    ctx = SceneContext()
                    ctx.safety.level = SafetyStatus.LEVEL_STOP
                    node._on_context(ctx)
            return False

        # 첫 tick에서 pause를 넣도록 한다 (_run_session이 control.start 한 뒤).
        return py_trees.composites.Selector("test", memory=False, children=[
            py_trees.composites.Sequence("pause", memory=True, children=[
                Do("request", lambda: node._pause("test") and None),
                WaitUntil("rest", lambda: session.allow_trial() or while_paused()),
            ])])

    monkeypatch.setattr(bt_runner, "build_session_tree", build)
    goal = NS(request=NS(modes=[7], trials_per_mode=2), is_cancel_requested=False,
              publish_feedback=lambda _: None, abort=lambda: outcomes.append("abort"),
              canceled=lambda: outcomes.append("cancel"), succeed=lambda: outcomes.append("success"))
    node._run_session(goal)
    assert outcomes == ["cancel" if cancel else "abort"]
    assert node.control.phase == "ended"
    assert not node._resume(node.control.session_id, "old")[0]


def test_mode1_explicit_difficulty_and_rest_preserve_level(node):
    session = node._session
    session.current_mode = 1
    session.difficulty = {1: 0}
    trial = TrialResult(mode=1, success=True)
    session.trials = [trial] * 4
    node._after_trial(session, trial)
    assert session.difficulty[1] == 0  # 받기 후보 연속 성공으로 자동 승급 금지
    node._pending_intent = 'harder'
    node._after_trial(session, trial)
    assert session.difficulty[1] == 1
    sid, pid = paused(node)
    node._pending_intent = 'easier'
    node._after_trial(session, trial)
    assert session.difficulty[1] == 1
    assert node._resume(sid, pid)[0]
    assert session.difficulty[1] == 1
    for _ in range(2):
        node._pending_intent = 'easier'
        node._after_trial(session, trial)
    assert session.difficulty[1] == 0


def test_rest_proposal_accept_uses_existing_boundary(node):
    response = node._test_proposal(None, NS())
    assert response.success and node.control.phase == 'running'
    pid = node.control_snapshot()['proposal']['proposal_id']
    assert node._respond_proposal(node.control.session_id, pid, 'accept')[0]
    assert node.control.phase == 'pause_pending'
    assert not node._respond_proposal(node.control.session_id, pid, 'accept')[0]
    assert not node._allow_trial()
    assert node.control.phase == 'paused' and node._session.difficulty == {7: 2}


@pytest.mark.parametrize('change', ['reject', 'expire', 'stop', 'stale', 'cancel', 'mode', 'difficulty', 'pause'])
def test_rest_proposal_never_applies_after_invalidation(node, change):
    assert node._test_proposal(None, NS()).success
    pid = node.proposals.item['proposal_id']
    if change == 'reject':
        assert node._respond_proposal(node.control.session_id,pid,'reject')[0]
    elif change == 'expire':
        node.proposals.deadline = time.monotonic() - 1
    elif change == 'stop':
        ctx = SceneContext()
        ctx.safety.level = SafetyStatus.LEVEL_STOP
        node._on_context(ctx)
    elif change == 'stale':
        node._context_rx = time.monotonic() - 10
    elif change == 'cancel':
        node._cancel_session(None)
    elif change == 'mode':
        node._session.current_mode = 5
    elif change == 'difficulty':
        node._session.difficulty[7] = 3
    elif change == 'pause':
        node._pause('직접 요청')
    assert not node._respond_proposal(node.control.session_id,pid,'accept')[0]
    assert node.control.phase == ('pause_pending' if change == 'pause' else 'running')


@pytest.mark.parametrize("mode", [1, 5])
@pytest.mark.parametrize("last_trial", [False, True])
def test_accepted_proposal_survives_deadline_until_whole_trial_finishes(node, monkeypatch, mode, last_trial):
    """응답 시계가 끝나도 수락 예약은 유지하며, 회수/RECEIVE·기록 뒤에만 쉰다."""
    from rally_brain import behaviours
    from rally_interfaces.action import ReturnBall
    from test_mode1 import arrange, hand, OK

    clock = [0.0]
    monkeypatch.setattr(behaviours, "time", NS(monotonic=lambda: clock[0]))
    node.proposals.clock = lambda: clock[0]
    count = 1 if last_trial else 2
    if mode == 1:
        first, session, trials = arrange([OK] * count)
        session.skills_cfg["mode1"]["balls_per_cycle"] = count
        session.get_context().hand = hand(1.0)
        session.get_context().hand.distance_to_centerline = 0.15
        final = session.action_client(ReturnBall, "/rally/skill/return_ball")
    else:
        ok = (True, {"success": True})
        first = FakeClient(PickCompliant, [ok] * count)
        final = FakeClient(Handover, [ok] * (2 * count))
        rod = TrackedObject(class_name="rod", confidence=0.9, position=Point(x=0.3, y=0.2))
        session, trials, _, _ = make_session({PickCompliant: first, Handover: final}, objects=[rod])
    session.session_id = node.control.session_id
    session.difficulty[mode] = 2
    session.allow_trial = node._allow_trial
    session.after_trial = node._after_trial
    node._session = session
    node._on_context(session.get_context())
    tree = build_session_tree(session, [mode], count)
    tree.tick_once()
    assert first.sent == 1 and trials == []
    assert node._test_proposal(None, NS()).success
    proposal_id = node.proposals.item["proposal_id"]
    assert node._respond_proposal(session.session_id, proposal_id, "accept")[0]

    clock[0] = 21.0  # 제안의 원래 20초 기한 이후에도 예약은 취소되지 않는다.
    assert node.control_snapshot()["proposal"]["status"] == "accepted"
    assert node.control.phase == "pause_pending"
    for decision in ("accept", "reject"):
        assert not node._respond_proposal(session.session_id, proposal_id, decision)[0]
    if mode == 1:
        tree.tick_once()  # 타격 완료, 받기 관측 시작
        clock[0] += 1.0
        tree.tick_once()  # 받기 timeout, 손 이탈 대기
        assert first.sent == 1 and final.sent == 0 and trials == []
        assert node.control.phase == "pause_pending"
        session.get_context().hand = hand(1.0)
        tree.tick_once()  # 이탈 확인 뒤 회수 요청
        assert final.sent == 1 and trials == []
    else:
        tree.tick_once()  # 집기 완료, GIVE 요청
        assert final.sent == 1 and trials == []
        tree.tick_once()  # GIVE 완료, RECEIVE 요청
        assert final.sent == 2 and trials == []
    assert node.control.phase == "pause_pending"
    for _ in range(10):
        tree.tick_once()
        if tree.status == py_trees.common.Status.SUCCESS:
            break
    assert len(trials) == 1 and first.sent == 1
    assert session.difficulty == {mode: 2}
    assert node.proposals.item["status"] == "accepted"
    if last_trial:
        assert tree.status == py_trees.common.Status.SUCCESS
        assert node.control.phase == "pause_pending"  # 실제 실행 루프가 이어서 finish한다.
    else:
        assert tree.status == py_trees.common.Status.RUNNING
        assert node.control.phase == "paused"
