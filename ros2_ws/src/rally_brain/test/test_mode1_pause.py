"""모드 1 회수까지 한 시도: 휴식은 회수 뒤, 재개는 같은 난이도로 다음 공부터."""
from types import SimpleNamespace as NS

import pytest

from fakes import Status
from test_mode1 import arrange, hand, OK
from rally_brain import behaviours
from rally_brain.session_control import SessionControl
from rally_brain.trees.session_tree import build_session_tree
from rally_interfaces.action import ReturnBall


@pytest.mark.parametrize('request_at', ['push', 'clear', 'return'])
def test_pause_waits_for_whole_mode1_then_resumes(request_at, monkeypatch):
    clock = [0.]
    monkeypatch.setattr(behaviours, 'time', NS(monotonic=lambda: clock[0]))
    client, session, trials = arrange([OK, OK])
    session.skills_cfg['mode1']['balls_per_cycle'] = 2
    session.difficulty[1] = 2
    control = SessionControl()
    control.start('session')
    session.allow_trial = lambda: control.at_boundary(10)
    ret = session.action_client(ReturnBall, '/rally/skill/return_ball')
    session.get_context().hand = hand(1.0)
    session.get_context().hand.distance_to_centerline = .20
    tree = build_session_tree(session, [1], 2)
    tree.tick_once()  # 타격 goal
    if request_at == 'push': control.pause('test')
    tree.tick_once()  # 타격 결과·받기 관측
    clock[0] = 1.
    tree.tick_once()  # 받기 timeout·손 이탈 대기
    if request_at == 'clear': control.pause('test')
    clock[0] = 100.
    tree.tick_once()
    assert tree.status == Status.RUNNING and session.trial_active
    assert client.sent == 1 and ret.sent == 0 and trials == []
    if request_at != 'return':
        assert control.phase == 'pause_pending'
    session.get_context().hand = hand(1.0)
    tree.tick_once()  # 이탈 확인·회수 goal
    assert ret.sent == 1 and trials == []
    if request_at == 'return': control.pause('test')
    for _ in range(4): tree.tick_once()
    assert control.phase == 'paused' and not session.trial_active
    assert len(trials) == 1 and client.sent == ret.sent == 1
    assert session.difficulty == {1: 2}
    assert control.resume('session', control.pause_id, '')[0]
    session.get_context().hand = hand(1.0)
    tree.tick_once()
    assert client.sent == 2 and session.difficulty == {1: 2}
    control.pause('last trial')
    for _ in range(10):
        clock[0] += 1.
        tree.tick_once()
        if tree.status != Status.RUNNING: break
    assert tree.status == Status.SUCCESS and len(trials) == 2
    assert ret.sent == 2  # 마지막 시도는 휴식 대신 세션 완료
