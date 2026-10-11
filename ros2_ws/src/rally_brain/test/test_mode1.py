"""모드 1 손 기준 목표·실측 범위·양수 속도·기록을 가짜 스킬로 검증한다."""
import math
from pathlib import Path
import random
import time
from types import SimpleNamespace as NS

import pytest
import yaml
from geometry_msgs.msg import Point

from fakes import FakeClient, Status, make_session, metric, param, run
from rally_brain import behaviours, metrics
from rally_brain.trees import mode1_push as m
from rally_brain.trees.session_tree import Session, build_session_tree
from rally_interfaces.action import PushBall, ReturnBall
from rally_interfaces.msg import HandState, TrialResult

CONFIG = Path(__file__).resolve().parents[2] / 'rally_bringup' / 'config'
OK = (True, {"success": True, "contact_force_peak": 3.0})


def cfg():
    c = yaml.safe_load((CONFIG / 'skills.yaml').read_text())
    c['mode1'].update(catch_timeout_s=.01, hand_clear_wait_s=.02)
    return c


def hand(x=.86, y=.0):
    h = HandState(detected=True)
    h.header.frame_id = 'base_link'
    h.header.stamp.sec, h.header.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
    h.distance_to_centerline = .30
    h.palm_pose.position = Point(x=x, y=y, z=.05)
    return h


def arrange(outcomes, config=None):
    client = FakeClient(PushBall, outcomes)
    session, published, _, ctx = make_session({PushBall: client, ReturnBall: FakeClient(ReturnBall, [(True, {"success": True})]*10)}, config if config is not None else cfg())
    ctx.hand = hand()
    session.table_cfg = {'centerline_x_m': .568}
    session.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=time.time_ns(), to_msg=lambda: hand().header.stamp))
    return client, session, published


def test_nearest_lane_and_positive_recipe_speed():
    client, session, published = arrange([OK] * 4)
    tree = m.build_trial(session, rng=random.Random(1))
    for _ in range(4):
        session.get_context().hand = hand()
        assert run(tree) == Status.SUCCESS
    assert [g.lane for g in client.goals] == [1]*4
    for g, t in zip(client.goals, published):
        lane = session.skills_cfg['mode1']['lanes'][g.lane]
        r = lane['recipe']
        assert .2 <= g.push_speed <= .37
        assert g.target.header.frame_id == 'base_link' and g.target.point.x >= .618 - 1e-7
        assert param(t, 'push_speed') == pytest.approx(g.push_speed)
        assert param(t, 'target_d_m') == pytest.approx(g.push_speed**2 / (2*r['decel_mps2']) + r['d0_m'])
        assert math.isnan(t.error_mm) and math.isnan(metric(t, 'stop_x_m'))
    assert session.trials_for_rules(1) == []  # 타격 4회 성공이어도 환자 성공으로 승급하지 않음


def test_fixed_goal_and_hand_start_increase_with_level():
    c = m.mode1_config(cfg())
    for lane in range(3):
        a = m.plan_push(hand(.86), lane, 0, c, {'centerline_x_m': .568})
        b = m.plan_push(hand(.86), lane, 2, c, {'centerline_x_m': .568})
        near = m.plan_push(hand(.80), lane, 0, c, {'centerline_x_m': .568})
        assert a['target'] == b['target'] == near['target']
        assert a['speed'] == b['speed'] == near['speed']
        assert .2 <= a['speed'] <= .37
        assert a['target_d'] == pytest.approx(.35)
        step = 2*c['lanes'][lane]['recipe']['rmse_m']
        assert b['hand_start_d'] - a['hand_start_d'] == pytest.approx(2*step)
        assert a['reach_requested'] == pytest.approx(step)


@pytest.mark.parametrize('invalid', ['absent', 'frame', 'nan', 'stale', 'future', 'zero'])
def test_invalid_hand_times_out_without_goal(invalid):
    c = cfg(); c['mode1'].update(hand_wait_s=.001, hand_wait_tries=1)
    client, session, published = arrange([], c)
    h = session.get_context().hand
    if invalid == 'absent': h.detected = False
    if invalid == 'frame': h.header.frame_id = 'camera_link'
    if invalid == 'nan': h.palm_pose.position.x = math.nan
    if invalid == 'stale': h.header.stamp.sec -= 10
    if invalid == 'future': h.header.stamp.sec += 10
    if invalid == 'zero': h.header.stamp.sec = h.header.stamp.nanosec = 0
    assert run(m.build_trial(session)) == Status.SUCCESS
    assert client.sent == 0 and len(published) == 1 and published[0].notes == 'no_hand'
    assert metric(published[0], 'attempts') == 0


def test_hand_arrives_during_wait():
    client, session, published = arrange([OK])
    session.get_context().hand.detected = False
    tree = m.build_trial(session)
    tree.tick_once()
    assert client.sent == 0 and tree.status == Status.RUNNING
    session.get_context().hand = hand()
    assert run(tree) == Status.SUCCESS and client.sent == 1 and len(published) == 1


@pytest.mark.parametrize('bad', ['missing', 'nan', 'negative', 'range', 'speed_range', 'lane', 'table', 'unreachable'])
def test_invalid_recipe_or_target_never_falls_back_to_fixed_speed(bad):
    c = cfg()
    for lane in c['mode1']['lanes']:
        if bad == 'missing': lane.pop('recipe')
        if bad == 'nan': lane['recipe']['decel_mps2'] = math.nan
        if bad == 'negative': lane['recipe']['decel_mps2'] = -1
        if bad == 'range': lane['recipe']['distance_range_m'] = [.45, .15]
        if bad == 'speed_range': lane['recipe']['speed_range_mps'] = [.4, .5]
        if bad == 'lane': lane['dir_deg'] = 180
    client, session, published = arrange([], c)
    if bad == 'table': session.table_cfg = {}
    if bad == 'unreachable': session.table_cfg['centerline_x_m'] = 2.0
    assert run(m.build_trial(session)) == Status.SUCCESS
    assert client.sent == 0 and len(published) == 1 and not published[0].success


def test_table_is_only_centerline_source():
    c = m.mode1_config(cfg())
    c['brain_speed_range_mps'] = [.1, .4]
    c['centerline_x_m'] = 99  # skills에 있더라도 무시
    p = m.plan_push(hand(.9), 0, 100, c, {'centerline_x_m': .60})
    assert p['target'][0] >= .80 - 1e-9


def test_measured_error_and_failure_recorded():
    result = (True, {'success': True, 'error_mm': 42.0, 'stop_position': Point(x=.7, y=.1)})
    client, session, published = arrange([result, (False, {'failure_reason': 'no_ball_detected (설명)'})])
    tree = m.build_trial(session)
    assert run(tree) == Status.SUCCESS
    assert published[0].error_mm == pytest.approx(42) and metric(published[0], 'stop_x_m') == pytest.approx(.7)
    assert run(tree) == Status.FAILURE
    assert published[1].notes == 'no_ball_detected' and client.sent == 2


def test_server_missing_fails_session(monkeypatch):
    clock = [0.0]
    def fast():
        clock[0] += 1.0
        return clock[0]
    monkeypatch.setattr(behaviours, 'time', NS(monotonic=fast))
    client, session, published = arrange([])
    client.ready = False
    assert run(m.build_trial(session)) == Status.FAILURE
    assert client.sent == 0 and published == []


def test_stale_hand_while_waiting_for_server_waits_for_new_sample():
    client, session, published = arrange([OK])
    client.ready = False
    tree = m.build_trial(session)
    tree.tick_once()
    session.get_context().hand.header.stamp.sec -= 10
    client.ready = True
    tree.tick_once()
    assert tree.status == Status.RUNNING and client.sent == 0 and published == []
    session.get_context().hand = hand()
    assert run(tree) == Status.SUCCESS and client.sent == 1


@pytest.mark.parametrize('blocked', ['short_reach', 'slow', 'unknown'])
def test_start_wait_rechecks_latest_hand_and_normal(blocked):
    from rally_interfaces.msg import SafetyStatus
    client, session, published = arrange([OK])
    said = []
    session.say = said.append
    ctx = session.get_context()
    if blocked == 'short_reach': ctx.hand = hand(.80)
    if blocked == 'slow': ctx.safety.level = SafetyStatus.LEVEL_SLOW
    if blocked == 'unknown': ctx.safety.level = 99
    tree = m.build_trial(session)
    for _ in range(4): tree.tick_once()
    assert tree.status == Status.RUNNING and client.sent == 0 and published == []
    assert said
    ctx.hand = hand(.90)
    ctx.safety.level = SafetyStatus.LEVEL_NORMAL
    assert run(tree) == Status.SUCCESS and client.sent == 1
    assert param(published[0], 'reach_actual_m') >= param(published[0], 'reach_requested_m')


def test_failed_push_does_not_start_another_ball():
    client, session, published = arrange([(False, {'failure_reason': 'slow_at_strike'}), OK])
    assert run(build_session_tree(session, [1], 10)) == Status.FAILURE
    assert client.sent == 1 and len(published) == 1
    assert published[0].notes == 'slow_at_strike'
    assert session.action_client(ReturnBall, m.RETURN_ACTION).sent == 0


def test_stop_records_once():
    client, session, published = arrange([OK])
    tree = m.build_trial(session)
    tree.tick_once()
    tree.stop(Status.INVALID)
    tree.stop(Status.INVALID)
    assert len(published) == 1 and published[0].notes == "interrupted"


def test_session_uses_balls_per_cycle():
    client, session, _ = arrange([OK] * 4)
    tree = build_session_tree(session, [1], 10)
    assert run(tree) == Status.SUCCESS and client.sent == 4


def test_stop_before_goal_and_midflight_cancels():
    from fakes import Done
    client, session, published = arrange([])
    # STOP 판정 상수 사용 (숫자에 기대지 않음)
    from rally_interfaces.msg import SafetyStatus
    session.get_context().safety.level = SafetyStatus.LEVEL_STOP
    tree = m.build_trial(session)
    tree.tick_once()
    tree.stop(Status.INVALID)
    assert client.sent == 0 and published == []

    session.get_context().safety.level = SafetyStatus.LEVEL_NORMAL
    canceled = []
    pending = NS(done=lambda: False)
    def send(goal, feedback_callback=None):
        client.goals.append(goal)
        return Done(NS(accepted=True, get_result_async=lambda: pending,
                       cancel_goal_async=lambda: canceled.append(True)))
    client.send_goal_async = send
    tree = m.build_trial(session)
    tree.tick_once()
    tree.stop(Status.INVALID)
    tree.stop(Status.INVALID)
    assert canceled == [True] and len(published) == 1
    assert published[0].notes == 'interrupted'


@pytest.mark.parametrize('blocked', ['near', 'absent', 'stale', 'frame', 'nan'])
def test_late_hand_clear_keeps_trial_active_then_returns(blocked, monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(behaviours, 'time', NS(monotonic=lambda: clock[0]))
    client, session, published = arrange([OK])
    ret = session.action_client(ReturnBall, m.RETURN_ACTION)
    said = []
    session.say = said.append
    tree = m.build_trial(session)
    tree.tick_once()  # PushBall goal
    h = session.get_context().hand
    if blocked == 'near': h.distance_to_centerline = .20
    if blocked == 'absent': h.detected = False
    if blocked == 'stale': h.header.stamp.sec -= 10
    if blocked == 'frame': h.header.frame_id = 'camera_link'
    if blocked == 'nan': h.distance_to_centerline = math.nan
    tree.tick_once()  # PushBall result → catch watch
    clock[0] = 1.0
    tree.tick_once()  # catch timeout → clear wait
    for t in (10., 100., 1000.):
        clock[0] = t
        tree.tick_once()
        assert tree.status == Status.RUNNING
        assert session.trial_active and ret.sent == 0 and published == []
    assert said.count(session.skills_cfg['mode1']['hand_clear_prompt']) == 1
    session.get_context().hand = hand()
    assert run(tree) == Status.SUCCESS
    assert ret.sent == 1 and len(published) == 1
    assert metric(published[0], 'ball_returned') == 1


def test_cancel_during_clear_wait_records_once_and_never_returns(monkeypatch):
    clock = [0.]
    monkeypatch.setattr(behaviours, 'time', NS(monotonic=lambda: clock[0]))
    _, session, published = arrange([OK])
    session.get_context().hand.distance_to_centerline = .20
    ret = session.action_client(ReturnBall, m.RETURN_ACTION)
    tree = m.build_trial(session)
    tree.tick_once(); tree.tick_once()
    clock[0] = 100.
    tree.tick_once()
    assert tree.status == Status.RUNNING
    tree.stop(Status.INVALID); tree.stop(Status.INVALID)
    session.get_context().hand = hand()
    assert ret.sent == 0 and len(published) == 1
    assert published[0].notes == 'interrupted'


def test_catch_needs_fresh_open_grab_hold_release():
    c = m.mode1_config(cfg())
    w = m.CatchWatch(c, 1_000_000_000)
    def sample(t, grip, **kwargs):
        h = hand()
        h.header.stamp.sec, h.header.stamp.nanosec = divmod(int(t*1e9), 1_000_000_000)
        h.grip_closure = float(grip)
        for k, v in kwargs.items(): setattr(h, k, v)
        return h
    assert not w.update(sample(.9, 0), 1_100_000_000)  # 타격 전
    assert not w.update(sample(1.1, .8), 1_100_000_000)  # 펼침부터 필요
    assert not w.update(sample(1.2, 0), 1_200_000_000)
    assert not w.update(sample(1.3, .8), 1_300_000_000)
    assert not w.update(sample(1.3, .8), 1_800_000_000)  # 반복 표본으로 유지 판정 금지
    assert not w.grabbed
    assert not w.update(sample(1.65, .8), 1_650_000_000)
    assert w.grabbed
    assert w.update(sample(1.7, 0), 1_700_000_000)
    w = m.CatchWatch(c, 1_000_000_000)
    for t, g in ((1.1, 0), (1.2, .8), (1.6, .8)):
        assert not w.update(sample(t, g), int(t*1e9))
    assert not w.update(sample(1.7, 0, detected=False), 1_700_000_000)
    assert not w.update(sample(1.8, 0), 1_800_000_000)  # 미검출 이후 이전 파지로 성공 금지


def test_hand_view_failure_does_not_push():
    from fakes import Done
    client, session, published = arrange([])
    session.service_client = lambda *_: NS(service_is_ready=lambda: True,
        call_async=lambda _: Done(NS(success=False, message='motion failed')))
    assert run(m.build_trial(session)) == Status.FAILURE
    assert client.sent == 0 and len(published) == 1
    assert published[0].notes == 'hand_view_failed'


def test_return_failure_stops_next_trial():
    client, session, published = arrange([OK, OK])
    ret = session.action_client(ReturnBall, m.RETURN_ACTION)
    ret.outcomes = [(False, {'failure_reason': 'pick_failed'})]
    assert run(build_session_tree(session, [1], 10)) == Status.FAILURE
    assert client.sent == ret.sent == 1 and len(published) == 1
    assert published[0].notes == 'return_pick_failed'


def test_hand_returns_while_return_server_pending_waits_again(monkeypatch):
    clock = [0.]
    monkeypatch.setattr(behaviours, 'time', NS(monotonic=lambda: clock[0]))
    _, session, published = arrange([OK])
    ret = session.action_client(ReturnBall, m.RETURN_ACTION)
    ret.ready = False
    tree = m.build_trial(session)
    tree.tick_once(); tree.tick_once()
    clock[0] = 1.
    tree.tick_once()  # catch timeout, 손 이탈 확인 후 서버 대기
    session.get_context().hand.distance_to_centerline = .20
    ret.ready = True
    tree.tick_once()
    assert tree.status == Status.RUNNING and session.trial_active
    assert ret.sent == 0 and published == []
    session.get_context().hand = hand()
    assert run(tree) == Status.SUCCESS and ret.sent == 1
