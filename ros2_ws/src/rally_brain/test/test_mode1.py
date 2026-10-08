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
from rally_interfaces.action import PushBall
from rally_interfaces.msg import HandState, TrialResult

CONFIG = Path(__file__).resolve().parents[2] / 'rally_bringup' / 'config'
OK = (True, {"success": True, "contact_force_peak": 3.0})


def cfg():
    return yaml.safe_load((CONFIG / 'skills.yaml').read_text())


def hand(x=.78, y=.0):
    h = HandState(detected=True)
    h.header.frame_id = 'base_link'
    h.header.stamp.sec, h.header.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
    h.palm_pose.position = Point(x=x, y=y, z=.05)
    return h


def arrange(outcomes, config=None):
    client = FakeClient(PushBall, outcomes)
    session, published, _, ctx = make_session({PushBall: client}, config if config is not None else cfg())
    ctx.hand = hand()
    session.table_cfg = {'centerline_x_m': .568}
    session.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=time.time_ns(), to_msg=lambda: hand().header.stamp))
    return client, session, published


def test_random_lanes_and_positive_recipe_speed():
    client, session, published = arrange([OK] * 4)
    tree = m.build_trial(session, rng=random.Random(1))
    for _ in range(4):
        session.get_context().hand = hand()
        assert run(tree) == Status.SUCCESS
    rng = random.Random(1)
    assert [g.lane for g in client.goals] == [rng.randrange(3) for _ in range(4)]
    for g, t in zip(client.goals, published):
        lane = session.skills_cfg['mode1']['lanes'][g.lane]
        r = lane['recipe']
        assert .2 <= g.push_speed <= .35
        assert g.target.header.frame_id == 'base_link' and g.target.point.x >= .618 - 1e-7
        assert param(t, 'push_speed') == pytest.approx(g.push_speed)
        assert param(t, 'target_d_m') == pytest.approx(g.push_speed**2 / (2*r['decel_mps2']) + r['d0_m'])
        assert math.isnan(t.error_mm) and math.isnan(metric(t, 'stop_x_m'))
    assert session.trials_for_rules(1) == []  # 타격 4회 성공이어도 환자 성공으로 승급하지 않음


def test_goal_changes_with_hand_and_level_and_saturates():
    c = m.mode1_config(cfg())
    # 속도 .4는 실측 범위지만 운용 상한 .35는 그대로 유지. 각 경우 수식 일치.
    for lane in range(3):
        a = m.plan_push(hand(.74), lane, 0, c, {'centerline_x_m': .568})
        b = m.plan_push(hand(.74), lane, 2, c, {'centerline_x_m': .568})
        near = m.plan_push(hand(.69), lane, 0, c, {'centerline_x_m': .568})
        assert b['target_d'] < a['target_d'] and b['speed'] < a['speed']
        assert near['target_d'] < a['target_d']
        assert a['reach_requested'] == pytest.approx(2*c['lanes'][lane]['recipe']['rmse_m'])
        far = m.plan_push(hand(.9), lane, 0, c, {'centerline_x_m': .568})
        assert far['speed'] <= .35 and far['reach_actual'] >= far['reach_requested']


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
    c['centerline_x_m'] = 99  # skills에 있더라도 무시
    p = m.plan_push(hand(.8), 0, 100, c, {'centerline_x_m': .60})
    assert p['target'][0] >= .65 - 1e-9
    assert p['target'][0] == pytest.approx(.65)


def test_reach_projection_and_boundary():
    lane = {'start': [.43, .1], 'dir_deg': 30}
    p = m.reach_target(hand(.8, .3), lane, .1, .568)
    sx, sy, dx, dy = m.lane_geometry(lane)
    assert (p[0]-sx)*dy == pytest.approx((p[1]-sy)*dx)
    assert p[0] >= .618
    assert m.reach_target(hand(.5), lane, .1, .568) is None


def test_measured_error_and_failure_recorded():
    result = (True, {'success': True, 'error_mm': 42.0, 'stop_position': Point(x=.7, y=.1)})
    client, session, published = arrange([result, (False, {'failure_reason': 'no_ball_detected (설명)'})])
    tree = m.build_trial(session)
    assert run(tree) == Status.SUCCESS
    assert published[0].error_mm == pytest.approx(42) and metric(published[0], 'stop_x_m') == pytest.approx(.7)
    assert run(tree) == Status.SUCCESS
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


def test_target_expires_while_waiting_for_server():
    client, session, published = arrange([])
    client.ready = False
    tree = m.build_trial(session)
    tree.tick_once()  # 손 확인·목표 계산 후 서버 대기
    session.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=time.time_ns()+2_000_000_000, to_msg=lambda: hand().header.stamp))
    client.ready = True
    assert run(tree) == Status.SUCCESS
    assert client.sent == 0 and len(published) == 1 and published[0].notes == 'skill_precondition_failed'


def test_stop_records_once():
    client, session, published = arrange([OK])
    tree = m.build_trial(session)
    tree.tick_once()
    tree.stop(Status.INVALID)
    tree.stop(Status.INVALID)
    assert len(published) == 1 and published[0].success


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
    def send(goal):
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
