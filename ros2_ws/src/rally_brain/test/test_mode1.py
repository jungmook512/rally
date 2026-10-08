"""모드 1 시도 — 손 위치로 레인 · 난이도로 목표 거리(레시피) · 받기(쥐었다 폄) · 공 회수 · 기록. 가짜 스킬로 로봇 없이 확인한다."""
import math
import random
import time
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import Point

from fakes import FakeClient, Status, make_session, metric, param, run
from rally_brain import behaviours, metrics
from rally_brain.trees import mode1_push
from rally_brain.trees.session_tree import Session, build_session_tree
from rally_interfaces.action import PushBall, ReturnBall
from rally_interfaces.msg import HandState, TrialResult

OK = (True, {"success": True, "contact_force_peak": 3.0})
RET_OK = (True, {"success": True})
FAST = {"catch_timeout_s": 0.2, "catch_grab_hold_s": 0.0, "hand_clear_wait_s": 0.2, "hand_wait_s": 0.2}
LANES = [{"name": "a", "start": [0.43, 0.14, 0.0], "dir_deg": 0.0, "recipe": {"decel_mps2": 0.2, "d0_m": 0.05}},
         {"name": "b", "start": [0.43, 0.0, 0.0], "dir_deg": 0.0, "recipe": {"decel_mps2": 0.2, "d0_m": 0.05}},
         {"name": "c", "start": [0.43, -0.15, 0.0], "dir_deg": 0.0, "recipe": {"decel_mps2": 0.2, "d0_m": 0.05}}]


def arrange(outcomes, cfg=None, returns=None, ret_feedbacks=None, hand_y=None):
    """손은 기본으로 보이고 중앙선에서 0.5 m (빠져 있음). hand_y 를 주면 base_link 손바닥 y (레인 고르기용)."""
    client = FakeClient(PushBall, outcomes)
    ret = FakeClient(ReturnBall, returns if returns is not None else [RET_OK] * 8, feedbacks=ret_feedbacks)
    m1 = {**FAST, **((cfg or {}).get("mode1") or {})}
    session, published, said, ctx = make_session({PushBall: client, ReturnBall: ret}, {**(cfg or {}), "mode1": m1})
    ctx.hand.distance_to_centerline = 0.5
    if hand_y is not None:
        ctx.hand.header.frame_id = "base_link"
        ctx.hand.palm_pose.position.y = hand_y
    return client, ret, session, published, said, ctx


def run_caught(tree, client, ctx, max_s=3.0):
    """공을 친 뒤 손이 쥐었다(0.9) 폈다(0) 하게 흉내 내며 트리를 끝까지."""
    t0, phase = time.monotonic(), 0
    while time.monotonic() - t0 < max_s:
        tree.tick_once()
        if tree.status != Status.RUNNING:
            return tree.status
        if client.sent and phase == 0:
            ctx.hand.grip_closure, phase = 0.9, 1
        elif phase == 1:
            ctx.hand.grip_closure, phase = 0.0, 2
        time.sleep(0.005)
    return tree.status


# ── 순수 함수 ──
def test_lane_for_hand_nearest():
    cfg = mode1_push.mode1_config({"mode1": {"lanes": LANES}})
    hand = HandState(detected=True)
    hand.header.frame_id = "base_link"
    for y, lane in ((0.20, 0), (0.08, 0), (0.06, 1), (-0.02, 1), (-0.30, 2)):
        hand.palm_pose.position.y = y
        assert mode1_push.lane_for_hand(cfg, hand) == lane
    hand.header.frame_id = "camera_color_optical_frame"          # base 좌표가 아니면 못 고름
    assert mode1_push.lane_for_hand(cfg, hand) is None
    assert mode1_push.lane_for_hand(cfg, HandState(detected=False)) is None


def test_catch_watch_needs_grab_then_release():
    w = mode1_push.CatchWatch({"catch_grab_closure": 0.6, "catch_grab_hold_s": 0.3, "catch_release_closure": 0.3})
    h = HandState(detected=True)
    h.grip_closure = 0.0
    assert not w.update(h, 0.0)                     # 펴고만 있음
    h.grip_closure = 0.9
    assert not w.update(h, 1.0) and not w.update(h, 1.2)   # 0.2 s 쥠 — 아직
    assert not w.update(h, 1.35)                     # 0.35 s 쥠 → 쥠 확정
    h.grip_closure = 0.1
    assert w.update(h, 1.5)                          # 폄 → 받음


def test_harder_stops_farther_from_patient():
    # 난이도 +1 마다 5 cm 로봇 쪽, [0.27, 0.45] 로 자름 (하한: 손이 중앙선 15 cm 안이면 안전감시 STOP)
    cfg = mode1_push.mode1_config({"mode1": {"lanes": LANES}})
    assert [mode1_push.target_distance(k, cfg) for k in (-2, -1, 0, 1, 2, 3, 9)] == \
        pytest.approx([0.45, 0.45, 0.40, 0.35, 0.30, 0.27, 0.27])


def test_speed_clamped_to_range():
    cfg = {"mode1": {"brain_push_speed_mps": 0.9, "push_speed_range": [0.1, 0.6]}}
    assert mode1_push.next_push_speed(None, 0, mode1_push.mode1_config(cfg)) == pytest.approx(0.6)


def test_random_lane_covers_all_lanes():
    cfg = mode1_push.mode1_config({})
    rng = random.Random(0)
    assert {mode1_push.random_lane(cfg, rng) for _ in range(300)} == {0, 1, 2}


# ── 시도 한 번 ──
def test_hand_lane_recipe_target_caught_and_returned():
    client, ret, session, published, said, ctx = arrange([OK], {"mode1": {"lanes": LANES}}, hand_y=-0.13)
    assert run_caught(mode1_push.build_trial(session, rng=random.Random(1)), client, ctx) == Status.SUCCESS
    g = client.goals[0]
    assert g.lane == 2                                               # 손바닥 y −0.13 → 레인 2
    assert g.push_speed == 0.0 and g.target.header.frame_id == "base_link"
    assert g.target.point.x == pytest.approx(0.43 + 0.40) and g.target.point.y == pytest.approx(-0.15)
    assert [(r.lane, r.timeout_s > 0) for r in ret.goals] == [(2, True)]   # 공 회수
    t = published[0]
    assert t.success and t.notes == ""                               # 성공 = 잘 받음
    assert param(t, "target_d_m") == pytest.approx(0.40) and param(t, "level") == 0.0
    assert param(t, "push_speed") == pytest.approx(math.sqrt(2 * 0.2 * (0.40 - 0.05)))
    assert metric(t, "lane_by_hand") == 1.0 and metric(t, "ball_returned") == 1.0


def test_harder_level_moves_target():
    client, _, session, _, _, ctx = arrange([OK], {"mode1": {"lanes": LANES}}, hand_y=0.0)
    session.difficulty[TrialResult.MODE_PUSH_BALL] = 2
    run_caught(mode1_push.build_trial(session, rng=random.Random(1)), client, ctx)
    assert client.goals[0].lane == 1 and client.goals[0].target.point.x == pytest.approx(0.43 + 0.30)


def test_not_caught_is_failure_but_ball_returned():
    client, ret, session, published, _, ctx = arrange([OK], {"mode1": {"lanes": LANES}}, hand_y=0.0)
    assert run(mode1_push.build_trial(session, rng=random.Random(1)), max_s=3.0) == Status.SUCCESS   # 손을 쥐지 않음
    assert [(t.success, t.notes) for t in published] == [(False, "not_caught")]
    assert ret.sent == 1 and metric(published[0], "ball_returned") == 1.0


def test_return_out_of_reach_prompts_and_retries():
    fails = [(False, {"success": False, "failure_reason": "ball_out_of_reach"}), RET_OK]
    client, ret, session, published, said, ctx = arrange([OK], {"mode1": {"lanes": LANES}}, returns=fails,
                                                         ret_feedbacks=[[{"phase": "out_of_reach"}], []], hand_y=0.0)
    run_caught(mode1_push.build_trial(session, rng=random.Random(1)), client, ctx)
    assert ret.sent == 2                                             # 다시 시도
    assert said.count(mode1_push.DEFAULTS["out_of_reach_prompt"]) >= 1
    assert published[0].success and metric(published[0], "ball_returned") == 1.0


def test_return_failure_keeps_trial_result():
    fails = [(False, {"success": False, "failure_reason": "pickup_failed"})]
    client, ret, session, published, said, ctx = arrange([OK], {"mode1": {"lanes": LANES}}, returns=fails, hand_y=0.0)
    run_caught(mode1_push.build_trial(session, rng=random.Random(1)), client, ctx)
    assert ret.sent == 1                                             # pickup_failed 는 재시도 안 함
    assert published[0].success and metric(published[0], "ball_returned") == 0.0
    assert mode1_push.DEFAULTS["return_fail_prompt"] in said


def test_hand_not_in_base_falls_back_to_random_lane_and_fixed_speed():
    client, _, session, published, _, ctx = arrange([OK] * 4)       # 레인 설정·레시피 없음, 손 좌표 base 아님
    tree = mode1_push.build_trial(session, rng=random.Random(1))
    for _ in range(4):
        assert run_caught(tree, client, ctx) == Status.SUCCESS
        ctx.hand.grip_closure = 0.0
    rng = random.Random(1)
    assert [g.lane for g in client.goals] == [rng.randrange(3) for _ in range(4)]
    assert all(g.push_speed == pytest.approx(0.3) and g.target.point.x == 0.0 for g in client.goals)
    assert all(metric(t, "lane_by_hand") == 0.0 for t in published)
    assert math.isnan(param(published[0], "target_d_m"))


def test_no_hand_records_no_hand():
    client, ret, session, published, said, ctx = arrange([OK])
    ctx.hand.detected = False
    assert run(mode1_push.build_trial(session, rng=random.Random(1)), max_s=3.0) == Status.SUCCESS
    assert [(t.success, t.notes) for t in published] == [(False, "no_hand")] and client.sent == 0


def test_unmeasured_error_is_nan():
    client, _, session, published, _, ctx = arrange([OK])
    run_caught(mode1_push.build_trial(session, rng=random.Random(1)), client, ctx)
    assert math.isnan(published[0].error_mm)
    assert metrics.key_value(published[0]) is None


def test_measured_error_kept():
    client, _, session, published, _, ctx = arrange([(True, {"success": True, "error_mm": 42.0, "stop_position": Point(x=0.5, y=0.1)})])
    run_caught(mode1_push.build_trial(session, rng=random.Random(1)), client, ctx)
    assert published[0].error_mm == pytest.approx(42.0)
    assert metric(published[0], "stop_x_m") == pytest.approx(0.5)


def test_failure_reason_code_in_notes():
    client, ret, session, published, _, _ = arrange([(False, {"failure_reason": "no_ball_detected (관측 자세에서 공이 안 보임)"})])
    assert run(mode1_push.build_trial(session, rng=random.Random(1))) == Status.SUCCESS   # 시도는 끝남 → 다음 공
    assert client.sent == 1 and ret.sent == 0                        # 재시도·회수 없음
    assert [(t.success, t.notes) for t in published] == [(False, "no_ball_detected")]


def test_server_missing_fails_session(monkeypatch):
    clock = [0.0]

    def fast():
        clock[0] += 1.0
        return clock[0]

    monkeypatch.setattr(behaviours, "time", NS(monotonic=fast))
    client = FakeClient(PushBall, [], ready=False)
    session, published, _, ctx = make_session({PushBall: client, ReturnBall: FakeClient(ReturnBall, [])}, {"mode1": FAST})
    assert run(mode1_push.build_trial(session, rng=random.Random(1))) == Status.FAILURE
    assert published == []


def test_stop_records_once_as_interrupted():
    client, _, session, published, _, _ = arrange([OK])
    tree = mode1_push.build_trial(session, rng=random.Random(1))
    tree.tick_once()
    tree.stop(Status.INVALID)
    assert [(t.success, t.notes) for t in published] == [(False, "interrupted")]   # 받기 확인 전에 멈춤
    tree.stop(Status.INVALID)
    assert len(published) == 1


def test_session_uses_balls_per_cycle():
    log = NS(info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None)
    s = Session(node=NS(get_logger=lambda: log), session_id="s", get_context=lambda: None, publish=lambda _: None,
                action_client=lambda *_: None, skills_cfg={"mode1": {"balls_per_cycle": 4}})
    tree = build_session_tree(s, [1], 10)
    assert tree.children[0].children[1].name.endswith("× 4")
