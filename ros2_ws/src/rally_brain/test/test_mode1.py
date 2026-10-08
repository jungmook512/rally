"""모드 1 시도 — 레인 순서·고정 속도·결과 기록 (못 잰 오차는 NaN). 가짜 스킬로 로봇 없이 확인한다."""
import math
import random
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import Point

from fakes import FakeClient, Status, make_session, metric, param, run
from rally_brain import behaviours, metrics
from rally_brain.trees import mode1_push
from rally_brain.trees.session_tree import Session, build_session_tree
from rally_interfaces.action import PushBall
from rally_interfaces.msg import TrialResult

OK = (True, {"success": True, "contact_force_peak": 3.0})


def arrange(outcomes, cfg=None):
    client = FakeClient(PushBall, outcomes)
    session, published, said, _ = make_session({PushBall: client}, cfg)
    return client, session, published


def test_random_lanes_and_fixed_speed():
    client, session, published = arrange([OK] * 4)
    tree = mode1_push.build_trial(session, rng=random.Random(1))
    for _ in range(4):
        assert run(tree) == Status.SUCCESS
    rng = random.Random(1)
    assert [g.lane for g in client.goals] == [rng.randrange(3) for _ in range(4)]   # 시드가 같으면 같은 순서
    assert all(g.push_speed == pytest.approx(0.3) for g in client.goals)
    assert all(g.target.point.x == 0.0 and g.target.point.y == 0.0 for g in client.goals)   # target 비움 → 속도 그대로
    assert [param(t, "lane") for t in published] == [g.lane for g in client.goals]
    assert all(t.success and t.mode == TrialResult.MODE_PUSH_BALL for t in published)


def test_speed_clamped_to_range():
    cfg = {"mode1": {"brain_push_speed_mps": 0.9, "push_speed_range": [0.1, 0.6]}}
    assert mode1_push.next_push_speed(None, 0, mode1_push.mode1_config(cfg)) == pytest.approx(0.6)


def test_lane_count_from_config():
    cfg = {"mode1": {"lanes": [{"name": "a"}, {"name": "b"}]}}
    client, session, _ = arrange([OK] * 3, cfg)
    tree = mode1_push.build_trial(session, rng=random.Random(1))
    for _ in range(3):
        run(tree)
    assert all(g.lane in (0, 1) for g in client.goals)


def test_random_lane_covers_all_lanes():
    cfg = mode1_push.mode1_config({})
    rng = random.Random(0)
    lanes = [mode1_push.random_lane(cfg, rng) for _ in range(300)]
    assert set(lanes) == {0, 1, 2}


def test_unmeasured_error_is_nan():
    # 로봇이 아직 정지 위치를 안 채움 → 0 mm("완벽")가 아니라 NaN, 지표에서도 빠짐
    _, session, published = arrange([OK])
    run(mode1_push.build_trial(session, rng=random.Random(1)))
    assert math.isnan(published[0].error_mm)
    assert metrics.key_value(published[0]) is None


def test_measured_error_kept():
    _, session, published = arrange([(True, {"success": True, "error_mm": 42.0, "stop_position": Point(x=0.5, y=0.1)})])
    run(mode1_push.build_trial(session, rng=random.Random(1)))
    assert published[0].error_mm == pytest.approx(42.0)
    assert metric(published[0], "stop_x_m") == pytest.approx(0.5)


def test_failure_reason_code_in_notes():
    client, session, published = arrange([(False, {"failure_reason": "no_ball_detected (관측 자세에서 공이 안 보임)"})])
    assert run(mode1_push.build_trial(session, rng=random.Random(1))) == Status.SUCCESS   # 시도는 끝남 → 다음 공
    assert client.sent == 1                                         # 재시도 없음
    assert [(t.success, t.notes) for t in published] == [(False, "no_ball_detected")]


def test_server_missing_fails_session(monkeypatch):
    clock = [0.0]

    def fast():
        clock[0] += 1.0
        return clock[0]

    monkeypatch.setattr(behaviours, "time", NS(monotonic=fast))
    client = FakeClient(PushBall, [], ready=False)
    session, published, _, _ = make_session({PushBall: client})
    assert run(mode1_push.build_trial(session, rng=random.Random(1))) == Status.FAILURE
    assert published == []


def test_stop_records_once():
    client, session, published = arrange([OK])
    tree = mode1_push.build_trial(session, rng=random.Random(1))
    tree.tick_once()
    tree.stop(Status.INVALID)
    assert [(t.success) for t in published] == [True]   # 결과가 이미 와 있었음 → 실제 결과로
    tree.stop(Status.INVALID)
    assert len(published) == 1


def test_session_uses_balls_per_cycle():
    log = NS(info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None)
    s = Session(node=NS(get_logger=lambda: log), session_id="s", get_context=lambda: None, publish=lambda _: None,
                action_client=lambda *_: None, skills_cfg={"mode1": {"balls_per_cycle": 4}})
    tree = build_session_tree(s, [1], 10)
    assert tree.children[0].children[1].name.endswith("× 4")


LANES = [{"name": "a", "start": [0.43, 0.14, 0.0], "dir_deg": 0.0, "recipe": {"decel_mps2": 0.2, "d0_m": 0.05}},
         {"name": "b", "start": [0.43, 0.0, 0.0], "dir_deg": 0.0, "recipe": {"decel_mps2": 0.2, "d0_m": 0.05}},
         {"name": "c", "start": [0.43, -0.15, 0.0], "dir_deg": 0.0, "recipe": {"decel_mps2": 0.2, "d0_m": 0.05}}]


def test_recipe_target_starts_near_patient():
    # 레시피가 있으면 target(입구 + 40 cm) + push_speed 0 → 속도는 로봇 레시피. 기록엔 예상 속도·목표 거리·난이도
    client, session, published = arrange([OK], {"mode1": {"lanes": LANES}})
    run(mode1_push.build_trial(session, rng=random.Random(1)))
    g = client.goals[0]
    assert g.push_speed == 0.0 and g.target.header.frame_id == "base_link"
    assert g.target.point.x == pytest.approx(0.43 + 0.40)
    assert g.target.point.y == pytest.approx(LANES[g.lane]["start"][1])
    assert param(published[0], "target_d_m") == pytest.approx(0.40)
    assert param(published[0], "level") == 0.0
    assert param(published[0], "push_speed") == pytest.approx(math.sqrt(2 * 0.2 * (0.40 - 0.05)))


def test_harder_stops_farther_from_patient():
    # 난이도 +1 마다 5 cm 로봇 쪽(환자에게서 멀리), 레시피 범위 [0.15, 0.45] 로 자름
    cfg = mode1_push.mode1_config({"mode1": {"lanes": LANES}})
    assert [mode1_push.target_distance(k, cfg) for k in (-2, -1, 0, 1, 2, 5, 9)] == \
        pytest.approx([0.45, 0.45, 0.40, 0.35, 0.30, 0.15, 0.15])
    client, session, _ = arrange([OK], {"mode1": {"lanes": LANES}})
    session.difficulty[TrialResult.MODE_PUSH_BALL] = 2
    run(mode1_push.build_trial(session, rng=random.Random(1)))
    assert client.goals[0].target.point.x == pytest.approx(0.43 + 0.30)


def test_no_recipe_falls_back_to_fixed_speed():
    lanes = [{k: v for k, v in ln.items() if k != "recipe"} for ln in LANES]
    client, session, published = arrange([OK], {"mode1": {"lanes": lanes}})
    run(mode1_push.build_trial(session, rng=random.Random(1)))
    assert client.goals[0].push_speed == pytest.approx(0.3) and client.goals[0].target.point.x == 0.0
    assert math.isnan(param(published[0], "target_d_m"))
