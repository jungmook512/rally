import math
from types import SimpleNamespace

import pytest

pytest.importorskip("rally_interfaces")
from rally_robot.skills import resist_push as rp  # noqa: E402

TABLE = {"centerline_x_m": 0.568, "z_m": 0.0144}
NOW = 10_000_000_000


def hand(y, d, age_s=0.1, frame="base_link", detected=True):
    t = NOW - int(age_s * 1e9)
    return SimpleNamespace(detected=detected, distance_to_centerline=d,
                           header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=t // 1_000_000_000, nanosec=t % 1_000_000_000)),
                           palm_pose=SimpleNamespace(position=SimpleNamespace(y=y)))


def test_present_x_fixed_y_follows_far_hand_only():
    cfg = dict(rp.DEFAULTS)
    base, _ = rp.present_pose(cfg, TABLE, None, NOW)
    assert base[0] == pytest.approx((0.568 - 0.12) * 1000) and base[1] == 0.0
    far, why = rp.present_pose(cfg, TABLE, hand(0.12, 0.45), NOW)
    assert far[0] == base[0] and far[1] == pytest.approx(120.0) and "맞춤" in why    # 앞뒤는 그대로, 좌우만
    clip, _ = rp.present_pose(cfg, TABLE, hand(0.50, 0.45), NOW)
    assert clip[1] == pytest.approx(200.0)                                          # 범위로 자름
    near, why = rp.present_pose(cfg, TABLE, hand(0.12, 0.10), NOW)
    assert near[1] == 0.0 and "가까워서" in why                                      # 손이 가까우면 안 따라감
    old, _ = rp.present_pose(cfg, TABLE, hand(0.12, 0.45, age_s=3.0), NOW)
    assert old[1] == 0.0
    nan, _ = rp.present_pose(cfg, TABLE, hand(math.nan, 0.45), NOW)
    assert nan[1] == 0.0
    assert far[0] / 1000 < TABLE["centerline_x_m"]                                  # 제시 TCP 는 보라색 선 안쪽


def test_yield_only_moves_away_from_patient():
    p = [450.0, 30.0, 270.0, 1.0, 2.0, 3.0]
    for d in (0.05, -0.05, 0.0):
        t = rp.yield_target(p, d)
        assert t[0] <= p[0] and t[1:] == p[1:]                                       # 절대 +x 없음
    assert rp.yield_target(p, 0.05)[0] == pytest.approx(400.0)


def test_push_force_is_minus_x_only():
    f0 = [0.5, 0.0, -1.0, 0, 0, 0]
    assert rp.push_force([-7.5, 0, 0, 0, 0, 0], f0) == pytest.approx(8.0)           # 로봇을 −x 로 밂
    assert rp.push_force([5.0, 0, 0, 0, 0, 0], f0) == 0.0                           # 당기는 쪽은 0


def test_hold_tracker_held_released_and_reset():
    tr = rp.HoldTracker(5.0, 2.0, 1.5, 0.5, 2.0)
    t = 0.0
    for f in [1.0, 3.0, 6.0, 6.0, 4.0, 6.0]:                                        # 중간에 목표 아래로 떨어지면 유지 시간 다시
        assert tr.update(t, f) == ""
        t += 0.5
    out = ""
    while not out:
        out = tr.update(t, 6.0)
        t += 0.05
    assert out == "held" and tr.best_hold >= 2.0 and tr.peak == 6.0
    tr = rp.HoldTracker(5.0, 2.0, 1.5, 0.5, 2.0)
    assert tr.update(0.0, 0.5) == "" and tr.update(1.0, 0.5) == ""                  # 밀기 전 0 N 은 손 뗌 아님
    tr.update(1.1, 3.0)
    assert tr.update(1.2, 0.5) == "" and tr.update(1.8, 0.5) == "released"


def test_smoothness():
    assert math.isnan(rp.smoothness([1.0, 1.0], 5.0))
    assert rp.smoothness([5.0, 5.0, 5.0, 5.0], 5.0) == pytest.approx(0.0)
    assert rp.smoothness([0.1, 4.0, 6.0, 5.0], 5.0) > 0.0
