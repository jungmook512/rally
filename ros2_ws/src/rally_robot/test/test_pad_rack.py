import math

import pytest

from rally_robot import pad_rack as pr
from rally_robot.dsr_helpers import SkillAbort

# 10/12 교시 (컨트롤러 fkin 값)
G = [236.9, -139.8, 217.0, 47.7, 88.6, 86.8]
A = [210.0, -169.4, 216.0, 47.7, 88.5, 86.8]
U = [241.1, -140.4, 417.0, 47.7, 88.6, 86.8]


def test_insert_points_follow_slot_axis():
    P, B = pr.insert_points(G, U, 0.03, 0.01)
    d = [U[i] - G[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in d))
    assert math.dist(P[:3], G[:3]) == pytest.approx(30.0, abs=1e-6)
    assert math.dist(B[:3], G[:3]) == pytest.approx(10.0, abs=1e-6)
    assert P[2] > G[2] > B[2]                                   # 위에서 아래로
    assert (P[0] - G[0]) / 30.0 == pytest.approx(d[0] / n)    # 홈 축(살짝 기운 G→U) 위
    assert P[3:] == G[3:] and B[3:] == G[3:]
    with pytest.raises(SkillAbort):
        pr.insert_points(G, G, 0.03, 0.01)


def test_retreat_keeps_offset_from_compliance():
    cur = [G[0] + 2.0, G[1] - 1.5, G[2] + 0.5, *G[3:]]         # 순응으로 G 에서 비켜 섬
    r = pr.retreat_from(cur, G, A)
    assert [r[i] - cur[i] for i in range(3)] == pytest.approx([A[i] - G[i] for i in range(3)])
    assert r[3:] == A[3:]


def test_grip_ok_width_window():
    cfg = dict(pr.DEFAULTS)
    assert pr.grip_ok(True, 30.5, True, cfg) and pr.grip_ok(True, 28.7, True, cfg)
    assert not pr.grip_ok(True, 12.4, True, cfg)                # 빈손으로 끝까지 닫힘
    assert not pr.grip_ok(True, 62.7, True, cfg)                # 그리퍼 잠김(안 닫힘) — 10/12
    assert not pr.grip_ok(True, 30.0, False, cfg) and not pr.grip_ok(False, 30.0, True, cfg)


def test_config_and_rack_points():
    from types import SimpleNamespace
    with pytest.raises(SkillAbort):
        pr.config(SimpleNamespace(cfg={"mode3": {}}))
    node = SimpleNamespace(cfg={"mode3": {"pad_rack_grasp_posj": [1] * 6, "pad_rack_approach_posj": [2] * 6,
                                          "pad_rack_lift_posj": [3] * 6, "pad_insert_pre_m": 0.05}})
    cfg = pr.config(node)
    assert cfg["pad_insert_pre_m"] == 0.05 and cfg["pad_insert_below_m"] == pr.DEFAULTS["pad_insert_below_m"]
    pts = pr.rack_points(lambda q: [float(q[0])] * 6, cfg)
    assert pts["G"][0] == 1.0 and pts["A"][0] == 2.0 and pts["U"][0] == 3.0
    with pytest.raises(SkillAbort):
        pr.rack_points(lambda q: -1, cfg)                       # fkin 실패


def test_ready_route_moves_arm_first_then_wrist():
    U = [91.7, -7.9, -103.8, -109.3, 132.7, -31.8]
    READY = [89.8, 30.0, -121.7, 90.2, -80.2, 88.3]
    via, end = pr.ready_route(U, READY)
    assert via[:3] == READY[:3] and via[3:] == U[3:]       # 팔만 먼저
    assert end == READY
    assert pr._near(READY, [v + 2.0 for v in READY]) and not pr._near(READY, U)


def test_taught_needs_all_three_points():
    from types import SimpleNamespace
    full = {"pad_rack_grasp_posj": [1] * 6, "pad_rack_approach_posj": [2] * 6, "pad_rack_lift_posj": [3] * 6}
    assert pr.taught(SimpleNamespace(cfg={"mode3": full}))
    assert not pr.taught(SimpleNamespace(cfg={"mode3": {**full, "pad_rack_lift_posj": []}}))
    assert not pr.taught(SimpleNamespace(cfg={}))


def test_above_seat_along_slot_axis():
    P, B = pr.insert_points(G, U, 0.08, 0.01)
    assert pr.above_seat(P, G, U) == pytest.approx(80.0)
    assert pr.above_seat(B, G, U) == pytest.approx(-10.0)
    assert pr.above_seat([G[0] + 3.0, G[1] - 2.0, G[2], *G[3:]], G, U) == pytest.approx(0.0, abs=0.1)   # 옆으로 비켜도 높이는 그대로
