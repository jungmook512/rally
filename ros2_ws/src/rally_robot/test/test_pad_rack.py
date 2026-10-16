import math

import numpy as np

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


def test_back_point_behind_approach_on_handle_axis():
    """#208: A 뒤 back_m 는 G → A 축 위, 자세는 A."""
    b = pr.back_point(A, G, 0.06)
    d = np.array(A[:3]) - np.array(G[:3])
    assert np.allclose(np.array(b[:3]) - np.array(A[:3]), d / np.linalg.norm(d) * 60.0)
    assert b[3:6] == A[3:6]
    assert pr.DEFAULTS["pad_rack_back_m"] == 0.0                   # 기본은 예전 경로


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


def test_ready_route_uses_via_posj_when_given():
    """10/14 Y0: ready_via_posj 가 있으면 그 경유 자세 → 대기 자세 (팔만 먼저 돌리는 기본 경로 대신)."""
    VIA = [-58.7, 0.0, 0.0, 98.8, 63.6, -100.3]
    Y0 = [-58.7, 7.0, 100.2, 98.8, 63.6, -100.3]
    via, end = pr.ready_route(U, Y0, VIA)
    assert via == VIA and end == Y0
    assert pr.ready_route(U, Y0)[0][:3] == Y0[:3]               # 없으면 기본 그대로


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


def test_search_offsets_pattern_around_shift():
    cfg = {**pr.DEFAULTS, "pad_insert_shift_m": [0.006, -0.004, 0.0],
           "pad_insert_search_pattern_m": [[0.0, -0.003, 0.0], [0.003, 0.0, 0.0]]}
    offs = pr.search_offsets(cfg)
    assert len(offs) == 3                                       # 첫 시도 + 패턴 2자리
    assert offs[0].tolist() == pytest.approx([6.0, -4.0, 0.0, 0.0])  # 첫 시도는 고정 보정만 [mm…, rz°]
    assert offs[1].tolist() == pytest.approx([6.0, -7.0, 0.0, 0.0])
    assert offs[2].tolist() == pytest.approx([9.0, -4.0, 0.0, 0.0])
    assert len(pr.search_offsets({**cfg, "pad_insert_search_pattern_m": []})) == 1


def test_lift_goes_out_of_slot_before_side_move():
    assert pr.lift_height_mm(21.0, 45.0, 8.0) == 45.0          # 홈 안에서 걸림 (10/14 G 위 21 mm) → 홈 밖까지
    assert pr.lift_height_mm(59.0, 45.0, 8.0) == 67.0          # 입구 위쪽에서 걸림 (10/13 59 mm) → 걸린 자리 + 8 mm


def test_defaults_keep_old_behaviour_first_try():
    assert pr.search_offsets(pr.DEFAULTS)[0].tolist() == [0.0, 0.0, 0.0, 0.0]   # 고정 보정 0 → 첫 시도는 예전과 같은 경로


def test_shifted_moves_position_only():
    s = pr.shifted(G, [3.0, -1.0, 0.0])
    assert s[:3] == pytest.approx([G[0] + 3.0, G[1] - 1.0, G[2]]) and s[3:] == G[3:]
    assert pr.slot_axis(G, U) @ [0, 0, 1] > 0.99                # 홈 축은 거의 수직 위


def test_slow_point_sits_between_entry_and_seat():
    P, _ = pr.insert_points(G, U, 0.08, 0.01)
    S, _ = pr.insert_points(G, U, pr.DEFAULTS["pad_insert_slow_from_m"], 0.01)
    assert pr.above_seat(S, G, U) == pytest.approx(20.0)
    assert pr.above_seat(P, G, U) > pr.above_seat(S, G, U) > 0.0
    assert pr.DEFAULTS["pad_insert_abs_stop_n"] < 25.0          # safety force_limit_n 보다 먼저 멈춰야 함


def test_search_offsets_with_base_z_rotation():
    cfg = {**pr.DEFAULTS, "pad_insert_shift_m": [0.006, -0.004, 0.0],
           "pad_insert_search_pattern_m": [[0.0, 0.0, 0.0, 2.0], [0.0, 0.0, 0.0, -4.0], [0.0, -0.003, 0.0]]}
    offs = pr.search_offsets(cfg)
    assert offs[0].tolist() == pytest.approx([6.0, -4.0, 0.0, 0.0])
    assert offs[1].tolist() == pytest.approx([6.0, -4.0, 0.0, 2.0])
    assert offs[2].tolist() == pytest.approx([6.0, -4.0, 0.0, -4.0])
    assert offs[3].tolist() == pytest.approx([6.0, -7.0, 0.0, 0.0])   # 회전 생략 = 0
    assert "회전 +2.0°" in pr.offset_text(offs[1]) and "옆 [6.0, -4.0, 0.0] mm" in pr.offset_text(offs[1])


def test_shifted_rz_is_rotation_about_base_z():
    from scipy.spatial.transform import Rotation
    s = pr.shifted(G, [0.0, 0.0, 0.0, 3.0])
    assert s[:3] == pytest.approx(G[:3]) and s[4:] == G[4:]           # 위치·B·C 그대로
    want = Rotation.from_euler("z", 3.0, degrees=True) * Rotation.from_euler("ZYZ", G[3:6], degrees=True)
    got = Rotation.from_euler("ZYZ", s[3:6], degrees=True)
    assert (want.inv() * got).magnitude() == pytest.approx(0.0, abs=1e-9)
    assert pr.shifted(G, [1.0, 0.0, 0.0])[3:] == G[3:]                 # 세 값이면 회전 없음


def test_near_rack_and_slow_speeds():
    """10/14: 거치대 G 근처에서 출발하는 관절 이동은 pad_joint 속도로 (관측 자세로 가다 관성 힘 force_limit STOP)."""
    cfg = {**pr.DEFAULTS}
    assert pr.near_rack([84, -310, 252, 0, 0, 0], G, 0.35)          # 20:41 실제 정지 자세 (G 위 4 cm)
    assert not pr.near_rack([600.0, 100.0, 480.0, 0, 0, 0], G, 0.35)   # 테이블 가운데 쪽 (G 에서 50 cm 넘게)
    assert not pr.near_rack([84, -310, 252], G, 0.0)                 # 0 = 끔
    assert pr.slow_speeds(cfg, 60.0, 120.0, True) == (30.0, 60.0)
    assert pr.slow_speeds(cfg, 60.0, 120.0, False) == (60.0, 120.0)
    assert pr.slow_speeds(cfg, 20.0, 40.0, True) == (20.0, 40.0)     # 이미 느리면 그대로
