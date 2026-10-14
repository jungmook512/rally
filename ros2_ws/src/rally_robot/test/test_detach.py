"""detach.py 순수 함수 (로봇 없이): 벗기기 기하·힘 판정·공 고르기 (10/14 정정인)."""
import math
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rally_robot import detach as d
from rally_robot.dsr_helpers import SkillAbort

N = [1.0, 0.0, 0.0]            # 거치대에 꽂힌 패드 면 법선 (+x)
BALL = [0.30, -0.15, 0.40]     # 공 중심 (base, m)
R = 0.031


def _obj(cls, conf, x, y, z):
    return SimpleNamespace(class_name=cls, confidence=conf, position=SimpleNamespace(x=x, y=y, z=z))


def test_peel_axis_is_horizontal_in_pad_plane():
    a = d.peel_axis(N)
    assert a == pytest.approx([0.0, 1.0, 0.0])                   # ẑ × x̂ = ŷ
    with pytest.raises(SkillAbort):
        d.peel_axis([0.0, 0.0, 1.0])                             # 누운 패드는 범위 밖


def test_grasp_orientation_faces_pad_with_vertical_fingers():
    zyz = d.grasp_orientation(N, [1.0, 0.0, 0.0])
    Rm = Rotation.from_euler("ZYZ", zyz, degrees=True).as_matrix()
    assert Rm[:, 2] == pytest.approx([-1.0, 0.0, 0.0], abs=1e-9)   # 툴 z = −n (패드 정면)
    assert Rm[:, 0] == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)    # 손가락 축(툴 x) = base ẑ
    zyz_y = d.grasp_orientation(N, [0.0, 1.0, 0.0])
    Ry = Rotation.from_euler("ZYZ", zyz_y, degrees=True).as_matrix()
    assert Ry[:, 1] == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)


def test_grasp_pose_and_along():
    orient = d.grasp_orientation(N, [1.0, 0.0, 0.0])
    g = d.grasp_pose(BALL, N, 0.0, orient)
    assert g[:3] == pytest.approx([300.0, -150.0, 400.0])
    pre = d.along(g, N, 0.08)
    assert pre[:3] == pytest.approx([380.0, -150.0, 400.0])      # +n 쪽 = 패드에서 멀어짐
    assert pre[3:] == pytest.approx(g[3:])


def test_peel_pose_rotates_top_away_from_pad_and_keeps_pivot():
    orient = d.grasp_orientation(N, [1.0, 0.0, 0.0])
    g = d.grasp_pose(BALL, N, 0.0, orient)
    p0 = d.peel_pose(g, BALL, N, R, 0.5, 0.0, 0.0)
    assert p0 == pytest.approx(g, abs=1e-6)                      # 0° = 그대로
    p = d.peel_pose(g, BALL, N, R, 0.5, 10.0, 0.0)
    # 회전 중심 = 접촉점(c − r n) 아래 r/2: TCP(공 중심)는 pivot 보다 위 → +θ 면 +n(+x) 쪽으로, z 는 약간 내려감
    assert p[0] > g[0] and p[2] < g[2]
    pivot = np.array(BALL) * 1000 - np.array(N) * R * 1000 - np.array([0, 0, 1]) * R * 0.5 * 1000
    assert math.dist(p[:3], pivot) == pytest.approx(math.dist(g[:3], pivot), abs=1e-6)   # pivot 까지 거리 보존 (강체 회전)
    pr = d.peel_pose(g, BALL, N, R, 0.5, 10.0, 0.002)
    assert np.array(pr[:3]) - np.array(p[:3]) == pytest.approx([2.0, 0.0, 0.0], abs=1e-6)   # 후퇴는 n 방향 2 mm
    Rp = Rotation.from_euler("ZYZ", p[3:], degrees=True).as_matrix()
    assert np.degrees(np.arccos(np.clip(Rp[:, 2] @ np.array([-1.0, 0, 0]), -1, 1))) == pytest.approx(10.0, abs=1e-6)   # 자세도 10° 돌아감


def test_peel_schedule():
    s = d.peel_schedule(5.0, 40.0, 0.002)
    assert len(s) == 8 and s[0] == (5.0, 0.002) and s[-1] == pytest.approx((40.0, 0.016))
    assert d.peel_schedule(10.0, 3.0, 0.0) == [(10.0, 0.0)]      # 최소 한 스텝


def test_peel_watch_detects_drop_after_peak_and_limit():
    w = d.PeelWatch(limit_n=15.0, drop_n=5.0, min_peak_n=4.0)
    assert w.feed([1.0, 0, 0]) == "" and w.feed([3.0, 0, 0]) == ""
    assert w.feed([0.5, 0, 0]) == ""                              # peak 3 < min_peak 4 → 급락 아님 (잡음)
    assert w.feed([8.0, 0, 0]) == "" and w.feed([9.0, 1.0, 0]) == ""
    assert w.feed([2.0, 0, 0]) == "detached" and w.detached and w.drop == pytest.approx(math.hypot(9.0, 1.0) - 2.0)
    w2 = d.PeelWatch(15.0, 5.0, 4.0)
    assert w2.feed([10.0, 10.0, 5.0]) == "over" and w2.over


def test_pick_ball_filters_class_conf_and_radius():
    objs = [_obj("velcro_ball", 0.9, 0.9, 0.0, 0.2),             # G 에서 멀리 (다른 공)
            _obj("velcro_ball", 0.3, 0.30, -0.15, 0.40),         # 신뢰도 미달
            _obj("ball", 0.95, 0.31, -0.15, 0.40),               # 다른 클래스
            _obj("velcro_ball", 0.6, 0.30, -0.15, 0.41),
            _obj("velcro_ball", 0.8, 0.29, -0.14, 0.40)]
    assert d.pick_ball(objs, "velcro_ball", 0.4, [0.24, -0.14, 0.22], 0.25) == (0.29, -0.14, 0.40)
    assert d.pick_ball(objs, "velcro_ball", 0.4) == (0.9, 0.0, 0.2)   # 반경 제한 없으면 신뢰도 최고
    assert d.pick_ball([], "velcro_ball", 0.4) is None
    assert d.pick_ball(objs, ["velcro_ball", "ball"], 0.4, [0.24, -0.14, 0.22], 0.25) == (0.31, -0.15, 0.40)   # 목록이면 ball(0.95) 도 후보
    assert d.ball_classes("ball") == ("ball",) and d.ball_classes(["a", "b"]) == ("a", "b")


def test_ball_median_and_grip_ok():
    assert d.ball_median([(0.3, 0, 0.4), (0.31, 0, 0.4), (0.9, 0, 0.4)], 3) == pytest.approx([0.31, 0.0, 0.4])
    assert d.ball_median([(0.3, 0, 0.4)], 5) is None
    assert d.grip_ok(True, 60.0, True, R, 10.0)
    assert not d.grip_ok(True, 40.0, True, R, 10.0)              # 너무 좁음 — 헛쥠
    assert not d.grip_ok(True, 62.0, False, R, 10.0)


def test_config_requires_both_teach_points():
    node = SimpleNamespace(cfg={"mode3": {"detach_ball_view_posj": [1, 2, 3, 4, 5, 6], "detach_limit_n": 12.0}})
    assert not d.taught(node)
    with pytest.raises(SkillAbort):
        d.config(node)
    node.cfg["mode3"]["detach_box_drop_posj"] = [6, 5, 4, 3, 2, 1]
    assert d.taught(node)
    cfg = d.config(node)
    assert cfg["limit_n"] == 12.0 and cfg["ball_radius_m"] == R      # detach_ 접두 키만 덮어씀, 나머지 기본
