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


def _fk_linear(q):
    """시험용 가짜 fkin: TCP x = 2·J1, y = J2, z = 400 + J3 (mm). 관절 보간 경로의 극값이 쉽게 보인다."""
    return [2.0 * q[0], q[1], 400.0 + q[2], 0.0, 0.0, 0.0]


def test_joint_path_extent_and_inside():
    lo, hi = d.joint_path_extent([0, 0, 0, 0, 0, 0], [100, -50, 20, 0, 0, 0], _fk_linear)
    assert lo == pytest.approx([0.0, -50.0, 400.0]) and hi == pytest.approx([200.0, 0.0, 420.0])
    assert d.path_inside([0] * 6, [100, -50, 20, 0, 0, 0], _fk_linear, 250.0, 100.0, 300.0)
    assert not d.path_inside([0] * 6, [300, 0, 0, 0, 0, 0], _fk_linear, 540.0, 580.0, 30.0)        # x 600 > 540
    assert d.path_inside([0] * 6, [1] * 6, lambda q: None, 1, 1, 1) is None                      # fkin 실패 = 모름


def test_plan_joint_path_direct_or_two_stage():
    fk = _fk_linear
    assert d.plan_joint_path([0] * 6, [100, 0, 0, 0, 0, 0], fk, 540.0, 580.0, 30.0) == [[100.0, 0, 0, 0, 0, 0]]   # 바로
    # x 가 J1 로 결정되는데 목표 J1 = 300 → 어떤 경로든 x 600 > 540 → 계획 없음
    assert d.plan_joint_path([0] * 6, [300, 0, 0, 0, 0, 0], fk, 540.0, 580.0, 30.0) is None
    # fk 실패면 확인 없이 바로
    assert d.plan_joint_path([0] * 6, [300, 0, 0, 0, 0, 0], lambda q: None, 540.0, 580.0, 30.0) == [[300.0, 0, 0, 0, 0, 0]]

    def fk_bulge(q):
        # 손목(J4)이 45° 넘게 돌아간 채 J1 이 20~60° 구간을 지나면 TCP 가 x +400 mm 튀어나오는 가짜 기구학 (손목이 풀리며 휘는 경우)
        bulge = 400.0 if abs(q[3]) > 45 and 20 < q[0] < 60 else 0.0
        return [2.0 * q[0] + bulge, q[1], 400.0 + q[2], 0, 0, 0]
    # 바로 가면 J1 42·J4 54 쯤에서 x 484 > 450. '손목 먼저'(J4 90 뒤 J1 0→70)도 J1 20~60 에서 튐. '팔 먼저'(J1 70 뒤 J4 0→90)는 J1 70 이라 안 튐
    plan = d.plan_joint_path([0] * 6, [70, 0, 0, 90, 0, 0], fk_bulge, 450.0, 580.0, 30.0)
    assert plan == [[70.0, 0.0, 0.0, 0.0, 0.0, 0.0], [70.0, 0.0, 0.0, 90.0, 0.0, 0.0]]



def test_plan_joint_path_uses_via_poses_when_two_stage_fails():
    # J1 50~200 구간을 J2 > −50 으로 지나면 x 가 +400 튐 → 직접·두 단계(J1 먼저 등) 모두 실패. 먼저 J2 를 −300 으로 내린 경유 자세를 거치면 안
    def fk(q):
        bulge = 400.0 if 50 < q[0] < 200 and q[1] > -50 else 0.0
        return [q[0] + bulge, 0.0, 400.0, 0, 0, 0]
    q0, q1 = [0.0] * 6, [250.0, -100.0, 0, 0, 0, 0]
    assert d.plan_joint_path(q0, q1, fk, 450.0, 580.0, 30.0) is None
    via = [0.0, -300.0, 0, 0, 0, 0]
    plan = d.plan_joint_path(q0, q1, fk, 450.0, 580.0, 30.0, vias=[[[150.0, 0, 0, 0, 0, 0]], [via]])   # 첫 후보는 튀는 자리
    assert plan == [via, q1]
