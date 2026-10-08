"""body_geometry 단위 테스트 — 가짜 Pose 33점(엉덩이 원점, y 아래)으로 몸통 기울기·어깨 들림·팔꿈치·보상 판정."""
import math

import numpy as np
import pytest

from rally_human import body_geometry as bg


def upright(lean_deg: float = 0.0, lean_axis: str = "side", hike_m: float = 0.0, elbow_deg: float = 180.0):
    """똑바로 앉은 사람. lean_deg 만큼 엉덩이를 축으로 기울임 (side = 옆, front = 앞). hike_m 만큼 오른 어깨를 올림."""
    w = np.zeros((33, 3))
    w[bg.L_HIP], w[bg.R_HIP] = [-0.1, 0, 0], [0.1, 0, 0]
    w[bg.L_SH], w[bg.R_SH] = [-0.18, -0.5, 0], [0.18, -0.5, 0]
    w[bg.L_EAR], w[bg.R_EAR] = [-0.08, -0.68, 0], [0.08, -0.68, 0]
    w[bg.R_SH, 1] -= hike_m
    # 오른팔: 어깨 → 팔꿈치(아래로 0.3) → 손목 (elbow_deg 로 꺾음)
    w[bg.R_EL] = w[bg.R_SH] + [0, 0.3, 0]
    a = math.radians(180 - elbow_deg)
    w[bg.R_WR] = w[bg.R_EL] + 0.25 * np.array([math.sin(a), math.cos(a), 0])
    w[bg.L_EL], w[bg.L_WR] = w[bg.L_SH] + [0, 0.3, 0], w[bg.L_SH] + [0, 0.3, 0] + [0.2, 0, 0]   # 왼팔은 90° 굽힘
    t = math.radians(lean_deg)
    if lean_axis == "side":      # z 축 회전 (옆으로)
        R = np.array([[math.cos(t), -math.sin(t), 0], [math.sin(t), math.cos(t), 0], [0, 0, 1]])
    else:                        # x 축 회전 (앞으로)
        R = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])
    return w @ R.T


ALL = np.ones(33)


@pytest.mark.parametrize("axis", ["side", "front"])
def test_trunk_lean_with_hips(axis):
    lean, how = bg.trunk_lean_deg(upright(20, axis), ALL)
    assert how == "torso" and lean == pytest.approx(20, abs=0.5)


def test_trunk_lean_shoulders_when_hips_hidden():
    vis = ALL.copy()
    vis[[bg.L_HIP, bg.R_HIP]] = 0.1                       # 책상에 가려짐
    lean, how = bg.trunk_lean_deg(upright(15, "side"), vis)
    assert how == "shoulders" and lean == pytest.approx(15, abs=0.5)


def test_upright_is_zero_and_hidden_shoulders_is_nan():
    assert bg.trunk_lean_deg(upright(), ALL)[0] == pytest.approx(0, abs=1e-6)
    vis = ALL.copy()
    vis[bg.R_SH] = 0.0
    assert math.isnan(bg.trunk_lean_deg(upright(), vis)[0])


def test_shoulder_hike_from_baseline():
    base = bg.ear_shoulder_gaps(upright(), ALL)
    w = upright(hike_m=0.05)                               # 오른 어깨 5 cm 으쓱
    hike = bg.shoulder_elevation_deg(bg.ear_shoulder_gaps(w, ALL), base, bg.shoulder_half_width(upright()))
    assert hike == pytest.approx(math.degrees(math.asin(0.05 / 0.18)), abs=1.0)   # 쇄골 0.18 m 에 대한 각


def test_baseline_needs_frames():
    b = bg.ShoulderBaseline(frames=3)
    g = bg.ear_shoulder_gaps(upright(), ALL)
    assert b.update(g) is None and b.update(g) is None
    assert b.update(g) == pytest.approx(g)


@pytest.mark.parametrize("deg", [180.0, 120.0, 90.0])
def test_elbow_extension_takes_straighter_arm(deg):
    w = upright(elbow_deg=deg)
    assert bg.elbow_extension_deg(w, ALL, "right") == pytest.approx(deg, abs=0.5)
    assert bg.elbow_extension_deg(w, ALL, "auto") == pytest.approx(max(deg, 90.0), abs=0.5)


def test_compensation_needs_consecutive_frames_and_priority():
    f = bg.CompensationFilter(lean_deg=15, hike_deg=10, frames=3)
    assert [f.update(True, 20, 0) for _ in range(3)] == ["none", "none", "trunk_lean"]
    assert f.update(True, 5, 0) == "trunk_lean"           # 한 프레임으로는 안 풀림
    assert [f.update(True, 5, 12) for _ in range(3)][-1] == "shoulder_hike"
    assert [f.update(True, 20, 12) for _ in range(3)][-1] == "trunk_lean"   # 둘 다면 몸통 기울임 먼저
    assert [f.update(False, math.nan, math.nan) for _ in range(3)][-1] == "none"
