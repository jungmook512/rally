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
    assert all(math.isnan(v) for v in b.update(g)) and all(math.isnan(v) for v in b.update(g))
    assert b.update(g) == pytest.approx(g)


def test_baseline_hidden_side_fills_later():
    """측면 촬영: 처음엔 한쪽 귀가 가려졌다가 나중에 보이면 그쪽 기준도 그때 잡힌다 (#83 리뷰 1)."""
    b = bg.ShoulderBaseline(frames=3)
    for _ in range(3):
        base = b.update((0.18, math.nan))
    assert base[0] == pytest.approx(0.18) and math.isnan(base[1])
    for _ in range(3):
        base = b.update((0.18, 0.17))
    assert base == pytest.approx((0.18, 0.17))
    assert b.update((0.10, 0.10)) == pytest.approx((0.18, 0.17))     # 확정된 뒤엔 안 바뀜


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


def test_compensation_cleared_immediately_when_person_lost():
    """확정 뒤 사람을 놓치면 첫 프레임부터 none, 다시 보이면 처음부터 연속 확인 (#83 리뷰 2)."""
    f = bg.CompensationFilter(lean_deg=15, hike_deg=10, frames=3)
    for _ in range(3):
        f.update(True, 20, 0)
    assert f.update(False, math.nan, math.nan) == "none"
    assert [f.update(True, 20, 0) for _ in range(3)] == ["none", "none", "trunk_lean"]


def test_shoulder_baseline_reset_recaptures():
    """자리를 잡는 동안 잡힌 기준을 버리고, 바르게 앉은 뒤 다시 잡는다 (reset_baseline 서비스)."""
    b = bg.ShoulderBaseline(frames=3)
    for _ in range(3):
        b.update((0.10, 0.10))                 # 움직이는 중 (잘못된 기준)
    assert b.update((0.10, 0.10)) == (0.10, 0.10)
    b.reset()
    assert all(math.isnan(v) for v in b.update((0.18, 0.18)))   # 다시 쌓는 중
    b.update((0.18, 0.18))
    assert b.update((0.18, 0.18)) == (0.18, 0.18)


def _rot_x(deg: float) -> np.ndarray:
    r = np.radians(deg)
    return np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])


def _meter(frames: int = 3) -> "bg.ReachMeter":
    return bg.ReachMeter(arm="right", baseline_frames=frames, window_s=0.5)


def _feed(m, w, n=5, t0=0.0, vis=ALL):
    st = None
    for i in range(n):
        st = m.update(t0 + i / 15, w, vis)
    return st


def test_reach_meter_zero_at_baseline_even_with_tilted_camera():
    """카메라가 25° 내려다봐도 바르게 앉아 팔을 뻗으면 0 (기준 자세 대비)."""
    tilt = _rot_x(25)
    m = _meter()
    st = _feed(m, upright() @ tilt.T)
    assert st.baseline_ready and st.extended
    assert st.lean_deg == pytest.approx(0, abs=1e-6)
    assert st.hike_deg == pytest.approx(0, abs=1e-6)


def test_reach_meter_forward_lean_when_extended():
    tilt = _rot_x(25)
    m = _meter()
    _feed(m, upright() @ tilt.T)
    st = _feed(m, upright(20, "front") @ tilt.T, n=10, t0=1.0)
    assert st.lean_deg == pytest.approx(20, abs=1.0)


def test_reach_meter_holds_value_while_arm_bent():
    """팔을 굽히면 관절이 틀어지므로 값을 갱신하지 않고 직전 값 유지."""
    m = _meter()
    _feed(m, upright())
    _feed(m, upright(), t0=1.0)
    st = _feed(m, upright(30, "front", elbow_deg=80), n=10, t0=2.0)   # 굽힌 채 크게 숙임 (믿을 수 없는 값)
    assert not st.extended
    assert st.lean_deg == pytest.approx(0, abs=1e-6)


def test_reach_meter_shoulder_hike_reaching_side():
    m = _meter()
    _feed(m, upright())
    st = _feed(m, upright(hike_m=0.04), n=10, t0=1.0)
    assert st.hike_deg > 10


def test_reach_meter_head_tilt_is_not_hike():
    """고개를 옆으로 기울이면 한쪽 귀는 내려오고 다른 귀는 올라감 — 양 귀 가운데는 그대로라 으쓱으로 안 셈."""
    m = _meter()
    _feed(m, upright())
    w = upright()
    w[bg.R_EAR, 1] += 0.03
    w[bg.L_EAR, 1] -= 0.03
    st = _feed(m, w, n=10, t0=1.0)
    assert abs(st.hike_deg) < 1.0


def test_reach_meter_detects_with_only_reaching_shoulder():
    vis = ALL.copy()
    vis[[bg.L_SH, bg.L_EL, bg.L_WR, bg.L_EAR]] = 0.1     # 먼 쪽이 몸에 가려짐 (측면·대각)
    st = _feed(_meter(), upright(), vis=vis)
    assert st.detected and st.extended


def test_reach_meter_reset_baseline():
    m = _meter()
    _feed(m, upright(30, "front"))                            # 자리 잡는 중에 잡힌 잘못된 기준
    assert _feed(m, upright(), t0=1.0).lean_deg == pytest.approx(30, abs=1.0)
    m.reset_baseline()
    st = _feed(m, upright(), n=8, t0=2.0)
    assert st.lean_deg == pytest.approx(0, abs=1e-6)

