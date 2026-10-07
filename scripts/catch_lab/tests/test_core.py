"""수학·검출 단위 테스트 (bag 없이, 빠름)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from catch_lab.ballistics import Plane, Sigma, find_release, fit_trajectory, intersect_plane  # noqa: E402
from catch_lab.camera import Camera, intrinsics_from_hfov  # noqa: E402
from catch_lab.detect import Blob, HSVRange, ball_center_depth_raw, find_ball  # noqa: E402

G = np.array([0.0, 9.81, 0.0])
RS = Camera(K=np.array([[605.0, 0, 424], [0, 605.0, 240], [0, 0, 1]]), D=np.zeros(5), width=848, height=480)


def _wc_cam() -> Camera:
    from tests.synth import look_at
    R, t = look_at(np.array([2.6, -0.3, 1.4]), np.array([0, -0.4, 1.4]), np.array([0, -1.0, 0]))
    return Camera(K=intrinsics_from_hfov(640, 480, 62), D=np.zeros(5), width=640, height=480, R=R, t=t)


def _throw(T=1.0, release=(0.1, 0.25, 2.6), target=(0.05, 0.12, 0.15)):
    p0, target = np.array(release), np.array(target)
    v0 = (target - p0 - 0.5 * G * T * T) / T
    return p0, v0, target


def _observe(p0, v0, cam, ts, px_noise, depth_rel, rng, with_depth=True):
    P = p0 + np.outer(ts, v0) + 0.5 * np.outer(ts ** 2, G)
    uv, z = cam.project(P)
    vis = cam.in_image(uv, z)
    uv = uv + rng.normal(0, px_noise, uv.shape)
    zc = (cam.to_cam(P)[:, 2]) * (1 + rng.normal(0, depth_rel, len(P)))
    if with_depth:
        return np.column_stack([ts, uv, zc])[vis]
    return np.column_stack([ts, uv])[vis]


def test_project_backproject_roundtrip():
    P = np.array([[0.3, -0.2, 1.5], [-0.5, 0.4, 2.5]])
    uv, z = RS.project(P)
    np.testing.assert_allclose(RS.backproject(uv, z), P, atol=1e-9)
    wc = _wc_cam()
    uv, z = wc.project(P)
    np.testing.assert_allclose(wc.backproject(uv, z), P, atol=1e-9)


def test_intersect_plane_exact():
    p0, v0, target = _throw()
    from catch_lab.ballistics import Fit
    f = Fit(p0=p0, v0=v0, t0=0.0, g=G, rms_rs_px=0, rms_wc_px=0, rms_depth_m=0, n_rs=0, n_wc=0)
    t_hit, P = intersect_plane(f, Plane(point=np.array([0, 0, 0.15]), normal=np.array([0, 0, 1.0])), 0.0)
    assert t_hit == pytest.approx(1.0, abs=1e-9)
    np.testing.assert_allclose(P, target, atol=1e-9)


def test_fit_noiseless_recovers_exactly():
    rng = np.random.default_rng(0)
    p0, v0, target = _throw()
    rs = _observe(p0, v0, RS, np.arange(0, 0.3, 1 / 60), 0, 0, rng)
    fit = fit_trajectory(rs, None, RS, None, G)
    np.testing.assert_allclose(fit.p0, p0, atol=1e-6)
    np.testing.assert_allclose(fit.v0, v0, atol=1e-5)


def test_webcam_reduces_error_with_noise():
    """깊이 잡음이 클 때 RealSense 만 vs 웹캠 추가: 웹캠이 도착점 오차를 줄여야 한다 (여러 번 평균)."""
    plane = Plane(point=np.array([0, 0.10, 0.15]), normal=np.array([0, 0, 1.0]))
    wc = _wc_cam()
    errs_rs, errs_both = [], []
    for seed in range(20):
        rng = np.random.default_rng(seed)
        p0, v0, target = _throw()
        rs = _observe(p0, v0, RS, np.arange(0, 0.3, 1 / 60), 1.0, 0.02, rng)
        w = _observe(p0, v0, wc, np.arange(0.005, 0.3, 1 / 30), 2.0, 0, rng, with_depth=False)
        f1 = fit_trajectory(rs, None, RS, None, G)
        f2 = fit_trajectory(rs, w, RS, wc, G, 0.0, Sigma())
        errs_rs.append(np.linalg.norm(intersect_plane(f1, plane, 0)[1] - target))
        errs_both.append(np.linalg.norm(intersect_plane(f2, plane, 0)[1] - target))
    assert np.median(errs_both) < np.median(errs_rs)
    assert np.median(errs_both) < 0.05            # 0.3 s 만 보고도 5 cm 이내


def test_webcam_time_offset_matters():
    """웹캠 시각을 30 ms 틀리게 주면 잔차가 커져야 한다 (dt 자동 추정의 근거)."""
    rng = np.random.default_rng(1)
    p0, v0, _ = _throw()
    wc = _wc_cam()
    rs = _observe(p0, v0, RS, np.arange(0, 0.3, 1 / 60), 0.5, 0.01, rng)
    ts = np.arange(0.005, 1.0, 1 / 30)
    w = _observe(p0, v0, wc, ts, 1.0, 0, rng, with_depth=False)
    good = fit_trajectory(rs, w, RS, wc, G, 0.0)
    bad = fit_trajectory(rs, w, RS, wc, G, 0.03)
    assert bad.rms_wc_px > 2 * good.rms_wc_px


def test_find_release_skips_hand_motion():
    rng = np.random.default_rng(2)
    p0, v0, _ = _throw()
    ts = np.arange(0, 0.3, 1 / 60)
    flight = _observe(p0, v0, RS, ts, 0.5, 0.01, rng)
    # 놓기 전 0.15 s: 아래에서 올라오는 손 (등속, 포물선 아님)
    th = np.arange(-0.15, 0, 1 / 60)
    Ph = p0 + np.outer(-th, [0, 2.0, 1.0])
    uv, z = RS.project(Ph)
    hand = np.column_stack([th, uv, z])
    k = find_release(np.vstack([hand, flight]), RS, G)
    assert k is not None and len(hand) - 2 <= k <= len(hand) + 1


def test_find_ball_center_and_radius():
    import cv2
    img = np.full((480, 640, 3), 90, np.uint8)
    cv2.circle(img, (int(300.25 * 16), int(200.75 * 16)), 12 * 16, (40, 255, 200), -1, cv2.LINE_AA, shift=4)
    b = find_ball(img, HSVRange())
    assert b is not None
    assert abs(b.u - 300.25) < 0.3 and abs(b.v - 200.75) < 0.3
    assert abs(b.r - 12) < 2.0          # 블러·낮은 채도 기준으로 약간 커짐. 반지름은 깊이 읽을 영역 크기에만 쓴다


def test_find_ball_ignores_skin_and_white():
    import cv2
    img = np.full((480, 640, 3), 90, np.uint8)
    cv2.circle(img, (100, 100), 30, (120, 160, 220), -1)     # 피부색 (BGR)
    cv2.circle(img, (300, 300), 30, (255, 255, 255), -1)
    assert find_ball(img, HSVRange()) is None


def test_raw_depth_mapping():
    """정렬 안 된 깊이 영상(렌즈 15 mm 옆)에서 공 중심 z 를 맞게 찾는다."""
    import cv2
    color = RS
    K_d = np.array([[425.0, 0, 423], [0, 425.0, 241], [0, 0, 1]])
    depth_cam = Camera(K=K_d, D=np.zeros(5), width=848, height=480, t=np.array([0.015, 0, 0]))
    P = np.array([0.4, -0.2, 2.2])
    r_ball = 0.0325
    uv, z = color.project(P[None])
    blob = Blob(u=uv[0, 0], v=uv[0, 1], r=605 * r_ball / z[0], area=0, circularity=1)
    d = np.full((480, 848), 4.0, np.float32)
    uvd, zd = depth_cam.project(P[None])
    cv2.circle(d, tuple(np.round(uvd[0]).astype(int)), int(425 * r_ball / zd[0]), float(zd[0] - r_ball), -1)
    zc = ball_center_depth_raw(d, blob, color, depth_cam, r_ball)
    assert zc == pytest.approx(P[2], abs=0.005)
