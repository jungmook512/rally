"""모드 3 받기 계산 (catch_plan) — 로봇 없이. CI 에 scipy 가 없으면 건너뛴다."""
import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from rally_robot.skills.catch_plan import impact_detected, intercept, pad_center, parse_info, plane_axes  # noqa: E402

N45 = [0.7071, 0.0, 0.7071]
READY = [335.0, -224.0, 478.0, 90.0, 90.0, 0.0]           # posx (mm, ZYZ deg) — 자세 값은 계산에 영향 없음(위치만 옮김)
PAD = np.array([0.327, -0.271, 0.531])


def test_plane_axes_right_is_minus_y():
    n, u, w = plane_axes(N45)
    assert np.allclose(u, [0, -1, 0], atol=1e-6)            # u = 로봇 기준 오른쪽
    assert abs(n @ u) < 1e-9 and abs(n @ w) < 1e-9 and w[2] < 0   # w = 면 안, 투구자 쪽 아래


def test_pad_center_identity_orientation():
    p = pad_center([300.0, 0.0, 500.0, 0.0, 0.0, 0.0], [0.1, 0.0, 0.0])
    assert np.allclose(p, [0.4, 0.0, 0.5])


def test_intercept_moves_pad_onto_prediction():
    _, u, w = plane_axes(N45)
    pred = PAD + 0.08 * u - 0.05 * w
    plan = intercept(READY, PAD, N45, pred, right_max_m=0.15, updown_max_m=0.15)
    moved = (np.array(plan.tcp_posx[:3]) - np.array(READY[:3])) / 1000.0
    assert np.allclose(PAD + moved, pred, atol=1e-6)
    assert plan.tcp_posx[3:] == READY[3:] and not plan.clipped


def test_intercept_never_moves_left_and_clips():
    _, u, w = plane_axes(N45)
    left = intercept(READY, PAD, N45, PAD - 0.10 * u, 0.15, 0.15)
    assert left.du == 0.0 and left.clipped                    # 왼쪽 공은 따라가지 않음
    far = intercept(READY, PAD, N45, PAD + 0.40 * u + 0.30 * w, 0.15, 0.15)
    assert far.du == pytest.approx(0.15) and far.dw == pytest.approx(0.15)
    assert far.offset_m == pytest.approx(0.5)                 # 반경 판정은 자르기 전 거리


def test_intercept_ignores_normal_component():
    n, _, _ = plane_axes(N45)
    plan = intercept(READY, PAD, N45, PAD + 0.05 * n, 0.15, 0.15)
    assert np.allclose(plan.tcp_posx[:3], READY[:3])


def test_impact_window():
    samples = [(0.00, 1.0), (0.10, 1.2), (0.20, 9.0), (0.30, 1.1)]
    assert impact_detected(samples, 1.0, 0.15, 0.40, 4.0) == (True, pytest.approx(8.0))
    hit, peak = impact_detected(samples, 1.0, 0.25, 0.40, 4.0)   # 창 밖의 충격은 안 셈
    assert not hit and peak == pytest.approx(0.1)


def test_parse_info():
    d = parse_info([100.5, 0.4, 0.02, 0.03, 7.0, 0.2])
    assert d["t_cross"] == 100.5 and d["n_obs"] == 7.0 and d["since_release"] == 0.2
    assert parse_info([1.0, 2.0]) is None
