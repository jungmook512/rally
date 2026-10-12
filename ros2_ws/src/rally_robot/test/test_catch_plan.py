"""모드 3 받기 계산 (catch_plan) — 로봇 없이. CI 에 scipy 가 없으면 건너뛴다."""
import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from rally_robot.skills.catch_plan import (  # noqa: E402
    clamp_cross, hold_run, intercept, pad_caught, pad_center, parse_info, plane_axes, quality_ok)

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


def test_parse_info():
    d = parse_info([100.5, 0.4, 0.02, 0.03, 7.0, 0.2])
    assert d["t_cross"] == 100.5 and d["n_obs"] == 7.0 and d["since_release"] == 0.2
    assert parse_info([1.0, 2.0]) is None


def test_down_limited_separately():
    """아래(+w, 투구자 쪽)는 down_max_m 로 따로 자름 — 대기 자세 −9.4 cm 자기충돌 (10/11)."""
    _, u, w = plane_axes(N45)
    down = intercept(READY, PAD, N45, PAD + 0.20 * w, 0.15, 0.15, down_max_m=0.08)
    assert abs(down.dw - 0.08) < 1e-9 and down.clipped
    up = intercept(READY, PAD, N45, PAD - 0.20 * w, 0.15, 0.15, down_max_m=0.08)
    assert abs(up.dw + 0.15) < 1e-9
    same = intercept(READY, PAD, N45, PAD + 0.20 * w, 0.15, 0.15)
    assert abs(same.dw - 0.15) < 1e-9


def test_quality_gate():
    assert quality_ok((0.03, 0.05), 6, 0.10, 4)
    assert not quality_ok((0.03, 0.12), 6, 0.10, 4)        # 한 축이라도 크면 미달
    assert not quality_ok((0.03, 0.05), 3, 0.10, 4)


def test_clamp_cross():
    assert clamp_cross(100.3, 100.0, 1.5) == 100.3
    assert clamp_cross(110.0, 100.0, 1.5) == 101.5           # 이상한 교차 시각은 1.5 s 로


def test_hold_run_and_pad_caught():
    """패드 위 공 관측(60 Hz 흉내, 틈 0.3 s 허용): 마지막으로 끝나는 연속 구간만 센다."""
    t = [0.0, 0.02, 0.04, 1.0, 1.02, 1.04, 1.06]              # 0.04 → 1.0 틈 0.96 s 로 끊김
    run, n = hold_run(t, 0.3)
    assert run == pytest.approx(0.06) and n == 4
    assert hold_run([], 0.3) == (0.0, 0)
    long = [i * 0.02 for i in range(160)]                     # 3.18 s 연속
    assert pad_caught(long, long[-1] + 0.1, 3.0, 0.3)
    assert not pad_caught(long, long[-1] + 0.5, 3.0, 0.3)     # 마지막 관측이 0.3 s 보다 오래됨 → 지금은 안 보임
    assert not pad_caught(long[:100], long[99], 3.0, 0.3)     # 1.98 s 뿐
    gappy = long[:80] + [v + 0.2 for v in long[80:]]          # 가운데 0.22 s 틈 하나 (허용)
    assert pad_caught(gappy, gappy[-1], 3.0, 0.3)
