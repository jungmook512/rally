"""flight_tracking 단위 테스트 — 로봇·카메라 없이. 가상 두 카메라로 합성 투구를 찍어 요격점이 맞는지."""
import numpy as np

from rally_vision.flight_predictor import propagate
from rally_vision.flight_tracking import StereoAssociator, ThrowSession, WristDet, projection_matrix, reproj_px, triangulate

K = np.array([[605.0, 0, 430], [0, 605.0, 245], [0, 0, 1]])
PLANE_P = np.array([0.327, -0.271, 0.531])
PLANE_N = np.array([0.7071, 0.0, 0.7071])
K_DRAG = 0.075


def look_at(eye, target):
    """camera ← base 4x4 (광학 프레임: z 앞, x 오른쪽, y 아래)."""
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = -R @ eye
    return T


P_WRIST = projection_matrix(K, look_at(np.array([0.30, -0.15, 0.60]), [2.0, -0.3, 1.2]))
P_SIDE = projection_matrix(K, look_at(np.array([0.74, -2.50, 0.85]), [1.5, -0.3, 1.2]))


def project(P, X):
    x = P @ np.r_[X, 1.0]
    return (x[0] / x[2], x[1] / x[2])


def throw(t_end=1.2, dt=1 / 60):
    """투구자(+x 3 m) → 패드 쪽. 반환 [(t, p)] (손목 시계)."""
    x = np.r_[3.0, -0.35, 0.85, -4.6, 0.10, 3.4]
    out, t = [], 0.0
    while t < t_end:
        out.append((t, x[:3].copy()))
        x = propagate(x, K_DRAG, dt)
        t += dt
    return out


def truth_crossing():
    x, t, h = np.r_[3.0, -0.35, 0.85, -4.6, 0.10, 3.4], 0.0, 0.0005
    s_prev = (x[:3] - PLANE_P) @ PLANE_N
    while True:
        x2 = propagate(x, K_DRAG, h)
        s = (x2[:3] - PLANE_P) @ PLANE_N
        if s <= 0:
            a = s_prev / (s_prev - s)
            return t + a * h, x[:3] + a * (x2[:3] - x[:3])
        x, s_prev, t = x2, s, t + h


def test_triangulate_roundtrip():
    X = np.array([1.5, -0.4, 1.3])
    Y = triangulate(P_WRIST, P_SIDE, project(P_WRIST, X), project(P_SIDE, X))
    assert np.allclose(X, Y, atol=1e-6)
    assert reproj_px(P_SIDE, Y, project(P_SIDE, X)) < 1e-4


def test_pipeline_predicts_crossing():
    rng = np.random.default_rng(0)
    dt_side = 0.015
    traj = throw()
    ts = np.array([t for t, _ in traj])
    ps = np.array([p for _, p in traj])

    def pos(t):
        return np.array([np.interp(t, ts, ps[:, i]) for i in range(3)])

    assoc = StereoAssociator(P_SIDE, dt_side)
    sess = ThrowSession(plane_p=PLANE_P, plane_n=PLANE_N, k_drag=K_DRAG)
    t_true, p_true = truth_crossing()
    last = None
    first_t = None
    for i in range(70):
        t_w = 0.1 + i / 60                      # 손목 프레임 (손목 시계)
        t_s = t_w + 0.006 - dt_side             # 측면 stamp (측면 시계): 손목 시계로는 t_w + 6 ms
        uv_w = np.array(project(P_WRIST, pos(t_w))) + rng.normal(0, 0.5, 2)
        uv_s = np.array(project(P_SIDE, pos(t_s + dt_side))) + rng.normal(0, 0.5, 2)
        pts = assoc.add_wrist(WristDet(t=t_w, uv=tuple(uv_w), P=P_WRIST)) + assoc.add_side(t_s, tuple(uv_s))
        for t, X in pts:
            assert np.linalg.norm(X - pos(t)) < 0.02          # 보간·삼각측량 오차 < 2 cm
            c = sess.add(t, X)
            if c is not None:
                last = c
                first_t = first_t if first_t is not None else t
    assert last is not None and first_t is not None
    assert first_t < 0.25                                     # 릴리스 0.1 s 뒤 몇 프레임 안에 첫 예측
    assert np.linalg.norm(last.point - p_true) < 0.02
    assert abs(last.t - t_true) < 0.01


def test_return_throw_ignored():
    """로봇에서 투구자 쪽(+x)으로 가는 공은 투구로 보지 않는다."""
    sess = ThrowSession(plane_p=PLANE_P, plane_n=PLANE_N)
    for i in range(10):
        t = i / 60
        assert sess.add(t, np.array([0.8 + 3.0 * t, -0.3, 1.0 + 2.0 * t])) is None
    assert sess.state == "idle"


def test_session_ends_after_loss():
    sess = ThrowSession(plane_p=PLANE_P, plane_n=PLANE_N)
    for t, p in throw()[:10]:
        sess.add(t, p)
    assert sess.state == "flight"
    sess.tick(10.0)
    assert sess.state == "idle"


# ---------------------------------------------------------------- 10/12 추가: 고정 오검출 제외 · 패드 위 공
from rally_vision.flight_tracking import PadBallJudge, StaticFilter, ray_plane_point  # noqa: E402


def test_static_filter_drops_fixed_object_keeps_moving_ball():
    """초록 장식(한 자리, conf 0.8) + 움직이는 공(conf 0.6): 처음엔 장식을 고르지만 창이 차면 공을 고른다."""
    f = StaticFilter(cell_px=24, window=60, frac=0.5, min_frames=30)
    picks = []
    for i in range(90):
        cands = [(120.0, 400.0, 0.8)]                      # 고정 물체
        if i >= 40:
            cands.append((300.0 + 8 * (i - 40), 200.0, 0.6))   # 공: 프레임당 8 px 이동
        picks.append(f.pick(cands))
    assert picks[0] == (120.0, 400.0)                      # 아직 통계 없음 → conf 최고
    assert picks[-1] == (300.0 + 8 * 49, 200.0)            # 고정 칸 제외 → 공
    assert any(p is None for p in picks[30:40])            # 공 없고 장식만 → None (장식은 안 고름)


def test_static_filter_window_forgets():
    f = StaticFilter(cell_px=24, window=40, frac=0.5, min_frames=10)
    for _ in range(40):
        f.pick([(10.0, 10.0, 0.9)])
    assert f.static_cells()
    for _ in range(40):
        f.pick([])
    assert not f.static_cells()


def test_ray_plane_point_roundtrip():
    T = look_at(np.array([0.9, -2.6, 0.85]), PLANE_P)
    X = PLANE_P + 0.05 * np.array([0.0, -1.0, 0.0])        # 평면 위 점 (u 방향은 평면 안)
    uv = (K @ (T[:3, :3] @ X + T[:3, 3]))
    uv = uv[:2] / uv[2]
    back = ray_plane_point(K, T, uv, PLANE_P, PLANE_N)
    assert np.allclose(back, X, atol=1e-6)


def test_pad_ball_judge_radius():
    T = look_at(np.array([0.9, -2.6, 0.85]), PLANE_P)
    j = PadBallJudge(K, T, radius_m=0.10)
    n = PLANE_N / np.linalg.norm(PLANE_N)
    u = np.cross(n, [0, 0, 1.0]); u /= np.linalg.norm(u)

    def px(X):
        x = T[:3, :3] @ X + T[:3, 3]
        q = K @ x
        return (q[0] / q[2], q[1] / q[2], 0.7)
    on_pad = PLANE_P + 0.05 * u
    off_pad = PLANE_P + 0.20 * u
    hit = j.check([px(off_pad), px(on_pad)], PLANE_P, PLANE_N)
    assert hit is not None and abs(hit[1] - 0.05) < 1e-3 and np.allclose(hit[0], on_pad, atol=1e-3)
    assert j.check([px(off_pad)], PLANE_P, PLANE_N) is None
    assert j.check([], PLANE_P, PLANE_N) is None
