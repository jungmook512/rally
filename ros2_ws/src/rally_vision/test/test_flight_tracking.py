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
