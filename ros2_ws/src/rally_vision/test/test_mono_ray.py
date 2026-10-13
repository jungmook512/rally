"""한 카메라 픽셀 관측(update_ray)·MonoThrowSession — 가짜 두 카메라로 (10/13 B)."""
import numpy as np

from rally_vision.flight_predictor import FlightPredictor, propagate
from rally_vision.flight_tracking import MonoThrowSession, projection_matrix

K = np.array([[600.0, 0, 424], [0, 600.0, 240], [0, 0, 1]])


def look_at(eye, target):
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z])                       # camera ← base 회전
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, -R @ eye
    return projection_matrix(K, T)


P_W = look_at(np.array([0.4, -0.3, 0.6]), [3.0, -0.2, 0.3])      # 손목: 로봇 쪽에서 투구자를 봄
P_S = look_at(np.array([1.6, -1.6, 0.6]), [1.8, -0.2, 0.4])      # 측면
X0 = np.r_[3.4, -0.15, 0.0, -5.0, -0.1, 3.0]                       # 투구자 손에서 로봇 쪽으로
k = 0.073


def pix(P, p):
    h = P @ np.r_[p, 1.0]
    return h[:2] / h[2]


def test_rays_only_converge_to_true_state():
    rng = np.random.default_rng(0)
    fp = FlightPredictor(k=k)
    x = X0.copy()
    fp.update(0.0, x[:3])
    x1 = propagate(X0, k, 1 / 60)
    fp.update(1 / 60, x1[:3])                      # 3D 두 점으로 초기화 (속도 대충)
    t, acc = 1 / 60, 0
    for i in range(2, 30):
        t = i / 60
        p = propagate(X0, k, t)[:3]
        P = P_S if i % 2 else P_W                  # 카메라 번갈아 하나씩만
        acc += fp.update_ray(t, P, pix(P, p) + rng.normal(0, 1.0, 2))
    assert acc >= 25
    truth = propagate(X0, k, t)
    assert np.linalg.norm(fp.x[:3] - truth[:3]) < 0.02
    assert np.linalg.norm(fp.x[3:] - truth[3:]) < 0.3


def test_ray_outlier_rejected():
    fp = FlightPredictor(k=k)
    fp.update(0.0, X0[:3])
    fp.update(1 / 60, propagate(X0, k, 1 / 60)[:3])
    for i in range(2, 12):
        t = i / 60
        fp.update_ray(t, P_S, pix(P_S, propagate(X0, k, t)[:3]))
    n = fp.n
    assert not fp.update_ray(12 / 60, P_S, pix(P_S, propagate(X0, k, 12 / 60)[:3]) + [150.0, 0.0])
    assert fp.n == n


def test_session_backdates_release_from_side_motion():
    plane_p, plane_n = np.array([0.35, -0.33, 0.5]), np.array([0.7, -0.12, 0.7])
    s = MonoThrowSession(plane_p=plane_p, plane_n=plane_n / np.linalg.norm(plane_n), k_drag=k, min_speed=2.5)
    t_rel = 0.0
    for i in range(-10, 0):                        # 놓기 전: 측면만 공을 봄 (손에 들고 휘두름 — 빠르게 움직임)
        t = t_rel + i / 60
        if i >= -6:
            s.add_ray(t, P_S, pix(P_S, X0[:3] + np.r_[0.08 * (i + 6), 0, 0]), "side")
        else:
            s.add_ray(t, P_S, pix(P_S, X0[:3] + np.r_[0.0, 0, 0]), "side")
    for i in range(0, 3):                          # 손목이 다시 보며 3D 점 → 시작 판정
        t = (i + 2) / 60
        s.add(t, propagate(X0, k, t)[:3])
    assert s.state == "flight"
    assert s.t_start == 2 / 60
    assert s.t_release < s.t_start - 0.05          # 측면 움직임 시작까지 앞당김
    for i in range(5, 25):
        t = i / 60
        s.add_ray(t, P_S if i % 2 else P_W, pix(P_S if i % 2 else P_W, propagate(X0, k, t)[:3]), "side" if i % 2 else "wrist")
    assert s.n_rays >= 18
    assert s.add(0.5, X0[:3]) is None              # 비행 중 3D 점은 안 씀


def test_held_ball_false_start_ends():
    """10/13: 4 m 밖 손에 든 공이 흔들려 시작으로 잡혀도 비행이 끝나야 다음 진짜 투구를 잡는다."""
    from rally_vision.flight_tracking import ThrowSession
    plane_p, plane_n = np.array([0.35, -0.33, 0.5]), np.array([0.7, -0.12, 0.7])
    s = ThrowSession(plane_p=plane_p, plane_n=plane_n / np.linalg.norm(plane_n), k_drag=k, min_speed=2.5)
    rng = np.random.default_rng(1)
    t = 0.0
    for i in range(3):                                   # 가짜 시작: 로봇 쪽으로 빠르게 세 점
        t = i / 60
        s.add(t, np.r_[4.0 - 0.06 * i, 0.0, -0.27])
    assert s.state == "flight"
    for i in range(3, 74):                               # 손에 든 공 (제자리 흔들림) 계속 → max_flight_s 1.2 s 에서 끝
        t = i / 60
        s.add(t, np.r_[3.9, 0.0, -0.27] + rng.normal(0, 0.03, 3))
    assert s.state == "idle"


def test_fit_linear_recovers_throw_from_alternating_cameras():
    """3D 점 없이 두 카메라가 번갈아 한 장씩 본 픽셀만으로 궤적 (10/13 rays)."""
    from rally_vision.flight_rays import Obs, fit_linear
    rng = np.random.default_rng(2)
    obs = []
    for i in range(18):
        t = i / 60
        P = P_S if i % 2 else P_W
        obs.append(Obs(t, "side" if i % 2 else "wrist", P, pix(P, propagate(X0, k, t)[:3]) + rng.normal(0, 1.0, 2)))
    obs[7].uv = obs[7].uv + [80.0, 0.0]                      # 오검출 하나
    r = fit_linear(obs, obs[-1].t, k)
    truth = propagate(X0, k, obs[-1].t)
    assert r is not None and r["ok"] and not r["keep"][7]
    assert np.linalg.norm(r["x"][:3] - truth[:3]) < 0.02
    assert np.linalg.norm(r["x"][3:] - truth[3:]) < 0.3


def test_ray_session_starts_and_predicts():
    from rally_vision.flight_rays import RayThrowSession
    plane_p = propagate(X0, k, 0.5)[:3]
    n = -propagate(X0, k, 0.5)[3:]
    s = RayThrowSession(plane_p=plane_p, plane_n=n / np.linalg.norm(n), k_drag=k)
    c = None
    for i in range(0, 25):
        t = i / 60
        P, cam = (P_S, "side") if i % 2 else (P_W, "wrist")
        s.add(t, cam, P, pix(P, propagate(X0, k, t)[:3]))
        c = s.latest() or c
    assert s.state == "flight" and s.t_release is not None and s.t_release <= 0.02
    assert c is not None and np.linalg.norm(c.point - plane_p) < 0.03


def test_ray_session_one_camera_never_starts():
    from rally_vision.flight_rays import RayThrowSession
    s = RayThrowSession(plane_p=np.array([0.35, -0.33, 0.5]), plane_n=np.array([0.7, -0.12, 0.7]), k_drag=k)
    for i in range(0, 30):
        t = i / 60
        s.add(t, "side", P_S, pix(P_S, propagate(X0, k, t)[:3]))
    assert s.state == "idle"


def test_trim_lead_drops_in_hand_phase():
    """손에 있던 구간(포물선 아님)이 앞에 섞여도 잘라내고 진짜 궤적을 맞춘다 (10/13 실로봇: 초반 예측이 아래로 35~70 cm)."""
    from rally_vision.flight_rays import Obs, fit_linear, fit_trim_lead
    obs = []
    for i in range(-6, 0):                                   # 손에 든 채 가속: 위치가 포물선보다 뒤처짐
        t = i / 60
        p = X0[:3] + X0[3:] * t * 0.3 + np.r_[0, 0, -0.04 * (i + 6) / 6]
        P = P_S if i % 2 else P_W
        obs.append(Obs(t, "side" if i % 2 else "wrist", P, pix(P, p)))
    for i in range(0, 16):
        t = i / 60
        P = P_S if i % 2 else P_W
        obs.append(Obs(t, "side" if i % 2 else "wrist", P, pix(P, propagate(X0, k, t)[:3])))
    truth = propagate(X0, k, 15 / 60)
    raw = fit_linear(obs, 15 / 60, k)
    r, kept = fit_trim_lead(obs, k)
    assert kept[0].t >= -1 / 60 - 1e-9                       # 손 구간 대부분 잘림
    assert np.linalg.norm(r["x"][3:] - truth[3:]) < np.linalg.norm(raw["x"][3:] - truth[3:])
    assert np.linalg.norm(r["x"][3:] - truth[3:]) < 0.3



def test_fit_rejects_nan_cov(monkeypatch):
    """#182 리뷰: 공분산에 NaN 이 섞이면 ok 가 거짓 — NaN 이면 std 비교(>)가 통과해 버린다."""
    from rally_vision import flight_rays as fr
    from rally_vision.flight_rays import Obs, fit_linear
    rng = np.random.default_rng(0)
    obs = []
    for i in range(12):
        t = i / 60
        P = P_S if i % 2 else P_W
        obs.append(Obs(t, "side" if i % 2 else "wrist", P, pix(P, propagate(X0, k, t)[:3]) + rng.normal(0, 1.0, 2)))
    assert fit_linear(obs, obs[-1].t, k)["ok"]
    real_inv = np.linalg.inv
    monkeypatch.setattr(fr.np.linalg, "inv", lambda m: real_inv(m) * np.nan)
    assert not fit_linear(obs, obs[-1].t, k)["ok"]
