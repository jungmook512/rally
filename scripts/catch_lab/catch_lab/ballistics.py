"""포물선 모델과 두 카메라 동시 피팅.

모델 (기준 좌표계 C, 시각 t, 기준 시각 t0):
    p(t) = p0 + v0·τ + ½·g·τ²,   τ = t − t0
g 는 측정한 중력 벡터(C 좌표, |g| ≈ 9.81). 공기 저항은 무시한다.

관측:
    rs  (N,4) [t, u, v, z]   RealSense 컬러 픽셀 + 공 중심 깊이 z (없으면 nan)
    wc  (M,3) [t, u, v]      웹캠 픽셀. 웹캠 시각은 t_wc + dt 로 RealSense 시계에 맞춘다.
"""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from .camera import Camera


@dataclass
class Sigma:
    rs_px: float = 1.5           # RealSense 픽셀 오차 [px]
    wc_px: float = 3.0           # 웹캠 픽셀 오차 [px] (자동 노출 잔상 포함)
    depth_a: float = 0.01        # 깊이 오차 σ = a + b·z² [m]
    depth_b: float = 0.01


@dataclass
class Fit:
    p0: np.ndarray
    v0: np.ndarray
    t0: float
    g: np.ndarray
    rms_rs_px: float
    rms_wc_px: float
    rms_depth_m: float
    n_rs: int
    n_wc: int

    def at(self, t: np.ndarray) -> np.ndarray:
        tau = np.asarray(t, dtype=np.float64).reshape(-1, 1) - self.t0
        return self.p0 + tau * self.v0 + 0.5 * tau ** 2 * self.g


@dataclass
class Plane:
    point: np.ndarray            # 평면 위 한 점 (패드 중심), C 좌표
    normal: np.ndarray           # 단위 법선

    def basis(self, g: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """평면 안의 (가로, 세로) 단위벡터. 세로 = 중력을 평면에 투영한 방향(아래가 +)."""
        n = self.normal / np.linalg.norm(self.normal)
        down = g - np.dot(g, n) * n
        if np.linalg.norm(down) < 1e-6:          # 평면이 바닥과 평행하면 세로를 정의할 수 없음
            down = np.cross(n, [1.0, 0, 0])
        down /= np.linalg.norm(down)
        return np.cross(down, n), down

    def to_2d(self, P: np.ndarray, g: np.ndarray) -> np.ndarray:
        h, v = self.basis(g)
        d = np.asarray(P) - self.point
        return np.array([np.dot(d, h), np.dot(d, v)])


def linear_init(P: np.ndarray, t: np.ndarray, t0: float, g: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """3D 점들로 p0, v0 선형 최소제곱 (g 고정)."""
    tau = t - t0
    A = np.column_stack([np.ones_like(tau), tau])
    B = P - 0.5 * tau[:, None] ** 2 * g
    sol, *_ = np.linalg.lstsq(A, B, rcond=None)
    return sol[0], sol[1]


def fit_trajectory(rs: np.ndarray, wc: np.ndarray, rs_cam: Camera, wc_cam: Camera | None,
                   g: np.ndarray, dt: float = 0.0, sigma: Sigma | None = None) -> Fit | None:
    """두 카메라 관측을 한 포물선으로 동시 피팅. RealSense 깊이 있는 점이 3개 미만이면 None."""
    sigma = sigma or Sigma()
    rs = np.asarray(rs, dtype=np.float64).reshape(-1, 4)
    wc = np.asarray(wc, dtype=np.float64).reshape(-1, 3) if wc is not None and wc_cam is not None \
        else np.zeros((0, 3))
    has_z = np.isfinite(rs[:, 3])
    if has_z.sum() < 3:
        return None
    t0 = rs[0, 0]
    P = rs_cam.backproject(rs[has_z, 1:3], rs[has_z, 3])
    p0, v0 = linear_init(P, rs[has_z, 0], t0, g)

    t_wc = wc[:, 0] + dt
    z_rs = rs[has_z, 3]
    sd = sigma.depth_a + sigma.depth_b * z_rs ** 2

    def model(theta: np.ndarray, t: np.ndarray) -> np.ndarray:
        tau = (t - t0)[:, None]
        return theta[:3] + tau * theta[3:] + 0.5 * tau ** 2 * g

    def residuals(theta: np.ndarray) -> np.ndarray:
        out = []
        uv, _ = rs_cam.project(model(theta, rs[:, 0]))
        out.append(((uv - rs[:, 1:3]) / sigma.rs_px).ravel())
        out.append((model(theta, rs[has_z, 0])[:, 2] - z_rs) / sd)
        if len(wc):
            uvw, zw = wc_cam.project(model(theta, t_wc))
            r = (uvw - wc[:, 1:3]) / sigma.wc_px
            r[zw <= 0] = 1e3                      # 웹캠 뒤로 간 해는 버린다
            out.append(r.ravel())
        return np.concatenate(out)

    sol = least_squares(residuals, np.concatenate([p0, v0]), loss="soft_l1", f_scale=2.0,
                        x_scale="jac", max_nfev=200)
    # 깊이는 가끔 배경·손을 읽어 수십 cm 튄다 → 4σ 넘는 깊이를 버리고 한 번 더
    zres = np.abs(model(sol.x, rs[has_z, 0])[:, 2] - z_rs) / sd
    if np.any(zres > 4) and (zres <= 4).sum() >= 3:
        keep = zres <= 4
        idx = np.flatnonzero(has_z)
        has_z = np.zeros(len(rs), dtype=bool)
        has_z[idx[keep]] = True
        z_rs, sd = z_rs[keep], sd[keep]
        sol = least_squares(residuals, sol.x, loss="soft_l1", f_scale=2.0, x_scale="jac", max_nfev=200)
    th = sol.x
    uv, _ = rs_cam.project(model(th, rs[:, 0]))
    rms_rs = float(np.sqrt(np.mean(np.sum((uv - rs[:, 1:3]) ** 2, axis=1))))
    rms_d = float(np.sqrt(np.mean((model(th, rs[has_z, 0])[:, 2] - z_rs) ** 2)))
    rms_wc = float("nan")
    if len(wc):
        uvw, _ = wc_cam.project(model(th, t_wc))
        rms_wc = float(np.sqrt(np.mean(np.sum((uvw - wc[:, 1:3]) ** 2, axis=1))))
    return Fit(p0=th[:3], v0=th[3:], t0=t0, g=g, rms_rs_px=rms_rs, rms_wc_px=rms_wc,
               rms_depth_m=rms_d, n_rs=len(rs), n_wc=len(wc))


def intersect_plane(fit: Fit, plane: Plane, t_after: float) -> tuple[float, np.ndarray] | None:
    """포물선이 평면을 처음 지나는 시각과 점. t_after 이후 해만. 없으면 None.
    n·(p0 + v0τ + ½gτ² − q) = 0  →  ½(n·g)τ² + (n·v0)τ + n·(p0−q) = 0"""
    n = plane.normal / np.linalg.norm(plane.normal)
    a = 0.5 * np.dot(n, fit.g)
    b = np.dot(n, fit.v0)
    c = np.dot(n, fit.p0 - plane.point)
    if abs(a) < 1e-9:
        roots = [-c / b] if abs(b) > 1e-9 else []
    else:
        disc = b * b - 4 * a * c
        if disc < 0:
            return None
        s = np.sqrt(disc)
        roots = [(-b - s) / (2 * a), (-b + s) / (2 * a)]
    taus = sorted(r for r in roots if r + fit.t0 > t_after)
    if not taus:
        return None
    t_hit = taus[0] + fit.t0
    return t_hit, fit.at(np.array([t_hit]))[0]


def find_release(rs: np.ndarray, rs_cam: Camera, g: np.ndarray | None, min_frames: int = 4,
                 max_rms_px: float = 3.0) -> int | None:
    """손에서 떠난 첫 프레임 인덱스. 던지기 전 손 동작은 중력 포물선이 아니라서 앞쪽을 포함하면 잔차가 커진다.
    앞에서부터 하나씩 잘라 RealSense 만으로 피팅하고, 잔차가 바닥 수준(최소값 × 1.3 + 0.3 px)까지
    내려온 첫 지점을 고른다. 바닥 자체가 max_rms_px 를 넘으면 포물선이 아닌 구간으로 보고 None."""
    rs = np.asarray(rs, dtype=np.float64).reshape(-1, 4)
    rms = []
    for k in range(0, len(rs) - min_frames + 1):
        if g is None:                       # 중력 방향을 아직 모를 때 (IMU 없음)
            r = fit_free_gravity(rs[k:], rs_cam)
            rms.append(r[1] if r is not None else np.inf)
            continue
        fit = fit_trajectory(rs[k:], None, rs_cam, None, g)
        rms.append(fit.rms_rs_px if fit is not None else np.inf)
    if not rms or not np.isfinite(min(rms)):
        return None
    floor = min(rms)
    if floor > max_rms_px:
        return None
    for k, r in enumerate(rms):
        if r <= floor * 1.3 + 0.3:
            return k
    return None


def fit_free_gravity(rs: np.ndarray, rs_cam: Camera,
                     sigma: Sigma | None = None) -> tuple[np.ndarray, float] | None:
    """중력 벡터까지 모르는 값으로 두고 한 투구를 피팅 → (g, 픽셀 잔차 rms). IMU 가 없을 때 중력 방향 추정용.
    미지수 9개 (p0, v0, g). 깊이 있는 점이 6개 미만이면 None."""
    sigma = sigma or Sigma()
    rs = np.asarray(rs, dtype=np.float64).reshape(-1, 4)
    has_z = np.isfinite(rs[:, 3])
    if has_z.sum() < 6:
        return None
    t0 = rs[0, 0]
    tau_all = rs[:, 0] - t0
    P = rs_cam.backproject(rs[has_z, 1:3], rs[has_z, 3])
    tau = tau_all[has_z]
    A = np.column_stack([np.ones_like(tau), tau, 0.5 * tau ** 2])
    sol, *_ = np.linalg.lstsq(A, P, rcond=None)                 # 초기값: 3D 점에 2차식
    z_rs = rs[has_z, 3]
    sd = sigma.depth_a + sigma.depth_b * z_rs ** 2

    def residuals(th: np.ndarray) -> np.ndarray:
        def m(t):
            return th[:3] + t[:, None] * th[3:6] + 0.5 * t[:, None] ** 2 * th[6:9]
        uv, _ = rs_cam.project(m(tau_all))
        return np.concatenate([((uv - rs[:, 1:3]) / sigma.rs_px).ravel(), (m(tau)[:, 2] - z_rs) / sd])

    res = least_squares(residuals, sol.ravel(), loss="soft_l1",
                        f_scale=2.0, x_scale="jac", max_nfev=300)
    th = res.x
    uv, _ = rs_cam.project(th[:3] + tau_all[:, None] * th[3:6] + 0.5 * tau_all[:, None] ** 2 * th[6:9])
    return th[6:9], float(np.sqrt(np.mean(np.sum((uv - rs[:, 1:3]) ** 2, axis=1))))


def combine_gravity(gs: list[np.ndarray], max_angle_deg: float = 25.0) -> tuple[np.ndarray, float, int]:
    """투구별 g 추정을 합친다. 방향만 쓰고 크기는 9.81 로. 중앙 방향에서 25° 넘게 벗어난 것은 버림.
    반환: (g, 남은 것들의 각도 흩어짐 deg, 쓴 개수)"""
    dirs = np.array([g / np.linalg.norm(g) for g in gs if np.linalg.norm(g) > 1.0])
    if not len(dirs):
        raise ValueError("중력 추정에 쓸 투구가 없음")
    med = np.median(dirs, axis=0)
    med /= np.linalg.norm(med)
    ang = np.degrees(np.arccos(np.clip(dirs @ med, -1, 1)))
    keep = dirs[ang <= max_angle_deg]
    d = keep.mean(axis=0)
    d /= np.linalg.norm(d)
    spread = float(np.degrees(np.sqrt(np.mean(np.arccos(np.clip(keep @ d, -1, 1)) ** 2))))
    return d * 9.81, spread, len(keep)


def find_flight(rs: np.ndarray, rs_cam: Camera, g: np.ndarray | None, seed: int = 6, seed_rms_px: float = 3.0,
                gate_px: float = 8.0, gate_rel: float = 0.4, max_miss: int = 2, min_len: int = 8,
                radii: np.ndarray | None = None, k_step: int = 2) -> np.ndarray | None:
    """구간 안에서 '날아가는 부분'의 프레임 인덱스 배열 (중간에 빠진 프레임은 제외). 실시간 추적과 같은 방식:
    시작 후보 k 마다 seed 프레임을 중력 포물선에 맞춰 보고(잔차 seed_rms_px 이하), 그 피팅으로 다음 프레임 위치를
    예측해 오차가 문턱(gate_px 또는 공 반지름×gate_rel 중 큰 값) 안이면 붙이고 다시 피팅한다.
    연속 max_miss 번 벗어나면 그 후보는 끝. 가장 길게 이어진 후보를 고른다.
    (0.1 초짜리 seed 는 손 동작에도 잘 맞으므로 '처음 맞는 곳'이 아니라 '가장 길게 이어지는 곳'이 비행이다.)
    g=None 이면 중력도 자유 변수 (seed 를 12 로 늘리고 |g| 가 7~13 인 경우만)."""
    rs = np.asarray(rs, dtype=np.float64).reshape(-1, 4)
    free = g is None
    if free:
        seed = max(seed, 12)

    def fit(seg):
        if free:
            r = fit_free_gravity(seg, rs_cam)
            if r is None or not 7.0 <= np.linalg.norm(r[0]) <= 13.0:
                return None, np.inf
            return fit_trajectory(seg, None, rs_cam, None, r[0]), r[1]
        f = fit_trajectory(seg, None, rs_cam, None, g)
        return f, (f.rms_rs_px if f is not None else np.inf)

    best: list[int] = []
    for k in range(0, len(rs) - seed + 1, k_step):
        if len(best) >= len(rs) - k:
            break                                   # 남은 프레임으로는 더 길어질 수 없음
        f, rms = fit(rs[k:k + seed])
        if f is None or rms > seed_rms_px:
            continue
        idx = list(range(k, k + seed))
        miss = 0
        for j in range(k + seed, len(rs)):
            uv, z = rs_cam.project(f.at(rs[j:j + 1, 0]))
            r_px = radii[j] if radii is not None else 0.0
            if z[0] > 0 and np.linalg.norm(uv[0] - rs[j, 1:3]) <= max(gate_px, gate_rel * r_px):
                idx.append(j)
                miss = 0
                if len(idx) % 2 == 0:               # 두 프레임마다 다시 피팅 (속도)
                    f2, _ = fit(rs[idx])
                    if f2 is not None:
                        f = f2
            else:
                miss += 1
                if miss > max_miss:
                    break
        if len(idx) > len(best):
            best = idx
    # 앞쪽에 붙은 손 동작 프레임 떼기: 나머지로 피팅했을 때 첫 프레임만 유난히 벗어나면 제외
    while len(best) > max(min_len, seed):
        f, _ = fit(rs[best[1:]])
        if f is None:
            break
        uv, _ = rs_cam.project(f.at(rs[best, 0]))
        res = np.linalg.norm(uv - rs[best, 1:3], axis=1)
        if res[0] > max(2.0, 3.0 * np.median(res[1:])):
            best = best[1:]
        else:
            break
    return np.array(best) if len(best) >= min_len else None
