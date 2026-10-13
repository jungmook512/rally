"""rally_vision/flight_rays.py — 두 카메라 픽셀을 각자 타임스탬프 그대로 써서 던진 공 궤적 맞추기 (3D 점 짝짓기 없음, ROS 없는 순수 로직)
담당: 박성현 · 10/13 정정인 요청으로 추가 (B 안: "3D 점은 안 맞추고 타임스탬프로만")

지금 방식(flight_tracking.StereoAssociator + ThrowSession)은 측면 프레임 시각에 손목 픽셀을 앞뒤 두 장으로 보간해 3D 점을 만들고,
3D 점이 연달아 3 개 나와야 투구 시작으로 본다. 던지는 순간 손목 카메라가 공을 놓치면(10/13 실로봇: 손·흐림) 시작이 늦게 잡히고
결정 시점(놓은 뒤 0.25 s)에 점이 5~6 개뿐이다.

여기서는 관측 하나 = (카메라 투영행렬 P, 픽셀 uv, 그 카메라의 시각 t). 짝짓기·보간 없이
  1) 시작 후보: 어느 카메라든 픽셀이 프레임마다 onset_px 넘게 움직이기 시작한 시각 t0
  2) t0 이후 두 카메라 관측 전부로 궤적(중력 + 공기 저항) 하나를 맞춤 — fit_linear: 저항을 상수 속도 크기로 근사하면
     관측마다 선형 식 두 줄이라 행렬 한 번으로 풀린다 (scipy 반복 맞춤은 한 번 0.1~2 s 라 실시간 불가, 10/13)
  3) 맞춘 궤적이 투구자 쪽에서 출발해 로봇 쪽으로 빠르게 오고(vx ≤ −min_vx) 재투영 잔차가 작으면 투구 확정 → t_release = t0
  4) 확정 뒤엔 관측이 올 때마다 놓음부터 지금까지 전부로 다시 맞춤(가장 최근 관측 시각 기준 상태·공분산) → 패드 평면 교차점·std
깊이는 두 카메라가 비행 중 서로 다른 순간에라도 한 번씩 보면 맞춰진다. 한 카메라만으로는 맞추지 않는다(min_per_cam).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from rally_vision.flight_predictor import Crossing, FlightPredictor, G, _rk4


@dataclass
class Obs:
    t: float
    cam: str
    P: np.ndarray          # 3x4 base → 픽셀
    uv: np.ndarray         # (2,)


def project(P: np.ndarray, X: np.ndarray) -> np.ndarray | None:
    h = P @ np.r_[X, 1.0]
    if h[2] <= 1e-6:
        return None
    return h[:2] / h[2]


def ray(P: np.ndarray, uv) -> tuple[np.ndarray, np.ndarray]:
    """카메라 중심과 픽셀 방향 단위 벡터 (base)."""
    M = P[:, :3]
    c = -np.linalg.solve(M, P[:, 3])
    d = np.linalg.solve(M, np.r_[np.asarray(uv, float), 1.0])
    return c, d / np.linalg.norm(d)


def _drag_basis(dt: np.ndarray, c: float):
    """속도 크기를 상수(c = k·|v|)로 본 공기 저항 운동의 닫힌 해. X(t_ref + dt) = p + f·v + g  (f 스칼라, g 3벡터).
    v̇ = G − c·v → v(dt) = G/c + (v − G/c)e^(−c·dt), X = p + (G/c)·dt + (v − G/c)(1 − e^(−c·dt))/c.
    10/13 확인: 실제(속도 따라 변하는 저항, RK4)와 0.8 s 비행에서 1.4 cm 안."""
    dt = np.asarray(dt, float)
    if c < 1e-6:
        return dt, 0.5 * np.outer(dt ** 2, G)
    f = (1.0 - np.exp(-c * dt)) / c
    return f, np.outer(dt - f, G / c)


def fit_linear(obs: list[Obs], t_ref: float, k: float, speed0: float = 5.5, iters: int = 3, outlier_px: float = 15.0):
    """t_ref 시각의 상태 [p, v] 를 픽셀 관측들로 선형 최소제곱 (반복마다 깊이 가중·저항 c·튀는 관측 갱신).
    관측 하나 = 투영 P·[X;1] 이 픽셀 uv 를 지나는 두 줄 선형 식 (u·P3 − P1)·[X;1] = 0, (v·P3 − P2)·[X;1] = 0 → [p, v] 에 선형.
    반환 dict(x=[p,v] at t_ref, cov 6x6, res_px 관측별 재투영 오차, keep 쓴 관측 마스크, ok)."""
    n = len(obs)
    if n < 4:
        return None
    ts = np.array([o.t for o in obs]) - t_ref
    Ps = np.stack([o.P for o in obs])
    uv = np.stack([o.uv for o in obs])
    keep = np.ones(n, bool)
    depth = np.ones(n)
    c = k * speed0
    x = None
    for it in range(iters + 1):
        f, g = _drag_basis(ts, c)
        A = np.empty((2 * n, 6))
        b = np.empty(2 * n)
        for j in range(2):
            a = uv[:, j:j + 1] * Ps[:, 2, :] - Ps[:, j, :]            # (n, 4)
            w = 1.0 / depth
            A[j::2, :3] = a[:, :3] * w[:, None]
            A[j::2, 3:] = a[:, :3] * (f * w)[:, None]
            b[j::2] = -(np.einsum("ij,ij->i", a[:, :3], g) + a[:, 3]) * w
        m = np.repeat(keep, 2)
        try:
            x, *_ = np.linalg.lstsq(A[m], b[m], rcond=None)
        except np.linalg.LinAlgError:
            return None
        X = x[:3] + f[:, None] * x[3:] + g                             # 관측 시각 위치
        h = np.einsum("nij,nj->ni", Ps, np.c_[X, np.ones(n)])
        if np.any(h[keep, 2] <= 1e-6):
            return None
        depth = np.maximum(h[:, 2], 0.1)
        res = np.hypot(h[:, 0] / h[:, 2] - uv[:, 0], h[:, 1] / h[:, 2] - uv[:, 1])
        c = k * float(np.linalg.norm(x[3:]))
        if it < iters:
            new_keep = res <= max(outlier_px, 3.0 * float(np.median(res[keep])))
            if new_keep.sum() >= 4:
                keep = new_keep
    Am = A[np.repeat(keep, 2)]
    sig2 = max(float(np.median(res[keep] ** 2)), 4.0)
    try:
        cov = np.linalg.inv(Am.T @ Am) * sig2
    except np.linalg.LinAlgError:
        cov = None
    return dict(x=x, cov=cov, res_px=res, keep=keep, ok=cov is not None and bool(np.all(np.isfinite(x))))


def fit_trim_lead(obs: list[Obs], k: float, lead_px: float = 4.0, lead_k: float = 2.5, max_trim_frac: float = 0.4,
                  min_per_cam: int = 2, outlier_px: float = 15.0, rounds: int = 2):
    """fit_linear + 앞쪽 자르기 (10/13 실로봇: 손에 있던 구간이 섞여 초반 예측이 패드면 아래로 35~70 cm 쏠림).
    맞춘 뒤 맨 앞 관측부터 재투영 오차가 max(lead_px, lead_k × 중앙값) 넘는 것이 이어지면 그 구간을 빼고 다시 맞춘다
    (최대 전체의 max_trim_frac, 카메라마다 min_per_cam 개는 남김). 반환 (결과 dict 또는 None, 남은 관측 시간순)."""
    obs = sorted(obs, key=lambda o: o.t)
    r = fit_linear(obs, obs[-1].t, k, outlier_px=outlier_px) if len(obs) >= 4 else None
    for _ in range(rounds):
        if r is None or not r["ok"]:
            break
        res = r["res_px"]
        thr = max(lead_px, lead_k * float(np.median(res[r["keep"]])))
        i = 0
        while i < len(obs) and res[i] > thr:
            i += 1
        if i == 0 or i > max_trim_frac * len(obs):
            break
        rest = obs[i:]
        cams = {}
        for o in rest:
            cams[o.cam] = cams.get(o.cam, 0) + 1
        if len(cams) < 2 or min(cams.values()) < min_per_cam or len(rest) < 4:
            break
        r2 = fit_linear(rest, rest[-1].t, k, outlier_px=outlier_px)
        if r2 is None or not r2["ok"]:
            break
        obs, r = rest, r2
    return r, obs


@dataclass
class RayThrowSession:
    """관측 스트림(카메라별 픽셀 + 시각) → 투구 시작 판정 → 관측마다 다시 맞춤 → 패드 평면 교차점."""
    plane_p: np.ndarray
    plane_n: np.ndarray
    k_drag: float = 0.075
    onset_px: float = 6.0            # 시작 후보: 프레임당 픽셀 이동
    onset_steps: int = 2             # 그런 이동이 연달아 몇 번
    min_start_x: float = 1.2         # 확정: 놓음 시각 위치가 투구자 쪽
    min_vx: float = 2.5              # 확정: 로봇 쪽(−x) 속도
    min_per_cam: int = 2             # 확정: 카메라마다 관측 최소 수 (깊이)
    min_span_s: float = 0.05
    max_rms_px: float = 5.0          # 확정: 재투영 오차 중앙값
    max_pos_std_m: float = 0.3       # 확정: 위치 표준편차 (한 카메라뿐이면 깊이가 안 정해져 커짐)
    outlier_px: float = 15.0
    lead_px: float = 4.0             # 앞쪽 자르기: 맨 앞부터 이 픽셀(또는 중앙값 × lead_k) 넘게 안 맞는 관측이 이어지면 손에 있던 구간
    lead_k: float = 2.5
    cand_max_s: float = 0.6          # 후보가 이만큼 지나도 확정 못 하면 버림
    slide_after_s: float = 0.10      # 확정 실패 때 후보 구간이 이보다 길면 최근 이만큼만 남김 (던지기 전 팔 휘두름)
    max_cross_dist: float = 0.6
    min_obs: int = 4
    lost_s: float = 0.25
    end_margin_s: float = 0.1
    max_flight_s: float = 1.2
    no_cross_s: float = 0.45         # 놓은 뒤 이만큼 지나도 패드 근처(max_cross_dist) 교차점이 한 번도 안 나오면 투구 아님
    no_any_cross_s: float = 0.3      # 놓은 뒤 이만큼 지나도 패드 평면과 만나는 궤적이 한 번도 안 나오면 투구 아님
    _any_cross: bool = False
    end_reason: str = ""
    state: str = "idle"
    fp: FlightPredictor | None = None
    last: Crossing | None = None
    t_release: float | None = None
    n_fit: int = 0
    _hist: dict = field(default_factory=dict)          # cam → deque[Obs] (시작 후보 찾기)
    _cand: float | None = None
    _obs: list = field(default_factory=list)
    _t_last: float = -1e9

    def __post_init__(self):
        self.plane_p = np.asarray(self.plane_p, float)
        n = np.asarray(self.plane_n, float)
        self.plane_n = n / np.linalg.norm(n)

    def reset(self) -> None:
        self.state, self.fp, self.last, self.t_release, self._cand, self._obs = "idle", None, None, None, None, []
        self._any_cross = False

    def tick(self, now: float) -> None:
        if self.state == "flight":
            why = None
            if now - self._t_last > self.lost_s:
                why = "lost"                                 # 두 카메라 다 lost_s 동안 공 못 봄
            elif self.last is not None and now > self.last.t + self.end_margin_s:
                why = "crossed"                              # 예측 교차 시각 지남
            elif self.t_release is not None and now - self.t_release > self.max_flight_s:
                why = "max_flight"
            elif self.last is None and self.t_release is not None and now - self.t_release > self.no_cross_s:
                why = "no_pad_cross"                         # 던지기 전 동작이 투구로 잡혀 버티는 동안 진짜 투구를 놓침 (10/13 bag 3·7번)
            elif not self._any_cross and self.t_release is not None and now - self.t_release > self.no_any_cross_s:
                why = "no_cross"
            if why:
                self.end_reason = why
                self.reset()
        elif self._cand is not None and now - self._cand > self.cand_max_s:
            self._cand, self._obs = None, []

    def add(self, t: float, cam: str, P: np.ndarray, uv) -> None:
        """한 카메라 한 프레임 (공 픽셀이 없으면 uv=None — 시각만 진행)."""
        self.tick(t)
        if uv is None:
            return
        o = Obs(t=t, cam=cam, P=np.asarray(P, float), uv=np.asarray(uv, float))
        if self.state == "flight":
            self._obs.append(o)
            self._t_last = max(self._t_last, t)
            r, kept = fit_trim_lead(self._obs, self.k_drag, self.lead_px, self.lead_k, min_per_cam=self.min_per_cam, outlier_px=self.outlier_px)
            self.n_fit += 1
            if r is not None and r["ok"]:
                if kept[0].t > self.t_release + 1e-6:          # 손에 있던 앞쪽을 잘라냈으면 놓음 시각도 뒤로
                    self.t_release, self._obs = kept[0].t, kept
                self._set(r)
            return
        h = self._hist.setdefault(cam, deque(maxlen=12))
        h.append(o)
        if self._cand is None:
            on = self._onset(h)
            if on is None:
                return
            self._cand = on
            self._obs = [x for hh in self._hist.values() for x in hh if x.t >= on - 1e-6]
        else:
            self._obs.append(o)
        self._try_start()

    def _onset(self, h) -> float | None:
        """최근 onset_steps 번 연달아 프레임당 onset_px 넘게 움직였으면 그 처음 시각."""
        pts = list(h)[-self.onset_steps - 1:]
        if len(pts) < self.onset_steps + 1:
            return None
        for a, b in zip(pts[:-1], pts[1:]):
            if not (0 < b.t - a.t <= 0.04 and np.hypot(*(b.uv - a.uv)) >= self.onset_px):
                return None
        return pts[0].t

    def _try_start(self) -> None:
        obs = self._obs
        cams = {}
        for o in obs:
            cams[o.cam] = cams.get(o.cam, 0) + 1
        if len(cams) < 2 or min(cams.values()) < self.min_per_cam:
            return
        t_ref = max(o.t for o in obs)
        if t_ref - min(o.t for o in obs) < self.min_span_s:
            return
        r, kept_lead = fit_trim_lead(obs, self.k_drag, self.lead_px, self.lead_k, min_per_cam=self.min_per_cam, outlier_px=self.outlier_px)
        self.n_fit += 1
        if r is None or not r["ok"]:
            self._slide()
            return
        if kept_lead[0].t > self._cand + 1e-6:
            self._obs, self._cand = kept_lead, kept_lead[0].t
        obs = kept_lead                                      # r 의 관측 순서(시간순)와 맞춤
        kept = [o for o, k in zip(obs, r["keep"]) if k]
        kc = {}
        for o in kept:
            kc[o.cam] = kc.get(o.cam, 0) + 1
        f, g = _drag_basis(np.array([self._cand - t_ref]), self.k_drag * float(np.linalg.norm(r["x"][3:])))
        p_start = r["x"][:3] + f[0] * r["x"][3:] + g[0]
        pos_std = float(np.sqrt(max(np.max(np.diag(r["cov"])[:3]), 0.0)))
        if (len(kc) < 2 or min(kc.values()) < self.min_per_cam or float(np.median(r["res_px"][r["keep"]])) > self.max_rms_px
                or p_start[0] < self.min_start_x or r["x"][3] > -self.min_vx or pos_std > self.max_pos_std_m):
            self._slide()
            return
        self.state, self.t_release, self.fp, self._t_last = "flight", self._cand, FlightPredictor(k=self.k_drag), t_ref
        self._set(r)

    def _set(self, r) -> None:
        fp = self.fp
        fp.x, fp.P, fp.t, fp.n = r["x"].copy(), r["cov"], max(o.t for o in self._obs), int(r["keep"].sum())
        fp._first = (self.t_release, r["x"][:3])

    def _slide(self) -> None:
        """확정 실패: 후보 구간이 slide_after_s 보다 길면 최근 slide_after_s 관측만 남기고 후보 시각을 그 처음으로.
        (10/13 bag 10번: 한 개씩 빼면 던지기 전 팔 휘두름이 계속 섞여 0.15 s 늦게 확정)"""
        obs = sorted(self._obs, key=lambda o: o.t)
        if len(obs) > 2 and obs[-1].t - obs[0].t > self.slide_after_s:
            keep = [o for o in obs if o.t >= obs[-1].t - self.slide_after_s]
            self._obs, self._cand = keep, keep[0].t

    def latest(self) -> Crossing | None:
        if self.state != "flight" or self.fp is None or self.fp.n < self.min_obs:
            return None
        c = self.fp.predict_crossing(self.plane_p, self.plane_n)
        if c is None:
            return None
        self._any_cross = True
        if np.linalg.norm(c.point - self.plane_p) > self.max_cross_dist:
            return None
        self.last = c
        return c
