"""rally_vision/flight_tracking.py — 두 카메라 픽셀 → 3D 공 위치 → 투구 구간 → 요격점 (ROS 없는 순수 로직)
담당: 박성현

flight_predictor_node.py 가 YOLO 검출(픽셀)을 넣고, 여기서 3D·예측을 계산한다. 오프라인 검증(scripts/catch_lab/mode3_traj.py)과 같은 방식:
- 손목 카메라: 영상 시각의 카메라 자세(base ← camera_color_optical_frame), 측면 카메라: 고정 자세(side_calib.json)
- 측면 시각 + dt_side_s = 손목 시계 (10/11 캘리브레이션 +15 ms). 측면 검출 시각에 손목 픽셀을 선형 보간해 두 광선을 DLT 로 교차
- 재투영 오차가 max_reproj_px 를 넘는 점은 버림 (검출 오류·시각 불일치)
- ThrowSession: 로봇 쪽(−x)으로 빠르게 오는 점이 연속으로 나오면 투구 시작 → FlightPredictor(EKF, 중력 + 공기 저항) → 패드 평면 교차
좌표: base_link [m], 시각 [s] (ROS 시각을 float 로).
"""
from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from rally_vision.flight_predictor import Crossing, FlightPredictor


# ---------------------------------------------------------------- 기하
def projection_matrix(K: np.ndarray, T_cam_from_base: np.ndarray) -> np.ndarray:
    """3x4 투영행렬 (base 점 → 픽셀). T_cam_from_base = camera ← base (4x4)."""
    return np.asarray(K, float).reshape(3, 3) @ np.asarray(T_cam_from_base, float)[:3, :]


def triangulate(P1: np.ndarray, P2: np.ndarray, uv1, uv2) -> np.ndarray:
    """두 시점 DLT (cv2.triangulatePoints 와 같은 식). 반환 base 3D (3,)."""
    u1, v1 = uv1
    u2, v2 = uv2
    A = np.stack([u1 * P1[2] - P1[0], v1 * P1[2] - P1[1], u2 * P2[2] - P2[0], v2 * P2[2] - P2[1]])
    X = np.linalg.svd(A)[2][-1]
    return X[:3] / X[3]


def reproj_px(P: np.ndarray, X: np.ndarray, uv) -> float:
    x = P @ np.r_[X, 1.0]
    if x[2] <= 0:
        return float("inf")
    return float(np.hypot(x[0] / x[2] - uv[0], x[1] / x[2] - uv[1]))


@dataclass
class SideCalib:
    K: np.ndarray                 # 측면 카메라 내부 행렬 (camera_info 로 덮어씀)
    T_cam_from_base: np.ndarray   # 측면 카메라 ← base 4x4
    dt_side_s: float              # 측면 stamp + dt = 손목 시계
    k_drag: float                 # 번들 조정 공기 저항 [1/m]

    @staticmethod
    def load(path: str) -> "SideCalib":
        """mode3_traj.py calib3 결과 side_calib.json."""
        c = json.loads(open(path).read())
        T = np.eye(4)
        T[:3, :3] = np.array(c["R_side_base"])
        T[:3, 3] = np.array(c["t_side_base"])
        K = np.array(c["K_side"]) if "K_side" in c else np.eye(3)
        return SideCalib(K=K, T_cam_from_base=T, dt_side_s=float(c.get("dt_side_to_wrist_s", 0.0)),
                         k_drag=float(c.get("k_drag", 0.075)))


# ---------------------------------------------------------------- 두 카메라 짝짓기
@dataclass
class WristDet:
    t: float
    uv: tuple[float, float] | None   # None = 이 프레임에 공 없음
    P: np.ndarray                    # 그 시각의 손목 투영행렬


class StereoAssociator:
    """측면 검출마다 손목 픽셀을 측면 시각(손목 시계)으로 보간해 삼각측량.

    손목 검출이 측면 시각보다 늦은 것이 하나 들어올 때까지 측면 검출을 기다린다(최대 한 프레임, ~16 ms).
    """

    def __init__(self, P_side: np.ndarray, dt_side_s: float, max_gap_s: float = 0.025,
                 max_reproj_px: float = 6.0, max_wait_s: float = 0.06):
        self.P_side = P_side
        self.dt = dt_side_s
        self.max_gap = max_gap_s
        self.max_reproj = max_reproj_px
        self.max_wait = max_wait_s
        self.wrist: deque[WristDet] = deque(maxlen=12)
        self.pending: deque[tuple[float, tuple[float, float]]] = deque(maxlen=12)
        self.n_rejected = 0

    def add_wrist(self, d: WristDet) -> list[tuple[float, np.ndarray]]:
        if self.wrist and d.t <= self.wrist[-1].t:
            return []
        self.wrist.append(d)
        return self._flush()

    def add_side(self, t_side_stamp: float, uv: tuple[float, float] | None) -> list[tuple[float, np.ndarray]]:
        if uv is not None:
            self.pending.append((t_side_stamp + self.dt, uv))
        return self._flush()

    def _flush(self) -> list[tuple[float, np.ndarray]]:
        out = []
        if not self.wrist:
            return out
        latest = self.wrist[-1].t
        keep = deque(maxlen=self.pending.maxlen)
        for t, uv_s in self.pending:
            if t > latest:                       # 아직 뒤쪽 손목 프레임이 안 옴 (손목이 max_wait 넘게 늦으면 버림)
                if t - latest < self.max_wait:
                    keep.append((t, uv_s))
                continue
            X = self._at(t, uv_s)
            if X is not None:
                out.append((t, X))
        self.pending = keep
        return out

    def _at(self, t: float, uv_s) -> np.ndarray | None:
        ws = list(self.wrist)
        for a, b in zip(ws[:-1], ws[1:]):
            if a.t <= t <= b.t:
                break
        else:
            return None
        if a.uv is None or b.uv is None or b.t - a.t > self.max_gap:
            return None
        f = (t - a.t) / (b.t - a.t) if b.t > a.t else 0.0
        uv_w = (a.uv[0] + f * (b.uv[0] - a.uv[0]), a.uv[1] + f * (b.uv[1] - a.uv[1]))
        Pw = a.P if f < 0.5 else b.P
        X = triangulate(Pw, self.P_side, uv_w, uv_s)
        if max(reproj_px(Pw, X, uv_w), reproj_px(self.P_side, X, uv_s)) > self.max_reproj:
            self.n_rejected += 1
            return None
        return X


# ---------------------------------------------------------------- 투구 구간 + 예측
@dataclass
class ThrowSession:
    """3D 점 스트림 → 투구 시작 판정 → EKF → 패드 평면 교차.

    시작: 연속 start_pts 개 점의 구간 속도가 min_speed 이상이고 x 가 줄어듦(투구자 +x → 로봇), 첫 점이 min_start_x 보다 투구자 쪽.
          되던지기(+x 방향)·로봇 근처에서 떨어지는 공은 무시.
    발행 조건: 교차점이 패드 중심에서 max_cross_dist 안 (투구자 근처 바닥에 떨어지는 공이 먼 평면 연장선과 만나는 경우를 거름, 10/10 bag 6건).
    끝: 예측 교차 시각 + end_margin_s 지남, 또는 lost_s 동안 점 없음 → idle.
    """
    plane_p: np.ndarray
    plane_n: np.ndarray
    k_drag: float = 0.075
    meas_std_m: float = 0.015
    min_speed: float = 1.5
    start_pts: int = 3
    min_obs: int = 4
    min_start_x: float = 1.2
    max_cross_dist: float = 0.6
    lost_s: float = 0.25
    end_margin_s: float = 0.1
    state: str = "idle"
    fp: FlightPredictor | None = None
    last: Crossing | None = None
    t_release: float | None = None
    _buf: deque = field(default_factory=lambda: deque(maxlen=8))
    _t_last: float = -1e9

    def __post_init__(self):
        self.plane_p = np.asarray(self.plane_p, float)
        n = np.asarray(self.plane_n, float)
        self.plane_n = n / np.linalg.norm(n)

    def reset(self) -> None:
        self.state, self.fp, self.last, self.t_release = "idle", None, None, None
        self._buf.clear()

    def tick(self, now: float) -> None:
        """점이 없어도 주기적으로 불러 끝을 판정."""
        if self.state == "flight":
            if now - self._t_last > self.lost_s or (self.last is not None and now > self.last.t + self.end_margin_s):
                self.reset()

    def add(self, t: float, p: np.ndarray) -> Crossing | None:
        p = np.asarray(p, float)
        self.tick(t)
        self._t_last = t
        if self.state == "idle":
            self._buf.append((t, p))
            if self._started():
                self.state = "flight"
                pts = list(self._buf)[-self.start_pts:]
                self.t_release = pts[0][0]
                self.fp = FlightPredictor(k=self.k_drag, meas_std_m=self.meas_std_m)
                for tt, pp in pts:
                    self.fp.update(tt, pp)
                self._buf.clear()
            else:
                return None
        else:
            self.fp.update(t, p)
        if self.fp.n < self.min_obs:
            return None
        c = self.fp.predict_crossing(self.plane_p, self.plane_n)
        if c is None or np.linalg.norm(c.point - self.plane_p) > self.max_cross_dist:
            return None
        self.last = c
        return c

    def _started(self) -> bool:
        if len(self._buf) < self.start_pts:
            return False
        pts = list(self._buf)[-self.start_pts:]
        for (t0, p0), (t1, p1) in zip(pts[:-1], pts[1:]):
            dt = t1 - t0
            if dt <= 0 or dt > 0.05:
                return False
            v = (p1 - p0) / dt
            if np.linalg.norm(v) < self.min_speed or v[0] >= 0:
                return False
        # 투구자 쪽에서 시작하고, 패드 평면 앞(투구자 쪽)에 있어야 함
        return pts[0][1][0] >= self.min_start_x and float((pts[-1][1] - self.plane_p) @ self.plane_n) > 0.0


# ---------------------------------------------------------------- 검출 고르기 · 패드 위 공 (10/12 실제 받기 시도 뒤 추가)
class StaticFilter:
    """프레임마다 YOLO 후보 [(u, v, conf), ...] 중 하나를 고른다. 같은 자리에 오래 머무는 검출은 뺀다.

    10/12 정정인 PC: 손목 카메라는 패드 손잡이의 초록 브래킷(conf 0.82), 측면은 테이블 위 초록 장식(0.69~0.84)을 매 프레임 공으로
    봤고 노드가 conf 최고 하나만 쓰니 날아오는 공 대신 그것을 골라 예측 0 개. 공은 비행 중 한 칸(cell_px)에 몇 프레임만 머무르므로
    최근 window 프레임 중 frac 넘게 검출이 나온 칸은 '고정 물체'로 본다. 패드에 붙은 공도 고정이지만 그건 PadBallJudge 가 따로 본다.
    """

    def __init__(self, cell_px: float = 24.0, window: int = 120, frac: float = 0.5, min_frames: int = 30):
        self.cell_px, self.window, self.frac, self.min_frames = float(cell_px), int(window), float(frac), int(min_frames)
        self._hist: deque[set] = deque()
        self._count: dict[tuple[int, int], int] = {}

    def _cell(self, u: float, v: float) -> tuple[int, int]:
        return int(u // self.cell_px), int(v // self.cell_px)

    def static_cells(self) -> set[tuple[int, int]]:
        n = len(self._hist)
        if n < self.min_frames:
            return set()
        return {c for c, k in self._count.items() if k > self.frac * n}

    def pick(self, cands) -> tuple[float, float] | None:
        """cands: [(u, v, conf), ...] (빈 리스트 가능). 고정 칸이 아닌 후보 중 conf 최고의 (u, v). 없으면 None."""
        cells = {self._cell(u, v) for u, v, _ in cands}
        self._hist.append(cells)
        for c in cells:
            self._count[c] = self._count.get(c, 0) + 1
        while len(self._hist) > self.window:
            for c in self._hist.popleft():
                self._count[c] -= 1
                if self._count[c] <= 0:
                    del self._count[c]
        static = self.static_cells()
        live = [c for c in cands if self._cell(c[0], c[1]) not in static]
        if not live:
            return None
        u, v, _ = max(live, key=lambda c: c[2])
        return float(u), float(v)


def ray_plane_point(K: np.ndarray, T_cam_from_base: np.ndarray, uv, plane_p: np.ndarray, plane_n: np.ndarray) -> np.ndarray | None:
    """픽셀 uv 의 광선과 평면(plane_p, plane_n) 의 교점 (base, m). 카메라 뒤·평행이면 None."""
    T = np.asarray(T_cam_from_base, float)
    R, t = T[:3, :3], T[:3, 3]
    cam_pos = -R.T @ t
    d_cam = np.linalg.solve(np.asarray(K, float), np.array([uv[0], uv[1], 1.0]))
    d = R.T @ d_cam
    n = np.asarray(plane_n, float) / np.linalg.norm(plane_n)
    den = float(d @ n)
    if abs(den) < 1e-9:
        return None
    s = float((np.asarray(plane_p, float) - cam_pos) @ n) / den
    if s <= 0:
        return None
    return cam_pos + s * d


class PadBallJudge:
    """측면 카메라 검출 가운데 '패드 위에 있는 공'. 패드 중심(base, 로봇 TCP 로 실시간 계산)을 측면 영상에 투영하고,
    그 주변 radius_m(패드 반지름 7.5 cm + 여유) 안의 검출을 패드 평면과 교차해 base 3D 로 돌려준다.
    받기 판정(catch_ball)은 이 점이 몇 초 연속 들어오는지로 한다 — 힘 센서는 이동 관성·패드 끝 튕김과 구분이 안 됨 (10/12 결정)."""

    def __init__(self, K: np.ndarray, T_cam_from_base: np.ndarray, radius_m: float = 0.10):
        self.K, self.T, self.radius_m = np.asarray(K, float), np.asarray(T_cam_from_base, float), float(radius_m)

    def check(self, cands, pad_center: np.ndarray, plane_n: np.ndarray) -> tuple[np.ndarray, float] | None:
        """cands [(u, v, conf), ...] → 패드 중심에서 radius_m 안의 가장 가까운 검출 (base 3D 점, 중심까지 거리 m). 없으면 None."""
        pc = np.asarray(pad_center, float)
        x = self.T[:3, :3] @ pc + self.T[:3, 3]
        if x[2] <= 0.05:
            return None
        uv_c = (self.K @ x)[:2] / x[2]
        r_px = self.K[0, 0] * self.radius_m / x[2]
        best = None
        for u, v, _ in cands:
            if np.hypot(u - uv_c[0], v - uv_c[1]) > r_px:
                continue
            p = ray_plane_point(self.K, self.T, (u, v), pc, plane_n)
            if p is None:
                continue
            dist = float(np.linalg.norm(p - pc))
            if dist <= self.radius_m and (best is None or dist < best[1]):
                best = (p, dist)
        return best
