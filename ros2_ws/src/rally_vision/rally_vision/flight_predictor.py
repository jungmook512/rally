"""rally_vision/flight_predictor.py — 던진 공의 요격점 예측 (모드 3 핵심)
담당: 박성현

ROS 에 의존하지 않는 순수 클래스 (predictor.py 와 같은 방식). ROS 노드 래퍼는 다음 단계:
segmenter 의 velcro_ball 3D 위치(base_link, 센서 stamp)를 받아 update() → predict_crossing() →
/rally/perception/landing (PointStamped) 발행. 토픽 이름·의미는 docs/INTERFACES.md 4.1 을 따른다.

입력 (2026-10-11 실측 구성)
- 손목 RealSense + 측면 RealSense (컬러 848×480 @60) → YOLO(velcro_ball) 픽셀 → 두 카메라 삼각측량 3D [m, base_link]
- 측면 카메라 자세·두 카메라 시각 차(+10 ms)는 scripts/catch_lab/mode3_traj.py calib3 (번들 조정) 결과

모델
- 상태 x = [p(3), v(3)], 운동 ṗ = v, v̇ = g − k‖v‖v (중력 + 2차 공기 저항). k 는 10/10 투구 23회 번들 조정 0.075 /m
  (공 지름 63.4 mm · 73.51 g 로 계산한 0.013~0.018 보다 크다 — 추가 확인 필요. 생성자 인자로 바꿀 수 있다)
- 확장 칼만 필터: 예측은 RK4 + 수치 야코비안, 관측은 3D 위치(측정 잡음 meas_std_m).
- 요격점: 현재 상태를 앞으로 적분해 패드 평면 (p − plane_p)·n = 0 을 지나는 점·시각. 공분산은 같은 야코비안으로 전파해
  평면 위 2축 표준편차를 confidence 로 준다.
- 릴리스 직후 첫 관측 두세 개로 속도를 초기화한다 (첫 점 + 두 번째 점 차분, 큰 초기 공분산).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

G = np.array([0.0, 0.0, -9.81])


def _acc(v: np.ndarray, k: float) -> np.ndarray:
    return G - k * np.linalg.norm(v) * v


def _rk4(x: np.ndarray, k: float, h: float) -> np.ndarray:
    def f(s):
        return np.r_[s[3:], _acc(s[3:], k)]
    k1 = f(x)
    k2 = f(x + 0.5 * h * k1)
    k3 = f(x + 0.5 * h * k2)
    k4 = f(x + h * k3)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def propagate(x: np.ndarray, k: float, T: float, h: float = 0.004) -> np.ndarray:
    n = max(1, int(np.ceil(T / h)))
    hh = T / n
    for _ in range(n):
        x = _rk4(x, k, hh)
    return x


def _jac(x: np.ndarray, k: float, T: float) -> np.ndarray:
    """상태 전이 야코비안 (수치 미분)."""
    J = np.zeros((6, 6))
    base = propagate(x, k, T)
    for i in range(6):
        e = np.zeros(6)
        e[i] = 1e-5
        J[:, i] = (propagate(x + e, k, T) - base) / 1e-5
    return J


@dataclass
class Crossing:
    t: float                # 절대 시각 [s] (입력 t 와 같은 시계)
    point: np.ndarray       # base_link [m]
    std_m: np.ndarray       # 평면 위 2축 표준편차 [m] (u = 평면 안 수평, w = 평면 안 나머지)
    time_to_go: float       # 마지막 관측부터 교차까지 [s]


class FlightPredictor:
    def __init__(self, k: float = 0.075, meas_std_m: float = 0.015, accel_std: float = 1.5,
                 init_vel_std: float = 3.0, reject_reset: int = 0):
        self.k = k
        self.reject_reset = int(reject_reset)
        self.R = np.eye(3) * meas_std_m ** 2
        self.q = accel_std ** 2          # 모델 밖 가속(회전·검출 흔들림)을 흡수하는 백색 가속 잡음
        self.init_vel_std = init_vel_std
        self.reset()

    def reset(self) -> None:
        self.x = None
        self.P = None
        self.t = None
        self._first = None
        self.n = 0
        self._rej, self._rej_pts = 0, []

    # ------------------------------------------------------------------ 갱신
    def update(self, t: float, p: np.ndarray) -> None:
        p = np.asarray(p, float)
        if self._first is None:
            self._first = (t, p)
            self.n = 1
            return
        if self.x is None:
            t0, p0 = self._first
            dt = t - t0
            if dt <= 1e-4:
                return
            v0 = (p - p0) / dt - 0.5 * G * dt     # 두 점 차분 (중력 보정)
            self.x = np.r_[p, v0 + G * dt]
            self.P = np.diag([self.R[0, 0]] * 3 + [self.init_vel_std ** 2] * 3)
            self.t = t
            self.n = 2
            return
        dt = t - self.t
        if dt <= 0:
            return
        F = _jac(self.x, self.k, dt)
        self.x = propagate(self.x, self.k, dt)
        Gq = np.vstack([0.5 * dt ** 2 * np.eye(3), dt * np.eye(3)])
        self.P = F @ self.P @ F.T + self.q * Gq @ Gq.T
        H = np.hstack([np.eye(3), np.zeros((3, 3))])
        y = p - self.x[:3]
        S = H @ self.P @ H.T + self.R
        # 큰 잔차(검출 오류)는 버린다: 마할라노비스 거리 > 16 (3 자유도 99.9 %)
        # 10/13 실로봇: 놓는 순간 손과 섞인 점으로 처음 5개 궤적이 틀어지면 이후 진짜 점이 전부 버려져 '관측 5' 에서 멈춤(투구 11 중 6).
        # 연속 reject_reset 번 버려지면 필터 쪽이 틀린 것으로 보고 공분산을 처음 크기로 키운 뒤 받아들인다 (한두 번 튀는 점은 그대로 버림)
        # 10/13 오후: 공분산만 키우고 틀린 속도를 그대로 두면 한 점으로 속도가 크게 꺾여 교차점이 −1 m 밖으로 튐 → 버려진 연속 점들로
        # 처음처럼 다시 초기화 (버려진 첫·끝 두 점 차분 속도). 관측 수는 이어서 센다
        if float(y @ np.linalg.solve(S, y)) > 16.0 and self.n > 4:
            self._rej = getattr(self, "_rej", 0) + 1
            self._rej_pts = getattr(self, "_rej_pts", []) + [(t, p)]
            if self.reject_reset <= 0 or self._rej < self.reject_reset:   # 0 = 끔 (10/13 오후: 투구 28 중 18 번 리셋돼 예측이 늦고 std 2 m — 아침 3/5·3/3 때처럼 끔)
                self.t = t                                   # 상태는 이미 t 로 예측해 둠
                return
            pts, n_keep = self._rej_pts, self.n
            self.reset()
            self.update(*pts[0])
            self.update(*pts[-1])
            self.n = n_keep + len(pts)
            self.n_resets = getattr(self, "n_resets", 0) + 1
            return
        self._rej, self._rej_pts = 0, []
        Kg = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + Kg @ y
        self.P = (np.eye(6) - Kg @ H) @ self.P
        self.t = t
        self.n += 1

    def update_ray(self, t: float, P: np.ndarray, uv, pix_std: float = 3.0, gate: float = 13.8) -> bool:
        """한 카메라 픽셀 관측 갱신 (10/13 정정인 B). P = 3x4 투영행렬(base → 픽셀), uv = 공 픽셀.
        두 카메라가 같은 순간에 다 봐야 3D 점이 나오는 대신 카메라마다 프레임이 올 때 바로 '이 광선 위에 공' 으로 갱신한다.
        깊이는 그 프레임만으로는 모르지만 운동 모델로 앞뒤 관측과 묶여 맞춰진다. 초기화 전(x 없음)이면 안 씀.
        큰 잔차(마할라노비스 2 자유도 > gate, 99.9 %)는 버림 — 한 카메라 오검출. 반환: 받아들였는지."""
        if self.x is None or self.t is None:
            return False
        dt = t - self.t
        if dt < 0:
            return False
        if dt > 0:
            F = _jac(self.x, self.k, dt)
            x_pred = propagate(self.x, self.k, dt)
            Gq = np.vstack([0.5 * dt ** 2 * np.eye(3), dt * np.eye(3)])
            P_pred = F @ self.P @ F.T + self.q * Gq @ Gq.T
        else:
            x_pred, P_pred = self.x, self.P
        P = np.asarray(P, float)
        h = P @ np.r_[x_pred[:3], 1.0]
        if h[2] <= 1e-6:
            return False
        z = np.array([h[0] / h[2], h[1] / h[2]])
        J = np.zeros((2, 6))
        J[0, :3] = (P[0, :3] * h[2] - h[0] * P[2, :3]) / h[2] ** 2
        J[1, :3] = (P[1, :3] * h[2] - h[1] * P[2, :3]) / h[2] ** 2
        y = np.asarray(uv, float) - z
        S = J @ P_pred @ J.T + np.eye(2) * pix_std ** 2
        if float(y @ np.linalg.solve(S, y)) > gate:
            self.n_ray_rej = getattr(self, "n_ray_rej", 0) + 1
            return False
        Kg = P_pred @ J.T @ np.linalg.inv(S)
        self.x = x_pred + Kg @ y
        self.P = (np.eye(6) - Kg @ J) @ P_pred
        self.t = t
        self.n += 1
        return True

    @property
    def ready(self) -> bool:
        return self.x is not None

    # ------------------------------------------------------------------ 예측
    def predict_crossing(self, plane_p: np.ndarray, plane_n: np.ndarray, t_max: float = 2.0,
                         h: float = 0.002) -> Crossing | None:
        if self.x is None:
            return None
        n = np.asarray(plane_n, float)
        n = n / np.linalg.norm(n)
        x = self.x.copy()
        s_prev = float((x[:3] - plane_p) @ n)
        if s_prev <= 0:
            return None
        tc = 0.0
        while tc < t_max:
            x2 = _rk4(x, self.k, h)
            s = float((x2[:3] - plane_p) @ n)
            if s <= 0:
                a = s_prev / (s_prev - s)
                T = tc + a * h
                pt = x[:3] + a * (x2[:3] - x[:3])
                F = _jac(self.x, self.k, T)
                Pc = (F @ self.P @ F.T)[:3, :3]
                u = np.cross(n, [0.0, 0.0, 1.0])
                u = u / (np.linalg.norm(u) + 1e-12)
                w = np.cross(n, u)
                std = np.sqrt([max(u @ Pc @ u, 0.0), max(w @ Pc @ w, 0.0)])
                return Crossing(t=self.t + T, point=pt, std_m=std, time_to_go=T)
            x, s_prev, tc = x2, s, tc + h
        return None
