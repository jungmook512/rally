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
                 init_vel_std: float = 3.0):
        self.k = k
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
        if float(y @ np.linalg.solve(S, y)) > 16.0 and self.n > 4:
            self.t = t
            return
        Kg = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + Kg @ y
        self.P = (np.eye(6) - Kg @ H) @ self.P
        self.t = t
        self.n += 1

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
