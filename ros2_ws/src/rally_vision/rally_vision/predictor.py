"""rally_vision/predictor.py — 모드 1용: 칼만 필터(위치·속도) + 마찰 감속 모델 → 굴러가는 공의 정지 위치·시간, 마찰 계수 온라인 추정. 던진 공은 flight_predictor.py
담당: 정동휘 (10/7 박성현에게서 넘겨받음). tracker_node 연결은 박성현
보류 (#69, 10/7): 모드 1 정지 예측은 밀기 전 레시피로 한다. 이 클래스는 모드 3 에 칼만을 먼저 쓴 뒤 모드 1 에 붙일지 정할 때까지 유지만.

ROS 에 의존하지 않는 순수 클래스다. tracker_node 가 공 트랙 하나마다 RollingBallPredictor 를 하나 두고
매 프레임 update(t, xyz) 를 부른 뒤 결과를 TrackedObject 의 velocity·predicted_stop·predicted_stop_time·
prediction_confidence·is_moving 에 채운다.

모델
- 공은 가이드 길(skills.yaml mode1.lanes) 안에서만 구르므로 길 방향 1D 로 본다.
  위치를 길 축으로 투영 → s(길 따라 거리), d(옆 오차). 칼만은 s 에만, d·z 는 지수 평균.
- 상태 [s, v], 측정 s. 굴러가는 동안 일정 감속 a(= 구름 마찰 μ·g) 를 제어 입력으로 넣는다.
  정지 거리 v²/(2a), 정지까지 시간 v/a.
- 로봇이 미는 동안(가속·등속 구간)은 감속 모델이 틀리다. 속도가 최고점보다 확실히(칼만 속도 표준편차 2배)
  떨어진 프레임 수로 confidence 를 준다 — 손을 떠나 감속하는 게 보이기 전에는 낮다.
- a 는 skills.yaml mode1.rolling.decel_init_mps2 에서 시작해, 시도가 끝날 때마다(finish_trial) 실제 궤적에
  포물선을 맞춰 얻은 값으로 지수 평균 갱신한다.

입력 좌표는 base_link [m], 시각은 header.stamp 를 초로 바꾼 값 (센서 원본 시각 기준이어야 dt 가 맞다).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class LaneAxis:
    """가이드 길 하나 — 입구 xy 와 진행 방향 (dir_deg: +x 기준 반시계, skills.yaml 과 같은 정의)."""
    start_xy: np.ndarray
    dir_unit: np.ndarray

    @classmethod
    def from_cfg(cls, lane: dict) -> LaneAxis:
        """skills.yaml mode1.lanes[i] ({start: [x, y, z], dir_deg}) → LaneAxis."""
        rad = math.radians(float(lane["dir_deg"]))
        return cls(np.asarray(lane["start"][:2], dtype=float), np.array([math.cos(rad), math.sin(rad)]))

    @property
    def normal(self) -> np.ndarray:
        return np.array([-self.dir_unit[1], self.dir_unit[0]])

    def to_sd(self, xy: np.ndarray) -> tuple[float, float]:
        """base_link xy → (s 길 따라 거리, d 진행 방향 왼쪽 + 옆 오차) [m]."""
        rel = np.asarray(xy, dtype=float) - self.start_xy
        return float(rel @ self.dir_unit), float(rel @ self.normal)

    def to_xy(self, s: float, d: float) -> np.ndarray:
        return self.start_xy + s * self.dir_unit + d * self.normal


@dataclass
class RollingParams:
    """skills.yaml mode1.rolling 과 같은 키. 단위는 이름 끝에."""
    decel_init_mps2: float = 0.30
    decel_range_mps2: tuple[float, float] = (0.05, 1.5)
    decel_update_alpha: float = 0.3      # finish_trial 에서 새 추정값 반영 비율
    meas_std_m: float = 0.01             # 공 중심 측정 잡음 (마스크 깊이 중앙값 기준 가정)
    accel_std_mps2: float = 0.5          # 감속 모델이 못 설명하는 가속 (밀기, 길 벽 마찰 등)
    stop_speed_mps: float = 0.02         # 이보다 느리면 멈춘 것으로
    stop_frames: int = 5                 # 이만큼 연속으로 느리거나, 이 프레임 동안 칼만 위치가 meas_std 안에 머물면 is_moving=False
    move_start_m: float = 0.02           # 정지 자리에서 이만큼 벗어나야 움직이기 시작한 것으로 (잡음 들썩임 무시)
    min_frames: int = 5                  # 감속이 확인된 프레임이 이만큼 쌓여야 confidence 최대
    stop_std_ref_m: float = 0.03         # 정지 위치 표준편차가 이 값이면 confidence 절반
    smooth_alpha: float = 0.2            # d·z 지수 평균 비율

    @classmethod
    def from_cfg(cls, cfg: dict | None) -> RollingParams:
        cfg = dict(cfg or {})
        if "decel_range_mps2" in cfg:
            cfg["decel_range_mps2"] = tuple(cfg["decel_range_mps2"])
        return cls(**{k: v for k, v in cfg.items() if k in cls.__dataclass_fields__})


@dataclass
class StopPrediction:
    position: np.ndarray        # 정지 예측 위치 xyz [m] (base_link). 멈췄으면 지금 위치
    time_s: float               # 정지 예측 시각 (입력 시각과 같은 시계) [s]
    velocity: np.ndarray        # 지금 속도 xyz [m/s] (길 방향 성분만)
    speed_mps: float
    confidence: float           # 0~1
    is_moving: bool


class RollingBallPredictor:
    """공 하나의 굴림 추적·정지 예측. 시도(밀기 한 번)마다 reset(), 멈추면 finish_trial()."""

    def __init__(self, lane: LaneAxis, params: RollingParams | None = None, decel_mps2: float | None = None):
        self.lane = lane
        self.p = params or RollingParams()
        self.decel = self._clamp_decel(decel_mps2 if decel_mps2 is not None else self.p.decel_init_mps2)
        self.reset()

    # ---------- 시도 단위 ----------
    def reset(self) -> None:
        """새 시도 시작. 감속도(self.decel)는 유지한다."""
        self.x = np.zeros(2)                 # [s, v]
        self.P = np.eye(2)
        self.t: float | None = None
        self.d = 0.0
        self.z = 0.0
        self.s_rest = 0.0                    # 마지막으로 멈춰 있던 자리
        self.peak_speed = 0.0
        self.decel_frames = 0                # 최고 속도보다 확실히 느려진 연속 프레임 수
        self.slow_frames = 0
        self.s_recent: list[float] = []      # 최근 stop_frames 개 칼만 위치 (멈춤 판정)
        self.moving = False
        self.stopped = False
        self.history: list[tuple[float, float, float]] = []   # (t, 측정 s, 칼만 v)
        self.i_move: int | None = None       # history 에서 움직이기 시작한 인덱스

    def finish_trial(self) -> float | None:
        """멈춘 뒤 호출. 이번 시도 궤적으로 감속도를 다시 구해 갱신한다 (_fit_decel 참고).
        평균은 1/a 로 낸다 — 정지 거리가 1/a 에 비례하고, 한 시도 추정의 잡음이 1/a 쪽에서 대칭이라 덜 치우친다.
        반환: 이번 시도에서 맞춘 감속도 (데이터가 모자라거나 범위 밖이면 None, 기존 값 유지)."""
        fitted = self._fit_decel()
        if fitted is not None:
            a = self.p.decel_update_alpha
            self.decel = self._clamp_decel(1.0 / ((1.0 - a) / self.decel + a / fitted))
        return fitted

    # ---------- 프레임 단위 ----------
    def update(self, t: float, xyz: np.ndarray | None) -> StopPrediction | None:
        """t [s], xyz [m] base_link. 공을 못 본 프레임은 xyz=None (예측만). 첫 측정 전에는 None."""
        if xyz is None:
            if self.t is None:
                return None
            self._predict(t - self.t)
            self.t = t
            return self._result(t)

        xyz = np.asarray(xyz, dtype=float)
        s, d = self.lane.to_sd(xyz[:2])
        if self.t is None:
            self.x[:] = (s, 0.0)
            self.P = np.diag([self.p.meas_std_m ** 2, self.p.stop_speed_mps ** 2])   # 시도 시작 땐 공이 길 입구에 서 있다
            self.d, self.z = d, float(xyz[2])
            self.s_rest = s
        else:
            self._predict(t - self.t)
            self._correct(s)
            a = self.p.smooth_alpha
            self.d = (1 - a) * self.d + a * d
            self.z = (1 - a) * self.z + a * float(xyz[2])
        self.t = t
        self._update_motion_state()
        self.history.append((t, s, float(self.x[1])))
        return self._result(t)

    # ---------- 칼만 ----------
    def _predict(self, dt: float) -> None:
        if dt <= 0.0:
            return
        s, v = self.x
        moving = not self.stopped and abs(v) > self.p.stop_speed_mps
        sign = math.copysign(1.0, v)
        if moving and abs(v) < self.decel * dt:      # 이번 dt 안에 멈춤 → 속도가 0 을 넘어가지 않게
            s += sign * v * v / (2.0 * self.decel)
            v = 0.0
        else:
            u = -self.decel * sign if moving else 0.0
            s += v * dt + 0.5 * u * dt * dt
            v += u * dt
        self.x[:] = (s, v)
        F = np.array([[1.0, dt], [0.0, 1.0]])
        G = np.array([[0.5 * dt * dt], [dt]])
        self.P = F @ self.P @ F.T + (G @ G.T) * self.p.accel_std_mps2 ** 2

    def _correct(self, s_meas: float) -> None:
        H = np.array([1.0, 0.0])
        S = float(H @ self.P @ H) + self.p.meas_std_m ** 2
        K = self.P @ H / S
        self.x = self.x + K * (s_meas - self.x[0])
        self.P = (np.eye(2) - np.outer(K, H)) @ self.P

    # ---------- 상태 판정 ----------
    def _update_motion_state(self) -> None:
        s, speed = float(self.x[0]), abs(float(self.x[1]))
        if not self.moving:
            if abs(s - self.s_rest) > self.p.move_start_m and speed > 2.0 * self.p.stop_speed_mps:
                self.moving, self.stopped = True, False
                self.i_move = len(self.history)
                self.peak_speed, self.decel_frames, self.slow_frames = speed, 0, 0
            elif speed < self.p.stop_speed_mps:     # 가만히 있을 때만 정지 자리를 천천히 따라감
                a = self.p.smooth_alpha
                self.s_rest = (1 - a) * self.s_rest + a * s
            return
        if speed > self.peak_speed:
            self.peak_speed, self.decel_frames = speed, 0
        elif speed < self.peak_speed - 2.0 * math.sqrt(max(self.P[1, 1], 0.0)):
            self.decel_frames += 1
        else:                                          # 잡음으로 한 번 떨어진 건 무시 — 연속일 때만
            self.decel_frames = 0
        self.slow_frames = self.slow_frames + 1 if speed < self.p.stop_speed_mps else 0
        self.s_recent = (self.s_recent + [s])[-self.p.stop_frames:]
        # 잡음 1 cm 면 칼만 속도도 ±0.04 m/s 흔들려 속도만으로는 멈춤 판정이 늦다 → 위치가 안 변하는 것으로도 판정
        still = (len(self.s_recent) == self.p.stop_frames and max(self.s_recent) - min(self.s_recent) < self.p.meas_std_m
                 and speed < 2.0 * self.p.stop_speed_mps)
        if self.slow_frames >= self.p.stop_frames or still:
            self.moving, self.stopped = False, True
            self.s_rest = s
            self.x[1] = 0.0

    def _result(self, t: float) -> StopPrediction:
        s, v = float(self.x[0]), float(self.x[1])
        if not self.moving:
            pos = np.append(self.lane.to_xy(s, self.d), self.z)
            return StopPrediction(pos, t, np.zeros(3), 0.0, 1.0 if self.stopped else 0.0, False)
        a, speed = self.decel, abs(v)
        s_stop = s + v * speed / (2.0 * a)
        J = np.array([1.0, speed / a])               # ∂s_stop/∂[s, v]
        stop_std = math.sqrt(max(float(J @ self.P @ J), 0.0))
        conf_frames = min(1.0, self.decel_frames / max(self.p.min_frames, 1))
        conf_std = 1.0 / (1.0 + stop_std / self.p.stop_std_ref_m)
        pos = np.append(self.lane.to_xy(s_stop, self.d), self.z)
        vel = np.append(v * self.lane.dir_unit, 0.0)
        return StopPrediction(pos, t + speed / a, vel, speed, conf_frames * conf_std, True)

    # ---------- 마찰 추정 ----------
    def _fit_decel(self) -> float | None:
        """감속도 = v_r² / (2·굴러간 거리).
        1) 움직인 구간에 s(t) = c0 + c1·t − a/2·max(t − tk, 0)² 를 tk 후보마다 최소제곱으로 맞춰 손 떠난 시각 tk 와
           그때 속도 v_r = c1 (로봇이 미는 등속 구간 기울기라 잘 정해진다)을 얻는다. tk 가 정해지면 선형이라 전수 탐색이 싸다.
        2) 굴러간 거리 = 멈춘 뒤 측정 중앙값 − s(tk). 곡률(a 계수)을 직접 쓰면 잡음 1 cm 에 감속 구간 10 cm 로는
           ±50 % 넘게 흔들려서, 속도·거리로 다시 계산한다.
        그래도 한 시도 추정은 ±30~50 % 흔들린다(잡음 1 cm 가정) → finish_trial 이 여러 시도를 평균한다."""
        if len(self.history) < 8:
            return None
        t, s, v = (np.array(c) for c in zip(*self.history))
        speed = np.abs(v)
        i_peak = int(np.argmax(speed))
        if speed[i_peak] < 2.0 * self.p.stop_speed_mps:
            return None
        # 맞출 구간: 움직이기 시작한 프레임(정지 자리에서 move_start_m 벗어남, 이미 미는 중) ~ 다시 정지 속도 2배 아래.
        # 칼만 속도는 늦게 따라오므로 속도로 시작점을 잡으면 손을 떠난 뒤가 돼 버린다
        if self.i_move is None:
            return None
        i0 = self.i_move
        slow = np.nonzero(speed[i_peak:] < 2.0 * self.p.stop_speed_mps)[0]
        i1 = i_peak + int(slow[0]) if len(slow) else len(s)
        tt, ss = t[i0:i1] - t[i0], s[i0:i1]
        if len(tt) < 8 or tt[-1] < 0.3 or len(s) - i1 < 3:
            return None
        s_stop = float(np.median(s[i1:]))
        best: tuple[float, float, float] | None = None
        for k in range(1, len(tt) - 5):              # 감속 구간이 최소 5 프레임은 되게
            A = np.column_stack([np.ones_like(tt), tt, -0.5 * np.maximum(tt - tt[k], 0.0) ** 2])
            coef, res, *_ = np.linalg.lstsq(A, ss, rcond=None)
            sse = float(res[0]) if len(res) else float(np.sum((A @ coef - ss) ** 2))
            if best is None or sse < best[0]:
                best = (sse, float(coef[1]), float(coef[0] + coef[1] * tt[k]))
        _, v_r, s_r = best
        dist = s_stop - s_r
        if v_r * dist <= 0.0 or abs(dist) < 2.0 * self.p.meas_std_m:
            return None
        a = v_r * v_r / (2.0 * abs(dist))
        lo, hi = self.p.decel_range_mps2
        return a if lo <= a <= hi else None

    def _clamp_decel(self, a: float) -> float:
        lo, hi = self.p.decel_range_mps2
        return float(min(max(a, lo), hi))
