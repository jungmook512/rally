"""rally_vision/rolling_sim.py — predictor.py 검증용 합성 궤적 (모드 1 공 굴림). 실데이터(bag)가 생기기 전 확인용
담당: 정동휘

궤적: 정지 → 로봇이 밀어 v0 까지 가속 → v0 로 같이 감 → 손을 떠나 감속 a 로 굴러 정지 → 정지.
측정: fps 로 샘플, xyz 에 가우시안 잡음, 일정 확률로 미검출(None), 시각 흔들림.

    python3 -m rally_vision.rolling_sim          # 조건별 예측 오차 표
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rally_vision.predictor import LaneAxis, RollingBallPredictor, RollingParams


@dataclass
class RollScenario:
    v0_mps: float = 0.25
    decel_true_mps2: float = 0.30
    fps: float = 30.0
    noise_std_m: float = 0.01
    dropout: float = 0.0
    jitter_s: float = 0.002
    lateral_m: float = 0.01        # 길 안에서 옆으로 치우친 정도
    z_m: float = 0.046             # 공 중심 높이 (테이블 14 mm + 반지름 32 mm)
    idle_s: float = 0.3
    ramp_s: float = 0.1
    push_s: float = 0.15
    tail_s: float = 0.6
    s0_m: float = 0.05

    def release_s(self) -> float:
        return self.idle_s + self.ramp_s + self.push_s

    def stop_s(self) -> float:
        return self.release_s() + self.v0_mps / self.decel_true_mps2

    def s_at(self, t: float) -> float:
        v0, a = self.v0_mps, self.decel_true_mps2
        t1 = self.idle_s
        t2 = t1 + self.ramp_s
        t3 = t2 + self.push_s
        if t < t1:
            return self.s0_m
        if t < t2:
            return self.s0_m + 0.5 * v0 / self.ramp_s * (t - t1) ** 2
        s2 = self.s0_m + 0.5 * v0 * self.ramp_s
        if t < t3:
            return s2 + v0 * (t - t2)
        s3 = s2 + v0 * self.push_s
        tr = min(t - t3, v0 / a)
        return s3 + v0 * tr - 0.5 * a * tr * tr

    def stop_s_m(self) -> float:
        return self.s_at(self.stop_s() + 1.0)


def generate(sc: RollScenario, lane: LaneAxis, rng: np.random.Generator) -> list[tuple[float, np.ndarray | None]]:
    """(측정 시각, xyz 또는 None) 목록."""
    out: list[tuple[float, np.ndarray | None]] = []
    t_end = sc.stop_s() + sc.tail_s
    for k in range(int(t_end * sc.fps) + 1):
        t_true = k / sc.fps
        xy = lane.to_xy(sc.s_at(t_true), sc.lateral_m)
        t_meas = t_true + rng.normal(0.0, sc.jitter_s)
        if rng.random() < sc.dropout:
            out.append((t_meas, None))
            continue
        out.append((t_meas, np.append(xy, sc.z_m) + rng.normal(0.0, sc.noise_std_m, 3)))
    return out


@dataclass
class RollEval:
    err_mid_mm: float          # 굴러간 거리 절반 지점에서 예측 오차
    err_conf_mm: float         # confidence ≥ 0.5 가 처음 된 프레임의 예측 오차 (짧게 구르면 멈춤 판정 프레임일 수 있음)
    lead_conf_s: float         # 그때 실제 정지까지 남은 시간
    err_final_mm: float        # is_moving=False 판정 시 위치 오차
    stop_time_err_s: float     # 절반 지점에서 정지 시각 예측 오차
    decel_fit: float | None


def run(sc: RollScenario, lane: LaneAxis, pred: RollingBallPredictor, rng: np.random.Generator) -> RollEval:
    pred.reset()
    true_stop = np.append(lane.to_xy(sc.stop_s_m(), sc.lateral_m), sc.z_m)
    t_mid = sc.release_s() + (sc.v0_mps / sc.decel_true_mps2) * (1 - 1 / np.sqrt(2))   # 남은 거리가 절반이 되는 시각
    nan = float("nan")
    err_mid = err_conf = lead = err_final = t_err = nan
    for t, xyz in generate(sc, lane, rng):
        r = pred.update(t, xyz)
        if r is None:
            continue
        err = float(np.linalg.norm(r.position[:2] - true_stop[:2])) * 1000
        if r.is_moving and np.isnan(err_mid) and t >= t_mid:
            err_mid, t_err = err, r.time_s - sc.stop_s()
        if np.isnan(err_conf) and r.confidence >= 0.5:
            err_conf, lead = err, sc.stop_s() - t
        if pred.stopped and np.isnan(err_final):
            err_final = err
    return RollEval(err_mid, err_conf, lead, err_final, t_err, pred.finish_trial())


def _table() -> None:
    lane = LaneAxis.from_cfg({"start": [0.45, 0.0, 0.02], "dir_deg": 90})
    rows = [
        ("기본 v0=0.25 a=0.30 30fps σ1cm", RollScenario(), RollingParams()),
        ("느림 v0=0.15", RollScenario(v0_mps=0.15), RollingParams()),
        ("빠름 v0=0.35", RollScenario(v0_mps=0.35), RollingParams()),
        ("20fps + 미검출 20%", RollScenario(fps=20, dropout=0.2), RollingParams()),
        ("잡음 σ2cm (meas_std 1cm 그대로)", RollScenario(noise_std_m=0.02), RollingParams()),
        ("잡음 σ2cm (meas_std 2cm 로 맞춤)", RollScenario(noise_std_m=0.02), RollingParams(meas_std_m=0.02)),
        ("감속 초기값 틀림 (참 0.45, 시작 0.30)", RollScenario(decel_true_mps2=0.45), RollingParams()),
        ("감속 초기값 틀림 (참 0.20, 시작 0.30)", RollScenario(decel_true_mps2=0.20), RollingParams()),
    ]
    print(f"{'조건':40s} {'절반지점':>8s} {'conf≥.5':>8s} {'선행s':>6s} {'멈춤판정':>8s} {'정지시각s':>9s}  (오차 mm, 시드 30개 평균/최대)")
    for name, sc, params in rows:
        evs = [run(sc, lane, RollingBallPredictor(lane, params, 0.30), np.random.default_rng(i)) for i in range(30)]
        f = lambda xs: f"{np.nanmean(xs):4.0f}/{np.nanmax(xs):3.0f}"   # noqa: E731
        print(f"{name:40s} {f([e.err_mid_mm for e in evs]):>8s} {f([e.err_conf_mm for e in evs]):>8s} "
              f"{np.nanmean([e.lead_conf_s for e in evs]):6.2f} {f([e.err_final_mm for e in evs]):>8s} "
              f"{np.nanmean([abs(e.stop_time_err_s) for e in evs]):9.2f}")
    # 감속도 온라인 갱신: 참 0.45 인데 0.30 에서 시작해 시도를 반복
    sc = RollScenario(decel_true_mps2=0.45)
    pred = RollingBallPredictor(lane, RollingParams(), 0.30)
    print("\n감속도 온라인 갱신 (참 0.45, 시작 0.30): 시도별 절반지점 오차 mm / 갱신 후 감속도")
    for i in range(12):
        e = run(sc, lane, pred, np.random.default_rng(100 + i))
        print(f"  시도 {i + 1}: {e.err_mid_mm:5.1f} mm  → decel {pred.decel:.3f} (이번 맞춤 {e.decel_fit})")


if __name__ == "__main__":
    _table()
