"""predictor.py 합성 궤적 테스트 (로봇·카메라 없이). 수치 근거: python3 -m rally_vision.rolling_sim"""
import numpy as np
import pytest

from rally_vision.predictor import LaneAxis, RollingBallPredictor, RollingParams
from rally_vision.rolling_sim import RollScenario, generate, run

LANE = LaneAxis.from_cfg({"start": [0.45, 0.0, 0.02], "dir_deg": 90})


def _evals(sc: RollScenario, n: int = 20, decel0: float = 0.30):
    return [run(sc, LANE, RollingBallPredictor(LANE, RollingParams(), decel0), np.random.default_rng(i)) for i in range(n)]


def test_lane_roundtrip():
    lane = LaneAxis.from_cfg({"start": [0.45, -0.20, 0.02], "dir_deg": 100})
    s, d = lane.to_sd(lane.to_xy(0.12, -0.01))
    assert s == pytest.approx(0.12) and d == pytest.approx(-0.01)


def test_params_from_cfg_ignores_unknown_keys():
    p = RollingParams.from_cfg({"decel_init_mps2": 0.4, "decel_range_mps2": [0.1, 1.0], "unknown": 1})
    assert p.decel_init_mps2 == 0.4 and p.decel_range_mps2 == (0.1, 1.0)


def test_stationary_ball_is_not_moving():
    rng = np.random.default_rng(0)
    pred = RollingBallPredictor(LANE)
    xyz0 = np.append(LANE.to_xy(0.05, 0.0), 0.046)
    for k in range(90):
        r = pred.update(k / 30, xyz0 + rng.normal(0, 0.01, 3))
        assert not r.is_moving


def test_missing_frames_predict_only():
    pred = RollingBallPredictor(LANE)
    assert pred.update(0.0, None) is None
    pred.update(0.033, np.array([0.45, 0.05, 0.046]))
    assert pred.update(0.066, None) is not None


@pytest.mark.parametrize("sc", [RollScenario(), RollScenario(v0_mps=0.15), RollScenario(v0_mps=0.35),
                                RollScenario(fps=20, dropout=0.2)])
def test_confident_prediction_within_30mm(sc):
    # FR-03: 0.3 m/s 이하에서 예측–실제 ≤ 30 mm. confidence ≥ 0.5 가 처음 된 프레임 기준, 시드 평균
    errs = [e.err_conf_mm for e in _evals(sc)]
    assert np.nanmean(errs) <= 30


def test_stop_detected_near_true_position():
    evs = _evals(RollScenario())
    assert all(not np.isnan(e.err_final_mm) for e in evs)
    assert np.mean([e.err_final_mm for e in evs]) <= 15


def test_decel_online_update_moves_toward_truth():
    sc = RollScenario(decel_true_mps2=0.45)
    pred = RollingBallPredictor(LANE, RollingParams(), 0.30)
    for i in range(12):
        run(sc, LANE, pred, np.random.default_rng(100 + i))
    assert 0.33 < pred.decel < 0.60


def test_generate_has_dropouts():
    out = generate(RollScenario(dropout=0.5), LANE, np.random.default_rng(0))
    assert any(xyz is None for _, xyz in out)
