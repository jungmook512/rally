"""모드 3 정책 모드 (catch_policy) — 로봇·onnxruntime 없이. 실제 ONNX 는 파일·onnxruntime 이 있을 때만."""
import math
import os

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from rally_robot.skills.catch_plan import Prediction, plane_axes  # noqa: E402
from rally_robot.skills.catch_policy import (  # noqa: E402
    OnnxPolicy, decide_with_policy, find_policy, policy_obs, policy_target)

N = [0.697, -0.123, 0.706]
PAD = np.array([0.345, -0.332, 0.496])
CFG = dict(policy_hz=50.0, policy_start_s=0.10, policy_max_s=0.40)


def pred_at(t, du=0.05, dw=-0.03, std=(0.04, 0.06), t_cross_ros=100.7):
    _, u, w = plane_axes(N)
    return Prediction(point=PAD + du * u + dw * w, t_cross=t_cross_ros, since_release=t, std=std, n_obs=8)


def test_obs_matches_sim_layout():
    """시뮬 stage1_env_torch._obs_tensor 와 같은 순서·배율."""
    o = policy_obs(pred_at(0.2), PAD, N, now_ros=100.3)
    assert o.shape == (1, 10) and o.dtype == np.float32
    assert o[0, 0] == pytest.approx(0.05 * 5, abs=1e-5) and o[0, 1] == pytest.approx(-0.03 * 5, abs=1e-5)
    assert (o[0, 2], o[0, 3]) == (pytest.approx(0.235), pytest.approx(0.235))   # 기본: 학습 표 sim_std(0.2) = 2.35 cm
    o2 = policy_obs(pred_at(0.2), PAD, N, now_ros=100.3, std_source="landing_info")
    assert (o2[0, 2], o2[0, 3]) == (pytest.approx(0.4), pytest.approx(0.6))
    assert o[0, 4] == pytest.approx(0.2)                       # max(0.2·30, 1)/30
    assert o[0, 5] == pytest.approx(0.4, abs=1e-5)             # 100.7 − 100.3
    assert o[0, 6] == pytest.approx(0.2) and list(o[0, 7:]) == [0.0, 0.0, 0.0]
    assert policy_obs(pred_at(0.01), PAD, N, 100.0)[0, 4] == pytest.approx(1 / 30)


def test_target_adds_clipped_correction_in_pad_plane():
    _, u, w = plane_axes(N)
    p = pred_at(0.2).point
    t = policy_target(p, [1.0, 1.0, -4.0], N)                  # u +5 cm, w −20 cm → −10 cm 로 자름
    assert (t - p) @ u == pytest.approx(0.05) and (t - p) @ w == pytest.approx(-0.10)


class Seq:
    """since_release 가 0.02 s 씩 늘어나는 예측 흐름 + 가짜 시계."""
    def __init__(self, t0=0.06, stop_at=None):
        self.t, self.stop_at, self.ros = t0, stop_at, 100.0

    def latest(self):
        if self.stop_at is not None and self.t > self.stop_at:
            return None
        return pred_at(self.t)

    def sleep(self, s):
        self.t += s
        self.ros += s


def run(policy_fn, seq):
    return decide_with_policy(seq.latest, policy_fn, PAD, N, lambda: seq.ros, seq.sleep, lambda: None, CFG, pred_at(seq.t))


def test_go_when_first_positive_and_not_before_start():
    seen = []

    def pol(obs):
        seen.append(float(obs[0, 6]))
        return np.array([1.0 if obs[0, 6] >= 0.19 else -1.0, 0.2, 0.0])
    seq = Seq()
    pred, action, calls = run(pol, seq)
    assert action is not None and action[0] > 0 and pred.since_release == pytest.approx(0.20, abs=0.011)
    assert min(seen) >= 0.10                                    # policy_start_s 전엔 안 부름
    assert calls == len(seen)


def test_falls_back_when_never_goes_or_prediction_lost():
    pred, action, _ = run(lambda o: np.array([-1.0, 0.0, 0.0]), Seq())
    assert action is None and pred.since_release >= 0.40
    pred, action, _ = run(lambda o: np.array([-1.0, 0.0, 0.0]), Seq(stop_at=0.15))
    assert action is None and pred.since_release <= 0.16


class FakeSession:
    def __init__(self, shape=(1, 10), out=(0.5, 0.1, -0.2)):
        self._shape, self._out = shape, out

    def get_inputs(self):
        return [type("I", (), {"name": "obs", "shape": list(self._shape)})()]

    def run(self, _names, feed):
        assert feed["obs"].dtype == np.float32
        return [np.array([self._out], np.float32)]


def test_onnx_wrapper_checks_shapes_and_output():
    pol = OnnxPolicy("fake.onnx", session=FakeSession())
    assert pol(np.zeros((1, 10), np.float32)).tolist() == pytest.approx([0.5, 0.1, -0.2])
    with pytest.raises(ValueError):
        OnnxPolicy("fake.onnx", session=FakeSession(shape=(1, 7)))
    with pytest.raises(ValueError):
        OnnxPolicy("fake.onnx", session=FakeSession(out=(math.nan, 0, 0)))(np.zeros((1, 10), np.float32))


def test_find_policy_missing_returns_empty(tmp_path):
    assert find_policy(str(tmp_path / "nope.onnx")) == ""
    f = tmp_path / "p.onnx"
    f.write_bytes(b"x")
    assert find_policy(str(f)) == str(f)


def test_real_policy_if_available():
    """로봇 PC·박성현 PC 에 models/policy_stage1.onnx 와 onnxruntime 이 있으면 실제로 한 번 돌려 본다."""
    pytest.importorskip("onnxruntime")
    path = find_policy()
    if not path:
        pytest.skip("policy_stage1.onnx 없음")
    pol = OnnxPolicy(path)
    a = pol(policy_obs(pred_at(0.2), PAD, N, 100.3))
    assert a.shape == (3,) and np.all(np.isfinite(a))
