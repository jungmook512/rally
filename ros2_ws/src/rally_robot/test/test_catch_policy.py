"""모드 3 정책 모드 (catch_policy) — 로봇·onnxruntime 없이. 실제 ONNX 는 파일·onnxruntime 이 있을 때만."""
import math
import os

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from rally_robot.skills.catch_plan import Prediction, plane_axes  # noqa: E402
from rally_robot.skills.catch_policy import (  # noqa: E402
    OnnxPolicy, decide_with_policy, find_policy, meta_mismatches, policy_meta, policy_obs, policy_target)

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


def test_runtime_policy_error_falls_back_with_log():
    """정책 실행 중 예외(NaN 출력·onnxruntime 오류)는 투구를 버리지 않고 (예측, None) — catch_ball 이 규칙으로 (#175 리뷰 1)."""
    errors = []

    def bad(obs):
        raise ValueError("정책 출력 이상 [nan nan nan]")
    seq = Seq()
    pred, action, calls = decide_with_policy(seq.latest, bad, PAD, N, lambda: seq.ros, seq.sleep, lambda: None, CFG, pred_at(seq.t),
                                             on_error=errors.append)
    assert action is None and calls == 1 and len(errors) == 1 and isinstance(errors[0], ValueError)
    assert 0.10 <= pred.since_release <= 0.12                   # 첫 호출에서 바로 폴백 (policy_max_s 까지 안 기다림)
    pred, action, _ = run(bad, Seq())                           # on_error 없어도 예외가 밖으로 안 나감
    assert action is None


def test_tick_abort_still_propagates():
    """check_stop(tick) 의 중단은 폴백으로 삼키지 않는다."""
    class Abort(Exception):
        pass

    def tick():
        raise Abort()
    seq = Seq()
    with pytest.raises(Abort):
        decide_with_policy(seq.latest, lambda o: np.array([1.0, 0.0, 0.0]), PAD, N, lambda: seq.ros, seq.sleep, tick, CFG, pred_at(seq.t),
                           on_error=lambda e: None)


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


def test_policy_meta_reads_sidecar_and_tolerates_missing(tmp_path):
    """ONNX 옆 <이름>.onnx.json — 없으면 None, 깨졌어도 None (정책 적재를 막지 않는다)."""
    f = tmp_path / "p.onnx"
    f.write_bytes(b"x")
    assert policy_meta(str(f)) is None
    (tmp_path / "p.onnx.json").write_text(
        '{"run": "stage1_x", "checkpoint": "model_300.pt", "env": {"accel": "movej", "limits": [0.0, 0.35, -0.35, 0.08], "reach_max": 0.35, "overshoot": 0.0}}',
        encoding="utf-8")
    meta = policy_meta(str(f))
    assert meta["run"] == "stage1_x" and meta["env"]["accel"] == "movej"
    (tmp_path / "p.onnx.json").write_text("{broken", encoding="utf-8")
    assert policy_meta(str(f)) is None


def test_meta_mismatches_lists_only_differences():
    """학습 조건(10/14: movej·35/35/8·두 축 합 35·지나침 0)과 yaml 이 같으면 비고 없음, 다르면 항목별 한 줄."""
    meta = {"env": {"accel": "movej", "limits": [0.0, 0.35, -0.35, 0.08], "reach_max": 0.35, "overshoot": 0.0}}
    same = dict(right_max_m=0.35, updown_max_m=0.35, down_max_m=0.08, reach_max_m=0.35, catch_move="amovej", catch_overshoot_m=0.0)
    assert meta_mismatches(meta, same) == []
    assert meta_mismatches(None, same) == [] and meta_mismatches({}, same) == []
    msgs = meta_mismatches(meta, dict(same, right_max_m=0.15, updown_max_m=0.15, catch_move="amovel", catch_overshoot_m=0.05))
    assert len(msgs) == 4
    assert any(m.startswith("right_max_m") for m in msgs) and any(m.startswith("updown_max_m") for m in msgs)
    assert any("catch_move 학습 amovej vs yaml amovel" in m for m in msgs) and any(m.startswith("catch_overshoot_m") for m in msgs)
    assert meta_mismatches(meta, dict(same, reach_max_m=0.30)) == ["reach_max_m 학습 0.35 vs yaml 0.30"]


def test_real_policy_if_available():
    """로봇 PC·박성현 PC 에 models/policy_stage1.onnx 와 onnxruntime 이 있으면 실제로 한 번 돌려 본다."""
    pytest.importorskip("onnxruntime")
    path = find_policy()
    if not path:
        pytest.skip("policy_stage1.onnx 없음")
    pol = OnnxPolicy(path)
    a = pol(policy_obs(pred_at(0.2), PAD, N, 100.3))
    assert a.shape == (3,) and np.all(np.isfinite(a))


def test_meta_mismatches_tolerates_non_numeric_json():
    """#212 리뷰: 손으로 만든 json 에 null·문자열이 있어도 예외 없이 대조만 건너뛴다."""
    meta = {"env": {"accel": "movej", "limits": [0.0, "abc", None, 0.08], "reach_max": None, "overshoot": "x"}}
    cfg = dict(right_max_m=0.35, updown_max_m=0.35, down_max_m=0.08, reach_max_m=0.35, catch_move="amovej", catch_overshoot_m=0.05)
    assert meta_mismatches(meta, cfg) == []
    assert meta_mismatches({"env": {"limits": [0.0, 0.15, -0.35, 0.08]}}, cfg) == ["right_max_m 학습 0.15 vs yaml 0.35"]
