"""embeddings.py 단위 테스트 — DB 없이. 실제 embeddings.yaml·skills.yaml 을 읽는다."""
import math

import pytest

from rally_db import embeddings
from rally_db.embeddings import Feature

SPEC = embeddings.load_spec()


def row(mode=1, success=True, error_mm=50.0, duration_s=6.0, params=None, metrics=None, notes=""):
    return {"mode": mode, "success": success, "error_mm": error_mm, "duration_s": duration_s,
            "params": params or {}, "metrics": metrics or {}, "decision": {"notes": notes} if notes else None}


def dist(a, b):
    return math.dist(a, b)


def test_real_config_loads():
    assert SPEC.dim == 32
    assert set(SPEC.modes) == {1, 3, 5, 7}
    push = next(f for f in SPEC.modes[1] if f.name == "push_speed")
    assert (push.lo, push.hi) == (0.10, 0.60)          # skills.yaml mode1.push_speed_range


def test_vector_shape_and_scale():
    vec = embeddings.trial_vector(row(params={"push_speed": 0.20, "lane": 1}), SPEC)
    assert len(vec) == 32
    assert vec[:3] == pytest.approx([1.0, 0.25, 0.2])   # 성공, 50/200, 6/30
    assert vec[3] == pytest.approx(0.2)                  # (0.20 - 0.10) / 0.50 (#42 실측 범위)
    assert vec[4] == pytest.approx(0.5)                  # lane 1 / 2
    assert all(v == 0.0 for v in vec[6:])                # push_dir_deg 없음(0°→0)·남는 칸 0


def test_clipped_to_0_1():
    vec = embeddings.trial_vector(row(error_mm=900.0, duration_s=-1.0, params={"push_speed": 9.0}), SPEC)
    assert vec[1] == 1.0 and vec[2] == 0.0 and vec[3] == 1.0


def test_nan_becomes_zero():
    assert Feature("field", "x", 0, 1).scale(float("nan")) == 0.0


def test_skip_notes_and_unknown_mode():
    assert embeddings.trial_vector(row(mode=5, success=False, notes="no_hand"), SPEC) is None
    assert embeddings.trial_vector(row(mode=5, success=False, notes="interrupted"), SPEC) is None
    assert embeddings.trial_vector(row(mode=9), SPEC) is None
    assert embeddings.trial_vector(row(mode=5, success=False, notes="timeout"), SPEC) is not None


def test_mode5_uses_recorded_metrics():
    vec = embeddings.trial_vector(row(mode=5, error_mm=0.0, duration_s=10.0,
                                      metrics={"handover_time_s": 5.0, "wrist_angle_at_grasp_deg": 45.0,
                                               "attempts": 2.0}), SPEC)
    assert vec[3:7] == pytest.approx([0.5, 0.25, 0.25, 1.0])   # tilt 없음→0°→0.5, 5/20, 손목 45/180 (범위 0~180, #92), 2회


def test_similar_trial_is_closer():
    # 같은 속도·비슷한 오차 쪽이 더 가까워야 한다 (L2)
    base = embeddings.trial_vector(row(error_mm=40, params={"push_speed": 0.20, "lane": 1}), SPEC)
    near = embeddings.trial_vector(row(error_mm=45, params={"push_speed": 0.21, "lane": 1}), SPEC)
    far = embeddings.trial_vector(row(success=False, error_mm=180, params={"push_speed": 0.33, "lane": 2}), SPEC)
    assert dist(base, near) < dist(base, far)


def test_scale_matters_unlike_cosine():
    # 코사인이면 같다고 볼 두 시도(값이 모두 2배)를 L2 는 구분한다 — L2 를 고른 이유
    a = embeddings.trial_vector(row(success=False, error_mm=20, duration_s=3, params={"push_speed": 0.08}), SPEC)
    b = [2 * v for v in a]
    assert dist(a, b) > 0.1


@pytest.mark.parametrize("bad, msg", [
    ({"dim": 32, "common": [{"source": "field", "name": "x", "range": [1, 1]}]}, "min < max"),
    ({"dim": 32, "common": [{"source": "oops", "name": "x", "range": [0, 1]}]}, "source"),
    ({"dim": 32, "modes": {1: [{"source": "param", "name": "x", "range_ref": "mode1.nope"}]}}, "없음"),
    ({"dim": 1, "common": [{"source": "field", "name": "a", "range": [0, 1]}],
      "modes": {1: [{"source": "param", "name": "b", "range": [0, 1]}]}}, "dim"),
    ({"dim": 32, "common": [{"source": "field", "name": "a", "range": [0, 1]}],
      "modes": {1: [{"source": "param", "name": "a", "range": [0, 1]}]}}, "두 번"),
])
def test_bad_config_raises(bad, msg):
    with pytest.raises(ValueError, match=msg):
        embeddings.load_spec(bad, {"mode1": {"push_speed_range": [0.05, 0.35]}})


def test_backfill_uses_repo():
    class Repo:
        def __init__(self):
            self.saved = {}

        def trials_for_embedding(self, everything):
            return [dict(row(), id=1), dict(row(mode=5, notes="no_hand"), id=2)]

        def set_feature_vec(self, trial_id, vec):
            self.saved[trial_id] = vec

    repo = Repo()
    assert embeddings.backfill(repo, SPEC) == (1, 1)
    assert len(repo.saved[1]) == 32 and repo.saved[2] is None
