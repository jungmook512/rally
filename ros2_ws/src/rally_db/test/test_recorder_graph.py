"""시도 + 관찰된 보상 동작 → 지식 그래프 관계 (가짜 저장소, DB 없이)."""
import pytest
import rclpy

from rally_db.recorder_node import Recorder
from rally_db.repository import trial_graph
from rally_interfaces.msg import SceneContext, TrialResult

SID = "11111111-2222-3333-4444-555555555555"


class FakeRepo:
    database_url = "postgresql://user:pw@db/rally"
    redis_url = "redis://redis"

    def __init__(self):
        self.trials = []

    def add_trial(self, row, observed=()):
        self.trials.append((row["trial_index"], tuple(observed)))
        return len(self.trials)

    def __getattr__(self, name):
        return lambda *a, **k: None


@pytest.fixture
def recorder():
    rclpy.init()
    repo = FakeRepo()
    node = Recorder(repo=repo)
    yield node, repo
    node.destroy_node()
    rclpy.shutdown()


def ctx(comp: str = "", sid: str = SID) -> SceneContext:
    msg = SceneContext(session_id=sid, mode=1)
    msg.body.compensation_detected = bool(comp)
    msg.body.compensation_type = comp or "none"
    return msg


def trial(idx: int, sid: str = SID) -> TrialResult:
    return TrialResult(session_id=sid, mode=1, trial_index=idx, success=True, duration_s=4.0)


def test_compensation_seen_during_trial_goes_with_it(recorder):
    node, repo = recorder
    for m in (ctx(), ctx("trunk_lean"), ctx("trunk_lean"), ctx("shoulder_hike"), ctx("bogus")):
        node._on_context(m)
    node._on_trial(trial(1))
    node._on_context(ctx())
    node._on_trial(trial(2))                       # 그 사이 보상 동작 없음 → 비어 있음
    node._flush()
    assert repo.trials == [(1, ("shoulder_hike", "trunk_lean")), (2, ())]


def test_new_session_forgets_seen(recorder):
    node, repo = recorder
    node._on_context(ctx("trunk_lean"))
    other = "22222222-2222-3333-4444-555555555555"
    node._on_context(ctx("", sid=other))
    node._on_trial(trial(1, sid=other))
    node._flush()
    assert repo.trials == [(1, ())]


def test_trial_graph_edges():
    nodes, edges = trial_graph({"session_id": SID, "mode": 5}, 42, ["trunk_lean", "trunk_lean", "x"])
    assert ("trial:42", "Trial", None, None) in nodes and ("rally:mode5", "Mode", None, None) in nodes
    assert edges == [(f"session:{SID}", "hasTrial", "trial:42", SID), ("trial:42", "inMode", "rally:mode5", SID),
                     ("trial:42", "observed", "rally:trunk_lean", SID)]
    assert not any(n[1] == "Patient" for n in nodes)
