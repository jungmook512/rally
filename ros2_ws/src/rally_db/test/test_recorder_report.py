"""세션 리포트 → sessions.report_md 저장 (가짜 저장소, DB 없이)."""
import json

import pytest
import rclpy
from std_msgs.msg import String

from rally_db.recorder_node import Recorder
from rally_db.repository import StorageUnavailable

SID = "11111111-2222-3333-4444-555555555555"


class FakeRepo:
    database_url = "postgresql://user:pw@db/rally"
    redis_url = "redis://redis"

    def __init__(self):
        self.calls, self.down = [], False

    def _check(self):
        if self.down:
            raise StorageUnavailable("offline")

    def ensure_session(self, sid):
        self._check()
        self.calls.append(("ensure", sid))

    def end_session(self, sid, md):
        self._check()
        self.calls.append(("end", sid, md))

    def publish_event(self, kind, payload, session_id=None):
        self.calls.append(("event", kind, session_id))

    def __getattr__(self, name):          # 그 밖의 저장 함수는 아무것도 안 함
        return lambda *a, **k: None


@pytest.fixture
def recorder():
    rclpy.init()
    repo = FakeRepo()
    node = Recorder(repo=repo)
    yield node, repo
    node.destroy_node()
    rclpy.shutdown()


def report(sid=SID, md="## 세션 요약"):
    return String(data=json.dumps({"session_id": sid, "report_md": md, "source": "numeric:LLM 키 없음"}, ensure_ascii=False))


def test_report_saved_and_announced(recorder):
    node, repo = recorder
    node._on_report(report())
    node._flush()
    assert ("ensure", SID) in repo.calls and ("end", SID, "## 세션 요약") in repo.calls
    assert ("event", "session_report", SID) in repo.calls


def test_report_retried_when_db_down(recorder):
    node, repo = recorder
    repo.down = True
    node._on_report(report())
    node._flush()
    assert not [c for c in repo.calls if c[0] == "end"]
    repo.down = False
    node._flush()
    assert ("end", SID, "## 세션 요약") in repo.calls


@pytest.mark.parametrize("raw", ["not json", json.dumps({"report_md": "x"}), json.dumps({"session_id": "", "report_md": "x"})])
def test_bad_report_dropped(recorder, raw):
    node, repo = recorder
    node._on_report(String(data=raw))
    node._flush()
    assert not [c for c in repo.calls if c[0] in ("ensure", "end")]
