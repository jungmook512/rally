"""HTTP 요청이 brain 승인 없이 성공하지 않는지 확인. 실제 Redis 없이 실행."""
import asyncio
import importlib.util
import json
import sys
from pathlib import Path
import time
import uuid

import httpx
import pytest
from redis.exceptions import ConnectionError

spec = importlib.util.spec_from_file_location("dashboard_main", Path(__file__).parents[1] / "app" / "main.py")
web = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = web
spec.loader.exec_module(web)


class FakeRedis:
    def __init__(self, state, reply=True, subscribers=1, down=False):
        self.state, self.reply, self.subscribers, self.down = state, reply, subscribers, down
        self.command = None

    async def get(self, key):
        if self.down:
            raise ConnectionError("offline")
        if key == web.CONTROL_KEY:
            return json.dumps(self.state) if self.state else None
        if self.command is not None and self.reply is not None:
            return json.dumps({"request_id": self.command["request_id"], "success": self.reply,
                               "message": "승인" if self.reply else "안전 상태 변경 — 거절"})
        return None

    async def publish(self, channel, raw):
        self.command = json.loads(raw)
        return self.subscribers


def snapshot():
    return {"session_id": str(uuid.uuid4()), "pause_id": str(uuid.uuid4()), "phase": "paused",
            "can_resume": True, "blocked_reason": "", "updated_at": time.time()}


def request(monkeypatch, store, body):
    monkeypatch.setattr(web, "_redis", store)

    async def call():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=web.app), base_url="http://test") as client:
            return await client.post("/api/session/resume", json=body)
    return asyncio.run(call())


def test_resume_waits_for_matching_brain_reply(monkeypatch):
    state = snapshot()
    store = FakeRedis(state)
    r = request(monkeypatch, store, {k: state[k] for k in ("session_id", "pause_id")})
    assert r.status_code == 200 and r.json()["success"]
    assert store.command["command"] == "resume"
    assert store.command["pause_id"] == state["pause_id"]
    assert 0 < store.command["expires_at"] - time.time() <= 2


@pytest.mark.parametrize("case, expected", [("old_pause", 409), ("stale", 503), ("blocked", 409),
                                           ("no_listener", 503), ("down", 503), ("brain_reject", 409)])
def test_resume_failures_are_visible(monkeypatch, case, expected):
    state = snapshot()
    body = {k: state[k] for k in ("session_id", "pause_id")}
    if case == "old_pause":
        body["pause_id"] = str(uuid.uuid4())
    if case == "stale":
        state["updated_at"] -= 10
    if case == "blocked":
        state.update(can_resume=False, blocked_reason="준비 안 됨")
    store = FakeRedis(state, reply=case != "brain_reject", subscribers=0 if case == "no_listener" else 1,
                      down=case == "down")
    r = request(monkeypatch, store, body)
    assert r.status_code == expected and r.json()["detail"]
    if case in ("old_pause", "stale", "blocked", "down"):
        assert store.command is None


def test_no_reply_does_not_claim_success(monkeypatch):
    state = snapshot()
    store = FakeRedis(state, reply=None)
    r = request(monkeypatch, store, {k: state[k] for k in ("session_id", "pause_id")})
    assert r.status_code == 504


def test_invalid_request_never_published(monkeypatch):
    store = FakeRedis(snapshot())
    r = request(monkeypatch, store, {"session_id": "wrong", "pause_id": "wrong"})
    assert r.status_code == 422 and store.command is None


@pytest.mark.parametrize('case,code', [('accept',200),('reject',200),('old',409),('expired',409),
                                     ('stale',503),('brain_reject',409),('no_reply',504),('invalid',422)])
def test_proposal_response_checks_brain_and_current_proposal(monkeypatch, case, code):
    state = snapshot()
    state.update(phase='running',proposal={'proposal_id':str(uuid.uuid4()),'status':'pending','remaining_s':20})
    body = {'session_id':state['session_id'],'proposal_id':state['proposal']['proposal_id'],
            'decision':'reject' if case=='reject' else 'accept'}
    if case == 'old': body['proposal_id'] = str(uuid.uuid4())
    if case == 'expired': state['proposal']['status'] = 'expired'
    if case == 'stale': state['updated_at'] -= 10
    if case == 'invalid': body['decision'] = 'anything'
    store = FakeRedis(state,reply=None if case=='no_reply' else case!='brain_reject')
    monkeypatch.setattr(web,'_redis',store)
    async def call():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=web.app),base_url='http://test') as client:
            return await client.post('/api/session/proposal/respond',json=body)
    r = asyncio.run(call())
    assert r.status_code == code
    if case in ('old','expired','stale','invalid'): assert store.command is None
    if code == 200: assert store.command['command'] == 'proposal_'+body['decision']


def test_training_illustrations_are_served_without_control_requests(monkeypatch):
    store = FakeRedis(None, down=True)
    monkeypatch.setattr(web, "_redis", store)

    async def call():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=web.app), base_url="http://test") as client:
            response = await client.get("/images/training-scenes.js")
            assert response.status_code == 200
            assert "javascript" in response.headers["content-type"]
            assert "function renderTrainingScene" in response.text
            assert (await client.get("/images/missing.svg")).status_code == 404
            assert (await client.get("/images/%2e%2e/index.html")).status_code == 404
    asyncio.run(call())
    assert store.command is None


def test_latest_report(monkeypatch):
    from datetime import datetime, timezone
    seen = []

    async def fake_query(sql, params=()):
        seen.append(sql)
        return [{"session_id": "11111111-2222-3333-4444-555555555555", "report_md": "## 세션 요약\n| 1 |",
                 "started_at": datetime(2026, 10, 11, 1, 0, tzinfo=timezone.utc),
                 "ended_at": datetime(2026, 10, 11, 1, 12, tzinfo=timezone.utc)}]

    async def empty(sql, params=()):
        return []

    async def call():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=web.app), base_url="http://test") as client:
            return await client.get("/api/report/latest")

    monkeypatch.setattr(web, "_query", fake_query)
    r = asyncio.run(call())
    assert r.status_code == 200 and r.json()["report_md"].startswith("## 세션 요약")
    assert r.json()["ended_at"].startswith("2026-10-11T01:12") and "report_md IS NOT NULL" in seen[0]
    monkeypatch.setattr(web, "_query", empty)
    assert asyncio.run(call()).json() is None
