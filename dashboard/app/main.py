"""RALLY 대시보드 — FastAPI + WebSocket (Redis 실시간 · PostgreSQL 기록)
담당: 조정묵

  GET  /              화면 (web/index.html)
  GET  /api/state     현재 상태 — Redis rally:state (recorder 가 /rally/brain/context 로 0.5 s 마다 덮어씀)
  GET  /api/trials    최근 세션의 시도 목록 — DB trials
  GET  /api/events    최근 사건 — DB events (안전·음성·모드 바뀜)
  WS   /ws            {"type": "state", "data": …} 0.5 s 마다 바뀌었을 때 / {"type": "event", "data": …} rally:events 그대로
  GET /api/session · POST /api/session/resume — Redis를 통해 brain 상태 조회·재개 승인 요청
대시보드는 ROS 를 쓰지 않는다. 휴식 재개·휴식 제안 응답을 Redis로 연결하며 안전 reset과 구분한다.
Redis·DB 가 죽어 있어도 서버는 뜨고, 해당 API 는 503 + 이유를 돌려준다. 화면은 다시 연결될 때까지 재시도한다.
"""
from __future__ import annotations

import asyncio
import json
import os
import time
import uuid
from contextlib import suppress
from pathlib import Path
from typing import Any, Literal

import psycopg
import redis.asyncio as aioredis
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from psycopg.rows import dict_row
from pydantic import BaseModel

DATABASE_URL = os.environ.get("DATABASE_URL", "postgresql://rally:rally@127.0.0.1:5432/rally")
REDIS_URL = os.environ.get("REDIS_URL", "redis://127.0.0.1:6379/0")
STATE_KEY = "rally:state"            # rally_db/repository.py 와 같은 이름
EVENTS_CHANNEL = "rally:events"
CONTROL_KEY = "rally:session_control"
COMMAND_CHANNEL = "rally:session_commands"
REPLY_PREFIX = "rally:session_reply:"
STATE_PERIOD_S = 0.5
WEB_DIR = Path(__file__).resolve().parents[1] / "web"

app = FastAPI(title="RALLY dashboard")
app.mount("/images", StaticFiles(directory=WEB_DIR / "images"), name="images")
_redis = aioredis.from_url(REDIS_URL, decode_responses=True, socket_connect_timeout=0.5, socket_timeout=0.5)


async def _query(sql: str, params: tuple = ()) -> list[dict[str, Any]]:
    try:
        async with await psycopg.AsyncConnection.connect(DATABASE_URL, connect_timeout=2, row_factory=dict_row) as conn:
            cur = await conn.execute(sql, params)
            return await cur.fetchall()
    except psycopg.OperationalError as e:
        raise HTTPException(503, f"DB 연결 안 됨: {e}") from e


async def _get_state() -> dict[str, Any] | None:
    raw = await _redis.get(STATE_KEY)
    return json.loads(raw) if raw else None


async def _get_control() -> dict[str, Any] | None:
    raw = await _redis.get(CONTROL_KEY)
    state = json.loads(raw) if raw else None
    return state if state and 0 <= time.time() - state.get("updated_at", 0) < 2 else None


class ResumeRequest(BaseModel):
    session_id: uuid.UUID
    pause_id: uuid.UUID


@app.get("/api/session")
async def session_state() -> dict[str, Any] | None:
    try:
        return await _get_control()
    except aioredis.RedisError as e:
        raise HTTPException(503, "세션 상태 연결이 끊겼습니다") from e


@app.post("/api/session/resume")
async def resume_session(body: ResumeRequest) -> dict[str, Any]:
    request_id = str(uuid.uuid4())
    try:
        state = await _get_control()
        if not state:
            raise HTTPException(503, "brain의 최신 상태가 없습니다")
        if (state["phase"] != "paused" or state["session_id"] != str(body.session_id)
                or state["pause_id"] != str(body.pause_id)):
            raise HTTPException(409, "화면의 휴식 상태가 바뀌었습니다. 새 상태를 확인해 주세요")
        if not state["can_resume"]:
            raise HTTPException(409, state["blocked_reason"] or "현재 재개할 수 없습니다")
        command = {"request_id": request_id, "session_id": str(body.session_id),
                   "pause_id": str(body.pause_id), "command": "resume", "expires_at": time.time() + 2}
        return await send_command(command)
    except aioredis.RedisError as e:
        raise HTTPException(503, "제어 연결이 끊겼습니다. 상태를 다시 확인해 주세요") from e


class ProposalResponse(BaseModel):
    session_id: uuid.UUID
    proposal_id: uuid.UUID
    decision: Literal["accept", "reject"]


@app.post("/api/session/proposal/respond")
async def respond_proposal(body: ProposalResponse) -> dict[str, Any]:
    try:
        state = await _get_control()
        if not state:
            raise HTTPException(503, "brain의 최신 상태가 없습니다")
        proposal = state.get("proposal") or {}
        if (state["session_id"] != str(body.session_id) or state["phase"] != "running"
                or proposal.get("proposal_id") != str(body.proposal_id)
                or proposal.get("status") != "pending" or proposal.get("remaining_s", 0) <= 0):
            raise HTTPException(409, "제안이 만료·종료됐거나 상태가 바뀌었습니다")
        return await send_command({"request_id": str(uuid.uuid4()), "session_id": str(body.session_id),
            "proposal_id": str(body.proposal_id), "command": "proposal_" + body.decision,
            "expires_at": time.time() + 2})
    except aioredis.RedisError as e:
        raise HTTPException(503, "제어 연결이 끊겼습니다. 상태를 다시 확인해 주세요") from e


async def send_command(command: dict) -> dict[str, Any]:
    request_id = command["request_id"]
    try:
        if not await _redis.publish(COMMAND_CHANNEL, json.dumps(command)):
            raise HTTPException(503, "brain 제어 연결이 없습니다")
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline:
            raw = await _redis.get(REPLY_PREFIX + request_id)
            if raw:
                reply = json.loads(raw)
                if not reply["success"]:
                    raise HTTPException(409, reply["message"])
                return reply
            await asyncio.sleep(0.05)
    except aioredis.RedisError as e:
        raise HTTPException(503, "결과를 확인할 수 없습니다. 상태 갱신 후 확인해 주세요") from e
    raise HTTPException(504, "응답 미확인 — 자동 재시도하지 않고 상태를 확인해 주세요")


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(WEB_DIR / "index.html")


@app.get("/health")
async def health() -> dict[str, Any]:
    status = {"ok": True, "redis": False, "db": False}
    with suppress(Exception):
        status["redis"] = bool(await _redis.ping())
    with suppress(Exception):
        await _query("SELECT 1")
        status["db"] = True
    return status


@app.get("/api/state")
async def state() -> dict[str, Any] | None:
    try:
        return await _get_state()
    except aioredis.RedisError as e:
        raise HTTPException(503, f"Redis 연결 안 됨: {e}") from e


@app.get("/api/trials")
async def trials(limit: int = 30) -> dict[str, Any]:
    """가장 최근 시도가 있는 세션의 시도들 (최신 순)."""
    rows = await _query(
        """WITH latest AS (SELECT session_id FROM trials ORDER BY id DESC LIMIT 1)
           SELECT id AS trial_id, session_id::text, mode, trial_index, success, error_mm, duration_s, metrics,
                  COALESCE(decision->>'notes', '') AS notes, started_at + make_interval(secs => duration_s) AS ended_at
           FROM trials WHERE session_id = (SELECT session_id FROM latest)
           ORDER BY id DESC LIMIT %s""", (min(limit, 200),))
    for r in rows:
        r["ended_at"] = r["ended_at"].isoformat() if r["ended_at"] else None
    return {"session_id": rows[0]["session_id"] if rows else None, "trials": rows}


@app.get("/api/events")
async def events(limit: int = 30) -> list[dict[str, Any]]:
    rows = await _query("SELECT time, session_id::text, kind, payload FROM events ORDER BY time DESC LIMIT %s",
                        (min(limit, 200),))
    return [{"kind": r["kind"], "time": r["time"].isoformat(), "session_id": r["session_id"], **(r["payload"] or {})}
            for r in rows]


@app.websocket("/ws")
async def ws(socket: WebSocket) -> None:
    await socket.accept()

    async def push_state() -> None:
        last = None
        while True:
            control = None
            with suppress(aioredis.RedisError):
                control = await _get_control()
            await socket.send_json({"type": "session", "data": control})
            with suppress(aioredis.RedisError):
                current = await _get_state()
                if current != last:
                    await socket.send_json({"type": "state", "data": current})
                    last = current
            await asyncio.sleep(STATE_PERIOD_S)

    async def forward_events() -> None:
        while True:
            try:
                async with _redis.pubsub() as pubsub:
                    await pubsub.subscribe(EVENTS_CHANNEL)
                    async for msg in pubsub.listen():
                        if msg.get("type") == "message":
                            await socket.send_json({"type": "event", "data": json.loads(msg["data"])})
            except aioredis.RedisError:
                await asyncio.sleep(1.0)   # Redis 가 돌아오면 다시 구독

    tasks = [asyncio.create_task(push_state()), asyncio.create_task(forward_events())]
    try:
        while True:
            await socket.receive_text()   # 화면이 닫히면 WebSocketDisconnect
    except WebSocketDisconnect:
        pass
    finally:
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
