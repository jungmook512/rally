"""RALLY 대시보드 — FastAPI + WebSocket (Redis 실시간 · PostgreSQL 기록)
담당: 조정묵

  GET  /              화면 (web/index.html)
  GET  /api/state     현재 상태 — Redis rally:state (recorder 가 /rally/brain/context 로 0.5 s 마다 덮어씀)
  GET  /api/trials    최근 세션의 시도 목록 — DB trials
  GET  /api/events    최근 사건 — DB events (안전·음성·모드 바뀜)
  WS   /ws            {"type": "state", "data": …} 0.5 s 마다 바뀌었을 때 / {"type": "event", "data": …} rally:events 그대로
대시보드는 ROS 를 쓰지 않는다 (가벼운 python 이미지). 모드 전환·비상정지 버튼은 ROS 다리가 생긴 뒤 (todo Day 6).
Redis·DB 가 죽어 있어도 서버는 뜨고, 해당 API 는 503 + 이유를 돌려준다. 화면은 다시 연결될 때까지 재시도한다.
"""
from __future__ import annotations

import asyncio
import json
import os
from contextlib import suppress
from pathlib import Path
from typing import Any

import psycopg
import redis.asyncio as aioredis
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from psycopg.rows import dict_row

DATABASE_URL = os.environ.get("DATABASE_URL", "postgresql://rally:rally@127.0.0.1:5432/rally")
REDIS_URL = os.environ.get("REDIS_URL", "redis://127.0.0.1:6379/0")
STATE_KEY = "rally:state"            # rally_db/repository.py 와 같은 이름
EVENTS_CHANNEL = "rally:events"
STATE_PERIOD_S = 0.5
WEB_DIR = Path(__file__).resolve().parents[1] / "web"

app = FastAPI(title="RALLY dashboard")
_redis = aioredis.from_url(REDIS_URL, decode_responses=True, socket_connect_timeout=1.0)


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
            with suppress(aioredis.RedisError):
                current = await _get_state()
                if current != last:
                    await socket.send_json({"type": "state", "data": current})
                    last = current
            await asyncio.sleep(STATE_PERIOD_S)

    async def forward_events() -> None:
        while True:
            try:
                pubsub = _redis.pubsub()
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
