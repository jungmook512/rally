"""rally_db/rally_db/repository.py — PostgreSQL(Timescale) 기록 + Redis 상태·이벤트
담당: 조정묵

ROS 를 모르는 순수 파이썬. recorder_node 가 메시지를 dict·튜플로 바꿔서 여기 함수를 부른다 (로봇 없이 pytest 가능).

DB  : sessions · trials(즉시) · ts_wrench · ts_hand(묶어서) · events
Redis: rally:state  (현재 상태 JSON, 덮어쓰기)
       rally:events (pub/sub 채널, 대시보드가 구독)
       rally:dialog (최근 대화 목록, TTL 5분)
Redis 에는 영속 데이터를 두지 않는다 (CLAUDE.md DB 규칙). 기록의 원본은 항상 DB.

DB·Redis 가 잠깐 죽어도 노드는 살아야 한다 → 연결 실패는 전부 StorageUnavailable 로 바꿔 던지고,
다음 호출 때 다시 연결을 시도한다. 버퍼링·재시도 판단은 부르는 쪽(recorder_node)이 한다.
"""
from __future__ import annotations

import json
import os
import uuid
from datetime import datetime
from typing import Any, Sequence

import psycopg
import redis
from psycopg.types.json import Jsonb

from rally_db.convert import HAND_COLUMNS, WRENCH_COLUMNS

STATE_KEY = "rally:state"
EVENTS_CHANNEL = "rally:events"
DIALOG_KEY = "rally:dialog"
DIALOG_TTL_S = 300
DIALOG_MAX_LEN = 50

DEFAULT_DATABASE_URL = "postgresql://rally:rally@127.0.0.1:5432/rally"
DEFAULT_REDIS_URL = "redis://127.0.0.1:6379/0"



class StorageUnavailable(RuntimeError):
    """DB 또는 Redis 연결 실패. 잠시 뒤 다시 시도하면 된다."""


class Repository:
    def __init__(self, database_url: str | None = None, redis_url: str | None = None,
                 connect_timeout_s: float = 2.0):
        self.database_url = database_url or os.environ.get("DATABASE_URL", DEFAULT_DATABASE_URL)
        self.redis_url = redis_url or os.environ.get("REDIS_URL", DEFAULT_REDIS_URL)
        self.connect_timeout_s = connect_timeout_s
        self._db: psycopg.Connection | None = None
        self._redis = redis.Redis.from_url(self.redis_url, socket_timeout=connect_timeout_s,
                                           socket_connect_timeout=connect_timeout_s, decode_responses=True)

    # ---------- 연결 ----------
    def _conn(self) -> psycopg.Connection:
        if self._db is None or self._db.closed:
            try:
                self._db = psycopg.connect(self.database_url, autocommit=True,
                                           connect_timeout=max(1, int(self.connect_timeout_s)))
            except psycopg.OperationalError as e:
                self._db = None
                raise StorageUnavailable(f"DB 연결 실패: {e}") from e
        return self._db

    def _execute(self, sql: str, params: Sequence[Any] = (), many: bool = False) -> psycopg.Cursor:
        conn = self._conn()
        try:
            cur = conn.cursor()
            if many:
                cur.executemany(sql, params)
            else:
                cur.execute(sql, params)
            return cur
        except psycopg.OperationalError as e:
            conn.close()   # 다음 호출에서 새로 연결
            raise StorageUnavailable(f"DB 쓰기 실패: {e}") from e

    def _redis_call(self, fn, *args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except redis.RedisError as e:
            raise StorageUnavailable(f"Redis 실패: {e}") from e

    def close(self) -> None:
        if self._db is not None:
            self._db.close()
        self._redis.close()

    # ---------- DB ----------
    def ensure_session(self, session_id: str | None = None, patient_id: str | None = None) -> str:
        """세션 줄이 없으면 만든다. session_id 가 비면 새 UUID. 돌려준 id 를 이후 기록에 쓴다."""
        session_id = session_id or str(uuid.uuid4())
        uuid.UUID(session_id)   # 형식이 틀리면 ValueError — DB 에 넣기 전에 막는다
        if patient_id:
            self._execute("INSERT INTO patients (id) VALUES (%s) ON CONFLICT DO NOTHING", (patient_id,))
        self._execute("INSERT INTO sessions (id, patient_id) VALUES (%s, %s) ON CONFLICT DO NOTHING",
                      (session_id, patient_id))
        return session_id

    def end_session(self, session_id: str, report_md: str | None = None) -> None:
        self._execute("UPDATE sessions SET ended_at = now(), report_md = COALESCE(%s, report_md) WHERE id = %s",
                      (report_md, session_id))

    def add_trial(self, trial: dict[str, Any]) -> int:
        """trials 한 줄. trial 키: session_id, mode, trial_index, started_at, duration_s, success, error_mm,
        params, metrics (dict), decision (dict|None), feature_vec (list[float]|None). 새 trial id 를 돌려준다."""
        vec = trial.get("feature_vec")
        cur = self._execute(
            """INSERT INTO trials (session_id, mode, trial_index, started_at, duration_s, success, error_mm,
                                   params, metrics, decision, feature_vec)
               VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s::vector) RETURNING id""",
            (trial["session_id"], trial["mode"], trial["trial_index"], trial["started_at"],
             trial.get("duration_s"), trial.get("success"), trial.get("error_mm"),
             Jsonb(trial.get("params") or {}), Jsonb(trial.get("metrics") or {}),
             Jsonb(trial["decision"]) if trial.get("decision") is not None else None,
             _vector_literal(vec) if vec is not None else None))
        # 세션에서 해 본 모드 목록 갱신 (중복 없이)
        self._execute("""UPDATE sessions SET modes = array_append(COALESCE(modes, '{}'), %s::smallint)
                         WHERE id = %s AND NOT (%s::smallint = ANY(COALESCE(modes, '{}')))""",
                      (trial["mode"], trial["session_id"], trial["mode"]))
        return cur.fetchone()[0]

    def find_similar_trials(self, vec: Sequence[float], mode: int, k: int = 3,
                            exclude_session_id: str | None = None) -> list[dict[str, Any]]:
        """같은 모드에서 feature_vec 가 가장 가까운(L2) 시도 k 개. 가까운 순. exclude_session_id 의 시도는 뺀다
        (지금 세션의 최근 시도는 따로 보내므로). 시도가 수천 개 수준이라 인덱스 없이 정확히 다 비교한다."""
        cur = self._execute(
            """SELECT id, session_id::text, mode, trial_index, success, error_mm, duration_s, params, metrics,
                      decision, feature_vec <-> %s::vector AS distance
               FROM trials
               WHERE mode = %s AND feature_vec IS NOT NULL
                 AND (%s::uuid IS NULL OR session_id <> %s::uuid)
               ORDER BY distance, id LIMIT %s""",
            (_vector_literal(vec), mode, exclude_session_id, exclude_session_id, k))
        names = [d.name for d in cur.description]
        return [dict(zip(names, r)) for r in cur.fetchall()]

    def trials_for_embedding(self, everything: bool = False) -> list[dict[str, Any]]:
        """벡터를 (다시) 계산할 trials. everything=False 면 feature_vec 가 없는 줄만."""
        cur = self._execute(
            "SELECT id, mode, success, error_mm, duration_s, params, metrics, decision FROM trials"
            + ("" if everything else " WHERE feature_vec IS NULL") + " ORDER BY id")
        names = [d.name for d in cur.description]
        return [dict(zip(names, r)) for r in cur.fetchall()]

    def set_feature_vec(self, trial_id: int, vec: Sequence[float] | None) -> None:
        self._execute("UPDATE trials SET feature_vec = %s::vector WHERE id = %s",
                      (_vector_literal(vec) if vec is not None else None, trial_id))

    def add_wrench_rows(self, rows: Sequence[tuple]) -> int:
        """ts_wrench 여러 줄 (WRENCH_COLUMNS 순서 튜플). 넣은 줄 수."""
        return self._add_rows("ts_wrench", WRENCH_COLUMNS, rows)

    def add_hand_rows(self, rows: Sequence[tuple]) -> int:
        """ts_hand 여러 줄 (HAND_COLUMNS 순서 튜플). 넣은 줄 수."""
        return self._add_rows("ts_hand", HAND_COLUMNS, rows)

    def _add_rows(self, table: str, columns: Sequence[str], rows: Sequence[tuple]) -> int:
        if not rows:
            return 0
        placeholders = ", ".join(["%s"] * len(columns))
        self._execute(f"INSERT INTO {table} ({', '.join(columns)}) VALUES ({placeholders})", rows, many=True)
        return len(rows)

    def add_event(self, time: datetime, session_id: str | None, kind: str, payload: dict[str, Any]) -> None:
        """events 한 줄 + Redis rally:events 알림. kind: safety_stop | voice_intent | mode_change | llm_call ...
        DB 기록이 먼저다. 알림(Redis)이 실패해도 DB 기록은 남는다."""
        self._execute("INSERT INTO events (time, session_id, kind, payload) VALUES (%s, %s, %s, %s)",
                      (time, session_id, kind, Jsonb(payload)))
        self.publish_event(kind, payload, time=time, session_id=session_id)

    # ---------- Redis ----------
    def publish_event(self, kind: str, payload: dict[str, Any], time: datetime | None = None,
                      session_id: str | None = None) -> None:
        msg = {"kind": kind, "time": time.isoformat() if time else None, "session_id": session_id, **payload}
        self._redis_call(self._redis.publish, EVENTS_CHANNEL, json.dumps(msg, ensure_ascii=False))

    def set_state(self, state: dict[str, Any]) -> None:
        """rally:state 를 통째로 덮어쓴다 (대시보드가 처음 열릴 때 읽는 현재 상태)."""
        self._redis_call(self._redis.set, STATE_KEY, json.dumps(state, ensure_ascii=False))

    def get_state(self) -> dict[str, Any] | None:
        raw = self._redis_call(self._redis.get, STATE_KEY)
        return json.loads(raw) if raw else None

    def push_dialog(self, role: str, text: str) -> None:
        """최근 대화 한 줄 추가. 최대 DIALOG_MAX_LEN 줄, 마지막 대화 뒤 5분이면 통째로 사라진다."""
        pipe = self._redis.pipeline()
        pipe.rpush(DIALOG_KEY, json.dumps({"role": role, "text": text}, ensure_ascii=False))
        pipe.ltrim(DIALOG_KEY, -DIALOG_MAX_LEN, -1)
        pipe.expire(DIALOG_KEY, DIALOG_TTL_S)
        self._redis_call(pipe.execute)

    def get_dialog(self) -> list[dict[str, str]]:
        return [json.loads(x) for x in self._redis_call(self._redis.lrange, DIALOG_KEY, 0, -1)]


def _vector_literal(vec: Sequence[float]) -> str:
    """pgvector 입력 형식 '[0.1,0.2,...]' (pgvector 파이썬 어댑터 없이)"""
    return "[" + ",".join(f"{float(v):.6g}" for v in vec) + "]"
