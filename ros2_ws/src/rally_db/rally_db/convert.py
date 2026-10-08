"""rally_db/rally_db/convert.py — ROS 메시지 → repository 에 넘길 dict·튜플
담당: 조정묵

메시지 필드만 읽고 ROS 를 import 하지 않는다 → pytest 에서 SimpleNamespace 로 가짜 메시지를 만들어 시험한다.
"""
from __future__ import annotations

import math
import uuid
from datetime import datetime, timedelta, timezone
from typing import Any

# ts_* 열 순서 — add_*_rows 에 넘기는 튜플도 이 순서
WRENCH_COLUMNS = ("time", "session_id", "trial_id", "fx", "fy", "fz", "tx", "ty", "tz")
HAND_COLUMNS = ("time", "session_id", "trial_id",
                "palm_x", "palm_y", "palm_z", "wrist_angle_deg", "grip_closure", "gesture")

SAFETY_LEVELS = {0: "normal", 1: "slow", 2: "stop"}
ROBOT_STATES = {0: "idle", 1: "moving", 2: "holding", 3: "stopped"}
SESSION_NAMESPACE = uuid.UUID("6f1c8a52-3d4e-4b7a-9c1e-5a0d2b7e9f10")   # 고정값 — 바꾸면 같은 이름이 다른 세션이 된다


def finite_or_none(value: float) -> float | None:
    """못 잰 값(NaN, #69) → None (DB NULL, JSON null). AVG 등 SQL 집계가 NULL 은 건너뛴다."""
    value = float(value)
    return value if math.isfinite(value) else None


def stamp_to_datetime(stamp) -> datetime:
    """header.stamp → UTC datetime. stamp 를 안 채운 발행자(0)면 지금 시각."""
    if stamp.sec == 0 and stamp.nanosec == 0:
        return datetime.now(timezone.utc)
    return datetime.fromtimestamp(stamp.sec + stamp.nanosec * 1e-9, tz=timezone.utc)


def session_uuid(session_id: str) -> str | None:
    """메시지의 session_id → DB sessions.id(UUID). 빈 값이면 None.
    UUID 형식이 아니면(예: "S001") 같은 이름이 항상 같은 UUID 가 되도록 uuid5 로 바꾼다."""
    if not session_id:
        return None
    try:
        return str(uuid.UUID(session_id))
    except ValueError:
        return str(uuid.uuid5(SESSION_NAMESPACE, session_id))


def trial_to_row(msg, session_id: str) -> dict[str, Any]:
    """TrialResult → trials 한 줄. header.stamp 는 시도가 끝난 시각으로 보고 started_at = stamp - duration_s."""
    ended_at = stamp_to_datetime(msg.header.stamp)
    return {
        "session_id": session_id,
        "mode": int(msg.mode),
        "trial_index": int(msg.trial_index),
        "started_at": ended_at - timedelta(seconds=max(0.0, float(msg.duration_s))),
        "duration_s": float(msg.duration_s),
        "success": bool(msg.success),
        "error_mm": finite_or_none(msg.error_mm),
        "params": _pairs(msg.param_keys, msg.param_values),
        "metrics": _pairs(msg.metric_keys, msg.metric_values),
        "decision": {"notes": msg.notes} if msg.notes else None,
    }


def trial_to_event(row: dict[str, Any], trial_id: int) -> dict[str, Any]:
    """trials 한 줄 → 대시보드 알림 (JSON 으로 보낼 수 있는 값만)."""
    return {
        "trial_id": trial_id,
        "session_id": row["session_id"],
        "mode": row["mode"],
        "trial_index": row["trial_index"],
        "success": row["success"],
        "error_mm": None if row["error_mm"] is None else round(row["error_mm"], 1),
        "duration_s": round(row["duration_s"], 2),
        "metrics": row["metrics"],
        "notes": (row["decision"] or {}).get("notes", ""),
        "ended_at": (row["started_at"] + timedelta(seconds=row["duration_s"])).isoformat(),
    }


def wrench_to_row(msg, session_id: str | None) -> tuple:
    """WrenchStamped → ts_wrench 튜플 (WRENCH_COLUMNS 순서). trial_id 는 비워 두고 시각으로 잇는다."""
    f, t = msg.wrench.force, msg.wrench.torque
    return (stamp_to_datetime(msg.header.stamp), session_id, None, f.x, f.y, f.z, t.x, t.y, t.z)


def hand_to_row(msg, session_id: str | None) -> tuple | None:
    """HandState → ts_hand 튜플 (HAND_COLUMNS 순서). 손이 안 잡힌 프레임은 None (기록 안 함)."""
    if not msg.detected:
        return None
    p = msg.palm_pose.position
    return (stamp_to_datetime(msg.header.stamp), session_id, None,
            p.x, p.y, p.z, msg.wrist_angle_deg, msg.grip_closure, int(msg.gesture))


def safety_to_payload(msg) -> dict[str, Any]:
    return {
        "level": SAFETY_LEVELS.get(int(msg.level), str(msg.level)),
        "reason": msg.reason,
        "speed_override": round(float(msg.speed_override), 3),
        "min_hand_distance": finite_or_none(round(float(msg.min_hand_distance), 3)),   # NaN = 손은 보이는데 거리 모름 → null (jsonb 는 NaN 거부)
    }


def intent_to_payload(msg) -> dict[str, Any]:
    return {
        "intent": msg.intent,
        "text": msg.text,
        "slots": dict(zip(msg.slot_keys, msg.slot_values)),
        "confidence": round(float(msg.confidence), 3),
        "is_emergency": bool(msg.is_emergency),
    }


def context_to_state(msg) -> dict[str, Any]:
    """SceneContext → Redis rally:state (대시보드가 처음 열릴 때 보는 요약). 영상·관절 원본은 넣지 않는다."""
    return {
        "time": stamp_to_datetime(msg.header.stamp).isoformat(),
        "session_id": session_uuid(msg.session_id),
        "mode": int(msg.mode),
        "trial_index": int(msg.trial_index),
        "robot_state": ROBOT_STATES.get(int(msg.robot_state), str(msg.robot_state)),
        "safety": safety_to_payload(msg.safety),
        "hand_detected": bool(msg.hand.detected),
        "gesture": int(msg.hand.gesture),
        "last_intent": msg.last_intent.intent,
        "recent_success_rate": round(float(msg.recent_success_rate), 3),
        "fatigue_score": round(float(msg.fatigue_score), 3),
    }


def _pairs(keys, values) -> dict[str, float | None]:
    # JSON 에는 NaN 이 없다 (Postgres jsonb 가 거부) → 못 잰 값은 null
    return {k: finite_or_none(v) for k, v in zip(keys, values)}
