"""convert.py 단위 테스트 — ROS 없이 SimpleNamespace 로 가짜 메시지를 만든다."""
from datetime import timezone
from types import SimpleNamespace as NS

from rally_db import convert
from rally_db.convert import HAND_COLUMNS, WRENCH_COLUMNS

SID = "6f1c8a52-0000-4000-8000-000000000001"


def header(sec=1_760_000_000, nanosec=500_000_000):
    return NS(stamp=NS(sec=sec, nanosec=nanosec))


def vec(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def test_stamp_to_datetime_utc():
    dt = convert.stamp_to_datetime(header().stamp)
    assert dt.tzinfo == timezone.utc
    assert dt.timestamp() == 1_760_000_000.5


def test_stamp_zero_uses_now():
    dt = convert.stamp_to_datetime(NS(sec=0, nanosec=0))
    assert dt.year >= 2026


def test_session_uuid():
    assert convert.session_uuid("") is None
    assert convert.session_uuid(SID) == SID
    # UUID 가 아닌 이름은 항상 같은 UUID 로
    assert convert.session_uuid("S001") == convert.session_uuid("S001")
    assert convert.session_uuid("S001") != convert.session_uuid("S002")


def test_trial_to_row():
    msg = NS(header=header(), session_id=SID, mode=1, trial_index=3, success=True, error_mm=12.5,
             duration_s=4.0, param_keys=["push_speed"], param_values=[0.25],
             metric_keys=["wrist_angle_deg"], metric_values=[41.0], notes="")
    row = convert.trial_to_row(msg, SID)
    assert row["mode"] == 1 and row["trial_index"] == 3 and row["success"] is True
    assert row["params"] == {"push_speed": 0.25}
    assert row["metrics"] == {"wrist_angle_deg": 41.0}
    assert row["decision"] is None
    # header.stamp 는 끝난 시각 → 시작은 4 s 전
    assert row["started_at"].timestamp() == 1_760_000_000.5 - 4.0


def test_wrench_row_matches_columns():
    msg = NS(header=header(), wrench=NS(force=vec(1, 2, 3), torque=vec(4, 5, 6)))
    row = convert.wrench_to_row(msg, SID)
    assert len(row) == len(WRENCH_COLUMNS)
    assert row[1:] == (SID, None, 1, 2, 3, 4, 5, 6)


def hand(detected=True):
    return NS(header=header(), detected=detected, palm_pose=NS(position=vec(0.1, 0.2, 0.3)),
              wrist_angle_deg=30.0, grip_closure=0.8, gesture=1)


def test_hand_row():
    row = convert.hand_to_row(hand(), None)
    assert len(row) == len(HAND_COLUMNS)
    assert row[3:] == (0.1, 0.2, 0.3, 30.0, 0.8, 1)
    assert convert.hand_to_row(hand(detected=False), None) is None


def safety(level=2, reason="force_limit"):
    return NS(header=header(), level=level, reason=reason, speed_override=0.0, min_hand_distance=0.12345)


def test_safety_payload():
    p = convert.safety_to_payload(safety())
    assert p == {"level": "stop", "reason": "force_limit", "speed_override": 0.0, "min_hand_distance": 0.123}


def test_context_to_state():
    msg = NS(header=header(), session_id=SID, mode=5, trial_index=2, robot_state=1, safety=safety(0, ""),
             hand=hand(), last_intent=NS(intent="harder"), recent_success_rate=0.6, fatigue_score=0.2)
    s = convert.context_to_state(msg)
    assert s["robot_state"] == "moving" and s["safety"]["level"] == "normal"
    assert s["session_id"] == SID and s["last_intent"] == "harder"
