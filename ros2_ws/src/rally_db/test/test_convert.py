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


def test_trial_to_event_is_json_and_has_end_time():
    import json
    msg = NS(header=header(), session_id=SID, mode=5, trial_index=2, success=False, error_mm=0.0,
             duration_s=4.0, param_keys=[], param_values=[],
             metric_keys=["handover_time_s"], metric_values=[3.25], notes="interrupted")
    ev = convert.trial_to_event(convert.trial_to_row(msg, SID), trial_id=7)
    assert json.loads(json.dumps(ev))["trial_id"] == 7
    assert ev["notes"] == "interrupted" and ev["metrics"] == {"handover_time_s": 3.25}
    assert ev["ended_at"].startswith("2025-10-09T08:53:20.5")   # header.stamp 그대로


def test_unmeasured_values_become_null():
    # 못 잰 값(NaN, #69) → DB NULL·JSON null. jsonb 는 NaN 을 거부하고, AVG 는 NULL 을 건너뛴다
    import json
    nan = float("nan")
    msg = NS(header=header(), session_id=SID, mode=1, trial_index=1, success=True, error_mm=nan,
             duration_s=2.0, param_keys=["lane"], param_values=[0.0],
             metric_keys=["contact_force_peak", "stop_x_m"], metric_values=[3.0, nan], notes="")
    row = convert.trial_to_row(msg, SID)
    assert row["error_mm"] is None and row["metrics"] == {"contact_force_peak": 3.0, "stop_x_m": None}
    ev = convert.trial_to_event(row, trial_id=1)
    assert json.loads(json.dumps(ev, allow_nan=False))["error_mm"] is None


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


def test_hand_row_nan_wrist_is_null():
    """손목 필터(#92): 손은 보이는데 손목을 못 잰 프레임은 NaN → NULL (0° 가 아니라 '모름')."""
    msg = hand()
    msg.wrist_angle_deg = float("nan")
    assert convert.hand_to_row(msg, None)[6] is None


def safety(level=2, reason="force_limit"):
    return NS(header=header(), level=level, reason=reason, speed_override=0.0, min_hand_distance=0.12345)


def test_safety_payload():
    p = convert.safety_to_payload(safety())
    assert p == {"level": "stop", "reason": "force_limit", "speed_override": 0.0, "min_hand_distance": 0.123}


def test_safety_payload_nan_hand_distance_is_null():
    # safety_monitor 는 '손은 보이는데 거리 모름' 을 NaN 으로 싣는다 (#88) — jsonb·브라우저 JSON 은 NaN 을 못 받는다
    import json
    p = convert.safety_to_payload(NS(header=header(), level=1, reason="hand_distance_unknown", speed_override=0.5,
                                     min_hand_distance=float("nan")))
    assert p["min_hand_distance"] is None
    json.dumps(p, allow_nan=False)


def body(lean=12.34, hike=float("nan"), elbow=151.2, comp="trunk_lean", detected=True):
    return NS(detected=detected, trunk_lean_deg=lean, shoulder_elevation_deg=hike, elbow_extension_deg=elbow,
              compensation_detected=comp != "none", compensation_type=comp)


def test_context_to_state():
    msg = NS(header=header(), session_id=SID, mode=5, trial_index=2, robot_state=1, safety=safety(0, ""),
             hand=hand(), last_intent=NS(intent="harder"), recent_success_rate=0.6, fatigue_score=0.2, body=body())
    s = convert.context_to_state(msg)
    assert s["robot_state"] == "moving" and s["safety"]["level"] == "normal"
    assert s["session_id"] == SID and s["last_intent"] == "harder"
    # 몸 자세 게이지 (10/15): 각도 소수 1자리, NaN(팔 안 뻗음·기준 전) 은 null, 보상 종류 그대로
    assert s["body"] == {"detected": True, "trunk_lean_deg": 12.3, "shoulder_elevation_deg": None,
                         "elbow_extension_deg": 151.2, "compensation_detected": True, "compensation_type": "trunk_lean"}
    import json
    json.dumps(s, allow_nan=False)


def test_body_payload_without_person():
    p = convert.body_to_payload(body(float("nan"), float("nan"), float("nan"), "none", detected=False))
    assert p == {"detected": False, "trunk_lean_deg": None, "shoulder_elevation_deg": None, "elbow_extension_deg": None,
                 "compensation_detected": False, "compensation_type": "none"}
