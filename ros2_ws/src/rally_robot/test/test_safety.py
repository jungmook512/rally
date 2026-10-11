"""safety_monitor 판정 함수 (로봇 없이): 손 거리 NaN·접촉 중 완화(#59), 입력 끊김(#14·#59), 손 ↔ 로봇 거리(#98)·손 놓침(#95).
로봇 거리 없이(robot_dist=None) 부르는 테스트는 tcp_pose 를 모를 때의 선 기준 경로다."""
import math

import pytest

from rally_interfaces.msg import SafetyStatus
from rally_robot.safety_monitor import (hand_level, hand_lost_level, hand_robot_dist, hand_unknown_level, in_workspace,
                                        input_level, point_segment_dist, robot_segment, worst)

NORMAL, SLOW, STOP = SafetyStatus.LEVEL_NORMAL, SafetyStatus.LEVEL_SLOW, SafetyStatus.LEVEL_STOP
CFG = {"slow_distance_m": 0.25, "stop_distance_m": 0.15, "hand_deep_stop_m": -0.15,
       "hand_stop_robot_m": 0.20, "hand_slow_robot_m": 0.35, "hand_lost_stop_s": 1.0, "hand_lost_slow_s": 3.0}
CFG_98 = {**CFG, "slow_distance_m": 0.10}       # safety.yaml 10/8 값


def test_no_hand_is_normal():
    assert hand_level(None, False, CFG) == (NORMAL, "")


def test_hand_far_is_normal_and_slow_zone():
    assert hand_level(0.40, False, CFG)[0] == NORMAL
    assert hand_level(0.20, False, CFG) == (SLOW, "hand_near_centerline")


def test_nan_distance_is_slow():
    # 손은 보이는데 거리 모름 — 예전엔 비교가 전부 거짓이라 아무 반응이 없었다 (#59)
    assert hand_level(math.nan, False, CFG) == (SLOW, "hand_distance_unknown")
    assert hand_level(math.nan, True, CFG) == (SLOW, "hand_distance_unknown")


def test_near_without_patient_contact_is_stop():
    # 치기·집기·놓기(contact_expected 만)는 완화 대상이 아니다 — 호출자는 patient_contact=False 로 부른다
    assert hand_level(0.10, False, CFG) == (STOP, "hand_near_centerline")
    assert hand_level(-0.06, False, CFG) == (STOP, "hand_near_centerline")


def test_near_with_patient_contact_is_slow():
    # 건네기 당김 대기·후퇴, 저항 밀기: 환자가 중앙선 앞 6 cm 의 물체를 잡는 순간 (#59 3번)
    assert hand_level(-0.06, True, CFG) == (SLOW, "hand_near_patient_contact")
    assert hand_level(0.10, True, CFG) == (SLOW, "hand_near_patient_contact")


def test_deep_intrusion_is_stop_even_with_patient_contact():
    assert hand_level(-0.20, True, CFG) == (STOP, "hand_near_centerline")
    assert hand_level(-0.15, True, CFG)[0] == SLOW        # 경계는 완화 쪽 (>= hand_deep_stop_m)


# ----- 손 ↔ 로봇 거리 (#98) -----

def test_mode1_hand_near_line_but_robot_far_is_not_stop():
    # 10/8 실로봇: 손끝이 선 0.10 m, 로봇(손 보기 자세)은 손에서 0.30 m 넘게 → 예전엔 STOP, 이제 SLOW(로봇 0.35 안)·NORMAL
    assert hand_level(0.10, False, CFG_98, robot_dist=0.30) == (SLOW, "hand_near_robot")
    assert hand_level(0.12, False, CFG_98, robot_dist=0.50) == (NORMAL, "")
    assert hand_level(0.05, False, CFG_98, robot_dist=0.50) == (SLOW, "hand_near_centerline")


def test_hand_near_robot_is_stop_unless_patient_contact():
    assert hand_level(0.30, False, CFG_98, robot_dist=0.15) == (STOP, "hand_near_robot")
    assert hand_level(0.30, True, CFG_98, robot_dist=0.15) == (SLOW, "hand_near_patient_contact")
    assert hand_level(0.30, False, CFG_98, robot_dist=0.20)[0] == SLOW      # 경계는 SLOW 쪽


def test_deep_intrusion_is_stop_even_if_robot_far():
    assert hand_level(-0.20, False, CFG_98, robot_dist=0.60) == (STOP, "hand_near_centerline")
    assert hand_level(-0.20, True, CFG_98, robot_dist=0.60) == (STOP, "hand_near_centerline")


def test_unknown_robot_distance_falls_back_to_line_stop():
    # tcp_pose 끊김 → 예전 선 기준 STOP (그때는 tcp_stale SLOW 도 같이 나온다)
    assert hand_level(0.12, False, CFG_98, robot_dist=None) == (STOP, "hand_near_centerline")


def test_robot_segment_follows_tool_z():
    # 툴이 아래를 향함 (x 축 180° 회전, q = (1, 0, 0, 0)) → 툴 z = −z, 플랜지는 TCP 위 0.228 m
    tcp, flange = robot_segment((0.4, 0.0, 0.1), (1.0, 0.0, 0.0, 0.0), 0.228)
    assert tcp == (0.4, 0.0, 0.1) and flange == pytest.approx((0.4, 0.0, 0.328))
    # 단위 쿼터니언 (0,0,0,1) → 툴 z = +z, 플랜지는 TCP 아래
    assert robot_segment((0.0, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0), 0.2)[1] == pytest.approx((0.0, 0.0, 0.3))


def test_hand_robot_dist_uses_nearest_finite_landmark():
    seg = ((0.4, 0.0, 0.1), (0.4, 0.0, 0.328))
    assert point_segment_dist((0.6, 0.0, 0.2), *seg) == pytest.approx(0.2)        # 선분 옆
    assert point_segment_dist((0.4, 0.0, 0.0), *seg) == pytest.approx(0.1)        # TCP 아래 (끝점)
    pts = [(0.9, 0.0, 0.05), (0.55, 0.0, 0.2), (math.nan, 0.0, 0.0)]
    assert hand_robot_dist(pts, seg) == pytest.approx(0.15)
    assert hand_robot_dist([(math.nan, 0.0, 0.0)], seg) is None
    assert hand_robot_dist([], seg) is None


# ----- 손 놓침 (#95) -----

def test_lost_after_stop_holds_stop_then_slow():
    assert hand_lost_level(0.5, STOP, False, CFG) == (STOP, "hand_lost_near")
    assert hand_lost_level(1.5, STOP, False, CFG) == (SLOW, "hand_lost_near")
    assert hand_lost_level(3.0, STOP, False, CFG) == (NORMAL, "")


def test_lost_after_slow_or_patient_contact_is_slow_only():
    assert hand_lost_level(0.2, SLOW, False, CFG) == (SLOW, "hand_lost_near")
    assert hand_lost_level(0.2, STOP, True, CFG) == (SLOW, "hand_lost_near")
    assert hand_lost_level(3.5, SLOW, False, CFG) == (NORMAL, "")


def test_lost_from_far_is_normal():
    assert hand_lost_level(0.1, NORMAL, False, CFG) == (NORMAL, "")


TIMEOUTS = {"hand": 0.5, "wrench": 0.5, "tcp": 0.5}


def test_never_received_is_not_judged():
    # 손 노드 없이 로봇만 시험할 때 속도가 바뀌지 않게 (경고 로그만)
    assert input_level({"hand": None, "wrench": None, "tcp": None}, 100.0, TIMEOUTS) == (NORMAL, "")


def test_fresh_inputs_are_normal():
    assert input_level({"hand": 99.8, "wrench": 99.9, "tcp": 99.7}, 100.0, TIMEOUTS) == (NORMAL, "")


@pytest.mark.parametrize("name", ["hand", "wrench", "tcp"])
def test_stale_input_is_slow(name):
    rx = {"hand": 99.9, "wrench": 99.9, "tcp": 99.9}
    rx[name] = 99.0
    assert input_level(rx, 100.0, TIMEOUTS) == (SLOW, f"{name}_stale")


def test_worst_picks_most_severe():
    assert worst((SLOW, "hand_stale"), (STOP, "hand_near_centerline")) == (STOP, "hand_near_centerline")
    assert worst((SLOW, "hand_distance_unknown"), (SLOW, "tcp_stale")) == (SLOW, "hand_distance_unknown")
    assert worst((NORMAL, ""), (NORMAL, "")) == (NORMAL, "")


def test_tf_robot_segment_and_bad_tf_rejected():
    from rally_robot.safety_monitor import tf_robot_segment
    # 플랜지 (0.4, 0, 0.33), link_6 z 가 아래 (q = (1,0,0,0)) → TCP 는 0.228 m 아래
    seg = tf_robot_segment((0.4, 0.0, 0.33), (1.0, 0.0, 0.0, 0.0), 0.228, (0.401, 0.0, 0.10), 0.20)
    assert seg[0] == pytest.approx((0.4, 0.0, 0.102)) and seg[1] == (0.4, 0.0, 0.33)
    # 관절값 끊김 → tf 가 팔을 똑바로 세움 (TCP 가 1 m 넘게 위) → tcp_pose 와 달라서 안 씀
    assert tf_robot_segment((0.0, 0.0, 1.1), (0.0, 0.0, 0.0, 1.0), 0.228, (0.4, 0.0, 0.10), 0.20) is None
    assert tf_robot_segment((0.0, 0.0, 1.1), (0.0, 0.0, 0.0, 1.0), 0.228, None, 0.20) is not None   # 비교할 tcp_pose 없음


def test_required_input_never_received_is_stop_after_grace():
    rx = {"hand": None, "wrench": None, "tcp": 99.9}
    req = ("wrench", "tcp")
    assert input_level(rx, 100.0, TIMEOUTS, req, since_start=3.0, grace_s=5.0) == (NORMAL, "")       # 기동 직후
    assert input_level(rx, 100.0, TIMEOUTS, req, since_start=6.0, grace_s=5.0) == (STOP, "wrench_missing")
    rx["wrench"] = 99.9
    assert input_level(rx, 100.0, TIMEOUTS, req, since_start=6.0, grace_s=5.0) == (NORMAL, "")       # 받으면 풀림
    # 손은 필수 아님 (손 노드 없이 로봇만 시험)
    assert input_level({"hand": None, "wrench": 99.9, "tcp": 99.9}, 100.0, TIMEOUTS, req, 60.0, 5.0) == (NORMAL, "")


def test_hand_off_table_uses_robot_distance_only():
    from rally_robot.safety_monitor import hand_on_table
    # 옆 책상 손 (y 0.52): x 로는 '선 너머 0.19 m' 지만 테이블 밖 → 로봇에서 멀면 NORMAL, 가까우면 STOP
    assert hand_level(-0.19, False, CFG_98, robot_dist=0.40, on_table=False) == (NORMAL, "")
    assert hand_level(-0.19, False, CFG_98, robot_dist=0.30, on_table=False) == (SLOW, "hand_near_robot")
    assert hand_level(-0.19, False, CFG_98, robot_dist=0.15, on_table=False) == (STOP, "hand_near_robot")
    # 로봇 위치를 모르면 테이블 밖이어도 선 기준
    assert hand_level(-0.19, False, CFG_98, robot_dist=None, on_table=False) == (STOP, "hand_near_centerline")
    assert hand_on_table([(0.44, 0.52, 0.1), (0.45, 0.60, 0.1)], -0.40, 0.40) is False
    assert hand_on_table([(0.44, 0.52, 0.1), (0.45, 0.38, 0.1)], -0.40, 0.40) is True    # 한 점이라도 안이면 테이블 위
    assert hand_on_table([], -0.40, 0.40) is True


# ----- 리뷰 #104 -----

def test_unknown_distance_after_stop_holds_stop():
    # 손이 로봇 STOP 구역에 들어와 깊이가 빠짐(카메라 30 cm 안) → 바로 SLOW 로 풀리면 안 됨
    assert hand_unknown_level(0.5, STOP, False, CFG) == (STOP, "hand_lost_near")
    assert hand_unknown_level(1.5, STOP, False, CFG) == (SLOW, "hand_distance_unknown")
    assert hand_unknown_level(10.0, STOP, False, CFG) == (SLOW, "hand_distance_unknown")    # NaN 인 동안은 계속 SLOW
    assert hand_unknown_level(0.5, STOP, True, CFG) == (SLOW, "hand_distance_unknown")     # 환자 접촉 중은 SLOW
    assert hand_unknown_level(0.1, NORMAL, False, CFG) == (SLOW, "hand_distance_unknown")
    assert hand_unknown_level(0.1, SLOW, False, CFG) == (SLOW, "hand_distance_unknown")


def test_wrench_lost_mid_session_is_stop():
    rx = {"hand": 99.9, "wrench": 97.5, "tcp": 99.9}
    assert input_level(rx, 100.0, TIMEOUTS, ("wrench", "tcp"), 60.0, 5.0) == (SLOW, "wrench_stale")       # 예전 동작
    assert input_level(rx, 100.0, TIMEOUTS, ("wrench", "tcp"), 60.0, 5.0, ("wrench",), 2.0) == (STOP, "wrench_lost")
    rx["wrench"] = 98.5
    assert input_level(rx, 100.0, TIMEOUTS, ("wrench", "tcp"), 60.0, 5.0, ("wrench",), 2.0) == (SLOW, "wrench_stale")
    rx["wrench"] = 99.9
    assert input_level(rx, 100.0, TIMEOUTS, ("wrench", "tcp"), 60.0, 5.0, ("wrench",), 2.0) == (NORMAL, "")   # 받으면 풀림
    # 한 번도 못 받은 것은 _missing 규칙 (grace 전에는 판정 안 함)
    assert input_level({"hand": None, "wrench": None, "tcp": 99.9}, 100.0, TIMEOUTS, (), 1.0, 5.0, ("wrench",), 2.0) == (NORMAL, "")


def test_in_workspace():
    lim = {"x": (0.20, 0.57), "y": (-0.45, 0.45), "z": (0.0, 0.60)}
    assert in_workspace((0.40, 0.0, 0.30), lim)
    assert not in_workspace((0.65, -0.15, 0.45), lim)
    assert not in_workspace((0.0, 0.0, 1.1), lim)       # 관절값 0 으로 채운 '똑바로 선 자세'


# ---------- 개발용 hand_stop_mode (10/11) ----------
def test_hand_mode_level():
    from rally_robot.safety_monitor import hand_mode_level
    S = SafetyStatus
    assert hand_mode_level((S.LEVEL_STOP, "hand_near_robot"), "stop") == (S.LEVEL_STOP, "hand_near_robot")
    assert hand_mode_level((S.LEVEL_STOP, "hand_near_robot"), "slow") == (S.LEVEL_SLOW, "hand_near_robot")
    assert hand_mode_level((S.LEVEL_SLOW, "hand_near_centerline"), "slow") == (S.LEVEL_SLOW, "hand_near_centerline")
    assert hand_mode_level((S.LEVEL_STOP, "hand_near_centerline"), "off") == (S.LEVEL_NORMAL, "")


@pytest.fixture
def monitor():
    import rclpy
    rclpy.init(args=["--ros-args", "-p", "robot_tf_enabled:=false", "-p", "required_inputs:=['']"])
    from rally_robot.safety_monitor import SafetyMonitor
    node = SafetyMonitor()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def hand_msg(detected, dist):
    from rally_interfaces.msg import HandState
    m = HandState(detected=detected)
    m.distance_to_centerline = dist
    return m


def set_mode(node, mode):
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter("hand_stop_mode", Parameter.Type.STRING, mode)])


@pytest.mark.parametrize("mode, near, lost", [
    ("stop", SafetyStatus.LEVEL_STOP, SafetyStatus.LEVEL_STOP),     # 기본: 근접 STOP, 놓쳐도 1 s STOP 유지(#95)
    ("slow", SafetyStatus.LEVEL_SLOW, SafetyStatus.LEVEL_NORMAL),   # 근접 SLOW, 놓치면 바로 NORMAL (보이는 손만)
    ("off", SafetyStatus.LEVEL_NORMAL, SafetyStatus.LEVEL_NORMAL),
])
def test_hand_stop_mode_in_node(monitor, mode, near, lost):
    set_mode(monitor, mode)
    monitor.on_hand(hand_msg(True, 0.05))           # 로봇 위치 모름 → 선 기준 0.15 안 = STOP
    monitor.tick()
    assert monitor.level == near
    monitor.on_hand(hand_msg(False, 0.0))           # 놓침
    monitor.tick()
    assert monitor.level == lost


def test_hand_stop_mode_ignores_stale_hand_and_keeps_other_stops(monitor):
    import time as _t
    set_mode(monitor, "slow")
    monitor.on_hand(hand_msg(True, 0.05))
    monitor.last_rx["hand"] = _t.monotonic() - 10   # 손 메시지 끊김 — 마지막 거리(가까움)를 들고 있지 않는다
    monitor.tick()
    assert monitor.level == SafetyStatus.LEVEL_NORMAL
    monitor._stop("force_limit")                    # 다른 STOP 은 그대로
    monitor.tick()
    assert monitor.level == SafetyStatus.LEVEL_STOP and monitor.reason == "force_limit"


def test_unknown_mode_falls_back_to_stop(monitor):
    set_mode(monitor, "nothing")
    monitor.on_hand(hand_msg(True, 0.05))
    monitor.tick()
    assert monitor.level == SafetyStatus.LEVEL_STOP
