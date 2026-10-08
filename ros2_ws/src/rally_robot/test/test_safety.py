"""safety_monitor 판정 함수 (로봇 없이): 손 거리 NaN·접촉 중 완화(#59), 입력 끊김(#14·#59)."""
import math

import pytest

from rally_interfaces.msg import SafetyStatus
from rally_robot.safety_monitor import hand_level, input_level, worst

NORMAL, SLOW, STOP = SafetyStatus.LEVEL_NORMAL, SafetyStatus.LEVEL_SLOW, SafetyStatus.LEVEL_STOP
CFG = {"slow_distance_m": 0.25, "stop_distance_m": 0.15, "hand_deep_stop_m": -0.15}


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
