"""rally_robot/safety_monitor.py — 100 Hz 안전 감시 → /rally/safety/status (Reliable)
담당: 정정인
입력: /rally/perception/hand (distance_to_centerline), /rally/voice/intent (is_emergency), /rally/safety/estop (Bool),
      /rally/robot/wrench (힘 한계·nudge), /rally/robot/tcp_pose (작업 공간·이동 속도), /rally/safety/contact_expected (skill_server)
규칙 (safety.yaml): 손 ↔ 로봇 0.35 m → SLOW 0.5, 0.20 m → STOP · 손 ↔ 중앙선 0.10 m → SLOW, 선 너머 0.15 m → STOP (#98)
      / |F| > 25 N → STOP / nudge: 정지 중·접촉 예정 아님 && 한 주기 |Δ|F|| > 임계 → STOP
      / TCP 가 작업 공간 밖 → STOP / "그만"·estop → STOP (주먹 제스처 정지는 #59 에서 뺌)
- nudge 는 "접촉이 없어야 할 때 누가 로봇을 쳤다" 용이다. 스킬이 접촉 예정 플래그를 올린 동안(집기 하강·핸드오버 당김·저항 밀기)과
  로봇이 움직이는 동안(가속 관성)은 판정하지 않는다. 절대 힘 한계는 항상 적용한다.
  '움직이는 중' = TCP 직선 속도 ≥ nudge_speed_max_mps **또는** 툴 회전 속도 ≥ nudge_ang_speed_max_dps.
  멈춘 뒤 nudge_settle_s 까지도 '움직이는 중'으로 본다 (감속 관성 힘이 풀리는 구간, 10/8 실로봇 오탐).
  (10/8 실로봇: 손끝은 거의 제자리에서 손목만 45° 도는 관절 이동 중 툴 힘이 출렁여 nudge STOP — 직선 속도만 봐서 '정지'로 오판)
- 접촉 예정 플래그는 contact_timeout_s(0.5 s) 안에 갱신되지 않으면 꺼진 것으로 본다 (skill_server 가 죽어도 감시 복귀).
- 작업 공간 x 상한 (10/14 정정인): 평소 workspace_x_max_m. 모드 3 공 떼기 스킬이 /rally/safety/workspace_extend 하트비트를 켠 동안
  (박스로 가서 놓고 돌아오는 구간, 최대 workspace_extend_max_s)만 workspace_x_max_extended_m. y·z·손 근접·힘 한계는 그대로.
- 힘 한계 (#144, 10/12 정정인 조건부 승인): 평소 force_limit_n(25 N). 모드 3 받기 창 — /rally/safety/catch_window 와
  contact_expected 가 둘 다 켜진 동안(catch_ball 이 투구 감지 ~ 교차 + 0.5 s, 약 1.2 s)에만 catch_window_force_limit_n(130 N).
  가속 2000 요격 이동 관성 59~73 N + 공 충격 15 N. 창 신호가 끊기면 contact_timeout_s 뒤 25 N. 두산 자체 충돌 감지는 그대로.
  창이 열린 뒤 catch_window_max_s(2 s) 가 지나면 신호가 계속 와도 25 N (skill 스레드가 멈춰 하트비트만 도는 경우 — #165 리뷰).
  다시 130 N 이 되려면 창이 한 번 꺼졌다(False 또는 하트비트 끊김) 새로 열려야 한다.
- 손 거리 (hand_level, #59 · #98):
  손 없음 → NORMAL / 손은 보이는데 거리 NaN(깊이·tf 없음) → SLOW (hand_distance_unknown). 단 직전이 STOP 이었으면 놓침과 같이
  hand_lost_stop_s STOP 유지 (깊이는 카메라 30 cm 안에서 빠지므로 손이 로봇에 가까울수록 NaN — 리뷰 #104) /
  손이 중앙선 너머 hand_deep_stop_m(−0.15) 보다 깊이 → STOP (접촉 중이어도). 사유는 hand_deep_end_m(−0.30) 보다 얕으면 hand_near_centerline
  (손 빼면 brain 이 이어 감), 더 깊으면 hand_deep_intrusion (brain 대기 목록에 없는 이름 → 세션 종료) — 10/12 정정인 결정 /
  손 관절 21점 ↔ 로봇 끝 선분(TCP → 플랜지, robot_segment_m) 최소 거리가 hand_stop_robot_m 안 → STOP (hand_near_robot),
  hand_slow_robot_m 안 → SLOW / 중앙선에서 slow_distance_m 안 → SLOW.
  로봇 위치(tf·tcp_pose 둘 다)를 모르거나 손 관절이 base_link 가 아니면 예전 선 기준(stop_distance_m 안 → STOP).
  로봇 거리는 선분 '중심축'까지라 그리퍼·카메라 반경만큼 실제 여유가 작다 (safety.yaml hand_stop_robot_m 주석).
  10/8 왜 바꿨나: 테이블이 선 너머 35 cm 뿐이라 모드 1 에서 환자 손끝이 레인 끝에만 있어도 선 15 cm 안 → 로봇이 30 cm 넘게
  떨어져 있어도 STOP 으로 세션이 끝났다 (#98, 실로봇 3회). 위험은 손과 로봇이 가까운 것이므로 로봇 기준으로 본다.
  단 skill_server 가 /rally/safety/patient_contact(환자가 물체·손잡이를 잡는 구간: 건네기 당김
  대기 → 후퇴, 저항 밀기)를 올린 동안은 근접 STOP 대신 SLOW (hand_near_patient_contact). 이 구간엔 스킬이 환자 쪽(+x) 명령 이동을 하지 않는다
  (skill_server 설계 메모) — 환자에게 끌려 움직이는 건 힘 한계가 막는다. 손이 로봇 구역 깊이(hand_deep_stop_m) 들어오거나
  손 메시지가 끊겼으면(마지막 거리를 믿을 수 없음) 완화하지 않는다. 치기·집기·놓기의 contact_expected 는 완화 대상이 아니다.
- 손 놓침 (hand_lost_level, #95): 보이던 손이 detected=false 가 되면, 직전 판정이 STOP 이었으면 hand_lost_stop_s STOP →
  놓친 때부터 hand_lost_slow_s 까지 SLOW, SLOW 였으면(또는 환자 접촉 중) 그때까지 SLOW (hand_lost_near). 손은 그리퍼(=카메라)에
  가까울 때 잘 놓치므로 바로 NORMAL 로 풀지 않는다.
- 입력 끊김 (input_level, #14·#59): 손·힘·tcp_pose 를 받다가 *_timeout_s 넘게 끊기면 SLOW (hand_stale · wrench_stale ·
  tcp_stale). 손은 한 번도 받은 적이 없으면 경고 로그만 (손 노드 없이 로봇만 시험할 때 속도가 바뀌지 않게).
  힘·tcp_pose(required_inputs)는 시작 후 required_grace_s 넘게 한 번도 못 받으면 STOP (wrench_missing · tcp_missing, 받으면 풀림).
  힘(stale_stop_inputs)은 받다가 stale_stop_s 넘게 끊겨도 STOP (wrench_lost, 받으면 풀림) — 힘 한계 없이 반속으로 돌지 않게.
- 로봇 위치 tf (base_link → link_6): tf 의 header.stamp 가 tcp_timeout_s 넘게 오래됐으면(joint_states 멈춤 → tf 얼어붙음) 안 쓴다.
  tcp_pose 가 robot_tf_tcp_cover_s 안에 있었을 때만 쓰고(교차 검사 기준), 그동안 tcp_pose 끊김은 tcp_stale 이 아니다.
  tf 로 구한 TCP 로도 작업 공간을 검사한다 (tcp_pose 만 멈춰도 작업 공간 STOP 이 빠지지 않게 — 리뷰 #104).
- 개발용 손 STOP 스위치 (hand_stop_mode: stop | slow | off, 10/11 조정묵 요청 · 정정인 결정): 실기 동작 확인 중 손 근접 STOP 이 잦아서.
  slow = 손 근접 STOP 을 SLOW 로 낮춤 (단 hand_deep_intrusion — 선 너머 30 cm 넘게 — 은 STOP 유지, #160 10/13 정정인 결정)
  · off = 손 판정 안 함. 둘 다 **지금 보이는 손만** 본다 — 손 놓침 STOP·SLOW 유지(#95),
  끊긴 손의 마지막 거리 사용, 손 입력 끊김 SLOW 를 하지 않는다. 힘 한계·nudge·작업 공간·estop·음성·필수 입력은 그대로.
  safety.yaml·launch 기본은 slow (#158 — 10/11~10/12 개발 기간 off 종료), 노드 자체 기본(설정 파일 없이 띄울 때)은 stop.
  리허설 결과로 시연 값 slow/stop 확정. off 는 개발 시험 전용 (시연·환자 앞 금지).
  기동할 때 한 번 정하고 실행 중에는 못 바꾼다(read_only). stop 이 아니면 5 s 마다 경고 로그,
  reason 에 hand_stop_dev_off · hand_stop_dev_slow 를 붙인다 (NORMAL 이면 그것만, 다른 사유가 있으면 "사유,hand_stop_dev_…").
  YAML 은 따옴표 없는 off 를 거짓(false)으로 읽으므로 false 도 off 로 받는다 (-p hand_stop_mode:=off 그대로 기동되게).
손 거리·끊김에 의한 SLOW/STOP 은 매 주기 다시 판정한다 (래치 아님). 그 밖의 STOP 은 /rally/robot/reset_safety (Trigger) 로만 해제.
"""
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from rally_interfaces.msg import HandState, SafetyStatus, VoiceIntent


def quat_angle_deg(q1, q2):
    """두 쿼터니언 (x, y, z, w) 사이 회전각 [deg]. 단위 쿼터니언이 아니어도 정규화해서 쓴다."""
    n1 = math.sqrt(sum(v * v for v in q1)) or 1.0
    n2 = math.sqrt(sum(v * v for v in q2)) or 1.0
    d = abs(sum(a * b for a, b in zip(q1, q2))) / (n1 * n2)
    return math.degrees(2.0 * math.acos(min(1.0, d)))


LEVEL_NAMES = {SafetyStatus.LEVEL_NORMAL: "NORMAL", SafetyStatus.LEVEL_SLOW: "SLOW", SafetyStatus.LEVEL_STOP: "STOP"}


def robot_segment(tcp, q, length):
    """로봇 끝 선분 (TCP, 플랜지). TCP = 손가락 끝, 플랜지 = TCP 에서 툴 z 축 반대로 length [m] (그리퍼 몸통·카메라 포함).
    tcp: (x, y, z) base_link [m], q: (x, y, z, w)."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    ez = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))   # 툴 z 축 (base_link)
    return tuple(tcp), tuple(t - length * e for t, e in zip(tcp, ez))


def tf_robot_segment(flange, q, length, tcp_pose_xyz, max_diff_m):
    """tf base_link → link_6(플랜지) 로 로봇 끝 선분 (TCP, 플랜지). 서비스 호출이 없어 이동 중에도 안 끊긴다.
    tcp_pose_xyz: 최근 /rally/robot/tcp_pose 위치 (없으면 None). tf 로 구한 TCP 가 이것과 max_diff_m 넘게 다르면 None —
    관절값이 끊기면 joint_state_publisher 가 팔을 0° 로 채워 tf 가 '똑바로 선 자세'를 낸다 (#77 정동휘). 이동 중 tcp_pose 지연(≤ 0.5 s)보다 크게."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    ez = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))   # link_6 z 축 = 툴 방향
    tcp = tuple(f + length * e for f, e in zip(flange, ez))
    if tcp_pose_xyz is not None and math.dist(tcp, tcp_pose_xyz) > max_diff_m:
        return None
    return tcp, tuple(flange)


def point_segment_dist(p, a, b):
    ab = [bb - aa for aa, bb in zip(a, b)]
    l2 = sum(v * v for v in ab)
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, sum((pp - aa) * v for pp, aa, v in zip(p, a, ab)) / l2))
    return math.dist(p, [aa + t * v for aa, v in zip(a, ab)])


def hand_robot_dist(landmarks, segment):
    """손 관절(유한한 것만) 중 로봇 선분에 가장 가까운 거리 [m]. 관절이 하나도 없으면 None (= 모름)."""
    ds = [point_segment_dist(p, *segment) for p in landmarks if all(math.isfinite(v) for v in p)]
    return min(ds) if ds else None


def hand_on_table(points, y_min, y_max):
    """손 관절 중 하나라도 테이블 y 범위 안이면 True. 관절을 모르면(빈 목록) True — 선 기준 판정을 그대로 한다."""
    ys = [p[1] for p in points if all(math.isfinite(v) for v in p)]
    return not ys or any(y_min <= y <= y_max for y in ys)


def hand_level(dist, patient_contact, cfg, robot_dist=None, on_table=True):
    """손 거리 → (레벨, 이유). 순수 함수 (test_safety.py).
    dist: None = 손 없음, NaN = 손은 보이는데 거리 모름, 숫자 = distance_to_centerline [m] (음수 = 로봇 구역 침범).
    patient_contact: 환자가 잡는 구간이고 손 메시지가 살아 있음 (끊겼으면 호출자가 False 로).
    robot_dist: 손 관절 ↔ 로봇 끝 선분 최소 거리 [m] (#98). None = 모름(tcp_pose 끊김 등) → 예전 선 기준 STOP(stop_distance_m).
    on_table: 손이 테이블 y 범위 안. 밖(옆 책상 사람 손)이면 선 기준 판정(깊이 STOP·선 SLOW)은 안 하고 로봇 거리로만 —
      distance_to_centerline 은 x 만 봐서, 테이블 옆 손도 '로봇 구역 깊이'로 계산된다 (10/8 실로봇 STOP 2회). 로봇 거리를 모르면 그대로 선 기준."""
    if dist is None:
        return SafetyStatus.LEVEL_NORMAL, ""
    if math.isnan(dist):
        return SafetyStatus.LEVEL_SLOW, "hand_distance_unknown"
    if robot_dist is not None and not on_table:
        if robot_dist < cfg["hand_stop_robot_m"]:
            return SafetyStatus.LEVEL_STOP, "hand_near_robot"
        if robot_dist < cfg["hand_slow_robot_m"]:
            return SafetyStatus.LEVEL_SLOW, "hand_near_robot"
        return SafetyStatus.LEVEL_NORMAL, ""
    if dist < cfg["hand_deep_stop_m"]:                                   # 로봇 구역 깊이 — 접촉 중이어도 STOP
        if dist < cfg.get("hand_deep_end_m", cfg["hand_deep_stop_m"]):
            return SafetyStatus.LEVEL_STOP, "hand_deep_intrusion"      # 아주 깊이 — 자동 재진행 안 함 (INTERFACES 9절)
        return SafetyStatus.LEVEL_STOP, "hand_near_centerline"         # 15~30 cm — 멈추되 손 빼면 이어 감
    if robot_dist is None:
        near, reason = dist < cfg["stop_distance_m"], "hand_near_centerline"
    else:
        near, reason = robot_dist < cfg["hand_stop_robot_m"], "hand_near_robot"
    if near:
        if patient_contact:
            return SafetyStatus.LEVEL_SLOW, "hand_near_patient_contact"
        return SafetyStatus.LEVEL_STOP, reason
    if robot_dist is not None and robot_dist < cfg["hand_slow_robot_m"]:
        return SafetyStatus.LEVEL_SLOW, "hand_near_robot"
    if dist < cfg["slow_distance_m"]:
        return SafetyStatus.LEVEL_SLOW, "hand_near_centerline"
    return SafetyStatus.LEVEL_NORMAL, ""


def hand_lost_level(lost_for_s, level_before, patient_contact, cfg):
    """가까운 구역에서 손을 놓쳤을 때 (#95) → (레벨, 이유). 손은 그리퍼(=카메라)에 가까울 때 잘 놓친다.
    level_before: 놓치기 직전 손 판정 레벨. STOP 이었으면 hand_lost_stop_s 동안 STOP, 그 뒤 놓친 때부터 hand_lost_slow_s 까지 SLOW.
    SLOW 였거나 환자 접촉 중이면 hand_lost_slow_s 까지 SLOW. 다시 보이면 호출자가 이 함수를 안 부른다."""
    if level_before == SafetyStatus.LEVEL_NORMAL or lost_for_s >= cfg["hand_lost_slow_s"]:
        return SafetyStatus.LEVEL_NORMAL, ""
    if level_before == SafetyStatus.LEVEL_STOP and not patient_contact and lost_for_s < cfg["hand_lost_stop_s"]:
        return SafetyStatus.LEVEL_STOP, "hand_lost_near"
    return SafetyStatus.LEVEL_SLOW, "hand_lost_near"


def hand_unknown_level(unknown_for_s, level_before, patient_contact, cfg):
    """손은 보이는데 거리 NaN → (레벨, 이유). 직전 판정이 STOP 이었으면 놓침과 같이 hand_lost_stop_s STOP (hand_lost_near),
    아니면 SLOW (hand_distance_unknown). 깊이는 카메라 30 cm 안에서 빠지므로(10/8 실측) 손이 로봇 STOP 구역에 들어오면
    NaN 이 되기 쉽다 — 그때 STOP 이 바로 SLOW 로 풀리면 안 된다 (리뷰 #104)."""
    lv = hand_lost_level(unknown_for_s, level_before, patient_contact, cfg)
    return lv if lv[0] == SafetyStatus.LEVEL_STOP else (SafetyStatus.LEVEL_SLOW, "hand_distance_unknown")


HAND_STOP_MODES = ("stop", "slow", "off")


def hand_mode_from_param(value):
    """파라미터 값 → (모드, 문제). YAML 1.1 은 따옴표 없는 off 를 false 로 읽는다 → off. 모르는 값은 stop (안전 쪽)."""
    if value is False:
        return "off", None
    mode = str(value).strip().lower()
    if mode in HAND_STOP_MODES:
        return mode, None
    return "stop", f"hand_stop_mode={value!r} 모름 → stop 으로 ({'|'.join(HAND_STOP_MODES)})"


def hand_mode_reason(reason, mode):
    """stop 이 아니면 reason 에 개발용 표시 (대시보드·DB 에 남게). 사유 문자열 자체는 앞에 그대로."""
    if mode == "stop":
        return reason
    tag = f"hand_stop_dev_{mode}"
    return f"{reason},{tag}" if reason else tag


def hand_mode_level(level_reason, mode):
    """hand_stop_mode 적용 (순수 함수). stop = 그대로, slow = STOP → SLOW (사유 유지), off = NORMAL.
    slow 라도 hand_deep_intrusion(선 너머 hand_deep_end_m 보다 깊이 — 손이 로봇 쪽 테이블 위)은 STOP 그대로 (#160)."""
    level, reason = level_reason
    if mode == "off":
        return SafetyStatus.LEVEL_NORMAL, ""
    if mode == "slow" and level == SafetyStatus.LEVEL_STOP and reason != "hand_deep_intrusion":
        return SafetyStatus.LEVEL_SLOW, reason
    return level, reason


def in_workspace(xyz, lim):
    """lim: {"x": (min, max), "y": ..., "z": ...} [m]."""
    return all(lim[a][0] <= v <= lim[a][1] for a, v in zip("xyz", xyz))


def input_level(last_rx, now, timeouts, required=(), since_start=0.0, grace_s=math.inf, stale_stop=(), stale_stop_s=math.inf):
    """입력 끊김 → (레벨, 이유). last_rx: {이름: 마지막 수신 monotonic 또는 None(한 번도 못 받음)}.
    required 입력(wrench·tcp)을 시작 후 grace_s 넘게 한 번도 못 받았으면 STOP (<이름>_missing, 래치 아님 — 받으면 풀림).
    10/8 실로봇: force_publisher 가 두산 서비스에서 멈춘 채 떠 있어 힘 한계·nudge·작업 공간 판정 없이 세션이 돌았다.
    stale_stop 입력(wrench)은 받다가 stale_stop_s 넘게 끊겨도 STOP (<이름>_lost, 래치 아님) — 세션 중 force_publisher 가 멈추면
    힘 한계 없이 반속으로 계속 돌던 것 (리뷰 #104).
    그 밖에 한 번도 못 받은 입력(손)은 여기서 판정하지 않는다 (경고 로그는 노드가). 받다가 끊긴 것은 SLOW."""
    if since_start > grace_s:
        for name in required:
            if last_rx.get(name) is None:
                return SafetyStatus.LEVEL_STOP, f"{name}_missing"
    for name in stale_stop:
        t = last_rx.get(name)
        if t is not None and now - t > stale_stop_s:
            return SafetyStatus.LEVEL_STOP, f"{name}_lost"
    for name, t in last_rx.items():
        if t is not None and now - t > timeouts[name]:
            return SafetyStatus.LEVEL_SLOW, f"{name}_stale"
    return SafetyStatus.LEVEL_NORMAL, ""


def workspace_x_max_for(base_m, extended_m, extend_on):
    """작업 공간 x 상한 [m]. 확장 신호(/rally/safety/workspace_extend, 모드 3 공 떼기 → 박스)가 켜진 동안만 extended_m
    (0 이하면 확장 없음). 평소·신호 끊김·workspace_extend_max_s 초과면 base_m. 확장은 x 상한만 — y·z·손 근접·힘 한계는 그대로."""
    if extend_on and float(extended_m) > 0.0:
        return max(float(base_m), float(extended_m))
    return float(base_m)


def force_limit_for(base_n, catch_n, catch_window_on):
    """지금 힘 한계 [N] (순수 함수). 받기 창이면 catch_n — 단 base 보다 낮게는 안 함 (설정 실수로 한계가 내려가 STOP 이 잦아지지 않게만)."""
    return max(float(base_n), float(catch_n)) if catch_window_on else float(base_n)


def worst(*levels):
    """(레벨, 이유) 중 가장 심한 것. 같으면 앞의 것."""
    best = (SafetyStatus.LEVEL_NORMAL, "")
    for lv in levels:
        if lv[0] > best[0]:
            best = lv
    return best


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("rally_safety_monitor")
        self.declare_parameter("slow_distance_m", 0.25)
        self.declare_parameter("stop_distance_m", 0.15)
        self.declare_parameter("slow_override", 0.5)
        self.declare_parameter("force_limit_n", 25.0)
        self.declare_parameter("catch_window_force_limit_n", 25.0)   # 모드 3 받기 창에서만 (#144). 기본은 평소와 같음 — safety.yaml 에서 정함
        self.declare_parameter("catch_window_max_s", 2.0)            # 받기 창 한 번의 최대 길이. 넘으면 신호가 와도 평소 한계 (#165 리뷰)
        self.declare_parameter("nudge_dfdt_n", 8.0)          # wrench 한 샘플(50 Hz = 20 ms) 사이 |F| 변화량 [N]. scripts/measure_wrench.py 로 확정
        self.declare_parameter("nudge_speed_max_mps", 0.02)  # 이보다 빨리 움직이는 중에는 nudge 판정 안 함 (가속 관성 오탐, CP7)
        self.declare_parameter("nudge_ang_speed_max_dps", 5.0)  # 툴이 이보다 빨리 도는 중에도 nudge 판정 안 함 (회전 중 힘 출렁임)
        self.declare_parameter("nudge_settle_s", 0.5)        # 멈춘 뒤 이 시간까지도 nudge 판정 안 함 (감속 관성 힘이 풀리는 구간)
        self.declare_parameter("nudge_joint_speed_max_dps", 1.0)   # 관절 하나라도 이보다 빠르면 움직이는 중 (/dsr01/joint_states 100 Hz)
        self.declare_parameter("joint_states_topic", "/dsr01/joint_states")
        self.declare_parameter("contact_timeout_s", 0.5)     # contact_expected 하트비트 유효 시간
        self.declare_parameter("hand_stop_mode", "stop", ParameterDescriptor(   # 개발용: stop | slow | off — 모듈 docstring
            read_only=True, dynamic_typing=True,                                # off 가 YAML 에서 false 로 와도 기동 (hand_mode_from_param)
            description="손 근접 STOP: stop | slow | off. 실행 중 변경 불가. 시연·환자 앞에서는 slow 또는 stop (off 는 개발 시험 전용)"))
        # 손 거리 완화 (#59) — 값은 safety.yaml
        self.declare_parameter("hand_deep_end_m", -0.30)             # 이보다 깊으면 사유 hand_deep_intrusion (세션 종료). STOP 기준은 아님
        self.declare_parameter("hand_deep_stop_m", -0.15)            # 이보다 로봇 구역 깊이 들어오면 환자 접촉 중이어도 STOP
        self.declare_parameter("patient_contact_timeout_s", 0.5)     # /rally/safety/patient_contact 하트비트 유효 시간
        # 손 ↔ 로봇 거리 (#98) · 손 놓침 (#95) — 값은 safety.yaml
        self.declare_parameter("hand_stop_robot_m", 0.20)            # 손 관절 ↔ 로봇 끝 선분 이 안이면 STOP
        self.declare_parameter("hand_slow_robot_m", 0.35)            # 이 안이면 SLOW
        self.declare_parameter("robot_segment_m", 0.228)             # TCP(손가락 끝) → 플랜지 길이 (펜던트 툴 오프셋)
        self.declare_parameter("hand_lost_stop_s", 1.0)
        self.declare_parameter("hand_lost_slow_s", 3.0)
        self.declare_parameter("table_y_min_m", -0.40)              # 테이블 y 범위 — 밖의 손은 선 기준 판정 안 함 (로봇 거리로만)
        self.declare_parameter("table_y_max_m", 0.40)
        # 로봇 위치를 tf 로도 (tcp_pose 는 get_current_posx 서비스라 이동 중 0.5 s 넘게 끊김, 10/8 실로봇)
        self.declare_parameter("robot_tf_enabled", True)
        self.declare_parameter("robot_tf_frame", "link_6")
        self.declare_parameter("robot_tf_max_diff_m", 0.20)       # tf TCP 와 tcp_pose 가 이보다 다르면 tf 를 안 씀
        self.declare_parameter("robot_tf_tcp_cover_s", 2.0)       # tcp_pose 가 이 시간 안에 있었을 때만 tf 를 씀 (그 사이 끊김은 tcp_stale 아님)
        # 입력 끊김 (#14·#59)
        self.declare_parameter("hand_timeout_s", 0.5)
        self.declare_parameter("wrench_timeout_s", 0.5)
        self.declare_parameter("tcp_timeout_s", 0.5)
        self.declare_parameter("never_received_warn_s", 5.0)        # 시작 후 이 시간 동안 한 번도 못 받은 입력은 경고 로그
        self.declare_parameter("required_inputs", ["wrench", "tcp"])   # 이 시간 넘게 한 번도 못 받으면 STOP (<이름>_missing)
        self.declare_parameter("required_grace_s", 5.0)
        self.declare_parameter("stale_stop_inputs", ["wrench"])     # 받다가 stale_stop_s 넘게 끊기면 STOP (<이름>_lost)
        self.declare_parameter("stale_stop_s", 2.0)
        for k, v in dict(x_min=0.25, x_max=0.80, y_min=-0.45, y_max=0.45, z_min=0.0, z_max=0.60).items():
            self.declare_parameter(f"workspace_{k}_m", v)
        self.declare_parameter("workspace_enabled", True)
        # 모드 3 공 떼기 → 박스 놓기 전용 (10/14 정정인): 스킬이 /rally/safety/workspace_extend 를 켠 동안만 x 상한을 넓힘. 값은 safety.yaml
        self.declare_parameter("workspace_x_max_extended_m", 0.0)   # 0 = 확장 없음 (기본). 받기 창(catch_window)과 같은 하트비트 방식
        self.declare_parameter("workspace_extend_max_s", 15.0)      # 한 번 켠 뒤 이 시간 넘으면 신호가 와도 평소 상한 (스킬 스레드 멈춤 대비)
        self.declare_parameter("return_speed_mm_s", 30.0)

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        reliable1 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(SafetyStatus, "/rally/safety/status", reliable)
        self.create_subscription(HandState, "/rally/perception/hand", self.on_hand, qos_profile_sensor_data)
        self.create_subscription(VoiceIntent, "/rally/voice/intent", self.on_intent, reliable)
        self.create_subscription(Bool, "/rally/safety/estop", self.on_estop, latched)
        self.create_subscription(Bool, "/rally/safety/contact_expected", self.on_contact, reliable1)
        self.create_subscription(Bool, "/rally/safety/catch_window", self.on_catch_window, reliable1)
        self.create_subscription(Bool, "/rally/safety/workspace_extend", self.on_workspace_extend, reliable1)
        self.create_subscription(Bool, "/rally/safety/patient_contact", self.on_patient_contact, reliable1)
        self.create_subscription(WrenchStamped, "/rally/robot/wrench", self.on_wrench, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/rally/robot/tcp_pose", self.on_tcp, qos_profile_sensor_data)
        self.create_subscription(JointState, self.p("joint_states_topic"), self.on_joints, qos_profile_sensor_data)
        self.create_service(Trigger, "/rally/robot/reset_safety", self.on_reset)

        self.level, self.reason, self.override = SafetyStatus.LEVEL_NORMAL, "", 1.0
        self.hand_dist = None         # None = 손 없음, NaN = 보이는데 거리 모름
        self.hand_points = []         # 손 관절 base_link (x, y, z). base_link 가 아니면 비움 (→ 로봇 거리 모름)
        self.robot_seg = None         # (TCP, 플랜지) base_link — tcp_pose 로
        self.robot_seg_tf = None      # 같은 것 — tf 로 (이동 중에도 안 끊김)
        self.tf_rx = None             # 마지막으로 쓴 tf 값의 시각 (monotonic 으로 환산한 stamp)
        self._tf_bad_logged = False
        self._tf_old_logged = False
        if self.p("robot_tf_enabled"):
            import tf2_ros
            self.tf_buf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
            self.create_timer(0.02, self.on_tf_timer)
        self.hand_level_before = SafetyStatus.LEVEL_NORMAL   # 손이 보일 때 마지막 손 판정 레벨 (놓침 판정용)
        self.hand_lost_t = None       # 손을 놓친 monotonic (보이면 None)
        self.hand_unknown_t = None    # 손은 보이는데 거리 NaN 이 시작된 monotonic (아니면 None)
        self.last_rx = {"hand": None, "wrench": None, "tcp": None}   # 마지막 수신 monotonic (None = 한 번도 못 받음)
        self.t_start = time.monotonic()
        self._mode_warn_t = -math.inf
        self._hand_mode, problem = hand_mode_from_param(self.p("hand_stop_mode"))
        if problem:
            self.get_logger().error(problem)
        if self._hand_mode == "stop":
            self.get_logger().info("hand_stop_mode=stop (손 근접 STOP 켜짐)")
        else:
            self.get_logger().warning(f"★ hand_stop_mode={self._hand_mode} — "
                                      + ("개발 시험 전용. 손 판정 전체 꺼짐" if self._hand_mode == "off"
                                         else "손 근접은 SLOW 만 (선 너머 깊은 침입 hand_deep_intrusion 은 STOP)")
                                      + ". 남는 STOP: 힘 한계·nudge·작업 공간·estop·음성·필수 입력"
                                      + (". 시연·환자 앞 금지 — slow 또는 stop 으로" if self._hand_mode == "off" else ""))
        # 시뮬(RALLY_SIM=1)은 에뮬레이터 툴 힘이 0 이라 force_publisher 가 발행을 건너뛸 수 있다 → 필수·끊김 STOP 검사 안 함 (경고 로그만)
        sim = os.environ.get("RALLY_SIM") == "1"
        self.required = [] if sim else list(self.p("required_inputs"))
        self.stale_stop = [] if sim else list(self.p("stale_stop_inputs"))
        self._warned_never = set()
        self._last_logged = (SafetyStatus.LEVEL_NORMAL, "")
        self._last_log_t = 0.0
        self.latched_stop = False
        self.contact_until = 0.0
        self.catch_until = 0.0
        self.catch_start = 0.0                              # 이번 받기 창이 열린 monotonic (catch_window_max_s 기준)
        self._catch_limit_on = False                        # 로그용 — 130 N ↔ 25 N 바뀔 때만 (#144 ③ 25 N 복귀 확인)
        self.extend_until = 0.0                             # 작업 공간 확장 하트비트 유효 시각 (workspace_extend)
        self.extend_start = 0.0
        self._extend_on = False                             # 로그용 — 상한이 바뀔 때만
        self.patient_contact_until = 0.0
        self.prev_fmag = None
        self.prev_tcp = None          # (t, x, y, z, qx, qy, qz, qw)
        self.tcp_speed = 0.0
        self.tcp_ang_speed = 0.0      # deg/s
        self.last_moving_t = None     # 마지막으로 '움직이는 중'이었던 monotonic
        self.joint_moving_t = None    # 관절 속도로 마지막으로 움직인 monotonic
        self.create_timer(0.01, self.tick)

    def p(self, name):
        return self.get_parameter(name).value

    def _stop(self, reason):
        if not self.latched_stop:
            self.get_logger().error(f"STOP: {reason}")
        self.latched_stop = True
        self.level, self.reason, self.override = SafetyStatus.LEVEL_STOP, reason, 0.0

    # ----- 입력 -----
    def on_hand(self, m):
        now = time.monotonic()
        self.last_rx["hand"] = now
        if not m.detected and self.hand_dist is not None:
            self.hand_lost_t = self.hand_unknown_t or now   # 보이다가 놓침. NaN 이던 때부터 셈 (STOP 유지를 늘리지 않게)
        elif m.detected:
            self.hand_lost_t = None
        unknown = m.detected and math.isnan(m.distance_to_centerline)
        self.hand_unknown_t = (self.hand_unknown_t or now) if unknown else None
        self.hand_dist = float(m.distance_to_centerline) if m.detected else None
        self.hand_points = ([(p.x, p.y, p.z) for p in m.landmarks]
                            if m.detected and m.header.frame_id == "base_link" else [])

    def on_intent(self, m):
        if m.is_emergency:
            self._stop("voice_stop")

    def on_estop(self, m):
        if m.data:
            self._stop("estop")

    def on_contact(self, m):
        self.contact_until = time.monotonic() + self.p("contact_timeout_s") if m.data else 0.0

    def contact_expected(self):
        return time.monotonic() < self.contact_until

    def on_catch_window(self, m):
        now = time.monotonic()
        if not m.data:
            self.catch_until = 0.0
            return
        if now >= self.catch_until:                         # 새 창 (꺼져 있었거나 하트비트가 끊긴 뒤)
            self.catch_start = now
        self.catch_until = now + self.p("contact_timeout_s")

    def catch_window_overdue(self):
        """창 신호는 살아 있는데 catch_window_max_s 를 넘김 (skill 스레드 멈춤 등)."""
        now = time.monotonic()
        return now < self.catch_until and now - self.catch_start > self.p("catch_window_max_s")

    def catch_window(self):
        return time.monotonic() < self.catch_until and not self.catch_window_overdue() and self.contact_expected()

    def on_workspace_extend(self, m):
        now = time.monotonic()
        if not m.data:
            self.extend_until = 0.0
            return
        if now >= self.extend_until:                        # 새로 켬 (꺼져 있었거나 하트비트가 끊긴 뒤)
            self.extend_start = now
        self.extend_until = now + self.p("contact_timeout_s")

    def workspace_extend_overdue(self):
        now = time.monotonic()
        return now < self.extend_until and now - self.extend_start > self.p("workspace_extend_max_s")

    def workspace_extend(self):
        return time.monotonic() < self.extend_until and not self.workspace_extend_overdue()

    def workspace_x_max_now(self):
        on = self.workspace_extend()
        x_max = workspace_x_max_for(self.p("workspace_x_max_m"), self.p("workspace_x_max_extended_m"), on)
        if on != self._extend_on:
            self._extend_on = on
            why = ("모드 3 공 떼기 → 박스" if on else
                   f"확장 {self.p('workspace_extend_max_s'):.0f} s 초과 → 평소 값 (신호는 계속 옴)" if self.workspace_extend_overdue() else
                   "확장 끝 → 평소 값")
            self.get_logger().warning(f"작업 공간 x 상한 {x_max:.2f} m ({why})")
        return x_max

    def force_limit_now(self):
        on = self.catch_window()
        lim = force_limit_for(self.p("force_limit_n"), self.p("catch_window_force_limit_n"), on)
        if on != self._catch_limit_on:
            self._catch_limit_on = on
            why = ("모드 3 받기 창 — #144" if on else
                   f"받기 창 {self.p('catch_window_max_s'):.1f} s 초과 → 평소 값 (신호는 계속 옴)" if self.catch_window_overdue() else
                   "받기 창 끝 → 평소 값")
            self.get_logger().warning(f"힘 한계 {lim:.0f} N ({why})")
        return lim

    def on_patient_contact(self, m):
        self.patient_contact_until = time.monotonic() + self.p("patient_contact_timeout_s") if m.data else 0.0

    def patient_contact(self):
        return time.monotonic() < self.patient_contact_until

    def on_tcp(self, m):
        now = time.monotonic()
        self.last_rx["tcp"] = now
        x, y, z = m.pose.position.x, m.pose.position.y, m.pose.position.z
        o = m.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        if self.prev_tcp is not None:
            dt = now - self.prev_tcp[0]
            # 끊겼다 돌아온 첫 샘플은 dt 가 끊긴 시간 전체라 속도가 과소평가된다 → 이번엔 갱신하지 않고 다음 샘플부터
            if 1e-3 < dt <= self.p("tcp_timeout_s"):
                self.tcp_speed = math.dist((x, y, z), self.prev_tcp[1:4]) / dt
                self.tcp_ang_speed = quat_angle_deg(self.prev_tcp[4:], q) / dt
        self.prev_tcp = (now, x, y, z, *q)
        self.robot_seg = robot_segment((x, y, z), q, self.p("robot_segment_m"))
        self.check_workspace((x, y, z))

    def check_workspace(self, xyz):
        if not self.p("workspace_enabled"):
            return
        lim = {a: (self.p(f"workspace_{a}_min_m"), self.p(f"workspace_{a}_max_m")) for a in "xyz"}
        lim["x"] = (lim["x"][0], self.workspace_x_max_now())      # 공 떼기 → 박스 동안만 넓힘 (workspace_extend)
        if not in_workspace(xyz, lim):
            self._stop("workspace_limit")

    def tcp_fresh(self, now):
        t = self.last_rx["tcp"]
        return t is not None and now - t <= self.p("tcp_timeout_s")

    def on_tf_timer(self):
        try:
            tr = self.tf_buf.lookup_transform("base_link", self.p("robot_tf_frame"), Time())
        except Exception:
            return
        # lookup_transform(Time()) 은 버퍼의 마지막 값을 계속 준다 → joint_states 가 멈추면 tf 가 얼어붙어도 조회는 성공한다.
        # 신선도는 조회 시각이 아니라 tf stamp 로 (10/10 실로봇 bag: link_6 stamp ↔ 수신 0.3 ms, 최대 4 ms)
        age = (self.get_clock().now() - Time.from_msg(tr.header.stamp)).nanoseconds * 1e-9
        if age > self.p("tcp_timeout_s"):
            if not self._tf_old_logged:
                self.get_logger().error(f"tf {self.p('robot_tf_frame')} 가 {age:.1f} s 전 값 (joint_states 끊김?) — tf 를 안 씀")
            self._tf_old_logged = True
            return
        self._tf_old_logged = False
        now = time.monotonic()
        # 교차 검사 기준 tcp_pose 가 없으면 tf 를 안 쓴다 (관절값 0 으로 채운 tf 를 걸러낼 수 없음)
        if self.prev_tcp is None or now - self.prev_tcp[0] > self.p("robot_tf_tcp_cover_s"):
            return
        t, r = tr.transform.translation, tr.transform.rotation
        seg = tf_robot_segment((t.x, t.y, t.z), (r.x, r.y, r.z, r.w), self.p("robot_segment_m"), self.prev_tcp[1:4],
                               self.p("robot_tf_max_diff_m"))
        if seg is None:
            if not self._tf_bad_logged:
                self.get_logger().error("tf 로봇 위치가 tcp_pose 와 크게 다름 (관절값 끊김?) — tf 를 안 쓰고 tcp_pose 로")
            self._tf_bad_logged = True
            return
        self._tf_bad_logged = False
        self.robot_seg_tf, self.tf_rx = seg, now - age     # tf 값의 시각 (조회 시각이 아니라) — 얼어붙으면 tcp_timeout_s 안에 끊김
        self.check_workspace(seg[0])             # tcp_pose 만 멈춰도 작업 공간 STOP 이 빠지지 않게 (리뷰 #104)

    def tf_fresh(self, now):
        return self.tf_rx is not None and now - self.tf_rx <= self.p("tcp_timeout_s")

    def on_wrench(self, m):
        self.last_rx["wrench"] = time.monotonic()
        f = math.sqrt(m.wrench.force.x ** 2 + m.wrench.force.y ** 2 + m.wrench.force.z ** 2)
        if f > self.force_limit_now():
            self._stop("force_limit")                       # 접촉 예정이어도 절대 한계는 항상 (모드 3 받기 창만 130 N, #144)
        if self.prev_fmag is not None and not self.contact_expected() and not self.moving():
            if abs(f - self.prev_fmag) > self.p("nudge_dfdt_n"):
                self._stop("nudge")
        self.prev_fmag = f

    def on_joints(self, m):
        lim = math.radians(self.p("nudge_joint_speed_max_dps"))
        if any(abs(v) > lim for v in m.velocity if math.isfinite(v)):
            self.joint_moving_t = time.monotonic()

    def moving(self):
        """nudge 판정을 쉬는 '움직이는 중' — 직선 속도 또는 툴 회전 속도, 그리고 멈춘 뒤 nudge_settle_s 까지.
        10/8 실로봇: 손 보기 자세에 도착해 멈춘 직후 감속 관성 힘(19 N)이 한 샘플에 2.5 N 으로 풀려 nudge STOP (아무도 안 건드림).
        두산 툴 힘은 관절 토크 추정이고 실제 갱신은 10 Hz 라 변화가 한 샘플에 몰린다.
        출발 순간도 같은 문제(10 Hz tcp_pose 로는 아직 '정지' → 가속 관성 힘 1.3 → 6.8 N 에 nudge STOP) → 관절 속도(100 Hz)로도 본다."""
        now = time.monotonic()
        joint_moving = self.joint_moving_t is not None and now - self.joint_moving_t < 0.05   # 100 Hz 두세 샘플
        # tcp_pose 가 끊기면 마지막 속도가 남아 '움직이는 중'으로 굳는다 → nudge 가 표시 없이 꺼짐 (리뷰 #104). 끊긴 동안은 관절 속도로만
        tcp_moving = self.tcp_fresh(now) and (self.tcp_speed >= self.p("nudge_speed_max_mps")
                                              or self.tcp_ang_speed >= self.p("nudge_ang_speed_max_dps"))
        if joint_moving or tcp_moving:
            self.last_moving_t = now
            return True
        return self.last_moving_t is not None and now - self.last_moving_t < self.p("nudge_settle_s")

    def on_reset(self, req, res):
        self.latched_stop = False
        self.prev_fmag = None
        self.last_moving_t = None
        res.success, res.message = True, "safety reset"
        self.get_logger().warning("safety reset (운영자)")
        return res

    # ----- 100 Hz -----
    def cfg(self):
        return {k: self.p(k) for k in ("slow_distance_m", "stop_distance_m", "hand_deep_stop_m", "hand_deep_end_m", "hand_stop_robot_m",
                                       "hand_slow_robot_m", "hand_lost_stop_s", "hand_lost_slow_s")}

    def robot_dist(self, now):
        """손 관절 ↔ 로봇 끝 선분. tf(우선)·tcp_pose 둘 다 끊겼거나 손 관절이 base_link 가 아니면 None (선 기준으로)."""
        if self.tf_fresh(now):
            return hand_robot_dist(self.hand_points, self.robot_seg_tf)
        if self.robot_seg is None or not self.tcp_fresh(now):
            return None
        return hand_robot_dist(self.hand_points, self.robot_seg)

    def hand_judge(self, now, relax, cfg):
        if self.hand_dist is None and self.hand_lost_t is not None:
            return hand_lost_level(now - self.hand_lost_t, self.hand_level_before, relax, cfg)
        if self.hand_unknown_t is not None:      # 보이는데 거리 NaN — 직전 레벨(hand_level_before)은 갱신하지 않는다
            return hand_unknown_level(now - self.hand_unknown_t, self.hand_level_before, relax, cfg)
        on_table = hand_on_table(self.hand_points, self.p("table_y_min_m"), self.p("table_y_max_m"))
        level = hand_level(self.hand_dist, relax, cfg, self.robot_dist(now), on_table)
        if self.hand_dist is not None:
            self.hand_level_before = level[0]
        return level

    def hand_mode(self):
        return self._hand_mode          # 기동 때 한 번 정함 (read_only)

    def hand_judge_visible(self, now, relax, cfg, mode):
        """개발용 slow/off: 지금 보이는(신선한) 손만 판정. 놓침·끊김은 손 없음과 같다 (직전 값을 들고 있지 않는다)."""
        if self.hand_dist is None or not self.hand_fresh(now):
            return SafetyStatus.LEVEL_NORMAL, ""
        on_table = hand_on_table(self.hand_points, self.p("table_y_min_m"), self.p("table_y_max_m"))
        return hand_mode_level(hand_level(self.hand_dist, relax, cfg, self.robot_dist(now), on_table), mode)

    def hand_fresh(self, now):
        t = self.last_rx["hand"]
        return t is not None and now - t <= self.p("hand_timeout_s")

    def _warn_never_received(self, now):
        if now - self.t_start < self.p("never_received_warn_s"):
            return
        for name, t in self.last_rx.items():
            if t is None and name not in self._warned_never:
                self._warned_never.add(name)
                self.get_logger().warning(f"{name} 입력을 아직 한 번도 못 받음 — 이 입력의 안전 판정 없이 동작 중")

    def _log_level(self, level, reason, now):
        """레벨이 바뀔 때 로그. STOP 은 항상, 나머지는 1 s 에 한 번까지 (깜빡임). 못 찍은 변화는 다음에 다시 시도."""
        if level == SafetyStatus.LEVEL_STOP:
            self.get_logger().error(f"안전 레벨 STOP ({reason})")
        elif now - self._last_log_t < 1.0:
            return
        elif level == SafetyStatus.LEVEL_SLOW:
            self.get_logger().warning(f"안전 레벨 SLOW ({reason})")
        else:
            self.get_logger().info("안전 레벨 NORMAL")
        self._last_logged, self._last_log_t = (level, reason), now

    def tick(self):
        now = time.monotonic()
        self._warn_never_received(now)
        if not self.latched_stop:
            timeouts = {k: self.p(f"{k}_timeout_s") for k in self.last_rx}
            relax = self.patient_contact() and self.hand_fresh(now)     # 손이 끊겼으면 마지막 거리로 완화하지 않는다
            rx = dict(self.last_rx)
            # 로봇 위치는 tf 로 알고 있음 → tcp_pose 짧은 끊김(robot_tf_tcp_cover_s 안)은 SLOW 아님. tf 는 cover 안에서만 갱신된다
            if self.tf_fresh(now) and rx["tcp"] is not None and now - rx["tcp"] <= self.p("robot_tf_tcp_cover_s"):
                rx["tcp"] = max(rx["tcp"], self.tf_rx)
            mode = self.hand_mode()
            if mode == "stop":
                hand = self.hand_judge(now, relax, self.cfg())
            else:
                hand = self.hand_judge_visible(now, relax, self.cfg(), mode)
                rx.pop("hand", None)                                    # 손 끊김 SLOW 도 안 함 (보이는 손만)
                if now - self._mode_warn_t >= 5.0:
                    self.get_logger().warning("손 판정 꺼짐 (hand_stop_mode=off) — 개발 시험 전용. 시연·환자 앞 금지" if mode == "off"
                                              else "손 근접 STOP → SLOW (hand_stop_mode=slow, 깊은 침입만 STOP)")
                    self._mode_warn_t = now
            level, reason = worst(hand,
                                  input_level(rx, now, timeouts, self.required, now - self.t_start, self.p("required_grace_s"),
                                              self.stale_stop, self.p("stale_stop_s")))
            override = {SafetyStatus.LEVEL_NORMAL: 1.0, SafetyStatus.LEVEL_SLOW: self.p("slow_override")}.get(level, 0.0)
            self.level, self.reason, self.override = level, reason, override
            if (level, reason) != self._last_logged:
                self._log_level(level, reason, now)
        msg = SafetyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        hand = 9.9 if self.hand_dist is None else self.hand_dist      # NaN = 보이는데 거리 모름 (그대로 싣는다)
        msg.level, msg.reason, msg.speed_override, msg.min_hand_distance = (
            self.level, hand_mode_reason(self.reason, self._hand_mode), self.override, float(hand))   # 래치 STOP 에도 개발용 표시
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    n = SafetyMonitor()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
