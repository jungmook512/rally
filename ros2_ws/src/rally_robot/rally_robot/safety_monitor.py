"""rally_robot/safety_monitor.py — 100 Hz 안전 감시 → /rally/safety/status (Reliable)
담당: 정정인
입력: /rally/perception/hand (distance_to_centerline, gesture), /rally/voice/intent (is_emergency), /rally/safety/estop (Bool),
      /rally/robot/wrench (힘 한계·nudge), /rally/robot/tcp_pose (작업 공간·이동 속도), /rally/safety/contact_expected (skill_server)
규칙 (safety.yaml): 손 0.25 m → SLOW 0.5, 0.15 m → STOP / |F| > 25 N → STOP / nudge: 정지 중·접촉 예정 아님 && 한 주기 |Δ|F|| > 임계 → STOP
      / TCP 가 작업 공간 밖 → STOP / 주먹·"그만"·estop → STOP
- nudge 는 "접촉이 없어야 할 때 누가 로봇을 쳤다" 용이다. 스킬이 접촉 예정 플래그를 올린 동안(집기 하강·핸드오버 당김·저항 밀기)과
  로봇이 움직이는 동안(가속 관성)은 판정하지 않는다. 절대 힘 한계는 항상 적용한다.
  '움직이는 중' = TCP 직선 속도 ≥ nudge_speed_max_mps **또는** 툴 회전 속도 ≥ nudge_ang_speed_max_dps.
  (10/8 실로봇: 손끝은 거의 제자리에서 손목만 45° 도는 관절 이동 중 툴 힘이 출렁여 nudge STOP — 직선 속도만 봐서 '정지'로 오판)
- 접촉 예정 플래그는 contact_timeout_s(0.5 s) 안에 갱신되지 않으면 꺼진 것으로 본다 (skill_server 가 죽어도 감시 복귀).
- 손 거리 (hand_level, #59):
  손 없음 → NORMAL / 손은 보이는데 거리 NaN(깊이·tf 없음) → SLOW (hand_distance_unknown) / slow_distance_m 안 → SLOW /
  stop_distance_m 안 → STOP. 단 접촉 예정이고 로봇이 거의 멈춰 있거나 환자에게서 멀어지는 쪽(−x)으로만 움직이면 SLOW
  (hand_near_contact — 건네기 당김 대기·저항 밀기에서 환자가 물체·손잡이를 잡는 순간). 손이 로봇 구역 깊이
  (hand_deep_stop_m) 들어오면 접촉 예정이어도 STOP. tcp_pose 가 끊겼으면 움직임을 모르므로 완화하지 않는다.
- 입력 끊김 (input_level, #14·#59): 손·힘·tcp_pose 를 받다가 *_timeout_s 넘게 끊기면 SLOW (hand_stale · wrench_stale ·
  tcp_stale). 한 번도 받은 적이 없으면 경고 로그만 (손 노드 없이 로봇만 시험할 때 속도가 바뀌지 않게).
손 거리·끊김에 의한 SLOW/STOP 은 매 주기 다시 판정한다 (래치 아님). 그 밖의 STOP 은 /rally/robot/reset_safety (Trigger) 로만 해제.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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


def robot_still_or_retreating(speed, ang_speed, vx, cfg):
    """접촉 중 손 거리 완화 조건: 거의 정지, 또는 −x(환자에게서 멀어짐) 방향으로만 이동 (모드 7 져주기)."""
    if ang_speed >= cfg["contact_slow_ang_speed_max_dps"]:
        return False
    if speed < cfg["contact_slow_speed_max_mps"]:
        return True
    return vx < 0.0 and -vx >= cfg["contact_retreat_ratio"] * speed


def hand_level(dist, contact, motion, cfg):
    """손 거리 → (레벨, 이유). 순수 함수 (test_safety.py).
    dist: None = 손 없음, NaN = 손은 보이는데 거리 모름, 숫자 = distance_to_centerline [m] (음수 = 로봇 구역 침범).
    motion: None = tcp_pose 없음·끊김, 아니면 (speed m/s, ang_speed deg/s, vx m/s)."""
    if dist is None:
        return SafetyStatus.LEVEL_NORMAL, ""
    if math.isnan(dist):
        return SafetyStatus.LEVEL_SLOW, "hand_distance_unknown"
    if dist < cfg["stop_distance_m"]:
        relax = (contact and dist >= cfg["hand_deep_stop_m"] and motion is not None
                 and robot_still_or_retreating(*motion, cfg))
        if relax:
            return SafetyStatus.LEVEL_SLOW, "hand_near_contact"
        return SafetyStatus.LEVEL_STOP, "hand_near_centerline"
    if dist < cfg["slow_distance_m"]:
        return SafetyStatus.LEVEL_SLOW, "hand_near_centerline"
    return SafetyStatus.LEVEL_NORMAL, ""


def input_level(last_rx, now, timeouts):
    """입력 끊김 → (레벨, 이유). last_rx: {이름: 마지막 수신 monotonic 또는 None(한 번도 못 받음)}.
    받다가 끊긴 것만 SLOW — 한 번도 못 받은 입력은 여기서 판정하지 않는다 (경고 로그는 노드가)."""
    for name, t in last_rx.items():
        if t is not None and now - t > timeouts[name]:
            return SafetyStatus.LEVEL_SLOW, f"{name}_stale"
    return SafetyStatus.LEVEL_NORMAL, ""


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
        self.declare_parameter("nudge_dfdt_n", 8.0)          # wrench 한 샘플(50 Hz = 20 ms) 사이 |F| 변화량 [N]. scripts/measure_wrench.py 로 확정
        self.declare_parameter("nudge_speed_max_mps", 0.02)  # 이보다 빨리 움직이는 중에는 nudge 판정 안 함 (가속 관성 오탐, CP7)
        self.declare_parameter("nudge_ang_speed_max_dps", 5.0)  # 툴이 이보다 빨리 도는 중에도 nudge 판정 안 함 (회전 중 힘 출렁임)
        self.declare_parameter("contact_timeout_s", 0.5)     # contact_expected 하트비트 유효 시간
        # 손 거리 완화 (#59) — 값은 safety.yaml
        self.declare_parameter("hand_deep_stop_m", -0.15)            # 이보다 로봇 구역 깊이 들어오면 접촉 예정이어도 STOP
        self.declare_parameter("contact_slow_speed_max_mps", 0.03)   # 접촉 예정 + 이보다 느리면 손 거리 STOP → SLOW
        self.declare_parameter("contact_slow_ang_speed_max_dps", 5.0)
        self.declare_parameter("contact_retreat_ratio", 0.7)         # 속도의 이 비율 이상이 −x(환자에게서 멀어짐)면 '후퇴'
        # 입력 끊김 (#14·#59)
        self.declare_parameter("hand_timeout_s", 0.5)
        self.declare_parameter("wrench_timeout_s", 0.5)
        self.declare_parameter("tcp_timeout_s", 0.5)
        self.declare_parameter("never_received_warn_s", 5.0)        # 시작 후 이 시간 동안 한 번도 못 받은 입력은 경고 로그
        for k, v in dict(x_min=0.25, x_max=0.80, y_min=-0.45, y_max=0.45, z_min=0.0, z_max=0.60).items():
            self.declare_parameter(f"workspace_{k}_m", v)
        self.declare_parameter("workspace_enabled", True)
        self.declare_parameter("return_speed_mm_s", 30.0)

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        reliable1 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(SafetyStatus, "/rally/safety/status", reliable)
        self.create_subscription(HandState, "/rally/perception/hand", self.on_hand, qos_profile_sensor_data)
        self.create_subscription(VoiceIntent, "/rally/voice/intent", self.on_intent, reliable)
        self.create_subscription(Bool, "/rally/safety/estop", self.on_estop, latched)
        self.create_subscription(Bool, "/rally/safety/contact_expected", self.on_contact, reliable1)
        self.create_subscription(WrenchStamped, "/rally/robot/wrench", self.on_wrench, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/rally/robot/tcp_pose", self.on_tcp, qos_profile_sensor_data)
        self.create_service(Trigger, "/rally/robot/reset_safety", self.on_reset)

        self.level, self.reason, self.override = SafetyStatus.LEVEL_NORMAL, "", 1.0
        self.hand_dist = None         # None = 손 없음, NaN = 보이는데 거리 모름
        self.last_rx = {"hand": None, "wrench": None, "tcp": None}   # 마지막 수신 monotonic (None = 한 번도 못 받음)
        self.t_start = time.monotonic()
        self._warned_never = set()
        self._last_logged = (SafetyStatus.LEVEL_NORMAL, "")
        self.latched_stop = False
        self.contact_until = 0.0
        self.prev_fmag = None
        self.prev_tcp = None          # (t, x, y, z, qx, qy, qz, qw)
        self.tcp_speed = 0.0
        self.tcp_ang_speed = 0.0      # deg/s
        self.tcp_vx = 0.0             # m/s (base x — 환자 쪽이 +)
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
        self.last_rx["hand"] = time.monotonic()
        self.hand_dist = float(m.distance_to_centerline) if m.detected else None
        if m.detected and m.gesture == HandState.GESTURE_FIST:
            self._stop("fist_gesture")

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

    def on_tcp(self, m):
        now = time.monotonic()
        self.last_rx["tcp"] = now
        x, y, z = m.pose.position.x, m.pose.position.y, m.pose.position.z
        o = m.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        if self.prev_tcp is not None:
            dt = now - self.prev_tcp[0]
            if dt > 1e-3:
                self.tcp_speed = math.dist((x, y, z), self.prev_tcp[1:4]) / dt
                self.tcp_vx = (x - self.prev_tcp[1]) / dt
                self.tcp_ang_speed = quat_angle_deg(self.prev_tcp[4:], q) / dt
        self.prev_tcp = (now, x, y, z, *q)
        if self.p("workspace_enabled"):
            for axis, v in (("x", x), ("y", y), ("z", z)):
                if not self.p(f"workspace_{axis}_min_m") <= v <= self.p(f"workspace_{axis}_max_m"):
                    self._stop("workspace_limit")
                    break

    def on_wrench(self, m):
        self.last_rx["wrench"] = time.monotonic()
        f = math.sqrt(m.wrench.force.x ** 2 + m.wrench.force.y ** 2 + m.wrench.force.z ** 2)
        if f > self.p("force_limit_n"):
            self._stop("force_limit")                       # 접촉 예정이어도 절대 한계는 항상
        if self.prev_fmag is not None and not self.contact_expected() and not self.moving():
            if abs(f - self.prev_fmag) > self.p("nudge_dfdt_n"):
                self._stop("nudge")
        self.prev_fmag = f

    def moving(self):
        """nudge 판정을 쉬는 '움직이는 중' — 직선 속도 또는 툴 회전 속도."""
        return self.tcp_speed >= self.p("nudge_speed_max_mps") or self.tcp_ang_speed >= self.p("nudge_ang_speed_max_dps")

    def on_reset(self, req, res):
        self.latched_stop = False
        self.prev_fmag = None
        res.success, res.message = True, "safety reset"
        self.get_logger().warning("safety reset (운영자)")
        return res

    # ----- 100 Hz -----
    def cfg(self):
        keys = ("slow_distance_m", "stop_distance_m", "hand_deep_stop_m", "contact_slow_speed_max_mps",
                "contact_slow_ang_speed_max_dps", "contact_retreat_ratio")
        return {k: self.p(k) for k in keys}

    def motion(self, now):
        """(speed, ang_speed, vx) — tcp_pose 를 못 받았거나 끊겼으면 None (움직임 모름)."""
        t = self.last_rx["tcp"]
        if t is None or now - t > self.p("tcp_timeout_s"):
            return None
        return self.tcp_speed, self.tcp_ang_speed, self.tcp_vx

    def _warn_never_received(self, now):
        if now - self.t_start < self.p("never_received_warn_s"):
            return
        for name, t in self.last_rx.items():
            if t is None and name not in self._warned_never:
                self._warned_never.add(name)
                self.get_logger().warning(f"{name} 입력을 아직 한 번도 못 받음 — 이 입력의 안전 판정 없이 동작 중")

    def tick(self):
        now = time.monotonic()
        self._warn_never_received(now)
        if not self.latched_stop:
            timeouts = {k: self.p(f"{k}_timeout_s") for k in self.last_rx}
            level, reason = worst(hand_level(self.hand_dist, self.contact_expected(), self.motion(now), self.cfg()),
                                  input_level(self.last_rx, now, timeouts))
            override = {SafetyStatus.LEVEL_NORMAL: 1.0, SafetyStatus.LEVEL_SLOW: self.p("slow_override")}.get(level, 0.0)
            self.level, self.reason, self.override = level, reason, override
            if (level, reason) != self._last_logged:
                self._last_logged = (level, reason)
                # rclpy 로거는 같은 호출 위치에서 심각도를 바꾸면 ValueError → 호출을 나눈다
                if level == SafetyStatus.LEVEL_NORMAL:
                    self.get_logger().info("안전 레벨 NORMAL", throttle_duration_sec=1.0)
                else:
                    self.get_logger().warning(f"안전 레벨 {LEVEL_NAMES[level]} ({reason})", throttle_duration_sec=1.0)
        msg = SafetyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        hand = 9.9 if self.hand_dist is None else self.hand_dist      # NaN = 보이는데 거리 모름 (그대로 싣는다)
        msg.level, msg.reason, msg.speed_override, msg.min_hand_distance = self.level, self.reason, self.override, float(hand)
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
