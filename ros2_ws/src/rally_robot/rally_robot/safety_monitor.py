"""rally_robot/safety_monitor.py — 100 Hz 안전 감시 → /rally/safety/status (Reliable)
담당: 정정인
입력: /rally/perception/hand (distance_to_centerline, gesture), /rally/voice/intent (is_emergency), /rally/safety/estop (Bool),
      /rally/robot/wrench (힘 한계·nudge), /rally/robot/tcp_pose (작업 공간·이동 속도), /rally/safety/contact_expected (skill_server)
규칙 (safety.yaml): 손 0.25 m → SLOW 0.5, 0.15 m → STOP / |F| > 25 N → STOP / nudge: 정지 중·접촉 예정 아님 && 한 주기 |Δ|F|| > 임계 → STOP
      / TCP 가 작업 공간 밖 → STOP / 주먹·"그만"·estop → STOP
- nudge 는 "접촉이 없어야 할 때 누가 로봇을 쳤다" 용이다. 스킬이 접촉 예정 플래그를 올린 동안(집기 하강·핸드오버 당김·저항 밀기)과
  로봇이 움직이는 동안(가속 관성)은 판정하지 않는다. 절대 힘 한계는 항상 적용한다.
- 접촉 예정 플래그는 contact_timeout_s(0.5 s) 안에 갱신되지 않으면 꺼진 것으로 본다 (skill_server 가 죽어도 감시 복귀).
STOP 은 /rally/robot/reset_safety (Trigger) 로만 해제.
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


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("rally_safety_monitor")
        self.declare_parameter("slow_distance_m", 0.25)
        self.declare_parameter("stop_distance_m", 0.15)
        self.declare_parameter("slow_override", 0.5)
        self.declare_parameter("force_limit_n", 25.0)
        self.declare_parameter("nudge_dfdt_n", 8.0)          # 한 주기(10 ms) |F| 변화량 [N]. scripts/measure_wrench.py 로 확정
        self.declare_parameter("nudge_speed_max_mps", 0.02)  # 이보다 빨리 움직이는 중에는 nudge 판정 안 함 (가속 관성 오탐, CP7)
        self.declare_parameter("contact_timeout_s", 0.5)     # contact_expected 하트비트 유효 시간
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

        self.level, self.reason, self.override, self.min_hand = SafetyStatus.LEVEL_NORMAL, "", 1.0, 9.9
        self.latched_stop = False
        self.contact_until = 0.0
        self.prev_fmag = None
        self.prev_tcp = None          # (t, x, y, z)
        self.tcp_speed = 0.0
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
        self.min_hand = m.distance_to_centerline if m.detected else 9.9
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
        x, y, z = m.pose.position.x, m.pose.position.y, m.pose.position.z
        if self.prev_tcp is not None:
            dt = now - self.prev_tcp[0]
            if dt > 1e-3:
                self.tcp_speed = math.dist((x, y, z), self.prev_tcp[1:]) / dt
        self.prev_tcp = (now, x, y, z)
        if self.p("workspace_enabled"):
            for axis, v in (("x", x), ("y", y), ("z", z)):
                if not self.p(f"workspace_{axis}_min_m") <= v <= self.p(f"workspace_{axis}_max_m"):
                    self._stop("workspace_limit")
                    break

    def on_wrench(self, m):
        f = math.sqrt(m.wrench.force.x ** 2 + m.wrench.force.y ** 2 + m.wrench.force.z ** 2)
        if f > self.p("force_limit_n"):
            self._stop("force_limit")                       # 접촉 예정이어도 절대 한계는 항상
        if self.prev_fmag is not None and not self.contact_expected() and self.tcp_speed < self.p("nudge_speed_max_mps"):
            if abs(f - self.prev_fmag) > self.p("nudge_dfdt_n"):
                self._stop("nudge")
        self.prev_fmag = f

    def on_reset(self, req, res):
        self.latched_stop = False
        self.prev_fmag = None
        res.success, res.message = True, "safety reset"
        self.get_logger().warning("safety reset (운영자)")
        return res

    # ----- 100 Hz -----
    def tick(self):
        if not self.latched_stop:
            slow, stop = self.p("slow_distance_m"), self.p("stop_distance_m")
            if self.min_hand < stop:
                self.level, self.reason, self.override = SafetyStatus.LEVEL_STOP, "hand_near_centerline", 0.0
            elif self.min_hand < slow:
                self.level, self.reason, self.override = SafetyStatus.LEVEL_SLOW, "hand_near_centerline", self.p("slow_override")
            else:
                self.level, self.reason, self.override = SafetyStatus.LEVEL_NORMAL, "", 1.0
        msg = SafetyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.level, msg.reason, msg.speed_override, msg.min_hand_distance = self.level, self.reason, self.override, float(self.min_hand)
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
