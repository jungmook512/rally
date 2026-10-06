"""rally_robot/safety_monitor.py — 100 Hz 안전 감시 → /rally/safety/status (Reliable)
담당: 정정인
입력: /rally/perception/hand (distance_to_centerline, gesture), /rally/voice/intent (is_emergency), /rally/safety/estop (Bool),
      /rally/robot/wrench (힘 한계·nudge), /rally/robot/tcp_pose (작업 공간)
규칙 (safety.yaml): 손 0.25 m → SLOW 0.5, 0.15 m → STOP / 힘 25 N → STOP / nudge: |dF/dt| 급변 → STOP / 주먹·"그만"·estop → STOP
STOP 은 /rally/robot/reset_safety (Trigger) 로만 해제.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import WrenchStamped
from rally_interfaces.msg import SafetyStatus, HandState, VoiceIntent


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("rally_safety_monitor")
        self.declare_parameter("slow_distance_m", 0.25)
        self.declare_parameter("stop_distance_m", 0.15)
        self.declare_parameter("force_limit_n", 25.0)
        self.declare_parameter("nudge_dfdt_n", 8.0)
        self.declare_parameter("slow_override", 0.5)
        for k, v in dict(x_min=0.25, x_max=0.80, y_min=-0.45, y_max=0.45, z_min=0.0, z_max=0.60).items():
            self.declare_parameter(f"workspace_{k}_m", v)        # TODO(정정인): tcp_pose 로 작업 공간 감시에 사용
        self.declare_parameter("return_speed_mm_s", 30.0)
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(SafetyStatus, "/rally/safety/status", reliable)
        self.create_subscription(HandState, "/rally/perception/hand", self.on_hand, qos_profile_sensor_data)
        self.create_subscription(VoiceIntent, "/rally/voice/intent", self.on_intent, reliable)
        self.create_subscription(Bool, "/rally/safety/estop", self.on_estop, latched)
        self.create_subscription(WrenchStamped, "/rally/robot/wrench", self.on_wrench, qos_profile_sensor_data)
        self.create_service(Trigger, "/rally/robot/reset_safety", self.on_reset)
        self.level, self.reason, self.override, self.min_hand = SafetyStatus.LEVEL_NORMAL, "", 1.0, 9.9
        self.latched_stop = False
        self.create_timer(0.01, self.tick)

    def _stop(self, reason):
        self.latched_stop = True; self.level = SafetyStatus.LEVEL_STOP; self.reason = reason; self.override = 0.0

    def on_hand(self, m):
        self.min_hand = m.distance_to_centerline if m.detected else 9.9
        if m.detected and m.gesture == HandState.GESTURE_FIST: self._stop("fist_gesture")
    def on_intent(self, m):
        if m.is_emergency: self._stop("voice_stop")
    def on_estop(self, m):
        if m.data: self._stop("estop")
    def on_wrench(self, m):
        f = (m.wrench.force.x**2 + m.wrench.force.y**2 + m.wrench.force.z**2) ** 0.5
        if f > self.get_parameter("force_limit_n").value: self._stop("force_limit")
        # TODO(정정인): dF/dt 로 nudge 감지, tcp_pose 로 작업 공간(중앙선) 감시
    def on_reset(self, req, res):
        self.latched_stop = False; res.success = True; res.message = "safety reset"; return res

    def tick(self):
        if not self.latched_stop:
            slow = self.get_parameter("slow_distance_m").value; stop = self.get_parameter("stop_distance_m").value
            if self.min_hand < stop: self.level, self.reason, self.override = SafetyStatus.LEVEL_STOP, "hand_near_centerline", 0.0
            elif self.min_hand < slow: self.level, self.reason, self.override = SafetyStatus.LEVEL_SLOW, "hand_near_centerline", self.get_parameter("slow_override").value
            else: self.level, self.reason, self.override = SafetyStatus.LEVEL_NORMAL, "", 1.0
        msg = SafetyStatus(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.level, msg.reason, msg.speed_override, msg.min_hand_distance = self.level, self.reason, self.override, float(self.min_hand)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); n = SafetyMonitor(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
