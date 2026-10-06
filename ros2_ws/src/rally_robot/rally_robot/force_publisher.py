"""rally_robot/force_publisher.py — get_tool_force() → /rally/robot/wrench (WrenchStamped 100 Hz), get_current_posx() → /rally/robot/tcp_pose
담당: 정정인
주의: DSR_ROBOT2 는 init_dsr() 이후 import. 이 노드는 skill_server 와 같은 프로세스에 두거나(권장) 별도 프로세스면 자체 init_dsr() 필요.
- 힘은 base 기준(get_tool_force 기본 ref=DR_BASE) → frame_id=base_link. N·Nm 그대로.
- 자세는 mm·ZYZ deg → m·quaternion (dsr_helpers.posx_to_pose). 변환은 여기서만.
- 로봇 미연결 등으로 DRL 호출이 실패하면 발행을 건너뛰고 5 s 에 한 번만 경고.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
from rclpy.qos import qos_profile_sensor_data
from rally_robot.dsr_helpers import init_dsr, posx_to_pose


class ForcePublisher(Node):
    def __init__(self):
        super().__init__("rally_force_publisher")
        self.declare_parameter("rate_hz", 100.0)
        self.pub_w = self.create_publisher(WrenchStamped, "/rally/robot/wrench", qos_profile_sensor_data)
        self.pub_p = self.create_publisher(PoseStamped, "/rally/robot/tcp_pose", qos_profile_sensor_data)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)

    def tick(self):
        try:
            from DSR_ROBOT2 import get_tool_force, get_current_posx
            f = get_tool_force()            # [fx, fy, fz, tx, ty, tz]  N · Nm, base 기준
            p = get_current_posx()[0]       # [x, y, z, rx, ry, rz]     mm · deg(ZYZ)
        except Exception as e:  # DSR 미연결·타임아웃 — 안전 감시가 이 토픽 끊김을 감지하도록 발행하지 않는다
            self.get_logger().warning(f"DRL 읽기 실패, 발행 생략: {e}", throttle_duration_sec=5.0)
            return
        now = self.get_clock().now().to_msg()
        w = WrenchStamped()
        w.header.stamp = now
        w.header.frame_id = "base_link"
        w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = float(f[0]), float(f[1]), float(f[2])
        w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z = float(f[3]), float(f[4]), float(f[5])
        self.pub_w.publish(w)
        ps = PoseStamped()
        ps.header = w.header
        ps.pose = posx_to_pose(p)
        self.pub_p.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    dsr = init_dsr("rally_force_dsr")
    node = ForcePublisher()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    ex.add_node(dsr)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        dsr.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
