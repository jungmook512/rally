"""rally_robot/force_publisher.py — get_tool_force() → /rally/robot/wrench (WrenchStamped 100 Hz), get_current_posx() → /rally/robot/tcp_pose
담당: 정정인
주의: DSR_ROBOT2 는 init_dsr() 이후 import. 이 노드는 skill_server 와 같은 프로세스에 두거나(권장) 별도 프로세스면 자체 init_dsr() 필요.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
from rclpy.qos import qos_profile_sensor_data
from rally_robot.dsr_helpers import init_dsr


class ForcePublisher(Node):
    def __init__(self):
        super().__init__("rally_force_publisher")
        self.pub_w = self.create_publisher(WrenchStamped, "/rally/robot/wrench", qos_profile_sensor_data)
        self.pub_p = self.create_publisher(PoseStamped, "/rally/robot/tcp_pose", qos_profile_sensor_data)
        self.create_timer(0.01, self.tick)

    def tick(self):
        from DSR_ROBOT2 import get_tool_force, get_current_posx
        f = get_tool_force()            # [fx, fy, fz, tx, ty, tz]
        p = get_current_posx()[0]       # [x, y, z, rx, ry, rz]
        now = self.get_clock().now().to_msg()
        w = WrenchStamped(); w.header.stamp = now; w.header.frame_id = "base_link"
        w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = f[0], f[1], f[2]
        w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z = f[3], f[4], f[5]
        self.pub_w.publish(w)
        ps = PoseStamped(); ps.header = w.header
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p[0] / 1000.0, p[1] / 1000.0, p[2] / 1000.0
        self.pub_p.publish(ps)          # TODO(정정인): ZYZ → quaternion


def main(args=None):
    rclpy.init(args=args)
    dsr = init_dsr("rally_force_dsr")
    node = ForcePublisher()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(); ex.add_node(node); ex.add_node(dsr); ex.spin()
