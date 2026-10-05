"""rally_robot/gripper_node.py — RG2 그리퍼 ROS 2 노드 (선생님 onrobot.RG 래핑)
담당: 정정인
- /rally/gripper/command (GripperCommand srv): width_mm, force_n → RG.move_gripper(width*10, force*10)
- /rally/gripper/state   (sensor_msgs/JointState, 20 Hz): position=[width_mm], effort=[force]
- object_detected = get_status()[1] (grip detected) 또는 목표 폭 대비 실제 폭 차이
TODO(정정인): Pymodbus 비동기 클라이언트(AsyncModbusTcpClient)로 교체해 이동 중 명령 가능하게
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rally_interfaces.srv import GripperCommand
from rally_robot.onrobot import RG
from rally_robot.dsr_helpers import GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT


class GripperNode(Node):
    def __init__(self):
        super().__init__("rally_gripper")
        self.rg = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        self.pub = self.create_publisher(JointState, "/rally/gripper/state", 10)
        self.create_service(GripperCommand, "/rally/gripper/command", self.on_cmd)
        self.create_timer(0.05, self.publish_state)

    def on_cmd(self, req, res):
        # TODO(정정인): RG.move_gripper(int(req.width_mm*10), int(req.force_n*10)), wait_until_done 처리
        res.success = False
        res.actual_width_mm = 0.0
        res.object_detected = False
        return res

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["rg2_width"]
        try:
            msg.position = [float(self.rg.get_width())] if self.rg.connected else [0.0]
        except Exception:
            msg.position = [0.0]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
