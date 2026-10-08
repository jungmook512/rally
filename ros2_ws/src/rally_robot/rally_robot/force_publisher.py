"""rally_robot/force_publisher.py — get_tool_force() → /rally/robot/wrench (WrenchStamped 100 Hz), get_current_posx() → /rally/robot/tcp_pose
담당: 정정인
주의: DSR_ROBOT2 는 init_dsr() 이후 import. 이 노드는 skill_server 와 같은 프로세스에 두거나(권장) 별도 프로세스면 자체 init_dsr() 필요.
- 힘은 base 기준(get_tool_force 기본 ref=DR_BASE) → frame_id=base_link. N·Nm 그대로.
- 자세는 mm·ZYZ deg → m·quaternion (dsr_helpers.posx_to_pose). 변환은 여기서만.
- 로봇 미연결 등으로 DRL 호출이 실패하면 발행을 건너뛰고 5 s 에 한 번만 경고. get_tool_force 는 실패 시 0 여섯 개를 돌려주므로
  **전부 0 이면 실패로 보고 발행하지 않는다** (0 을 발행하면 안전 모니터의 힘 한계가 눈을 감는다).
- 두산 dsr_controller2 는 서비스 호출을 한 스레드에서 처리한다. 100 Hz × 서비스 2개 = 400 회/s 로 두드리면 motion·system 서비스까지
  무응답이 된다 (10/6 실측: force_publisher 2개 중복 실행 → 힘 0, 모션 서비스 멈춤). 그래서
  - 기본 50 Hz, tcp_pose 는 매 5번째 틱(10 Hz)만 서비스 호출
  - 시작할 때 /rally/robot/wrench 발행자가 이미 있으면 중복 실행으로 보고 종료
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
from rclpy.qos import qos_profile_sensor_data
from rally_robot.dsr_helpers import init_dsr, posx_to_pose


class ForcePublisher(Node):
    def __init__(self):
        super().__init__("rally_force_publisher")
        self.declare_parameter("rate_hz", 50.0)          # 서비스 폴링이라 100 Hz 는 컨트롤러를 막는다 (docstring)
        self.declare_parameter("pose_every_n", 5)        # tcp_pose 는 매 N 틱 (50 Hz / 5 = 10 Hz)
        self.pub_w = self.create_publisher(WrenchStamped, "/rally/robot/wrench", qos_profile_sensor_data)
        self.pub_p = self.create_publisher(PoseStamped, "/rally/robot/tcp_pose", qos_profile_sensor_data)
        self.duplicate = self.count_publishers("/rally/robot/wrench") > 1   # 나 말고 이미 있음
        self._n = 0
        self._last_pose = None
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)

    def tick(self):
        try:
            from DSR_ROBOT2 import get_tool_force, get_current_posx
            f = get_tool_force()            # [fx, fy, fz, tx, ty, tz]  N · Nm, base 기준 — 실패 시 [0]*6
            self._n += 1
            if self._n % int(self.get_parameter("pose_every_n").value) == 1 or self._last_pose is None:
                self._last_pose = get_current_posx()[0]   # [x, y, z, rx, ry, rz] mm · deg(ZYZ)
            p = self._last_pose
        except Exception as e:  # DSR 미연결·타임아웃 — 안전 감시가 이 토픽 끊김을 감지하도록 발행하지 않는다
            self.get_logger().warning(f"DRL 읽기 실패, 발행 생략: {e}", throttle_duration_sec=5.0)
            return
        if not any(abs(float(v)) > 1e-9 for v in f):
            self.get_logger().warning("get_tool_force 가 전부 0 — 발행 생략. ① 특이점 근처 자세면 컨트롤러가 툴 힘을 0 으로 낸다 → 관측 자세처럼 팔을 뻗은 자세로 옮기기 ② 중복 force_publisher·서비스 과부하 확인", throttle_duration_sec=5.0)
            return
        now = self.get_clock().now().to_msg()
        w = WrenchStamped()
        w.header.stamp = now
        w.header.frame_id = "base_link"
        w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = float(f[0]), float(f[1]), float(f[2])
        w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z = float(f[3]), float(f[4]), float(f[5])
        self.pub_w.publish(w)
        if self._n % int(self.get_parameter("pose_every_n").value) == 1:
            ps = PoseStamped()
            ps.header = w.header
            ps.pose = posx_to_pose(p)
            self.pub_p.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    dsr = init_dsr("rally_force_dsr")
    node = ForcePublisher()
    if node.duplicate:
        node.get_logger().fatal("/rally/robot/wrench 발행자가 이미 있음 — force_publisher 중복 실행. 이 인스턴스는 종료합니다 (컨트롤러 서비스 과부하 방지)")
        node.destroy_node()
        dsr.destroy_node()
        rclpy.shutdown()
        return 2
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
