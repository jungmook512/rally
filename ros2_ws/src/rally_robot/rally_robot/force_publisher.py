"""rally_robot/force_publisher.py — 두산 get_tool_force → /rally/robot/wrench (WrenchStamped 50 Hz), get_current_posx → /rally/robot/tcp_pose (10 Hz)
담당: 정정인
- 힘은 base 기준(ref=DR_BASE) → frame_id=base_link. N·Nm 그대로.
- 자세는 mm·ZYZ deg → m·quaternion (dsr_helpers.posx_to_pose). 변환은 여기서만.
- get_tool_force 는 실패 시 0 여섯 개를 돌려주므로 **전부 0 이면 실패로 보고 발행하지 않는다** (0 을 발행하면 안전 모니터의 힘 한계가 눈을 감는다).
- 두산 dsr_controller2 는 서비스 호출을 한 스레드에서 처리한다. 100 Hz × 서비스 2개 = 400 회/s 로 두드리면 motion·system 서비스까지
  무응답이 된다 (10/6 실측: force_publisher 2개 중복 실행 → 힘 0, 모션 서비스 멈춤). 그래서
  - 힘 50 Hz, tcp_pose 10 Hz. 서비스마다 응답을 기다리는 요청은 하나만 (앞 요청이 안 끝났으면 이번 틱은 건너뜀)
  - 시작할 때 /rally/robot/wrench 발행자가 이미 있으면 중복 실행으로 보고 종료
- 10/8: DSR_ROBOT2 함수(get_tool_force 등)는 응답이 올 때까지 spin_until_future_complete 로 무한정 기다린다. 같은 노드를
  다른 executor 도 돌리고 있어 응답을 놓치면 영원히 멈췄다 (켤 때마다 될 때·안 될 때가 있었고, 멈추면 경고도 없이 힘·tcp_pose 가 0 개).
  → 서비스를 직접 비동기로 부르고 request_timeout_s 안에 응답이 없으면 버리고 다시 요청한다. 힘·자세 요청이 서로를 막지 않는다
  (예전엔 한 콜백에서 차례로 불러 이동 중 자세 호출이 느리면 tcp_pose 가 0.5 s 넘게 끊겼다).
"""
import time

import rclpy
from dsr_msgs2.srv import GetCurrentPosx, GetToolForce
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from rally_robot.dsr_helpers import posx_to_pose

DR_BASE = 0


class ForcePublisher(Node):
    def __init__(self):
        super().__init__("rally_force_publisher")
        self.declare_parameter("rate_hz", 50.0)              # 힘 요청 주기. 서비스 폴링이라 100 Hz 는 컨트롤러를 막는다 (docstring)
        self.declare_parameter("pose_rate_hz", 10.0)         # tcp_pose 요청 주기
        self.declare_parameter("request_timeout_s", 0.5)     # 이 안에 응답이 없으면 버리고 다시 요청
        self.declare_parameter("dsr_prefix", "/dsr01/dsr_controller2/")
        prefix = self.get_parameter("dsr_prefix").value
        self.pub_w = self.create_publisher(WrenchStamped, "/rally/robot/wrench", qos_profile_sensor_data)
        self.pub_p = self.create_publisher(PoseStamped, "/rally/robot/tcp_pose", qos_profile_sensor_data)
        self.duplicate = self.count_publishers("/rally/robot/wrench") > 1   # 나 말고 이미 있음
        self.cli_f = self.create_client(GetToolForce, prefix + "aux_control/get_tool_force")
        self.cli_p = self.create_client(GetCurrentPosx, prefix + "aux_control/get_current_posx")
        self.pending = {"force": None, "pose": None}         # 이름 → (future, 보낸 monotonic)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick_force)
        self.create_timer(1.0 / self.get_parameter("pose_rate_hz").value, self.tick_pose)

    def _send(self, name, client, req, on_result):
        """앞 요청이 아직이면 건너뜀(시간 초과면 버리고 새로). 응답은 on_result(response) 로."""
        now = time.monotonic()
        p = self.pending[name]
        if p is not None:
            if now - p[1] < self.get_parameter("request_timeout_s").value:
                return
            client.remove_pending_request(p[0])
            self.get_logger().warning(f"{name} 요청 {now - p[1]:.1f} s 무응답 — 버리고 다시 요청", throttle_duration_sec=5.0)
        if not client.service_is_ready():
            self.pending[name] = None
            self.get_logger().warning(f"{client.srv_name} 서비스 없음 — 발행 생략 (두산 bringup 확인)", throttle_duration_sec=5.0)
            return
        fut = client.call_async(req)
        self.pending[name] = (fut, now)

        def done(f, name=name):
            self.pending[name] = None
            try:
                res = f.result()
            except Exception as e:     # 취소·실패 — 안전 감시가 이 토픽 끊김을 감지하도록 발행하지 않는다
                self.get_logger().warning(f"{name} 읽기 실패, 발행 생략: {e!r}", throttle_duration_sec=5.0)
                return
            if res is not None and getattr(res, "success", True):
                on_result(res)

        fut.add_done_callback(done)

    def tick_force(self):
        self._send("force", self.cli_f, GetToolForce.Request(ref=DR_BASE), self.on_force)

    def tick_pose(self):
        self._send("pose", self.cli_p, GetCurrentPosx.Request(ref=DR_BASE), self.on_pose)

    def on_force(self, res):
        f = [float(v) for v in res.tool_force]
        if len(f) < 6 or not any(abs(v) > 1e-9 for v in f):
            self.get_logger().warning("get_tool_force 가 전부 0 — 발행 생략. ① 특이점 근처 자세면 컨트롤러가 툴 힘을 0 으로 낸다 → 관측 자세처럼 팔을 뻗은 자세로 옮기기 ② 중복 force_publisher·서비스 과부하 확인", throttle_duration_sec=5.0)
            return
        w = WrenchStamped()
        w.header.stamp = self.get_clock().now().to_msg()
        w.header.frame_id = "base_link"
        w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = f[0], f[1], f[2]
        w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z = f[3], f[4], f[5]
        self.pub_w.publish(w)

    def on_pose(self, res):
        if not res.task_pos_info or len(res.task_pos_info[0].data) < 6:
            return
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "base_link"
        ps.pose = posx_to_pose(list(res.task_pos_info[0].data[:6]))   # [x, y, z, rx, ry, rz] mm · deg(ZYZ)
        self.pub_p.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = ForcePublisher()
    if node.duplicate:
        node.get_logger().fatal("/rally/robot/wrench 발행자가 이미 있음 — force_publisher 중복 실행. 이 인스턴스는 종료합니다 (컨트롤러 서비스 과부하 방지)")
        node.destroy_node()
        rclpy.shutdown()
        return 2
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
