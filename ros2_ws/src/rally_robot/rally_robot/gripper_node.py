"""rally_robot/gripper_node.py — RG2 그리퍼 ROS 2 노드 (선생님 onrobot.RG 래핑)
담당: 정정인
- /rally/gripper/command (GripperCommand srv): width_mm, force_n → RG.move_gripper(width*10, force*10)
    wait_until_done=True 면 motion 비트가 꺼질 때까지(최대 3 s) 기다렸다가 실제 폭을 돌려준다.
    object_detected = grip detected 비트 OR (닫는 명령인데 목표보다 5 mm 이상 벌어진 채 멈춤)
- /rally/gripper/state   (sensor_msgs/JointState, 20 Hz): name=[rg2_width_mm, rg2_grip], position=[폭 mm, grip 비트]
- 파지 방식: 물체별 목표 폭을 정밀하게 주지 않는다. 폭 0 + 물체별 힘(skills.yaml gripper.*)으로 닫으면 물체에서 멈춘다.
- pymodbus 동기 클라이언트는 스레드 안전하지 않으므로 Lock 으로 감싼다. 서비스와 타이머는 Reentrant 그룹 + MultiThreadedExecutor.
- 연결 실패 시 onrobot.RG 는 조용히 '가상 그리퍼' 모드로 빠지므로 connected 를 직접 확인해 로그를 남긴다.
TODO(정정인): 이동 중에도 명령을 받으려면 AsyncModbusTcpClient 로 교체 (지금은 wait_until_done 동안 서비스가 점유됨)
"""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from rally_interfaces.srv import GripperCommand
from rally_robot.onrobot import RG
from rally_robot.dsr_helpers import GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT

MOTION_BIT, GRIP_BIT = 0, 1
WIDTH_MIN_MM, WIDTH_MAX_MM = 0.0, 110.0     # RG2
FORCE_MIN_N, FORCE_MAX_N = 3.0, 40.0
MOVE_TIMEOUT_S = 3.0
CLOSE_GAP_DETECT_MM = 5.0                   # 닫는 명령인데 이보다 벌어진 채 멈추면 물체 있음


class GripperNode(Node):
    def __init__(self):
        super().__init__("rally_gripper")
        self.declare_parameter("ip", TOOLCHARGER_IP)
        self.declare_parameter("port", TOOLCHARGER_PORT)
        ip = self.get_parameter("ip").value
        port = str(self.get_parameter("port").value)
        self.lock = threading.Lock()
        self.rg = RG(GRIPPER_NAME, ip, port)
        if self.rg.connected:
            self.get_logger().info(f"RG2 연결됨 {ip}:{port}")
        else:
            self.get_logger().error(f"RG2 연결 실패 {ip}:{port} — 명령은 success=False 로 응답. scripts/check_gripper.py 로 점검")
        self.cb = ReentrantCallbackGroup()
        self.pub = self.create_publisher(JointState, "/rally/gripper/state", 10)
        self.create_service(GripperCommand, "/rally/gripper/command", self.on_cmd, callback_group=self.cb)
        self.create_timer(0.05, self.publish_state, callback_group=self.cb)
        self._last_width, self._last_grip = 0.0, 0

    # ----- 서비스 -----
    def on_cmd(self, req, res):
        res.success = False
        res.actual_width_mm = float(self._last_width)
        res.object_detected = False
        if not self.rg.connected:
            self.get_logger().warning("RG2 미연결 상태에서 명령 수신 — 무시")
            return res
        width = min(max(float(req.width_mm), WIDTH_MIN_MM), WIDTH_MAX_MM)
        force = min(max(float(req.force_n), FORCE_MIN_N), FORCE_MAX_N)
        if width != req.width_mm or force != req.force_n:
            self.get_logger().warning(f"그리퍼 명령 범위 클램핑: {req.width_mm:.1f} mm/{req.force_n:.1f} N → {width:.1f}/{force:.1f}")
        try:
            with self.lock:
                start_width = float(self.rg.get_width())
                self.rg.move_gripper(int(round(width * 10)), int(round(force * 10)))
            closing = width < start_width
            if req.wait_until_done:
                done = self._wait_motion_done()
                if not done:
                    self.get_logger().warning(f"그리퍼 이동 {MOVE_TIMEOUT_S} s 타임아웃")
            with self.lock:
                actual = float(self.rg.get_width())
                grip = int(self.rg.get_status()[GRIP_BIT])
        except Exception as e:
            self.get_logger().error(f"그리퍼 명령 실패: {e}")
            return res
        res.success = True
        res.actual_width_mm = actual
        res.object_detected = bool(grip) or (closing and actual - width >= CLOSE_GAP_DETECT_MM)
        self.get_logger().info(
            f"gripper {start_width:.1f} → 목표 {width:.1f} / 실제 {actual:.1f} mm, {force:.0f} N, grip={grip}, object={res.object_detected}")
        return res

    def _wait_motion_done(self):
        t0 = time.monotonic()
        time.sleep(0.05)                                 # 명령 직후 motion 비트가 서기까지 잠깐
        while time.monotonic() - t0 < MOVE_TIMEOUT_S:
            with self.lock:
                moving = self.rg.get_status()[MOTION_BIT]
            if not moving:
                return True
            time.sleep(0.02)
        return False

    # ----- 상태 토픽 -----
    def publish_state(self):
        if self.rg.connected and self.lock.acquire(blocking=False):   # 이동 대기 중이면 직전 값 재사용
            try:
                self._last_width = float(self.rg.get_width())
                self._last_grip = int(self.rg.get_status()[GRIP_BIT])
            except Exception as e:
                self.get_logger().warning(f"그리퍼 상태 읽기 실패: {e}", throttle_duration_sec=5.0)
            finally:
                self.lock.release()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["rg2_width_mm", "rg2_grip"]
        msg.position = [self._last_width, float(self._last_grip)]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
