"""rally_vision/camera_tf_node.py — eye-in-hand 카메라 tf 발행 (hand_eye.yaml → camera_link)

docs/INTERFACES.md 1절의 `camera_link` 를 올린다. 손 인식(rally_human)·segmenter(transform_source=tf) 가
`base_link ← camera_color_optical_frame` 을 tf2 로 조회할 수 있게 하는 것이 목적.

hand_eye.yaml 의 값은 TCP ← camera_color_optical_frame 이고, RealSense 드라이버는 camera_link → … →
camera_color_optical_frame 을 스스로 발행한다. 그래서 이 노드는 RealSense 의 camera_link → optical 을 tf 에서
읽어 TCP ← camera_link 를 만든 뒤 발행한다 (optical 프레임에 부모가 둘이 되지 않게).

mode (파라미터):
  tcp_pose (기본) — `/rally/robot/tcp_pose`(force_publisher, 10 Hz) 를 받아 `base_link → camera_link` 를 매번 발행.
                    두산 URDF tf 가 없어도 되고, segmenter 의 tcp_npy 경로와 같은 값이라 서로 일치한다.
  static   — `parent_frame`(기본 link_6, 플랜지) → camera_link 를 static 으로 한 번 발행.
             플랜지 → TCP 는 hand_eye.yaml 의 tcp_offset_from_flange_m. 두산 robot_state_publisher 가 떠 있을 때.

    ros2 run rally_vision camera_tf
    ros2 run rally_vision camera_tf --ros-args -p mode:=static -p parent_frame:=link_6
"""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

from rally_vision import projection as pj


def _rot_to_quat(Rm: np.ndarray) -> tuple[float, float, float, float]:
    """3×3 → (x, y, z, w)."""
    t = np.trace(Rm)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (Rm[2, 1] - Rm[1, 2]) / s
        y = (Rm[0, 2] - Rm[2, 0]) / s
        z = (Rm[1, 0] - Rm[0, 1]) / s
    elif Rm[0, 0] > Rm[1, 1] and Rm[0, 0] > Rm[2, 2]:
        s = np.sqrt(1.0 + Rm[0, 0] - Rm[1, 1] - Rm[2, 2]) * 2
        w = (Rm[2, 1] - Rm[1, 2]) / s
        x = 0.25 * s
        y = (Rm[0, 1] + Rm[1, 0]) / s
        z = (Rm[0, 2] + Rm[2, 0]) / s
    elif Rm[1, 1] > Rm[2, 2]:
        s = np.sqrt(1.0 + Rm[1, 1] - Rm[0, 0] - Rm[2, 2]) * 2
        w = (Rm[0, 2] - Rm[2, 0]) / s
        x = (Rm[0, 1] + Rm[1, 0]) / s
        y = 0.25 * s
        z = (Rm[1, 2] + Rm[2, 1]) / s
    else:
        s = np.sqrt(1.0 + Rm[2, 2] - Rm[0, 0] - Rm[1, 1]) * 2
        w = (Rm[1, 0] - Rm[0, 1]) / s
        x = (Rm[0, 2] + Rm[2, 0]) / s
        y = (Rm[1, 2] + Rm[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def matrix_to_tf(T_m: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = (float(v) for v in T_m[:3, 3])
    qx, qy, qz, qw = _rot_to_quat(T_m[:3, :3])
    msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = qx, qy, qz, qw
    return msg


class CameraTfNode(Node):
    def __init__(self):
        super().__init__("rally_camera_tf")
        d = self.declare_parameter
        d("mode", "tcp_pose")                     # tcp_pose | static
        d("hand_eye_path", pj.default_hand_eye_path())
        d("base_frame", "base_link")
        d("parent_frame", "link_6")               # static 모드의 부모 (두산 플랜지 프레임 이름 — view_frames 로 확인)
        d("camera_frame", "camera_link")
        d("optical_frame", "camera_color_optical_frame")
        d("tcp_pose_topic", "/rally/robot/tcp_pose")
        d("fallback_after_s", 5.0)                # 이 시간 안에 RealSense tf 가 없으면 optical 프레임을 자식으로 직접 발행
        g = lambda k: self.get_parameter(k).value  # noqa: E731

        self.mode = str(g("mode"))
        self.base_frame, self.parent_frame = str(g("base_frame")), str(g("parent_frame"))
        self.camera_frame, self.optical_frame = str(g("camera_frame")), str(g("optical_frame"))
        he = pj.load_hand_eye(str(g("hand_eye_path")))
        if he["T_tcp_cam_mm"] is None:
            raise SystemExit(f"hand_eye.yaml 에 tcp_T_camera_optical 이 없음: {g('hand_eye_path')}")
        self.T_tcp_opt = he["T_tcp_cam_mm"].copy()
        self.T_tcp_opt[:3, 3] /= 1000.0                       # m
        self.tcp_offset = np.asarray(he["tcp_offset_m"], float)
        self.get_logger().info(f"hand_eye.yaml {g('hand_eye_path')} (TCP←optical xyz {np.round(self.T_tcp_opt[:3, 3], 4)} m, mode={self.mode})")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.T_tcp_cam: np.ndarray | None = None               # TCP ← child_frame (RealSense 내부 tf 반영 뒤)
        self.child_frame = self.camera_frame
        self.fallback_after = float(g("fallback_after_s"))
        self._t0 = self.get_clock().now()
        self.static_bc = StaticTransformBroadcaster(self)
        self.dyn_bc = TransformBroadcaster(self)
        self._static_done = False
        self._warned = False
        if self.mode == "tcp_pose":
            self.create_subscription(PoseStamped, str(g("tcp_pose_topic")), self._on_tcp, qos_profile_sensor_data)
        self.create_timer(1.0, self._tick)

    # RealSense 가 발행하는 camera_link → optical 을 읽어 TCP ← camera_link 로 바꾼다
    def _resolve(self) -> bool:
        if self.T_tcp_cam is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(self.camera_frame, self.optical_frame, Time(), timeout=Duration(seconds=0.1))
        except Exception:  # noqa: BLE001
            waited = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
            if waited < self.fallback_after:
                if not self._warned:
                    self.get_logger().warn(f"tf {self.camera_frame}→{self.optical_frame} 없음 — {self.fallback_after:.0f} s 기다린 뒤 {self.optical_frame} 을 직접 발행")
                    self._warned = True
                return False
            # RealSense 가 tf 를 안 내면(publish_tf:=false 등) optical 프레임을 자식으로 직접 발행 — 부모 충돌 없음
            self.child_frame = self.optical_frame
            self.T_tcp_cam = self.T_tcp_opt.copy()
            self.get_logger().info(f"RealSense tf 없음 → {self.base_frame if self.mode == 'tcp_pose' else self.parent_frame}→{self.optical_frame} 을 직접 발행")
            return True
        tr, q = tf.transform.translation, tf.transform.rotation
        T_cam_opt = pj.pose_to_matrix(tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)
        self.T_tcp_cam = self.T_tcp_opt @ np.linalg.inv(T_cam_opt)
        self.child_frame = self.camera_frame
        self.get_logger().info(f"TCP←{self.camera_frame} xyz {np.round(self.T_tcp_cam[:3, 3], 4)} m 확정")
        return True

    def _tick(self):
        if not self._resolve() or self.mode != "static" or self._static_done:
            return
        T_fl_tcp = np.eye(4)
        T_fl_tcp[:3, 3] = self.tcp_offset
        T = T_fl_tcp @ self.T_tcp_cam
        self.static_bc.sendTransform(matrix_to_tf(T, self.parent_frame, self.child_frame, self.get_clock().now().to_msg()))
        self._static_done = True
        self.get_logger().info(f"static tf {self.parent_frame}→{self.child_frame} 발행 (플랜지→TCP {self.tcp_offset} m 포함)")

    def _on_tcp(self, msg: PoseStamped):
        if not self._resolve():
            return
        p, q = msg.pose.position, msg.pose.orientation
        T_base_tcp = pj.pose_to_matrix(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        T = T_base_tcp @ self.T_tcp_cam
        self.dyn_bc.sendTransform(matrix_to_tf(T, self.base_frame, self.child_frame, msg.header.stamp))


def main(args=None):
    rclpy.init(args=args)
    node = CameraTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
