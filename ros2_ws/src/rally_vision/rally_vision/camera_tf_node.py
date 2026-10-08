"""rally_vision/camera_tf_node.py — eye-in-hand 카메라 tf 발행 (hand_eye.yaml → camera_link)

docs/INTERFACES.md 1절의 `camera_link` 를 올린다. 손 인식(rally_human)·segmenter(transform_source=tf) 가
`base_link ← camera_color_optical_frame` 을 tf2 로 조회할 수 있게 하는 것이 목적.

hand_eye.yaml 의 값은 TCP ← camera_color_optical_frame 이고, RealSense 드라이버는 camera_link → … →
camera_color_optical_frame 을 스스로 발행한다. 그래서 이 노드는 RealSense 의 camera_link → optical 을 tf 에서
읽어 TCP ← camera_link 를 만든 뒤 발행한다 (optical 프레임에 부모가 둘이 되지 않게).

mode (파라미터):
  static (기본) — `parent_frame`(기본 link_6, 플랜지) → camera_link. 플랜지 → TCP 는 hand_eye.yaml 의 tcp_offset_from_flange_m.
             두산 robot_state_publisher 의 base_link→link_6(10/8 실로봇 약 56 Hz, /dsr01/joint_states 102 Hz) 위에 얹혀
             영상 시각으로 보간되는 tf 가 된다 (10/8 정동휘 실로봇 확인, PR #77).
             값은 고정이지만 /tf 에 republish_hz 로 계속 발행하고, `joint_states_topic` 이 `joint_stale_s` 넘게 끊기거나
             팔 6축이 전부 0(ros2_control 이 죽어 joint_state_publisher 가 0 으로 채운 상태)이면 경고하고 발행을 멈춘다 —
             그래야 손 노드·segmenter 의 tf 조회가 실패해 "그럴듯하지만 틀린 좌표"(손 z 1.4 m) 대신 NaN/폴백이 된다.
  tcp_pose — `/rally/robot/tcp_pose`(force_publisher, 10 Hz) 를 받아 `base_link → camera_link` 를 발행.
             두산 tf 가 없는 환경(노트북·에뮬레이터)용. segmenter 의 tcp_npy 경로와 같은 값.

RealSense 가 tf 를 안 내는 환경(publish_tf:=false)에서만 `optical_fallback:=true` — 기본은 RealSense tf 를 기다린다.
폴백으로 optical 을 직접 발행하면 그걸 먼저 받은 노드는 RealSense 가 나중에 켜져도 base→optical 조회가 계속 실패한다 (PR #77 리뷰).

    ros2 run rally_vision camera_tf                                   # static, link_6
    ros2 run rally_vision camera_tf --ros-args -p mode:=tcp_pose      # 두산 tf 없을 때
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

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
        d("mode", "static")                       # static | tcp_pose
        d("joint_states_topic", "/dsr01/joint_states")   # static 모드 건강 확인 — 두산 컨트롤러가 직접 내는 토픽 (joint_state_publisher 가 0 으로 채운 /joint_states 가 아님)
        d("joint_stale_s", 1.0)                   # 이 시간 넘게 관절값이 없으면 tf 발행 중단 + 경고
        d("hand_eye_path", pj.default_hand_eye_path())
        d("base_frame", "base_link")
        d("parent_frame", "link_6")               # static 모드의 부모 (두산 플랜지 프레임 이름 — view_frames 로 확인)
        d("camera_frame", "camera_link")
        d("optical_frame", "camera_color_optical_frame")
        d("tcp_pose_topic", "/rally/robot/tcp_pose")
        d("fallback_after_s", 5.0)                # 이 시간 안에 RealSense tf 가 없으면 optical 프레임을 자식으로 직접 발행
        d("optical_fallback", False)              # True 면 fallback_after_s 뒤 optical 프레임을 직접 발행. 기본 False: RealSense tf 가 올 때까지 기다림
                                                  # (PR #77 리뷰: 폴백 중 optical 을 dynamic 으로 받은 노드는 RealSense 가 늦게 켜져도 조회가 계속 실패)
        d("republish_hz", 30.0)                   # tcp_pose 모드: 마지막 변환을 현재 시각으로 다시 발행 (hold-last) — 영상 stamp 조회가 되게
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

        self._static: dict[str, tuple[str, np.ndarray]] = {}   # child → (parent, T_parent_child) — /tf_static 에서 모음
        qos_static = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, qos_static)
        self.T_tcp_cam: np.ndarray | None = None               # TCP ← child_frame (RealSense 내부 tf 반영 뒤)
        self.child_frame = self.camera_frame
        self.fallback_after = float(g("fallback_after_s"))
        self.optical_fallback = bool(g("optical_fallback"))
        self._fallback_on = False
        self._t0 = self.get_clock().now()
        self._last_T: np.ndarray | None = None               # tcp_pose 모드: 마지막 base←child
        self.dyn_bc = TransformBroadcaster(self)
        self._static_done = False
        self._warned = False
        self.joint_stale = float(g("joint_stale_s"))
        self._joint_t: float | None = None                     # 마지막 관절값 수신(monotonic)
        self._joint_zero = False
        self._joint_bad_logged = False
        hz = float(g("republish_hz"))
        if self.mode == "tcp_pose":
            self.create_subscription(PoseStamped, str(g("tcp_pose_topic")), self._on_tcp, qos_profile_sensor_data)
        else:
            self.create_subscription(JointState, str(g("joint_states_topic")), self._on_joints, qos_profile_sensor_data)
        if hz > 0:
            self.create_timer(1.0 / hz, self._republish)
        self.create_timer(1.0, self._tick)

    # RealSense 가 발행하는 camera_link → optical 을 읽어 TCP ← camera_link 로 바꾼다.
    # 폴백(optical 직접 발행) 중에도 매번 다시 찾아, RealSense 가 늦게 켜지면 camera_link 로 바꿔 부모가 둘이 되지 않게 한다.
    def _resolve(self) -> bool:
        if self.T_tcp_cam is not None and not self._fallback_on:
            return True
        T_cam_opt = self._static_chain(self.camera_frame, self.optical_frame)
        if T_cam_opt is None:
            if self._fallback_on:
                return True
            waited = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
            if not self.optical_fallback or waited < self.fallback_after:
                if not self._warned:
                    what = f"{self.fallback_after:.0f} s 기다린 뒤 {self.optical_frame} 을 직접 발행" if self.optical_fallback else "올 때까지 발행 안 함"
                    self.get_logger().warn(f"tf {self.camera_frame}→{self.optical_frame} 없음 — RealSense 가 떠 있어야 함. {what}")
                    self._warned = True
                return False
            # RealSense 가 tf 를 안 내면(publish_tf:=false 등) optical 프레임을 자식으로 직접 발행. 나중에 나타나면 아래서 전환
            self.child_frame = self.optical_frame
            self.T_tcp_cam = self.T_tcp_opt.copy()
            self._fallback_on = True
            self.get_logger().info(f"RealSense tf 없음 → {self.base_frame if self.mode == 'tcp_pose' else self.parent_frame}→{self.optical_frame} 을 직접 발행 (RealSense tf 가 나타나면 {self.camera_frame} 로 전환)")
            return True
        self.T_tcp_cam = self.T_tcp_opt @ np.linalg.inv(T_cam_opt)
        if self._fallback_on:
            self.get_logger().info(f"RealSense tf 나타남 → 자식을 {self.optical_frame} 에서 {self.camera_frame} 로 전환")
            self._fallback_on = False
            self._static_done = False
            self._last_T = None
        self.child_frame = self.camera_frame
        self.get_logger().info(f"TCP←{self.camera_frame} xyz {np.round(self.T_tcp_cam[:3, 3], 4)} m 확정")
        return True

    def _on_tf_static(self, msg: TFMessage):
        for t in msg.transforms:
            if t.header.frame_id == self.base_frame or t.header.frame_id == self.parent_frame:
                continue                                        # 우리(또는 다른 노드)가 올린 카메라 tf 는 체인에 넣지 않음
            tr, q = t.transform.translation, t.transform.rotation
            self._static[t.child_frame_id] = (t.header.frame_id, pj.pose_to_matrix(tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w))

    def _static_chain(self, root: str, leaf: str) -> np.ndarray | None:
        """/tf_static 만으로 root → … → leaf 합성 (RealSense: camera_link → camera_color_frame → camera_color_optical_frame).
        tf2 버퍼 조회 대신 쓰는 이유: 폴백으로 optical 을 직접 발행하는 동안엔 optical 의 부모가 둘이라 버퍼 조회가 안 됨."""
        T = np.eye(4)
        f = leaf
        for _ in range(8):
            if f == root:
                return T
            if f not in self._static:
                return None
            parent, T_pc = self._static[f]
            T = T_pc @ T
            f = parent
        return None

    def _tick(self):
        ok = self._resolve()
        if not ok or self.mode != "static" or self._static_done:
            return
        T_fl_tcp = np.eye(4)
        T_fl_tcp[:3, 3] = self.tcp_offset
        self._last_T = T_fl_tcp @ self.T_tcp_cam
        self._static_done = True
        self.get_logger().info(f"{self.parent_frame}→{self.child_frame} 확정 (플랜지→TCP {self.tcp_offset} m 포함) — "
                               f"{self.joint_stale:.0f} s 안에 관절값이 오는 동안만 발행")

    def _on_joints(self, msg: JointState):
        self._joint_t = time.monotonic()
        pos = list(msg.position[:6])
        self._joint_zero = len(pos) >= 6 and all(abs(v) < 1e-6 for v in pos)

    def _joints_ok(self) -> bool:
        """static 모드 건강 조건: 관절값이 최근에 왔고, 팔 6축이 전부 0 이 아님."""
        if self._joint_t is None or time.monotonic() - self._joint_t > self.joint_stale:
            why = "관절값 없음" if self._joint_t is None else f"관절값 {self.joint_stale:.0f} s 넘게 끊김"
        elif self._joint_zero:
            why = "팔 6축이 전부 0 (ros2_control 죽음?)"
        else:
            if self._joint_bad_logged:
                self.get_logger().info("관절값 복구 → tf 발행 재개")
                self._joint_bad_logged = False
            return True
        if not self._joint_bad_logged:
            self.get_logger().error(f"{why} → {self.parent_frame}→{self.child_frame} tf 발행 중단 (틀린 좌표 방지). 두산 bringup 확인")
            self._joint_bad_logged = True
        return False

    def _on_tcp(self, msg: PoseStamped):
        if not self._resolve():
            return
        p, q = msg.pose.position, msg.pose.orientation
        T_base_tcp = pj.pose_to_matrix(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        T = T_base_tcp @ self.T_tcp_cam
        self._last_T = T
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:               # stamp 없는 발행기(ros2 topic pub 등) → 현재 시각
            stamp = self.get_clock().now().to_msg()
        self.dyn_bc.sendTransform(matrix_to_tf(T, self.base_frame, self.child_frame, stamp))

    def _republish(self):
        """tcp_pose(10 Hz) 사이를 메운다: 마지막 변환을 현재 시각으로 다시 발행(hold-last).
        영상(30 Hz) stamp 가 마지막 tf 보다 새로우면 tf2 는 외삽을 안 해 조회가 실패하므로(PR #77 리뷰 1),
        손 노드·segmenter 가 영상 시각으로 조회해도 되게 한다. 값은 최근 tcp_pose(≤ 0.1 s 전) — 로봇 정지 중엔 정확,
        이동 중엔 그만큼 뒤처진다. 근본 해결은 static 모드(robot_state_publisher, 관절 토픽 속도)."""
        if self._last_T is None:
            return
        if self.mode == "static":
            if not self._joints_ok():
                return
            parent = self.parent_frame
        else:
            parent = self.base_frame
        self.dyn_bc.sendTransform(matrix_to_tf(self._last_T, parent, self.child_frame, self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = CameraTfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
