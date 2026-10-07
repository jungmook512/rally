"""rally_human/hand_pose_node.py — RealSense 컬러+깊이 → MediaPipe 손 21점 → 3D → /rally/perception/hand (HandState)
담당: 정동휘

처리 (최신 프레임만, 별도 콜백 그룹 — 추론이 밀려도 다른 콜백을 막지 않게):
  1. 컬러를 image_rotation_deg 만큼 돌린 뒤 MediaPipe HandLandmarker (VIDEO 모드). 기본 0.
     RealSense 가 세로로 달려도 10/7 bag(투구 구간 13 s)에선 돌려도 검출률이 같았다 (63 % vs 65 %)
  2. 손 위치: 손바닥 5점(손목·네 MCP) 주변 정렬 깊이의 중앙값 = 손바닥 깊이. 21점 각각은
     MediaPipe 상대 깊이(z)를 손바닥 깊이에 더해 픽셀과 함께 3D 로 (깊이 구멍·손가락 가장자리 튐에 강함)
  3. tf2 로 카메라 → base_link (이미지 시각 기준). 실패하면 카메라 좌표로 내고 distance_to_centerline = NaN
  4. 손 모양 값(쥔 정도·제스처)은 MediaPipe hand_world_landmarks(손 모양만의 3D)로 — 깊이 없이 계산, 회전과 무관
  5. 제스처는 N 프레임 연속일 때만 확정 (gesture.py). 주먹 확정 시 /rally/safety/estop 한 번
손이 없으면 detected=false 를 계속 발행한다 (안 보내면 "손 없음" 과 "노드 죽음" 을 구분할 수 없음).
"""
import math
import os
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from rally_interfaces.msg import HandState
from rally_human import hand_geometry as hg
from rally_human.gesture import GestureFilter

# 모델 찾는 순서: 파라미터 → 컨테이너(/opt/mediapipe) → 저장소 models/mediapipe (git 제외, scripts/check_mediapipe.py 참고)
MODEL_CANDIDATES = ["/opt/mediapipe/hand_landmarker.task",
                    str(Path(__file__).resolve().parents[4] / "models" / "mediapipe" / "hand_landmarker.task"),
                    os.path.expanduser("~/rally/models/mediapipe/hand_landmarker.task")]


def rotate_image(img: np.ndarray, deg: int) -> np.ndarray:
    """시계 방향으로 deg(0/90/180/270)만큼 돌림."""
    return {0: img, 90: cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE), 180: cv2.rotate(img, cv2.ROTATE_180),
            270: cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)}[deg]


def unrotate_norm(u: np.ndarray, v: np.ndarray, deg: int) -> tuple[np.ndarray, np.ndarray]:
    """돌린 영상의 정규화 좌표(0~1) → 원래 영상의 정규화 좌표."""
    if deg == 90:        # 시계 방향으로 돌렸으면: x = v', y = 1 − u'
        return v, 1.0 - u
    if deg == 270:       # 반시계: x = 1 − v', y = u'
        return 1.0 - v, u
    if deg == 180:
        return 1.0 - u, 1.0 - v
    return u, v


def palm_depth(depth_m: np.ndarray, px: np.ndarray, radius: int = 4) -> float:
    """손바닥 점들 주변 (2r+1)² 창의 유효 깊이 중 가까운 무리의 중앙값 [m]. 손 뒤 배경이 섞이는 것을 막는다."""
    h, w = depth_m.shape
    vals = []
    for x, y in px.astype(int):
        win = depth_m[max(0, y - radius):min(h, y + radius + 1), max(0, x - radius):min(w, x + radius + 1)]
        vals.append(win[(win > 0.15) & (win < 4.0)])
    v = np.concatenate(vals) if vals else np.zeros(0)
    if len(v) < 5:
        return float("nan")
    near = v[v <= np.percentile(v, 30) + 0.05]
    return float(np.median(near))


def quat_to_rot(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class HandPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("rally_hand_pose")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_rotation_deg", 0)          # 세로 장착이면 270 시험 가능 (10/7 bag 에선 차이 없음)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("gesture_frames", 5)               # 30 Hz 기준 약 0.17 s
        self.declare_parameter("fist_estop_enabled", True)
        self.declare_parameter("centerline_x_m", float("nan"))    # NaN 이면 rally_bringup/config/table.yaml
        self.declare_parameter("min_detection_confidence", 0.5)

        self.rot = int(self.get_parameter("image_rotation_deg").value) % 360
        if self.rot not in (0, 90, 180, 270):
            raise ValueError("image_rotation_deg 는 0/90/180/270")
        self.base_frame = self.get_parameter("base_frame").value
        self.centerline_x = self._load_centerline()
        self.gestures = GestureFilter(int(self.get_parameter("gesture_frames").value),
                                      bool(self.get_parameter("fist_estop_enabled").value))
        self.landmarker = self._make_landmarker()

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.latest_color: Image | None = None
        self.latest_depth: Image | None = None
        self.K: np.ndarray | None = None
        self.t0_ns: int | None = None
        self.last_ts_ms = -1

        io_group = MutuallyExclusiveCallbackGroup()
        work_group = MutuallyExclusiveCallbackGroup()
        p = self.get_parameter
        self.create_subscription(Image, p("color_topic").value, self._on_color, qos_profile_sensor_data,
                                 callback_group=io_group)
        self.create_subscription(Image, p("depth_topic").value, self._on_depth, qos_profile_sensor_data,
                                 callback_group=io_group)
        self.create_subscription(CameraInfo, p("info_topic").value, self._on_info, qos_profile_sensor_data,
                                 callback_group=io_group)
        self.pub = self.create_publisher(HandState, "/rally/perception/hand", qos_profile_sensor_data)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.estop_pub = self.create_publisher(Bool, "/rally/safety/estop", latched)
        self.create_timer(1.0 / float(p("rate_hz").value), self._tick, callback_group=work_group)
        self.get_logger().info(f"손 인식 시작: 회전 {self.rot}°, 중앙선 x={self.centerline_x:.3f} m, "
                               f"주먹 정지 {'켬' if self.gestures.fist_enabled else '끔'}")

    # ----- 준비 -----
    def _make_landmarker(self):
        from mediapipe.tasks.python import BaseOptions, vision
        path = self.get_parameter("model_path").value or next((c for c in MODEL_CANDIDATES if os.path.exists(c)), "")
        if not path or not os.path.exists(path):
            raise FileNotFoundError(f"hand_landmarker.task 없음. 찾은 곳: {MODEL_CANDIDATES}")
        conf = float(self.get_parameter("min_detection_confidence").value)
        return vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=path), running_mode=vision.RunningMode.VIDEO,
            num_hands=2, min_hand_detection_confidence=conf, min_tracking_confidence=conf))

    def _load_centerline(self) -> float:
        v = float(self.get_parameter("centerline_x_m").value)
        if not math.isnan(v):
            return v
        try:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory("rally_bringup")) / "config" / "table.yaml"
            return float(yaml.safe_load(path.read_text())["table"]["centerline_x_m"])
        except Exception as e:      # noqa: BLE001 — 설정 없이도 노드는 돌아야 한다
            self.get_logger().warn(f"table.yaml 을 못 읽음 ({e}) → 중앙선 x=0.60 m 로 가정")
            return 0.60

    # ----- 입력 -----
    def _on_color(self, m: Image) -> None:
        with self.lock:
            self.latest_color = m

    def _on_depth(self, m: Image) -> None:
        with self.lock:
            self.latest_depth = m

    def _on_info(self, m: CameraInfo) -> None:
        self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)

    # ----- 처리 -----
    def _tick(self) -> None:
        with self.lock:
            cmsg, dmsg = self.latest_color, self.latest_depth
            self.latest_color = None
        if cmsg is None:
            return
        out = HandState()
        out.header = cmsg.header
        try:
            self._process(cmsg, dmsg, out)
        except Exception as e:      # noqa: BLE001 — 한 프레임 실패로 노드가 죽으면 안전 감시가 손을 잃는다
            self.get_logger().error(f"손 처리 실패: {type(e).__name__}: {e}", throttle_duration_sec=2.0)
            out.detected = False
        stable, estop = self.gestures.update(out.detected, out.gesture)
        out.gesture = stable
        self.pub.publish(out)
        if estop:
            self.estop_pub.publish(Bool(data=True))
            self.get_logger().warn("주먹 제스처 → 비상정지 요청")

    def _process(self, cmsg: Image, dmsg: Image | None, out: HandState) -> None:
        import mediapipe as mp
        bgr = self.bridge.imgmsg_to_cv2(cmsg, "bgr8")
        H, W = bgr.shape[:2]
        img = rotate_image(bgr, self.rot)
        stamp_ns = Time.from_msg(cmsg.header.stamp).nanoseconds
        if self.t0_ns is None:
            self.t0_ns = stamp_ns
        ts_ms = max(self.last_ts_ms + 1, (stamp_ns - self.t0_ns) // 1_000_000)   # VIDEO 모드는 시각이 늘어나야 함
        self.last_ts_ms = ts_ms
        res = self.landmarker.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), ts_ms)
        if not res.hand_landmarks:
            out.detected = False
            return
        # 손이 여럿이면 화면에서 가장 큰 손 (카메라에 가장 가까운 환자 손이라고 본다)
        sizes = [np.ptp([p.x for p in h]) * np.ptp([p.y for p in h]) for h in res.hand_landmarks]
        i = int(np.argmax(sizes))
        norm = res.hand_landmarks[i]
        world = np.array([[p.x, p.y, p.z] for p in res.hand_world_landmarks[i]])
        out.detected = True
        out.handedness = res.handedness[i][0].category_name.lower()

        # 손 모양 (깊이 불필요)
        out.grip_closure = hg.grip_closure(world)
        out.gesture = hg.classify_gesture(world)

        # 위치 (깊이 필요)
        u, v = unrotate_norm(np.array([p.x for p in norm]), np.array([p.y for p in norm]), self.rot)
        px = np.column_stack([u * W, v * H])
        z_rel = np.array([p.z for p in norm])            # 손목 기준 상대 깊이, 돌린 영상의 너비 단위
        if dmsg is None or self.K is None:
            out.distance_to_centerline = float("nan")
            self.get_logger().warn("깊이·카메라 정보 없음 → 손 모양만 발행", throttle_duration_sec=5.0)
            return
        depth = self.bridge.imgmsg_to_cv2(dmsg).astype(np.float32) * 1e-3
        z_palm = palm_depth(depth, px[list(hg.PALM_POINTS)])
        if math.isnan(z_palm):
            out.distance_to_centerline = float("nan")
            return
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        w_rot = img.shape[1]
        z_scale = w_rot * z_palm / (fx if self.rot in (0, 180) else fy)   # 정규화 z → m
        z = z_palm + (z_rel - z_rel[list(hg.PALM_POINTS)].mean()) * z_scale
        P_cam = np.column_stack([(px[:, 0] - cx) * z / fx, (px[:, 1] - cy) * z / fy, z])

        P, up, frame = self._to_base(P_cam, cmsg)
        out.header.frame_id = frame
        out.landmarks = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in P]
        origin, R = hg.palm_frame(P, out.handedness)
        out.palm_pose.position = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
        qx, qy, qz, qw = hg.rot_to_quat(R)
        out.palm_pose.orientation.x, out.palm_pose.orientation.y = qx, qy
        out.palm_pose.orientation.z, out.palm_pose.orientation.w = qz, qw
        out.wrist_angle_deg = hg.wrist_rotation_deg(R, up)
        d = hg.pointing_dir(P)
        out.pointing_dir.x, out.pointing_dir.y, out.pointing_dir.z = float(d[0]), float(d[1]), float(d[2])
        out.distance_to_centerline = (hg.distance_to_centerline(P, self.centerline_x)
                                      if frame == self.base_frame else float("nan"))

    def _to_base(self, P_cam: np.ndarray, cmsg: Image) -> tuple[np.ndarray, np.ndarray, str]:
        """카메라 → base_link. 이미지 시각의 tf (카메라가 팔과 같이 움직이므로 시각이 맞아야 함).
        실패하면 카메라 좌표 그대로 + '위' 는 카메라 −y 로 근사."""
        src = cmsg.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, src, Time.from_msg(cmsg.header.stamp),
                                                 timeout=Duration(seconds=0.02))
        except Exception:           # noqa: BLE001
            self.get_logger().warn(f"tf {self.base_frame} ← {src} 없음 → 카메라 좌표로 발행, 중앙선 거리 NaN",
                                   throttle_duration_sec=5.0)
            up_cam = np.array([0.0, -1.0, 0.0])
            up_cam = rotate_up(up_cam, self.rot)
            return P_cam, up_cam, src
        R = quat_to_rot(tf.transform.rotation)
        t = tf.transform.translation
        return P_cam @ R.T + np.array([t.x, t.y, t.z]), np.array([0.0, 0.0, 1.0]), self.base_frame


def rotate_up(up_cam: np.ndarray, deg: int) -> np.ndarray:
    """tf 가 없을 때 '위' 근사: 사람이 똑바로 보이게 돌린 영상의 위(−y') 를 원래 카메라 좌표로."""
    if deg == 90:       # 시계 방향으로 돌린 영상의 위 = 원래 영상의 왼쪽(−x)
        return np.array([-1.0, 0.0, 0.0])
    if deg == 270:      # 반시계로 돌린 영상의 위 = 원래 영상의 오른쪽(+x)
        return np.array([1.0, 0.0, 0.0])
    if deg == 180:
        return np.array([0.0, 1.0, 0.0])
    return up_cam


def main() -> None:
    rclpy.init()
    node = HandPoseNode()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
