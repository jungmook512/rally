"""rally_vision/segmenter_node.py — YOLO-Seg → TrackedObjectArray (/rally/perception/objects)
담당: 박성현

흐름 (타이머, rate_hz): 최신 컬러·정렬 깊이·camera_info → ultralytics seg 추론 → 인스턴스마다
  마스크 픽셀 깊이 중앙값으로 3D 중심 (배경 깊이 오염 제거) → camera → base_link → PCA 주축 yaw, mask_area, bbox
  → 같은 클래스 직전 위치와 비교해 velocity·is_moving (간단 추적, ByteTrack 은 tracker_node) → 발행
camera → base_link 변환 (projection.py):
  1) tf2  base_link ← 이미지 frame_id (이미지 시각)      ← INTERFACES 1절. camera_link static tf 가 있을 때
  2) 폴백 /rally/robot/tcp_pose × T_gripper2camera.npy    ← 과도기 (선생님 방식). tf 가 없고 tcp_pose 가 0.5 s 안이면
  3) 둘 다 없으면 카메라 좌표 그대로 (header.frame_id = 카메라 프레임). 가짜 base 좌표를 만들지 않는다
모델: model_path 가 비면 /models(컨테이너 볼륨)·~/rally/models 에서 yolov8n-seg_rally.pt → yolov8n-seg_ball.pt 를 찾고,
      없으면 COCO yolov8n-seg (sports ball → ball, cup → cup) 임시. 컨테이너에서 base_link 변환을 쓰려면 T_gripper2camera.npy 도 models/ 에 복사.
      class_alias 로 모델 클래스 이름을 INTERFACES 이름(ball, velcro_ball, rod, cup)으로 바꾼다.
검증: 로봇·카메라 없이 — python3 -m pytest ros2_ws/src/rally_vision/test, scripts/segment_offline.py (저장 사진+깊이)
"""
from __future__ import annotations

import os
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, Vector3
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from tf2_ros import Buffer, TransformListener

from rally_interfaces.msg import TrackedObject, TrackedObjectArray
from rally_vision import projection as pj

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

DEFAULT_ALIAS = "sports ball:ball,tennis_ball:ball,tennis ball:ball,velcro ball:velcro_ball,stick:rod"
KNOWN_CLASSES = ("ball", "velcro_ball", "pad", "rod", "cup", "hand")   # TrackedObject.msg 주석과 같게


def parse_alias(s: str) -> dict[str, str]:
    out = {}
    for item in s.split(","):
        if ":" in item:
            k, v = item.split(":", 1)
            out[k.strip()] = v.strip()
    return out


MODEL_SEARCH = ("yolov8n-seg_rally.pt", "yolov8n-seg_ball.pt")      # model_path 가 비면 이 순서로 찾는다
MODEL_DIRS = ("/models", os.path.expanduser("~/rally/models"), "models")  # 컨테이너 볼륨(compose ../models:/models) → 네이티브


def default_model_path() -> str:
    """학습 모델을 찾으면 그 경로, 없으면 "yolov8n-seg.pt"(COCO 임시, 인터넷 필요)."""
    for d in MODEL_DIRS:
        for name in MODEL_SEARCH:
            p = os.path.join(d, name)
            if os.path.isfile(p):
                return p
    return "yolov8n-seg.pt"


def default_gripper2cam_path() -> str:
    """rally_robot 패키지의 npy, 없으면(컨테이너) /models 또는 ~/rally/models 에 복사해 둔 npy."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("rally_robot"), "resource", "T_gripper2camera.npy")
        if os.path.isfile(p):
            return p
    except Exception:  # noqa: BLE001  rally_robot 이 없는 컨테이너
        pass
    for d in MODEL_DIRS:
        p = os.path.join(d, "T_gripper2camera.npy")
        if os.path.isfile(p):
            return p
    return ""


def masks_from_result(res, shape_hw) -> list[np.ndarray]:
    """ultralytics Results → 원본 해상도 bool 마스크 목록. seg 모델이면 폴리곤을 채우고, detect 모델이면 bbox 안 타원."""
    h, w = shape_hw
    masks = []
    n = len(res.boxes)
    polys = res.masks.xy if getattr(res, "masks", None) is not None else None
    for i in range(n):
        m = np.zeros((h, w), dtype=np.uint8)
        if polys is not None and i < len(polys) and len(polys[i]) >= 3:
            cv2.fillPoly(m, [np.round(polys[i]).astype(np.int32)], 1)
        else:
            x0, y0, x1, y1 = res.boxes.xyxy[i].tolist()
            cv2.ellipse(m, (int((x0 + x1) / 2), int((y0 + y1) / 2)), (max(1, int((x1 - x0) / 2)), max(1, int((y1 - y0) / 2))), 0, 0, 360, 1, -1)
        masks.append(m.astype(bool))
    return masks


def extract_objects(res, depth_mm: np.ndarray, K, T_cam_to_frame: np.ndarray | None, names: dict, alias: dict,
                    conf_min: float, known_only: bool = True) -> list[dict]:
    """추론 결과 1프레임 → 물체 dict 목록 (ROS 메시지 없음, 단위 테스트용).
    dict: class_name, confidence, center(3,) [m, T 적용 프레임], yaw[rad], mask_area[px], bbox(x,y,w,h), uv(u,v), z_m, mask"""
    out = []
    if res is None or len(res.boxes) == 0:
        return out
    masks = masks_from_result(res, depth_mm.shape[:2])
    T = np.eye(4) if T_cam_to_frame is None else T_cam_to_frame
    for i, m in enumerate(masks):
        conf = float(res.boxes.conf[i])
        if conf < conf_min:
            continue
        raw = names.get(int(res.boxes.cls[i]), str(int(res.boxes.cls[i])))
        cname = alias.get(raw, raw)
        if known_only and cname not in KNOWN_CLASSES:      # COCO 임시 모델의 엉뚱한 클래스(mouse 등) 는 버림
            continue
        z = pj.mask_depth_median(depth_mm, m)
        axis = pj.mask_principal_axis(m)
        if z is None or axis is None:
            continue
        cx, cy = axis[0], axis[1]
        p_cam = pj.pixel_to_camera(cx, cy, z, K)
        center = pj.transform_points(T, p_cam)[0]
        yaw = pj.yaw_in_frame(T, p_cam, axis, z, K) if axis[3] > 1.3 else 0.0   # 둥근 물체는 yaw 의미 없음 → 0
        x0, y0, x1, y1 = res.boxes.xyxy[i].tolist()
        out.append(dict(class_name=cname, confidence=conf, center=center, yaw=yaw, mask_area=float(m.sum()),
                        bbox=(int(x0), int(y0), int(x1 - x0), int(y1 - y0)), uv=(cx, cy), z_m=z, mask=m))
    return out


def pick_transform_source(source: str, tf_ok: bool, tcp_ok: bool) -> str | None:
    """transform_source 규칙 (순수 함수, 테스트용).
    tcp_npy: tcp_pose×npy 만 · tf: tf2 만 · auto: tf 우선, 없으면 tcp_npy. 쓸 수 있는 게 없으면 None(카메라 좌표)."""
    if source == "tf":
        return "tf" if tf_ok else None
    if source == "auto" and tf_ok:
        return "tf"
    return "tcp_npy" if tcp_ok else None


class SimpleTracks:
    """클래스별 최근 위치로 velocity·is_moving·track_id. ByteTrack 전 임시."""

    def __init__(self, match_m: float = 0.15, moving_mps: float = 0.05, max_age_s: float = 1.0):
        self.match_m, self.moving_mps, self.max_age_s = match_m, moving_mps, max_age_s
        self.tracks: list[dict] = []
        self.next_id = 1

    def update(self, objs: list[dict], t: float) -> None:
        self.tracks = [tr for tr in self.tracks if t - tr["t"] < self.max_age_s]
        used = set()
        for o in objs:
            best, bd = None, self.match_m
            for j, tr in enumerate(self.tracks):
                if j in used or tr["class_name"] != o["class_name"]:
                    continue
                d = float(np.linalg.norm(tr["center"] - o["center"]))
                if d < bd:
                    best, bd = j, d
            if best is None:
                o["track_id"], o["velocity"], o["is_moving"] = self.next_id, np.zeros(3), False
                self.tracks.append(dict(id=self.next_id, class_name=o["class_name"], center=o["center"], t=t, vel=np.zeros(3)))
                self.next_id += 1
                continue
            used.add(best)
            tr = self.tracks[best]
            dt = max(t - tr["t"], 1e-3)
            v_raw = (o["center"] - tr["center"]) / dt
            vel = 0.5 * tr["vel"] + 0.5 * v_raw
            o["track_id"], o["velocity"], o["is_moving"] = tr["id"], vel, bool(np.linalg.norm(vel) > self.moving_mps)
            tr.update(center=o["center"], t=t, vel=vel)


class SegmenterNode(Node):
    def __init__(self):
        super().__init__("rally_segmenter")
        d = self.declare_parameter
        d("model_path", "")                       # 비면 /models·~/rally/models 의 yolov8n-seg_rally.pt → _ball.pt → COCO 임시 순으로 찾음
        d("class_alias", DEFAULT_ALIAS)           # "모델이름:INTERFACES이름,..."
        d("conf", 0.5)
        d("imgsz", 640)
        d("device", "")                           # "" 자동, "cpu", "0"
        d("rate_hz", 30.0)
        d("color_topic", "/camera/color/image_raw")
        d("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        d("info_topic", "/camera/color/camera_info")
        d("base_frame", "base_link")
        d("tcp_pose_topic", "/rally/robot/tcp_pose")
        d("tcp_pose_max_age_s", 0.5)
        d("gripper2cam_path", default_gripper2cam_path())
        d("hand_eye_path", pj.default_hand_eye_path())   # rally_bringup/config/hand_eye.yaml — 있으면 npy 보다 우선, depth_scale 도 여기서
        d("depth_scale", 0.0)                     # 0 = hand_eye.yaml 값(없으면 1.0). RealSense 깊이 보정 배율
        d("transform_source", "tcp_npy")          # 카메라→base 변환: "tcp_npy"(기본, tcp_pose × hand_eye) · "tf" · "auto"(tf 우선). 로봇 PC URDF 가 카메라
                                                  # 체인을 설계값으로 발행하는 동안(PR #77 정정인 확인) tf/auto 는 그 값을 집어 1 cm 치우침 — URDF 에서 빼면 auto 로
        d("debug_hz", 10.0)
        d("moving_mps", 0.05)
        d("known_only", True)                     # INTERFACES 클래스(ball·velcro_ball·pad·rod·cup·hand) 만 발행
        g = lambda k: self.get_parameter(k).value  # noqa: E731

        self.bridge = CvBridge()
        self.alias = parse_alias(g("class_alias"))
        self.conf, self.imgsz, self.device = float(g("conf")), int(g("imgsz")), str(g("device"))
        self.known_only = bool(g("known_only"))
        self.base_frame = g("base_frame")
        self.tcp_max_age = float(g("tcp_pose_max_age_s"))
        self.transform_source = str(g("transform_source"))
        if self.transform_source not in ("tcp_npy", "tf", "auto"):
            self.get_logger().warn(f"transform_source={self.transform_source!r} 모름 → tcp_npy")
            self.transform_source = "tcp_npy"
        self._tf_src_logged = None
        self.g2c = None
        he = pj.load_hand_eye(str(g("hand_eye_path")))
        self.depth_scale = float(g("depth_scale")) or he["depth_scale"]
        if he["T_tcp_cam_mm"] is not None:
            self.g2c = he["T_tcp_cam_mm"]
            self.get_logger().info(f"hand_eye.yaml: {g('hand_eye_path')} (transform_source={self.transform_source}, depth_scale={self.depth_scale})")
        elif g("gripper2cam_path"):
            self.get_logger().warn(f"hand_eye.yaml 없음({g('hand_eye_path') or '경로 못 찾음'}) → 선생님 npy + depth_scale {self.depth_scale} 로 폴백 "
                                   "(10/8 검증 오차 10~42 mm — rally_bringup 이 설치됐는지, hand_eye_path 파라미터 확인)")
            try:
                self.g2c = np.load(g("gripper2cam_path"))
                self.get_logger().info(f"T_gripper2camera: {g('gripper2cam_path')} (transform_source={self.transform_source}, depth_scale={self.depth_scale})")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"npy 로드 실패 {g('gripper2cam_path')}: {e}")

        from ultralytics import YOLO  # 무거운 import 는 여기서
        path = g("model_path") or default_model_path()
        self.model = YOLO(path)
        self.names = dict(self.model.names)
        self.get_logger().info(f"모델 {path} 클래스 {sorted(set(self.alias.get(v, v) for v in self.names.values()))[:12]}…")

        self.color = self.depth = self.K = self.tcp = None
        self.color_stamp = self.color_frame = None
        sub_cg = ReentrantCallbackGroup()                 # 구독은 가볍게, 추론(타이머)은 별도 그룹 — 추론이 수신을 막지 않게
        self.infer_cg = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, g("color_topic"), self._on_color, qos_profile_sensor_data, callback_group=sub_cg)
        self.create_subscription(Image, g("depth_topic"), self._on_depth, qos_profile_sensor_data, callback_group=sub_cg)
        self.create_subscription(CameraInfo, g("info_topic"), self._on_info, qos_profile_sensor_data, callback_group=sub_cg)
        self.create_subscription(PoseStamped, g("tcp_pose_topic"), self._on_tcp, qos_profile_sensor_data, callback_group=sub_cg)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(TrackedObjectArray, "/rally/perception/objects", qos_profile_sensor_data)
        self.pub_dbg = self.create_publisher(Image, "/rally/perception/objects/debug_image", qos_profile_sensor_data)
        self.tracks = SimpleTracks(moving_mps=float(g("moving_mps")))
        self._last_dbg = 0.0
        self._dbg_period = 1.0 / max(float(g("debug_hz")), 0.1)
        self._done_stamp = None
        self._n, self._t_infer = 0, 0.0
        self.create_timer(1.0 / max(float(g("rate_hz")), 1.0), self._tick, callback_group=self.infer_cg)
        self.create_timer(5.0, self._stats, callback_group=self.infer_cg)

    # --- 구독 ---
    def _on_color(self, msg: Image):
        self.color, self.color_stamp, self.color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8"), msg.header.stamp, msg.header.frame_id

    def _on_depth(self, msg: Image):
        d = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth = d if d.dtype == np.uint16 else (d * 1000.0).astype(np.uint16)   # 16UC1 mm 또는 32FC1 m

    def _on_info(self, msg: CameraInfo):
        self.K = list(msg.k)

    def _on_tcp(self, msg: PoseStamped):
        self.tcp = (msg, time.monotonic())

    # --- 변환 ---
    def _cam_to_base(self, stamp, cam_frame: str):
        """(T 4x4, frame_id). transform_source 에 따라 tf2 / tcp_pose×npy 를 고르고, 둘 다 없으면 None(카메라 좌표).

        기본은 tcp_npy. 로봇 PC bringup URDF(m0609_with_rg2_camera)가 tool0→bracket→camera_link→…→optical 을 설계값으로 발행해서
        (10/8 정정인 view_frames) tf/auto 는 그 값을 집어 1 cm·2.5° 치우친다(PR #42 의 원인). URDF 에서 카메라 체인을 뺀 뒤 camera_tf(static)
        가 유일한 camera_link 부모가 되면 auto 로 바꾼다 — 그때 영상 시각 보간이 되어 이동 중에도 정확.

        시각 주의 (PR #73 리뷰): tcp_npy 는 영상 stamp 가 아니라 **마지막으로 받은 tcp_pose**(10 Hz, 수신 0.5 s 안)를 쓴다.
        → 로봇이 멈춰 있을 때만 base 좌표가 정확하다. 움직이는 중의 좌표는 로봇 이동에 쓰지 말 것.
        tf 경로는 영상 stamp 로 조회하므로(tf2 보간) camera_tf 노드가 있으면 "auto" 가 더 낫다.
        """
        src = self.transform_source
        tf_T = None
        if src in ("tf", "auto"):
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, cam_frame, Time.from_msg(stamp), timeout=Duration(seconds=0.02))
                tr, q = tf.transform.translation, tf.transform.rotation
                tf_T = pj.pose_to_matrix(tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)
            except Exception:  # noqa: BLE001
                pass
        tcp_ok = self.tcp is not None and self.g2c is not None and time.monotonic() - self.tcp[1] < self.tcp_max_age
        pick = pick_transform_source(src, tf_T is not None, tcp_ok)
        if pick == "tf":
            self._log_source("tf", f"{self.base_frame}←{cam_frame}")
            return tf_T, self.base_frame
        if pick == "tcp_npy":
            p, q = self.tcp[0].pose.position, self.tcp[0].pose.orientation
            T = pj.base_from_tcp(pj.pose_to_matrix(p.x, p.y, p.z, q.x, q.y, q.z, q.w), self.g2c)
            self._log_source("tcp_npy", "tcp_pose × hand_eye (마지막 tcp_pose 기준 — 로봇 정지 중에만 정확)")
            return T, self.base_frame
        if src == "tf":
            why = "tf 없음"
        else:
            why = "tcp_pose 없음" if self.tcp is None else ("hand_eye/npy 없음" if self.g2c is None else "tcp_pose 오래됨")
        self.get_logger().warn(f"base 변환 불가({why}, transform_source={src}) → 카메라 좌표로 발행", throttle_duration_sec=5.0)
        return None, cam_frame

    def _log_source(self, kind: str, detail: str):
        if self._tf_src_logged != kind:
            self._tf_src_logged = kind
            self.get_logger().info(f"카메라→base 변환: {kind} ({detail})")

    # --- 주기 ---
    def _tick(self):
        if self.color is None or self.depth is None or self.K is None or self.color_stamp == self._done_stamp:
            return
        img, depth, stamp, cam_frame = self.color, self.depth, self.color_stamp, self.color_frame
        self._done_stamp = stamp
        if abs(self.depth_scale - 1.0) > 1e-6:                 # hand_eye.yaml depth_scale (RealSense 깊이 3 % 보정)
            depth = (depth.astype(np.float32) * self.depth_scale).astype(np.uint16)
        if depth.shape[:2] != img.shape[:2]:
            self.get_logger().warn("컬러·깊이 해상도 불일치 — align_depth.enable:=true 확인", throttle_duration_sec=5.0)
            return
        t0 = time.monotonic()
        res = self.model.predict(img, imgsz=self.imgsz, conf=self.conf, device=self.device or None, verbose=False)[0]
        T, frame = self._cam_to_base(stamp, cam_frame)
        objs = extract_objects(res, depth, self.K, T, self.names, self.alias, self.conf, self.known_only)
        self.tracks.update(objs, time.monotonic())
        self._n += 1; self._t_infer += time.monotonic() - t0

        arr = TrackedObjectArray()
        arr.header.stamp, arr.header.frame_id = stamp, frame
        for o in objs:
            m = TrackedObject()
            m.header = arr.header
            m.track_id, m.class_name, m.confidence = int(o["track_id"]), o["class_name"], float(o["confidence"])
            m.position = Point(x=float(o["center"][0]), y=float(o["center"][1]), z=float(o["center"][2]))
            v = o["velocity"]; m.velocity = Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))
            m.yaw, m.mask_area, m.is_moving = float(o["yaw"]), float(o["mask_area"]), bool(o["is_moving"])
            x, y, w, h = o["bbox"]; m.bbox = RegionOfInterest(x_offset=max(x, 0), y_offset=max(y, 0), width=max(w, 0), height=max(h, 0))
            arr.objects.append(m)
        self.pub.publish(arr)

        now = time.monotonic()
        if now - self._last_dbg >= self._dbg_period and self.pub_dbg.get_subscription_count() > 0:
            self._last_dbg = now
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(draw_debug(img, objs, frame), "bgr8", header=arr.header))

    def _stats(self):
        if self._n:
            self.get_logger().info(f"{self._n / 5.0:.1f} fps, 추론+후처리 {1000 * self._t_infer / self._n:.0f} ms/프레임, 추적 {len(self.tracks.tracks)}")
            self._n, self._t_infer = 0, 0.0


def draw_debug(img, objs, frame):
    vis = img.copy()
    for o in objs:
        m = o["mask"]
        vis[m] = (0.55 * vis[m] + 0.45 * np.array([0, 200, 255])).astype(np.uint8)
        x, y, w, h = o["bbox"]
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 200, 255), 1)
        c = o["center"]
        cv2.putText(vis, f"{o['class_name']}#{o.get('track_id', 0)} {o['confidence']:.2f} ({c[0]:.2f},{c[1]:.2f},{c[2]:.2f}) yaw{np.degrees(o['yaw']):.0f}",
                    (x, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(vis, f"frame: {frame}", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return vis


def main(args=None):
    rclpy.init(args=args)
    n = SegmenterNode()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
