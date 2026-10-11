"""rally_vision/flight_predictor_node.py — 모드 3 던진 공 요격점 발행 (/rally/perception/landing)
담당: 박성현

흐름: 손목·측면 RealSense 컬러(60 Hz) → velcro_ball YOLO(두 장 묶음, TensorRT .engine 또는 .pt) → 픽셀
  → flight_tracking.StereoAssociator(측면 시각에 손목 픽셀 보간, DLT 삼각측량, 재투영 검사) → base_link 3D
  → ThrowSession(투구 시작 판정 + EKF 중력·공기 저항) → 패드 평면 교차점 발행 (비행 중만)
발행
  /rally/perception/landing       geometry_msgs/PointStamped  교차점 (base_link). header.stamp = 마지막 관측의 센서 시각(손목 시계)
  /rally/perception/landing_info  std_msgs/Float64MultiArray  [교차 시각(s, ROS 시각), 교차까지 남은 시간(stamp 기준 s),
                                                               평면 위 표준편차 u·w (m), 관측 수, 릴리스 이후 s] — landing 과 같은 순서로 발행
손목 카메라 자세: transform_source
  "auto"(기본) tf base_link←link_6 (영상 시각 보간, 로봇이 움직여도 정확) 이 있으면 그것, 없으면 마지막 tcp_pose(로봇 정지 중에만 정확)
  "tf" · "tcp_pose" 고정. 둘 다 × hand_eye.yaml (플랜지→TCP, TCP←컬러 광학)
측면 카메라: side_calib.json (scripts/catch_lab/mode3_traj.py calib3 결과 — 위치·dt·공기 저항 k). 카메라 위치가 바뀌면 다시 캘리브레이션.
검증: 로봇·카메라 없이 python3 -m pytest ros2_ws/src/rally_vision/test/test_flight_tracking.py.
      bag 재생: ros2 bag play throw_1010_1646 + 이 노드 (wrist_color_topic:=/camera/camera/color/image_raw).
"""
from __future__ import annotations

import os
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

from rally_vision import projection as pj
from rally_vision.flight_tracking import SideCalib, StereoAssociator, ThrowSession, WristDet, projection_matrix

MODEL_NAMES = ("velcro_ball_yolo11n-seg_1024_rot_final.engine", "velcro_ball_yolo11n-seg_1024_rot_final.pt")
MODEL_DIRS = ("/models", os.path.expanduser("~/rally/models"), "models")
CALIB_NAME = "mode3_side_calib.json"


def find_file(names, dirs=MODEL_DIRS) -> str:
    for n in names:
        for d in dirs:
            p = os.path.join(d, n)
            if os.path.isfile(p):
                return p
    return ""


def stamp_s(st) -> float:
    return st.sec + st.nanosec * 1e-9


def to_stamp(t: float):
    from builtin_interfaces.msg import Time as TimeMsg
    sec = int(np.floor(t))
    return TimeMsg(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


class FlightPredictorNode(Node):
    def __init__(self):
        super().__init__("rally_flight_predictor")
        d = self.declare_parameter
        d("model_path", "")                        # 비면 /models·~/rally/models 에서 MODEL_NAMES 순서(.engine 먼저)
        d("imgsz", 1024)
        d("conf", 0.25)
        d("device", "")
        d("wrist_color_topic", "/camera/color/image_raw")
        d("wrist_info_topic", "/camera/color/camera_info")
        d("side_color_topic", "/side/side/color/image_raw")
        d("side_info_topic", "/side/side/color/camera_info")
        d("side_calib_path", "")                   # 비면 /models·~/rally/models 의 mode3_side_calib.json
        d("dt_side_s", float("nan"))               # nan = calib 값 (10/11 +0.015)
        d("k_drag", float("nan"))                  # nan = calib 값 (10/11 0.073)
        d("meas_std_m", 0.015)
        d("plane_p", [0.327, -0.271, 0.531])       # 패드 중심 (기존 대기 자세 + 실측 패드 오프셋, 10/11)
        d("plane_n", [0.7071, 0.0, 0.7071])        # 패드 면 법선 (투구자 쪽 45° 위)
        d("min_speed_mps", 1.5)
        d("min_obs", 4)
        d("min_start_x_m", 1.2)                    # 투구 시작점이 이보다 투구자 쪽(+x)이어야 (로봇 근처 낙하·튕김 무시)
        d("max_cross_dist_m", 0.6)                 # 교차점이 패드 중심에서 이 안일 때만 발행
        d("max_reproj_px", 6.0)
        d("max_pair_gap_s", 0.04)                  # 손목 보간 두 프레임 간격 상한 — 60 Hz 에서 한 프레임 건너뜀까지 허용
        d("base_frame", "base_link")
        d("flange_frame", "link_6")
        d("transform_source", "auto")              # auto | tf | tcp_pose
        d("tcp_pose_topic", "/rally/robot/tcp_pose")
        d("tcp_pose_max_age_s", 0.5)
        d("hand_eye_path", pj.default_hand_eye_path())
        d("rate_hz", 120.0)                        # 새 측면 프레임 확인 주기 (추론은 새 프레임이 있을 때만)
        g = lambda k: self.get_parameter(k).value  # noqa: E731

        self.bridge = CvBridge()
        self.imgsz, self.conf, self.device = int(g("imgsz")), float(g("conf")), str(g("device"))
        self.base_frame, self.flange_frame = g("base_frame"), g("flange_frame")
        self.transform_source = str(g("transform_source"))
        self.tcp_max_age = float(g("tcp_pose_max_age_s"))

        he = pj.load_hand_eye(str(g("hand_eye_path")))
        if he["T_tcp_cam_mm"] is None:
            raise RuntimeError(f"hand_eye.yaml 을 못 읽음: {g('hand_eye_path')}")
        self.T_tcp_cam = np.array(he["T_tcp_cam_mm"], float)
        self.T_tcp_cam[:3, 3] /= 1000.0                     # mm → m
        self.T_flange_tcp = np.eye(4)
        self.T_flange_tcp[:3, 3] = he["tcp_offset_m"]

        calib_path = g("side_calib_path") or find_file((CALIB_NAME,))
        if not calib_path or not os.path.isfile(calib_path):
            raise RuntimeError(f"측면 캘리브레이션 {CALIB_NAME} 없음 (/models 또는 ~/rally/models) — side_calib_path 파라미터로 지정")
        self.calib = SideCalib.load(calib_path)
        dt, k = float(g("dt_side_s")), float(g("k_drag"))
        self.dt_side = self.calib.dt_side_s if np.isnan(dt) else dt
        k_drag = self.calib.k_drag if np.isnan(k) else k
        self.get_logger().info(f"측면 캘리브레이션 {calib_path}: dt {self.dt_side * 1000:+.0f} ms · k {k_drag:.4f} /m")

        from ultralytics import YOLO  # 무거운 import 는 여기서
        path = g("model_path") or find_file(MODEL_NAMES)
        if not path:
            raise RuntimeError(f"모델 없음: {MODEL_NAMES} 를 /models 또는 ~/rally/models 에 두거나 model_path 지정")
        self.model = YOLO(path, task="segment")
        # 워밍업: 첫 predict 의 지연 import·CUDA 초기화를 spin 전에 끝낸다. spin 중에 하면 실행기 스레드와 GIL 을 다퉈
        # 수십 초 멈춤 (10/11 bag 재생에서 첫 추론 51~66 s)
        t0 = time.monotonic()
        dummy = np.random.default_rng(0).integers(0, 255, (480, 848, 3), dtype=np.uint8)   # 잡음 + 낮은 conf → NMS·마스크 경로까지 로드
        for _ in range(2):
            self.model.predict([dummy, dummy], imgsz=self.imgsz, conf=0.001, half=True, device=self.device or None, verbose=False)
        self.get_logger().info(f"모델 {path} · 입력 {self.imgsz} · 워밍업 {time.monotonic() - t0:.1f} s "
                               "(.engine 은 두 장 묶음 batch=2 로 export 한 것)")

        self.session = ThrowSession(plane_p=np.array(g("plane_p"), float), plane_n=np.array(g("plane_n"), float),
                                    k_drag=k_drag, meas_std_m=float(g("meas_std_m")),
                                    min_speed=float(g("min_speed_mps")), min_obs=int(g("min_obs")),
                                    min_start_x=float(g("min_start_x_m")), max_cross_dist=float(g("max_cross_dist_m")))
        self.max_reproj = float(g("max_reproj_px"))
        self.max_pair_gap = float(g("max_pair_gap_s"))
        self.assoc: StereoAssociator | None = None          # 측면 camera_info 를 받으면 만든다

        self.wrist = self.side = None                         # (bgr, stamp_s)
        self.K_wrist = None
        self.tcp = None
        self._done_side = None
        self._last_wrist_t = None
        self.create_subscription(Image, g("wrist_color_topic"), self._on_wrist, qos_profile_sensor_data)
        self.create_subscription(Image, g("side_color_topic"), self._on_side, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g("wrist_info_topic"), self._on_wrist_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g("side_info_topic"), self._on_side_info, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, g("tcp_pose_topic"), self._on_tcp, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointStamped, "/rally/perception/landing", qos_profile_sensor_data)
        self.pub_info = self.create_publisher(Float64MultiArray, "/rally/perception/landing_info", qos_profile_sensor_data)
        self._n = 0
        self._t_infer = 0.0
        self._n_pub = 0
        self._rx = [0, 0]
        self.create_timer(1.0 / max(float(g("rate_hz")), 1.0), self._tick)
        self.create_timer(5.0, self._stats)

    # --- 구독 ---
    def _on_wrist(self, msg: Image):
        self.wrist = (self.bridge.imgmsg_to_cv2(msg, "bgr8"), stamp_s(msg.header.stamp))
        self._rx[0] += 1

    def _on_side(self, msg: Image):
        self.side = (self.bridge.imgmsg_to_cv2(msg, "bgr8"), stamp_s(msg.header.stamp))
        self._rx[1] += 1

    def _on_wrist_info(self, msg: CameraInfo):
        self.K_wrist = np.array(msg.k, float).reshape(3, 3)

    def _on_side_info(self, msg: CameraInfo):
        if self.assoc is None:
            K = np.array(msg.k, float).reshape(3, 3)
            self.assoc = StereoAssociator(projection_matrix(K, self.calib.T_cam_from_base), self.dt_side,
                                          max_gap_s=self.max_pair_gap, max_reproj_px=self.max_reproj)
            self.get_logger().info("측면 camera_info 받음 → 삼각측량 준비")

    def _on_tcp(self, msg: PoseStamped):
        self.tcp = (msg, time.monotonic())

    # --- 손목 카메라 자세 ---
    def _wrist_cam_from_base(self, t: float) -> np.ndarray | None:
        """camera ← base 4x4 (영상 시각 t). 없으면 None."""
        T_base_tcp = None
        if self.transform_source in ("auto", "tf"):
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, self.flange_frame, Time(nanoseconds=int(t * 1e9)),
                                                     timeout=Duration(seconds=0.005))
                tr, q = tf.transform.translation, tf.transform.rotation
                T_base_tcp = pj.pose_to_matrix(tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w) @ self.T_flange_tcp
            except Exception:  # noqa: BLE001
                T_base_tcp = None
        if T_base_tcp is None and self.transform_source in ("auto", "tcp_pose"):
            if self.tcp is not None and time.monotonic() - self.tcp[1] < self.tcp_max_age:
                p, q = self.tcp[0].pose.position, self.tcp[0].pose.orientation
                T_base_tcp = pj.pose_to_matrix(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if T_base_tcp is None:
            return None
        return np.linalg.inv(T_base_tcp @ self.T_tcp_cam)

    # --- 주기 ---
    def _tick(self):
        if self.assoc is None or self.K_wrist is None or self.side is None or self.wrist is None:
            return
        (img_s, ts), (img_w, tw) = self.side, self.wrist
        if ts == self._done_side:
            self.session.tick(ts + self.dt_side)
            return
        self._done_side = ts
        t0 = time.monotonic()
        res = self.model.predict([img_s, img_w], imgsz=self.imgsz, conf=self.conf, half=True,
                                 device=self.device or None, verbose=False)
        self._n += 1
        dt_inf = time.monotonic() - t0
        self._t_infer += dt_inf
        if dt_inf > 0.5:
            self.get_logger().warn(f"추론 {dt_inf:.1f} s (첫 호출 워밍업이면 정상)")
        uv_s, uv_w = (self._best_uv(r) for r in res)

        pts = []
        if tw != self._last_wrist_t:
            self._last_wrist_t = tw
            T = self._wrist_cam_from_base(tw)
            if T is None:
                self.get_logger().warn("손목 카메라 자세 없음 (tf base_link←link_6 · tcp_pose 둘 다 없음/오래됨)", throttle_duration_sec=5.0)
            else:
                pts += self.assoc.add_wrist(WristDet(t=tw, uv=uv_w, P=projection_matrix(self.K_wrist, T)))
        pts += self.assoc.add_side(ts, uv_s)
        for t, X in sorted(pts, key=lambda a: a[0]):
            c = self.session.add(t, X)
            if c is not None and c.time_to_go > 0:
                self._publish(t, c)
        self.session.tick(ts + self.dt_side)

    @staticmethod
    def _best_uv(r):
        if r.boxes is None or not len(r.boxes):
            return None
        k = int(r.boxes.conf.argmax())
        x1, y1, x2, y2 = r.boxes.xyxy[k].tolist()
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    def _publish(self, t_obs: float, c):
        m = PointStamped()
        m.header.stamp = to_stamp(t_obs)
        m.header.frame_id = self.base_frame
        m.point.x, m.point.y, m.point.z = (float(v) for v in c.point)
        self.pub.publish(m)
        info = Float64MultiArray()
        info.data = [float(c.t), float(c.t - t_obs), float(c.std_m[0]), float(c.std_m[1]),
                     float(self.session.fp.n), float(t_obs - (self.session.t_release or t_obs))]
        self.pub_info.publish(info)
        self._n_pub += 1

    def _stats(self):
        if self._n:
            self.get_logger().info(f"수신 손목 {self._rx[0] / 5.0:.0f} · 측면 {self._rx[1] / 5.0:.0f} Hz · 추론 {self._n / 5.0:.0f} Hz · 평균 {self._t_infer / self._n * 1000:.1f} ms · "
                                   f"발행 {self._n_pub} · 재투영 탈락 누적 {self.assoc.n_rejected if self.assoc else 0}")
        self._n, self._t_infer, self._n_pub, self._rx = 0, 0.0, 0, [0, 0]


def main(args=None):
    # 단일 스레드 실행기: 영상 수신 콜백과 추론이 번갈아 돈다. 멀티 스레드로 하면 실행기 스레드들의 메시지 역직렬화와 YOLO forward 의
    # 파이썬 연산이 GIL 을 다퉈 추론이 수 초씩 늘어지고 영상의 20~30 % 를 놓쳤다 (10/11 bag 재생). 이 노드에는 안전 감시 콜백이 없어
    # 추론 동안 다른 콜백이 기다려도 되고, 그동안 온 영상은 DDS 큐(depth 5)에 남는다.
    rclpy.init(args=args)
    node = FlightPredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
