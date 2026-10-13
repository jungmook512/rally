"""rally_vision/flight_predictor_node.py — 모드 3 던진 공 요격점 발행 (/rally/perception/landing)
담당: 박성현

흐름: 손목·측면 RealSense 컬러(60 Hz) → velcro_ball YOLO(두 장 묶음, TensorRT .engine 또는 .pt) → 픽셀
  → flight_tracking.StereoAssociator(측면 시각에 손목 픽셀 보간, DLT 삼각측량, 재투영 검사) → base_link 3D
  → ThrowSession(투구 시작 판정 + EKF 중력·공기 저항) → 패드 평면 교차점 발행 (비행 중만)
발행
  /rally/perception/landing       geometry_msgs/PointStamped  교차점 (base_link). header.stamp = 마지막 관측의 센서 시각(손목 시계)
  /rally/perception/landing_info  std_msgs/Float64MultiArray  [교차 시각(s, ROS 시각), 교차까지 남은 시간(stamp 기준 s),
                                                               평면 위 표준편차 u·w (m), 관측 수, 릴리스 이후 s] — landing 과 같은 순서로 발행
  /rally/perception/ball_on_pad   geometry_msgs/PointStamped  측면 카메라가 패드 중심(실시간 TCP + pad_offset_tool_m) 주변 pad_ball_radius_m
                                                               안에서 본 공의 base 3D (패드 평면과 교차). 보일 때만 측면 프레임마다 → catch_ball 이
                                                               몇 초 연속이면 '잡음' (10/12 결정: 힘 판정 대신)
검출 고르기: 카메라마다 StaticFilter — 같은 자리에 오래 머무는 검출(패드 손잡이 초록 브래킷·테이블 초록 장식, 10/12 예측 0 의 원인)은 빼고
  움직이는 후보 중 conf 최고. 패드 위 공 판정은 측면 후보 전체로 한다 (붙은 공은 고정이므로).
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
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

from rally_vision import projection as pj
from rally_vision.flight_tracking import PadBallJudge, SideCalib, StaticFilter, StereoAssociator, ThrowSession, WristDet, projection_matrix

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


def parse_image_cdr(buf: bytes):
    """직렬화된(CDR, little endian) sensor_msgs/Image → (stamp_s, height, width, encoding, step, data memoryview). 복사 없음.
    10/12 밤 실로봇: 1 MB 영상을 받을 때마다 rclpy 메시지로 풀고 bgr 로 바꾸니(초당 120장) 단일 스레드 노드가 18~20 Hz 밖에 못 돌아
    손목 프레임 간격(53 ms)이 짝짓기 상한(40 ms)을 넘어 예측 0. → raw 로 받아 시각만 읽고, 추론에 쓰는 최신 두 장만 배열로."""
    mv = memoryview(buf)
    if len(mv) < 4 or mv[1] not in (0x01, 0x03):            # 캡슐화 헤더: CDR_LE(0x0001) / PL_CDR_LE
        raise ValueError("little endian CDR 아님")
    o = 4

    def u32(off):
        return int.from_bytes(mv[off:off + 4], "little")

    def align(off, n):
        r = (off - 4) % n
        return off + (n - r if r else 0)

    sec = int.from_bytes(mv[o:o + 4], "little", signed=True)
    nsec = u32(o + 4)
    o += 8
    n = u32(o)
    o += 4 + n                                               # frame_id (끝 NUL 포함 길이)
    o = align(o, 4)
    h, w = u32(o), u32(o + 4)
    o += 8
    n = u32(o)
    enc = bytes(mv[o + 4:o + 4 + n]).rstrip(b"\0").decode()
    o += 4 + n
    o += 1                                                   # is_bigendian
    o = align(o, 4)
    step = u32(o)
    o += 4
    n = u32(o)
    o += 4
    return sec + nsec * 1e-9, h, w, enc, step, mv[o:o + n]


def image_from_cdr(buf: bytes) -> np.ndarray:
    """parse_image_cdr → BGR uint8 (h, w, 3) 연속 배열. rgb8·bgr8 만."""
    _, h, w, enc, step, data = parse_image_cdr(buf)
    a = np.frombuffer(data, np.uint8).reshape(h, step)[:, :w * 3].reshape(h, w, 3)
    if enc == "rgb8":
        return np.ascontiguousarray(a[:, :, ::-1])
    if enc == "bgr8":
        return np.ascontiguousarray(a)
    raise ValueError(f"인코딩 {enc} 은 안 받음 (rgb8·bgr8)")


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
        # 패드 평면: 기본은 skills.yaml mode3.pad_center_base_m·pad_normal (받기 스킬 catch_ball 과 같은 파일, 10/12).
        # 파라미터로 세 값을 주면 그것이 우선 (bag 재생 시험용). nan 이면 "안 줌".
        d("plane_p", [float("nan")] * 3)           # 패드 중심 base [m]
        d("plane_n", [float("nan")] * 3)           # 패드 면 법선 base
        d("skills_path", "")                       # 비면 rally_bringup/config/skills.yaml (설치본 → 소스 트리)
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
        d("pad_offset_tool_m", [float("nan")] * 3)  # TCP → 패드 중심 (공구 좌표). nan = skills.yaml mode3.pad_offset_tool_m (받기 스킬과 같은 값)
        d("pad_ball_radius_m", 0.10)               # 패드 중심에서 이 안의 측면 검출 = 패드 위 공 (반지름 7.5 + 여유)
        d("static_cell_px", 24.0)                  # 고정 물체 오검출 제외: 격자 칸
        d("static_window", 120)                    #   최근 프레임 수 (60 Hz → 2 s)
        d("static_frac", 0.5)                      #   그중 이 비율 넘게 같은 칸이면 고정 물체
        # 10/13 정정인 B: "stereo"(기본, 측면 시각에 손목 보간 → 3D 점) | "rays"(3D 점 없이 카메라별 픽셀·시각으로 궤적 맞춤, flight_rays.py)
        #   bag 비교(scripts/mode3_mono_lab.py): 손목이 드문드문 볼 때 rays 는 0.25 s 에 6/6 예측(stereo 0/6), 다 볼 때 0.30 s 오차 5.6 vs 8.0 cm
        d("estimator", "stereo")
        d("debug_log", False)                      # 프레임·점마다 [진단] 로그 (공이 보이는 동안 ≤ 60/s + 교차 계산 한 번 더). 기본 끔 — #182 리뷰
        g = lambda k: self.get_parameter(k).value  # noqa: E731

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

        plane_p, plane_n, plane_src = self._pad_plane(np.array(g("plane_p"), float), np.array(g("plane_n"), float), str(g("skills_path")))
        self.get_logger().info(f"패드 평면 ({plane_src}): 중심 {np.round(plane_p, 3).tolist()} m · 법선 {np.round(plane_n, 3).tolist()}")
        self.estimator = str(g("estimator"))
        self.debug_log = bool(g("debug_log"))
        self._last_rays_fit_t = None                 # rays: 마지막으로 발행한 맞춤의 시각 — 새 관측이 없으면 다시 발행하지 않음
        if self.estimator not in ("stereo", "rays"):
            raise RuntimeError(f"estimator 는 stereo | rays (받은 값 {self.estimator})")
        if self.estimator == "rays":
            from rally_vision.flight_rays import RayThrowSession
            self.rays = RayThrowSession(plane_p=plane_p, plane_n=plane_n, k_drag=k_drag,
                                        min_start_x=float(g("min_start_x_m")), max_cross_dist=float(g("max_cross_dist_m")))
            self.get_logger().info("예측 방식: rays (3D 점 없이 카메라별 픽셀·시각으로 궤적 맞춤)")
        self.session = ThrowSession(plane_p=plane_p, plane_n=plane_n,
                                    k_drag=k_drag, meas_std_m=float(g("meas_std_m")),
                                    min_speed=float(g("min_speed_mps")), min_obs=int(g("min_obs")),
                                    min_start_x=float(g("min_start_x_m")), max_cross_dist=float(g("max_cross_dist_m")))
        self.max_reproj = float(g("max_reproj_px"))
        self.plane_n = plane_n                                # 패드 위 공 판정도 같은 평면 법선
        off = np.array(g("pad_offset_tool_m"), float)
        if not np.all(np.isfinite(off)):
            off = np.array(self._skills_mode3(str(g("skills_path"))).get("pad_offset_tool_m", [np.nan] * 3), float)
        if not np.all(np.isfinite(off)):
            raise RuntimeError("pad_offset_tool_m 을 못 읽음 — skills.yaml mode3 또는 파라미터로 지정")
        self.pad_offset = off
        self.get_logger().info(f"패드 오프셋(공구) {np.round(self.pad_offset, 4).tolist()} m · 패드 위 공 반경 {float(g('pad_ball_radius_m')):.2f} m")
        self.pad_ball_radius = float(g("pad_ball_radius_m"))
        sf = lambda: StaticFilter(float(g("static_cell_px")), int(g("static_window")), float(g("static_frac")))  # noqa: E731
        self.static = {"side": sf(), "wrist": sf()}
        self.judge: PadBallJudge | None = None               # 측면 camera_info 를 받으면 만든다
        self.max_pair_gap = float(g("max_pair_gap_s"))
        self.assoc: StereoAssociator | None = None          # 측면 camera_info 를 받으면 만든다

        self.wrist = self.side = None                         # (직렬화된 Image 바이트, stamp_s) — 10/12 raw 수신
        self.K_wrist = None
        self.tcp = None
        self._done_side = None
        self._last_wrist_t = None
        self.create_subscription(Image, g("wrist_color_topic"), self._on_wrist, qos_profile_sensor_data, raw=True)
        self.create_subscription(Image, g("side_color_topic"), self._on_side, qos_profile_sensor_data, raw=True)
        self.create_subscription(CameraInfo, g("wrist_info_topic"), self._on_wrist_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g("side_info_topic"), self._on_side_info, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, g("tcp_pose_topic"), self._on_tcp, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointStamped, "/rally/perception/landing", qos_profile_sensor_data)
        self.pub_info = self.create_publisher(Float64MultiArray, "/rally/perception/landing_info", qos_profile_sensor_data)
        self.pub_pad = self.create_publisher(PointStamped, "/rally/perception/ball_on_pad", qos_profile_sensor_data)
        self._n_pad = 0
        self._n = 0
        self._t_infer = 0.0
        self._n_pub = 0
        self._rx = [0, 0]
        self._dbg = [0, 0, 0]
        self.create_timer(1.0 / max(float(g("rate_hz")), 1.0), self._tick)
        self.create_timer(5.0, self._stats)

    # --- 패드 평면 ---
    @staticmethod
    def default_skills_path() -> str:
        cands = []
        try:
            from ament_index_python.packages import get_package_share_directory
            cands.append(os.path.join(get_package_share_directory("rally_bringup"), "config", "skills.yaml"))
        except Exception:  # noqa: BLE001
            pass
        here = os.path.dirname(os.path.realpath(__file__))
        cands.append(os.path.normpath(os.path.join(here, "..", "..", "rally_bringup", "config", "skills.yaml")))
        for p in cands:
            if os.path.isfile(p):
                return p
        return ""

    def _skills_mode3(self, skills_path: str) -> dict:
        """skills.yaml 의 mode3 절 (없으면 {})."""
        path = skills_path or self.default_skills_path()
        try:
            import yaml
            with open(path, encoding="utf-8") as f:
                return (yaml.safe_load(f) or {}).get("mode3") or {}
        except Exception:  # noqa: BLE001
            return {}

    def _pad_plane(self, p_param: np.ndarray, n_param: np.ndarray, skills_path: str):
        """(중심, 단위 법선, 출처). 파라미터 세 값이 다 있으면 그것, 아니면 skills.yaml mode3. 둘 다 없으면 시작하지 않는다
        (옛 하드코딩 값으로 조용히 돌면 받기 스킬과 다른 평면을 쓰게 됨 — 10/12 패드 TCP 재측정에서 y 5.7 cm 차이)."""
        if np.all(np.isfinite(p_param)) and np.all(np.isfinite(n_param)):
            return p_param, n_param / np.linalg.norm(n_param), "파라미터"
        path = skills_path or self.default_skills_path()
        try:
            import yaml
            with open(path, encoding="utf-8") as f:
                m3 = (yaml.safe_load(f) or {}).get("mode3") or {}
            c, n = np.array(m3["pad_center_base_m"], float), np.array(m3["pad_normal"], float)
            if c.shape != (3,) or n.shape != (3,) or not np.all(np.isfinite(c)) or np.linalg.norm(n) < 0.5:
                raise ValueError(f"pad_center_base_m={c.tolist()}, pad_normal={n.tolist()}")
        except Exception as e:  # noqa: BLE001
            raise RuntimeError(f"패드 평면을 못 읽음: {path or 'skills.yaml 없음'} mode3.pad_center_base_m·pad_normal ({e}) "
                               "— skills_path 또는 plane_p·plane_n 파라미터로 지정") from e
        return c, n / np.linalg.norm(n), path

    # --- 구독 ---
    def _on_wrist(self, buf: bytes):
        self.wrist = (buf, parse_image_cdr(buf)[0])                # raw — 배열은 _tick 에서 최신 것만
        self._rx[0] += 1

    def _on_side(self, buf: bytes):
        self.side = (buf, parse_image_cdr(buf)[0])
        self._rx[1] += 1

    def _on_wrist_info(self, msg: CameraInfo):
        self.K_wrist = np.array(msg.k, float).reshape(3, 3)

    def _on_side_info(self, msg: CameraInfo):
        if self.assoc is None:
            K = np.array(msg.k, float).reshape(3, 3)
            self.assoc = StereoAssociator(projection_matrix(K, self.calib.T_cam_from_base), self.dt_side,
                                          max_gap_s=self.max_pair_gap, max_reproj_px=self.max_reproj)
            self.judge = PadBallJudge(K, self.calib.T_cam_from_base, self.pad_ball_radius)
            self.get_logger().info("측면 camera_info 받음 → 삼각측량 준비")

    def _on_tcp(self, msg: PoseStamped):
        self.tcp = (msg, time.monotonic())

    # --- 손목 카메라 자세 ---
    def _base_from_tcp(self, t: float) -> np.ndarray | None:
        """base ← TCP 4x4 (시각 t). tf 가 있으면 그것, 없으면 마지막 tcp_pose. 없으면 None."""
        T_base_tcp = None
        if self.transform_source in ("auto", "tf"):
            try:
                # 기다리지 않음 (10/12 밤 py-spy: 단일 스레드라 기다리는 동안 tf 를 못 받아 시간의 63 % 를 여기서 씀 → 수신 18 Hz.
                # 영상 시각 tf 가 아직 없으면 아래 tcp_pose — 예측하는 동안 로봇은 대기 자세에 서 있다)
                tf = self.tf_buffer.lookup_transform(self.base_frame, self.flange_frame, Time(nanoseconds=int(t * 1e9)))
                tr, q = tf.transform.translation, tf.transform.rotation
                T_base_tcp = pj.pose_to_matrix(tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w) @ self.T_flange_tcp
            except Exception:  # noqa: BLE001
                T_base_tcp = None
        if T_base_tcp is None and self.transform_source in ("auto", "tcp_pose"):
            if self.tcp is not None and time.monotonic() - self.tcp[1] < self.tcp_max_age:
                p, q = self.tcp[0].pose.position, self.tcp[0].pose.orientation
                T_base_tcp = pj.pose_to_matrix(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        return T_base_tcp

    def _wrist_cam_from_base(self, t: float) -> np.ndarray | None:
        """camera ← base 4x4 (영상 시각 t). 없으면 None."""
        T_base_tcp = self._base_from_tcp(t)
        return None if T_base_tcp is None else np.linalg.inv(T_base_tcp @ self.T_tcp_cam)

    # --- 주기 ---
    def _tick(self):
        if self.assoc is None or self.K_wrist is None or self.side is None or self.wrist is None:
            return
        (buf_s, ts), (buf_w, tw) = self.side, self.wrist
        if ts == self._done_side:
            self.session.tick(ts + self.dt_side)
            return
        self._done_side = ts
        img_s, img_w = image_from_cdr(buf_s), image_from_cdr(buf_w)
        t0 = time.monotonic()
        res = self.model.predict([img_s, img_w], imgsz=self.imgsz, conf=self.conf, half=True,
                                 device=self.device or None, verbose=False)
        self._n += 1
        dt_inf = time.monotonic() - t0
        self._t_infer += dt_inf
        if dt_inf > 0.5:
            self.get_logger().warn(f"추론 {dt_inf:.1f} s (첫 호출 워밍업이면 정상)")
        cands_s, cands_w = (self._cands(r) for r in res)
        uv_s, uv_w = self.static["side"].pick(cands_s), self.static["wrist"].pick(cands_w)
        self._dbg[0] += uv_s is not None
        self._dbg[1] += uv_w is not None
        if self.debug_log and (uv_s is not None or uv_w is not None):   # 10/13 진단: 카메라별 검출 (비행 중 어느 쪽이 놓치는지)
            self.get_logger().info(f"[진단] t {ts % 1000:7.3f} 검출 측면 {None if uv_s is None else [round(v) for v in uv_s]} "
                                   f"손목 {None if uv_w is None else [round(v) for v in uv_w]} (측면 후보 {len(cands_s)} 손목 후보 {len(cands_w)})")
        self._judge_pad(ts, cands_s)

        if self.estimator == "rays":
            self._tick_rays(ts, tw, uv_s, uv_w)
            return
        pts = []
        if tw != self._last_wrist_t:
            self._last_wrist_t = tw
            T = self._wrist_cam_from_base(tw)
            if T is None:
                self.get_logger().warn("손목 카메라 자세 없음 (tf base_link←link_6 · tcp_pose 둘 다 없음/오래됨)", throttle_duration_sec=5.0)
            else:
                pts += self.assoc.add_wrist(WristDet(t=tw, uv=uv_w, P=projection_matrix(self.K_wrist, T)))
        pts += self.assoc.add_side(ts, uv_s)
        self._dbg[2] += len(pts)
        for t, X in sorted(pts, key=lambda a: a[0]):
            st0 = self.session.state
            c = self.session.add(t, X)
            if c is not None and c.time_to_go > 0:
                self._publish(t, c)
            # 10/13 진단: 3D 점은 있는데 예측이 안 나올 때 — 점 위치·속도·상태·교차 거리를 0.5 s 마다
            prev = getattr(self, "_dbg_prev", None)
            v = None if prev is None or t - prev[0] <= 0 or t - prev[0] > 0.1 else (X - prev[1]) / (t - prev[0])
            self._dbg_prev = (t, X)
            if self.session.state != st0:
                self.get_logger().info(f"[진단] 상태 {st0} → {self.session.state} (놓음 {self.session.t_release})")
            if self.debug_log:                                  # 10/13: 공이 보일 때만 점이 오므로 점마다 (≤ 60/s) — 교차 계산을 한 번 더 하므로 기본 끔
                vs = "-" if v is None else f"{np.linalg.norm(v):.2f} m/s (vx {v[0]:+.2f})"
                cs = ""
                if self.session.state == "flight" and self.session.fp is not None:
                    cc = self.session.fp.predict_crossing(self.session.plane_p, self.session.plane_n) if self.session.fp.n >= 1 else None
                    cs = " · 교차 없음" if cc is None else f" · 교차점 {np.round(cc.point, 2).tolist()} 패드에서 {np.linalg.norm(cc.point - self.session.plane_p):.2f} m · 관측 {self.session.fp.n}"
                self.get_logger().info(f"[진단] t {t % 1000:7.3f} 3D 점 {np.round(X, 2).tolist()} 속도 {vs} · 상태 {self.session.state}{cs}")
        self.session.tick(ts + self.dt_side)

    def _tick_rays(self, ts: float, tw: float, uv_s, uv_w) -> None:
        """rays: 측면(시각 + dt → 손목 시계)·새 손목 프레임 픽셀을 시각 순서로 RayThrowSession 에, 프레임 묶음마다 교차점 한 번."""
        r = self.rays
        ev = [(ts + self.dt_side, "side", self.assoc.P_side, uv_s)]
        if tw != self._last_wrist_t:
            self._last_wrist_t = tw
            T = self._wrist_cam_from_base(tw)
            if T is None:
                self.get_logger().warn("손목 카메라 자세 없음 (tf base_link←link_6 · tcp_pose 둘 다 없음/오래됨)", throttle_duration_sec=5.0)
            else:
                ev.append((tw, "wrist", projection_matrix(self.K_wrist, T), uv_w))
        st0 = r.state
        for t, cam, P, uv in sorted(ev, key=lambda e: e[0]):
            r.add(t, cam, P, uv)
            self._dbg[2] += uv is not None
        # 새 관측으로 맞춤이 바뀐 때만 교차점 계산·발행 (#182 리뷰): 같은 예측을 매 tick 다시 보내면 catch_ball 이 받은 시각으로
        # 신선도를 봐서 공을 놓친 뒤에도 '새 예측'으로 받는다. predict_crossing(10~27 ms)도 매 tick 하지 않게
        fit_t = r.fp.t if r.state == "flight" and r.fp is not None else None
        c = None
        if fit_t is not None and fit_t != self._last_rays_fit_t:
            self._last_rays_fit_t = fit_t
            c = r.latest()
        if r.state != st0:
            self.get_logger().info(f"[진단] 상태 {st0} → {r.state} (놓음 {r.t_release}{'' if r.state == 'flight' else ' · 끝 ' + r.end_reason})")
        if c is not None and c.time_to_go > 0:
            t_obs = r.fp.t
            self._publish(t_obs, c, n_obs=r.fp.n, t_release=r.t_release)
            if self.debug_log:
                self.get_logger().info(f"[진단] rays 교차점 {np.round(c.point, 3).tolist()} 패드에서 {np.linalg.norm(c.point - r.plane_p):.2f} m · "
                                       f"std {np.round(c.std_m * 100, 1).tolist()} cm · 관측 {r.fp.n} · 놓은 뒤 {t_obs - r.t_release:.3f} s")

    @staticmethod
    def _cands(r) -> list[tuple[float, float, float]]:
        """YOLO 결과 → [(u, v, conf), ...] (상자 중심)."""
        if r.boxes is None or not len(r.boxes):
            return []
        xyxy, conf = r.boxes.xyxy.tolist(), r.boxes.conf.tolist()
        return [((x1 + x2) / 2, (y1 + y2) / 2, float(c)) for (x1, y1, x2, y2), c in zip(xyxy, conf)]

    def _judge_pad(self, ts: float, cands_s) -> None:
        """측면 후보 중 패드 위 공이 있으면 /rally/perception/ball_on_pad 발행 (측면 프레임 시각 + dt → 손목 시계)."""
        if self.judge is None or not cands_s:
            return
        T_base_tcp = self._base_from_tcp(ts + self.dt_side)
        if T_base_tcp is None:
            return
        pad_center = T_base_tcp[:3, :3] @ self.pad_offset + T_base_tcp[:3, 3]
        hit = self.judge.check(cands_s, pad_center, self.plane_n)
        if time.monotonic() - getattr(self, "_jdbg_t", 0.0) >= 0.5:         # 10/13 진단: 판정 — 패드 중심 픽셀·후보 픽셀·거리
            self._jdbg_t = time.monotonic()
            x = self.judge.T[:3, :3] @ pad_center + self.judge.T[:3, 3]
            uvp = (self.judge.K @ x)[:2] / x[2]
            self.get_logger().info(f"[판정] 패드 중심 {np.round(pad_center, 3).tolist()} → 측면 픽셀 {[round(v) for v in uvp]} 반경 {self.judge.K[0, 0] * self.judge.radius_m / x[2]:.0f} px"
                                   f" · 후보 {[(round(u), round(v), round(c, 2)) for u, v, c in cands_s]} · 판정 {'없음' if hit is None else f'{hit[1] * 100:.1f} cm'}")
        if hit is None:
            return
        p, _ = hit
        m = PointStamped()
        m.header.stamp = to_stamp(ts + self.dt_side)
        m.header.frame_id = self.base_frame
        m.point.x, m.point.y, m.point.z = (float(v) for v in p)
        self.pub_pad.publish(m)
        self._n_pad += 1

    def _publish(self, t_obs: float, c, n_obs: int | None = None, t_release: float | None = None):
        m = PointStamped()
        m.header.stamp = to_stamp(t_obs)
        m.header.frame_id = self.base_frame
        m.point.x, m.point.y, m.point.z = (float(v) for v in c.point)
        self.pub.publish(m)
        info = Float64MultiArray()
        info.data = [float(c.t), float(c.t - t_obs), float(c.std_m[0]), float(c.std_m[1]),
                     float(self.session.fp.n if n_obs is None else n_obs),
                     float(t_obs - ((self.session.t_release if t_release is None else t_release) or t_obs))]
        self.pub_info.publish(info)
        self._n_pub += 1

    def _stats(self):
        if self._n:
            self.get_logger().info(f"수신 손목 {self._rx[0] / 5.0:.0f} · 측면 {self._rx[1] / 5.0:.0f} Hz · 추론 {self._n / 5.0:.0f} Hz · 평균 {self._t_infer / self._n * 1000:.1f} ms · "
                                   f"공 검출 프레임 측면 {self._dbg[0]} 손목 {self._dbg[1]} · 3D 점 {self._dbg[2]} · 발행 {self._n_pub} · 패드 위 공 {self._n_pad} · 재투영 탈락 누적 {self.assoc.n_rejected if self.assoc else 0} · "
                                   f"고정 오검출 칸 측면 {len(self.static['side'].static_cells())} 손목 {len(self.static['wrist'].static_cells())}")
        self._n, self._t_infer, self._n_pub, self._n_pad, self._rx = 0, 0.0, 0, 0, [0, 0]
        self._dbg = [0, 0, 0]          # 10/12 밤 진단: 5 s 동안 공 검출 프레임 측면·손목, 짝지어진 3D 점


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
