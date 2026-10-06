"""정답을 아는 가짜 장면 → 실제와 같은 형식의 bag.

장면 (RealSense 컬러 광학 프레임 C 기준):
    RealSense 는 원점, 아래로 약간 숙임(중력이 정확히 +y 가 아니게) → IMU 로 중력 추정 경로까지 시험
    던지는 사람 z≈2.6 m, 공이 패드 평면 z=0.15 에 도착
    웹캠은 옆(x=+2.6 m)에서 투구 통로를 봄, 30 fps, RealSense 시계보다 dt_true 만큼 늦게 찍힘(stamp)
    깊이는 원본(정렬 안 함) + 컬러와 다른 렌즈 위치(x −15 mm), 다른 시각(4 ms 어긋남)
"""
from dataclasses import dataclass, field

import cv2
import numpy as np
import rosbag2_py
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_msgs.msg import TFMessage

G = 9.81
BALL_R = 0.0325
BALL_BGR = (40, 255, 200)        # 형광 연두 (HSV H≈35)


def look_at(eye: np.ndarray, target: np.ndarray, up: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """C → 카메라 (R, t). 카메라 광학 프레임: z 앞, x 오른쪽, y 아래."""
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)          # up 은 C 의 "위" (−y 쪽)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])
    return R, -R @ eye


def rot_x(deg: float) -> np.ndarray:
    a = np.radians(deg)
    return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])


@dataclass
class Scene:
    tilt_deg: float = 8.0                      # RealSense 가 아래로 숙인 각
    dt_true: float = 0.035                     # 웹캠 stamp + dt_true = RealSense 시계
    depth_offset_s: float = 0.004
    depth_noise_rel: float = 0.01              # 깊이 잡음 σ = 1% × z
    px_noise: float = 0.0                      # 그림 그릴 때 중심 흔들기
    rs_size: tuple[int, int] = (848, 480)
    wc_size: tuple[int, int] = (640, 480)
    wc_hfov: float = 62.0
    seed: int = 0
    K_rs: np.ndarray = field(init=False)
    K_d: np.ndarray = field(init=False)
    R_wc: np.ndarray = field(init=False)
    t_wc: np.ndarray = field(init=False)
    K_wc: np.ndarray = field(init=False)
    g: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        w, h = self.rs_size
        s = w / 848                                 # 848x480 기준 D435 초점거리를 해상도에 맞춰 축소
        self.K_rs = np.array([[605.0 * s, 0, w / 2 + 3 * s], [0, 605.0 * s, h / 2 - 2 * s], [0, 0, 1]])
        self.K_d = np.array([[425.0 * s, 0, w / 2 - s], [0, 425.0 * s, h / 2 + s], [0, 0, 1]])
        # 카메라가 아래로 tilt 만큼 숙임 → 카메라 좌표에서 중력은 +y 에서 +z(앞) 쪽으로 기움
        self.g = rot_x(self.tilt_deg) @ np.array([0, G, 0])
        up = -self.g / G
        eye = np.array([2.6, -0.3, 1.4])
        self.R_wc, self.t_wc = look_at(eye, np.array([0.0, -0.4, 1.4]), up)
        f = (self.wc_size[0] / 2) / np.tan(np.radians(self.wc_hfov) / 2)
        self.K_wc = np.array([[f, 0, self.wc_size[0] / 2], [0, f, self.wc_size[1] / 2], [0, 0, 1]])
        self.rng = np.random.default_rng(self.seed)

    # 투구: 놓는 점 → 패드 평면 z=0.15 의 목표점, 비행시간 T
    def throw(self, release: np.ndarray, target: np.ndarray, T: float) -> tuple[np.ndarray, np.ndarray]:
        v0 = (target - release - 0.5 * self.g * T * T) / T
        return release, v0

    def project(self, K: np.ndarray, R: np.ndarray, t: np.ndarray, P: np.ndarray):
        Pc = R @ P + t
        if Pc[2] <= 0.05:
            return None, Pc[2]
        uv = K @ (Pc / Pc[2])
        return uv[:2], Pc[2]


def _stamp(t: float) -> Time:
    s = int(np.floor(t))
    return Time(sec=s, nanosec=int(round((t - s) * 1e9)))


def _info(K: np.ndarray, size: tuple[int, int], frame: str, t: float) -> CameraInfo:
    m = CameraInfo()
    m.header.stamp, m.header.frame_id = _stamp(t), frame
    m.width, m.height = size
    m.k = K.ravel().tolist()
    m.d = [0.0] * 5
    m.distortion_model = "plumb_bob"
    return m


def _img(arr: np.ndarray, enc: str, frame: str, t: float) -> Image:
    m = Image()
    m.header.stamp, m.header.frame_id = _stamp(t), frame
    m.height, m.width = arr.shape[:2]
    m.encoding = enc
    m.step = arr.strides[0]
    m.data = arr.tobytes()
    return m


class Writer:
    def __init__(self, uri: str) -> None:
        self.w = rosbag2_py.SequentialWriter()
        self.w.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
        self.topics: set[str] = set()

    def write(self, topic: str, type_name: str, msg, t: float) -> None:
        if topic not in self.topics:
            self.w.create_topic(rosbag2_py.TopicMetadata(id=len(self.topics), name=topic, type=type_name,
                                                         serialization_format="cdr"))
            self.topics.add(topic)
        self.w.write(topic, serialize_message(msg), int(t * 1e9))


def render_ball(img: np.ndarray, uv: np.ndarray, r_px: float, color=BALL_BGR) -> None:
    cv2.circle(img, (int(round(uv[0] * 16)), int(round(uv[1] * 16))), max(1, int(round(r_px * 16))),
               color, -1, cv2.LINE_AA, shift=4)


def write_bag(uri: str, sc: Scene, track) -> None:
    """track(t) → 공 중심 C 좌표 또는 None. t ∈ [0, track.duration]. RealSense 60 fps, 웹캠 30 fps."""
    wr = Writer(uri)
    T0 = 1000.0                                 # bag 시각 기준
    # tf_static: camera_link 아래에 color/depth/accel 광학 프레임
    tf = TFMessage()
    def tr(child, xyz, R):
        q = _R_to_quat(R)
        m = TransformStamped()
        m.header.stamp, m.header.frame_id, m.child_frame_id = _stamp(T0), "camera_link", child
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = xyz
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
        return m
    tf.transforms = [tr("camera_color_optical_frame", (0.0, 0.0, 0.0), np.eye(3)),
                     tr("camera_depth_optical_frame", (-0.015, 0.0, 0.0), np.eye(3)),
                     tr("camera_accel_optical_frame", (-0.01, 0.005, 0.0), np.eye(3))]
    wr.write("/tf_static", "tf2_msgs/msg/TFMessage", tf, T0)
    R_cd = np.eye(3)
    t_cd = -np.array([-0.015, 0.0, 0.0])        # X_d = X_c − p_depth_in_color

    rng = sc.rng
    events = []
    n_rs = int(track.duration * 60)
    for i in range(n_rs):
        events.append((i / 60.0, "rs"))
        events.append((i / 60.0 + sc.depth_offset_s, "depth"))
    for i in range(int(track.duration * 30)):
        events.append((i / 30.0 + 0.007, "wc"))
    for i in range(int(track.duration * 100)):
        events.append((i / 100.0, "imu"))
    events.sort()
    for t, kind in events:
        P = track(t)
        if kind == "rs":
            img = np.full((sc.rs_size[1], sc.rs_size[0], 3), 70, np.uint8)
            if P is not None:
                uv, z = sc.project(sc.K_rs, np.eye(3), np.zeros(3), P)
                if uv is not None:
                    render_ball(img, uv + rng.normal(0, sc.px_noise, 2), sc.K_rs[0, 0] * BALL_R / z)
            wr.write("/camera/camera/color/image_raw", "sensor_msgs/msg/Image", _img(img[..., ::-1].copy(), "rgb8",
                     "camera_color_optical_frame", T0 + t), T0 + t)
            wr.write("/camera/camera/color/camera_info", "sensor_msgs/msg/CameraInfo",
                     _info(sc.K_rs, sc.rs_size, "camera_color_optical_frame", T0 + t), T0 + t)
        elif kind == "depth":
            d = np.full((sc.rs_size[1], sc.rs_size[0]), 4000, np.uint16)   # 배경 4 m
            if P is not None:
                Pd = R_cd @ P + t_cd
                if Pd[2] > 0.1:
                    uv = sc.K_d @ (Pd / Pd[2])
                    r = sc.K_d[0, 0] * BALL_R / Pd[2]
                    surf = (Pd[2] - BALL_R + rng.normal(0, sc.depth_noise_rel * Pd[2])) * 1000
                    cv2.circle(d, (int(round(uv[0])), int(round(uv[1]))), max(1, int(round(r))), int(surf), -1)
            wr.write("/camera/camera/depth/image_rect_raw", "sensor_msgs/msg/Image",
                     _img(d, "16UC1", "camera_depth_optical_frame", T0 + t), T0 + t)
            wr.write("/camera/camera/depth/camera_info", "sensor_msgs/msg/CameraInfo",
                     _info(sc.K_d, sc.rs_size, "camera_depth_optical_frame", T0 + t), T0 + t)
        elif kind == "wc":
            img = np.full((sc.wc_size[1], sc.wc_size[0], 3), 90, np.uint8)
            if P is not None:
                uv, z = sc.project(sc.K_wc, sc.R_wc, sc.t_wc, P)
                if uv is not None:
                    render_ball(img, uv + rng.normal(0, sc.px_noise, 2), sc.K_wc[0, 0] * BALL_R / z)
            # 웹캠 stamp 는 RealSense 시계보다 dt_true 이르게 찍힌다 (t_rs = t_wc + dt_true)
            t_stamp = T0 + t - sc.dt_true
            wr.write("/webcam/image_raw", "sensor_msgs/msg/Image", _img(img, "bgr8", "webcam_link", t_stamp), T0 + t)
        else:
            m = Imu()
            m.header.stamp, m.header.frame_id = _stamp(T0 + t), "camera_accel_optical_frame"
            a = -sc.g + rng.normal(0, 0.05, 3)                     # 정지 가속도계 = −g
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = a
            wr.write("/camera/camera/accel/sample", "sensor_msgs/msg/Imu", m, T0 + t)
    del wr


def _R_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    w = np.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


class ThrowTrack:
    """손에 든 공(천천히 흔듦) → 놓기 → 포물선 → 패드에 붙음(정지). 여러 번 반복."""

    def __init__(self, sc: Scene, throws: list[tuple[np.ndarray, np.ndarray, float]], gap: float = 1.5):
        self.sc = sc
        self.items = []
        t = 0.6
        for release, target, T in throws:
            p0, v0 = sc.throw(release, target, T)
            self.items.append((t, p0, v0, T, target))
            t += 0.4 + T + gap
        self.duration = t

    def truth(self) -> list[dict]:
        return [dict(t_release=t0, p0=p0, v0=v0, T=T, hit=target) for t0, p0, v0, T, target in self.items]

    def __call__(self, t: float):
        for t0, p0, v0, T, target in self.items:
            if t0 - 0.3 <= t < t0:                  # 손 동작: 위로 가속하며 휘두름 (중력 포물선 아님)
                tau = t - t0                        # 음수. 놓는 순간 속도 0.6·v0, 가속도 위쪽 12 m/s²
                return p0 + 0.6 * v0 * tau + 0.5 * np.array([0.0, -12.0, 0.0]) * tau * tau
            if t0 <= t <= t0 + T:
                tau = t - t0
                return p0 + v0 * tau + 0.5 * self.sc.g * tau * tau
            if t0 + T < t <= t0 + T + 0.6:          # 패드에 붙어 있음
                return target
        return None


class WandTrack:
    """캘리브레이션용: 공을 들고 두 카메라가 모두 보는 공간을 천천히 훑는다 (멈칫멈칫)."""

    def __init__(self, sc: Scene, n: int = 30, seed: int = 1):
        rng = np.random.default_rng(seed)
        self.pts = np.column_stack([rng.uniform(-0.6, 0.6, n), rng.uniform(-0.5, 0.35, n), rng.uniform(1.0, 2.6, n)])
        self.hold, self.move = 0.5, 0.25
        self.duration = n * (self.hold + self.move)

    def __call__(self, t: float):
        k = int(t // (self.hold + self.move))
        s = t - k * (self.hold + self.move)
        if k >= len(self.pts) - 1:
            return self.pts[-1]
        if s < self.hold:
            return self.pts[k]
        a = (s - self.hold) / self.move
        a = a * a * (3 - 2 * a)
        return (1 - a) * self.pts[k] + a * self.pts[k + 1]
