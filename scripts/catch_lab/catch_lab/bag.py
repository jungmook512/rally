"""bag 읽기: 이미지 디코드, tf_static 으로 프레임 간 변환, IMU 로 중력 방향."""
from collections.abc import Iterator

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def open_reader(uri: str, topics: list[str] | None = None) -> tuple[rosbag2_py.SequentialReader, dict[str, str]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    return reader, types


def iter_messages(uri: str, topics: list[str], start_stamp: float | None = None) -> Iterator[tuple[str, object]]:
    """(topic, msg) 를 bag 기록 순서대로. 없는 토픽은 조용히 건너뛴다.
    start_stamp(header 시각, 초)를 주면 그 0.5 초 전쯤으로 건너뛴다 (큰 bag 에서 뒤쪽 구간 읽기용)."""
    reader, types = open_reader(uri, topics)
    classes = {t: get_message(types[t]) for t in topics if t in types}
    if start_stamp is not None and reader.has_next():
        # bag 기록 시각과 header 시각의 차이를 첫 메시지로 재서 보정
        topic, data, t_rec = reader.read_next()
        offset = t_rec * 1e-9 - stamp(deserialize_message(data, classes[topic])) if topic in classes else 0.0
        reader.seek(int((start_stamp + offset - 0.5) * 1e9))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in classes:
            yield topic, deserialize_message(data, classes[topic])


def topic_types(uri: str) -> dict[str, str]:
    return open_reader(uri)[1]


def stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def image_to_numpy(msg) -> np.ndarray:
    """sensor_msgs/Image → BGR uint8 (컬러) 또는 float32 미터 (깊이)."""
    import cv2
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    h, w = msg.height, msg.width
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img.copy()
    if enc in ("mono8", "8uc1"):
        return cv2.cvtColor(buf.reshape(h, msg.step)[:, :w].copy(), cv2.COLOR_GRAY2BGR)
    if enc in ("yuv422_yuy2", "yuyv", "yuv422"):
        img = buf.reshape(h, msg.step)[:, :w * 2].reshape(h, w, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
    if enc in ("16uc1", "mono16"):
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, msg.step // 2)[:, :w]
        return d.astype(np.float32) * 1e-3          # RealSense 깊이 단위 mm
    if enc == "32fc1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(h, msg.step // 4)[:, :w].copy()
    raise ValueError(f"지원하지 않는 인코딩: {msg.encoding}")


def _quat_to_R(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class StaticTF:
    """tf_static 만으로 프레임 간 변환. T_parent_child 를 모아 루트까지 합성."""

    def __init__(self) -> None:
        self.edges: dict[str, tuple[str, np.ndarray]] = {}   # child → (parent, 4x4 T_parent_child)

    def add(self, tf_msg) -> None:
        for tr in tf_msg.transforms:
            T = np.eye(4)
            T[:3, :3] = _quat_to_R(tr.transform.rotation)
            v = tr.transform.translation
            T[:3, 3] = [v.x, v.y, v.z]
            self.edges[tr.child_frame_id.lstrip("/")] = (tr.header.frame_id.lstrip("/"), T)

    def _to_root(self, frame: str) -> tuple[str, np.ndarray]:
        T = np.eye(4)
        seen = set()
        while frame in self.edges and frame not in seen:
            seen.add(frame)
            parent, T_pc = self.edges[frame]
            T = T_pc @ T
            frame = parent
        return frame, T

    def lookup(self, target: str, source: str) -> np.ndarray | None:
        """T_target_source (source 좌표 → target 좌표). 연결 안 되면 None."""
        rt, T_rt = self._to_root(target)
        rs, T_rs = self._to_root(source)
        if rt != rs:
            return None
        return np.linalg.inv(T_rt) @ T_rs


def gravity_from_accel(samples: np.ndarray, T_color_accel: np.ndarray | None) -> np.ndarray:
    """가속도계 평균 (정지 상태에서 위쪽 +9.81) → 컬러 광학 프레임의 중력 벡터 (아래 방향, 크기 9.81)."""
    a = np.median(samples, axis=0)
    g_accel = -a / np.linalg.norm(a) * 9.81
    R = T_color_accel[:3, :3] if T_color_accel is not None else np.eye(3)
    return R @ g_accel
