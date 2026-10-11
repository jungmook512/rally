#!/usr/bin/env python3
"""손 인식 체크리스트 3번 — 노드가 낸 손 거리가 RealSense 원본 깊이와 맞는지 (줄자 없이).

HandState.palm_pose(카메라 좌표, tf 없을 때)를 카메라 내부값으로 픽셀에 다시 찍고, 그 픽셀 주변(7×7)의
정렬 깊이 중간값과 비교한다. 같은 시각의 깊이 프레임을 쓴다. 손을 30~80 cm 사이에서 천천히 움직이면 된다.
    python3 scripts/check_hand_depth.py
    python3 scripts/check_hand_depth.py --depth-scale 0.971   # 손 노드가 hand_eye.yaml depth_scale 로 보정할 때 (#77)
통과: 100 프레임 이상에서 |노드 z − 원본 깊이 × depth_scale| 중간값 ≤ 5 mm
  — 원본 깊이(센서)는 보정 전 값이라, 보정이 들어가면 노드 z 는 원본보다 depth_scale 만큼 짧아야 맞다
키: q 종료(터미널에 요약) · r 다시
"""
import threading
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from rally_interfaces.msg import HandState

WIN = "check_hand_depth (q quit, r reset)"
COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"
INFO = "/camera/camera/color/camera_info"


class Depth(Node):
    def __init__(self, depth_scale: float = 1.0) -> None:
        super().__init__("rally_check_hand_depth")
        self.bridge, self.lock = CvBridge(), threading.Lock()
        self.depth_scale = depth_scale
        self.color, self.K, self.hand = None, None, None
        self.depths: deque = deque(maxlen=15)
        self.diffs: list[float] = []
        self.zs: list[float] = []
        self.note = "손을 카메라 앞 30~80 cm 에"
        self.create_subscription(Image, COLOR, self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, INFO, self._on_info, qos_profile_sensor_data)
        self.create_subscription(HandState, "/rally/perception/hand", self._on_hand, qos_profile_sensor_data)

    def _on_color(self, m: Image) -> None:
        with self.lock:
            self.color = self.bridge.imgmsg_to_cv2(m, "bgr8")

    def _on_depth(self, m: Image) -> None:
        with self.lock:
            self.depths.append(m)

    def _on_info(self, m: CameraInfo) -> None:
        self.K = np.array(m.k).reshape(3, 3)

    def _on_hand(self, m: HandState) -> None:
        if not m.detected or self.K is None:
            return
        if not m.header.frame_id.startswith("camera"):
            self.note = f"좌표가 {m.header.frame_id} — 카메라 좌표일 때만 비교 (tf 끄고 실행)"
            return
        t = Time.from_msg(m.header.stamp).nanoseconds
        with self.lock:
            cands = list(self.depths)
        if not cands:
            return
        d = min(cands, key=lambda x: abs(Time.from_msg(x.header.stamp).nanoseconds - t))
        if abs(Time.from_msg(d.header.stamp).nanoseconds - t) > 50e6:
            return
        p = m.palm_pose.position
        if not np.isfinite(p.z) or p.z <= 0:
            return
        u = int(round(self.K[0, 0] * p.x / p.z + self.K[0, 2]))
        v = int(round(self.K[1, 1] * p.y / p.z + self.K[1, 2]))
        depth = self.bridge.imgmsg_to_cv2(d)
        h, w = depth.shape[:2]
        if not (3 <= u < w - 3 and 3 <= v < h - 3):
            return
        win = depth[v - 3:v + 4, u - 3:u + 4].astype(np.float32).ravel()
        win = win[win > 0]
        if len(win) < 10:
            return
        raw = float(np.median(win)) * 1e-3
        with self.lock:
            self.hand = (u, v, p.z, raw)
            self.diffs.append((p.z - raw * self.depth_scale) * 1000)
            self.zs.append(raw)

    def summary(self) -> tuple[bool, str]:
        n = len(self.diffs)
        if not n:
            return False, "샘플 0 — 손을 보여 주세요"
        a = np.abs(self.diffs)
        ok = n >= 100 and np.median(a) <= 5.0
        return ok, (f"{n} 프레임  |차이| 중간값 {np.median(a):.1f} mm · 95% {np.percentile(a, 95):.1f} mm  "
                    f"(거리 {min(self.zs) * 100:.0f}~{max(self.zs) * 100:.0f} cm)")


def main() -> None:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--depth-scale", type=float, default=1.0, help="손 노드가 깊이에 곱하는 배율 (hand_eye.yaml depth_scale)")
    a = ap.parse_args()
    rclpy.init()
    node = Depth(a.depth_scale)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    while True:
        with node.lock:
            img = None if node.color is None else node.color.copy()
            hand = node.hand
            ok, text = node.summary()
        if img is None:
            img = np.zeros((480, 848, 3), np.uint8)
            cv2.putText(img, "no color image", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        if hand:
            u, v, z, raw = hand
            cv2.circle(img, (u, v), 8, (0, 255, 255), 2)
            exp = raw * node.depth_scale
            cv2.putText(img, f"node {z * 100:.1f} cm  raw x{node.depth_scale:g} {exp * 100:.1f} cm  diff {(z - exp) * 1000:+.0f} mm",
                        (u + 12, v), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(img, ("PASS  " if ok else "....  ") + text.encode("ascii", "ignore").decode(), (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 200, 80) if ok else (255, 255, 255), 2)
        cv2.imshow(WIN, img)
        k = cv2.waitKey(30) & 0xFF
        if k == ord("q"):
            break
        if k == ord("r"):
            with node.lock:
                node.diffs.clear()
                node.zs.clear()
    with node.lock:
        ok, text = node.summary()
    node.get_logger().info(f"3번 손 거리: {'통과' if ok else '미완'} — {text}  (기준: 100 프레임 이상, |노드 − 원본×{node.depth_scale:g}| 중간값 ≤ 5 mm)"
                           + ("" if node.diffs else f"  [{node.note}]"))
    cv2.destroyAllWindows()
    ex.shutdown()
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
