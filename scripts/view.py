#!/usr/bin/env python3
"""비전 확인 창 — segmenter 가 그린 마스크·바운딩 박스(/rally/perception/objects/debug_image)를 띄운다.
손 21점(HandState.landmarks, base_link)을 tcp_pose + T_gripper2camera 로 화면에 투영해 같이 그리고 (손 노드가 떠 있을 때),
아래에 안전 상태·손 거리·Handover 단계를 한 줄 덧붙인다. q 또는 Esc 로 닫기.

    source scripts/robot.sh env && python3 scripts/view.py      # (scripts/robot.sh session 이 자동으로 띄움, VIEW=0 이면 안 띄움)
debug_image 는 구독자가 있을 때만 10 Hz 로 나온다 (segmenter debug_hz). conf 0.5 미만 검출은 그려지지 않는다.
"""
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from rally_interfaces.action import Handover
from rally_interfaces.msg import HandState, SafetyStatus

HAND_LINKS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
              (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]
G2C_PATHS = [os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "T_gripper2camera.npy")]


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_g2c():
    try:
        from rally_robot.dsr_helpers import load_gripper2cam
        return load_gripper2cam()
    except Exception:
        for p in G2C_PATHS:
            if os.path.exists(p):
                return np.load(p)
    return None


LEVELS = {SafetyStatus.LEVEL_NORMAL: "NORMAL", SafetyStatus.LEVEL_SLOW: "SLOW", SafetyStatus.LEVEL_STOP: "STOP"}


class View(Node):
    def __init__(self):
        super().__init__("rally_view")
        self.img, self.safety, self.hand, self.phase, self.t_img = None, None, None, "", 0.0
        self.K, self.tcp, self.g2c = None, None, load_g2c()
        self.create_subscription(CameraInfo, "/camera/color/camera_info", lambda m: setattr(self, "K", np.array(m.k).reshape(3, 3)),
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/rally/robot/tcp_pose", lambda m: setattr(self, "tcp", m), qos_profile_sensor_data)
        self.create_subscription(Image, "/rally/perception/objects/debug_image", self.on_img, qos_profile_sensor_data)
        self.create_subscription(SafetyStatus, "/rally/safety/status", lambda m: setattr(self, "safety", m),
                                 QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10))
        self.create_subscription(HandState, "/rally/perception/hand", lambda m: setattr(self, "hand", m), qos_profile_sensor_data)
        self.create_subscription(Handover.Impl.FeedbackMessage, "/rally/skill/handover/_action/feedback",
                                 lambda m: setattr(self, "phase", m.feedback.phase), 10)

    def on_img(self, m: Image):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        self.img = a[:, :, ::-1].copy() if m.encoding == "rgb8" else a.copy()
        self.t_img = time.monotonic()

    def draw_hand(self, vis):
        """손 21점 투영. base → 카메라 = (base → TCP(tcp_pose) · TCP → 카메라(T_gripper2camera, mm))⁻¹."""
        h = self.hand
        if h is None or not h.detected or self.K is None or self.tcp is None or self.g2c is None:
            return
        p = self.tcp.pose
        b2t = np.eye(4)
        b2t[:3, :3], b2t[:3, 3] = quat_to_rot(p.orientation), [p.position.x, p.position.y, p.position.z]
        g2c = np.array(self.g2c, float)
        g2c[:3, 3] /= 1000.0
        c_from_b = np.linalg.inv(b2t @ g2c)
        uv = []
        for lm in h.landmarks:
            q = c_from_b @ np.array([lm.x, lm.y, lm.z, 1.0])
            if not np.isfinite(q).all() or q[2] < 0.05:
                uv.append(None)
                continue
            a = self.K @ q[:3]
            uv.append((int(a[0] / a[2]), int(a[1] / a[2])))
        for i, j in HAND_LINKS:
            if uv[i] and uv[j]:
                cv2.line(vis, uv[i], uv[j], (0, 255, 0), 2, cv2.LINE_AA)
        for k in uv:
            if k:
                cv2.circle(vis, k, 4, (0, 0, 255), -1)
        if uv[5] and uv[17]:
            cv2.line(vis, uv[5], uv[17], (255, 0, 255), 4, cv2.LINE_AA)       # 손가락 뿌리 줄

    def status(self) -> str:
        s = self.safety
        txt = f"safety {LEVELS.get(s.level, s.level)} {s.reason}" if s else "safety -"
        h = self.hand
        if h and h.detected:
            d = h.distance_to_centerline
            txt += f" | hand line {d:+.2f} m wrist {h.wrist_angle_deg:.0f}" if math.isfinite(d) else " | hand (no depth)"
        else:
            txt += " | hand -"
        return txt + (f" | handover {self.phase}" if self.phase else "")


def main():
    rclpy.init()
    n = View()
    win = "RALLY vision"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.03)
            if n.img is None:
                vis = np.zeros((480, 848, 3), np.uint8)
                cv2.putText(vis, "debug_image waiting (vision container?)", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            else:
                vis = n.img.copy()
                n.draw_hand(vis)
                if time.monotonic() - n.t_img > 1.0:
                    cv2.putText(vis, "STALE", (vis.shape[1] - 90, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.rectangle(vis, (0, vis.shape[0] - 24), (vis.shape[1], vis.shape[0]), (0, 0, 0), -1)
            cv2.putText(vis, n.status(), (8, vis.shape[0] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(win, vis)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
