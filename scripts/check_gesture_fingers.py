#!/usr/bin/env python3
"""제스처 인식률 측정 — 손가락별 접힘 값과 프레임별 판정을 보여 주고, 목표 제스처를 몇 % 맞히는지 잰다.

hand_pose 노드와 같은 MediaPipe 모델·같은 판정 함수(rally_human.hand_geometry)를 영상에 직접 돌린다.
    python3 scripts/check_gesture_fingers.py --image /webcam/image_raw
키: 1 주먹 · 2 펼침 · 3 검지 → 그 제스처를 하고 있다고 알려 주면 측정 시작 (다시 누르면 멈춤)
    q 종료 → 터미널에 제스처별 인식률과 손가락 접힘 중간값
"""
import argparse
import os
import threading
import time
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rally_human import hand_geometry as hg

WIN = "check_gesture_fingers (1 fist 2 open 3 point, q quit)"
NAMES = {0: "NONE", 1: "FIST", 2: "OPEN", 3: "POINT"}
MODELS = ["/opt/mediapipe/hand_landmarker.task", os.path.expanduser("~/rally/models/mediapipe/hand_landmarker.task")]


class Sub(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("rally_check_gesture_fingers")
        self.bridge, self.lock, self.frame, self.seq = CvBridge(), threading.Lock(), None, 0
        self.create_subscription(Image, topic, self._on, qos_profile_sensor_data)

    def _on(self, m: Image) -> None:
        img = self.bridge.imgmsg_to_cv2(m, "bgr8")
        with self.lock:
            self.frame = img
            self.seq += 1


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default="/camera/camera/color/image_raw")
    a = ap.parse_args()
    from mediapipe.tasks.python import BaseOptions, vision
    model = next(p for p in MODELS if Path(p).exists())
    lmk = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=model), running_mode=vision.RunningMode.VIDEO,
        num_hands=2, min_hand_detection_confidence=0.5, min_tracking_confidence=0.5))
    rclpy.init()
    node = Sub(a.image)
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    target, last_seq = 0, -1
    stats: dict[int, dict] = {g: {"frames": 0, "hand": 0, "hit": 0, "c": []} for g in (1, 2, 3)}
    t0 = time.monotonic()
    while True:
        with node.lock:
            img = None if node.frame is None or node.seq == last_seq else node.frame.copy()
            seq = node.seq
        if img is None:                      # 새 프레임이 올 때만 센다 (같은 프레임 중복 집계 방지)
            k = cv2.waitKey(5) & 0xFF
            if k == ord("q"):
                break
            continue
        last_seq = seq
        res = lmk.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)),
                                   int((time.monotonic() - t0) * 1000))
        raw, c = None, None
        if res.hand_landmarks:
            sizes = [np.ptp([p.x for p in h]) * np.ptp([p.y for p in h]) for h in res.hand_landmarks]
            i = int(np.argmax(sizes))
            world = np.array([[p.x, p.y, p.z] for p in res.hand_world_landmarks[i]])
            c = {f: hg.finger_closure(world, f) for f in hg.FINGERS}
            raw = hg.classify_gesture(world)
            H, W = img.shape[:2]
            for p in res.hand_landmarks[i]:
                cv2.circle(img, (int(p.x * W), int(p.y * H)), 3, (0, 255, 255), -1)
        if target:
            s = stats[target]
            s["frames"] += 1
            if raw is not None:
                s["hand"] += 1
                s["hit"] += raw == target
                s["c"].append([c[f] for f in hg.FINGERS])
        # 손가락별 막대: 초록 구간 = 펼침 기준(≤0.25), 빨강 선 = 접힘 기준(0.6 / 주먹 0.75)
        y = 20
        for f in hg.FINGERS:
            v = c[f] if c else 0.0
            cv2.putText(img, f"{f:6s} {v:.2f}", (10, y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.rectangle(img, (130, y), (330, y + 14), (60, 60, 60), -1)
            cv2.rectangle(img, (130, y), (130 + int(200 * v), y + 14), (80, 160, 230), -1)
            for th, col in ((0.25, (80, 200, 80)), (0.6, (60, 60, 230)), (0.75, (0, 0, 160))):
                x = 130 + int(200 * th)
                cv2.line(img, (x, y - 2), (x, y + 16), col, 1)
            y += 22
        cv2.putText(img, f"raw: {NAMES.get(raw, 'no hand') if raw is not None else 'no hand'}", (10, y + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if target:
            s = stats[target]
            rate = 100 * s["hit"] / s["hand"] if s["hand"] else 0
            cv2.putText(img, f"measuring {NAMES[target]}: {rate:.0f}% of {s['hand']} frames", (10, y + 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 200, 80), 2)
        cv2.imshow(WIN, img)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k in (ord("1"), ord("2"), ord("3")):
            g = k - ord("0")
            target = 0 if target == g else g
    lines = []
    for g, s in stats.items():
        if not s["frames"]:
            continue
        med = np.median(np.array(s["c"]), axis=0) if s["c"] else [float("nan")] * 4
        lines.append(f"  {NAMES[g]:5s} 인식 {100 * s['hit'] / max(1, s['hand']):3.0f}% ({s['hit']}/{s['hand']} 프레임, "
                     f"손 검출 {100 * s['hand'] / s['frames']:.0f}%)  접힘 중간값 "
                     + " ".join(f"{f}={m:.2f}" for f, m in zip(hg.FINGERS, med)))
    node.get_logger().info("결과\n" + ("\n".join(lines) if lines else "  측정 없음 (1·2·3 키로 시작)"))
    cv2.destroyAllWindows()
    ex.shutdown()
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
