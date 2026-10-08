#!/usr/bin/env python3
"""몸 자세(body_pose) 확인 창 — 카메라 화면 + BodyPose 값 + 항목별 통과 여부.

body_pose 노드가 켜진 상태에서 실행. 노드를 켠 뒤 처음 2초는 바르게 앉아 있기 (어깨 들림 기준).
    python3 scripts/check_body_live.py                              # /webcam/image_raw
키: q 종료(터미널에 결과 요약) · r 다시
"""
import argparse
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rally_interfaces.msg import BodyPose

WIN = "check_body_live (q quit, r reset)"
GREEN, GRAY, WHITE, RED, YELLOW = (80, 200, 80), (150, 150, 150), (240, 240, 240), (60, 60, 230), (60, 200, 230)


class Checker(Node):
    def __init__(self, image_topic: str) -> None:
        super().__init__("rally_check_body_live")
        self.bridge, self.lock = CvBridge(), threading.Lock()
        self.frame, self.last = None, None
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(BodyPose, "/rally/perception/body", self._on_body, qos_profile_sensor_data)
        self.reset()

    def reset(self) -> None:
        with getattr(self, "lock", threading.Lock()):
            self.n = self.n_det = 0
            self.upright = False          # 기울기 5° 안을 1 s 유지
            self.upright_since = None
            self.lean_max = 0.0
            self.lean_flag = self.hike_flag = False
            self.hike_max = 0.0
            self.elbow_min, self.elbow_max = math.inf, -math.inf

    def _on_image(self, m: Image) -> None:
        img = self.bridge.imgmsg_to_cv2(m, "bgr8")
        with self.lock:
            self.frame = img

    def _on_body(self, m: BodyPose) -> None:
        now = time.monotonic()
        with self.lock:
            self.last = m
            self.n += 1
            if not m.detected:
                return
            self.n_det += 1
            lean, hike, elbow = m.trunk_lean_deg, m.shoulder_elevation_deg, m.elbow_extension_deg
            if math.isfinite(lean):
                self.lean_max = max(self.lean_max, lean)
                if lean < 5:
                    self.upright_since = self.upright_since or now
                    if now - self.upright_since >= 1.0:
                        self.upright = True
                else:
                    self.upright_since = None
            if math.isfinite(hike):
                self.hike_max = max(self.hike_max, hike)
            if math.isfinite(elbow):
                self.elbow_min, self.elbow_max = min(self.elbow_min, elbow), max(self.elbow_max, elbow)
            self.lean_flag |= m.compensation_type == "trunk_lean"
            self.hike_flag |= m.compensation_type == "shoulder_hike"

    def items(self) -> list[tuple[str, bool, str]]:
        det = 100 * self.n_det / self.n if self.n else 0.0
        em = (f"{self.elbow_min:.0f}~{self.elbow_max:.0f}°" if math.isfinite(self.elbow_min) else "-")
        return [
            ("1 몸 검출", self.n >= 30 and det >= 90, f"{det:.0f}% ({self.n_det}/{self.n}) — 1 m 이상 떨어져 앉기"),
            ("2 바르게 앉기", self.upright, "기울기 5° 안을 1초 유지"),
            ("3 몸통 기울임 → 보상", self.lean_flag, f"최대 {self.lean_max:.0f}° (15° 넘게 옆으로 기울이기)"),
            ("4 어깨 으쓱 → 보상", self.hike_flag, f"최대 {self.hike_max:.0f}° (10° 넘게, 몸은 바르게)"),
            ("5 팔꿈치 폈다 굽히기", self.elbow_max >= 160 and self.elbow_min <= 100, f"범위 {em} (≥160 · ≤100 다 보기)"),
        ]


def put(img, text, org, color=WHITE) -> None:
    """한글은 OpenCV 기본 글꼴로 안 나와서 PIL 로 그림 (check_hand_live 와 같은 방식)."""
    from PIL import Image as PImage, ImageDraw, ImageFont
    global _FONT
    try:
        _FONT
    except NameError:
        try:
            _FONT = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 18)
        except OSError:
            _FONT = ImageFont.load_default()
    pil = PImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    ImageDraw.Draw(pil).text(org, text, font=_FONT, fill=color[::-1])
    img[:] = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


def bar(panel, y, label, v, vmax, thr, unit="°") -> None:
    cv2.rectangle(panel, (10, y + 22), (390, y + 38), (70, 70, 70), -1)
    if math.isfinite(v):
        w = int(380 * max(0.0, min(1.0, v / vmax)))
        cv2.rectangle(panel, (10, y + 22), (10 + w, y + 38), RED if thr is not None and v >= thr else (80, 160, 230), -1)
    if thr is not None:
        x = 10 + int(380 * thr / vmax)
        cv2.line(panel, (x, y + 18), (x, y + 42), YELLOW, 2)
    put(panel, f"{label}  {v:.0f}{unit}" if math.isfinite(v) else f"{label}  -", (10, y), WHITE)


def render(node: Checker) -> np.ndarray:
    with node.lock:
        frame = None if node.frame is None else node.frame.copy()
        last, items = node.last, node.items()
    if frame is None:
        frame = np.zeros((480, 640, 3), np.uint8)
    frame = cv2.resize(frame, (640, int(frame.shape[0] * 640 / frame.shape[1])))
    if frame.shape[0] < 480:
        frame = cv2.copyMakeBorder(frame, 0, 480 - frame.shape[0], 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    panel = np.full((frame.shape[0], 420, 3), 30, np.uint8)
    if last is None:
        put(panel, "BodyPose 없음 — body_pose 노드 켰는지", (10, 10), RED)
    elif not last.detected:
        put(panel, "사람 없음 — 1 m 이상 떨어져 어깨가 보이게", (10, 10), GRAY)
    else:
        kind = last.compensation_type
        put(panel, {"none": "보상 동작 없음", "trunk_lean": "보상: 몸통 기울임", "shoulder_hike": "보상: 어깨 으쓱"}.get(kind, kind),
            (10, 6), RED if last.compensation_detected else GREEN)
        bar(panel, 34, "몸통 기울기", last.trunk_lean_deg, 40, 15)
        bar(panel, 80, "어깨 들림", last.shoulder_elevation_deg, 30, 10)
        bar(panel, 126, "팔꿈치 펴짐", last.elbow_extension_deg, 180, None)
    y = 186
    for name, ok, info in items:
        put(panel, ("[통과] " if ok else "[ ] ") + name, (10, y), GREEN if ok else WHITE)
        put(panel, info, (30, y + 22), GRAY)
        y += 52
    return np.hstack([frame, panel])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default="/webcam/image_raw")
    a = ap.parse_args()
    rclpy.init()
    node = Checker(a.image)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    try:
        while True:
            cv2.imshow(WIN, render(node))
            k = cv2.waitKey(30) & 0xFF
            if k == ord("q"):
                break
            if k == ord("r"):
                node.reset()
    except KeyboardInterrupt:
        pass
    with node.lock:
        items = node.items()
    node.get_logger().info(f"결과 {sum(ok for _, ok, _ in items)}/{len(items)}\n"
                           + "\n".join(f"  {'통과' if ok else '미완'}  {n}: {i}" for n, ok, i in items))
    cv2.destroyAllWindows()
    ex.shutdown()
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
