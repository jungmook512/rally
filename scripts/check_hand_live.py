#!/usr/bin/env python3
"""손 인식 체크리스트 1·2·4·5·7번 자동 판정 창 (6번 주먹 정지는 #59 에서 뺌) — 카메라 화면 + HandState 값 + 항목별 통과 여부.

hand_pose 노드가 켜진 상태에서 실행. 창 안내대로 손을 움직이면 항목이 하나씩 초록(통과)으로 바뀐다.
    python3 scripts/check_hand_live.py --image /webcam/image_raw --min-hz 15     # 노트북 웹캠 (20 fps)
    python3 scripts/check_hand_live.py                                         # RealSense (30 Hz 기준)
키: q 종료(터미널에 결과 요약) · r 다시 시작
"""
import argparse
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rally_interfaces.msg import HandState

WIN = "check_hand_live (q quit, r reset)"   # 창 제목은 영어로 (한글 제목이면 창이 안 뜨는 환경이 있음)
GESTURE_NAME = {0: "NONE", 1: "FIST", 2: "OPEN", 3: "POINT"}
GREEN, GRAY, WHITE, RED = (80, 200, 80), (150, 150, 150), (240, 240, 240), (60, 60, 230)


class Checker(Node):
    def __init__(self, image_topic: str, min_hz: float) -> None:
        super().__init__("rally_check_hand_live")
        self.min_hz = min_hz
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame: np.ndarray | None = None
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(HandState, "/rally/perception/hand", self._on_hand, qos_profile_sensor_data)
        self.reset()

    def reset(self) -> None:
        with self.lock:
            self.t_start = time.monotonic()
            self.stamps: deque = deque(maxlen=200)
            self.last: HandState | None = None
            self.no_hand_since: float | None = None
            self.no_hand_ok = False
            self.grip_min, self.grip_max = 1.0, 0.0
            self.gestures_seen: set[int] = set()
            self.wrist_bins = {"아래(≤30)": False, "옆(60~120)": False, "위(≥150)": False}

    # ----- 입력 -----
    def _on_image(self, m: Image) -> None:
        img = self.bridge.imgmsg_to_cv2(m, "bgr8")
        with self.lock:
            self.frame = img

    def _on_hand(self, m: HandState) -> None:
        now = time.monotonic()
        with self.lock:
            self.stamps.append(now)
            self.last = m
            if not m.detected:
                self.no_hand_since = self.no_hand_since or now
                if now - self.no_hand_since >= 2.0:
                    self.no_hand_ok = True
                return
            self.no_hand_since = None
            self.grip_min = min(self.grip_min, m.grip_closure)
            self.grip_max = max(self.grip_max, m.grip_closure)
            if m.gesture:
                self.gestures_seen.add(m.gesture)
            w = m.wrist_angle_deg
            if w <= 30:
                self.wrist_bins["아래(≤30)"] = True
            elif 60 <= w <= 120:
                self.wrist_bins["옆(60~120)"] = True
            elif w >= 150:
                self.wrist_bins["위(≥150)"] = True

    # ----- 판정 -----
    def hz(self) -> float:
        s = [t for t in self.stamps if time.monotonic() - t < 2.0]
        return (len(s) - 1) / (s[-1] - s[0]) if len(s) > 2 and s[-1] > s[0] else 0.0

    def items(self) -> list[tuple[str, bool, str]]:
        """(항목, 통과, 지금 값·할 일)"""
        hz = self.hz()
        names = {1: "FIST", 2: "OPEN", 3: "POINT"}
        seen = " ".join(n for g, n in names.items() if g in self.gestures_seen)
        todo_w = ", ".join(k for k, v in self.wrist_bins.items() if not v)
        return [
            ("1 발행 속도", hz >= self.min_hz, f"{hz:.1f} Hz (기준 {self.min_hz:.0f})"),
            ("2 손 없을 때", self.no_hand_ok, "손을 2초 치우기"),
            ("4 쥔 정도", self.grip_min <= 0.2 and self.grip_max >= 0.8,
             f"최소 {self.grip_min:.2f}(≤0.2) 최대 {self.grip_max:.2f}(≥0.8)"),
            ("5 제스처", len(self.gestures_seen & {1, 2, 3}) == 3, f"본 것: {seen or '-'} / 주먹·펼침·검지"),
            ("7 손목 각도", all(self.wrist_bins.values()), (f"남은 것: {todo_w}" if todo_w else "아래·옆·위 다 봄")
             + "  (손가락은 카메라 쪽·옆으로 눕혀서, 위로 세우면 각도 없음)"),
        ]


def put(img, text, org, color=WHITE, scale=0.55, thick=1) -> None:
    """한글은 OpenCV 기본 글꼴로 안 나와서 PIL 로 그림."""
    from PIL import Image as PImage, ImageDraw, ImageFont
    global _FONT
    try:
        _FONT
    except NameError:
        try:
            _FONT = ImageFont.truetype("/usr/share/fonts/truetype/nanum/NanumGothic.ttf", 18)
        except OSError:
            try:
                _FONT = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 18)
            except OSError:
                _FONT = ImageFont.load_default()
    pil = PImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    ImageDraw.Draw(pil).text(org, text, font=_FONT, fill=color[::-1])
    img[:] = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


def render(node: Checker) -> np.ndarray:
    with node.lock:
        frame = None if node.frame is None else node.frame.copy()
        last = node.last
        items = node.items()
    if frame is None:
        frame = np.zeros((480, 640, 3), np.uint8)
    frame = cv2.resize(frame, (640, int(frame.shape[0] * 640 / frame.shape[1])))
    if frame.shape[0] < 480:            # RealSense 848×480 은 줄이면 높이 362 → 아래 항목이 잘리지 않게 아래를 채움
        frame = cv2.copyMakeBorder(frame, 0, 480 - frame.shape[0], 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    panel = np.full((frame.shape[0], 420, 3), 30, np.uint8)
    y = 10
    if last is None:
        put(panel, "HandState 없음 — hand_pose 노드 켰는지", (10, y), RED)
    elif not last.detected:
        put(panel, "손 없음", (10, y), GRAY)
    else:
        z = last.palm_pose.position.z
        if last.header.frame_id == "base_link":
            depth = "base_link 좌표"                     # tf 가 있으면 카메라 거리가 아님
        elif np.isfinite(z) and z > 0:
            depth = f"카메라 {z * 100:.0f} cm"
        else:
            depth = "깊이 없음"                          # 깊이 못 구한 프레임 (손목 각도는 손 모양으로 계산)
        put(panel, f"{GESTURE_NAME.get(last.gesture, '?')}  손목 {last.wrist_angle_deg:.0f}°  {depth}", (10, y), WHITE)
        y += 28
        bar = int(380 * max(0.0, min(1.0, last.grip_closure)))
        cv2.rectangle(panel, (10, y), (390, y + 16), (70, 70, 70), -1)
        cv2.rectangle(panel, (10, y), (10 + bar, y + 16), (80, 160, 230), -1)
        put(panel, f"쥔 정도 {last.grip_closure:.2f}", (14, y - 2), WHITE)
    y = 80
    for name, ok, info in items:
        put(panel, ("[통과] " if ok else "[ ] ") + name, (10, y), GREEN if ok else WHITE)
        put(panel, info, (30, y + 22), GRAY)
        y += 56
    put(panel, "q 종료 · r 다시", (10, panel.shape[0] - 26), GRAY)
    return np.hstack([frame, panel])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default="/camera/camera/color/image_raw")
    ap.add_argument("--min-hz", type=float, default=25.0)
    a = ap.parse_args()
    rclpy.init()
    node = Checker(a.image, a.min_hz)
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1060, 480)
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
    passed = sum(ok for _, ok, _ in items)
    lines = [f"  {'통과' if ok else '미완'}  {n}: {info}" for n, ok, info in items]
    node.get_logger().info(f"결과 {passed}/{len(items)}\n" + "\n".join(lines))
    cv2.destroyAllWindows()
    ex.shutdown()                    # 받는 스레드를 먼저 멈춰야 종료 때 core dump 가 안 남
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
