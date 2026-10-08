#!/usr/bin/env python3
"""몸 자세(body_pose) 확인 창 — 카메라 화면 + BodyPose 값 + 항목별 통과 여부.

body_pose 노드가 켜진 상태에서 실행. 자리를 잡고 바르게 앉은 뒤 b → 2초 가만히 (어깨 들림 기준을 다시 잡음).
    python3 scripts/check_body_live.py                              # /webcam/image_raw
    python3 scripts/check_body_live.py --no-skeleton                # 뼈대 안 그림 (CPU 아끼기)
키: b 기준 다시 잡기 · q 종료(터미널에 결과 요약) · r 판정 다시
뼈대: BodyPose 메시지에는 관절이 없어서, 이 창이 같은 영상에 MediaPipe Pose 를 한 번 더 돌려 그린다.
  점 색 = visibility (초록 ≥ 0.7 · 노랑 ≥ 0.5 · 빨강 < 0.5 = 가려짐을 추정한 점). 팔꿈치 옆 숫자 = 그 팔의 펴짐 각(3D).
  왼쪽 위 = 몸통 기울기를 어떤 방법으로 쟀는지 (torso: 엉덩이→어깨 / shoulders: 어깨선, 측면에선 앞으로 숙이기를 못 잼).
"""
import argparse
import math
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from rally_interfaces.msg import BodyPose
from rally_human import body_geometry as bg

WIN = "check_body_live (b baseline, q quit, r reset)"
GREEN, GRAY, WHITE, RED, YELLOW = (80, 200, 80), (150, 150, 150), (240, 240, 240), (60, 60, 230), (60, 200, 230)
MODEL_NAME = "pose_landmarker_full.task"           # body_pose 노드와 같은 모델
MODEL_CANDIDATES = [f"/opt/mediapipe/{MODEL_NAME}", str(Path(__file__).resolve().parents[1] / "models" / "mediapipe" / MODEL_NAME),
                    os.path.expanduser(f"~/rally/models/mediapipe/{MODEL_NAME}")]
# 상체 + 엉덩이만 (보상 동작에 쓰는 관절)
BONES = [(0, 7), (0, 8), (11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (11, 23), (12, 24), (23, 24)]
JOINTS = sorted({j for b in BONES for j in b})


def vis_color(v: float) -> tuple[int, int, int]:
    return GREEN if v >= 0.7 else YELLOW if v >= 0.5 else RED


class Skeleton:
    """확인 창 전용 PoseLandmarker — 새 프레임마다 한 번 돌려 2D 점·visibility·world 좌표를 보관."""

    def __init__(self) -> None:
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision
        path = next((c for c in MODEL_CANDIDATES if os.path.exists(c)), "")
        if not path:
            raise SystemExit(f"{MODEL_NAME} 없음. 찾은 곳: {MODEL_CANDIDATES}  (--no-skeleton 으로 끄고 실행 가능)")
        self.mp = mp
        self.lmk = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=path), running_mode=vision.RunningMode.VIDEO, num_poses=1))
        self.t0 = time.monotonic()
        self.last_ms = -1
        self.result = None

    def update(self, bgr: np.ndarray) -> None:
        ms = int((time.monotonic() - self.t0) * 1000)
        if ms <= self.last_ms:
            return
        self.last_ms = ms
        img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        res = self.lmk.detect_for_video(img, ms)
        self.result = res if res.pose_landmarks else None

    def draw(self, frame: np.ndarray) -> None:
        """frame 은 표시용으로 줄인 영상 (정규화 좌표라 크기 무관)."""
        if self.result is None:
            put(frame, "뼈대: 사람 없음", (8, 6), GRAY)
            return
        H, W = frame.shape[:2]
        lm = self.result.pose_landmarks[0]
        pt = {i: (int(lm[i].x * W), int(lm[i].y * H)) for i in JOINTS}
        vis = np.array([q.visibility for q in lm])
        for a, b in BONES:
            cv2.line(frame, pt[a], pt[b], vis_color(min(vis[a], vis[b])), 2)
        for i in JOINTS:
            cv2.circle(frame, pt[i], 5, vis_color(vis[i]), -1)
        w = np.array([[q.x, q.y, q.z] for q in self.result.pose_world_landmarks[0]])
        for side, (s, e, wr) in (("L", (bg.L_SH, bg.L_EL, bg.L_WR)), ("R", (bg.R_SH, bg.R_EL, bg.R_WR))):
            ang = bg._angle_deg(w[s] - w[e], w[wr] - w[e])
            txt = f"{side} {ang:.0f}" if math.isfinite(ang) else f"{side} -"
            cv2.putText(frame, f"{txt} v{min(vis[s], vis[e], vis[wr]):.1f}", (pt[e][0] + 8, pt[e][1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, vis_color(min(vis[s], vis[e], vis[wr])), 2)
        lean, how = bg.trunk_lean_deg(w, vis)
        hint = {"torso": "torso (엉덩이→어깨)", "shoulders": "shoulders (어깨선 · 측면은 앞으로 숙이기 못 잼)",
                "none": "어깨 안 보임"}.get(how, how)
        put(frame, f"몸통: {hint}  {lean:.0f}°" if math.isfinite(lean) else f"몸통: {hint}", (8, 6), WHITE)


class Checker(Node):
    def __init__(self, image_topic: str) -> None:
        super().__init__("rally_check_body_live")
        self.bridge, self.lock = CvBridge(), threading.Lock()
        self.frame, self.last = None, None
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(BodyPose, "/rally/perception/body", self._on_body, qos_profile_sensor_data)
        self.reset_cli = self.create_client(Trigger, "/rally/perception/body/reset_baseline")
        self.baseline_t: float | None = None       # b 누른 시각 (기준 잡는 중 표시용)
        self.baseline_msg = ""
        self.reset()

    def request_baseline(self) -> None:
        """b 키 — body_pose 노드에 기준 다시 잡기 요청 (응답은 기다리지 않음)."""
        if not self.reset_cli.service_is_ready():
            self.baseline_msg = "기준 서비스 없음 — body_pose 노드가 이 브랜치 버전인지"
            self.baseline_t = time.monotonic()
            return
        self.reset_cli.call_async(Trigger.Request())
        self.baseline_t, self.baseline_msg = time.monotonic(), ""

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
            self.seq = getattr(self, "seq", 0) + 1

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


def render(node: Checker, skel: "Skeleton | None" = None, state: dict | None = None) -> np.ndarray:
    with node.lock:
        frame = None if node.frame is None else node.frame.copy()
        seq = getattr(node, "seq", 0)
        last, items = node.last, node.items()
    if skel is not None and frame is not None and state is not None and seq != state.get("seq"):
        state["seq"] = seq
        skel.update(frame)                           # 새 프레임일 때만 (같은 프레임 중복 추론 방지)
    if frame is None:
        frame = np.zeros((480, 640, 3), np.uint8)
    frame = cv2.resize(frame, (640, int(frame.shape[0] * 640 / frame.shape[1])))
    if skel is not None:
        skel.draw(frame)
    if frame.shape[0] < 480:
        frame = cv2.copyMakeBorder(frame, 0, 480 - frame.shape[0], 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    panel = np.full((frame.shape[0], 420, 3), 30, np.uint8)
    if node.baseline_t is not None:
        dt = time.monotonic() - node.baseline_t
        if node.baseline_msg and dt < 3.0:
            put(frame, node.baseline_msg, (8, frame.shape[0] - 30), RED)
        elif not node.baseline_msg and dt < 2.5:
            put(frame, f"기준 잡는 중 — 바르게 가만히 {max(0.0, 2.0 - dt):.1f} s", (8, frame.shape[0] - 30), YELLOW)
        elif not node.baseline_msg and dt < 4.0:
            put(frame, "기준 완료", (8, frame.shape[0] - 30), GREEN)
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
    ap.add_argument("--no-skeleton", action="store_true", help="뼈대를 그리지 않음 (MediaPipe 를 한 번 덜 돌림)")
    a = ap.parse_args()
    skel = None if a.no_skeleton else Skeleton()
    state: dict = {}
    rclpy.init()
    node = Checker(a.image)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    try:
        while True:
            cv2.imshow(WIN, render(node, skel, state))
            k = cv2.waitKey(30) & 0xFF
            if k == ord("q"):
                break
            if k == ord("r"):
                node.reset()
            if k == ord("b"):
                node.request_baseline()
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
