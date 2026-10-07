#!/usr/bin/env python3
"""MediaPipe 손·자세 단독 실행 확인 (Day 0). 화면에 관절을 그리고 처리 시간·검출 수를 보여준다.

카메라 두 가지:
    --source webcam      측면 웹캠(C270)을 직접 연다. ROS 필요 없음
    --source realsense   RealSense ROS 토픽을 구독 (realsense2_camera 켜둔 상태, 또는 bag 재생 중)
                         정렬 깊이가 있으면 손목의 3D 위치(카메라 기준, m)도 표시

사용:
    python3 scripts/check_mediapipe.py --source webcam
    python3 scripts/check_mediapipe.py --source realsense --pose
조작: q 종료, p 자세(전신) 켜기/끄기
모델: models/mediapipe/hand_landmarker.task, pose_landmarker_full.task (git 제외, README 참고)
"""
import argparse
import glob
import sys
import time
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python import vision

MODEL_DIR = Path(__file__).resolve().parents[1] / "models" / "mediapipe"

# 손 21점 연결 (MediaPipe 손 관절 번호: 0 손목, 1~4 엄지, 5~8 검지, 9~12 중지, 13~16 약지, 17~20 새끼)
HAND_EDGES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11),
              (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]
# 자세 33점 중 팔·몸통·다리 골격만
POSE_EDGES = [(11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (11, 23), (12, 24), (23, 24), (23, 25),
              (25, 27), (24, 26), (26, 28)]


def make_detectors(with_pose: bool):
    hand_path = MODEL_DIR / "hand_landmarker.task"
    if not hand_path.exists():
        sys.exit(f"모델 없음: {hand_path}  (README 의 다운로드 명령 참고)")
    hands = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(hand_path)),
        running_mode=vision.RunningMode.VIDEO, num_hands=2,
        min_hand_detection_confidence=0.5, min_tracking_confidence=0.5))
    pose = None
    if with_pose:
        pose = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(MODEL_DIR / "pose_landmarker_full.task")),
            running_mode=vision.RunningMode.VIDEO))
    return hands, pose


def draw(img: np.ndarray, pts: list, edges: list, color: tuple) -> None:
    h, w = img.shape[:2]
    xy = [(int(p.x * w), int(p.y * h)) for p in pts]
    for a, b in edges:
        cv2.line(img, xy[a], xy[b], color, 2, cv2.LINE_AA)
    for p in xy:
        cv2.circle(img, p, 3, (255, 255, 255), -1)


def wrist_3d(depth: np.ndarray | None, K: np.ndarray | None, u: float, v: float) -> np.ndarray | None:
    """정렬 깊이에서 손목 주변 5x5 유효값 중앙값으로 3D (카메라 기준, m). 깊이 빈 곳이 많아 주변을 본다."""
    if depth is None or K is None:
        return None
    h, w = depth.shape
    x, y = int(round(u)), int(round(v))
    win = depth[max(0, y - 2):min(h, y + 3), max(0, x - 2):min(w, x + 3)].astype(np.float32) * 1e-3
    win = win[win > 0.1]
    if not len(win):
        return None
    z = float(np.median(win))
    return np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z])


def run(frames, hands, pose, label: str) -> None:
    """frames: (bgr, depth 또는 None, K 또는 None) 를 내는 반복자."""
    show_pose = pose is not None
    t0 = time.monotonic()
    fps, last = 0.0, time.monotonic()
    for bgr, depth, K in frames:
        ts_ms = int((time.monotonic() - t0) * 1000)
        rgb = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        t_inf = time.perf_counter()
        res = hands.detect_for_video(rgb, ts_ms)
        pres = pose.detect_for_video(rgb, ts_ms) if (show_pose and pose) else None
        inf_ms = (time.perf_counter() - t_inf) * 1000

        vis = bgr.copy()
        lines = []
        for i, lm in enumerate(res.hand_landmarks):
            draw(vis, lm, HAND_EDGES, (0, 200, 0))
            side = res.handedness[i][0].category_name if res.handedness else "?"
            u, v = lm[0].x * bgr.shape[1], lm[0].y * bgr.shape[0]
            p = wrist_3d(depth, K, u, v)
            lines.append(f"{side} hand  wrist px=({u:.0f},{v:.0f})"
                         + (f"  xyz=({p[0]:+.2f},{p[1]:+.2f},{p[2]:.2f}) m" if p is not None else ""))
        if pres is not None:
            for lm in pres.pose_landmarks:
                draw(vis, lm, POSE_EDGES, (255, 160, 0))

        now = time.monotonic()
        fps = 0.9 * fps + 0.1 / max(now - last, 1e-3)
        last = now
        head = [f"{label}  {fps:4.1f} fps  inference {inf_ms:5.1f} ms  hands {len(res.hand_landmarks)}"
                + ("  +pose" if show_pose else "")]
        for k, t in enumerate(head + lines):
            cv2.putText(vis, t, (8, 22 + 22 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(vis, t, (8, 22 + 22 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.imshow("check_mediapipe (q quit, p pose)", vis)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("p") and pose is not None:
            show_pose = not show_pose
    cv2.destroyAllWindows()


def webcam_frames(name: str):
    dev = None
    for path in sorted(glob.glob("/sys/class/video4linux/video*")):
        if name.lower() in Path(path, "name").read_text().lower() and Path(path, "index").read_text().strip() == "0":
            dev = f"/dev/{Path(path).name}"
            break
    if dev is None:
        sys.exit(f"'{name}' 카메라 없음 (v4l2-ctl --list-devices)")
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        yield frame, None, None
    cap.release()


def realsense_frames(color_topic: str, depth_topic: str, info_topic: str):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = rclpy.create_node("rally_check_mediapipe")
    bridge = CvBridge()
    st = {"color": None, "depth": None, "K": None}

    def on_color(m):
        st["color"] = bridge.imgmsg_to_cv2(m, "bgr8")

    def on_depth(m):
        st["depth"] = bridge.imgmsg_to_cv2(m)

    def on_info(m):
        st["K"] = np.array(m.k).reshape(3, 3)

    node.create_subscription(Image, color_topic, on_color, qos_profile_sensor_data)
    node.create_subscription(Image, depth_topic, on_depth, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, info_topic, on_info, qos_profile_sensor_data)
    node.get_logger().info(f"구독: {color_topic} (+ {depth_topic})")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if st["color"] is None:
                continue
            frame, st["color"] = st["color"], None      # 최신 프레임만 처리 (밀리면 건너뜀)
            yield frame, st["depth"], st["K"]
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=["webcam", "realsense"], default="webcam")
    ap.add_argument("--name", default="C270", help="webcam 장치 이름 일부")
    ap.add_argument("--pose", action="store_true", help="전신 자세도 같이")
    ap.add_argument("--color", default="/camera/camera/color/image_raw")
    ap.add_argument("--depth", default="/camera/camera/aligned_depth_to_color/image_raw")
    ap.add_argument("--info", default="/camera/camera/color/camera_info")
    a = ap.parse_args()
    hands, pose = make_detectors(a.pose)
    if a.source == "webcam":
        run(webcam_frames(a.name), hands, pose, "webcam")
    else:
        run(realsense_frames(a.color, a.depth, a.info), hands, pose, "realsense")


if __name__ == "__main__":
    main()
