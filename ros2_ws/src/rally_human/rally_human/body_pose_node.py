"""rally_human/body_pose_node.py — 측면 웹캠 MediaPipe Pose → 보상 동작 → /rally/perception/body (BodyPose)
담당: 정동휘 (기획 FR-05)

흐름 (영상 콜백은 최신 프레임만 저장, 추론은 타이머 — 손 노드와 같은 구조):
  1. 웹캠 영상(/webcam/image_raw, webcam_pub 또는 usb_cam)을 MediaPipe PoseLandmarker(VIDEO 모드, 1명)로
  2. pose_world_landmarks(엉덩이 원점 3D, m) 로 몸통 기울기·어깨 들림·팔꿈치 펴짐 (body_geometry.py)
     - 몸통 기울기: 엉덩이가 보이면 엉덩이→어깨 선과 수직의 각, 책상에 가려지면 어깨선 기울기 (옆 기울임만)
     - 어깨 들림: 처음 baseline_frames 프레임의 귀–어깨 거리를 평소로 잡고 줄어든 만큼 (기준 잡기 전엔 NaN)
  3. 같은 보상 동작이 confirm_frames 프레임 연속이면 compensation_detected (몸통 15° 이상 > 어깨 들림 10° 이상)
  header.stamp = 영상 원본 시각. 사람이 안 보이면 detected=false, 값은 NaN.
측면 웹캠은 환자에게서 1 m 이상 (10/8 확인: 가까우면 몸이 안 잡힘).
"""
import math
import os
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image

from rally_interfaces.msg import BodyPose
from rally_human import body_geometry as bg

MODEL_NAME = "pose_landmarker_full.task"
# 모델 찾는 순서: 파라미터 → 컨테이너(/opt/mediapipe) → 저장소 models/mediapipe (git 제외)
MODEL_CANDIDATES = [f"/opt/mediapipe/{MODEL_NAME}",
                    str(Path(__file__).resolve().parents[4] / "models" / "mediapipe" / MODEL_NAME),
                    os.path.expanduser(f"~/rally/models/mediapipe/{MODEL_NAME}")]


class BodyPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("rally_body_pose")
        d = self.declare_parameter
        d("image_topic", "/webcam/image_raw")
        d("model_path", "")
        d("rate_hz", 15.0)                         # INTERFACES 2.2: 15 Hz
        d("min_detection_confidence", 0.5)
        d("min_visibility", 0.5)                   # 관절 visibility 가 이보다 낮으면 안 보이는 것으로
        d("lean_threshold_deg", 15.0)              # 기획 FR-05
        d("hike_threshold_deg", 10.0)              # 어깨 들림 — 초기값, 실측 후 조정
        d("confirm_frames", 5)                     # 15 Hz 기준 약 0.3 s
        d("baseline_frames", 30)                   # 처음 약 2 s 를 평소 자세로
        d("arm", "auto")                           # 팔꿈치 펴짐을 볼 팔: left | right | auto
        p = lambda k: self.get_parameter(k).value  # noqa: E731

        self.vis_thr = float(p("min_visibility"))
        self.arm = str(p("arm"))
        self.baseline = bg.ShoulderBaseline(int(p("baseline_frames")))
        self.comp = bg.CompensationFilter(float(p("lean_threshold_deg")), float(p("hike_threshold_deg")),
                                          int(p("confirm_frames")))
        self.landmarker = self._make_landmarker()
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest: Image | None = None
        self.t0_ns: int | None = None
        self.last_ts_ms = -1

        self.create_subscription(Image, str(p("image_topic")), self._on_image, qos_profile_sensor_data,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.pub = self.create_publisher(BodyPose, "/rally/perception/body", qos_profile_sensor_data)
        self.create_timer(1.0 / float(p("rate_hz")), self._tick, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f"몸 자세 시작: {p('image_topic')}, 기울임 {p('lean_threshold_deg')}° · "
                               f"어깨 들림 {p('hike_threshold_deg')}° · 처음 {p('baseline_frames')} 프레임은 평소 자세로")

    def _make_landmarker(self):
        from mediapipe.tasks.python import BaseOptions, vision
        path = self.get_parameter("model_path").value or next((c for c in MODEL_CANDIDATES if os.path.exists(c)), "")
        if not path or not os.path.exists(path):
            raise FileNotFoundError(f"{MODEL_NAME} 없음. 찾은 곳: {MODEL_CANDIDATES}")
        conf = float(self.get_parameter("min_detection_confidence").value)
        return vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=path), running_mode=vision.RunningMode.VIDEO,
            num_poses=1, min_pose_detection_confidence=conf, min_tracking_confidence=conf))

    def _on_image(self, m: Image) -> None:
        with self.lock:
            self.latest = m

    def _tick(self) -> None:
        with self.lock:
            msg, self.latest = self.latest, None
        if msg is None:
            return
        out = BodyPose()
        out.header = msg.header
        try:
            self._process(msg, out)
        except Exception as e:      # noqa: BLE001 — 한 프레임 실패로 노드가 죽지 않게
            self.get_logger().error(f"몸 자세 처리 실패: {type(e).__name__}: {e}", throttle_duration_sec=2.0)
            out.detected = False
        if not out.detected:
            out.trunk_lean_deg = out.shoulder_elevation_deg = out.elbow_extension_deg = math.nan
        kind = self.comp.update(out.detected, out.trunk_lean_deg, out.shoulder_elevation_deg)
        out.compensation_type = kind
        out.compensation_detected = kind != "none"
        self.pub.publish(out)

    def _process(self, msg: Image, out: BodyPose) -> None:
        import mediapipe as mp
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if self.t0_ns is None:
            self.t0_ns = stamp_ns
        ts_ms = max(self.last_ts_ms + 1, (stamp_ns - self.t0_ns) // 1_000_000)   # VIDEO 모드는 시각이 늘어나야 함
        self.last_ts_ms = ts_ms
        res = self.landmarker.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)), ts_ms)
        if not res.pose_world_landmarks:
            out.detected = False
            return
        w = np.array([[q.x, q.y, q.z] for q in res.pose_world_landmarks[0]])
        vis = np.array([q.visibility for q in res.pose_landmarks[0]])
        if not bg._seen(vis, bg.L_SH, bg.R_SH, thr=self.vis_thr):      # 어깨가 안 보이면 아무것도 못 잼
            out.detected = False
            return
        out.detected = True
        out.trunk_lean_deg, _ = bg.trunk_lean_deg(w, vis, self.vis_thr)
        gaps = bg.ear_shoulder_gaps(w, vis, self.vis_thr)
        base = self.baseline.update(gaps)
        out.shoulder_elevation_deg = (bg.shoulder_elevation_deg(gaps, base, bg.shoulder_half_width(w))
                                      if base is not None else math.nan)
        out.elbow_extension_deg = bg.elbow_extension_deg(w, vis, self.arm, self.vis_thr)


def main() -> None:
    rclpy.init()
    node = BodyPoseNode()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
