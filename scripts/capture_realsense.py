#!/usr/bin/env python3
"""scripts/capture_realsense.py — RealSense 로 학습용 사진 찍기
담당: 박성현

두 가지 방법 중 하나로 프레임을 받는다.
  --ros   : realsense2_camera 노드가 떠 있을 때, ROS 토픽을 구독 (rqt 로 보면서 찍을 때. cv_bridge 불필요)
  (기본)  : pyrealsense2 로 카메라를 직접 연다 (ROS 없이. rs_launch 와 동시에 못 씀)

    # ROS (터미널 1 에서 rs_launch.py 가 돌고 있을 때)
    python3 scripts/capture_realsense.py --ros --out data/raw/ball_table
    python3 scripts/capture_realsense.py --ros --out data/raw/ball_pad --depth
    python3 scripts/capture_realsense.py --ros --out data/raw/ball_rolling --auto 30 --interval 0.1   # 창 없이 30장 자동
    # pyrealsense2
    python3 scripts/capture_realsense.py --out data/raw/ball_table --depth

키: SPACE 한 장 · b 연사(--burst 장, --burst-interval 간격) · q 종료.
파일명: <out>/<prefix>_<번호>.png  (깊이는 <prefix>_<번호>_depth.png, 단위 mm, uint16)
해상도는 segmenter 가 쓸 스트림과 같게 (848x480). 학습 사진과 추론 입력이 다르면 mAP 가 떨어진다.
data/ 는 git 에 올리지 않는다 (CLAUDE.md).
"""
import argparse
import os
import time

import cv2
import numpy as np


# ---------------------------------------------------------------- 저장
class Saver:
    def __init__(self, out_dir: str, prefix: str):
        os.makedirs(out_dir, exist_ok=True)
        self.out_dir, self.prefix = out_dir, prefix
        nums = []
        for name in os.listdir(out_dir):
            if name.startswith(prefix + "_") and name.endswith(".png") and "_depth" not in name:
                stem = name[len(prefix) + 1:-4]
                if stem.isdigit():
                    nums.append(int(stem))
        self.idx = max(nums) + 1 if nums else 0

    def save(self, color: np.ndarray, depth_mm: np.ndarray | None) -> str:
        path = os.path.join(self.out_dir, f"{self.prefix}_{self.idx:04d}.png")
        cv2.imwrite(path, color)
        if depth_mm is not None:
            cv2.imwrite(os.path.join(self.out_dir, f"{self.prefix}_{self.idx:04d}_depth.png"), depth_mm)
        self.idx += 1
        return path


# ---------------------------------------------------------------- 프레임 소스 1: ROS 토픽
class RosSource:
    """sensor_msgs/Image 를 직접 numpy 로 변환 (cv_bridge 없이). color: rgb8/bgr8, depth: 16UC1(mm)"""

    def __init__(self, color_topic: str, depth_topic: str | None):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        rclpy.init()
        self.rclpy = rclpy
        self.node = Node("rally_capture")
        self.color = None
        self.depth = None
        self.node.create_subscription(Image, color_topic, self._on_color, qos_profile_sensor_data)
        if depth_topic:
            self.node.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.want_depth = depth_topic is not None
        self.node.get_logger().info(f"구독: {color_topic}" + (f" + {depth_topic}" if depth_topic else ""))

    def _on_color(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 1)[:, : msg.width * 3]
        img = buf.reshape(msg.height, msg.width, 3)
        self.color = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img.copy()

    def _on_depth(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, : msg.width]
        self.depth = buf.copy()  # realsense2_camera 는 16UC1 mm

    def grab(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.color is None or (self.want_depth and self.depth is None):
            return None, None
        return self.color, (self.depth if self.want_depth else None)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


# ---------------------------------------------------------------- 프레임 소스 2: pyrealsense2
class RsSource:
    def __init__(self, width: int, height: int, fps: int, want_depth: bool):
        try:
            import pyrealsense2 as rs
        except ImportError:
            raise SystemExit("pyrealsense2 가 없습니다: pip3 install --break-system-packages pyrealsense2  (또는 --ros 사용)")
        self.rs = rs
        self.pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        if want_depth:
            cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color) if want_depth else None
        self.scale = profile.get_device().first_depth_sensor().get_depth_scale() if want_depth else None
        for _ in range(fps):  # 자동 노출 안정 (1 초 버림)
            self.pipe.wait_for_frames()

    def grab(self):
        frames = self.pipe.wait_for_frames()
        if self.align is not None:
            frames = self.align.process(frames)
        color = np.asanyarray(frames.get_color_frame().get_data())
        depth_mm = None
        if self.scale is not None:
            d = np.asanyarray(frames.get_depth_frame().get_data()).astype(np.float32)
            depth_mm = np.clip(d * self.scale * 1000.0, 0, 65535).astype(np.uint16)
        return color, depth_mm

    def close(self):
        self.pipe.stop()


# ---------------------------------------------------------------- main
def main() -> None:
    ap = argparse.ArgumentParser(description="RealSense 학습 사진 촬영")
    ap.add_argument("--out", required=True, help="저장 폴더 (예: data/raw/ball_table)")
    ap.add_argument("--prefix", default="img")
    ap.add_argument("--depth", action="store_true", help="정렬 깊이도 저장 (mm, uint16)")
    ap.add_argument("--ros", action="store_true", help="ROS 토픽 구독 (realsense2_camera 노드 필요)")
    ap.add_argument("--color-topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    ap.add_argument("--width", type=int, default=848)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--burst", type=int, default=10, help="b 키 연사 장수")
    ap.add_argument("--burst-interval", type=float, default=0.3, help="연사 간격 [s]")
    ap.add_argument("--auto", type=int, default=0, help="창 없이 N 장 자동 저장 후 종료")
    ap.add_argument("--interval", type=float, default=0.5, help="--auto 간격 [s]")
    args = ap.parse_args()

    saver = Saver(args.out, args.prefix)
    src = RosSource(args.color_topic, args.depth_topic if args.depth else None) if args.ros \
        else RsSource(args.width, args.height, args.fps, args.depth)
    print(f"[capture] 저장 {args.out}, 시작 번호 {saver.idx}, 소스 {'ROS' if args.ros else 'pyrealsense2'}")

    try:
        if args.auto:
            t0 = time.time()
            while saver.idx < args.auto:
                color, depth = src.grab()
                if color is None:
                    if time.time() - t0 > 10:
                        raise SystemExit("10 s 동안 프레임 없음 — rs_launch.py 가 떠 있는지, 토픽 이름이 맞는지 확인 (ros2 topic list)")
                    continue
                print("  저장", saver.save(color, depth))
                time.sleep(args.interval)
            return

        print("SPACE=한 장, b=연사, q=종료")
        while True:
            color, depth = src.grab()
            if color is None:
                continue
            view = color.copy()
            cv2.putText(view, f"{saver.idx} saved   SPACE / b / q", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("capture", view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" "):
                print("  저장", saver.save(color, depth))
            elif key == ord("b"):
                n = 0
                while n < args.burst:
                    c, d = src.grab()
                    if c is None:
                        continue
                    print("  저장", saver.save(c, d))
                    n += 1
                    time.sleep(args.burst_interval)
    finally:
        src.close()
        cv2.destroyAllWindows()
        print(f"[capture] 종료. 총 {saver.idx}장")


if __name__ == "__main__":
    main()
