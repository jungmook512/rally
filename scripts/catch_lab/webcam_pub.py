#!/usr/bin/env python3
"""웹캠 → /webcam/image_raw 발행 (usb_cam 이 없을 때 대용). 장치는 이름 일부로 찾는다.

사용:
    python3 webcam_pub.py [--name C270] [--exposure 50] [--fourcc MJPG] [--width 640] [--height 480] [--fps 30]
    --exposure: 노출 시간 (단위 0.1 ms, 50 = 5 ms). 짧을수록 잔상이 줄고 화면이 어두워진다. 생략하면 자동 노출.
    C270 은 수동 노출이 되고, 노트북 내장 웹캠("Web Camera")은 안 된다.
"""
import argparse
import glob
import subprocess

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def find_device(keyword: str) -> str:
    for path in sorted(glob.glob("/sys/class/video4linux/video*")):
        with open(f"{path}/name") as f:
            name = f.read().strip()
        with open(f"{path}/index") as f:
            index = int(f.read().strip())
        if keyword.lower() in name.lower() and index == 0:
            return f"/dev/{path.rsplit('/', 1)[-1]}"
    raise SystemExit(f"'{keyword}' 카메라를 못 찾음 (v4l2-ctl --list-devices 로 이름 확인)")


class WebcamPub(Node):
    def __init__(self, a: argparse.Namespace) -> None:
        super().__init__("rally_webcam_pub")
        dev = find_device(a.name)
        # 어두우면 프레임레이트를 낮추는 기능을 끈다 (30 fps 고정)
        subprocess.run(["v4l2-ctl", "-d", dev, "-c", "exposure_dynamic_framerate=0"], check=False,
                       capture_output=True)
        if a.exposure is not None:
            # 1 = 수동 노출. 장치가 지원하지 않으면 v4l2-ctl 이 실패하고 자동 노출 그대로
            r = subprocess.run(["v4l2-ctl", "-d", dev, "-c", "auto_exposure=1",
                                "-c", f"exposure_time_absolute={a.exposure}"], capture_output=True, text=True)
            if r.returncode != 0:
                self.get_logger().warn(f"수동 노출 설정 실패 (자동 노출 사용): {r.stderr.strip()}")
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        # 압축(MJPG)으로 받으면 USB 대역폭이 적게 들어 RealSense 와 같이 꽂아도 30 fps 가 나온다
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*a.fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
        self.cap.set(cv2.CAP_PROP_FPS, a.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise SystemExit(f"{dev} 열기 실패")
        self.pub = self.create_publisher(Image, "/webcam/image_raw", qos_profile_sensor_data)
        self.n = 0
        fcc = int(self.cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little").decode(errors="replace")
        self.get_logger().info(f"{dev} {int(self.cap.get(3))}x{int(self.cap.get(4))} {fcc} @ {self.cap.get(5):.0f} fps")

    def spin_capture(self) -> None:
        t_log = self.get_clock().now()
        while rclpy.ok():
            if not self.cap.grab():
                self.get_logger().error("프레임 읽기 실패")
                break
            stamp = self.get_clock().now()           # grab 직후 시각. 노출·전송 지연은 analyze 의 dt 로 흡수
            ok, frame = self.cap.retrieve()
            if not ok:
                continue
            msg = Image()
            msg.header.stamp = stamp.to_msg()
            msg.header.frame_id = "webcam_link"
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = "bgr8"
            msg.step = msg.width * 3
            msg.data = frame.tobytes()
            self.pub.publish(msg)
            self.n += 1
            if (stamp - t_log).nanoseconds > 5e9:
                self.get_logger().info(f"{self.n / ((stamp - t_log).nanoseconds * 1e-9):.1f} fps")
                self.n, t_log = 0, stamp


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="C270")
    ap.add_argument("--exposure", type=int, default=None, help="노출 [0.1 ms], 예: 50 = 5 ms")
    ap.add_argument("--fourcc", default="MJPG", help="MJPG(압축, 권장) 또는 YUYV")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = WebcamPub(a)
    try:
        node.spin_capture()
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
