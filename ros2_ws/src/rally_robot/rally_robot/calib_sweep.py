"""rally_robot/calib_sweep.py — 핸드-아이 캘리브레이션용 로봇 자세 순회 + 촬영 (Day 1 todo, 박성현 RunCalibration 에서 호출)
담당: 정정인

중심 자세 하나(카메라가 보드를 정면으로 보는 자세)를 티칭하면, 그 주변으로 15개 자세를 자동 생성해
movel 로 순회하며 /camera/color/image_raw 한 장씩 저장한다. 출력은 선생님 calibration/data_recording.py 와
같은 형식(calibrate_data.json: poses=[posx…], file_name=[…])이라 handeye_calibration.py 에 그대로 넣을 수 있다.

    ros2 run rally_robot calib_sweep --center 450 0 350 0 180 0 --dry-run      # 자세만 출력, 안 움직임
    ros2 run rally_robot calib_sweep --center 450 0 350 0 180 0 --out data/calib/run1
    ros2 run rally_robot calib_sweep --poses-file rally_bringup/config/calib_poses.yaml   # 직접 티칭한 목록

- 속도는 VELOCITY * 0.3 고정 (보드 근처 저속). 각 자세에서 0.7 s 정지 후 촬영(흔들림 안정).
- 오프셋 범위(mm, deg)는 기본 ±60 / ±40 / ±12 — 보드가 화면 밖으로 나가면 --span 으로 줄인다.
- 카메라는 PC2 에 있으므로 이미지는 토픽으로 받는다. 2 s 안에 프레임이 없으면 그 자세는 건너뛰고 기록한다.
- 안전: safety_monitor 가 STOP 을 내면 즉시 중단. 순회 전 자세 목록을 출력하고 --dry-run 으로 먼저 확인할 것.
"""
import argparse
import json
import os
import sys
import time
from itertools import product

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from rally_interfaces.msg import SafetyStatus
from rally_robot.dsr_helpers import init_dsr, VELOCITY, ACC

SETTLE_S = 0.7
FRAME_TIMEOUT_S = 2.0
SPEED_SCALE = 0.3


def generate_poses(center, span_xy=60.0, span_z=40.0, span_deg=12.0, count=15):
    """중심 posx 주변 count 개 자세. 위치 8방향(높이 번갈아) + 회전 4종을 섞어 위치·회전 다양성을 확보한다.
    ry≈180(카메라 아래 보기)에서 rx 변화는 ZYZ 특성상 툴 축 회전(요)이고, ry 변화가 실제 기울임이다. 둘 다 필요하다."""
    cx, cy, cz, rx, ry, rz = center
    dirs = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1)]
    rots = [(0, span_deg), (0, -span_deg), (span_deg, 0), (-span_deg, 0)]
    poses = [list(center)]
    for i, (dx, dy) in enumerate(dirs):                          # 바깥 8개
        dz = span_z if i % 2 == 0 else -span_z
        ta, tb = rots[i % 4]
        poses.append([cx + dx * span_xy, cy + dy * span_xy, cz + dz, rx + ta, ry + tb, rz])
    for i, (dx, dy) in enumerate(dirs):                          # 안쪽(반 거리) 나머지
        dz = -span_z if i % 2 == 0 else span_z
        ta, tb = rots[(i + 2) % 4]
        poses.append([cx + dx * span_xy / 2, cy + dy * span_xy / 2, cz + dz, rx + ta, ry + tb, rz])
    return poses[:count]

class CalibSweep(Node):
    def __init__(self):
        super().__init__("rally_calib_sweep")
        self.frame = None
        self.stop = False
        self.create_subscription(Image, "/camera/color/image_raw", self._on_image, qos_profile_sensor_data)
        self.create_subscription(SafetyStatus, "/rally/safety/status", self._on_safety, 10)

    def _on_image(self, msg):
        self.frame = msg

    def _on_safety(self, msg):
        if msg.level == SafetyStatus.LEVEL_STOP:
            self.stop = True

    def grab(self):
        """최신 프레임 한 장을 BGR numpy 로. 없으면 None."""
        self.frame = None
        t0 = time.monotonic()
        while self.frame is None and time.monotonic() - t0 < FRAME_TIMEOUT_S:
            rclpy.spin_once(self, timeout_sec=0.05)
        m = self.frame
        if m is None:
            return None
        arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step // 1)[:, : m.width * 3].reshape(m.height, m.width, 3)
        return arr[:, :, ::-1].copy() if m.encoding == "rgb8" else arr.copy()


def main(args=None):
    ap = argparse.ArgumentParser(description="핸드-아이 캘리브레이션 자세 순회·촬영")
    ap.add_argument("--center", type=float, nargs=6, metavar=("X", "Y", "Z", "RX", "RY", "RZ"), help="중심 posx (mm, deg)")
    ap.add_argument("--poses-file", help="posx 목록 yaml (poses: [[x,y,z,rx,ry,rz], …]) — --center 대신")
    ap.add_argument("--count", type=int, default=15)
    ap.add_argument("--span", type=float, nargs=3, default=(60.0, 40.0, 12.0), metavar=("XY_MM", "Z_MM", "DEG"))
    ap.add_argument("--out", default="data/calib/latest")
    ap.add_argument("--dry-run", action="store_true", help="자세만 출력하고 움직이지 않음")
    opt = ap.parse_args(args=[a for a in (args or sys.argv[1:]) if not a.startswith("--ros-args")])

    if opt.poses_file:
        import yaml
        with open(opt.poses_file, encoding="utf-8") as f:
            poses = [list(map(float, p)) for p in yaml.safe_load(f)["poses"]]
    elif opt.center:
        poses = generate_poses(opt.center, *opt.span, count=opt.count)
    else:
        ap.error("--center 또는 --poses-file 중 하나가 필요")

    print(f"자세 {len(poses)}개:")
    for i, p in enumerate(poses):
        print(f"  {i:2d}: " + " ".join(f"{v:8.1f}" for v in p))
    if opt.dry_run:
        return 0

    rclpy.init()
    dsr = init_dsr("rally_calib_sweep_dsr")
    node = CalibSweep()
    from DSR_ROBOT2 import movel, mwait, get_current_posx, posx
    import cv2

    os.makedirs(opt.out, exist_ok=True)
    record = {"poses": [], "file_name": [], "skipped": []}
    vel, acc = VELOCITY * SPEED_SCALE, ACC * SPEED_SCALE
    try:
        for i, p in enumerate(poses):
            rclpy.spin_once(node, timeout_sec=0.0)
            if node.stop:
                node.get_logger().error("safety STOP — 순회 중단")
                break
            node.get_logger().info(f"[{i+1}/{len(poses)}] movel {p}")
            movel(posx(*p), vel=vel, acc=acc)
            mwait()
            time.sleep(SETTLE_S)
            actual = [float(v) for v in get_current_posx()[0]]
            img = node.grab()
            if img is None:
                node.get_logger().warning("카메라 프레임 없음 — 건너뜀 (/camera/color/image_raw 발행 확인)")
                record["skipped"].append(actual)
                continue
            name = f"{actual[0]:.1f}_{actual[1]:.1f}_{actual[2]:.1f}.jpg"
            cv2.imwrite(os.path.join(opt.out, name), img)
            record["poses"].append(actual)
            record["file_name"].append(name)
            with open(os.path.join(opt.out, "calibrate_data.json"), "w", encoding="utf-8") as f:
                json.dump(record, f, indent=2, ensure_ascii=False)
        node.get_logger().info(f"저장 {len(record['poses'])}장 → {opt.out}/calibrate_data.json (건너뜀 {len(record['skipped'])})")
    finally:
        node.destroy_node()
        dsr.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
