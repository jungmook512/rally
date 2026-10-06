"""rally_robot/calib_sweep.py — 핸드-아이 캘리브레이션용 로봇 자세 순회 + 촬영 (Day 1 todo, 박성현 RunCalibration 에서 호출)
담당: 정정인

중심 자세 하나(카메라가 보드를 정면으로 보는 자세)를 티칭하면, 그 주변으로 15개 자세를 자동 생성해
저속 movel 로 순회하며 /camera/color/image_raw 한 장씩 저장한다. 출력은 선생님 calibration/data_recording.py 와
같은 형식(calibrate_data.json: poses=[posx…], file_name=[…])이라 handeye_calibration.py 에 그대로 넣을 수 있다.

    ros2 run rally_robot calib_sweep --center 450 0 350 0 180 0 --dry-run      # 자세·작업 공간만 검사, 두산 패키지 불필요
    ros2 run rally_robot calib_sweep --center 450 0 350 0 180 0 --out data/calib/run1
    ros2 run rally_robot calib_sweep --poses-file rally_bringup/config/calib_poses.yaml   # 직접 티칭한 목록
    ros2 run rally_robot calib_sweep --center … --safety-yaml ros2_ws/src/rally_bringup/config/safety.yaml

자세 생성: 위치 8방향 × 높이 2단 + 회전은 툴 X 기울기 · Y 기울기 · Z(툴 축) 회전 각각 ± 를 섞는다 (6종, 회전 축 다양성).
안전 흐름 (실제 이동 시):
  1. 모든 자세가 safety.yaml 작업 공간 안인지 먼저 검사. 하나라도 밖이면 움직이지 않는다.
  2. /rally/safety/status 를 받을 때까지 대기(5 s). 못 받으면 움직이지 않는다 (safety_monitor 먼저 기동).
  3. /camera/color/image_raw 가 오는지 확인(2 s). 안 오면 중단 (--no-camera 로 이동만 점검 가능).
  4. 이동은 amovel + check_motion 폴링. 이동 중 STOP → 정지 서비스(소프트 → 퀵 순으로 재시도) 호출 후 check_motion 으로
     **실제로 멈췄는지 확인**해야 '정지'로 기록. 확인 실패면 에러 로그 + 종료 코드 4 (하드웨어 비상정지 안내).
     SLOW 로 바뀌면 같은 절차로 정지 확인 후 감속 배율로 재이동.
  5. 속도 = VELOCITY × 0.3 × speed_override.
"""
import argparse
import json
import os
import sys
import time

import warnings

import numpy as np
from scipy.spatial.transform import Rotation

warnings.filterwarnings("ignore", message="Gimbal lock detected")   # ry≈180 에서 ZYZ 분해 경고. 값은 유효하다.

SETTLE_S = 0.7
FRAME_TIMEOUT_S = 2.0
SAFETY_TIMEOUT_S = 5.0
SPEED_SCALE = 0.3
MOTION_TIMEOUT_S = 20.0
STOP_MODES = (2, 1)          # 정지 재시도 순서: 2 SSTOP(소프트) → 1 QSTOP(퀵)
STOP_VERIFY_S = 1.5          # 정지 명령 후 check_motion()==0 을 기다리는 시간 (시도당)
WORKSPACE_DEFAULT_M = dict(x_min=0.25, x_max=0.80, y_min=-0.45, y_max=0.45, z_min=0.0, z_max=0.60)  # safety.yaml 과 동일


class SafetyStop(RuntimeError):
    """안전 사유로 순회 중단. stopped=False 면 정지 서비스가 실패해 로봇이 아직 움직일 수 있다."""

    def __init__(self, reason: str, stopped: bool):
        super().__init__(reason)
        self.reason, self.stopped = reason, stopped


# ---------------------------------------------------------------- 자세 생성·검사 (ROS·두산 불필요)
def generate_poses(center, span_xy=60.0, span_z=40.0, span_deg=12.0, count=15):
    """중심 posx(mm, ZYZ deg) 주변 count 개. 회전은 툴 좌표계 X·Y 기울기와 Z 회전을 ± 로 섞는다."""
    cx, cy, cz = center[:3]
    r_center = Rotation.from_euler("ZYZ", center[3:6], degrees=True)
    axes = [np.array(a) for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    rots = [axis * np.deg2rad(sign * span_deg) for axis in axes for sign in (1, -1)]   # +X -X +Y -Y +Z -Z
    dirs = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1)]

    def pose(dx, dy, dz, rotvec):
        r = r_center * Rotation.from_rotvec(rotvec)                 # 툴 프레임 기준 섭동
        rx, ry, rz = r.as_euler("ZYZ", degrees=True)
        return [cx + dx, cy + dy, cz + dz, float(rx), float(ry), float(rz)]

    poses = [list(map(float, center))]
    for i, (dx, dy) in enumerate(dirs):                            # 바깥 8개
        poses.append(pose(dx * span_xy, dy * span_xy, span_z if i % 2 == 0 else -span_z, rots[i % 6]))
    for i, (dx, dy) in enumerate(dirs):                            # 안쪽(반 거리) 나머지
        poses.append(pose(dx * span_xy / 2, dy * span_xy / 2, -span_z if i % 2 == 0 else span_z, rots[(i + 3) % 6]))
    return poses[:count]


def load_workspace(safety_yaml):
    """safety.yaml 의 workspace_*_m → dict(m). 파일이 없으면 기본값."""
    ws = dict(WORKSPACE_DEFAULT_M)
    if safety_yaml:
        import yaml
        with open(safety_yaml, encoding="utf-8") as f:
            params = yaml.safe_load(f)["rally_safety_monitor"]["ros__parameters"]
        for k in ws:
            ws[k] = float(params.get(f"workspace_{k}_m", ws[k]))
    return ws


def check_workspace(poses, ws):
    """작업 공간(m) 밖인 자세의 (index, 사유) 목록. posx 는 mm."""
    bad = []
    for i, p in enumerate(poses):
        x, y, z = (v / 1000.0 for v in p[:3])
        for axis, v in (("x", x), ("y", y), ("z", z)):
            lo, hi = ws[f"{axis}_min"], ws[f"{axis}_max"]
            if not lo <= v <= hi:
                bad.append((i, f"{axis}={v:.3f} m (허용 {lo}~{hi})"))
    return bad


# ---------------------------------------------------------------- 실제 순회 (ROS·두산 필요)
def run_sweep(poses, out_dir, no_camera=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from rally_interfaces.msg import SafetyStatus
    from rally_robot.dsr_helpers import init_dsr, move_stop, VELOCITY, ACC

    class Sweep(Node):
        def __init__(self):
            super().__init__("rally_calib_sweep")
            self.frame = None
            self.safety = None
            reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
            self.create_subscription(Image, "/camera/color/image_raw", self._on_image, qos_profile_sensor_data)
            self.create_subscription(SafetyStatus, "/rally/safety/status", self._on_safety, reliable)

        def _on_image(self, msg):
            self.frame = msg

        def _on_safety(self, msg):
            self.safety = msg

        def stop_requested(self):
            return self.safety is None or self.safety.level == SafetyStatus.LEVEL_STOP

        def speed_scale(self):
            return self.safety.speed_override if self.safety.level == SafetyStatus.LEVEL_SLOW else 1.0

        def wait_for(self, attr, timeout_s):
            t0 = time.monotonic()
            while getattr(self, attr) is None and time.monotonic() - t0 < timeout_s:
                rclpy.spin_once(self, timeout_sec=0.05)
            return getattr(self, attr) is not None

        def grab(self):
            self.frame = None
            if not self.wait_for("frame", FRAME_TIMEOUT_S):
                return None
            m = self.frame
            arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
            return arr[:, :, ::-1].copy() if m.encoding == "rgb8" else arr.copy()

    rclpy.init()
    node = Sweep()
    dsr = None
    try:
        # 1. 안전 상태 수신 전에는 움직이지 않는다
        if not node.wait_for("safety", SAFETY_TIMEOUT_S):
            node.get_logger().error("/rally/safety/status 수신 없음 — safety_monitor 를 먼저 띄우세요. 중단")
            return 2
        if node.stop_requested():
            node.get_logger().error(f"안전 레벨 STOP ({node.safety.reason}) — /rally/robot/reset_safety 후 재시도. 중단")
            return 2
        if not no_camera and not node.wait_for("frame", FRAME_TIMEOUT_S):
            node.get_logger().error("/camera/color/image_raw 수신 없음 — PC2 realsense 확인. 중단 (--no-camera 로 이동만 점검 가능)")
            return 2

        dsr = init_dsr("rally_calib_sweep_dsr")
        from DSR_ROBOT2 import amovel, check_motion, get_current_posx, posx
        import cv2

        def stop_robot(why):
            """정지 서비스 호출 → check_motion()==0 으로 실제 정지 확인. 실패하면 정지 모드를 올려 재시도.
            멈춘 것이 확인되면 True. 끝까지 확인 못 하면 에러 로그 후 False (호출자가 '정지'로 기록하면 안 됨)."""
            for attempt, mode in enumerate(STOP_MODES, 1):
                ok = move_stop(dsr, stop_mode=mode)
                t0 = time.monotonic()
                while time.monotonic() - t0 < STOP_VERIFY_S:
                    rclpy.spin_once(node, timeout_sec=0.0)
                    if check_motion() == 0:
                        node.get_logger().warning(f"정지 확인 ({why}) mode={mode} 시도 {attempt} 서비스 응답={ok}")
                        return True
                    time.sleep(0.05)
                node.get_logger().error(f"정지 확인 실패 ({why}) mode={mode} 시도 {attempt} 서비스 응답={ok} — 아직 움직임, 재시도")
            node.get_logger().fatal(f"정지 실패 ({why}) — 로봇이 아직 움직일 수 있음. 티치펜던트/하드웨어 비상정지 사용")
            return False

        def reason():
            return node.safety.reason if node.safety else "no_status"

        def move_checked(p):
            """amovel 후 check_motion 폴링. STOP → 정지 확인 후 예외, SLOW 변화 → 정지 확인 후 배율 바꿔 재이동."""
            while True:
                rclpy.spin_once(node, timeout_sec=0.0)
                if node.stop_requested():
                    raise SafetyStop(reason(), stopped=True)          # 아직 명령 전이라 움직이지 않음
                scale = SPEED_SCALE * node.speed_scale()
                slowed = node.speed_scale() < 1.0
                amovel(posx(*p), vel=VELOCITY * scale, acc=ACC * scale)
                t0 = time.monotonic()
                while check_motion() != 0:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if node.stop_requested():
                        raise SafetyStop(reason(), stopped=stop_robot("safety STOP"))
                    if (node.speed_scale() < 1.0) != slowed:              # 감속 배율이 바뀜 → 정지 확인 후 재이동
                        if not stop_robot("감속 배율 변경"):
                            raise SafetyStop("stop_failed_on_slow", stopped=False)
                        node.get_logger().info(f"안전 레벨 변경(speed_override={node.speed_scale():.2f}) — 같은 자세로 재이동")
                        break                                              # 바깥 while 이 STOP 을 다시 확인하고 재명령
                    if time.monotonic() - t0 > MOTION_TIMEOUT_S:
                        stopped = stop_robot("이동 타임아웃")
                        raise SafetyStop(f"motion_timeout_{MOTION_TIMEOUT_S:.0f}s", stopped=stopped)
                else:
                    return

        os.makedirs(out_dir, exist_ok=True)
        record = {"poses": [], "file_name": [], "skipped": []}
        for i, p in enumerate(poses):
            node.get_logger().info(f"[{i+1}/{len(poses)}] movel " + " ".join(f"{v:.1f}" for v in p))
            move_checked(p)
            time.sleep(SETTLE_S)
            actual = [float(v) for v in get_current_posx()[0]]
            img = None if no_camera else node.grab()
            if img is None:
                node.get_logger().warning("카메라 프레임 없음 — 건너뜀")
                record["skipped"].append(actual)
                continue
            name = f"{actual[0]:.1f}_{actual[1]:.1f}_{actual[2]:.1f}.jpg"
            cv2.imwrite(os.path.join(out_dir, name), img)
            record["poses"].append(actual)
            record["file_name"].append(name)
            with open(os.path.join(out_dir, "calibrate_data.json"), "w", encoding="utf-8") as f:
                json.dump(record, f, indent=2, ensure_ascii=False)
        node.get_logger().info(f"저장 {len(record['poses'])}장 → {out_dir}/calibrate_data.json (건너뜀 {len(record['skipped'])})")
        return 0
    except SafetyStop as e:
        if e.stopped:
            node.get_logger().error(f"안전 정지({e.reason}) — 순회 중단. 로봇 정지 확인됨. reset_safety 후 다시 실행")
            return 3
        node.get_logger().fatal(f"안전 정지({e.reason}) — 정지 명령 실패, 로봇이 아직 움직일 수 있음. 하드웨어 비상정지로 멈추고 점검")
        return 4
    finally:
        node.destroy_node()
        if dsr is not None:
            dsr.destroy_node()
        rclpy.shutdown()


def main(args=None):
    ap = argparse.ArgumentParser(description="핸드-아이 캘리브레이션 자세 순회·촬영")
    ap.add_argument("--center", type=float, nargs=6, metavar=("X", "Y", "Z", "RX", "RY", "RZ"), help="중심 posx (mm, deg)")
    ap.add_argument("--poses-file", help="posx 목록 yaml (poses: [[x,y,z,rx,ry,rz], …]) — --center 대신")
    ap.add_argument("--count", type=int, default=15)
    ap.add_argument("--span", type=float, nargs=3, default=(60.0, 40.0, 12.0), metavar=("XY_MM", "Z_MM", "DEG"))
    ap.add_argument("--safety-yaml", help="작업 공간 한계를 읽을 safety.yaml (없으면 코드 기본값)")
    ap.add_argument("--out", default="data/calib/latest")
    ap.add_argument("--no-camera", action="store_true", help="촬영 없이 이동만 (카메라 미연결 점검용)")
    ap.add_argument("--dry-run", action="store_true", help="자세·작업 공간만 검사하고 움직이지 않음")
    argv = [a for a in (args if args is not None else sys.argv[1:]) if not a.startswith("--ros-args")]
    opt = ap.parse_args(argv)

    if opt.poses_file:
        import yaml
        with open(opt.poses_file, encoding="utf-8") as f:
            poses = [list(map(float, p)) for p in yaml.safe_load(f)["poses"]]
    elif opt.center:
        poses = generate_poses(opt.center, *opt.span, count=opt.count)
    else:
        ap.error("--center 또는 --poses-file 중 하나가 필요")

    print(f"자세 {len(poses)}개 (mm, ZYZ deg):")
    for i, p in enumerate(poses):
        print(f"  {i:2d}: " + " ".join(f"{v:8.1f}" for v in p))
    ws = load_workspace(opt.safety_yaml)
    bad = check_workspace(poses, ws)
    if bad:
        print(f"작업 공간 밖 자세 {len(bad)}건 — 움직이지 않음. --span 을 줄이거나 중심 자세를 옮기세요:")
        for i, why in bad:
            print(f"  {i:2d}: {why}")
        return 2
    print(f"작업 공간 검사 통과 ({ws})")
    if opt.dry_run:
        return 0
    return run_sweep(poses, opt.out, no_camera=opt.no_camera)


if __name__ == "__main__":
    sys.exit(main())
