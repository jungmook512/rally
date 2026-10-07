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
  4. 이동은 dsr_helpers.move_checked (스킬과 같은 코드): amovel 반환값 확인 → 시작 대기 → check_motion 폴링 → 도착 검증(3 mm·2°).
     이동 중 STOP → 정지 서비스(소프트 → 퀵 재시도) 후 check_motion 으로 **실제로 멈췄는지 확인**해야 '정지'로 기록.
     확인 실패면 에러 로그 + 종료 코드 4 (하드웨어 비상정지 안내). SLOW 로 바뀌면 정지 확인 후 감속 배율로 재이동.
  5. 속도 = VELOCITY × 0.5 × speed_override (30 mm/s). 이동 타임아웃은 거리/속도 ×2 + 5 s 로 계산 (저속·장거리 오탐 방지).
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
SPEED_SCALE = 0.5            # VELOCITY 60 mm/s × 0.5 = 30 mm/s (movel 의 vel 은 mm/s)
WORKSPACE_DEFAULT_M = dict(x_min=0.25, x_max=0.80, y_min=-0.45, y_max=0.45, z_min=0.0, z_max=0.60)  # safety.yaml 과 동일


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


# 컨트롤러(펜던트 안전 설정)가 거부하는 관절 범위. 2026-10-07 실측: J5 131.3° 허용, 138.8° 는 1206 "NOT REACHABLE".
# 정확한 값은 펜던트 안전 설정 → 관절 범위에서 확인해 고친다. 여유 2° 를 둔다.
J5_ABS_MAX_DEG = 133.0


def check_joint_limits(joint_sets, j5_max=J5_ABS_MAX_DEG):
    """[(i, posj|None)] → 문제 있는 [(i, 이유)]. posj None = IK 실패."""
    bad = []
    for i, q in joint_sets:
        if q is None:
            bad.append((i, "IK 해 없음 (현재 팔 형태로 도달 불가)"))
        elif abs(q[4]) > j5_max:
            bad.append((i, f"J5 {q[4]:.1f}° > ±{j5_max:.0f}° (컨트롤러 관절 범위 밖)"))
    return bad


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
def run_sweep(poses, out_dir, no_camera=False, j5_max=J5_ABS_MAX_DEG):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from rally_interfaces.msg import SafetyStatus
    from rally_robot.dsr_helpers import ACC, VELOCITY, SkillAbort, controller_alive, init_dsr, move_checked

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
            self.safety = msg                 # 백그라운드 스레드에서 갱신 (참조 대입이라 잠금 불필요)

        def stop_requested(self):
            return self.safety is None or self.safety.level == SafetyStatus.LEVEL_STOP

        def speed_scale(self):
            return self.safety.speed_override if self.safety.level == SafetyStatus.LEVEL_SLOW else 1.0

        def wait_for(self, attr, timeout_s):
            # 콜백은 백그라운드 실행기가 처리한다 (이동 중에도 안전 상태가 갱신돼야 estop 이 먹는다)
            t0 = time.monotonic()
            while getattr(self, attr) is None and time.monotonic() - t0 < timeout_s:
                time.sleep(0.05)
            return getattr(self, attr) is not None

        def grab(self):
            self.frame = None
            if not self.wait_for("frame", FRAME_TIMEOUT_S):
                return None
            m = self.frame
            arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
            return arr[:, :, ::-1].copy() if m.encoding == "rgb8" else arr.copy()

    import threading
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()
    node = Sweep()
    # 2026-10-07 버그 수정: 예전엔 wait_for 안에서만 spin_once 해서, move_checked 가 도는 동안 /rally/safety/status 를
    # 못 받아 이동 중 estop 이 무시됐다. 안전 구독 노드는 별도 스레드에서 계속 spin 한다.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
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

        if os.environ.get("RALLY_FAKE") == "1":            # 로봇 없이 흐름·estop 검증 (skill_server --fake 와 같은 방식)
            from rally_robot import fake_dsr
            sys.modules["DSR_ROBOT2"] = fake_dsr
            node.get_logger().warning("RALLY_FAKE=1 — 가짜 DSR 로 순회 (로봇 안 움직임)")
        dsr = init_dsr("rally_calib_sweep_dsr")
        from DSR_ROBOT2 import get_current_posx
        if not controller_alive(dsr, 2.0):
            node.get_logger().error("두산 컨트롤러 motion 서비스(check_motion)가 2 s 안에 응답하지 않음 — bringup 재시작 필요. 중단")
            return 5
        import cv2

        # 1206 "NOT REACHABLE" 사전 차단: 현재 해 공간으로 IK → 관절 범위 밖 자세는 순회에서 뺀다 (움직이기 전에)
        try:
            from DSR_ROBOT2 import get_current_solution_space, ikin
        except ImportError:                           # fake_dsr 에는 IK 가 없다 → 검사 생략
            ikin = None
        bad = []
        sol = -1
        if ikin is not None:
            sol = int(get_current_solution_space())
            joint_sets = []
            for i, p in enumerate(poses):
                try:
                    q = [float(v) for v in ikin(p, sol)]
                except Exception:                     # noqa: BLE001 — IK 실패는 '도달 불가'로 본다
                    q = None
                joint_sets.append((i, q))
            bad = check_joint_limits(joint_sets, j5_max)
        for i, why in bad:
            node.get_logger().warning(f"자세 {i} 제외 — {why}")
        poses = [p for i, p in enumerate(poses) if i not in {b[0] for b in bad}]
        if not poses:
            node.get_logger().error("도달 가능한 자세가 없음 — 중심 자세의 J5 를 줄이거나(덜 기울이기) --span 각도를 줄이세요. 중단")
            return 2
        node.get_logger().info(f"관절 범위 검사: {len(poses)}개 순회 (제외 {len(bad)}, 해 공간 {sol})")

        node.dsr = dsr                                   # move_checked / stop_robot 이 정지 서비스에 쓴다
        node.safety_reason = lambda: node.safety.reason if node.safety else "no_status"

        os.makedirs(out_dir, exist_ok=True)
        record = {"poses": [], "file_name": [], "skipped": []}
        for i, p in enumerate(poses):
            node.get_logger().info(f"[{i+1}/{len(poses)}] movel " + " ".join(f"{v:.1f}" for v in p))
            move_checked(node, p, VELOCITY * SPEED_SCALE, ACC * SPEED_SCALE)
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
    except SkillAbort as e:
        if e.stopped:
            node.get_logger().error(f"안전 정지({e.reason}) — 순회 중단. 로봇 정지 확인됨. reset_safety 후 다시 실행")
            return 3
        node.get_logger().fatal(f"안전 정지({e.reason}) — 정지 명령 실패, 로봇이 아직 움직일 수 있음. 하드웨어 비상정지로 멈추고 점검")
        return 4
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=2.0)
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
    ap.add_argument("--j5-max", type=float, default=J5_ABS_MAX_DEG, help="허용 |J5| [deg] — 펜던트 관절 범위에 맞춘다")
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
    return run_sweep(poses, opt.out, no_camera=opt.no_camera, j5_max=opt.j5_max)


if __name__ == "__main__":
    sys.exit(main())
