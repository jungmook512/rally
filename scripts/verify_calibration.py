#!/usr/bin/env python3
"""scripts/verify_calibration.py — 기준점(ArUco) 터치 검증: 카메라가 계산한 마커 위치 vs 로봇 TCP 로 찍은 위치 (mm)
담당: 박성현 (로봇 자세·터치는 정정인)

1. 관측 자세에서 aruco_detect.py --json 으로 마커 검출 결과 저장 (그 순간의 TCP posx 도 같이 적어 둔다)
2. 로봇 TCP 끝을 마커 중심에 직접 교시로 대고 get_current_posx (mm) 를 읽는다 (마커 2개 이상이면 좋다)
3. 이 스크립트: 카메라 좌표 → base_link (posx × T_gripper2camera.npy) → 터치값과 비교

    python3 scripts/verify_calibration.py data/aruco_1007_observe.json \
        --posx 352.6 -29.3 481.6 146.9 -169.3 147.3 \
        --touch p1=412.3,105.0,12.0 --touch p4=410.8,-345.2,12.1
    # posx 를 토픽에서 읽으려면: --posx-topic /rally/robot/tcp_pose (ROS 환경)

판정: 오차 ≤ 5 mm 면 캘리브레이션 통과. 넘으면 calib_sweep → handeye_calibration.py 재실행.
카메라 좌표는 기본으로 "픽셀 + 깊이" (마커 크기 가정이 없어 PnP 보다 안정). --use-pnp 로 PnP tvec 사용.
"""
import argparse
import json
import os
import sys

import numpy as np


def rot_zyz(a_deg: float, b_deg: float, c_deg: float) -> np.ndarray:
    """두산 posx 회전 (ZYZ, deg) → 3×3. rally_robot.dsr_helpers.get_robot_pose_matrix 와 같은 정의."""
    a, b, c = np.radians([a_deg, b_deg, c_deg])
    def rz(t): return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])
    def ry(t): return np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])
    return rz(a) @ ry(b) @ rz(c)


def posx_to_T(posx) -> np.ndarray:
    T = np.eye(4); T[:3, :3] = rot_zyz(*posx[3:6]); T[:3, 3] = posx[:3]
    return T


def marker_cam_xyz_mm(m: dict, K: np.ndarray, use_pnp: bool, depth_scale: float = 1.0) -> np.ndarray | None:
    if use_pnp or m.get("z_depth") is None:
        return np.array(m["tvec"]) * 1000.0
    u, v = m["px"]; z = m["z_depth"] * 1000.0 * depth_scale
    return np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z])


def read_posx_from_topic(topic: str, timeout_s: float = 5.0):
    import rclpy, time
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    rclpy.init(); n = Node("verify_calib"); got = {}
    n.create_subscription(PoseStamped, topic, lambda m: got.setdefault("p", m), qos_profile_sensor_data)
    t0 = time.time()
    while "p" not in got and time.time() - t0 < timeout_s:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node(); rclpy.try_shutdown()
    if "p" not in got:
        raise SystemExit(f"{topic} 수신 없음")
    p, q = got["p"].pose.position, got["p"].pose.orientation
    # 쿼터니언 → 행렬 (m → mm). ZYZ 로 되돌릴 필요 없이 행렬을 바로 쓴다
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [p.x * 1000, p.y * 1000, p.z * 1000]
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("json", help="aruco_detect.py --json 출력")
    ap.add_argument("--posx", type=float, nargs=6, metavar=("X", "Y", "Z", "RX", "RY", "RZ"), help="검출 순간 TCP posx (mm, ZYZ deg)")
    ap.add_argument("--posx-topic", help="대신 /rally/robot/tcp_pose 에서 읽기")
    ap.add_argument("--npy", default="ros2_ws/src/rally_robot/resource/T_gripper2camera.npy")
    ap.add_argument("--touch", action="append", default=[], help="name=x,y,z  TCP 로 찍은 마커 중심 (base, mm). 여러 번")
    ap.add_argument("--use-pnp", action="store_true", help="깊이 대신 PnP tvec 사용")
    ap.add_argument("--tol-mm", type=float, default=5.0)
    ap.add_argument("--depth-scale", type=float, default=1.0, help="깊이 보정 배율 (hand_eye.yaml depth_scale, 10/8 기준 0.971)")
    a = ap.parse_args()

    d = json.load(open(a.json)); K = np.array(d["K"])
    g2c = np.load(a.npy)                                     # TCP ← 카메라 (mm)
    T_bg = read_posx_from_topic(a.posx_topic) if a.posx_topic else (posx_to_T(a.posx) if a.posx else None)
    if T_bg is None:
        raise SystemExit("--posx 또는 --posx-topic 이 필요합니다")
    T_bc = T_bg @ g2c                                        # base ← 카메라
    touch = {}
    for t in a.touch:
        name, xyz = t.split("="); touch[name] = np.array([float(v) for v in xyz.split(",")])

    print(f"TCP(base) = {np.round(T_bg[:3, 3], 1)} mm,  카메라(base) = {np.round(T_bc[:3, 3], 1)} mm")
    print(f"{'마커':<6}{'카메라 xyz(mm)':<28}{'base xyz(mm)':<28}{'터치 xyz(mm)':<28}{'오차'}")
    worst = 0.0
    for m in sorted(d["markers"], key=lambda m: m["name"]):
        pnp_only = (not a.use_pnp) and m.get("z_depth") is None     # 깊이 없는 마커(화면 가장자리) — PnP 는 2 cm 마커에서 수 cm 틀리므로 판정에서 뺌
        c = marker_cam_xyz_mm(m, K, a.use_pnp, a.depth_scale)
        if c is None:
            continue
        b = (T_bc @ np.append(c, 1.0))[:3]
        line = f"{m['name']:<6}{str(np.round(c, 1)):<28}{str(np.round(b, 1)):<28}"
        if pnp_only:
            print(line + "(깊이 없음 → PnP, 판정 제외)"); continue
        if m["name"] in touch:
            e = b - touch[m["name"]]; en = float(np.linalg.norm(e)); worst = max(worst, en)
            line += f"{str(np.round(touch[m['name']], 1)):<28}{en:5.1f} mm (dx {e[0]:+.1f} dy {e[1]:+.1f} dz {e[2]:+.1f})"
        print(line)
    if touch:
        print(f"\n최대 오차 {worst:.1f} mm → {'통과 (≤ %.0f mm)' % a.tol_mm if worst <= a.tol_mm else '불합격 → 핸드아이 캘리브레이션 재실행 (calib_sweep + handeye_calibration.py)'}")
        if "p1" in touch:
            p = touch["p1"] / 1000.0
            print(f"table.yaml 제안: table_link 원점(base) = [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}] m  (p1 터치값)")
    else:
        print("\n--touch 가 없어 base 좌표만 출력. 정정인이 TCP 로 마커를 찍은 값을 --touch p1=x,y,z 로 넣으면 오차가 나옵니다.")


if __name__ == "__main__":
    main()
