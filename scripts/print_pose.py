#!/usr/bin/env python3
"""현재 로봇 자세를 posx(mm, ZYZ deg) · posj(deg) · m 단위로 출력 — 티칭 값을 skills.yaml / table.yaml 에 적을 때.

    source scripts/env_robot_pc.sh          # 두산 bringup 이 떠 있어야 함
    python3 scripts/print_pose.py           # 한 번 출력
    python3 scripts/print_pose.py --watch   # 2 Hz 로 계속 출력 (직접교시로 움직이며 보기, Ctrl+C 로 종료)

/dsr01/joint_states(수동 모드·직접교시 중에도 나옴) + URDF 순기구학으로 계산한다. 두산 get_current_posx 서비스는
수동 모드에서 응답하지 않아 쓰지 않는다 (2026-10-07). TCP 는 플랜지에서 TCP_OFFSET_MM (실로봇 2FG_TCP 와 비교해 맞춘 값).
"""
import argparse
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ejt_wrench import JOINTS, Chain  # noqa: E402

TCP_OFFSET_MM = np.array([0.8, -1.4, 225.9])   # 플랜지(link_6) 프레임 기준. 10/7 실로봇 get_current_posx 와 비교해 실측


def zyz_deg(R):
    """회전행렬 → 두산 ZYZ (rx, ry, rz) [deg]."""
    ry = math.atan2(math.hypot(R[0, 2], R[1, 2]), R[2, 2])
    if abs(math.sin(ry)) < 1e-6:                      # 짐벌락: rz 를 0 으로
        rx, rz = math.atan2(R[1, 0], R[0, 0]), 0.0
    else:
        rx, rz = math.atan2(R[1, 2], R[0, 2]), math.atan2(R[2, 1], -R[2, 0])
    # 같은 회전의 다른 표기 (rx+180, -ry, rz+180) 중 두산 펜던트처럼 ry ≤ 0 쪽으로
    rx, ry, rz = rx + math.pi, -ry, rz + math.pi
    wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
    return [math.degrees(wrap(rx)), math.degrees(ry), math.degrees(wrap(rz))]


def tcp_posx(chain, q_rad):
    _, Tf = chain.fk_frames(q_rad)
    p = Tf[:3, 3] * 1000.0 + Tf[:3, :3] @ TCP_OFFSET_MM
    return [*p, *zyz_deg(Tf[:3, :3])]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", action="store_true")
    opt = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("--ros-args")])
    rclpy.init()
    node = Node("rally_print_pose")
    chain = Chain()
    latest = {}

    def on_js(m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in JOINTS):
            latest["q"] = np.array([d[n] for n in JOINTS])

    node.create_subscription(JointState, "/dsr01/joint_states", on_js, qos_profile_sensor_data)
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    t0 = time.monotonic()
    while "q" not in latest and time.monotonic() - t0 < 5.0:
        time.sleep(0.05)
    if "q" not in latest:
        print("/dsr01/joint_states 수신 없음 — bringup·env_robot_pc.sh 확인")
        ex.shutdown(timeout_sec=1.0)
        rclpy.try_shutdown()
        return 1
    try:
        while True:
            q = latest["q"].copy()
            px = tcp_posx(chain, q)
            pj = np.degrees(q)
            print(f"posx (mm, deg): [{', '.join(f'{v:.1f}' for v in px)}]")
            print(f"posj (deg)    : [{', '.join(f'{v:.1f}' for v in pj)}]")
            print(f"xyz (m)       : [{px[0]/1000:.4f}, {px[1]/1000:.4f}, {px[2]/1000:.4f}]")
            if not opt.watch:
                break
            print("---")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown(timeout_sec=1.0)
        th.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
