#!/usr/bin/env python3
"""손가락 끝(TCP) 속도 측정 — /dsr01/joint_states(100 Hz) + 순기구학. 스윙이 실제로 몇 m/s 나오는지 볼 때.

    source scripts/env_robot_pc.sh && python3 scripts/tcp_speed.py --seconds 40     # 그동안 PushBall 실행
출력: 0.1 m/s 넘는 구간마다 최고 속도·그 위치(x, z), 마지막에 전체 최고 속도.
"""
import argparse
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
from print_pose import TCP_OFFSET_MM  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=40.0)
    opt = ap.parse_args()
    rclpy.init()
    n = Node("rally_tcp_speed")
    chain = Chain()
    samples = []
    lock = threading.Lock()

    def on_js(m):
        d = dict(zip(m.name, m.position))
        if all(k in d for k in JOINTS):
            _, Tf = chain.fk_frames(np.array([d[k] for k in JOINTS]))
            tip = Tf[:3, 3] + Tf[:3, :3] @ (TCP_OFFSET_MM / 1000.0)
            t = time.monotonic()                 # 수신 시각 (header.stamp 는 몰려 들어와 속도가 튄다)
            with lock:
                samples.append((t, tip))

    n.create_subscription(JointState, "/dsr01/joint_states", on_js, qos_profile_sensor_data)
    th = threading.Thread(target=rclpy.spin, args=(n,), daemon=True)
    th.start()
    print(f"{opt.seconds:.0f} s 측정 중 — 지금 PushBall 을 보내세요")
    time.sleep(opt.seconds)
    with lock:
        s = list(samples)
    rclpy.try_shutdown()
    if len(s) < 10:
        print("joint_states 수신 없음")
        return 1
    # 0.1 s 창으로 속도 (조인트 스트림 지터 완화)
    sp = []
    j = 0
    for i in range(len(s)):
        while s[i][0] - s[j][0] > 0.1:
            j += 1
        if i > j and s[i][0] > s[j][0]:
            if s[i][0] - s[j][0] < 0.05:
                continue
            sp.append((s[i][0], float(np.linalg.norm(s[i][1] - s[j][1]) / (s[i][0] - s[j][0])), s[i][1]))
    seg, best = None, None
    for t, v, p in sp:
        if v > 0.1:
            if seg is None:
                seg = [t, v, p]
            elif v > seg[1]:
                seg[1], seg[2] = v, p
        elif seg is not None:
            print(f"  {seg[0] - s[0][0]:6.1f} s 부근 최고 {seg[1]:.2f} m/s @ x {seg[2][0] * 1000:.0f} z {seg[2][2] * 1000:.0f} mm")
            best = max(best or 0.0, seg[1])
            seg = None
    print(f"전체 최고 TCP 속도 {max(v for _, v, _ in sp):.2f} m/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
