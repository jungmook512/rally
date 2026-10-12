#!/usr/bin/env python3
"""베이스 좌표 상대 직선 이동 (자세 그대로) — 관측 자세 조정·카메라 화면 맞추기용. 실로봇 노드(터미널 2)가 떠 있어야 한다.

    source scripts/robot.sh env
    python3 scripts/jog.py --dy -0.10              # −y 로 10 cm (기본 50 mm/s)
    python3 scripts/jog.py --dx 0.03 --dz -0.02 --vel 30
안전: /rally/safety/status 가 NORMAL 일 때만 출발, 속도는 --vel × speed_override. 이동 중 STOP 이 오면 바로 move_stop.
      도착 위치가 safety.yaml 작업 공간 밖이면 출발하지 않는다. 끝나면 도착 posx 를 찍는다 (skills.yaml 에 옮겨 적기용).
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from rally_interfaces.msg import SafetyStatus

NS = "/dsr01/dsr_controller2"


class Jog(Node):
    def __init__(self):
        super().__init__("rally_jog")
        self.safety, self.tcp = None, None
        self.create_subscription(SafetyStatus, "/rally/safety/status", lambda m: setattr(self, "safety", m),
                                 QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10))
        self.create_subscription(PoseStamped, "/rally/robot/tcp_pose", lambda m: setattr(self, "tcp", m), qos_profile_sensor_data)
        from dsr_msgs2.srv import GetCurrentPosx, MoveLine, MoveStop
        self.MoveLine, self.MoveStop, self.GetPosx = MoveLine, MoveStop, GetCurrentPosx
        self.cli_line = self.create_client(MoveLine, f"{NS}/motion/move_line")
        self.cli_stop = self.create_client(MoveStop, f"{NS}/motion/move_stop")
        self.cli_posx = self.create_client(GetCurrentPosx, f"{NS}/aux_control/get_current_posx")

    def wait(self, pred, timeout):
        t = time.monotonic()
        while time.monotonic() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if pred():
                return True
        return False

    def call(self, cli, req, timeout=3.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"{cli.srv_name} 없음 (터미널 1 bringup?)")
        fut = cli.call_async(req)
        if not self.wait(fut.done, timeout):
            raise RuntimeError(f"{cli.srv_name} 무응답")
        return fut.result()

    def posx(self):
        r = self.call(self.cli_posx, self.GetPosx.Request(ref=0))
        return list(r.task_pos_info[0].data[:6])


def workspace():
    import yaml
    from ament_index_python.packages import get_package_share_directory
    p = f"{get_package_share_directory('rally_bringup')}/config/safety.yaml"
    c = yaml.safe_load(open(p))["rally_safety_monitor"]["ros__parameters"]
    return {a: (c[f"workspace_{a}_min_m"], c[f"workspace_{a}_max_m"]) for a in "xyz"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dx", type=float, default=0.0)
    ap.add_argument("--dy", type=float, default=0.0)
    ap.add_argument("--dz", type=float, default=0.0)
    ap.add_argument("--vel", type=float, default=50.0, help="mm/s (최대 100)")
    opt = ap.parse_args()
    d = [opt.dx * 1000.0, opt.dy * 1000.0, opt.dz * 1000.0]
    if math.hypot(*d) > 300.0:
        raise SystemExit("한 번에 30 cm 까지만")
    vel = min(max(opt.vel, 5.0), 100.0)

    rclpy.init()
    n = Jog()
    try:
        if not n.wait(lambda: n.safety is not None, 3.0):
            raise SystemExit("안전 상태 못 받음 (터미널 2 safety_monitor?) — 안 움직임")
        if n.safety.level != SafetyStatus.LEVEL_NORMAL:
            raise SystemExit(f"안전 {n.safety.level} ({n.safety.reason}) — NORMAL 일 때만 움직임. scripts/robot.sh reset")
        cur = n.posx()
        tgt = [cur[i] + (d[i] if i < 3 else 0.0) for i in range(6)]
        ws = workspace()
        for i, a in enumerate("xyz"):
            if not ws[a][0] <= tgt[i] / 1000.0 <= ws[a][1]:
                raise SystemExit(f"도착 {a}={tgt[i] / 1000.0:.3f} m 가 작업 공간 {ws[a]} 밖 — 안 움직임")
        v = vel * float(n.safety.speed_override or 1.0)
        print(f"지금 {[round(x, 1) for x in cur]}\n→ 목표 {[round(x, 1) for x in tgt]}  ({v:.0f} mm/s)")
        req = n.MoveLine.Request(pos=tgt, vel=[v, 30.0], acc=[2 * v, 60.0], ref=0, mode=0, sync_type=1)
        n.call(n.cli_line, req)
        t0 = time.monotonic()
        while True:
            rclpy.spin_once(n, timeout_sec=0.02)
            if n.safety is not None and n.safety.level == SafetyStatus.LEVEL_STOP:
                n.call(n.cli_stop, n.MoveStop.Request(stop_mode=1))
                raise SystemExit(f"안전 STOP ({n.safety.reason}) — 멈춤")
            if n.tcp is not None:
                p = n.tcp.pose.position
                if math.dist((p.x * 1000.0, p.y * 1000.0, p.z * 1000.0), tgt[:3]) < 1.5:
                    break
            if time.monotonic() - t0 > math.hypot(*d) / v + 5.0:
                n.call(n.cli_stop, n.MoveStop.Request(stop_mode=1))
                raise SystemExit("시간 초과 — 멈춤 (bringup 로그 알람 확인)")
        time.sleep(0.3)
        print(f"도착 posx {[round(x, 1) for x in n.posx()]}")
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
