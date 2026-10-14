#!/usr/bin/env python3
"""모드 3 대기 자세 '어깨 반대'(#208) 시뮬 — RViz 에 패드 거치대를 초록 상자로 그리고, 받기 이동을 돌리며 거치대 여유를 잰다.

두산 에뮬레이터는 충돌 모델이 없어 부딪혀도 그냥 지나간다. 그래서 이 스크립트가 /dsr01/joint_states 를 받아
두산 충돌 메시(m0609_collision.RobotBody) ↔ 거치대 상자 거리를 직접 계산한다:
  초록 = 여유 ≥ --warn(5 cm) · 노랑 = 0 ~ --warn · 빨강 = 겹침. RViz 에 지금 여유와 가장 가까운 부위를 글자로 띄운다.

거치대 (정정인 #208, base m): x 0.25~0.30 · y −0.25~−0.05 · z 0~0.17 (패드 없음 — 패드를 쥐고 받는 중) — 초록 상자
                               z 0.17~0.30 (패드 꽂힌 상태에서 더해지는 높이) — 옅은 초록 상자 (--pad-in 이면 여유 계산에도 넣음)

    scripts/sim_robot.sh up                                   # 에뮬레이터 + RViz (도메인 77)
    source scripts/sim_robot.sh env
    python3 scripts/mode3_sim_flip.py                          # 상자·여유 표시만 (로봇 안 움직임) — 펜던트·다른 스크립트로 움직이며 보기
    python3 scripts/mode3_sim_flip.py --run                    # 어깨 반대 자세로 movej → 패드 평면 12방향 × 15·23 cm amovej → 매번 복귀
    python3 scripts/mode3_sim_flip.py --run --dist 15 23 25 --angles 0 45 90 135 180 225 270 315 --vel 120 --acc 400

방향은 mode3_reach_test.directions 와 같다 (0° 오른쪽 −y · 90° 위 · 180° 왼쪽 · 270° 아래, 패드 실제 법선 평면).
왼쪽은 --left-max(기본 0.23 m)까지만 — 어깨 반대 자세는 왼쪽 24~25 cm 에서 손목이 특이점 쪽 (|sin J5| < 0.5).
시뮬 전용: 실로봇에서 돌리지 말 것 (안전 확인 순서는 docs/mode3/MODE3_READY_POSE_Y0.md 9절). DSR_ROBOT2 는 init_dsr() 뒤 import.
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ejt_wrench import JOINTS  # noqa: E402
from m0609_collision import Box, RobotBody  # noqa: E402
from mode3_reach_test import Kin, directions  # noqa: E402

FLIP_POSJ = [123.1, -7.0, -100.2, -80.5, 61.9, -100.6]          # #208 어깨 반대 (Y0 와 TCP·툴 회전 같음)
PAD_OFFSET_TOOL = (-0.0175, -0.1043, 0.0285)                    # skills.yaml mode3.pad_offset_tool_m (10/12 실측)
RACK_LO, RACK_HI, RACK_PAD_TOP = (0.25, -0.25, 0.0), (0.30, -0.05, 0.17), 0.30


class RackView(Node):
    def __init__(self, warn_m, pad_in):
        super().__init__("rally_mode3_sim_flip")
        self.body = RobotBody(pad_offset_tool_m=PAD_OFFSET_TOOL, max_pts_per_link=600)
        self.boxes = [Box(RACK_LO, RACK_HI, "rack")]
        if pad_in:
            self.boxes.append(Box((RACK_LO[0], RACK_LO[1], RACK_HI[2]), (RACK_HI[0], RACK_HI[1], RACK_PAD_TOP), "rack+pad"))
        self.warn = warn_m
        self.q, self.clear, self.who = None, math.inf, ""
        self.lock = threading.Lock()
        self.track_min = None                                     # 측정 구간 최소 여유 (move 단위)
        self.create_subscription(JointState, "/dsr01/joint_states", self.on_js, qos_profile_sensor_data)
        self.safety = None
        from rally_interfaces.msg import SafetyStatus
        self.create_subscription(SafetyStatus, "/rally/safety/status", lambda m: setattr(self, "safety", m),
                                 QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10))
        self.pub = self.create_publisher(MarkerArray, "/rally/sim/scene", 5)   # sim.rviz 가 보는 토픽 (sim_gripper 와 ns 만 다름)
        self.create_timer(0.1, self.tick)

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(k in d for k in JOINTS):
            with self.lock:
                self.q = np.array([d[k] for k in JOINTS])

    def tick(self):
        with self.lock:
            q = None if self.q is None else self.q.copy()
        if q is not None:
            c, who = self.body.clearance(q, self.boxes)
            self.clear, self.who = c, who
            if self.track_min is not None and c < self.track_min[0]:
                self.track_min = (c, who, np.degrees(q).round(1).tolist())
        self.publish_markers()

    def color(self):
        if self.clear < 0:
            return (0.90, 0.15, 0.15, 0.75)
        if self.clear < self.warn:
            return (0.98, 0.78, 0.10, 0.70)
        return (0.18, 0.75, 0.30, 0.65)

    def publish_markers(self):
        arr, now = MarkerArray(), self.get_clock().now().to_msg()

        def box(mid, lo, hi, rgba):
            m = Marker()
            m.header.stamp, m.header.frame_id = now, "base_link"
            m.ns, m.id, m.type, m.action = "mode3_rack", mid, Marker.CUBE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = [(a + b) / 2 for a, b in zip(lo, hi)]
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = [b - a for a, b in zip(lo, hi)]
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            arr.markers.append(m)

        box(0, RACK_LO, RACK_HI, self.color())
        box(1, (RACK_LO[0], RACK_LO[1], RACK_HI[2]), (RACK_HI[0], RACK_HI[1], RACK_PAD_TOP), (0.18, 0.75, 0.30, 0.18))
        t = Marker()
        t.header.stamp, t.header.frame_id = now, "base_link"
        t.ns, t.id, t.type, t.action = "mode3_rack", 2, Marker.TEXT_VIEW_FACING, Marker.ADD
        t.pose.position.x, t.pose.position.y, t.pose.position.z = 0.275, -0.15, RACK_PAD_TOP + 0.08
        t.pose.orientation.w = 1.0
        t.scale.z = 0.04
        t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0, 1.0
        t.text = "rack clearance --" if not math.isfinite(self.clear) else f"rack clearance {self.clear * 100:.1f} cm ({self.who})"   # RViz 글꼴에 한글 없음
        arr.markers.append(t)
        self.pub.publish(arr)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="store_true", help="어깨 반대 자세로 가서 받기 이동을 돌림 (없으면 표시만)")
    ap.add_argument("--posj", type=float, nargs=6, default=FLIP_POSJ)
    ap.add_argument("--angles", type=float, nargs="+", default=list(range(0, 360, 30)))
    ap.add_argument("--dist", type=float, nargs="+", default=[15, 23], help="cm")
    ap.add_argument("--left-max", type=float, default=0.23, help="왼쪽 성분 상한 m (어깨 반대 자세 왼쪽 24~25 cm 손목 특이점)")
    ap.add_argument("--vel", type=float, default=120.0, help="amovej 관절 °/s (받기와 같음)")
    ap.add_argument("--acc", type=float, default=400.0, help="amovej 관절 °/s²")
    ap.add_argument("--posj-vel", type=float, default=30.0, help="대기 자세로 가는 관절 °/s")
    ap.add_argument("--warn", type=float, default=0.05, help="노랑 기준 여유 m")
    ap.add_argument("--pad-in", action="store_true", help="꽂힌 패드 높이(z 0.30)까지 여유 계산에 넣기 (집기·꽂기 경로용)")
    opt = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("--ros-args")])

    rclpy.init()
    view = RackView(opt.warn, opt.pad_in)
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor()
    ex.add_node(view)
    threading.Thread(target=ex.spin, daemon=True).start()
    t0 = time.monotonic()
    while view.q is None and time.monotonic() - t0 < 5:
        time.sleep(0.05)
    if view.q is None:
        print("/dsr01/joint_states 를 못 받음 — scripts/sim_robot.sh up · source scripts/sim_robot.sh env 확인")
        return 1
    if not opt.run:
        print("RViz 에 거치대(초록) 표시 중 — Ctrl+C 로 끝. 여유는 1 s 마다 출력")
        try:
            while True:
                time.sleep(1.0)
                print(f"관절 {np.degrees(view.q).round(1).tolist()}  거치대 여유 {view.clear * 100:6.1f} cm  ({view.who})")
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
        return 0

    from rally_robot.dsr_helpers import init_dsr
    dsr = init_dsr("rally_mode3_sim_flip_dsr")
    from DSR_ROBOT2 import amovej, check_motion, get_robot_state  # noqa: E402  init_dsr 뒤

    def wait(label, limit=60.0):
        t = time.monotonic()
        while True:
            time.sleep(0.02)
            if view.safety is not None and view.safety.level == 2:
                raise SystemExit(f"{label} 중 안전 STOP ({view.safety.reason})")
            if time.monotonic() - t > 0.3 and check_motion() == 0:
                return time.monotonic() - t
            if time.monotonic() - t > limit:
                raise SystemExit(f"{label}: {limit:.0f} s 초과")

    def go(q_deg, vel, acc, label):
        view.track_min = (math.inf, "", None)
        amovej([float(v) for v in q_deg], vel=vel, acc=acc)
        dt = wait(label)
        tm, view.track_min = view.track_min, None
        if get_robot_state() not in (1, 2):
            raise SystemExit(f"{label} 뒤 로봇 상태 {get_robot_state()} (알람)")
        return dt, tm

    print(f"▶ 어깨 반대 자세로 (관절 {opt.posj_vel:.0f} °/s): {opt.posj}")
    dt, tm = go(opt.posj, opt.posj_vel, opt.posj_vel * 2, "대기 자세 이동")
    print(f"  도착 {dt:.1f} s · 경로 중 거치대 최소 여유 {tm[0] * 100:.1f} cm ({tm[1]})")

    kin = Kin()
    q0 = np.radians(opt.posj)
    p0, R0 = kin.tcp(q0)
    dirs = directions(opt.angles, R0)
    rows = []
    for dc in opt.dist:
        for name, d in dirs.items():
            a = math.radians(float(name.rstrip("°")))
            r = dc / 100.0
            du, dw = r * math.cos(a), r * math.sin(a)          # du>0 오른쪽(−y 쪽), du<0 왼쪽
            if du < -opt.left_max:
                scale = opt.left_max / -du
                du, dw = du * scale, dw * scale
            dist = math.hypot(du, dw)
            q = kin.ik(p0 + d * dist, R0, q0)
            if q is None:
                print(f"{name:>5} {dc:4.0f} cm: IK 실패 — 건너뜀"); continue
            dt, tm = go(np.degrees(q), opt.vel, opt.acc, f"{name} {dc:.0f} cm")
            rows.append((name, dc, dist * 100, dt, tm[0], tm[1]))
            mark = "겹침!" if tm[0] < 0 else ("가까움" if tm[0] < opt.warn else "")
            print(f"{name:>5} {dc:4.0f} cm (실제 {dist * 100:4.1f}): 이동 {dt:.2f} s · 거치대 최소 여유 {tm[0] * 100:5.1f} cm ({tm[1]}) {mark}")
            go(opt.posj, opt.vel / 2, opt.acc / 2, "복귀")

    if rows:
        worst = min(rows, key=lambda r: r[4])
        print(f"\n{len(rows)}번 이동 · 거치대 최소 여유 {worst[4] * 100:.1f} cm ({worst[0]} {worst[1]:.0f} cm, {worst[5]}) · "
              f"겹침 {sum(r[4] < 0 for r in rows)} · {opt.warn * 100:.0f} cm 안 {sum(0 <= r[4] < opt.warn for r in rows)}")
    dsr.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
