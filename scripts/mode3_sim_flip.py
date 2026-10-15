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
    python3 scripts/mode3_sim_flip.py --pick old back back2    # 패드 집기 접근 경로 3가지를 차례로 (대기 → … → G → 들어 올려 대기), 거치대 여유 비교

집기 접근 (--pick, 10/15 #208): 교시점 A·G·U(skills.yaml mode3.pad_rack_*_posj) 를 두산 fkin 으로 posx 로 바꿔 실제 스킬과 같은 명령으로 움직인다.
  old   : 대기 → (팔 대기·손목 A) → A [관절] → G [직선 4 cm]           — main 의 지금 경로. A 까지 관절 곡선으로 휘어 들어옴
  back  : 대기 → (팔 대기·손목 U) → U [관절] → B [직선] → A → G         — 브랜치 pad_rack_back_m 0.06. B = A 뒤 6 cm(손잡이 축), U → B 가 대각선
  back2 : … → U [관절] → U 높이 그대로 B 뒤로 [직선] → 아래로 B [직선] → A → G — 거치대에서 떨어진 곳에서 내려오고 손잡이 근처는 툴 z 직선만
  판정: 집기 전 = 빈 그리퍼(패드 없음) ↔ 상자 z 0.30(거치대+꽂힌 패드) / 집은 뒤 = 패드 포함 ↔ 상자 z 0.17(거치대만).
  A·G 구간은 손잡이를 쥐는 자리라 어느 경로든 상자에 붙는다(교시점이 같아 값도 같음) → 비교는 **A 도착 전 구간**만. 기준선 = 기존 자세·old 경로(실로봇에서 됨).

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
RACK_POSJ = dict(G=[91.1, -26.2, -117.8, -129.9, 116.6, -66.4], A=[87.4, -29.1, -112.6, -132.7, 119.8, -66.1],
                 U=[91.7, -7.9, -103.8, -109.3, 132.7, -31.8])   # skills.yaml mode3.pad_rack_*_posj (10/12 정정인 교시)


class RackView(Node):
    def __init__(self, warn_m, pad_in, pad_held=True):
        super().__init__("rally_mode3_sim_flip")
        # 로봇 쪽: 팔 link_2~6(두산 충돌 메시) + 그리퍼 캡슐(4 cm) + 패드 원판(쥐고 있을 때만). 집기 접근은 빈 그리퍼 → 패드 없음
        self.body_pad = RobotBody(pad_offset_tool_m=PAD_OFFSET_TOOL, max_pts_per_link=600)
        self.body_nopad = RobotBody(max_pts_per_link=600)
        self.box_pad = Box((RACK_LO[0], RACK_LO[1], RACK_HI[2]), (RACK_HI[0], RACK_HI[1], RACK_PAD_TOP), "rack+pad")
        self.pad_held = pad_held
        self.body = self.body_pad if pad_held else self.body_nopad
        self.boxes = [Box(RACK_LO, RACK_HI, "rack")] + ([self.box_pad] if pad_in else [])
        self.points = {}                                          # 이름 → base m (A·G·U·B·U2 표시)
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

    def set_state(self, pad_held: bool):
        """집기 전(패드가 거치대에: 상자 z 0.30, 빈 그리퍼) ↔ 집은 뒤(패드가 그리퍼에: 상자 z 0.17, 패드 원판 포함)."""
        self.pad_held = pad_held
        self.body = self.body_pad if pad_held else self.body_nopad
        self.boxes = [Box(RACK_LO, RACK_HI, "rack")] + ([] if pad_held else [self.box_pad])

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
        for k, (name, p) in enumerate(sorted(self.points.items())):
            sp = Marker()
            sp.header.stamp, sp.header.frame_id = now, "base_link"
            sp.ns, sp.id, sp.type, sp.action = "mode3_rack_pts", 2 * k, Marker.SPHERE, Marker.ADD
            sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = p
            sp.pose.orientation.w = 1.0
            sp.scale.x = sp.scale.y = sp.scale.z = 0.02
            sp.color.r, sp.color.g, sp.color.b, sp.color.a = (1.0, 0.85, 0.1, 1.0) if name == "G" else (0.3, 0.6, 1.0, 1.0)
            arr.markers.append(sp)
            tx = Marker()
            tx.header.stamp, tx.header.frame_id = now, "base_link"
            tx.ns, tx.id, tx.type, tx.action = "mode3_rack_pts", 2 * k + 1, Marker.TEXT_VIEW_FACING, Marker.ADD
            tx.pose.position.x, tx.pose.position.y, tx.pose.position.z = p[0], p[1], p[2] + 0.03
            tx.pose.orientation.w = 1.0
            tx.scale.z = 0.03
            tx.color.r = tx.color.g = tx.color.b = tx.color.a = 1.0
            tx.text = name
            arr.markers.append(tx)
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
    ap.add_argument("--pick", nargs="+", choices=["old", "back", "back2"], help="패드 집기 접근 경로를 차례로 시뮬 (--run 과 따로)")
    ap.add_argument("--back", type=float, default=0.06, help="back·back2: A 뒤 몇 m 에서 손잡이 축으로 들어올지")
    ap.add_argument("--lin-vel", type=float, default=60.0, help="직선 이동 mm/s (실제 스킬 pad_move_vel 40)")
    opt = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("--ros-args")])

    rclpy.init()
    view = RackView(opt.warn, opt.pad_in or bool(opt.pick), pad_held=not opt.pick)
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
    if opt.pick:
        return run_pick(view, opt)
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


def run_pick(view, opt):
    """집기 접근 경로 비교 (#208). 실제 pad_rack.pick_pad 와 같은 명령(관절 movej·직선 movel) 순서."""
    from rally_robot.dsr_helpers import init_dsr
    dsr = init_dsr("rally_mode3_sim_pick_dsr")
    from DSR_ROBOT2 import amovej, amovel, check_motion, fkin, get_robot_state  # noqa: E402  init_dsr 뒤

    def wait(label, limit=90.0):
        t = time.monotonic()
        while True:
            time.sleep(0.02)
            if view.safety is not None and view.safety.level == 2:
                raise SystemExit(f"{label} 중 안전 STOP ({view.safety.reason})")
            if time.monotonic() - t > 0.3 and check_motion() == 0:
                return
            if time.monotonic() - t > limit:
                raise SystemExit(f"{label}: {limit:.0f} s 초과 (에뮬레이터가 멈췄으면 sim_robot.sh down → up)")

    log = []

    def move(kind, target, label, phase="approach"):
        view.track_min = (math.inf, "", None)
        if kind == "j":
            amovej([float(v) for v in target], vel=opt.posj_vel, acc=opt.posj_vel * 2)
        else:
            amovel([float(v) for v in target], vel=opt.lin_vel, acc=opt.lin_vel * 2)
        wait(label)
        tm, view.track_min = view.track_min, None
        if get_robot_state() not in (1, 2):
            raise SystemExit(f"{label} 뒤 로봇 상태 {get_robot_state()} (알람)")
        mark = "겹침!" if tm[0] < 0 else ("가까움" if tm[0] < opt.warn else "")
        note = "" if phase == "approach" else "  (손잡이 쥐는 자리 — 교시점 같음, 비교 대상 아님)" if phase == "grasp" else "  (패드 쥠)"
        print(f"    {label:34s} 여유 {tm[0] * 100:6.1f} cm ({tm[1]}) {mark}{note}")
        log.append((label, tm[0], phase))

    P = {k: [float(v) for v in fkin(q)[:6]] if not isinstance(fkin(q), (int, float)) else None for k, q in RACK_POSJ.items()}
    if any(v is None for v in P.values()):
        raise SystemExit("fkin 실패 — 에뮬레이터 확인")
    A, G, U = P["A"], P["G"], P["U"]
    d = np.array(A[:3]) - np.array(G[:3]); d /= np.linalg.norm(d)                 # 손잡이 축, 거치대에서 나오는 방향 (= 툴 −z)
    back_mm = (np.linalg.norm(np.array(A[:3]) - np.array(G[:3])) + opt.back * 1000.0)
    B = [float(v) for v in np.array(G[:3]) + d * back_mm] + A[3:6]                 # G 뒤 (4 + back) cm, 손잡이 축 높이
    U2 = [float(v) for v in np.array(U[:3]) + d * back_mm] + U[3:6]                # U 높이 그대로 B 만큼 뒤
    view.points = {n: tuple(np.array(p[:3]) / 1000.0) for n, p in dict(A=A, G=G, U=U, B=B, U2=U2).items()}
    ready = [float(v) for v in opt.posj]
    print(f"G {np.round(G[:3])} · A {np.round(A[:3])} (G 뒤 {np.linalg.norm(np.array(A[:3]) - np.array(G[:3])):.0f} mm, 툴 z 축) · U {np.round(U[:3])}")
    print(f"B {np.round(B[:3])} (G 뒤 {back_mm:.0f} mm) · U2 {np.round(U2[:3])}")
    print(f"▶ 대기 자세로 {ready}")
    view.set_state(True)
    move("j", ready, "대기 자세", "held")
    summary = {}
    for mode in opt.pick:
        print(f"\n[{mode}]  집기 전: 빈 그리퍼, 상자 z 0.30(패드 꽂힘)")
        view.set_state(False); log.clear()
        if mode == "old":
            move("j", ready[:3] + RACK_POSJ["A"][3:], "대기 → (팔 대기·손목 A) 관절")
            move("j", RACK_POSJ["A"], "→ A 관절")
            move("l", G, "A → G 직선 (툴 z 4 cm)", "grasp")
        else:
            move("j", ready[:3] + RACK_POSJ["U"][3:], "대기 → (팔 대기·손목 U) 관절")
            move("j", RACK_POSJ["U"], "→ U 관절")
            if mode == "back":
                move("l", B, "U → B 직선 (대각선 아래)")
            else:
                move("l", U2, "U → U2 직선 (같은 높이로 뒤)")
                move("l", B, "U2 → B 직선 (아래로)")
            move("l", A, "B → A 직선 (툴 z)", "grasp")
            move("l", G, "A → G 직선 (툴 z)", "grasp")
        print("    — 쥠 — 집은 뒤: 패드가 그리퍼에, 상자 z 0.17(거치대만)")
        view.set_state(True)
        move("l", U, "G → U 직선 (홈 따라 들어 올림)", "held")
        move("j", ready[:3] + RACK_POSJ["U"][3:], "U → (팔 대기·손목 U) 관절", "held")
        move("j", ready, "→ 대기 자세", "held")
        app = min(r[1] for r in log if r[2] == "approach"); held = min(r[1] for r in log if r[2] == "held")
        summary[mode] = (app, held)
    print("\n요약 — A 도착 전 구간(비교 대상) 그리퍼·팔 최소 여유 / 집은 뒤 최소 여유:")
    for mode, (app, held) in summary.items():
        print(f"  {mode:6s} 접근 {app * 100:6.1f} cm   집은 뒤 {held * 100:6.1f} cm")
    print("  기준: 기존 자세 + old 경로가 실로봇에서 됨(접근 −2.6 cm 계산). 그보다 나쁘면 안 됨. A·G 구간은 교시점이 같아 모든 경로가 같음")
    dsr.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
