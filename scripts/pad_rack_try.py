#!/usr/bin/env python3
"""패드 거치대 교시 확인 — A(접근점) → G(잡는 자세) → 쥐기 → [--lift: U(들어 올림) → G(다시 꽂기)] → 놓기 → A.
단계마다 Enter 로 진행 (q 면 멈춤).

    source scripts/robot.sh env
    python3 scripts/pad_rack_try.py              # 20 mm/s
    python3 scripts/pad_rack_try.py --vel 10
    python3 scripts/pad_rack_try.py --vel 10 --lift     # 쥔 채 U 로 들어 올렸다가 다시 G 로 꽂기 (내려갈 땐 --vel 의 절반)
G·A 는 skills.yaml mode3.pad_rack_grasp_posj · pad_rack_approach_posj (교시한 관절값). print_pose.py 의 posx 는 URDF 계산이라
컨트롤러 좌표와 몇 mm 다르므로(10/12 실로봇: posx 로 movel 하니 z 로 살짝 내려감) 두산 fkin 으로 컨트롤러 posx 를 구해 쓴다.
A 로는 관절 이동(교시 자세 그대로), A ↔ G 는 컨트롤러 posx 직선 이동.
안전: jog.py 와 같다 — /rally/safety/status 가 NORMAL 일 때만 출발, 이동 중 STOP 이면 move_stop, 작업 공간 밖이면 출발 안 함.
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from jog import Jog, workspace  # noqa: E402

from rally_interfaces.msg import SafetyStatus  # noqa: E402
from rally_interfaces.srv import GripperCommand  # noqa: E402


def skills_cfg():
    from ament_index_python.packages import get_package_share_directory
    return yaml.safe_load(open(f"{get_package_share_directory('rally_bringup')}/config/skills.yaml"))


def approach_point(grasp, back_m):
    """G 에서 툴 z 반대로 back_m (mm·deg posx, ZYZ)."""
    z = Rotation.from_euler("ZYZ", grasp[3:6], degrees=True).as_matrix()[:, 2]
    return [grasp[i] - z[i] * back_m * 1000.0 for i in range(3)] + list(grasp[3:6])


def fkin(n, posj):
    """두산 컨트롤러 순기구학 posj(deg) → posx(mm·deg, base)."""
    from dsr_msgs2.srv import Fkin
    if not hasattr(n, "cli_fkin"):
        n.cli_fkin = n.create_client(Fkin, "/dsr01/dsr_controller2/motion/fkin")
    r = n.call(n.cli_fkin, Fkin.Request(pos=[float(v) for v in posj], ref=0))
    if not r.success:
        raise SystemExit(f"fkin 실패 {posj}")
    return [float(v) for v in r.conv_posx]


def movej_abs(n, posj, vel_deg, tgt_posx):
    """교시 관절값으로 관절 이동. 도착은 tcp_pose 가 tgt_posx(=fkin(posj)) 1.5 mm 안."""
    from dsr_msgs2.srv import MoveJoint
    if n.safety is None or n.safety.level != SafetyStatus.LEVEL_NORMAL:
        raise SystemExit(f"안전 {getattr(n.safety, 'reason', '?')} — NORMAL 일 때만 움직임. scripts/robot.sh reset")
    if not hasattr(n, "cli_movej"):
        n.cli_movej = n.create_client(MoveJoint, "/dsr01/dsr_controller2/motion/move_joint")
    v = vel_deg * float(n.safety.speed_override or 1.0)
    cur = n.posx()
    print(f"  관절 이동 → posj {posj} ({v:.0f} °/s) · 위치 {[round(x, 1) for x in cur[:3]]} → {[round(x, 1) for x in tgt_posx[:3]]}")
    n.call(n.cli_movej, MoveJoint.Request(pos=[float(x) for x in posj], vel=v, acc=2 * v, mode=0, sync_type=1))
    t0 = time.monotonic()
    while True:
        rclpy.spin_once(n, timeout_sec=0.02)
        if n.safety is not None and n.safety.level == SafetyStatus.LEVEL_STOP:
            n.call(n.cli_stop, n.MoveStop.Request(stop_mode=1))
            raise SystemExit(f"안전 STOP ({n.safety.reason}) — 멈춤")
        if n.tcp is not None and time.monotonic() - t0 > 0.5:
            p = n.tcp.pose.position
            if math.dist((p.x * 1000.0, p.y * 1000.0, p.z * 1000.0), tgt_posx[:3]) < 1.5:
                break
        if time.monotonic() - t0 > 30.0:
            n.call(n.cli_stop, n.MoveStop.Request(stop_mode=1))
            raise SystemExit("시간 초과 — 멈춤")
    time.sleep(0.3)
    print(f"  도착 posx {[round(x, 1) for x in n.posx()]}")


def move_abs(n, tgt, vel):
    if n.safety is None or n.safety.level != SafetyStatus.LEVEL_NORMAL:
        raise SystemExit(f"안전 {getattr(n.safety, 'reason', '?')} — NORMAL 일 때만 움직임. scripts/robot.sh reset")
    ws = workspace()
    for i, a in enumerate("xyz"):
        if not ws[a][0] <= tgt[i] / 1000.0 <= ws[a][1]:
            raise SystemExit(f"목표 {a}={tgt[i] / 1000.0:.3f} m 가 작업 공간 {ws[a]} 밖 — 안 움직임")
    cur = n.posx()
    dist = math.dist(cur[:3], tgt[:3])
    v = vel * float(n.safety.speed_override or 1.0)
    print(f"  {[round(x, 1) for x in cur[:3]]} → {[round(x, 1) for x in tgt[:3]]}  ({dist:.0f} mm, {v:.0f} mm/s)")
    n.call(n.cli_line, n.MoveLine.Request(pos=tgt, vel=[v, 20.0], acc=[2 * v, 40.0], ref=0, mode=0, sync_type=1))
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
        if time.monotonic() - t0 > dist / v + 6.0:
            n.call(n.cli_stop, n.MoveStop.Request(stop_mode=1))
            raise SystemExit("시간 초과 — 멈춤")
    time.sleep(0.3)
    print(f"  도착 posx {[round(x, 1) for x in n.posx()]}")


def gripper(n, cli, width, force):
    r = n.call(cli, GripperCommand.Request(width_mm=float(width), force_n=float(force), wait_until_done=True), timeout=6.0)
    print(f"  그리퍼 {width:.0f} mm 명령 → 폭 {r.actual_width_mm:.1f} mm, 물체 {r.object_detected}")
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vel", type=float, default=20.0, help="mm/s (최대 50)")
    ap.add_argument("--lift", action="store_true", help="쥔 뒤 U 로 들어 올렸다 다시 G 로 꽂기")
    opt = ap.parse_args()
    vel = min(max(opt.vel, 5.0), 50.0)
    c = skills_cfg()
    m3 = c.get("mode3") or {}
    g_posj = [float(v) for v in m3["pad_rack_grasp_posj"]]
    a_posj = [float(v) for v in m3["pad_rack_approach_posj"]]
    u_posj = [float(v) for v in m3.get("pad_rack_lift_posj") or []]
    if opt.lift and len(u_posj) != 6:
        raise SystemExit("skills.yaml mode3.pad_rack_lift_posj 없음")
    pad = (c.get("gripper") or {}).get("pad") or {"open_mm": 60, "force_n": 38}
    rclpy.init()
    n = Jog()
    gcli = n.create_client(GripperCommand, "/rally/gripper/command")
    try:
        if not n.wait(lambda: n.safety is not None and n.tcp is not None, 3.0):
            raise SystemExit("안전 상태·tcp_pose 못 받음 (터미널 2?)")
        grasp, a_pt = fkin(n, g_posj), fkin(n, a_posj)
        print(f"G 컨트롤러 posx {[round(x, 1) for x in grasp]}  (교시 posx {m3.get('pad_rack_grasp_posx')})")
        print(f"A 컨트롤러 posx {[round(x, 1) for x in a_pt]}  (교시 posx {m3.get('pad_rack_approach_posx')})")
        u_pt = fkin(n, u_posj) if opt.lift else None
        if u_pt:
            print(f"U 컨트롤러 posx {[round(x, 1) for x in u_pt]}  (G 에서 {u_pt[2] - grasp[2]:.0f} mm 위)")
        steps = [("A 로 관절 이동 (교시 자세 그대로, 그리퍼는 그대로)", lambda: movej_abs(n, a_posj, 10.0, a_pt)),
                 ("그리퍼 열기", lambda: gripper(n, gcli, pad["open_mm"], pad["force_n"])),
                 ("G 로 들어가기", lambda: move_abs(n, grasp, vel)),
                 ("쥐기", lambda: gripper(n, gcli, 0.0, pad["force_n"])),
                 *([("U 로 들어 올리기 (거치대 홈 따라)", lambda: move_abs(n, u_pt, vel)),
                    ("G 로 다시 꽂기 (절반 속도)", lambda: move_abs(n, grasp, vel / 2))] if opt.lift else []),
                 ("놓기", lambda: gripper(n, gcli, pad["open_mm"], pad["force_n"])),
                 ("A 로 빠지기", lambda: move_abs(n, a_pt, vel))]
        for i, (name, fn) in enumerate(steps, 1):
            if input(f"[{i}/{len(steps)}] {name} — Enter 진행, q 멈춤: ").strip().lower() == "q":
                print("멈춤")
                return
            rclpy.spin_once(n, timeout_sec=0.05)
            fn()
        print("끝")
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
