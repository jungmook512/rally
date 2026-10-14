#!/usr/bin/env python3
"""모드 1 공 거치대 집기 시험 — 비전 좌표 → 로봇 좌표 오차를 눈으로 보고 오프셋을 바로 바꿔 가며 반복한다. 로봇·비전 노드가 떠 있어야 한다.

    source scripts/env_robot_pc.sh && source ros2_ws/install/setup.bash     # (로컬 모드면 scripts/robot.sh env)
    python3 scripts/rack_pick_test.py                    # 1회: 거치대 보기 → 비전 → 공 위 8 cm 에 떠서 멈춤 → Enter 하강·쥐기 → 폭 판정 → 제자리에 놓기
    python3 scripts/rack_pick_test.py --count 5          # 5회 반복 (회차마다 Enter)
    python3 scripts/rack_pick_test.py --offset -9.5 9.2  # 비전 보정 [mm] 직접 지정 (기본: skills.yaml mode1.vision_offset_m)
    python3 scripts/rack_pick_test.py --no-offset        # 보정 없이 (hand_eye.yaml 새 보정의 순수 오차 보기)
    python3 scripts/rack_pick_test.py --keep             # 집은 공을 제자리에 놓지 않고 쥔 채 끝

공 위에 떠 있을 때 프롬프트에 'x -5' · 'y 3' · 'x 0 y -5' 처럼 치면 (단위 mm) 그만큼 [mm] 옮겨서 다시 뜬다 (이번 회차 보정에 더해짐). 손가락 가운데가 공 중심에
오게 맞춘 뒤 Enter → 그 보정값이 요약에 찍힌다 → skills.yaml mode1.vision_offset_m 에 옮겨 적는다 (스킬 서버 재시작 필요).
기하는 push_ball.pick_ball_by_vision 과 같다: 거치대 보는 자세 pickup_view_posj · x 최소 공(choose_ball) · tool_down_zyz + pickup_yaw ·
위 pickup_approach_m → pickup_slow_above_m 까지 빠르게 → 아래로 pickup_below_center_m (하한 테이블 + place_min_tip_m) 천천히 · 그리퍼 gripper.ball.
다른 점: 순응 없이 직선 이동이라 멈추면(진행 없음 1 s) 접촉으로 본다. 힘 한계 25 N STOP 은 safety_monitor 가 그대로 건다 → 이 스크립트는 멈추고 끝낸다.
안전: STOP 이면 출발 안 함·이동 중 STOP 이면 move_stop. SLOW 면 speed_override 만큼 느리게 (스킬과 같음). 해제는 scripts/robot.sh reset.
"""
import argparse
import math
import os
import re
import sys
import time

import rclpy
import yaml
from rclpy.qos import qos_profile_sensor_data

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from jog import Jog, workspace  # noqa: E402
from pad_rack_try import fkin, gripper, movej_abs  # noqa: E402

from rally_interfaces.msg import SafetyStatus, TrackedObjectArray  # noqa: E402
from rally_interfaces.srv import GripperCommand  # noqa: E402
from rally_robot.skills.push_ball import DEFAULTS as PUSH_DEFAULTS, choose_ball  # noqa: E402


def load_cfg():
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("rally_bringup")
    skills = yaml.safe_load(open(f"{share}/config/skills.yaml"))
    table = yaml.safe_load(open(f"{share}/config/table.yaml")) or {}
    m1 = skills.get("mode1") or {}
    cfg = {**PUSH_DEFAULTS, **{k: v for k, v in m1.items() if k in PUSH_DEFAULTS}}
    ball = (skills.get("gripper") or {}).get("ball") or {"open_mm": 90, "force_n": 12}
    return cfg, float((table.get("table") or table).get("z_m") or 0.0), ball


def move_until(n, tgt, vel_mm_s, stall_ok=False, label=""):
    """직선 이동. 도착(1.5 mm) 또는 (stall_ok) 1 s 동안 진행 없음 → 'stalled'. STOP 이면 move_stop 후 SystemExit."""
    if n.safety is None or n.safety.level == SafetyStatus.LEVEL_STOP:
        raise SystemExit(f"안전 STOP ({getattr(n.safety, 'reason', '?')}) — scripts/robot.sh reset")
    ws = workspace()
    for i, a in enumerate("xyz"):
        if not ws[a][0] <= tgt[i] / 1000.0 <= ws[a][1]:
            raise SystemExit(f"목표 {a}={tgt[i] / 1000.0:.3f} m 가 작업 공간 {ws[a]} 밖 — 안 움직임")
    cur = n.posx()
    dist = math.dist(cur[:3], tgt[:3])
    v = vel_mm_s * float(n.safety.speed_override or 1.0)
    print(f"  {label} {[round(x, 1) for x in cur[:3]]} → {[round(x, 1) for x in tgt[:3]]} ({dist:.0f} mm, {v:.0f} mm/s)")
    n.call(n.cli_line, n.MoveLine.Request(pos=[float(x) for x in tgt], vel=[v, 20.0], acc=[2 * v, 40.0], ref=0, mode=0, sync_type=1))
    t0, last_p, last_move = time.monotonic(), None, time.monotonic()
    while True:
        rclpy.spin_once(n, timeout_sec=0.02)
        if n.safety is not None and n.safety.level == SafetyStatus.LEVEL_STOP:
            n.call(n.cli_stop, n.MoveStop.Request(stop_mode=1))
            raise SystemExit(f"안전 STOP ({n.safety.reason}) — 멈춤")
        if n.tcp is not None:
            p = n.tcp.pose.position
            pm = (p.x * 1000.0, p.y * 1000.0, p.z * 1000.0)
            if math.dist(pm, tgt[:3]) < 1.5:
                return "done"
            if last_p is None or math.dist(pm, last_p) > 0.5:
                last_p, last_move = pm, time.monotonic()
            elif stall_ok and time.monotonic() - t0 > 1.0 and time.monotonic() - last_move > 1.0:
                n.call(n.cli_stop, n.MoveStop.Request(stop_mode=1))
                return "stalled"
        if time.monotonic() - t0 > dist / max(v, 1.0) + 8.0:
            n.call(n.cli_stop, n.MoveStop.Request(stop_mode=1))
            raise SystemExit("시간 초과 — 멈춤")


class Picker(Jog):
    def __init__(self):
        super().__init__()
        self.objects = None
        self.create_subscription(TrackedObjectArray, "/rally/perception/objects", lambda m: setattr(self, "objects", (time.monotonic(), m)),
                                 qos_profile_sensor_data)
        self.gcli = self.create_client(GripperCommand, "/rally/gripper/command")

    def observe(self, cfg, table_z):
        """스킬과 같은 규칙: 멈춘 뒤 pickup_settle_s 지난 프레임, 연속 pickup_consistent_frames 가 pickup_consistent_m 안 → 중앙값."""
        t0, last, recent = time.monotonic(), None, []
        while time.monotonic() - t0 < cfg["pickup_wait_s"]:
            rclpy.spin_once(self, timeout_sec=0.02)
            o = self.objects
            if o is None or o[0] == last or o[0] - t0 < cfg["pickup_settle_s"]:
                continue
            last = o[0]
            if o[1].header.frame_id not in ("base_link", ""):
                raise SystemExit(f"비전 좌표가 {o[1].header.frame_id} — base_link 아님 (force_publisher tcp_pose 확인)")
            c = choose_ball(o[1], cfg, table_z)
            if c is None:
                continue
            recent = (recent + [c])[-cfg["pickup_consistent_frames"]:]
            if len(recent) == cfg["pickup_consistent_frames"] and all(
                    math.dist(r[:3], recent[-1][:3]) <= cfg["pickup_consistent_m"] for r in recent):
                xs = sorted(r[0] for r in recent); ys = sorted(r[1] for r in recent); zs = sorted(r[2] for r in recent)
                k = len(recent) // 2
                return xs[k], ys[k], zs[k], recent[-1][3]
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=1)
    ap.add_argument("--offset", type=float, nargs=2, metavar=("DX_MM", "DY_MM"), help="비전 보정 [mm] (기본 skills.yaml vision_offset_m)")
    ap.add_argument("--no-offset", action="store_true")
    ap.add_argument("--keep", action="store_true", help="집은 공을 제자리에 놓지 않음")
    ap.add_argument("--vel", type=float, default=15.0, help="하강 속도 mm/s")
    opt = ap.parse_args()
    cfg, tz, ball = load_cfg()
    off0 = [0.0, 0.0] if opt.no_offset else (list(opt.offset) if opt.offset else [cfg["vision_offset_m"][0] * 1000.0, cfg["vision_offset_m"][1] * 1000.0])
    view = [float(v) for v in cfg["pickup_view_posj"]]
    rx, ry, rz = cfg["tool_down_zyz"]
    orient = [rx, ry, rz + cfg["pickup_yaw_deg"]]
    rows = []
    rclpy.init()
    n = Picker()
    try:
        if not n.wait(lambda: n.safety is not None and n.tcp is not None, 5.0):
            raise SystemExit("안전 상태·tcp_pose 못 받음 (터미널 2?)")
        view_x = fkin(n, view)
        print(f"보정 시작값 {off0} mm · 거치대 보기 자세 posj {view} · 테이블 z {tz * 1000:.0f} mm · 그리퍼 열림 {ball['open_mm']} mm")
        for i in range(1, opt.count + 1):
            if i > 1 and input(f"\n[{i}/{opt.count}] Enter 진행, q 멈춤: ").strip().lower() == "q":
                break
            off = list(off0)
            row = {"i": i, "raw": None, "off": off, "width": None, "ok": False, "note": ""}
            rows.append(row)
            gripper(n, n.gcli, ball["open_mm"], ball["force_n"])
            movej_abs(n, view, 30.0, view_x)
            seen = n.observe(cfg, tz)
            if seen is None:
                row["note"] = "공 안 보임"
                print("  비전에서 공을 못 찾음 (거치대에 공? segmenter?)")
                continue
            x, y, zc, cnt = seen
            row["raw"] = (round(x, 4), round(y, 4), round(zc, 4))
            print(f"  비전 공 {cnt}개 중 x 최소 ({x:.4f}, {y:.4f}, {zc:.4f}) m · 공 중심 예상 z {tz + 0.0315:.3f} (차 {(zc - tz - 0.0315) * 1000:+.0f} mm)")

            def pts():
                X, Y = (x + off[0] / 1000.0) * 1000.0, (y + off[1] / 1000.0) * 1000.0
                low_z = max(zc - cfg["pickup_below_center_m"], tz + cfg["place_min_tip_m"]) * 1000.0
                return ([X, Y, (zc + cfg["pickup_approach_m"]) * 1000.0, *orient],
                        [X, Y, max((zc + cfg["pickup_slow_above_m"]) * 1000.0, low_z + 10.0), *orient],
                        [X, Y, low_z, *orient])

            above, pre, low = pts()
            move_until(n, above, 60.0, label="공 위로")
            while True:
                s = input(f"  공 위 {cfg['pickup_approach_m'] * 100:.0f} cm (보정 x {off[0]:+.1f} y {off[1]:+.1f} mm). Enter=하강·쥐기, 'x -5' 'y 3' 'x 0 y -5'=옮기기[mm], q=이 회차 중단: ").strip().lower()
                if s == "q":
                    row["note"] = "중단"
                    break
                pairs = re.findall(r"([xy])\s*([-+]?\d+(?:\.\d+)?)", s)
                if s and pairs:
                    for a, v in pairs:
                        off["xy".index(a)] += float(v)
                    above, pre, low = pts()
                    move_until(n, above, 40.0, label="옮김")
                    continue
                if s == "":
                    break
            if row["note"]:
                continue
            move_until(n, pre, 50.0, label="빠른 하강")
            how = move_until(n, low, opt.vel, stall_ok=True, label="느린 하강")
            print(f"  하강 끝: {'접촉으로 멈춤' if how == 'stalled' else '하한까지'} · TCP z {n.posx()[2]:.1f} mm")
            r = gripper(n, n.gcli, 0.0, ball["force_n"])
            row["width"] = round(float(r.actual_width_mm), 1)
            row["ok"] = bool(r.object_detected) and 50.0 <= r.actual_width_mm <= 85.0
            print(f"  → {'공 집음' if row['ok'] else '실패'} (폭 {r.actual_width_mm:.1f} mm, 감지 {r.object_detected})")
            move_until(n, above, 60.0, label="들어 올림")
            if row["ok"] and not opt.keep:
                input("  Enter → 제자리에 다시 놓기: ")
                move_until(n, [low[0], low[1], low[2] + 15.0, *orient], 30.0, stall_ok=True, label="내려놓기")
                gripper(n, n.gcli, ball["open_mm"], ball["force_n"])
                move_until(n, above, 60.0, label="빠짐")
            elif not row["ok"]:
                gripper(n, n.gcli, ball["open_mm"], ball["force_n"])
        print("\n회차 | 비전 공 (x, y, z) m           | 보정 x,y mm   | 폭 mm | 결과")
        for r in rows:
            raw = f"{r['raw']}" if r["raw"] else "-"
            print(f"{r['i']:4d} | {raw:30s} | {r['off'][0]:+5.1f},{r['off'][1]:+5.1f} | {str(r['width'] or '-'):5s} | {'OK' if r['ok'] else ('✗ ' + r['note'])}")
        oks = [r for r in rows if r["ok"]]
        if oks:
            mx = sum(r["off"][0] for r in oks) / len(oks); my = sum(r["off"][1] for r in oks) / len(oks)
            print(f"성공 {len(oks)}/{len(rows)} · 성공한 회차 보정 평균 x {mx:+.1f} y {my:+.1f} mm → skills.yaml mode1.vision_offset_m: [{mx / 1000:.4f}, {my / 1000:.4f}, 0.0]")
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
