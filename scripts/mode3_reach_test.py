#!/usr/bin/env python3
"""모드 3 로봇 응답 시험 (MODE3_BRIEF 세션 D) — 대기 자세에서 패드 면 안으로 amovel, 명령 → 움직임 시작·도착·지나침을 잰다.

방향 (base_link, 공구 자세 고정, 위치만):
  +y / -y : 좌우            +u / -u : 패드 면(45°, 투구자 쪽을 봄) 안 '비스듬히 위로·뒤로' / '아래로·앞으로'
측정은 /dsr01/joint_states(100 Hz) + 순기구학 TCP (tcp_speed.py 와 같은 Chain·TCP 오프셋), 수신 시각 기준.
  t_start  = 명령 → TCP 가 출발점에서 2 mm 벗어남      t_arrive = 명령 → 목표 2 mm 안에 들어와 머묾
  overshoot = 이동 방향으로 목표를 지나친 최대 거리      peak = 최고 TCP 속도

    source scripts/robot.sh env
    python3 scripts/mode3_reach_test.py                     # 계획만 (로봇 안 움직임): 목표점·관절 한계·필요 관절 속도·작업 공간
    python3 scripts/mode3_reach_test.py --run --dist 5 10 --acc 1000 --vel 500 --reps 3 --dirs +y -y
결과 CSV: data/measure/mode3_reach_MMDD_HHMM.csv (한 줄 = 한 번 이동).
안전: 대기 자세는 지금 로봇 자세. 매 이동 전 /rally/safety/status 가 NORMAL 이 아니면 멈춤. 매 이동 뒤 대기 자세로 천천히(--return-vel) 돌아감.
      safety.yaml 작업 공간(x 0.20~0.57, y ±0.45, z 0~0.60) 밖 목표는 계획에서 빼고 알림. 속도는 --vel × 안전 speed_override.
DSR_ROBOT2 는 rally_robot.dsr_helpers.init_dsr() 뒤에 import (CLAUDE.md).
"""
import argparse
import csv
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ejt_wrench import JOINTS, Chain  # noqa: E402
from print_pose import TCP_OFFSET_MM  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
JOINT_VMAX = np.array([150, 150, 180, 225, 225, 225.0])   # M0609 관절 속도 한계 deg/s
J3_MAX = 148.0
WORKSPACE = dict(x=(0.20, 0.57), y=(-0.45, 0.45), z=(0.0, 0.60))   # safety.yaml (모드 1 기준)
PAD_TILT_DEG = 45.0


def directions():
    n = np.array([np.cos(np.radians(PAD_TILT_DEG)), 0.0, np.sin(np.radians(PAD_TILT_DEG))])   # 패드 면 법선 (투구자 쪽 위)
    y = np.array([0.0, 1.0, 0.0])
    u = np.cross(n, y)                                       # 면 안 '비스듬히 위·뒤'
    return {"+y": y, "-y": -y, "+u": u, "-u": -u}


class Kin:
    def __init__(self):
        self.c = Chain()
        self.off = TCP_OFFSET_MM / 1000.0

    def tcp(self, q_rad):
        _, T = self.c.fk_frames(q_rad)
        return T[:3, 3] + T[:3, :3] @ self.off, T[:3, :3]

    def ik(self, p, R, seed):
        def err(q):
            pp, RR = self.tcp(q)
            return np.r_[pp - p, 0.3 * Rotation.from_matrix(R @ RR.T).as_rotvec()]
        r = least_squares(err, seed, xtol=1e-12, ftol=1e-12)
        e = err(r.x)
        return r.x if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 3e-4 else None

    def joint_rate_for(self, q, v):
        """TCP 를 v (m/s, 자세 고정) 로 움직일 때 관절 속도 deg/s (수치 자코비안)."""
        eps = 1e-5
        p0, R0 = self.tcp(q)
        J = np.zeros((6, 6))
        for i in range(6):
            dq = q.copy(); dq[i] += eps
            p1, R1 = self.tcp(dq)
            J[:3, i] = (p1 - p0) / eps
            J[3:, i] = Rotation.from_matrix(R1 @ R0.T).as_rotvec() / eps
        return np.degrees(np.linalg.solve(J, np.r_[v, 0, 0, 0]))


def in_workspace(p):
    return all(WORKSPACE[a][0] <= v <= WORKSPACE[a][1] for a, v in zip("xyz", p))


def plan(kin, q0, dists_cm, dirs, vel_mm_s):
    p0, R0 = kin.tcp(q0)
    out = []
    for name in dirs:
        d = directions()[name]
        for dc in dists_cm:
            tgt = p0 + d * dc / 100.0
            q, worst, ok, why = q0.copy(), 0.0, True, ""
            for s in np.linspace(0.0, 1.0, max(2, int(dc / 0.5)) + 1)[1:]:   # 직선 경로 0.5 cm 마다 (특이점을 건너뛰지 않게)
                q = kin.ik(p0 + d * dc / 100.0 * s, R0, q)
                if q is None:
                    ok, why = False, "IK 실패(도달 불가)"; break
                if abs(np.degrees(q[2])) > J3_MAX:
                    ok, why = False, f"J3 {np.degrees(q[2]):.0f}° > {J3_MAX}"; break
                rate = np.abs(kin.joint_rate_for(q, d * vel_mm_s / 1000.0)) / JOINT_VMAX
                worst = max(worst, rate.max())
            if ok and not in_workspace(tgt):
                ok, why = False, "안전 작업 공간 밖"
            if ok and worst > 3.0:
                ok, why = False, f"경로가 특이점을 지남 (관절 속도 한계의 {worst * 100:.0f}%) — 이 방향·거리는 이 자세로 불가"
            elif ok and worst > 0.8:
                ok, why = False, f"관절 속도 한계의 {worst * 100:.0f}% (> 80%) — --vel 낮출 것"
            out.append(dict(dir=name, dist_cm=dc, target=tgt, q_end=None if q is None else np.degrees(q),
                            joint_rate_pct=worst * 100, ok=ok, why=why))
    return p0, R0, out


class Watch(Node):
    def __init__(self, kin):
        super().__init__("rally_mode3_reach_watch")
        self.kin, self.lock = kin, threading.Lock()
        self.samples = []          # (t, p)
        self.q = None
        self.safety = None
        self.create_subscription(JointState, "/dsr01/joint_states", self.on_js, qos_profile_sensor_data)
        from rally_interfaces.msg import SafetyStatus
        self.create_subscription(SafetyStatus, "/rally/safety/status", lambda m: setattr(self, "safety", m),
                                 QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10))

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if not all(k in d for k in JOINTS):
            return
        q = np.array([d[k] for k in JOINTS])
        p, _ = self.kin.tcp(q)
        with self.lock:
            self.q = q
            self.samples.append((time.monotonic(), p))

    def take(self, since):
        with self.lock:
            return [(t, p) for t, p in self.samples if t >= since]

    def clear(self):
        with self.lock:
            self.samples = []


def analyse(samples, t_cmd, p0, tgt, d):
    if not samples:
        return {}
    t = np.array([s[0] for s in samples]) - t_cmd
    P = np.array([s[1] for s in samples])
    moved = np.linalg.norm(P - p0, axis=1)
    to_tgt = np.linalg.norm(P - tgt, axis=1)
    along = (P - p0) @ d
    dist = np.linalg.norm(tgt - p0)
    i0 = np.argmax(moved > 0.002) if (moved > 0.002).any() else None
    arrive = None
    inside = to_tgt < 0.002
    for i in range(len(inside)):
        if inside[i:].all():
            arrive = t[i]; break
    # 속도: 30 ms 넘게 떨어진 두 표본으로 (joint_states 가 받은 시각 기준으로 몰려 들어와 이웃 차분은 튄다 — 10/10 실측 4 m/s 오계산)
    v = []
    j = 0
    for i in range(len(t)):
        while j < len(t) and t[j] - t[i] < 0.03:
            j += 1
        if j >= len(t):
            break
        v.append(np.linalg.norm(P[j] - P[i]) / (t[j] - t[i]))
    v = np.array(v)
    return dict(t_start_ms=None if i0 is None else t[i0] * 1000, t_arrive_s=arrive,
                overshoot_mm=max(0.0, (along.max() - dist) * 1000), peak_mps=float(v.max()) if len(v) else 0.0,
                final_err_mm=float(to_tgt[-1] * 1000))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="store_true", help="실제로 로봇을 움직임 (없으면 계획만)")
    ap.add_argument("--dist", type=float, nargs="+", default=[5, 10, 20, 30], help="cm")
    ap.add_argument("--dirs", nargs="+", default=["+y", "-y", "+u", "-u"], choices=["+y", "-y", "+u", "-u"])
    ap.add_argument("--acc", type=float, nargs="+", default=[1000.0], help="mm/s²")
    ap.add_argument("--vel", type=float, default=500.0, help="mm/s (안전 speed_override 를 곱함)")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--return-vel", type=float, default=100.0, help="대기 자세로 돌아가는 속도 mm/s")
    ap.add_argument("--settle", type=float, default=1.0, help="이동 사이 정지 시간 s")
    opt = ap.parse_args()

    rclpy.init()
    kin = Kin()
    w = Watch(kin)
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor()
    ex.add_node(w)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    t0 = time.monotonic()
    while w.q is None and time.monotonic() - t0 < 5:
        time.sleep(0.05)
    if w.q is None:
        print("/dsr01/joint_states 를 못 받음 — bringup 확인 (source scripts/robot.sh env)")
        return 1
    q0 = w.q.copy()
    p0, R0, steps = plan(kin, q0, opt.dist, opt.dirs, opt.vel)
    print(f"대기 자세 관절 {np.round(np.degrees(q0), 1)}  TCP {np.round(p0, 3)} m  · vel {opt.vel:.0f} mm/s")
    print(f"{'방향':>4} {'cm':>4}  {'목표 TCP (m)':<24} {'관절속도':>8}  판정")
    for s in steps:
        print(f"{s['dir']:>4} {s['dist_cm']:>4.0f}  {str(np.round(s['target'], 3)):<24} {s['joint_rate_pct']:>7.0f}%  "
              f"{'OK' if s['ok'] else '빼기 — ' + s['why']}")
    todo = [s for s in steps if s["ok"]]
    if not opt.run:
        print(f"\n계획만 (로봇 안 움직임). 실행할 이동 {len(todo)}개 × 가속 {len(opt.acc)} × {opt.reps}회 = {len(todo) * len(opt.acc) * opt.reps}번")
        ex.shutdown()
        rclpy.try_shutdown()
        return 0

    from rally_robot.dsr_helpers import init_dsr
    dsr = init_dsr("rally_mode3_reach")
    from DSR_ROBOT2 import amovel, check_motion, get_current_posx, movel, mwait  # noqa: E402  init_dsr 뒤
    start6 = list(get_current_posx()[0])
    out = ROOT / "data" / "measure" / f"mode3_reach_{datetime.now():%m%d_%H%M}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = []

    def scale():
        s = w.safety
        if s is None:
            raise SystemExit("안전 상태(/rally/safety/status)를 못 받음 — 노드(robot.sh nodes) 확인")
        if s.level != 0:
            raise SystemExit(f"안전 상태가 NORMAL 아님 (level {s.level}, {s.reason}) — 멈춤")
        return float(s.speed_override)

    try:
        for acc in opt.acc:
            for s in todo:
                d = directions()[s["dir"]]
                tgt6 = list(start6)
                for i in range(3):
                    tgt6[i] = start6[i] + d[i] * s["dist_cm"] * 10.0      # mm
                for rep in range(opt.reps):
                    k = scale()
                    time.sleep(opt.settle)
                    w.clear()
                    t_cmd = time.monotonic()
                    amovel(tgt6, vel=opt.vel * k, acc=acc * k)
                    while True:
                        time.sleep(0.01)
                        if time.monotonic() - t_cmd > 0.2 and check_motion() == 0:
                            break
                        if time.monotonic() - t_cmd > 10:
                            raise SystemExit("10 s 안에 안 끝남 — 멈춤")
                    time.sleep(0.3)
                    r = analyse(w.take(t_cmd), t_cmd, p0, s["target"], d)
                    row = dict(dir=s["dir"], dist_cm=s["dist_cm"], acc=acc, vel=opt.vel * k, rep=rep + 1, **r)
                    rows.append(row)
                    print(f"{s['dir']} {s['dist_cm']:.0f} cm acc {acc:.0f} #{rep + 1}: 시작 {r.get('t_start_ms') or float('nan'):.0f} ms · "
                          f"도착 {r.get('t_arrive_s') or float('nan'):.3f} s · 지나침 {r.get('overshoot_mm', 0):.1f} mm · "
                          f"최고 {r.get('peak_mps', 0):.2f} m/s · 끝 오차 {r.get('final_err_mm', 0):.1f} mm")
                    scale()
                    movel(start6, vel=opt.return_vel * scale(), acc=opt.return_vel * 2 * scale())
                    mwait()
    finally:
        if rows:
            with open(out, "w", newline="") as f:
                wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                wr.writeheader()
                wr.writerows(rows)
            print(f"\n저장: {out}")
        dsr.destroy_node()
        ex.shutdown()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
