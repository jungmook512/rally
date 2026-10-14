#!/usr/bin/env python3
"""모드 3 — 같은 목표를 amovel(직선) 과 amovej(관절) 로 가서 출발·도착·최고 속도·힘을 나란히 잰다 (10/13 정정인: 받기 이동을 amovej 로 바꿀지).

방향·측정은 mode3_reach_test.py 와 같다 (-y 오른쪽, +u / -u 패드 면 안 위·뒤 / 아래·앞. 왼쪽 +y 는 안 함, -u 는 8 cm 까지 — 자기충돌).
amovej 목표 관절값은 지금 관절에서 가장 가까운 해(수치 역기구학, 두산 ikin 의 해 선택 위험 회피)로 구하고, 관절 직선 보간 경로의
TCP 가 작업 공간 안인지·패드 각도가 중간에 얼마나 도는지 계획에서 확인한다.

    source scripts/robot.sh env
    python3 scripts/mode3_movej_test.py                                   # 계획만 (로봇 안 움직임): 관절 변화·예상 시간·경로 검사
    python3 scripts/mode3_movej_test.py --run --dist 10 20 --dirs -y +u --acc 2000 --jvel 120 --jacc 250 --reps 2
결과 CSV: data/measure/mode3_movej_MMDD_HHMMSS.csv (한 줄 = 한 번 이동). 끝에 (방향, 거리)별 amovel/amovej 중앙값 표.
  t_near5_s = 목표 5 cm 안에 처음 들어온 시각 (받기에선 5 cm 더 가기 때문에 이쪽이 실제 '도착').
안전: 매 이동 전 /rally/safety/status 가 NORMAL 아니면 멈춤, 이동 중 STOP 이면 정지 명령 → 정지 확인 후 끝냄. 속도·가속에 speed_override 곱함.
      매 이동 뒤 amovej 로 대기 자세(시작 관절값) 복귀 (느리게). 작업 공간 밖·관절 속도 한계 넘는 목표는 계획에서 빼고 알림.
DSR_ROBOT2 는 rally_robot.dsr_helpers.init_dsr() 뒤에 import (CLAUDE.md).
"""
from __future__ import annotations

import argparse
import csv
import statistics
import sys
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from mode3_reach_test import J3_MAX, JOINT_VMAX, ROOT, Kin, Watch, analyse, directions, in_workspace

DIRS = ("-y", "+u", "-u")
DOWN_MAX_CM = 8.0                                            # -u: 대기 자세에서 9.4 cm 에 두산 자기충돌 (10/11 실측)


def plan_joint(kin, q0, dists_cm, dirs, jvel_deg, jacc_deg):
    """목표마다 관절 해·경로 검사. 반환 [dict(dir, dist_cm, target, q_end(rad), dq_deg, t_est_s, tilt_deg, ok, why)]."""
    p0, R0 = kin.tcp(q0)
    out = []
    for name in dirs:
        d = directions()[name]
        for dc in dists_cm:
            tgt = p0 + d * dc / 100.0
            row = dict(dir=name, dist_cm=dc, target=tgt, q_end=None, dq_deg=None, t_est_s=None, tilt_deg=None, ok=True, why="")
            if name == "-u" and dc > DOWN_MAX_CM:
                row.update(ok=False, why=f"-u 는 {DOWN_MAX_CM:.0f} cm 까지 (자기충돌)")
                out.append(row); continue
            if not in_workspace(tgt):
                row.update(ok=False, why="목표가 안전 작업 공간 밖")
                out.append(row); continue
            q = kin.ik(tgt, R0, q0)                                  # 지금 관절에서 가장 가까운 해
            if q is None:
                row.update(ok=False, why="IK 실패(도달 불가)")
                out.append(row); continue
            dq = np.degrees(q - q0)
            if np.abs(dq).max() > 120.0:
                row.update(ok=False, why=f"관절 변화 {np.abs(dq).max():.0f}° — 다른 자세 해")
                out.append(row); continue
            tilt, bad = 0.0, None
            for s in np.linspace(0.0, 1.0, 41):
                qs = q0 + s * (q - q0)
                ps, Rs = kin.tcp(qs)
                if abs(np.degrees(qs[2])) > J3_MAX:
                    bad = f"경로 중 J3 {np.degrees(qs[2]):.0f}° > {J3_MAX}"; break
                if not in_workspace(ps):
                    bad = f"경로 중 TCP {np.round(ps, 3).tolist()} 작업 공간 밖"; break
                tilt = max(tilt, float(np.degrees(np.linalg.norm(Rotation.from_matrix(Rs @ R0.T).as_rotvec()))))
            if bad:
                row.update(ok=False, why=bad)
                out.append(row); continue
            # 예상 시간: 가장 멀리 도는 관절의 사다리꼴 프로파일 (vel·acc 모두 같은 값으로 주므로 그 관절이 결정)
            dmax = float(np.abs(dq).max())
            t_acc = jvel_deg / jacc_deg
            t_est = 2 * np.sqrt(dmax / jacc_deg) if dmax < jvel_deg * t_acc else dmax / jvel_deg + t_acc
            rate = float((np.abs(dq) / dmax * jvel_deg / JOINT_VMAX).max()) if dmax > 0 else 0.0
            if rate > 0.9:
                row.update(ok=False, why=f"관절 속도 한계의 {rate * 100:.0f}% — --jvel 낮출 것")
            row.update(q_end=q, dq_deg=dq, t_est_s=float(t_est), tilt_deg=tilt)
            out.append(row)
    return p0, R0, out


def near_time(samples, t_cmd, tgt, r_m=0.05):
    """목표 r_m 안에 처음 들어온 시각 (s, 명령 기준). 없으면 None."""
    for t, p in samples:
        if np.linalg.norm(p - tgt) < r_m:
            return t - t_cmd
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="store_true", help="실제로 로봇을 움직임 (없으면 계획만)")
    ap.add_argument("--dist", type=float, nargs="+", default=[10, 20, 30], help="cm")
    ap.add_argument("--dirs", nargs="+", type=str.strip, default=["-y", "+u"], choices=list(DIRS))
    ap.add_argument("--cases", nargs="+", default=["amovel", "amovej"], choices=["amovel", "amovej"])
    ap.add_argument("--vel", type=float, default=450.0, help="amovel mm/s (받기 스킬 catch_vel_mm_s)")
    ap.add_argument("--acc", type=float, default=2000.0, help="amovel mm/s² (받기 스킬 catch_acc_mm_s2)")
    ap.add_argument("--jvel", type=float, nargs="+", default=[120.0], help="amovej deg/s (M0609 한계 J1·2 150, J3 180, J4~6 225)")
    ap.add_argument("--jacc", type=float, nargs="+", default=[250.0], help="amovej deg/s²")
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--return-jvel", type=float, default=30.0, help="대기 자세 복귀 관절 속도 deg/s")
    ap.add_argument("--settle", type=float, default=1.0)
    opt = ap.parse_args([a + " " if a in ("-y", "-u") else a for a in sys.argv[1:] if not a.startswith("--ros-args")])

    rclpy.init()
    kin = Kin()
    w = Watch(kin)
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor()
    ex.add_node(w)
    threading.Thread(target=ex.spin, daemon=True).start()
    t0 = time.monotonic()
    while w.q is None and time.monotonic() - t0 < 5:
        time.sleep(0.05)
    if w.q is None:
        print("/dsr01/joint_states 를 못 받음 — bringup 확인 (source scripts/robot.sh env)")
        return 1
    q0 = w.q.copy()
    plans = {}
    for jv in opt.jvel:
        for ja in opt.jacc:
            p0, R0, steps = plan_joint(kin, q0, opt.dist, opt.dirs, jv, ja)
            plans[(jv, ja)] = steps
    p0, R0 = kin.tcp(q0)
    print(f"대기 자세 관절 {np.round(np.degrees(q0), 1)}  TCP {np.round(p0, 3)} m")
    for (jv, ja), steps in plans.items():
        print(f"\n[amovej vel {jv:.0f} °/s · acc {ja:.0f} °/s²]   (amovel 은 {opt.vel:.0f} mm/s · {opt.acc:.0f} mm/s²)")
        print(f"{'방향':>4} {'cm':>4}  {'관절 변화 (deg)':<40} {'예상 s':>6} {'패드 기울기':>9}  판정")
        for s in steps:
            dq = "-" if s["dq_deg"] is None else str(np.round(s["dq_deg"], 1))
            print(f"{s['dir']:>4} {s['dist_cm']:>4.0f}  {dq:<40} {s['t_est_s'] if s['t_est_s'] is not None else float('nan'):>6.2f} "
                  f"{s['tilt_deg'] if s['tilt_deg'] is not None else float('nan'):>8.1f}°  {'OK' if s['ok'] else '빼기 — ' + s['why']}")
    todo = [((jv, ja), s) for (jv, ja), steps in plans.items() for s in steps if s["ok"]]
    if not opt.run:
        n = len(todo) * opt.reps * (("amovej" in opt.cases) + 0) + sum(1 for s in next(iter(plans.values())) if s["ok"]) * opt.reps * ("amovel" in opt.cases)
        print(f"\n계획만 (로봇 안 움직임). 실행할 이동 약 {n}번")
        ex.shutdown(); rclpy.try_shutdown()
        return 0

    from rally_robot.dsr_helpers import init_dsr, stop_robot
    dsr = init_dsr("rally_mode3_movej")
    from DSR_ROBOT2 import amovej, amovel, check_motion, get_current_posx, get_robot_state  # noqa: E402  init_dsr 뒤
    start6 = list(get_current_posx()[0])
    q0_deg = [float(v) for v in np.degrees(q0)]
    out = ROOT / "data" / "measure" / f"mode3_movej_{datetime.now():%m%d_%H%M%S}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    t1 = time.monotonic()
    while w.safety is None and time.monotonic() - t1 < 5.0:
        time.sleep(0.05)

    def guard(label):
        s = w.safety
        if s is not None and s.level == 2:
            stopped = stop_robot(dsr, w.get_logger(), f"safety STOP {s.reason} ({label})")
            raise SystemExit(f"{label} 중 안전 STOP ({s.reason}) — 정지 {'확인' if stopped else '실패! 펜던트 확인'}")

    def wait_motion(t_cmd, label, limit_s):
        while True:
            time.sleep(0.01)
            guard(label)
            if time.monotonic() - t_cmd > 0.2 and check_motion() == 0:
                return
            if time.monotonic() - t_cmd > limit_s:
                stop_robot(dsr, w.get_logger(), f"{label} 시간 초과")
                raise SystemExit(f"{label}: {limit_s:.0f} s 안에 안 끝남 — 멈춤")

    def scale():
        s = w.wait_normal()
        if s is None:
            raise SystemExit("안전 상태(/rally/safety/status)를 못 받음 — 노드(robot.sh nodes) 확인")
        if s.level != 0:
            raise SystemExit(f"안전 상태가 NORMAL 아님 (level {s.level}, {s.reason}) — 멈춤")
        return float(s.speed_override)

    def go_home():
        kr = scale()
        t_ret = time.monotonic()
        amovej(q0_deg, vel=opt.return_jvel * kr, acc=opt.return_jvel * 2 * kr)
        wait_motion(t_ret, "대기 자세 복귀", 20.0)
        time.sleep(0.3)

    def one(case, s, jv, ja, rep):
        k = scale()
        time.sleep(opt.settle)
        w.clear()
        d = directions()[s["dir"]]
        w.catch_window(True)                                   # 받기 스킬과 같은 힘 한계(130 N)로 — 복귀 전에 끔
        t_cmd = time.monotonic()
        try:
            if case == "amovel":
                tgt6 = [start6[i] + d[i] * s["dist_cm"] * 10.0 for i in range(3)] + list(start6[3:6])
                ret = amovel(tgt6, vel=opt.vel * k, acc=opt.acc * k)
            else:
                ret = amovej([float(v) for v in np.degrees(s["q_end"])], vel=jv * k, acc=ja * k)
            wait_motion(t_cmd, f"{case} 이동", 10.0)
            time.sleep(0.3)
        finally:
            w.catch_window(False)
        smp = w.take(t_cmd)
        r = analyse(smp, t_cmd, p0, s["target"], d)
        r["t_near5_s"] = near_time(smp, t_cmd, s["target"])
        r.update(w.force_stats(t_cmd, time.monotonic()))
        r["robot_state"] = get_robot_state()
        row = dict(case=case, dir=s["dir"], dist_cm=s["dist_cm"], vel=opt.vel * k if case == "amovel" else jv * k,
                   acc=opt.acc * k if case == "amovel" else ja * k, rep=rep + 1, ret=ret, **r)
        rows.append(row)
        print(f"{case:6s} {s['dir']} {s['dist_cm']:.0f} cm #{rep + 1}: 반환 {ret} · 시작 {r.get('t_start_ms') or float('nan'):.0f} ms · "
              f"5 cm 안 {r['t_near5_s'] if r['t_near5_s'] is not None else float('nan'):.3f} s · 도착 {r.get('t_arrive_s') or float('nan'):.3f} s · "
              f"최고 {r.get('peak_mps', 0):.2f} m/s · 지나침 {r.get('overshoot_mm', 0):.1f} mm · 끝 오차 {r.get('final_err_mm', 0):.1f} mm · "
              f"힘 최대 {r.get('force_peak_n', float('nan')):.1f} N (변화 {r.get('force_dpeak_n', float('nan')):.1f})")
        if r["robot_state"] not in (1, 2):
            raise SystemExit(f"로봇 상태 {r['robot_state']} (알람·정지) — 펜던트에서 해제 후 대기 자세로")
        go_home()

    try:
        first_steps = next(iter(plans.values()))
        for s in first_steps:
            geom_ok = s["ok"] or s["why"].startswith("관절 속도")          # amovel 은 관절 속도 판정과 무관 (경로·작업 공간만)
            if not geom_ok:
                continue
            for rep in range(opt.reps):
                if "amovel" in opt.cases:
                    one("amovel", s, None, None, rep)
                if "amovej" in opt.cases:
                    for (jv, ja), steps in plans.items():
                        sj = next(x for x in steps if x["dir"] == s["dir"] and x["dist_cm"] == s["dist_cm"])
                        if sj["ok"]:
                            one("amovej", sj, jv, ja, rep)
    finally:
        if rows:
            with open(out, "w", newline="") as f:
                wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                wr.writeheader(); wr.writerows(rows)
            print(f"\n저장: {out}")
            print(f"\n{'방향':>4} {'cm':>4} {'방식':>7} {'vel/acc':>10} {'시작 ms':>7} {'5cm안 s':>8} {'도착 s':>7} {'최고 m/s':>8} {'힘 N':>6}")
            keys = sorted({(r["dir"], r["dist_cm"], r["case"], r["vel"], r["acc"]) for r in rows}, key=lambda k: (k[0], k[1], k[2], k[3]))
            med = lambda v: statistics.median(v) if v else float("nan")  # noqa: E731
            for k in keys:
                g = [r for r in rows if (r["dir"], r["dist_cm"], r["case"], r["vel"], r["acc"]) == k]
                print(f"{k[0]:>4} {k[1]:>4.0f} {k[2]:>7} {k[3]:>5.0f}/{k[4]:<4.0f} "
                      f"{med([r['t_start_ms'] for r in g if r.get('t_start_ms') is not None]):>7.0f} "
                      f"{med([r['t_near5_s'] for r in g if r.get('t_near5_s') is not None]):>8.3f} "
                      f"{med([r['t_arrive_s'] for r in g if r.get('t_arrive_s') is not None]):>7.3f} "
                      f"{med([r['peak_mps'] for r in g]):>8.2f} {med([r['force_peak_n'] for r in g if 'force_peak_n' in r]):>6.1f}")
        dsr.destroy_node()
        ex.shutdown()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
