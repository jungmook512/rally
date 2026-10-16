#!/usr/bin/env python3
"""모드 3 — 움직이는 중 amovel 다시 보내기 시험 (강화학습 2단계: 예측이 바뀌면 목표 고치기).

대기 자세에서 첫 목표(--first, 기본 −y 10 cm)로 amovel → --delay 초 뒤 둘째 목표(--second, 기본 −y 15 cm)로 amovel 한 번 더.
경우 (--cases):
  single    : 둘째 목표로 한 번만 (비교 기준)
  duplicate : 두 번째를 ra=DR_MV_RA_DUPLICATE(두산 기본)로 — 앞 동작 끝난 뒤 이어 가는지
  override  : 두 번째를 ra=DR_MV_RA_OVERRIDE 로 — 앞 동작을 바로 바꿔 가는지
  movel     : 두 번째를 movel(블로킹, 두산 기본 ra) 로 — 움직이는 중 movel 이 받아지는지 (10/12 추가)
기록 (한 줄 = 한 번): 두 amovel 반환값, 출발, 둘째 목표 도착 시각, 사이에 멈췄는지(속도가 --stop-mps 아래로 떨어졌다 다시 감),
  첫 목표 근처에 머문 시간, 지나침, 최고 속도, 힘 최대. 측정은 mode3_reach_test.py 와 같은 joint_states 순기구학.

    source scripts/robot.sh env
    python3 scripts/mode3_amovel_resend_test.py                      # 계획만 (로봇 안 움직임)
    python3 scripts/mode3_amovel_resend_test.py --run --reps 3
결과 CSV: data/measure/mode3_resend_MMDD_HHMM.csv
안전: 매 이동 전 /rally/safety/status 가 NORMAL 아니면 멈춤 (속도·가속에 speed_override 곱함). 매 시험 뒤 대기 자세로 천천히 복귀.
      왼쪽(+y)은 안 함. 계획(관절 속도·작업 공간)에서 걸리면 실행 안 함.
DSR_ROBOT2 는 rally_robot.dsr_helpers.init_dsr() 뒤에 import (CLAUDE.md).
"""
from __future__ import annotations

import argparse
import csv
import sys
import threading
import time
from datetime import datetime

import numpy as np
import rclpy

from mode3_reach_test import ROOT, Kin, Watch, analyse, directions, plan

DIRS = ("-y", "+u", "-u")


def motion_profile(samples, t_cmd, p0, d):
    """(t, p) → 시각(s, 명령 기준), 이동 방향 거리(m), 속도(m/s, 30 ms 넘게 떨어진 표본 차분 — reach 시험과 같은 이유)."""
    t = np.array([s[0] for s in samples]) - t_cmd
    along = (np.array([s[1] for s in samples]) - p0) @ d
    tv, v = [], []
    j = 0
    for i in range(len(t)):
        while j < len(t) and t[j] - t[i] < 0.03:
            j += 1
        if j >= len(t):
            break
        tv.append((t[i] + t[j]) / 2)
        v.append((along[j] - along[i]) / (t[j] - t[i]))
    return t, along, np.array(tv), np.array(v)


def resend_stats(samples, t_cmd, p0, d, first_m, second_m, stop_mps):
    """출발 뒤 둘째 목표에 닿기 전까지 속도가 stop_mps 아래로 떨어진 적이 있으면 '멈췄다 다시 감'."""
    if len(samples) < 5:
        return {}
    t, along, tv, v = motion_profile(samples, t_cmd, p0, d)
    out = {}
    moving = np.where(v > stop_mps)[0]
    reach2 = np.where(along > second_m - 0.002)[0]
    if len(moving) and len(reach2):
        t_go, t_r2 = tv[moving[0]], t[reach2[0]]
        mid = (tv > t_go) & (tv < t_r2 - 0.05)
        dips = np.where(mid & (v < stop_mps))[0]
        out["stopped_between"] = bool(len(dips))
        out["min_mps_between"] = float(v[mid].min()) if mid.any() else float("nan")
        out["t_reach_second_s"] = float(t_r2)
    near1 = np.abs(along - first_m) < 0.003
    out["dwell_first_s"] = float(np.sum(np.diff(t)[near1[:-1]])) if len(t) > 1 else 0.0   # 첫 목표 ±3 mm 안에 머문 시간
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="store_true", help="실제로 로봇을 움직임 (없으면 계획만)")
    ap.add_argument("--cases", nargs="+", default=["single", "duplicate", "override"], choices=["single", "duplicate", "override", "movel", "jsingle", "jdup"])
    ap.add_argument("--jvel", type=float, default=120.0, help="jsingle·jdup: amovej 관절 속도 °/s")
    ap.add_argument("--jacc", type=float, default=400.0, help="jsingle·jdup: amovej 관절 가속 °/s²")
    ap.add_argument("--dir", type=str.strip, default="-y", choices=list(DIRS))
    ap.add_argument("--first", type=float, default=10.0, help="첫 목표 cm")
    ap.add_argument("--second", type=float, default=15.0, help="둘째 목표 cm")
    ap.add_argument("--max-cm", type=float, default=25.0, help="둘째 목표 상한 cm. 10/16: 15 → 25 — 어깨 반대 대기 자세는 25 cm (예전 자세 right_max 15 시절 값). 계획 단계가 관절 속도·작업 공간을 따로 본다")
    ap.add_argument("--delay", type=float, nargs="+", default=[0.15], help="첫 amovel → 둘째 amovel s")
    ap.add_argument("--vel", type=float, default=450.0, help="mm/s (안전 speed_override 를 곱함)")
    ap.add_argument("--acc", type=float, default=1000.0, help="mm/s²")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--stop-mps", type=float, default=0.02, help="이 속도 아래면 '멈춤'으로 봄")
    ap.add_argument("--return-vel", type=float, default=100.0)
    ap.add_argument("--settle", type=float, default=1.0)
    opt = ap.parse_args([a + " " if a in ("-y", "-u") else a for a in sys.argv[1:]])   # "-y" 를 값으로 (reach 시험과 같음)
    if opt.second > opt.max_cm or opt.first > opt.second:
        print(f"--second 는 {opt.max_cm:.0f} cm 까지 (--max-cm), --first ≤ --second")
        return 1

    rclpy.init()
    kin = Kin()
    w = Watch(kin)
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(w)

    def spin():
        try:
            ex.spin()
        except Exception:  # noqa: BLE001  끝낼 때 rclpy 종료와 겹치는 예외는 무시
            pass
    threading.Thread(target=spin, daemon=True).start()
    t0 = time.monotonic()
    while w.q is None and time.monotonic() - t0 < 5:
        time.sleep(0.05)
    if w.q is None:
        print("/dsr01/joint_states 를 못 받음 — bringup 확인 (source scripts/robot.sh env)")
        return 1
    q0 = w.q.copy()
    p0, _, steps = plan(kin, q0, [opt.first, opt.second], [opt.dir], opt.vel)
    print(f"대기 자세 관절 {np.round(np.degrees(q0), 1)}  TCP {np.round(p0, 3)} m · {opt.dir} {opt.first:.0f}→{opt.second:.0f} cm · "
          f"vel {opt.vel:.0f} acc {opt.acc:.0f}")
    for s in steps:
        print(f"  {s['dir']} {s['dist_cm']:.0f} cm  관절속도 {s['joint_rate_pct']:.0f}%  {'OK' if s['ok'] else '안 됨 — ' + s['why']}")
    n = opt.reps * sum(1 if c == "single" else len(opt.delay) for c in opt.cases)
    if not all(s["ok"] for s in steps):
        print("계획에서 걸림 — 실행 안 함")
        ex.shutdown(); rclpy.try_shutdown()
        return 1
    if not opt.run:
        print(f"\n계획만 (로봇 안 움직임). 시험 {n}번")
        ex.shutdown(); rclpy.try_shutdown()
        return 0

    from rally_robot.dsr_helpers import init_dsr, stop_robot
    dsr = init_dsr("rally_mode3_resend")
    from DSR_ROBOT2 import DR_MV_RA_DUPLICATE, DR_MV_RA_OVERRIDE, amovej, amovel, check_motion, get_current_posj, get_current_posx, get_robot_state, movel  # noqa: E402
    from rally_robot.dsr_helpers import ik_nearest                     # 받기 스킬(catch_posj)과 같은 해 선택
    start6 = list(get_current_posx()[0])
    d = directions()[opt.dir]

    def target(cm):
        t6 = list(start6)
        for i in range(3):
            t6[i] = start6[i] + d[i] * cm * 10.0
        return t6

    def guard(label):
        """이동 중 안전 STOP 이면 정지 명령 후 끝냄 (mode3_movej_test 와 같음, #185 리뷰)."""
        s = w.safety
        if s is not None and s.level == 2:
            stopped = stop_robot(dsr, w.get_logger(), f"safety STOP {s.reason} ({label})")
            raise SystemExit(f"{label} 중 안전 STOP ({s.reason}) — 정지 {'확인' if stopped else '실패! 펜던트 확인'}")

    def wait_motion(t_cmd, label, limit_s, min_s=0.2):
        while True:
            time.sleep(0.01)
            guard(label)
            if time.monotonic() - t_cmd > min_s and check_motion() == 0:
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

    out = ROOT / "data" / "measure" / f"mode3_resend_{datetime.now():%m%d_%H%M%S}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    try:
        for case in opt.cases:
            for delay in ([0.0] if case in ("single", "jsingle") else opt.delay):
                for rep in range(opt.reps):
                    k = scale()
                    time.sleep(opt.settle)
                    w.clear()
                    w.catch_window(True)                       # 받기 창 힘 한계 130 N (10/13, mode3_reach_test.Watch)
                    t_cmd = time.monotonic()
                    ret2, t2 = None, float("nan")
                    try:                                       # 예외·STOP 에도 창을 끈다 (#185 리뷰)
                        if case in ("jsingle", "jdup"):                 # 10/13: amovej 로 출발 → (jdup) delay 뒤 amovej 재전송 — 겹침 확인
                            q1 = ik_nearest(target(opt.first if case == "jdup" else opt.second), get_current_posj())
                            ret1 = amovej(q1, vel=opt.jvel * k, acc=opt.jacc * k)
                            if case == "jdup":
                                while time.monotonic() - t_cmd < delay:
                                    time.sleep(0.001)
                                if w.safety is None or w.safety.level != 0:
                                    raise SystemExit("둘째 amovej 전 안전 상태가 NORMAL 아님 — 멈춤")
                                t2 = time.monotonic() - t_cmd
                                q2 = ik_nearest(target(opt.second), get_current_posj())
                                ret2 = amovej(q2, vel=opt.jvel * k, acc=opt.jacc * k)
                        elif case == "single":
                            ret1 = amovel(target(opt.second), vel=opt.vel * k, acc=opt.acc * k)
                        else:
                            ret1 = amovel(target(opt.first), vel=opt.vel * k, acc=opt.acc * k)
                            while time.monotonic() - t_cmd < delay:
                                time.sleep(0.001)
                            if w.safety is None or w.safety.level != 0:     # 움직이는 중에는 기다리지 않음
                                raise SystemExit("둘째 amovel 전 안전 상태가 NORMAL 아님 — 멈춤")
                            t2 = time.monotonic() - t_cmd
                            if case == "movel":
                                ret2 = movel(target(opt.second), vel=opt.vel * k, acc=opt.acc * k)   # 도착(또는 거부)까지 블로킹
                            else:
                                ra = DR_MV_RA_OVERRIDE if case == "override" else DR_MV_RA_DUPLICATE
                                ret2 = amovel(target(opt.second), vel=opt.vel * k, acc=opt.acc * k, ra=ra)
                        wait_motion(t_cmd, f"{case} 이동", 10.0, 0.2 + (0 if case in ("single", "jsingle") else delay))
                        time.sleep(0.3)
                    finally:
                        w.catch_window(False)
                    smp = w.take(t_cmd)
                    tgt = p0 + d * opt.second / 100.0
                    row = dict(case=case, dir=opt.dir, first_cm=opt.first, second_cm=opt.second, delay_s=delay,
                               t_second_cmd_s=t2, vel=opt.vel * k, acc=opt.acc * k, rep=rep + 1, ret1=ret1, ret2=ret2)
                    row.update(analyse(smp, t_cmd, p0, tgt, d))
                    row.update(resend_stats(smp, t_cmd, p0, d, opt.first / 100.0, opt.second / 100.0, opt.stop_mps))
                    row.update(w.force_stats(t_cmd, time.monotonic()))
                    row["robot_state"] = get_robot_state()     # 1 STANDBY · 2 MOVING 아니면 두산 알람·정지
                    rows.append(row)
                    print(f"{case} delay {delay:.2f} #{rep + 1}: 반환 {ret1}/{ret2} · 시작 {row.get('t_start_ms') or float('nan'):.0f} ms · "
                          f"{opt.second:.0f} cm 도착 {row.get('t_arrive_s') or float('nan'):.3f} s · "
                          f"멈춤 {'예' if row.get('stopped_between') else '아니오'} (최저 {row.get('min_mps_between', float('nan')):.2f} m/s) · "
                          f"첫 목표 머묾 {row.get('dwell_first_s', 0):.2f} s · 지나침 {row.get('overshoot_mm', 0):.1f} mm · "
                          f"끝 오차 {row.get('final_err_mm', float('nan')):.1f} mm · 힘 최대 {row.get('force_peak_n', float('nan')):.1f} N")
                    if row["robot_state"] not in (1, 2):
                        raise SystemExit(f"로봇 상태 {row['robot_state']} (알람·정지) — 펜던트에서 해제 후 대기 자세로")
                    k = scale()                                # 복귀도 amovel + 폴링 — 막는 movel·mwait 동안엔 STOP 을 못 본다 (mode3_reach_test 와 같음)
                    t_ret = time.monotonic()
                    amovel(start6, vel=opt.return_vel * k, acc=opt.return_vel * 2 * k)
                    wait_motion(t_ret, "대기 자세 복귀", 15.0)
    finally:
        if rows:
            keys = list(dict.fromkeys(k for r in rows for k in r))
            with open(out, "w", newline="") as fh:
                wr = csv.DictWriter(fh, fieldnames=keys)
                wr.writeheader()
                wr.writerows(rows)
            print(f"\n저장: {out}")
        dsr.destroy_node()
        ex.shutdown()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
