#!/usr/bin/env python3
"""모드 3 — speedl 로 P 제어 추종 시험 (10/13 정정인: 예측점을 향해 v = Kp·(목표 − 현재) 를 계속 보내는 방식이 amovel/amovej 보다 나은지).

루프(rate Hz): 현재 TCP(joint_states 순기구학) → 오차 e = 목표 − 현재 → v = Kp·e 를 |v| ≤ vmax 로 잘라 speedl(vel[xyz, 0,0,0], acc, time=watchdog).
  watchdog(time 인자) 가 지나면 로봇이 스스로 멈추므로 루프가 죽어도 그 시간 안에 선다. 오차 < 2 mm 가 0.2 s 이어지면 끝.
시험
  step   : 목표를 대기 자세에서 방향 d 로 amp 만큼 한 번에 (amovel/amovej 계단 응답과 비교: 시작 지연·5 cm 안·도착·최고 속도·힘)
  moving : 목표 amp1 로 가다가 --switch s 뒤 amp2 로 바뀜 (비행 중 갱신 흉내 — amovel 재전송 0.3 s 시험과 비교)
방향·측정·안전은 mode3_servol_test.py 와 같다 (-y 오른쪽, +u / -u 패드 면 안 위·뒤 / 아래·앞, -u 8 cm 까지, 왼쪽 안 함).
  speedl 은 두산 속도 슬라이더·안전 구역 감속을 따르지 않는다 → 매 틱 /rally/safety/status 를 보고 NORMAL 이 아니면 즉시 정지.
  목표가 작업 공간 밖이면 계획에서 뺀다. TCP 가 시작점에서 amp + 5 cm 넘게 벗어나면(폭주) 즉시 정지.
  시험 이동 동안 받기 창 하트비트(contact_expected + catch_window, 힘 한계 130 N — 받기 스킬과 같음), 복귀 전에 끔.

    source scripts/robot.sh env
    python3 scripts/mode3_speedl_test.py                                        # 계획만
    python3 scripts/mode3_speedl_test.py --run --tests step --dirs -y +u --amp 10 20 --kp 6 --vmax 450 --acc 2000
    python3 scripts/mode3_speedl_test.py --run --tests moving --dirs +u --amp 10 --amp2 15 --switch 0.3
결과 CSV: data/measure/mode3_speedl_MMDD_HHMMSS.csv
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

from mode3_reach_test import ROOT, Kin, Watch, analyse, directions, in_workspace
from mode3_servol_test import check, send_stats

DIRS = ("-y", "+u", "-u")
AMP_MAX_DOWN_CM = 8.0
ROBOT_OK = (1, 2)


def near_time(samples, t0, tgt, r_m):
    for t, p in samples:
        if np.linalg.norm(p - tgt) < r_m:
            return t - t0
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="store_true", help="실제로 로봇을 움직임 (없으면 계획만)")
    ap.add_argument("--tests", nargs="+", default=["step"], choices=["step", "moving"])
    ap.add_argument("--dirs", nargs="+", type=str.strip, default=["-y", "+u"], choices=list(DIRS))
    ap.add_argument("--amp", type=float, nargs="+", default=[10.0, 20.0], help="cm (step 목표 · moving 첫 목표)")
    ap.add_argument("--amp2", type=float, default=15.0, help="moving 둘째 목표 cm")
    ap.add_argument("--switch", type=float, default=0.3, help="moving: 목표 바꾸는 시각 s")
    ap.add_argument("--kp", type=float, nargs="+", default=[6.0], help="P 이득 1/s (v = kp·e)")
    ap.add_argument("--vmax", type=float, default=450.0, help="속도 상한 mm/s (받기 스킬 catch_vel_mm_s)")
    ap.add_argument("--acc", type=float, default=2000.0, help="speedl 가속 mm/s²")
    ap.add_argument("--rate", type=float, default=30.0, help="명령 주기 Hz")
    ap.add_argument("--watchdog", type=float, default=0.2, help="speedl time 인자 s — 이 시간 안에 다음 명령이 없으면 로봇이 섬")
    ap.add_argument("--hold", type=float, default=1.5, help="최대 시험 시간 s")
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--return-vel", type=float, default=100.0)
    ap.add_argument("--settle", type=float, default=1.0)
    opt = ap.parse_args([a + " " if a in ("-y", "-u") else a for a in sys.argv[1:] if not a.startswith("--ros-args")])
    amps = sorted(set(opt.amp))
    if "-u" in opt.dirs and max(amps + ([opt.amp2] if "moving" in opt.tests else [])) > AMP_MAX_DOWN_CM:
        print(f"-u 는 {AMP_MAX_DOWN_CM} cm 까지 (자기충돌 약 9.4 cm)")
        return 1

    rclpy.init()
    kin = Kin()
    w = Watch(kin)
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()                         # 멀티 스레드 실행기는 GIL 을 오래 잡아 명령 루프가 30 Hz 를 못 냄 (servol 시험)
    ex.add_node(w)

    def spin():
        try:
            ex.spin()
        except Exception:  # noqa: BLE001
            pass
    threading.Thread(target=spin, daemon=True).start()
    t0 = time.monotonic()
    while w.q is None and time.monotonic() - t0 < 5:
        time.sleep(0.05)
    if w.q is None:
        print("/dsr01/joint_states 를 못 받음 — bringup 확인 (source scripts/robot.sh env)")
        return 1
    q0 = w.q.copy()
    p0, _ = kin.tcp(q0)

    cases = []
    print(f"대기 자세 관절 {np.round(np.degrees(q0), 1)}  TCP {np.round(p0, 3)} m · vmax {opt.vmax:.0f} mm/s · acc {opt.acc:.0f} · {opt.rate:.0f} Hz · watchdog {opt.watchdog:.2f} s")
    print(f"{'시험':>6} {'방향':>4} {'cm':>6} {'kp':>4} {'관절속도':>8}  판정")
    for kind in opt.tests:
        for name in opt.dirs:
            for amp in amps:
                far = max(amp, opt.amp2) if kind == "moving" else amp
                ok, why, worst = check(kin, q0, name, far, opt.vmax / 1000.0)
                for kp in opt.kp:
                    lab = f"{amp:.0f}" if kind == "step" else f"{amp:.0f}→{opt.amp2:.0f}"
                    print(f"{kind:>6} {name:>4} {lab:>6} {kp:>4.1f} {worst * 100:>7.0f}%  {'OK' if ok else '빼기 — ' + why}")
                    if ok:
                        cases.append((kind, name, amp, kp))
    if not opt.run:
        print(f"\n계획만 (로봇 안 움직임). 실행할 시험 {len(cases)}개 × {opt.reps}회 = {len(cases) * opt.reps}번")
        ex.shutdown(); rclpy.try_shutdown()
        return 0

    from rally_robot.dsr_helpers import init_dsr, move_stop
    dsr = init_dsr("rally_mode3_speedl")
    from DSR_ROBOT2 import check_motion, get_current_posx, get_robot_state, movel, mwait, speedl  # noqa: E402  init_dsr 뒤
    start6 = list(get_current_posx()[0])
    out = ROOT / "data" / "measure" / f"mode3_speedl_{datetime.now():%m%d_%H%M%S}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = []

    def zero(n=3):
        for _ in range(n):
            speedl([0.0] * 6, [opt.acc, 60.0], 0.05)
            time.sleep(0.02)

    def safe_or_stop():
        s = w.safety
        if s is None or s.level != 0:
            zero()
            move_stop(dsr, stop_mode=2)
            why = "안전 상태 없음" if s is None else f"level {s.level}, {s.reason}"
            raise SystemExit(f"안전 상태가 NORMAL 아님 ({why}) — 정지하고 끝냄")

    def wait_still(timeout_s=3.0):
        t = time.monotonic()
        while time.monotonic() - t < timeout_s:
            if check_motion() == 0:
                return
            time.sleep(0.02)

    try:
        for kind, name, amp, kp in cases:
            d = directions()[name]
            tgt1 = p0 + d * amp / 100.0
            tgt2 = p0 + d * opt.amp2 / 100.0 if kind == "moving" else tgt1
            fence = max(amp, opt.amp2 if kind == "moving" else 0) / 100.0 + 0.05
            for rep in range(opt.reps):
                w.wait_normal()
                safe_or_stop()
                time.sleep(opt.settle)
                w.clear()
                w.catch_window(True)
                sent, err, settled, vpeak_cmd = [], "", None, 0.0
                t_start = time.monotonic()
                try:
                    k = 0
                    while True:
                        t_tick = t_start + k / opt.rate
                        while time.monotonic() < t_tick:
                            time.sleep(0.0005)
                        k += 1
                        now = time.monotonic() - t_start
                        if now > opt.hold:
                            break
                        safe_or_stop()
                        with w.lock:
                            p_now = w.samples[-1][1] if w.samples else p0
                        if np.linalg.norm(p_now - p0) > fence:
                            err = f"폭주 감지 (시작점에서 {np.linalg.norm(p_now - p0) * 100:.0f} cm) — 정지"
                            break
                        tgt = tgt2 if (kind == "moving" and now >= opt.switch) else tgt1
                        e = tgt - p_now                                     # m
                        v = kp * e * 1000.0                                 # mm/s
                        n = float(np.linalg.norm(v))
                        if n > opt.vmax:
                            v = v * (opt.vmax / n)
                        if np.linalg.norm(e) < 0.002:
                            v = np.zeros(3)
                            settled = settled if settled is not None else now
                            if now - settled > 0.2 and (kind == "step" or now > opt.switch + 0.3):
                                break
                        else:
                            settled = None
                        vpeak_cmd = max(vpeak_cmd, float(np.linalg.norm(v)))
                        ret = speedl([float(v[0]), float(v[1]), float(v[2]), 0.0, 0.0, 0.0], [opt.acc, 60.0], opt.watchdog)
                        if ret not in (None, 0):
                            err = f"speedl 반환 {ret}"
                            break
                        sent.append(time.monotonic())
                    zero()
                    wait_still()
                    time.sleep(0.3)
                finally:
                    w.catch_window(False)
                state = get_robot_state()
                if state not in ROBOT_OK:
                    err = (err + " · " if err else "") + f"로봇 상태 {state} (알람·정지 — 펜던트에서 해제)"
                smp = w.take(t_start)
                row = dict(test=kind, dir=name, amp_cm=amp, amp2_cm=opt.amp2 if kind == "moving" else "", switch_s=opt.switch if kind == "moving" else "",
                           kp=kp, vmax=opt.vmax, acc=opt.acc, rate_hz=opt.rate, watchdog_s=opt.watchdog, rep=rep + 1, error=err,
                           robot_state=state, vpeak_cmd_mps=vpeak_cmd / 1000.0, **send_stats(sent))
                row.update(analyse(smp, t_start, p0, tgt2, d))
                row["t_near5_s"] = near_time(smp, t_start, tgt2, 0.05)
                if kind == "moving":
                    t_sw = t_start + opt.switch
                    row["t_near5_after_switch_s"] = near_time([x for x in smp if x[0] >= t_sw], t_sw, tgt2, 0.05)
                row.update(w.force_stats(t_start, time.monotonic()))
                rows.append(row)
                print(f"{kind} {name} {amp:.0f}{'→' + format(opt.amp2, '.0f') if kind == 'moving' else ''} cm kp {kp:.0f} · 보냄 {row.get('send_hz', 0):.0f} Hz #{rep + 1}: "
                      f"시작 {row.get('t_start_ms') or float('nan'):.0f} ms · 5 cm 안 {row['t_near5_s'] if row['t_near5_s'] is not None else float('nan'):.3f} s · "
                      f"도착 {row.get('t_arrive_s') or float('nan'):.3f} s · 최고 {row.get('peak_mps', 0):.2f} m/s (명령 {vpeak_cmd / 1000:.2f}) · "
                      f"지나침 {row.get('overshoot_mm', 0):.1f} mm · 끝 오차 {row.get('final_err_mm', float('nan')):.1f} mm · 힘 최대 {row.get('force_peak_n', float('nan')):.1f} N"
                      + (f" · 바꾼 뒤 5 cm 안 {row['t_near5_after_switch_s'] if row.get('t_near5_after_switch_s') is not None else float('nan'):.3f} s" if kind == "moving" else "")
                      + (f" · {err}" if err else ""))
                if state not in ROBOT_OK:
                    raise SystemExit(err)
                w.wait_normal()
                safe_or_stop()
                movel(start6, vel=opt.return_vel, acc=opt.return_vel * 2)
                mwait()
                if err:
                    raise SystemExit(err)
    finally:
        if rows:
            keys = list(dict.fromkeys(k for r in rows for k in r))
            with open(out, "w", newline="") as fh:
                wr = csv.DictWriter(fh, fieldnames=keys)
                wr.writeheader(); wr.writerows(rows)
            print(f"\n저장: {out}")
        dsr.destroy_node()
        ex.shutdown()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
