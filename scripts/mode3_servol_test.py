#!/usr/bin/env python3
"""모드 3 servol 추종 시험 — 대기 자세에서 servol 로 목표를 계속 보내며 반응 지연·추종 오차를 잰다 (강화학습 2단계 로봇 모델용).
담당: 박성현

시험 (방향은 mode3_reach_test.py 와 같음: -y 오른쪽, +u / -u 패드 면 안 위·뒤 / 아래·앞. 왼쪽(+y)은 안 함)
  step : 목표를 amp 만큼 한 번에 옮기고 hold_s 동안 같은 목표를 rate Hz 로 계속 보냄 → 출발·도착·지나침 (amovel 시험과 비교)
  sine : 목표 = amp·(1 − cos 2πft)/2 (대기 자세에서 한쪽으로만 0 ~ amp 오감) 을 rate Hz 로 cycles 번 → 지연(ms)·진폭비·추종 오차
측정: /dsr01/joint_states(100 Hz) 순기구학 TCP (수신 시각 기준 — 지연에는 joint_states 전달 지연 수 ms 가 들어감).
     servol 은 servol_stream 토픽 발행이라 명령 자체는 막히지 않는다. 실제 보낸 간격(Hz·떨림)도 기록.

    source scripts/robot.sh env
    python3 scripts/mode3_servol_test.py                                   # 계획만 (로봇 안 움직임)
    python3 scripts/mode3_servol_test.py --run --tests step sine --dirs -y +u -u --amp 3 --freqs 0.5 1 2 --rates 30 60
결과 CSV: data/measure/mode3_servol_MMDD_HHMM.csv (한 줄 = 한 번 시험).
안전: servol 은 두산 속도 슬라이더·Safety Zone 감속을 따르지 않는다 → 매 명령 전에 /rally/safety/status 를 보고 NORMAL 이 아니면
      즉시 정지(move_stop) 후 끝냄 (SLOW 도 멈춤 — 이 시험은 감속 배율을 적용하지 않음). 폭은 5 cm 까지만, 시험마다 대기 자세로 천천히 복귀.
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

from mode3_reach_test import JOINT_VMAX, ROOT, Kin, Watch, analyse, directions, in_workspace

AMP_MAX_CM = 5.0
DIRS = ("-y", "+u", "-u")


def profile(kind: str, t: float, amp_m: float, freq_hz: float) -> float:
    """시각 t(s) 의 목표 이동량 (m, 방향 d 따라)."""
    if t < 0:
        return 0.0
    if kind == "step":
        return amp_m
    return amp_m * (1.0 - np.cos(2.0 * np.pi * freq_hz * t)) / 2.0


def peak_speed(kind: str, amp_m: float, freq_hz: float, vel_mm_s: float) -> float:
    """시험 중 TCP 최고 속도 예상 (m/s). step 은 servol vel 이 상한, sine 은 π·f·amp."""
    return vel_mm_s / 1000.0 if kind == "step" else np.pi * freq_hz * amp_m


def check(kin: Kin, q0, name: str, amp_cm: float, speed_mps: float) -> tuple[bool, str, float]:
    """직선 경로 0 ~ amp 를 0.5 cm 마다 IK·관절 속도(최고 속도에서)·작업 공간 검사."""
    d = directions()[name]
    p0, R0 = kin.tcp(q0)
    q, worst = q0.copy(), 0.0
    for s in np.linspace(0.0, 1.0, max(2, int(amp_cm / 0.5)) + 1)[1:]:
        q = kin.ik(p0 + d * amp_cm / 100.0 * s, R0, q)
        if q is None:
            return False, "IK 실패", worst
        worst = max(worst, float((np.abs(kin.joint_rate_for(q, d * speed_mps)) / JOINT_VMAX).max()))
    if not in_workspace(p0 + d * amp_cm / 100.0):
        return False, "안전 작업 공간 밖", worst
    if worst > 0.8:
        return False, f"관절 속도 한계의 {worst * 100:.0f}% (> 80%)", worst
    return True, "", worst


def lag_and_gain(t_cmd, s_cmd, t_act, s_act, max_lag_s: float = 0.4, dt: float = 0.005) -> dict:
    """명령·실제 (시각, 이동량) → 지연(실제가 명령을 몇 ms 늦게 따라오나), 진폭비, 오차.

    지연 = 실제(t) 와 명령(t − lag) 의 RMS 차이가 가장 작은 lag. 진폭비 = 그 지연에서 실제 대 명령 최소제곱 기울기.
    """
    t_cmd, s_cmd, t_act, s_act = (np.asarray(v, float) for v in (t_cmd, s_cmd, t_act, s_act))
    if len(t_cmd) < 4 or len(t_act) < 4:
        return {}
    grid = np.arange(max(t_cmd[0], t_act[0]) + max_lag_s, min(t_cmd[-1], t_act[-1]), dt)
    if len(grid) < 10:
        return {}
    act = np.interp(grid, t_act, s_act)
    lags = np.arange(0.0, max_lag_s + dt / 2, dt)
    rms = np.array([np.sqrt(np.mean((act - np.interp(grid - lag, t_cmd, s_cmd)) ** 2)) for lag in lags])
    k = int(rms.argmin())
    ref = np.interp(grid - lags[k], t_cmd, s_cmd)
    ref_c = ref - ref.mean()
    var = float(ref_c @ ref_c)
    gain = float(ref_c @ (act - act.mean()) / var) if var > 0 else float("nan")    # 지연 맞춘 뒤 최소제곱 기울기 (잡음에 강함)
    return dict(lag_ms=float(lags[k] * 1000), gain=gain,
                rms_err_mm=float(rms[0] * 1000), rms_err_after_lag_mm=float(rms[k] * 1000),
                max_err_mm=float(np.max(np.abs(act - np.interp(grid, t_cmd, s_cmd))) * 1000))


def send_stats(t_sent) -> dict:
    if len(t_sent) < 3:
        return {}
    dt = np.diff(np.asarray(t_sent, float))
    return dict(send_hz=float(1.0 / dt.mean()), send_jitter_ms=float(dt.std() * 1000))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="store_true", help="실제로 로봇을 움직임 (없으면 계획만)")
    ap.add_argument("--tests", nargs="+", default=["step", "sine"], choices=["step", "sine"])
    ap.add_argument("--dirs", nargs="+", type=str.strip, default=list(DIRS), choices=list(DIRS))
    ap.add_argument("--amp", type=float, default=3.0, help=f"cm (최대 {AMP_MAX_CM})")
    ap.add_argument("--freqs", type=float, nargs="+", default=[0.5, 1.0, 2.0], help="sine Hz")
    ap.add_argument("--rates", type=float, nargs="+", default=[30.0, 60.0], help="목표 보내는 주기 Hz")
    ap.add_argument("--cycles", type=int, default=3, help="sine 반복 수")
    ap.add_argument("--hold", type=float, default=1.0, help="step 목표 유지 s")
    ap.add_argument("--vel", type=float, default=450.0, help="servol 직선 속도 상한 mm/s")
    ap.add_argument("--acc", type=float, default=1000.0, help="servol 직선 가속 상한 mm/s²")
    ap.add_argument("--time", type=float, default=0.0, help="servol time 인자 (0 = 속도·가속으로)")
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--return-vel", type=float, default=100.0, help="대기 자세 복귀 mm/s")
    ap.add_argument("--settle", type=float, default=1.0, help="시험 사이 정지 s")
    opt = ap.parse_args([a + " " if a in ("-y", "-u") else a for a in sys.argv[1:]])   # "-y" 를 값으로 (reach 시험과 같음)
    if opt.amp > AMP_MAX_CM:
        print(f"--amp {opt.amp} cm > {AMP_MAX_CM} cm — 이 시험은 작은 폭만")
        return 1

    rclpy.init()
    kin = Kin()
    w = Watch(kin)
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()      # 멀티 스레드 실행기는 GIL 을 오래 잡아 명령 루프가 30 Hz 를 못 냄 (fake 시험 20 Hz·떨림 160 ms)
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
    p0, _ = kin.tcp(q0)

    cases = []
    print(f"대기 자세 관절 {np.round(np.degrees(q0), 1)}  TCP {np.round(p0, 3)} m · 폭 {opt.amp:.1f} cm · servol vel {opt.vel:.0f} acc {opt.acc:.0f}")
    print(f"{'시험':>5} {'방향':>4} {'Hz':>5} {'최고 m/s':>8} {'관절속도':>8}  판정")
    for kind in opt.tests:
        for name in opt.dirs:
            for f in (opt.freqs if kind == "sine" else [0.0]):
                v = peak_speed(kind, opt.amp / 100.0, f, opt.vel)
                ok, why, worst = check(kin, q0, name, opt.amp, v)
                print(f"{kind:>5} {name:>4} {f:>5.1f} {v:>8.2f} {worst * 100:>7.0f}%  {'OK' if ok else '빼기 — ' + why}")
                if ok:
                    cases += [(kind, name, f, rate) for rate in opt.rates]
    if not opt.run:
        print(f"\n계획만 (로봇 안 움직임). 실행할 시험 {len(cases)}개 × {opt.reps}회 = {len(cases) * opt.reps}번")
        ex.shutdown()
        rclpy.try_shutdown()
        return 0

    from rally_robot.dsr_helpers import init_dsr, move_stop
    dsr = init_dsr("rally_mode3_servol")
    from DSR_ROBOT2 import check_motion, get_current_posx, movel, mwait, servol  # noqa: E402  init_dsr 뒤
    start6 = list(get_current_posx()[0])
    out = ROOT / "data" / "measure" / f"mode3_servol_{datetime.now():%m%d_%H%M}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = []

    def safe_or_stop():
        s = w.safety
        if s is None or s.level != 0:
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
        for kind, name, f, rate in cases:
            d = directions()[name]
            dur = opt.hold if kind == "step" else opt.cycles / f
            for rep in range(opt.reps):
                w.wait_normal()                 # 멈춰 있을 때만 잠깐 기다림 (wrench_stale 순간). 스트리밍 중에는 그대로 즉시 정지
                safe_or_stop()
                time.sleep(opt.settle)
                w.clear()
                sent, cmd, err = [], [], ""
                t_start = time.monotonic()
                n = int(np.ceil((dur + 0.5) * rate))                 # 끝에 0.5 s 마지막 목표 유지
                for k in range(n):
                    t_tick = t_start + k / rate
                    while time.monotonic() < t_tick:
                        time.sleep(0.0005)
                    safe_or_stop()
                    s = profile(kind, min(k / rate, dur), opt.amp / 100.0, f)
                    pos = list(start6)
                    for i in range(3):
                        pos[i] = start6[i] + d[i] * s * 1000.0
                    tc = time.monotonic()
                    ret = servol(pos, vel=[opt.vel, 30.0], acc=[opt.acc, 60.0], time=opt.time)
                    if ret not in (None, 0):
                        err = f"servol 반환 {ret}"
                        break
                    sent.append(tc)
                    cmd.append(s)
                wait_still()
                time.sleep(0.3)
                smp = w.take(t_start)
                t_act = [t - t_start for t, _ in smp]
                s_act = [float((p - p0) @ d) for _, p in smp]
                row = dict(test=kind, dir=name, amp_cm=opt.amp, freq_hz=f, rate_hz=rate, vel=opt.vel, acc=opt.acc,
                           servo_time=opt.time, rep=rep + 1, error=err, **send_stats(sent))
                if kind == "step":
                    row.update(analyse(smp, t_start, p0, p0 + d * opt.amp / 100.0, d))
                else:
                    row.update(lag_and_gain([t - t_start for t in sent], cmd, t_act, s_act))
                row.update(w.force_stats(t_start, time.monotonic()))
                rows.append(row)
                print(f"{kind} {name} {f:.1f} Hz · 보냄 {row.get('send_hz', 0):.0f} Hz #{rep + 1}: "
                      + (f"시작 {row.get('t_start_ms') or float('nan'):.0f} ms · 도착 {row.get('t_arrive_s') or float('nan'):.3f} s · "
                         f"지나침 {row.get('overshoot_mm', 0):.1f} mm" if kind == "step" else
                         f"지연 {row.get('lag_ms', float('nan')):.0f} ms · 진폭비 {row.get('gain', float('nan')):.2f} · "
                         f"오차 {row.get('rms_err_mm', float('nan')):.1f} mm")
                      + f" · 힘 최대 {row.get('force_peak_n', float('nan')):.1f} N"
                      + (f" · {err}" if err else ""))
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
                wr.writeheader()
                wr.writerows(rows)
            print(f"\n저장: {out}")
        dsr.destroy_node()
        ex.shutdown()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
