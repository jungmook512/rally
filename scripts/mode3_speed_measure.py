#!/usr/bin/env python3
"""모드 3 — 가속 2000 속도·힘 실측 자동화 (docs/mode3/MODE3_PLAN.md 10절 1-b, 정동휘 10/12).

mode3_reach_test.py(박성현, PR #130 판 — force_peak_n 열이 있는 것) 를 밖에서 돌리는 래퍼다. 그 스크립트는 건드리지 않는다.
세 단계를 따로 부른다 (계획 결과를 사람이 보고 속도를 정한 뒤 실행하도록):

  1. plan     방향별로 속도 후보를 넣어 계획 모드(로봇 안 움직임)를 돌리고, 필요한 관절 속도 % 와 OK/빼기 표를 만든다
                 python3 scripts/mode3_speed_measure.py plan --go-ready            # 먼저 대기 자세로 이동(⚠ 20 °/s) 후 계획. 기본 후보 730 600 550 500 480 450
                 python3 scripts/mode3_speed_measure.py plan --vels 600 500 450 --dirs +u -u
  2. run      고른 속도로 실제 이동. 힘 한계 파라미터를 시험값으로 올렸다가 끝나면(오류·Ctrl+C 포함) 25 N 으로 되돌리고 확인한다 ⚠ 로봇 움직임
                 python3 scripts/mode3_speed_measure.py run --vel-y 730 --vel-u 480            # 힘 한계 기본 80 N (130 N 은 #144 ② 확인 전용, 명시할 때만)
  3. analyze  CSV 들을 읽어 방향별 "명령 뒤 --window s 안에 도착하는 거리"(보간) 와 관성 힘 최대를 뽑아 유효 범위·힘 한계 판정 요약을 쓴다
                 python3 scripts/mode3_speed_measure.py analyze data/measure/mode3_reach_1012_*.csv
              run 은 끝나면 analyze 를 자동으로 부른다.

시간 창 0.58 s = 공이 패드 면에 닿는 0.74 s − 출발 결정 0.16 s (MODE3_PLAN 5절). CSV 의 t_arrive_s 는 amovel 명령 시각 기준이라 그대로 비교한다.
안전: 로봇이 움직이는 run 은 안전감시·힘 노드가 켜져 있어야 하고(mode3_reach_test 가 매 이동 전 NORMAL 확인), 왼쪽(+y)은 받지 않는다.
      힘 한계 시험값은 정정인 승인 뒤에만. safety.yaml 파일은 바꾸지 않는다 (실행 중 ros2 param 만).
      아래(−u)는 대기 자세에서 약 9.4 cm 에 두산 자기충돌(10/11) → 기본 거리 5·8 cm, 8 cm 를 넘는 값은 받지 않는다.
전제: source scripts/robot.sh env (ros2 CLI 가 안전 노드를 찾아야 함).
"""
from __future__ import annotations

import argparse
import csv
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REACH = ROOT / "scripts" / "mode3_reach_test.py"
MEASURE = ROOT / "data" / "measure"
SAFETY_NODE = "/rally_safety_monitor"
FORCE_PARAM = "force_limit_n"
FORCE_DEFAULT = 25.0
FORCE_TEST_DEFAULT = 80.0  # 시험 기본 (10/13 정정인 #148): 실측 관성 최대 55 N + 여유. 130 N 은 받기 창(#165) 전용 값이라 시험 전체에는 명시할 때만
DOWN_MAX_CM = 8.0          # 10/11 −u 9.4 cm 자기충돌
PAD_R_CM = 7.5
WINDOW_S = 0.58
BALL_IMPACT_N = 15.4       # 10/11 정지 상태 공 충격 최대 (툴 힘 추정값)

READY_POSJ_DEFAULT = [89.8, 30.0, -121.7, 90.2, -80.2, 88.3]   # 10/11 정정인 결정 (skills.yaml mode3.ready_posj 가 있으면 그것)
SKILLS_YAML = ROOT / "ros2_ws" / "src" / "rally_bringup" / "config" / "skills.yaml"
READY_VEL_DEG_S, READY_ACC_DEG_S2 = 20.0, 40.0                  # 대기 자세로 가는 관절 속도·가속 (느리게)

PLAN_LINE = re.compile(r"^\s*([+-][yu])\s+(\d+)\s+\[.*?\]\s+(-?\d+)%\s+(OK|빼기.*)$")


def dir_args(dirs):
    """argparse 가 -y·-u 를 옵션으로 읽지 않게 mode3_reach_test 와 같은 방식(공백 붙임)으로 넘긴다."""
    return [d + " " if d in ("-y", "-u") else d for d in dirs]


def check_down(dists):
    bad = [d for d in dists if d > DOWN_MAX_CM]
    if bad:
        raise SystemExit(f"−u 거리 {bad} cm 는 자기충돌 한계 {DOWN_MAX_CM} cm 초과 — 받지 않음")


def ready_posj():
    try:
        import yaml
        v = (yaml.safe_load(SKILLS_YAML.read_text(encoding="utf-8")).get("mode3") or {}).get("ready_posj")
        if v and len(v) == 6:
            return [float(x) for x in v], str(SKILLS_YAML.relative_to(ROOT))
    except Exception:  # noqa: BLE001 — 파일 없음·키 없음이면 기본값
        pass
    return READY_POSJ_DEFAULT, "기본값(10/11 정정인)"


def go_ready(force_prompt=True):
    """대기 자세로 관절 이동 ⚠ 로봇 움직임. 이미 그 자세(0.5° 안)면 아무것도 안 함. bringup 만 있으면 됨 (안전 노드 불필요, 그래서 느리게)."""
    posj, src = ready_posj()
    from rally_robot.dsr_helpers import init_dsr
    dsr = init_dsr("rally_mode3_go_ready")
    from DSR_ROBOT2 import get_current_posj, movej, mwait  # noqa: E402  init_dsr 뒤
    cur = [float(v) for v in get_current_posj()]
    diff = max(abs(a - b) for a, b in zip(cur, posj))
    print(f"대기 자세 {posj} ({src}) · 지금 {[round(v, 1) for v in cur]} · 최대 차이 {diff:.1f}°")
    if diff < 0.5:
        print("이미 대기 자세 — 이동 없음")
        dsr.destroy_node()
        return
    if force_prompt and input(f"관절 이동 {READY_VEL_DEG_S:.0f} °/s 로 대기 자세로 간다. 로봇 반경에 사람 없고 비상정지 손 닿나? [y/N] ").strip().lower() != "y":
        dsr.destroy_node()
        raise SystemExit("취소")
    movej(posj, vel=READY_VEL_DEG_S, acc=READY_ACC_DEG_S2)
    mwait()
    cur = [float(v) for v in get_current_posj()]
    print(f"도착 {[round(v, 1) for v in cur]} (최대 차이 {max(abs(a - b) for a, b in zip(cur, posj)):.1f}°)")
    dsr.destroy_node()


# ------------------------------------------------------------------ 1. plan
def run_plan(dirs, dists, vel):
    """mode3_reach_test 계획 모드 한 번 → [(dir, dist_cm, pct, ok, why)]."""
    cmd = [sys.executable, str(REACH), "--dist", *map(str, dists), "--dirs", *dir_args(dirs), "--vel", str(vel), "--acc", "2000"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    rows = []
    for line in r.stdout.splitlines():
        m = PLAN_LINE.match(line)
        if m:
            rows.append((m.group(1), float(m.group(2)), float(m.group(3)), m.group(4) == "OK", m.group(4)))
    if r.returncode != 0 or not rows:
        raise SystemExit(f"계획 모드 실패 (vel {vel}):\n{r.stdout[-800:]}\n{r.stderr[-800:]}")
    return rows


def cmd_plan(opt):
    table = {}   # (dir, vel) -> (worst pct, all ok, why)
    for vel in opt.vels:
        dists_by_dir = {d: (opt.dist_down if d == "-u" else opt.dist) for d in opt.dirs}
        rows = []
        for d, ds in dists_by_dir.items():
            rows += run_plan([d], ds, vel)
        for d in opt.dirs:
            mine = [r for r in rows if r[0] == d]
            worst = max(r[2] for r in mine)
            ok = all(r[3] for r in mine)
            why = "; ".join(sorted({r[4] for r in mine if not r[3]}))
            table[(d, vel)] = (worst, ok, why)
            print(f"vel {vel:>4.0f}  {d:>3}  관절 속도 최대 {worst:>4.0f} %  {'OK' if ok else '빼기 — ' + why}")
    print("\n방향별로 OK 인 가장 빠른 속도 (mode3_reach_test 기준 80 % 이하):")
    for d in opt.dirs:
        oks = [v for v in opt.vels if table[(d, v)][1]]
        print(f"  {d:>3}: {max(oks):.0f} mm/s" if oks else f"  {d:>3}: 없음 — 후보를 더 낮게")
    print("\n이 표를 보고 run 의 --vel-y / --vel-u 를 정한다. 실행하지 않았다.")
    return 0


# ------------------------------------------------------------------ 2. run
def ros2_param(get_or_set, value=None):
    cmd = ["ros2", "param", get_or_set, SAFETY_NODE, FORCE_PARAM] + ([str(value)] if value is not None else [])
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    if r.returncode != 0:
        raise SystemExit(f"ros2 param {get_or_set} 실패 — 안전 노드({SAFETY_NODE}) 확인 (ros2 node list | grep safety)\n{r.stderr[-400:]}")
    m = re.search(r"(-?\d+(?:\.\d+)?)", r.stdout.splitlines()[-1]) if get_or_set == "get" else None
    return float(m.group(1)) if m else None


def run_reach(dirs, dists, vel, acc, reps):
    """실제 이동 한 묶음. 저장된 CSV 경로를 돌려준다 (없으면 None). STOP·알람이면 mode3_reach_test 가 메시지를 내고 끝난다."""
    cmd = [sys.executable, str(REACH), "--run", "--dist", *map(str, dists), "--dirs", *dir_args(dirs),
           "--vel", str(vel), "--acc", str(acc), "--reps", str(reps)]
    print("\n$ " + " ".join(cmd[1:]))
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(r.stdout[-3000:])
    if r.stderr.strip():
        print(r.stderr[-1500:], file=sys.stderr)
    m = re.search(r"저장: (\S+\.csv)", r.stdout)
    csv_path = Path(m.group(1)) if m else None
    if r.returncode != 0:
        print(f"!! 이 묶음은 정상 종료가 아님 (returncode {r.returncode}) — STOP·알람·거부 여부를 위 출력에서 확인. 다음 묶음은 멈춘다.")
    return csv_path, r.returncode == 0


def cmd_run(opt):
    check_down(opt.dist_down)
    before = ros2_param("get")
    restore = before if before is not None else FORCE_DEFAULT
    if restore > FORCE_DEFAULT:
        print(f"!! 시작 전 값 {restore} N 이 평소 {FORCE_DEFAULT} N 보다 큼 — 지난 시험이 원복 안 됐을 수 있음. 끝나면 {FORCE_DEFAULT} N 으로 되돌림")
        restore = FORCE_DEFAULT
    print(f"안전 노드 {SAFETY_NODE} {FORCE_PARAM} 현재 {before} N → 시험값 {opt.force_limit} N (끝나면 {restore} N 으로 되돌림)")
    batches = [("-y", opt.dist, opt.vel_y), ("+u", opt.dist, opt.vel_u), ("-u", opt.dist_down, opt.vel_u)]
    batches = [b for b in batches if b[0] in opt.dirs]
    print("실행 묶음 (가속 %.0f, 각 %d회):" % (opt.acc, opt.reps))
    for d, ds, v in batches:
        print(f"  {d:>3}  {ds} cm  vel {v:.0f}")
    if input("\n로봇이 움직인다. 사람이 로봇 옆에 있고 비상정지에 손이 닿으며, 힘 한계 시험값은 정정인 승인을 받았나? [y/N] ").strip().lower() != "y":
        print("취소"); return 1
    if opt.force_limit > FORCE_TEST_DEFAULT and input(
            f"\n힘 한계 {opt.force_limit:.0f} N 은 #144 ② (두산 충돌 감도가 먼저 걸리는지) 확인 전용이다. 팔 닿는 거리 안에 사람이 없고 "
            "운영자가 비상정지에 손을 올렸나? 결과는 #144 에 남긴다. [y/N] ").strip().lower() != "y":
        print("취소"); return 1
    csvs, notes = [], []
    try:                                          # set 직전부터 — 확인 get 실패·불일치·Ctrl+C 에도 finally 가 원복 (#148 리뷰)
        ros2_param("set", opt.force_limit)
        now = ros2_param("get")
        if now is None or abs(now - opt.force_limit) > 1e-6:
            raise SystemExit(f"힘 한계 설정 확인 실패 (읽은 값 {now})")
        print(f"힘 한계 {now} N 설정 확인")
        for d, ds, v in batches:
            path, ok = run_reach([d], ds, v, opt.acc, opt.reps)
            if path:
                csvs.append(path)
            notes.append(f"{d} vel {v:.0f}: {'정상' if ok else '비정상 종료(STOP·알람 확인)'}")
            if not ok:
                break
    finally:
        try:
            ros2_param("set", restore)
            back = ros2_param("get")
            print(f"\n힘 한계 원복: {back} N {'OK' if back is not None and abs(back - restore) < 1e-6 else '!! 확인 필요'}")
        except BaseException as e:  # noqa: BLE001 — SystemExit·TimeoutExpired·Ctrl+C 모두 수동 원복 안내 (#148 리뷰)
            print(f"!! 힘 한계 원복 실패 ({type(e).__name__}) — 직접: ros2 param set {SAFETY_NODE} {FORCE_PARAM} {restore}\n{e}")
            if isinstance(e, KeyboardInterrupt):
                raise
    print("\n묶음 결과: " + " · ".join(notes))
    if csvs:
        summarize(csvs, opt.window, opt.force_limit, stat="median", extra=f"힘 한계 시험값 {opt.force_limit} N · 가속 {opt.acc:.0f} · " + " · ".join(notes))
    return 0


# ------------------------------------------------------------------ 3. analyze
def load_rows(paths):
    rows = []
    for p in paths:
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                try:
                    rows.append(dict(src=Path(p).name, dir=r["dir"].strip(), dist=float(r["dist_cm"]), acc=float(r["acc"]), vel=float(r["vel"]),
                                     t_arrive=float(r["t_arrive_s"]) if r.get("t_arrive_s") not in (None, "", "None") else None,
                                     t_start=float(r["t_start_ms"]) if r.get("t_start_ms") not in (None, "", "None") else None,
                                     force=float(r["force_peak_n"]) if r.get("force_peak_n") not in (None, "", "None") else None,
                                     state=r.get("robot_state", "")))
                except (KeyError, ValueError) as e:
                    raise SystemExit(f"{p}: 열이 다름 ({e}) — PR #130 판 mode3_reach_test 의 CSV 인지 확인")
    return rows


def reach_within(points, window):
    """[(dist_cm, mean t_arrive_s)] 오름차순 → window 안에 도착하는 거리 (선형 보간). (거리, 설명)"""
    pts = sorted(points)
    if not pts:
        return None, "자료 없음"
    if pts[0][1] > window:
        return 0.0, f"가장 짧은 {pts[0][0]:.0f} cm 도 {pts[0][1]:.2f} s 로 창 밖"
    best = pts[0]
    for a, b in zip(pts[:-1], pts[1:]):
        if b[1] <= window:
            best = b; continue
        if a[1] <= window < b[1]:
            d = a[0] + (b[0] - a[0]) * (window - a[1]) / (b[1] - a[1])
            return d, f"{a[0]:.0f} cm ({a[1]:.2f} s) 와 {b[0]:.0f} cm ({b[1]:.2f} s) 사이 보간"
    return best[0], f"잰 거리 전부 창 안 (최대 {best[0]:.0f} cm, {best[1]:.2f} s) — 더 먼 거리 미측정이라 하한"


STATS = {"median": lambda v: sorted(v)[len(v) // 2] if len(v) % 2 else (sorted(v)[len(v) // 2 - 1] + sorted(v)[len(v) // 2]) / 2,
         "min": min, "mean": lambda v: sum(v) / len(v)}


def summarize(paths, window, force_limit, extra="", stat="median"):
    rows = load_rows(paths)
    lines = [f"# 모드 3 속도·힘 실측 요약 ({datetime.now():%Y-%m-%d %H:%M})", "", f"입력: {', '.join(Path(p).name for p in paths)}",
             f"시간 창 {window:.2f} s (0.74 − 0.16) · 도착 시간 대표값 {stat} · 판정 힘 한계 {force_limit:.0f} N · 패드 반지름 {PAD_R_CM} cm" + (f" · {extra}" if extra else ""), ""]
    lines += [f"| 방향 | 가속·속도 | 거리 cm | 도착 s ({stat}; 회차별) | 출발 ms | 관성 힘 최대 N | 로봇 상태 |", "|---|---|---|---|---|---|---|"]
    reach, force_max = {}, {}
    keys = sorted({(r["dir"], r["acc"], r["vel"]) for r in rows})
    for d, acc, vel in keys:
        pts = []
        for dist in sorted({r["dist"] for r in rows if (r["dir"], r["acc"], r["vel"]) == (d, acc, vel)}):
            g = [r for r in rows if (r["dir"], r["acc"], r["vel"], r["dist"]) == (d, acc, vel, dist)]
            ta = [r["t_arrive"] for r in g if r["t_arrive"] is not None]
            ts = [r["t_start"] for r in g if r["t_start"] is not None]
            fs = [r["force"] for r in g if r["force"] is not None]
            states = sorted({r["state"] for r in g})
            rep_ta = STATS[stat](ta) if ta else None
            if rep_ta is not None:
                pts.append((dist, rep_ta))
            if fs:
                force_max[d] = max(force_max.get(d, 0.0), max(fs))
            cells = [d, f"{acc:.0f}·{vel:.0f}", f"{dist:.0f}",
                     (f"{rep_ta:.2f} ({', '.join(f'{x:.2f}' for x in sorted(ta))})" if rep_ta is not None else "미도착"),
                     (f"{sum(ts) / len(ts):.0f}" if ts else "—"), (f"{max(fs):.0f}" if fs else "—"), ",".join(states) or "—"]
            lines.append("| " + " | ".join(cells) + " |")
        r_cm, why = reach_within(pts, window)
        reach[(d, acc, vel)] = (r_cm, why)
    lines += ["", "## 시간 창 안 도달 거리 → 유효 범위 (패드 반지름 + 도달 거리)", ""]
    for (d, acc, vel), (r_cm, why) in reach.items():
        if r_cm is None:
            lines.append(f"- {d} ({acc:.0f}·{vel:.0f}): {why}"); continue
        lines.append(f"- {d} ({acc:.0f}·{vel:.0f}): 도달 {r_cm:.1f} cm → 유효 {PAD_R_CM + r_cm:.1f} cm — {why}")
    lines += ["", "왼쪽(+y)은 이동하지 않으므로 유효 범위는 패드 끝 7.5 cm 까지 (10/11 규칙). 아래(−u)는 자기충돌 한계 8 cm 를 넘지 않는다.", "",
              "## 관성 힘 (패드 쥠, 공 안 맞음 — 툴 힘 추정값, 표본 끊김으로 과소 가능)", ""]
    for d, f in sorted(force_max.items()):
        verdict = "한계 안" if f + BALL_IMPACT_N <= force_limit else "!! 한계 초과 가능"
        lines.append(f"- {d}: 최대 {f:.0f} N + 공 충격 {BALL_IMPACT_N:.0f} N = {f + BALL_IMPACT_N:.0f} N → {force_limit:.0f} N {verdict}")
    bad = [r for r in rows if r["state"] not in ("1", "2", "1.0", "2.0", "")]
    if bad:
        lines += ["", f"!! 두산 알람·정지 상태로 끝난 이동 {len(bad)}건: " + ", ".join(f"{r['dir']} {r['dist']:.0f} cm (state {r['state']})" for r in bad)]
    text = "\n".join(lines)
    print("\n" + text)
    MEASURE.mkdir(parents=True, exist_ok=True)
    out = MEASURE / f"mode3_speed_summary_{datetime.now():%m%d_%H%M%S}.md"
    out.write_text(text + "\n", encoding="utf-8")
    print(f"\n저장: {out}")
    return out


def cmd_analyze(opt):
    summarize(opt.csv, opt.window, opt.force_limit, stat=opt.stat)
    return 0


# ------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("plan", help="계획 모드 속도 후보 표 (로봇 안 움직임)")
    p.add_argument("--vels", type=float, nargs="+", default=[730, 600, 550, 500, 480, 450])
    p.add_argument("--dirs", nargs="+", type=str.strip, default=["-y", "+u", "-u"], choices=["-y", "+u", "-u"])
    p.add_argument("--dist", type=float, nargs="+", default=[5, 10, 15], help="−y·+u 거리 cm")
    p.add_argument("--dist-down", type=float, nargs="+", default=[5, 8], help=f"−u 거리 cm (≤ {DOWN_MAX_CM})")
    p.add_argument("--go-ready", action="store_true", help="먼저 대기 자세로 관절 이동 ⚠ (skills.yaml mode3.ready_posj, 20 °/s)")
    r = sub.add_parser("run", help="실제 이동 + 힘 한계 시험값 설정·원복 + 요약 ⚠")
    r.add_argument("--vel-y", type=float, default=730.0, help="−y 속도 mm/s")
    r.add_argument("--vel-u", type=float, required=True, help="±u 속도 mm/s — plan 결과로 정한 값")
    r.add_argument("--acc", type=float, default=2000.0)
    r.add_argument("--reps", type=int, default=2)
    r.add_argument("--dirs", nargs="+", type=str.strip, default=["-y", "+u", "-u"], choices=["-y", "+u", "-u"])
    r.add_argument("--dist", type=float, nargs="+", default=[5, 10, 15])
    r.add_argument("--dist-down", type=float, nargs="+", default=[5, 8])
    r.add_argument("--force-limit", type=float, default=FORCE_TEST_DEFAULT,
                   help="시험 동안 힘 한계 N. 기본 80 (10/13 정정인). 130 은 #144 ② 확인 전용 — 받기 창 전용 값이라 전체 시험에는 명시할 때만")
    r.add_argument("--window", type=float, default=WINDOW_S)
    r.add_argument("--go-ready", action="store_true", help="먼저 대기 자세로 관절 이동 ⚠")
    a = sub.add_parser("analyze", help="CSV → 도달 거리·유효 범위·힘 요약")
    a.add_argument("csv", nargs="+")
    a.add_argument("--window", type=float, default=WINDOW_S)
    a.add_argument("--force-limit", type=float, default=FORCE_TEST_DEFAULT, help="판정에 쓸 힘 한계 N (기본 80)")
    a.add_argument("--stat", choices=list(STATS), default="median", help="회차별 도착 시간의 대표값 (표본 끊김 의심 회차가 있으면 min 도 같이 볼 것)")
    # "-y"·"-u" 를 값으로 넘기기 위한 처리 (mode3_reach_test 와 같은 이유)
    opt = ap.parse_args([x + " " if x in ("-y", "-u") else x for x in sys.argv[1:]])
    if opt.cmd in ("plan", "run"):
        check_down(opt.dist_down)
        if opt.go_ready:
            go_ready()
    if opt.cmd == "plan":
        return cmd_plan(opt)
    if opt.cmd == "run":
        return cmd_run(opt)
    return cmd_analyze(opt)


if __name__ == "__main__":
    sys.exit(main())
