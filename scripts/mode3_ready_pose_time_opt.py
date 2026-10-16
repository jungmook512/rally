#!/usr/bin/env python3
"""모드 3 대기 자세 최적화 ② — "반경 R 위 가장 늦게 도착하는 방향의 시간"을 최소로 (정동휘 10/14 조건).

조건 (10/14 정동휘)
  · 패드 피치(위로 45°)는 고정, 롤·요는 베이스 축 기준 ±10° (요는 정면 0° 기준 — --yaw-center 로 지금 자세(−10°) 기준도 가능)
  · 방향: 패드 평면 360° (왼·오른·위·아래 모두), 반경 R = 0.35 (15·25 cm 시간도 같이)
  · x·y 제한 없음 (safety.yaml 작업 공간은 결과에 표시만), z: 관절 3~6 원점(팔꿈치·손목) + TCP + 패드 아랫단 ≥ 0.20
    (관절 1·2 원점은 베이스 어깨라 z 0.134 고정 — 제외)
  · 직선 가속 2000 mm/s², 관절 속도 한계 150·150·180·225·225·225 × 80 %
  · 팔 형태: IK 가지 자유. 손목(관절 4~6 원점) ↔ 베이스 축 ≥ 0.16, 관절 한계 여유 3°, 자코비안 조건수 < 200
시간 계산 (실제 movel 방식): 방향 d 로 R 만큼 갈 때 경로 위 모든 점에서 관절 80 % 가 허용하는 직선 속도의 최솟값 v_c 를
  그 이동의 속도 명령으로 두고, 가속 a 사다리꼴: t = R/v_c + v_c/a (v_c < √(aR)), 아니면 2√(R/a).
  (참고로 '경로 따라 속도를 바꾸는 이상적 프로파일' 시간도 같이 출력 — 두산 movel 은 그렇게 못 함)
목적: max_d t(d) 최소화. 제약 위반은 큰 벌점.

    python3 scripts/mode3_ready_pose_time_opt.py                      # 전역(DE 다중 시작) + 국소, 결과 json
    python3 scripts/mode3_ready_pose_time_opt.py --eval 89.8 30.0 -121.7 90.2 -80.2 88.3
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import differential_evolution, minimize
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from mode3_ready_pose_opt import JOINT_LIM_DEG, JOINT_MARGIN_DEG, JOINT_VMAX_DEG, PAD_NORMAL_TOOL, PAD_R, Model, camera_axis_tool, load_cfg  # noqa: E402


class TimeModel(Model):
    def __init__(self, pad_off, ws, radius, zmin, acc, n_ang=16, step=0.02, base_clear=0.16,
                 cond_max=200.0, j5_min_sin=0.0, reach_max=9.9, keep_ws=False):
        super().__init__(pad_off, ws, radius, zmin, 0.8, n_ang=n_ang, step=step, base_clear=base_clear)
        self.acc, self.step = acc, step
        # 10/14 2차: 1차 결과가 손목 특이점(|sin J5| 0.26)·팔 거의 뻗음(TCP 0.83 m) 틈을 파서 조임 옵션
        self.cond_max, self.j5_min_sin, self.reach_max, self.keep_ws = cond_max, j5_min_sin, reach_max, keep_ws
        # 10/14 저녁 #208: Y0 받기 이동 중 J2 모터 몸통이 패드 거치대에 닿음 (실로봇). 설치물 상자 ↔ 충돌 메시 여유 제약 (선택)
        self.body, self.boxes, self.box_margin = None, [], 0.0
        self.transition_from, self.trans_ws = None, None

    def set_obstacles(self, boxes, margin, pad_off, pts_per_link=400, transition_from_deg=None, trans_ws=None):
        """boxes: m0609_collision.Box 목록 — 받기 이동의 모든 표본에서 여유 ≥ margin. transition_from_deg(예: 거치대 U) 가 있으면
        그 자세 → 대기 자세 관절 직선 이동도 같은 여유 + 작업 공간(trans_ws) 안이어야 함 (경유 자세 없이 가게)."""
        from m0609_collision import RobotBody
        self.body = RobotBody(pad_offset_tool_m=pad_off, max_pts_per_link=pts_per_link)
        self.boxes, self.box_margin = list(boxes), float(margin)
        self.transition_from = None if transition_from_deg is None else np.radians(transition_from_deg)
        self.trans_ws = trans_ws

    def box_clear(self, q):
        return self.body.clearance(q, self.boxes) if self.body is not None and self.boxes else (np.inf, "")

    def transition_bad(self, q0, n=24):
        """transition_from → q0 관절 직선 이동 중 문제 (없으면 빈 목록)."""
        if self.transition_from is None:
            return []
        bad = []
        for s in np.linspace(0.0, 1.0, n + 1)[1:-1]:
            q = self.transition_from + s * (q0 - self.transition_from)
            d, who = self.box_clear(q)
            if d < self.box_margin:
                bad.append(f"{s:.2f}:trans_box({who},{d * 100:.1f}cm)"); break
            if self.trans_ws is not None:
                _, T = self.c.fk_frames(q)
                tcp = T[:3, 3] + T[:3, :3] @ self.off
                if not (self.trans_ws["x"][0] <= tcp[0] <= self.trans_ws["x"][1] and self.trans_ws["y"][0] <= tcp[1] <= self.trans_ws["y"][1]):
                    bad.append(f"{s:.2f}:trans_ws"); break
        return bad

    def check_point(self, q):
        """한 점 제약: 관절 한계 · z ≥ zmin(관절 원점 전부 + TCP + 패드 아랫단) · 손목-베이스 · 특이점. 작업 공간은 안 봄 (10/14)."""
        why = []
        if np.any(np.abs(q) > self.lim):
            why.append("joint_limit")
        frames, T = self.c.fk_frames(q)
        R = T[:3, :3]
        tcp = T[:3, 3] + R @ self.off
        n = R @ PAD_NORMAL_TOOL
        pad = tcp + R @ self.pad_off
        pad_low = pad[2] - PAD_R * math.sqrt(max(0.0, 1.0 - n[2] ** 2))
        orig = np.array([F[:3, 3] for F in frames])
        zs = np.r_[orig[2:, 2], tcp[2], pad_low]              # 관절 3~6 원점(1·2 는 베이스 어깨, z 0.134 고정) + TCP + 패드 아랫단
        if zs.min() < self.zmin:
            why.append("z_min")
        if np.any(np.linalg.norm(orig[3:, :2], axis=1) < self.base_clear):
            why.append("wrist_near_base")
        if abs(math.sin(q[4])) < self.j5_min_sin:
            why.append("wrist_singular")                      # J4·J6 축이 나란해짐 (박성현 10/11 조건 |sin J5| ≥ 0.5)
        if np.linalg.norm(tcp) > self.reach_max:
            why.append("reach")                               # 팔 뻗음 (M0609 최대 0.9 m)
        if self.keep_ws and not (self.ws["x"][0] <= tcp[0] <= self.ws["x"][1] and self.ws["y"][0] <= tcp[1] <= self.ws["y"][1]):
            why.append("workspace")
        J, _ = self.c.jacobian(q)
        cond = float(np.linalg.cond(J))
        if cond > self.cond_max:
            why.append("singular")
        if self.body is not None and not why:                 # 비싼 검사는 다른 제약을 통과한 점만
            d, _ = self.box_clear(q)
            if d < self.box_margin:
                why.append("box")
        return why, J, cond, tcp, pad, n, orig

    def ray_speed(self, q0, d, radius):
        """방향 d 로 radius 까지 1 cm 씩: (경로 최소 허용 속도 v_c, 점별 v_lim 목록, 위반 목록, 최대 조건수, 끝 관절값)."""
        p0, R0 = self.flange(q0)
        n_step = max(1, int(round(radius / self.step)))
        q, vlims, bad, cmax, qs = q0, [], [], 0.0, [q0]
        for k in range(1, n_step + 1):
            s = radius * k / n_step
            q = self.ik(p0 + d * s, R0, q)
            if q is None:
                return 0.0, vlims, bad + [f"{s:.2f}:ik"], cmax, qs
            why, J, cond, *_ = self.check_point(q)
            cmax = max(cmax, cond)
            if why:
                bad.append(f"{s:.2f}:{'+'.join(why)}")
                if "singular" in why:
                    return 0.0, vlims, bad, cmax, qs
            qd = np.linalg.solve(J, np.r_[d, 0, 0, 0])
            vlims.append(float(np.min(self.wl / np.maximum(np.abs(qd), 1e-9))))
            qs.append(q)
        # 출발점 자체
        J0, _ = self.c.jacobian(q0)
        qd0 = np.linalg.solve(J0, np.r_[d, 0, 0, 0])
        vlims.insert(0, float(np.min(self.wl / np.maximum(np.abs(qd0), 1e-9))))
        return min(vlims), vlims, bad, cmax, qs

    def t_single(self, dist, v_c):
        if v_c <= 0:
            return math.inf
        v = min(v_c, math.sqrt(self.acc * dist))
        return dist / v + v / self.acc

    def t_profile(self, vlims, dist):
        """경로 따라 속도 상한이 바뀌는 이상적 최소 시간 (앞·뒤 패스, 양 끝 정지)."""
        n = len(vlims) - 1
        if n < 1:
            return math.inf
        ds = dist / n
        v = np.array(vlims, float)
        f = np.zeros(n + 1)
        for i in range(1, n + 1):
            f[i] = min(v[i], math.sqrt(f[i - 1] ** 2 + 2 * self.acc * ds))
        b = np.zeros(n + 1)
        for i in range(n - 1, -1, -1):
            b[i] = min(f[i], math.sqrt(b[i + 1] ** 2 + 2 * self.acc * ds))
        t = 0.0
        for i in range(n):
            va = max(0.5 * (b[i] + b[i + 1]), 1e-6)
            t += ds / va
        return t

    def evaluate(self, q0, radii=(0.15, 0.25, 0.35), detail=False):
        """대기 자세 q0: 방향별 v_c·시간(반경마다), 위반, 최악 시간(최대 반경)."""
        why0, J0, cond0, tcp, pad, n, orig = self.check_point(q0)
        if why0:
            return (math.inf, dict(bad={"0": why0}, per_dir={}, times={}, cond=cond0, tmax=math.inf, tcp=tcp, pad=pad, normal=n, origins=orig)) if detail else math.inf
        u, r = self.plane_basis(n)
        Rmax = max(radii)
        per_dir, times, bad, cmax = {}, {rr: {} for rr in radii}, {}, cond0
        tb = self.transition_bad(q0)
        if tb:
            bad["trans"] = tb
        tmax = 0.0
        for a in self.angles:
            d = math.cos(a) * r + math.sin(a) * u
            deg = round(math.degrees(a))
            v_c, vlims, b, cm, _ = self.ray_speed(q0, d, Rmax)
            cmax = max(cmax, cm)
            if b:
                bad[deg] = b
            per_dir[deg] = dict(v_c=v_c, v_profile=vlims)
            n_step = len(vlims) - 1
            for rr in radii:
                k = max(1, int(round(rr / Rmax * n_step)))
                vc_r = min(vlims[:k + 1]) if vlims else 0.0
                times[rr][deg] = dict(single=self.t_single(rr, vc_r), profile=self.t_profile(vlims[:k + 1], rr))
            tmax = max(tmax, times[Rmax][deg]["single"])
        nbad = sum(len(v) for v in bad.values())
        score = tmax + 2.0 * nbad                              # 위반 한 점당 2 s 벌점
        if detail:
            return score, dict(per_dir=per_dir, times=times, bad=bad, tmax=tmax, cond=cmax, tcp=tcp, pad=pad, normal=n, origins=orig)
        return score


def orientation(R_front, yaw_deg, roll_deg):
    """정면(요 0°) 회전 R_front 에 베이스 z 둘레 yaw, 베이스 x 둘레 roll."""
    return (Rotation.from_euler("z", yaw_deg, degrees=True) * Rotation.from_euler("x", roll_deg, degrees=True)).as_matrix() @ R_front


def fmt_q(q):
    return "[" + ", ".join(f"{v:.1f}" for v in np.degrees(q)) + "]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius", type=float, default=0.35)
    ap.add_argument("--radii", type=float, nargs="+", default=[0.15, 0.25, 0.35])
    ap.add_argument("--zmin", type=float, default=0.20)
    ap.add_argument("--acc", type=float, default=2.0, help="직선 가속 m/s²")
    ap.add_argument("--yaw-center", type=float, default=0.0, help="요 범위 중심 (정면 0°, 지금 자세는 -10)")
    ap.add_argument("--ry", type=float, default=10.0, help="요 ± 범위 deg")
    ap.add_argument("--rr", type=float, default=10.0, help="롤 ± 범위 deg")
    ap.add_argument("--n-ang", type=int, default=16)
    ap.add_argument("--step", type=float, default=0.02)
    ap.add_argument("--gens", type=int, default=30)
    ap.add_argument("--pop", type=int, default=10)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--eval", type=float, nargs=6)
    ap.add_argument("--cond-max", type=float, default=200.0)
    ap.add_argument("--j5-min-sin", type=float, default=0.0, help="|sin J5| 최소 (0.5 = 박성현 10/11 조건)")
    ap.add_argument("--reach-max", type=float, default=9.9, help="TCP ↔ 베이스 거리 상한 m")
    ap.add_argument("--keep-ws", action="store_true", help="safety.yaml 작업 공간(x ≤ 0.57 등)도 제약으로")
    ap.add_argument("--box", type=float, nargs=6, action="append", metavar=("XLO", "YLO", "ZLO", "XHI", "YHI", "ZHI"),
                    help="피할 설치물 상자 base [m] (여러 번). 예: 패드 거치대(패드 없음) 0.25 -0.25 0 0.30 -0.05 0.17 (#208)")
    ap.add_argument("--box-margin", type=float, default=0.05, help="상자 ↔ 충돌 메시 최소 여유 m")
    ap.add_argument("--transition-from", type=float, nargs=6, help="이 관절값(예: 거치대 U) → 대기 자세 직선 관절 이동도 상자·작업 공간 검사")
    ap.add_argument("--out", default=str(ROOT / "data/measure/mode3_ready_pose_time_opt.json"))
    a = ap.parse_args()

    q_ready, pad_off, ws = load_cfg()
    M = TimeModel(pad_off, ws, a.radius, a.zmin, a.acc, n_ang=a.n_ang, step=a.step,
                  cond_max=a.cond_max, j5_min_sin=a.j5_min_sin, reach_max=a.reach_max, keep_ws=a.keep_ws)
    if a.box:
        from m0609_collision import Box
        M.set_obstacles([Box(tuple(b[:3]), tuple(b[3:]), f"box{i}") for i, b in enumerate(a.box)], a.box_margin, M.pad_off,
                        transition_from_deg=a.transition_from, trans_ws=ws)
    rng = np.random.default_rng(a.seed)
    p_r, R_ready = M.flange(q_ready)
    n_now = R_ready @ PAD_NORMAL_TOOL
    yaw_now = math.degrees(math.atan2(n_now[1], n_now[0]))
    R_front = Rotation.from_euler("z", -yaw_now, degrees=True).as_matrix() @ R_ready     # 정면(요 0°) 기준 회전
    pad0, *_ = M.pad(q_ready)
    s0, d0 = M.evaluate(q_ready, a.radii, detail=True)
    print(f"지금 대기 자세 {fmt_q(q_ready)}  패드 {np.round(pad0, 3)}  법선 yaw {yaw_now:.1f}°")
    print(f"  반경 {a.radius}: 최악 시간 {d0['tmax']:.3f} s, 위반 {sum(len(v) for v in d0['bad'].values())} 점, 방향별 v_c " +
          ", ".join(f"{k}:{v['v_c']:.2f}" for k, v in d0["per_dir"].items()))

    def seeds():
        return [q_ready] + [rng.uniform(-M.lim, M.lim) for _ in range(a.seeds - 1)]

    def branches(x):
        R = orientation(R_front, x[3], x[4])
        tcp = np.array(x[:3]) - R @ M.pad_off
        p_fl = tcp - R @ M.off
        return M.solve_branches(p_fl, R, seeds()), R

    cache = {}

    def objective(x):
        key = tuple(np.round(x, 4))
        if key in cache:
            return cache[key]
        best = 50.0
        sols, _ = branches(x)
        for q in sols:
            best = min(best, M.evaluate(q, (a.radius,)))
        cache[key] = best
        return best

    if a.eval:
        q = np.radians(a.eval)
        s, d = M.evaluate(q, a.radii, detail=True)
        print(json.dumps(dict(score=s, tmax=d["tmax"], times={str(k): v for k, v in d["times"].items()}, bad=d["bad"]), ensure_ascii=False, indent=1, default=float))
        return

    bounds = [(-0.3, 0.9), (-0.9, 0.9), (a.zmin + 0.15, 1.1),
              (a.yaw_center - a.ry, a.yaw_center + a.ry), (-a.rr, a.rr)]
    t0 = time.time()
    res = differential_evolution(objective, bounds, maxiter=a.gens, popsize=a.pop, seed=a.seed, tol=1e-4, polish=False,
                                 init="latinhypercube",
                                 x0=np.r_[np.clip(pad0, [b[0] for b in bounds[:3]], [b[1] for b in bounds[:3]]),
                                          float(np.clip(yaw_now, *bounds[3])), 0.0])         # 지금 패드 위치·요에서 시작 (범위 안으로)
    loc = minimize(objective, res.x, method="Nelder-Mead", options=dict(xatol=1e-3, fatol=1e-4, maxfev=300))
    x = loc.x if loc.fun < res.fun else res.x
    sols, R = branches(x)
    cands = sorted(((M.evaluate(q, a.radii, detail=True), q) for q in sols), key=lambda t: t[0][0])
    (s, d), q = cands[0]
    cam = M.tcp(q)[1] @ M.cam_axis_tool
    nb = sum(len(v) for v in d["bad"].values())
    print(f"\n최적 ({time.time() - t0:.0f} s): 패드 중심 {np.round(d['pad'], 3)} (Δ {np.round(d['pad'] - pad0, 3)})  요 {x[3]:+.1f}° 롤 {x[4]:+.1f}°")
    print(f"  posj {fmt_q(q)}  TCP {np.round(d['tcp'], 3)}  법선 {np.round(d['normal'], 3)}  카메라축 x {cam[0]:+.2f}  조건수 최대 {d['cond']:.0f}")
    print(f"  반경 {a.radius} 최악 시간 {d['tmax']:.3f} s (지금 {d0['tmax']:.3f}), 위반 {nb} 점 {d['bad'] if nb else ''}")
    for rr in a.radii:
        ts = d["times"].get(rr)
        if not ts:
            print(f"  반경 {rr * 100:.0f} cm: 평가 불가 (출발점 위반 {d['bad']})"); continue
        worst = max(ts, key=lambda k: ts[k]["single"])
        print(f"  반경 {rr * 100:.0f} cm: 최악 {ts[worst]['single']:.3f} s ({worst}°), 이상 프로파일 {max(v['profile'] for v in ts.values()):.3f} s, 한계 없을 때 {2 * math.sqrt(rr / a.acc):.3f} s")
    print("  방향별 v_c(m/s): " + ", ".join(f"{k}:{v['v_c']:.2f}" for k, v in d["per_dir"].items()))
    out = dict(radius=a.radius, radii=a.radii, zmin=a.zmin, acc=a.acc, yaw_center=a.yaw_center, ry=a.ry, rr=a.rr,
               limits=dict(cond_max=a.cond_max, j5_min_sin=a.j5_min_sin, reach_max=a.reach_max, keep_ws=a.keep_ws,
                           boxes=a.box, box_margin=a.box_margin, transition_from=a.transition_from),
               x=x.tolist(), pad_center=d["pad"].tolist(), tcp=d["tcp"].tolist(), normal=d["normal"].tolist(), cam_axis=cam.tolist(),
               posj_deg=np.degrees(q).tolist(), score=s, tmax=d["tmax"], cond_max=d["cond"], violations=nb, bad=d["bad"],
               per_dir={str(k): v["v_c"] for k, v in d["per_dir"].items()},
               times={str(rr): {str(k): v for k, v in ts.items()} for rr, ts in d["times"].items()},
               origins=np.asarray(d["origins"]).tolist(),
               ready_now=dict(posj_deg=np.degrees(q_ready).tolist(), tmax=d0["tmax"], per_dir={str(k): v["v_c"] for k, v in d0["per_dir"].items()}, bad=d0["bad"]),
               other_branches=[dict(posj_deg=np.degrees(c[1]).tolist(), score=c[0][0], tmax=c[0][1]["tmax"]) for c in cands[1:4]])
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    print(f"  저장 {a.out}")


if __name__ == "__main__":
    main()
