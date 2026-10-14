#!/usr/bin/env python3
"""모드 3 대기 자세 최적화 — 관절 속도 80 % 제한에서 패드 평면 반경 R 안 모든 방향의 movel 허용 속도를 최대로 (정동휘 10/13).

무엇을 푸나
  변수: 패드 중심 p(x,y,z) + 패드 법선 둘레 회전 ψ (법선은 지금 대기 자세 값으로 고정 — 환자 쪽 위 45°) + IK 가지.
  목적: 패드 평면 원판(반경 R, 360°) 위 모든 점·모든 평면 내 이동 방향에서 "관절 속도 한계 × 80 %" 로 허용되는
        직선(movel, 자세 고정) 속도의 최솟값 v_worst 를 최대화. (가속 a 로 거리 R 을 가는 시간 = R/v + v/a, v ≥ √(aR) 이면 2√(R/a))
  제약(원판 위 모든 표본에서): 관절 한계(±360·95·135·360·135·360, 여유 3°), TCP 작업 공간(safety.yaml x ≤ 0.57·|y| ≤ 0.60),
        TCP·패드 아랫단 z ≥ Z_MIN(0.20, 설치물 여유), 팔꿈치·손목 원점 z ≥ 0.15, 손목(프레임 4~6)이 베이스 축에서 ≥ 0.15 m (박성현 10/12 조건, 지금 대기 자세 0.162)
        (어깨 특이점·베이스 기둥), 자코비안 조건수 < 200 (특이점), IK 수렴.
  모델 검증(10/13): 지금 대기 자세에서 위아래 0.489 m/s(실측 상한 ≈ 480 mm/s), 옆 0.739(실측 730 통과) — mode3_reach_test 와 같은 계산.
  못 넣은 것: 두산 자기충돌 모델·관절 가속 한계·케이블 → 결과 자세는 실로봇에서 천천히 확인.

    source scripts/robot.sh env   (또는 /opt/ros/jazzy + cobot2_ws — URDF 만 읽음, 로봇 불필요)
    python3 scripts/mode3_ready_pose_opt.py                       # 전역 탐색 + 국소 다듬기, 결과·선택지 표
    python3 scripts/mode3_ready_pose_opt.py --radius 0.35 --zmin 0.20 --fix-psi    # 회전까지 지금 값 고정
    python3 scripts/mode3_ready_pose_opt.py --eval 89.8 30.0 -121.7 90.2 -80.2 88.3  # 한 자세만 평가
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import differential_evolution, minimize
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from ejt_wrench import Chain  # noqa: E402
from print_pose import TCP_OFFSET_MM  # noqa: E402

SKILLS = ROOT / "ros2_ws/src/rally_bringup/config/skills.yaml"
HAND_EYE = ROOT / "ros2_ws/src/rally_bringup/config/hand_eye.yaml"
SAFETY = ROOT / "ros2_ws/src/rally_bringup/config/safety.yaml"
JOINT_VMAX_DEG = np.array([150.0, 150.0, 180.0, 225.0, 225.0, 225.0])   # M0609
JOINT_LIM_DEG = np.array([360.0, 95.0, 135.0, 360.0, 135.0, 360.0])
JOINT_MARGIN_DEG = 3.0
PAD_NORMAL_TOOL = np.array([-0.7071, 0.0, 0.7071])   # 공구 좌표 패드 법선 (pad_tcp_check.py)
PAD_R = 0.075


def camera_axis_tool():
    """손목 RealSense 광축(optical z)을 TCP 좌표로 — hand_eye.yaml tcp_T_camera_optical 쿼터니언. 없으면 공구 z."""
    try:
        he = yaml.safe_load(HAND_EYE.read_text(encoding="utf-8"))
        t = he.get("tcp_T_camera_optical") or next(v for v in he.values() if isinstance(v, dict) and "tcp_T_camera_optical" in v)["tcp_T_camera_optical"]
        qq = t.get("quat_xyzw") or t.get("quaternion")
        return Rotation.from_quat(qq).as_matrix() @ np.array([0.0, 0.0, 1.0])
    except Exception:  # noqa: BLE001
        return np.array([0.0, 0.0, 1.0])


def load_cfg():
    sk = yaml.safe_load(SKILLS.read_text(encoding="utf-8"))
    m3 = sk["mode3"]
    sa = yaml.safe_load(SAFETY.read_text(encoding="utf-8"))
    ws = None
    for node in sa.values():
        p = (node or {}).get("ros__parameters") if isinstance(node, dict) else None
        if p and "workspace_x_max_m" in p:
            ws = {a: (float(p[f"workspace_{a}_min_m"]), float(p[f"workspace_{a}_max_m"])) for a in "xyz"}
    return (np.radians(m3["ready_posj"]), np.array(m3["pad_offset_tool_m"], float),
            ws or {"x": (-2.0, 0.57), "y": (-0.60, 0.60), "z": (0.0, 2.0)})


class Model:
    def __init__(self, pad_off, ws, radius, zmin, scale=0.8, n_ang=12, step=0.05, no_left=False, cam_max_deg=None,
                 ws_margin=0.0, base_clear=0.15):
        self.c, self.off, self.pad_off, self.ws = Chain(), TCP_OFFSET_MM / 1000.0, pad_off, ws
        self.cam_axis_tool, self.cam_max_deg = camera_axis_tool(), cam_max_deg   # 손목 카메라가 환자 쪽(+x)을 봐야 공을 잡는다
        self.ws_margin, self.base_clear = ws_margin, base_clear
        self.R, self.zmin, self.wl = radius, zmin, np.radians(JOINT_VMAX_DEG) * scale
        self.lim = np.radians(JOINT_LIM_DEG - JOINT_MARGIN_DEG)
        ang = np.linspace(0, 2 * np.pi, n_ang, endpoint=False)
        # 0° = 평면 안 오른쪽(−y 쪽), 90° = 위. no_left 면 왼쪽(+y 쪽, 150~210°) 방향은 빼고 본다 (지금 규칙)
        self.angles = np.array([a for a in ang if not (no_left and 150 <= round(math.degrees(a)) <= 210)])
        n_step = max(1, int(round(radius / step)))
        self.radii = [radius * k / n_step for k in range(n_step + 1)]        # 0 … R, step 간격 (IK 가 가지를 안 바꾸게)

    # --- 기구학 ---
    def flange(self, q):
        _, T = self.c.fk_frames(q)
        return T[:3, 3], T[:3, :3]

    def tcp(self, q):
        p, R = self.flange(q)
        return p + R @ self.off, R

    def pad(self, q):
        p, R = self.tcp(q)
        return p + R @ self.pad_off, R @ PAD_NORMAL_TOOL, R

    def ik(self, p_flange, R_t, seed, iters=60):
        """플랜지 목표 (p, R) → 관절. 감쇠 최소제곱 뉴턴. 실패면 None."""
        q = np.array(seed, float)
        for _ in range(iters):
            J, T = self.c.jacobian(q)
            e = np.r_[p_flange - T[:3, 3], Rotation.from_matrix(R_t @ T[:3, :3].T).as_rotvec()]
            if np.linalg.norm(e[:3]) < 2e-5 and np.linalg.norm(e[3:]) < 1e-4:
                return q
            dq = np.linalg.solve(J.T @ J + 1e-6 * np.eye(6), J.T @ e)
            q = q + np.clip(dq, -0.5, 0.5)
        return None

    def plane_basis(self, n):
        up = np.array([0.0, 0.0, 1.0])
        u = up - n * (up @ n); u /= np.linalg.norm(u)      # 평면 안 '위'
        r = np.cross(n, u); r /= np.linalg.norm(r)         # 평면 안 '오른쪽'(−y 쪽)
        return u, r

    # --- 한 표본 자세의 점검 ---
    def check(self, q):
        """제약 위반 사유 목록(비면 OK) 와 이 점의 방향별 허용 속도 (평면 내 n_ang 방향)."""
        why = []
        if np.any(np.abs(q) > self.lim):
            why.append("joint_limit")
        frames, T = self.c.fk_frames(q)
        R = T[:3, :3]
        tcp = T[:3, 3] + R @ self.off
        n = R @ PAD_NORMAL_TOOL
        pad = tcp + R @ self.pad_off
        pad_low = pad[2] - PAD_R * math.sqrt(max(0.0, 1.0 - n[2] ** 2))
        mg = self.ws_margin
        if not (self.ws["x"][0] + mg <= tcp[0] <= self.ws["x"][1] - mg) or not (self.ws["y"][0] + mg <= tcp[1] <= self.ws["y"][1] - mg):
            why.append("workspace")
        if self.cam_max_deg is not None:
            cam = R @ self.cam_axis_tool
            if math.degrees(math.acos(float(np.clip(cam[0], -1, 1)))) > self.cam_max_deg:
                why.append("camera_away")
        if tcp[2] < self.zmin or pad_low < self.zmin:
            why.append("z_min")
        orig = np.array([F[:3, 3] for F in frames])          # 관절 1~6 원점
        if np.any(orig[2:, 2] < 0.15):
            why.append("link_low")
        if np.any(np.linalg.norm(orig[3:, :2], axis=1) < self.base_clear):
            why.append("wrist_near_base")
        J, _ = self.c.jacobian(q)
        singular = np.linalg.cond(J) > 200
        if singular:
            why.append("singular")
        u, r = self.plane_basis(n)
        if singular:
            return why, np.zeros(len(self.angles)), tcp, pad, n
        Jinv = np.linalg.inv(J)
        vmax = []
        for a in self.angles:                                 # 제약 위반과 무관하게 속도는 계산 (보고용). 목적함수가 위반을 따로 벌점
            d = math.cos(a) * r + math.sin(a) * u
            qd = Jinv @ np.r_[d, 0, 0, 0]
            vmax.append(float(np.min(self.wl / np.maximum(np.abs(qd), 1e-9))))
        return why, np.array(vmax), tcp, pad, n

    # --- 자세 하나(원판 전체) 평가 ---
    def evaluate(self, q0, detail=False):
        """q0 = 대기 자세. 원판(반경 R) 위 표본을 방사선마다 IK 로 따라가며 점검. 반환: (v_worst, 위반 수, 상세)."""
        p0, R0 = self.flange(q0)
        n = R0 @ PAD_NORMAL_TOOL
        u, r = self.plane_basis(n)
        worst, nbad, bad = np.inf, 0, {}
        per_dir, along = {}, {}
        for ia, a in enumerate(self.angles):
            d = math.cos(a) * r + math.sin(a) * u
            q, vmin_dir, vmin_along = q0, np.inf, np.inf
            for rad in self.radii:
                if rad > 0:
                    q = self.ik(p0 + d * rad, R0, q)
                if q is None:
                    nbad += 1; bad[f"{math.degrees(a):.0f}°/{rad:.2f}"] = ["ik"]; vmin_dir = 0.0; break
                why, vmax, *_ = self.check(q)
                if why:
                    nbad += 1; bad[f"{math.degrees(a):.0f}°/{rad:.2f}"] = why
                    if rad == 0:                                  # 대기 자세 자체가 위반이면 이 가지는 버림 (방사선마다 같은 점)
                        return (0.0, nbad * len(self.angles), dict(per_dir={}, bad=bad)) if detail else (0.0, nbad * len(self.angles))
                vmin_dir = min(vmin_dir, float(vmax.min()))       # 그 점에서 모든 방향 (최악)
                vmin_along = min(vmin_along, float(vmax[ia]))     # 그 방사선을 따라 그 방향으로 갈 때
            per_dir[round(math.degrees(a))] = vmin_dir
            along[round(math.degrees(a))] = vmin_along
            worst = min(worst, vmin_dir)
        if detail:
            return worst, nbad, dict(per_dir=per_dir, along=along, bad=bad)
        return worst, nbad

    # --- 변수 → 자세 ---
    def pose_from_x(self, x, R_ready):
        """x = [pad_x, pad_y, pad_z, ψ] → 플랜지 목표 (p, R). ψ 는 패드 법선 둘레 회전."""
        n = R_ready @ PAD_NORMAL_TOOL
        R = Rotation.from_rotvec(n * x[3]).as_matrix() @ R_ready
        pad = np.array(x[:3])
        tcp = pad - R @ self.pad_off
        return tcp - R @ self.off, R

    def solve_branches(self, p, R, seeds):
        sols = []
        for s in seeds:
            q = self.ik(p, R, s)
            if q is None:
                continue
            q = (q + np.pi) % (2 * np.pi) - np.pi
            if all(np.linalg.norm(q - t) > 0.05 for t in sols):
                sols.append(q)
        return sols


def move_time(d, v, a):
    if not v or v <= 0:
        return math.inf                                            # 허용 속도 0 (실현 불가)
    v = min(v, math.sqrt(a * d))
    return d / v + v / a


def fmt_q(q):
    return "[" + ", ".join(f"{v:.1f}" for v in np.degrees(q)) + "]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius", type=float, default=0.35)
    ap.add_argument("--zmin", type=float, default=0.20)
    ap.add_argument("--scale", type=float, default=0.8, help="관절 속도 한계 배율")
    ap.add_argument("--fix-psi", action="store_true", help="법선 둘레 회전 ψ 를 0 (지금 자세 회전 그대로)")
    ap.add_argument("--acc", type=float, nargs="+", default=[2.0, 4.0], help="시간 계산용 가속 m/s²")
    ap.add_argument("--seeds", type=int, default=12)
    ap.add_argument("--gens", type=int, default=40)
    ap.add_argument("--pop", type=int, default=12)
    ap.add_argument("--eval", type=float, nargs=6, help="posj deg 하나만 평가")
    ap.add_argument("--out", default=str(ROOT / "data/measure/mode3_ready_pose_opt.json"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-left", action="store_true", help="왼쪽(+y 쪽) 방향은 빼고 (지금 규칙)")
    ap.add_argument("--cam-max-deg", type=float, default=35.0, help="손목 카메라 광축과 +x 사이 최대 각 (지금 자세 약 11°). 음수면 제한 없음")
    ap.add_argument("--yaw-deg", type=float, default=0.0, help="지금 자세 회전에 베이스 z 둘레 이만큼 더 돌림 (지금 법선 yaw −10°, +10 이면 정면 +x)")
    ap.add_argument("--n-ang", type=int, default=16)
    ap.add_argument("--step", type=float, default=0.035)
    ap.add_argument("--ws-margin", type=float, default=0.02, help="작업 공간 안쪽 여유 m")
    ap.add_argument("--base-clear", type=float, default=0.16, help="손목 ↔ 베이스 축 최소 거리 m")
    a = ap.parse_args()

    q_ready, pad_off, ws = load_cfg()
    M = Model(pad_off, ws, a.radius, a.zmin, a.scale, n_ang=a.n_ang, step=a.step, no_left=a.no_left,
              cam_max_deg=None if a.cam_max_deg < 0 else a.cam_max_deg, ws_margin=a.ws_margin, base_clear=a.base_clear)
    rng = np.random.default_rng(a.seed)
    p_r, R_ready = M.flange(q_ready)
    if a.yaw_deg:
        R_ready = Rotation.from_euler("z", a.yaw_deg, degrees=True).as_matrix() @ R_ready   # 패드 법선·카메라를 베이스 z 둘레로
    pad0, n0, _ = M.pad(q_ready)
    cam0 = R_ready @ M.cam_axis_tool
    print(f"지금 대기 자세 {fmt_q(q_ready)}  패드 중심 {np.round(pad0, 3)}  법선 {np.round(n0, 3)}  카메라축 {np.round(cam0, 2)} (+x 와 {math.degrees(math.acos(cam0[0])):.0f}°)")
    w0, nb0, d0 = M.evaluate(q_ready, detail=True)
    print(f"  반경 {a.radius:.2f} 원판: v_worst {w0:.3f} m/s, 위반 표본 {nb0} 개  방향별 최소 {{{', '.join(f'{k}°:{v:.2f}' for k, v in d0['per_dir'].items())}}}")

    if a.eval:
        q = np.radians(a.eval)
        w, nb, d = M.evaluate(q, detail=True)
        print(f"평가 {fmt_q(q)}: v_worst {w:.3f}, 위반 {nb}, 방향별 {d['per_dir']}, 위반 {d['bad']}")
        return

    def seeds():
        base = [q_ready]
        for _ in range(a.seeds - 1):
            base.append(rng.uniform(-M.lim, M.lim))
        return base

    cache = {}

    def objective(x):
        key = tuple(np.round(x, 4))
        if key in cache:
            return cache[key]
        if a.fix_psi:
            x = np.r_[x[:3], 0.0]
        p, R = M.pose_from_x(x, R_ready)
        best = 10.0                                            # 최소화: -v_worst + 위반 벌점
        for q in M.solve_branches(p, R, seeds()):
            w, nb = M.evaluate(q)
            val = -w + 0.5 * nb if np.isfinite(w) else 10.0
            best = min(best, val)
        cache[key] = best
        return best

    bounds = [(0.05, ws["x"][1] - 0.02), (ws["y"][0] + 0.02, ws["y"][1] - 0.02), (a.zmin + 0.05, 0.95)]
    if not a.fix_psi:
        bounds.append((-math.pi, math.pi))
    t0 = time.time()
    res = differential_evolution(objective, bounds, maxiter=a.gens, popsize=a.pop, seed=a.seed, tol=1e-4,
                                 polish=False, init="latinhypercube",
                                 x0=np.r_[pad0, 0.0][:len(bounds)])
    loc = minimize(objective, res.x, method="Nelder-Mead", options=dict(xatol=1e-3, fatol=1e-4, maxfev=400))
    x = loc.x if loc.fun < res.fun else res.x
    if a.fix_psi:
        x = np.r_[x[:3], 0.0]
    p, R = M.pose_from_x(x, R_ready)
    cands = []
    for q in M.solve_branches(p, R, seeds()):
        w, nb, d = M.evaluate(q, detail=True)
        cands.append((w, nb, q, d))
    cands.sort(key=lambda t: (-t[0] + 0.5 * t[1]))
    w, nb, q, d = cands[0]
    tcp, _ = M.tcp(q)
    pad, n, _ = M.pad(q)
    print(f"\n최적 ({time.time() - t0:.0f} s): 패드 중심 {np.round(pad, 3)} (지금 대비 Δ {np.round(pad - pad0, 3)}) ψ {math.degrees(x[3]):.1f}°")
    cam = M.tcp(q)[1] @ M.cam_axis_tool
    print(f"  posj {fmt_q(q)}  TCP {np.round(tcp, 3)}  법선 {np.round(n, 3)}  카메라축 {np.round(cam, 2)} (+x 와 {math.degrees(math.acos(float(np.clip(cam[0], -1, 1)))):.0f}°)")
    print(f"  v_worst {w:.3f} m/s (지금 {w0:.3f}), 위반 표본 {nb} 개 {d['bad'] if nb else ''}")
    print("  방사선 위 모든 방향 최소(평면, 0°=오른쪽 −y 쪽, 90°=위): " + ", ".join(f"{k}°:{v:.2f}" for k, v in d["per_dir"].items()))
    print("  그 방향으로 갈 때 허용 속도: " + ", ".join(f"{k}°:{v:.2f}" for k, v in d["along"].items()))
    for acc in a.acc:
        print(f"  가속 {acc:.0f} m/s² 로 {a.radius * 100:.0f} cm: 최악 방향 {move_time(a.radius, w, acc):.3f} s, 한계 없을 때 {2 * math.sqrt(a.radius / acc):.3f} s")
    out = dict(radius=a.radius, zmin=a.zmin, scale=a.scale, fix_psi=a.fix_psi, yaw_deg=a.yaw_deg, cam_max_deg=a.cam_max_deg, no_left=a.no_left, cam_axis=cam.tolist(), pad_center=pad.tolist(), psi_deg=math.degrees(x[3]),
               posj_deg=np.degrees(q).tolist(), tcp=tcp.tolist(), normal=n.tolist(), v_worst=w, violations=nb,
               per_dir=d["per_dir"], along=d["along"], bad=d["bad"],
               ready_now=dict(posj_deg=np.degrees(q_ready).tolist(), v_worst=w0, violations=nb0, per_dir=d0["per_dir"], along=d0.get("along", {})),
               other_branches=[dict(posj_deg=np.degrees(c[2]).tolist(), v_worst=c[0], violations=c[1]) for c in cands[1:4]])
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"  저장 {a.out}")


if __name__ == "__main__":
    main()
