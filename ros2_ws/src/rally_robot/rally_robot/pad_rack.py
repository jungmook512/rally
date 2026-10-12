"""rally_robot/pad_rack.py — 모드 3 패드 거치대: 집기(pick_pad) · 꽂기(place_pad)
담당: 정정인 · 모드 3 (요격 catch_ball 은 박성현 — 이 파일은 요격 전후의 패드 옮기기만)

교시 (skills.yaml mode3, **관절값이 기준**). print_pose.py 의 posx 는 수동 모드에서 URDF 계산이라 컨트롤러와 z 4 mm 쯤 다르다 (10/12).
여기서는 posj 를 두산 fkin 으로 컨트롤러 posx 로 바꿔 쓴다.
  A pad_rack_approach_posj  그리퍼를 연 채 손잡이 앞 (G 에서 손잡이 축 따라 4 cm 뒤)
  G pad_rack_grasp_posj     거치대에 꽂힌 패드 손잡이를 쥔 자세 (그리퍼 수평, 손잡이 축 47.7°·위로 1.4°)
  U pad_rack_lift_posj      G 에서 쥔 채 거치대 홈 따라 20 cm 위 (수직에서 1.2° — 패드가 기울어 꽂힘)

집기 pick_pad:  A(관절) → 열기 → G(직선, 접촉 속도) → 쥐기 → 폭 확인 → U(직선, 옆·회전은 무르게 순응 — 홈에 안 걸리게)
                → 순응 해제 → U 다시 맞춤 → 요격 대기 자세 (mode3.ready_posj, 성현·동휘 결정)
  (요격·받은 뒤 처리는 그 사이에 — 나중에)
꽂기 place_pad: (대기 자세에 있으면) 대기 자세 → U 로 · 아니면 U(관절) → 홈 입구 위 P(G 에서 홈 축 따라 pre_m)
                → 순응(옆·회전 무르게) 하강 G 너머 below_m 까지,
                위아래 힘이 contact_n 넘으면(연속 3 샘플) 멈춤 → 열기 → 순응 해제 → 지금 자리에서 (A − G) 만큼 직선으로 빠짐
10/12 실로봇: 쥘 때마다 폭 28.7~31.2 mm — 손잡이를 쥐는 자리가 몇 mm 달라 패드가 그만큼 어긋난 채 단단한 직선으로 꽂다가
거치대에 부딪혀 force_limit STOP. 옆을 무르게 하면 패드가 홈 모서리를 따라 미끄러져 들어간다 (모드 5 내려놓기·공 집기와 같은 방식).
U ↔ 대기 자세: 바로 관절 이동하면 TCP 가 y −0.62 m 까지 나가 작업 공간(±0.60) 밖 (10/12 계산). 팔(J1~3)을 먼저 대기 자세 값으로,
그다음 손목(J4~6) — 돌아올 땐 반대 (y −0.51~0.07 m, z ≥ 0.34 m). ready_route()
공통: 단계 사이 check(), 접촉 구간은 node.set_contact(True) (nudge STOP 방지, 힘 한계는 그대로), 순응은 compliance() 로 반드시 해제.
"""
from __future__ import annotations

import time

import numpy as np

from rally_robot.dsr_helpers import SkillAbort, compliance, motion_speeds, move_checked, movej_checked

DEFAULTS = dict(
    pad_rack_grasp_posj=None, pad_rack_approach_posj=None, pad_rack_lift_posj=None,
    pad_grip_min_mm=24.0, pad_grip_max_mm=38.0,             # 손잡이 쥔 폭 (10/12: 28.7~31.2 mm). 밖이면 잘못 쥔 것
    pad_insert_pre_m=0.03,                                  # 꽂기: 홈 입구 위 이만큼에서 한 번 멈춘 뒤 순응 하강
    pad_insert_below_m=0.01,                                # G 보다 이만큼 더 내려가라고 명령 (닿으면 힘으로 멈춤)
    pad_insert_contact_n=6.0,                               # 위아래 힘 변화 이 이상 연속 3 샘플 → 다 꽂힘
    pad_insert_stiffness=[500, 500, 1000, 30, 30, 30],      # 꽂기 순응 [x y z N/m, rx ry rz Nm/rad] — 옆·회전 무르게
    pad_lift_stiffness=[1000, 1000, 3000, 50, 50, 50],      # 빼기 순응 — 홈에 걸려도 옆으로 따라가게
    pad_move_vel_mm_s=40.0,                                 # A↔G·G↔U 직선 속도 상한 (접촉 하강 속도와 작은 쪽)
    pad_joint_vel_deg_s=30.0, pad_joint_acc_deg_s2=60.0,    # A·U 로 가는 관절 이동 상한 — 거치대 공·손 보기와 같은 이유(60/120 에서 관성 힘 25 N 초과)
)


def config(node) -> dict:
    m3 = (getattr(node, "cfg", None) or {}).get("mode3") or {}
    cfg = {**DEFAULTS, **{k: v for k, v in m3.items() if k in DEFAULTS}}
    for k in ("pad_rack_grasp_posj", "pad_rack_approach_posj", "pad_rack_lift_posj"):
        if not cfg[k] or len(cfg[k]) != 6:
            raise SkillAbort(f"no_pad_rack_teach (skills.yaml mode3.{k})")
    return cfg


def rack_points(fk, cfg) -> dict:
    """교시 관절값 → 컨트롤러 posx {A, G, U} (mm·deg). fk 는 DSR fkin (실패하면 int)."""
    out = {}
    for name, key in (("A", "pad_rack_approach_posj"), ("G", "pad_rack_grasp_posj"), ("U", "pad_rack_lift_posj")):
        p = fk([float(v) for v in cfg[key]])
        if p is None or isinstance(p, (int, float)) or len(p) < 6:
            raise SkillAbort(f"fkin_failed ({name})")
        out[name] = [float(v) for v in p[:6]]
    return out


def insert_points(G, U, pre_m, below_m):
    """꽂기 직선: 홈 축(G → U 방향) 위의 입구 위 P = G + 축·pre, 끝 B = G − 축·below. 자세는 G. 순수 함수."""
    g, u = np.array(G[:3], float), np.array(U[:3], float)
    d = u - g
    n = float(np.linalg.norm(d))
    if n < 1.0:
        raise SkillAbort("pad_rack_lift_too_short")
    axis = d / n
    rot = [float(v) for v in G[3:6]]
    return [float(v) for v in g + axis * pre_m * 1000.0] + rot, [float(v) for v in g - axis * below_m * 1000.0] + rot


def retreat_from(cur, G, A):
    """지금 자리(순응으로 G 에서 몇 mm 비켜 있을 수 있음)에서 (A − G) 만큼 직선으로 빠지는 점. 자세는 A. 순수 함수.
    절대 A 로 가면 비켜 선 만큼 손가락이 손잡이를 옆으로 긁는다."""
    return [float(cur[i] + A[i] - G[i]) for i in range(3)] + [float(v) for v in A[3:6]]


def ready_route(u_posj, ready_posj):
    """U → 대기 자세 경유 관절값 [팔만 대기 자세로, 끝]. 손목(J4~6)은 U 그대로 두고 팔(J1~3)만 먼저 옮긴다. 순수 함수."""
    via = [float(v) for v in ready_posj[:3]] + [float(v) for v in u_posj[3:6]]
    return [via, [float(v) for v in ready_posj]]


def _near(a, b, tol_deg=3.0) -> bool:
    return max(abs(float(x) - float(y)) for x, y in zip(a, b)) <= tol_deg


def grip_ok(ok, width, detected, cfg) -> bool:
    return bool(ok) and bool(detected) and cfg["pad_grip_min_mm"] <= float(width) <= cfg["pad_grip_max_mm"]


def _speeds(node, cfg):
    tv, ta, cv, ca, jv, ja = motion_speeds(node)
    lv = min(float(cv), float(cfg["pad_move_vel_mm_s"]))
    return tv, ta, lv, ca, min(float(jv), float(cfg["pad_joint_vel_deg_s"])), min(float(ja), float(cfg["pad_joint_acc_deg_s2"]))


def pick_pad(node, log, check=lambda: None):
    """거치대에서 패드 집기 → U 에서 끝. 실패는 SkillAbort (잘못 쥐었으면 놓고 A 로 빠진 뒤)."""
    from DSR_ROBOT2 import fkin
    cfg = config(node)
    pts = rack_points(fkin, cfg)
    tv, ta, lv, ca, jv, ja = _speeds(node, cfg)
    open_mm, force = node.gripper_cfg("pad")
    log.info(f"[pad_rack] 집기 A {[round(v) for v in pts['A'][:3]]} → G {[round(v) for v in pts['G'][:3]]} → U {[round(v) for v in pts['U'][:3]]}")
    check()
    movej_checked(node, cfg["pad_rack_approach_posj"], jv, ja)
    ok, _, _ = node.gripper(open_mm, force)
    if not ok:
        raise SkillAbort("gripper_unavailable")
    check()
    node.set_contact(True)
    try:
        move_checked(node, pts["G"], lv, ca)
        ok, width, detected = node.gripper(0.0, force)
        if not grip_ok(ok, width, detected, cfg):
            log.warning(f"[pad_rack] 손잡이를 잘못 쥠 (폭 {width:.1f} mm, 감지 {detected}) — 놓고 A 로 빠짐")
            node.gripper(open_mm, force)
            move_checked(node, pts["A"], lv, ca)
            raise SkillAbort(f"pad_grasp_failed ({width:.1f} mm)")
        log.info(f"[pad_rack] 쥠 폭 {width:.1f} mm — 홈 따라 들어 올림")
        check()
        with compliance(cfg["pad_lift_stiffness"], log):
            move_checked(node, pts["U"], lv, ca, stall_ok=True)      # 순응 중엔 몇 mm 덜 갈 수 있음 → 아래에서 다시 맞춤
    finally:
        node.set_contact(False)
    check()
    move_checked(node, pts["U"], lv, ca)
    ready = _ready_posj(node)
    if ready is not None:
        for q in ready_route(cfg["pad_rack_lift_posj"], ready):
            check()
            movej_checked(node, q, jv, ja)
        log.info("[pad_rack] 집기 끝 — 요격 대기 자세")
    else:
        log.info("[pad_rack] 집기 끝 (U) — mode3.ready_posj 없음")
    return pts


def _ready_posj(node):
    q = ((getattr(node, "cfg", None) or {}).get("mode3") or {}).get("ready_posj")
    return [float(v) for v in q] if q and len(q) == 6 else None


def place_pad(node, log, check=lambda: None):
    """쥔 패드를 거치대에 꽂고 놓은 뒤 A 쪽으로 빠짐. 실패는 SkillAbort."""
    from DSR_ROBOT2 import fkin, get_current_posx, get_tool_force
    cfg = config(node)
    pts = rack_points(fkin, cfg)
    tv, ta, lv, ca, jv, ja = _speeds(node, cfg)
    open_mm, force = node.gripper_cfg("pad")
    P, B = insert_points(pts["G"], pts["U"], cfg["pad_insert_pre_m"], cfg["pad_insert_below_m"])
    log.info(f"[pad_rack] 꽂기 U → 입구 위 {[round(v) for v in P[:3]]} → 순응 하강 (끝 {[round(v) for v in B[:3]]}, {cfg['pad_insert_contact_n']:.0f} N 에서 멈춤)")
    from DSR_ROBOT2 import get_current_posj
    ready = _ready_posj(node)
    if ready is not None and _near(get_current_posj(), ready):
        via, _ = ready_route(cfg["pad_rack_lift_posj"], ready)
        check()
        movej_checked(node, via, jv, ja)                     # 대기 자세 → 손목 먼저 U 값으로 (팔은 그대로)
    check()
    movej_checked(node, cfg["pad_rack_lift_posj"], jv, ja)
    check()
    move_checked(node, P, lv, ca)
    time.sleep(0.3)                                          # 흔들림 가라앉힘
    check()
    node.set_contact(True)
    hits, fz0 = [0], {}

    def seated():                                            # 연속 3 샘플 넘어야 (이동 중 힘 튐 무시)
        hits[0] = hits[0] + 1 if abs(get_tool_force()[2] - fz0["z"]) >= cfg["pad_insert_contact_n"] else 0
        return hits[0] >= 3

    try:
        with compliance(cfg["pad_insert_stiffness"], log):
            fz0["z"] = get_tool_force()[2]                   # 순응을 켠 뒤의 영점
            how = move_checked(node, B, min(lv, 20.0), ca, on_poll=seated, stall_ok=True)
            cur = [float(v) for v in get_current_posx()[0]]
            off = float(np.linalg.norm(np.array(cur[:3]) - np.array(pts["G"][:3])))
            log.info(f"[pad_rack] 꽂힘 ({how}) — G 와 {off:.1f} mm")
            if how == "done":
                log.warning("[pad_rack] 힘 감지 없이 끝까지 내려감 — 패드가 홈에 들어갔는지 확인")
            ok, _, _ = node.gripper(open_mm, force)
            time.sleep(0.3)
        check()
        cur = [float(v) for v in get_current_posx()[0]]
        move_checked(node, retreat_from(cur, pts["G"], pts["A"]), lv, ca)
    finally:
        node.set_contact(False)
    if not ok:
        raise SkillAbort("gripper_open_failed")
    log.info("[pad_rack] 꽂기 끝 (A 쪽으로 빠짐)")
    return pts
