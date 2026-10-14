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
                → 순응(옆·회전 무르게) 하강 G 너머 below_m 까지 (G 위 slow_from 부터 slow_vel),
                세 축 힘 변화가 contact_n 넘으면(연속 3 샘플) 또는 합력이 abs_stop_n 넘으면(한 샘플) 멈춤 → G 위 max_gap 보다 높으면 입구에 걸린 것:
                홈 밖(G 위 clear)까지 올라가 search_pattern 의 옆 이동·base z 회전으로 바꿔 다시 하강 (패턴 수만큼) → 그래도 걸리면 쥔 채 P 로 빠져 중단
                → 아니면 열기 → 순응 해제 → 지금 자리에서 (A − G) 만큼 직선으로 빠짐
10/12 실로봇: 쥘 때마다 폭 28.7~31.2 mm — 손잡이를 쥐는 자리가 몇 mm 달라 패드가 그만큼 어긋난 채 단단한 직선으로 꽂다가
거치대에 부딪혀 force_limit STOP. 옆을 무르게 하면 패드가 홈 모서리를 따라 미끄러져 들어간다 (모드 5 내려놓기·공 집기와 같은 방식).
U ↔ 대기 자세: 바로 관절 이동하면 TCP 가 y −0.62 m 까지 나가 작업 공간(±0.60) 밖 (10/12 계산). 팔(J1~3)을 먼저 대기 자세 값으로,
그다음 손목(J4~6) — 돌아올 땐 반대 (y −0.51~0.07 m, z ≥ 0.34 m). ready_route()
공통: 단계 사이 check(), 접촉 구간은 node.set_contact(True) (nudge STOP 방지, 힘 한계는 그대로), 순응은 compliance() 로 반드시 해제.
"""
from __future__ import annotations

import time

import math

import numpy as np

from rally_robot.dsr_helpers import SkillAbort, compliance, motion_speeds, move_checked, movej_checked

DEFAULTS = dict(
    pad_rack_grasp_posj=None, pad_rack_approach_posj=None, pad_rack_lift_posj=None,
    pad_grip_min_mm=24.0, pad_grip_max_mm=38.0,             # 손잡이 쥔 폭 (10/12: 28.7~31.2 mm). 밖이면 잘못 쥔 것
    pad_insert_pre_m=0.08,                                  # 꽂기: G 위 이만큼(P)까지만 단단한 직선, 그 아래는 순응 하강. 10/12 20:29 실로봇: 0.03 이면
                                                            #   U → P 단단한 직선 끝에서 패드가 거치대에 걸려 96 N (컨트롤러 External_Force + force_limit STOP)
    pad_insert_max_gap_m=0.02,                              # 힘으로 멈춘 자리가 G 위 이보다 높으면 홈 입구에 걸린 것 — 놓지 않고 P 로 빠져 중단
    pad_insert_below_m=0.01,                                # G 보다 이만큼 더 내려가라고 명령 (닿으면 힘으로 멈춤)
    pad_insert_contact_n=6.0,                               # 세 축 힘 변화 이 이상 연속 3 샘플 → 다 꽂힘 (또는 걸림 — max_gap 으로 가름)
    pad_insert_abs_stop_n=15.0,                             # 툴 힘 합력 |F| 이 이 이상이면 한 샘플이라도 바로 멈춤 — safety force_limit 25 N(같은 get_tool_force 합력)보다 먼저.
                                                            #   10/14 실로봇: 다시 시도 뒤 눌린 채 영점을 잡아 변화량 10 N 전에 |F| 27.9 N → force_limit STOP (다 꽂힌 순간).
                                                            #   18 N 은 툴 힘 10 Hz 라 19.6~23.3 N 까지 넘침 (10:27) → 15
    pad_insert_slow_from_m=0.02,                            # G 위 이 높이부터는 slow_vel 로 (힘 판정이 0.3 s 늦어도 덜 눌리게)
    pad_insert_slow_vel_mm_s=10.0,
    pad_insert_shift_m=[0.0, 0.0, 0.0],                     # 꽂기 경로(P·B) 고정 옆 보정 [m, base]. 집기 G 는 그대로. 다시 시도 로그의 '옆 이동' 으로 정함
    pad_insert_search_pattern_m=[[0.0, 0.0, 0.0, 2.0], [0.0, 0.0, 0.0, -2.0], [0.0, 0.0, 0.0, 4.0], [0.0, 0.0, 0.0, -4.0]],
                                                            # 걸리면 shift 에서 이 순서로 다시 시도 [dx, dy, dz m (base), rz deg (base z 회전, 생략 가능)]. [] = 안 함
                                                            #   10/14 실로봇: xy 를 어디로 옮겨도(±3 mm) 5번 모두 G 위 35~36 mm 같은 높이에서 걸림 → 평행이동이 아니라
                                                            #   패드가 거치대와 돌아가 있어 한쪽 끝이 걸림 (정정인) → base z 회전으로 찾음. 툴 y ≈ base z 라 패드가 TCP 바로 아래 매달려 TCP 회전 ≈ 패드 세로축 회전
    pad_insert_clear_m=0.045,                               #   옆으로 옮기기 전에 G 위 이 높이(홈 밖)까지 올라감 — 홈 안에서 옆으로 끌지 않게
    pad_insert_backoff_m=0.008,                             #   입구 위쪽에서 걸렸으면 걸린 자리보다 이만큼은 올라감 (clear 보다 높을 때)
    pad_insert_stiffness=[500, 500, 1000, 30, 30, 30],      # 꽂기 순응 [x y z N/m, rx ry rz Nm/rad] — 옆·회전 무르게
    pad_lift_stiffness=[1000, 1000, 3000, 50, 50, 50],      # 빼기 순응 — 홈에 걸려도 옆으로 따라가게
    pad_move_vel_mm_s=40.0,                                 # A↔G·G↔U 직선 속도 상한 (접촉 하강 속도와 작은 쪽)
    pad_joint_vel_deg_s=30.0, pad_joint_acc_deg_s2=60.0,    # A·U 로 가는 관절 이동 상한 — 거치대 공·손 보기와 같은 이유(60/120 에서 관성 힘 25 N 초과)
    pad_rack_slow_radius_m=0.35,                            # ★ 10/14: TCP 가 거치대 G 에서 이 안에 있을 때 출발하는 관측·손 보기 관절 이동도 위 속도로.
                                                            #   세션 시작 '관측 자세로'(go_observe_pose, 60/120)가 거치대 자세에서 출발하며 가속 관성 힘으로
                                                            #   force_limit STOP 3회 (아무것도 안 걸림, 5 cm 만 가고 멈춤). 0 = 안 낮춤
)


def taught(node) -> bool:
    """거치대 세 자리(A·G·U) 관절값이 skills.yaml mode3 에 다 있는지."""
    m3 = (getattr(node, "cfg", None) or {}).get("mode3") or {}
    return all(len(m3.get(k) or []) == 6 for k in ("pad_rack_grasp_posj", "pad_rack_approach_posj", "pad_rack_lift_posj"))


def joint_speeds(node):
    """패드 쥔 관절 이동 속도 (deg/s, deg/s²) — motion: 과 pad_joint_* 중 작은 쪽."""
    m3 = (getattr(node, "cfg", None) or {}).get("mode3") or {}
    _, _, _, _, jv, ja = motion_speeds(node)
    return (min(float(jv), float(m3.get("pad_joint_vel_deg_s", DEFAULTS["pad_joint_vel_deg_s"]))),
            min(float(ja), float(m3.get("pad_joint_acc_deg_s2", DEFAULTS["pad_joint_acc_deg_s2"]))))


def near_rack(tcp_mm, g_mm, radius_m: float) -> bool:
    """TCP 가 거치대 G 에서 radius_m 안인가 (mm 입력). radius_m ≤ 0 이면 항상 False. 순수 함수."""
    if radius_m <= 0:
        return False
    return math.dist([float(v) for v in tcp_mm[:3]], [float(v) for v in g_mm[:3]]) <= radius_m * 1000.0


def slow_speeds(cfg, jv, ja, near: bool):
    """거치대 근처(near)면 관절 속도·가속을 pad_joint_* 이하로. 순수 함수."""
    if not near:
        return float(jv), float(ja)
    return min(float(jv), float(cfg["pad_joint_vel_deg_s"])), min(float(ja), float(cfg["pad_joint_acc_deg_s2"]))


def joint_speeds_here(node, jv, ja, log=None):
    """지금 TCP 가 거치대 G 에서 pad_rack_slow_radius_m 안이면 (jv, ja) 를 pad_joint 속도로 낮춰 돌려준다 (10/14 — 관측·손 보기
    자세로 가는 관절 이동이 거치대 자세에서 출발할 때 가속 관성 힘 25 N 초과). 교시가 없거나 fkin·posx 를 못 읽으면 그대로."""
    if not taught(node) or getattr(node, "fake", False):
        return float(jv), float(ja)
    try:
        from DSR_ROBOT2 import fkin, get_current_posx
        cfg = config(node)
        G = rack_points(fkin, cfg)["G"]
        cur = get_current_posx()[0]
    except Exception:  # noqa: BLE001 — 속도 판단 실패는 이동을 막지 않는다 (기존 속도)
        return float(jv), float(ja)
    near = near_rack(cur, G, float(cfg["pad_rack_slow_radius_m"]))
    out = slow_speeds(cfg, jv, ja, near)
    if near and log is not None and (out[0] < float(jv) or out[1] < float(ja)):
        log.info(f"[pad_rack] 거치대 G 에서 {math.dist(cur[:3], G[:3]):.0f} mm — 관절 이동 {out[0]:.0f}°/s·{out[1]:.0f}°/s² 로 (관성 힘 25 N 방지)")
    return out


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


def slot_axis(G, U) -> np.ndarray:
    """홈 축 단위 벡터 (G → U, base). 순수 함수."""
    d = np.array(U[:3], float) - np.array(G[:3], float)
    return d / np.linalg.norm(d)


def search_offsets(cfg) -> list[np.ndarray]:
    """꽂기 시도별 [dx, dy, dz mm, rz deg] (base): [shift, shift + pattern[0], ...]. 첫 시도는 고정 보정 shift 만.
    pattern 항목은 [dx, dy, dz] m 또는 [dx, dy, dz, rz] (m·deg). 순수 함수."""
    def vec(d, rz=0.0):
        d = [float(v) for v in d]
        return np.array([d[0] * 1000.0, d[1] * 1000.0, d[2] * 1000.0, d[3] if len(d) > 3 else rz])
    shift = vec(cfg["pad_insert_shift_m"])
    return [shift] + [shift + vec(d) for d in (cfg["pad_insert_search_pattern_m"] or [])]


def offset_text(o) -> str:
    """로그용: '옆 [dx, dy, dz] mm · 회전 rz°'. 순수 함수."""
    t = f"옆 {np.round(np.asarray(o[:3], float), 1).tolist()} mm"
    return t + (f" · 회전 {float(o[3]):+.1f}°" if len(o) > 3 else "")


def lift_height_mm(gap_mm, clear_mm, backoff_mm) -> float:
    """걸린 뒤 옆으로 옮기기 전에 올라갈 높이 (G 위 mm): 홈 밖(clear) 과 걸린 자리 + backoff 중 높은 쪽. 순수 함수."""
    return max(float(clear_mm), float(gap_mm) + float(backoff_mm))


def shifted(pose, off) -> list[float]:
    """posx(mm·deg, ZYZ) 위치를 off[:3] mm 만큼 옮기고, off[3] 이 있으면 TCP 를 base z 축으로 그만큼(deg) 돌림.
    ZYZ (A, B, C) 에서 base z 회전 Rz(r)·Rz(A)Ry(B)Rz(C) = Rz(A + r)Ry(B)Rz(C) → A 에 더하면 된다. 순수 함수."""
    out = [float(pose[i] + off[i]) for i in range(3)] + [float(v) for v in pose[3:6]]
    if len(off) > 3:
        out[3] += float(off[3])
    return out


def above_seat(cur, G, U) -> float:
    """지금 자리가 G 보다 홈 축(G → U) 방향으로 몇 mm 위인지 (아래면 음수). 순수 함수."""
    g, d = np.array(G[:3], float), np.array(U[:3], float) - np.array(G[:3], float)
    return float((np.array(cur[:3], float) - g) @ (d / np.linalg.norm(d)))


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
    from DSR_ROBOT2 import get_current_posj
    ready = _ready_posj(node)
    if ready is not None and _near(get_current_posj(), ready):
        # 10/13 실로봇: 대기 자세 → A 한 번에 관절 이동하면 TCP 가 y −0.675 까지 휘어 workspace_limit STOP (순기구학 여유 −7.5 cm).
        # 꽂기(place_pad)처럼 손목 먼저 A 값으로(팔은 그대로) → 팔을 A 로: 여유 +9.1 cm
        via = [float(v) for v in ready[:3]] + [float(v) for v in cfg["pad_rack_approach_posj"][3:6]]
        check()
        movej_checked(node, via, jv, ja)
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
    P = shifted(P, search_offsets(cfg)[0])                   # 고정 옆 보정(pad_insert_shift_m)은 입구 위부터
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
    hits, fz0, peak, mag_hit = [0], {}, [0.0], [0.0]
    S, _ = insert_points(pts["G"], pts["U"], cfg["pad_insert_slow_from_m"], cfg["pad_insert_below_m"])   # 감속 시작점 (G 위 slow_from)

    def seated():                                            # 연속 3 샘플 넘어야 (이동 중 힘 튐 무시)
        # 10/12 밤 실로봇: 홈 입구에 걸리면 옆 방향 힘이 먼저 커져 Z 변화 6 N 전에 합력 25 N(force_limit STOP) → 세 축 힘 변화로 본다
        f = get_tool_force()
        dev = float(np.linalg.norm([f[i] - fz0["f"][i] for i in range(3)]))
        peak[0] = max(peak[0], dev)
        mag = float(np.linalg.norm([float(v) for v in f[:3]]))
        if mag >= cfg["pad_insert_abs_stop_n"]:              # 영점과 상관없이 절대 합력 — 안전 한계보다 먼저 (10/14)
            mag_hit[0] = mag
            return True
        hits[0] = hits[0] + 1 if dev >= cfg["pad_insert_contact_n"] else 0
        return hits[0] >= 3

    def descend(o):
        """옆 이동 o 의 홈 축 따라 순응 하강: G 위 slow_from 까지는 보통 속도, 그 아래는 slow_vel. 힘으로 멈추면 'interrupted'."""
        cur0 = [float(v) for v in get_current_posx()[0]]
        how = "done"
        if above_seat(cur0, pts["G"], pts["U"]) > cfg["pad_insert_slow_from_m"] * 1000.0 + 1.0:
            how = move_checked(node, shifted(S, o), min(lv, 20.0), ca, on_poll=seated, stall_ok=True)
        if how != "interrupted":
            how = move_checked(node, shifted(B, o), min(lv, cfg["pad_insert_slow_vel_mm_s"]), ca, on_poll=seated, stall_ok=True)
        return how

    offsets = search_offsets(cfg)
    try:
        with compliance(cfg["pad_insert_stiffness"], log):
            gap = 0.0                                        # 걸린 높이 (G 위 mm) — 다시 시도 때 올라갈 높이 계산
            for k, o in enumerate(offsets, 1):
                if k > 1:
                    # 걸림: 홈 축 따라 홈 밖(clear)까지 올라가 → 옆으로 다음 자리 → 다시 순응 하강. 10/14: 홈 안에서 세게 다시 밀기는 1~2 mm 만 더 가고 합력만 23 N 까지 쌓여 뺌
                    h = lift_height_mm(gap, cfg["pad_insert_clear_m"] * 1000.0, cfg["pad_insert_backoff_m"] * 1000.0) / 1000.0
                    C, _ = insert_points(pts["G"], pts["U"], h, cfg["pad_insert_below_m"])
                    check()
                    move_checked(node, shifted(C, offsets[k - 2]), min(lv, 20.0), ca, stall_ok=True)
                    move_checked(node, shifted(C, o), min(lv, 20.0), ca, stall_ok=True)
                    time.sleep(0.3)                          # 흔들림 가라앉힌 뒤 영점
                    hits[0], peak[0] = 0, 0.0
                    log.info(f"[pad_rack] {k - 1}번째 다시 시도 — {offset_text(o)} 로 순응 하강 (G 위 {h * 1000.0:.0f} mm 에서 옮김)")
                fz0["f"] = [float(v) for v in get_tool_force()]   # 순응을 켠 뒤의 영점
                mag_hit[0] = 0.0
                check()
                try:
                    how = descend(o)
                finally:
                    try:                                     # STOP 예외 경로에서 이게 실패해도 원래 예외(SkillAbort)를 가리지 않게 (#170)
                        cur = [float(v) for v in get_current_posx()[0]]
                    except Exception as e:  # noqa: BLE001
                        log.warning(f"[pad_rack] 하강 끝 위치 읽기 실패: {e}")
                        cur = [float("nan")] * 6
                    log.info(f"[pad_rack] 하강 끝 TCP [{', '.join(f'{v:.0f}' for v in cur[:3])}] (입구 위 {[round(v) for v in P[:3]]} → G {[round(v) for v in pts['G'][:3]]})"
                             f" · 힘 변화 최대 {peak[0]:.1f} N · 영점 합력 {float(np.linalg.norm(fz0['f'][:3])):.1f} N"
                             + (f" · 합력 {mag_hit[0]:.1f} N 으로 멈춤" if mag_hit[0] else ""))
                gap = above_seat(cur, pts["G"], pts["U"])
                if gap <= cfg["pad_insert_max_gap_m"] * 1000.0:
                    break
                log.warning(f"[pad_rack] G 위 {gap:.0f} mm 에서 걸림 ({offset_text(o)})")
            else:
                log.warning(f"[pad_rack] {len(offsets)}번 다 걸림 — 패드를 쥔 채 입구 위로 빠지고 중단")
                move_checked(node, shifted(P, offsets[-1] - offsets[0]), lv, ca, stall_ok=True)   # P 에는 이미 offsets[0] 이 들어 있음
                raise SkillAbort(f"pad_insert_blocked ({gap:.0f} mm)")
            off = float(np.linalg.norm(np.array(cur[:3]) - np.array(pts["G"][:3])))
            log.info(f"[pad_rack] 꽂힘 ({how}) — G 와 {off:.1f} mm (G 위 {gap:.1f} mm) · {offset_text(o)} ({k}번째)")
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
