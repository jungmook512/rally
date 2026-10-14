"""rally_robot/detach.py — 모드 3 거치대에 꽂힌 패드에서 벨크로 공 떼기: 순수 계산 (스킬 skills/detach_ball.py 가 부른다)
담당: 정정인 · 모드 3

왜 '벗기기(peel)'인가 (10/14 정정인): 벨크로는 똑바로 당기면 접촉면 전체가 한꺼번에 버텨 손으로도 힘이 많이 든다 → force_limit 25 N 에 걸린다.
가장자리부터 한 줄씩 벗기면 힘이 훨씬 작다. 공을 쥔 채 공 아래 접촉점을 중심으로 돌려 **위부터** 벗긴다 — 반작용이 패드를 거치대 홈
아래로 누르는 쪽이라 패드가 같이 뽑히지 않는다 (똑바로 당기거나 아래부터 벗기면 패드가 올라온다).

기하 (base 좌표)
  n  패드 면 법선 (공이 붙은 쪽, 거치대에 꽂힌 패드는 +x — 10/14 정정인)
  c  공 중심 (비전, base). 접촉점 ≈ c − r·n. 회전 중심(pivot) 은 접촉점에서 아래로 pivot_down_r × r.
  회전축 a = ẑ × n (수평, 패드 면 안). a 축으로 +θ 돌리면 pivot 위쪽(공 윗부분)이 +n 쪽(패드에서 떨어지는 쪽)으로 간다.
  TCP 는 공과 한 몸 → TCP 위치도 pivot 중심으로 같이 돌린다 (p' = pivot + R(p − pivot)), 자세는 R·R0. 스텝마다 n 방향으로 조금 더 후퇴.
  손가락 축은 회전축과 직교해야 한다 — RG2 두 손가락이 집은 두 점을 잇는 축으로는 공이 헛돈다(볼 베어링). 손가락 축 = ẑ(세로) 로 집는다:
  툴 z = −n (패드를 정면으로), 툴 finger_axis 가 base ẑ 를 향하게.
힘 판정 (get_tool_force 10 Hz)
  합력이 limit_n 넘으면 그 스텝에서 멈춤 (스킬 자체 한계 — safety force_limit 25 N 보다 작게). 합력 최대값(peak) 뒤 drop_n 이상 급락하면
  떼어짐. 순응은 이동(x·y·z)만 무르게 — 회전 중심 오차를 흡수. 회전은 단단히 — 무르면 벗기는 토크가 흡수돼 안 떼어진다.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation

from rally_robot.dsr_helpers import SkillAbort

DEFAULTS = dict(
    ball_view_posj=[],                      # ★ 교시 필요: 손목 카메라가 거치대에 꽂힌 패드 면(공 붙은 쪽)을 보는 관절값
    box_drop_posj=[],                       # ★ 교시 필요: 뗀 공을 놓을 박스 위 관절값
    pad_normal=[1.0, 0.0, 0.0],             # 거치대에 꽂힌 패드의 공 붙은 면 법선 (base). 10/14 정정인: 로봇 반대쪽 +x
    ball_radius_m=0.031,                    # 벨크로 공 지름 62 mm (10/14 정정인)
    ball_class="velcro_ball",               # /rally/perception/objects 에서 고를 클래스
    ball_min_conf=0.4,
    ball_obs_s=1.0,                         # 이 시간 동안 본 위치의 중앙값
    ball_obs_min=5,                         # 최소 관측 프레임 수
    ball_search_m=0.25,                     # 패드 중심(거치대 G 기준)에서 이 거리 안의 검출만 (다른 공 제외). 0 = 안 가림
    approach_m=0.08,                        # 공 앞(n 방향) 이만큼에서 그리퍼 열고 시작
    grasp_depth_m=0.0,                      # 쥘 때 TCP 가 공 중심에서 n 방향으로 몇 m 앞 (0 = 손가락 중심이 공 중심. 손가락 길이 보정은 실측 뒤)
    finger_axis_tool=[1.0, 0.0, 0.0],       # RG2 손가락이 닫히는 툴 축 (10/14 추정 — 실로봇 확인)
    grip_tol_mm=10.0,                       # 쥔 폭이 공 지름 ± 이 밖이면 잘못 쥔 것
    pivot_down_r=0.5,                       # 회전 중심 = 접촉점에서 아래로 r × 이 값
    peel_step_deg=5.0,                      # 한 스텝 회전각
    peel_max_deg=40.0,                      # 이 각까지 급락 없으면 n 방향 pull_extra_m 당겨 보고 판정
    peel_retreat_per_step_m=0.002,          # 스텝마다 n 방향으로 더 후퇴
    pull_extra_m=0.03,
    peel_vel_mm_s=15.0,
    limit_n=15.0,                           # 스킬 자체 힘 한계 (goal.pull_force_limit_n 이 0 이면). safety 25 N 아래
    drop_n=5.0,                             # peak 뒤 이만큼 떨어지면 떼어짐
    min_peak_n=4.0,                         # 급락 판정은 peak 가 이 이상일 때만 (잡음 제외)
    stiffness=[300, 300, 300, 400, 400, 400],   # [x y z N/m, rx ry rz Nm/rad] — 이동 무르게, 회전 단단히 (DSR 회전 최대 400)
    retreat_m=0.05,                         # 떼어진 뒤 n 방향 후퇴
    place_release_s=0.3,
)
TEACH_KEYS = ("ball_view_posj", "box_drop_posj")


def config(node) -> dict:
    m3 = (getattr(node, "cfg", None) or {}).get("mode3") or {}
    cfg = {**DEFAULTS, **{k[len("detach_"):]: v for k, v in m3.items() if k.startswith("detach_") and k[len("detach_"):] in DEFAULTS}}
    for k in TEACH_KEYS:
        if not cfg[k] or len(cfg[k]) != 6:
            raise SkillAbort(f"no_detach_teach (skills.yaml mode3.detach_{k})")
    return cfg


def taught(node) -> bool:
    m3 = (getattr(node, "cfg", None) or {}).get("mode3") or {}
    return all(len(m3.get(f"detach_{k}") or []) == 6 for k in TEACH_KEYS)


def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        raise SkillAbort("detach_bad_normal")
    return v / n


def peel_axis(normal) -> np.ndarray:
    """회전축 a = ẑ × n (수평, 패드 면 안). n 이 수직이면 패드가 누운 것 — 이 스킬 범위 밖."""
    n = _unit(normal)
    a = np.cross([0.0, 0.0, 1.0], n)
    if np.linalg.norm(a) < 0.1:
        raise SkillAbort("detach_pad_horizontal")
    return a / np.linalg.norm(a)


def ball_median(observations, min_n: int):
    """[(x, y, z), ...] 중앙값 [m]. 관측이 min_n 보다 적으면 None. 순수 함수."""
    if len(observations) < min_n:
        return None
    return [float(v) for v in np.median(np.asarray(observations, float), axis=0)]


def pick_ball(objects, cls: str, min_conf: float, near_m=None, radius_m: float = 0.0):
    """TrackedObjectArray 한 프레임에서 공 하나 고르기 → (x, y, z) [m] 또는 None. near_m 에서 radius_m 안의 것 중 신뢰도 최고. 순수 함수."""
    best = None
    for o in objects:
        if o.class_name != cls or o.confidence < min_conf:
            continue
        p = (float(o.position.x), float(o.position.y), float(o.position.z))
        if near_m is not None and radius_m > 0 and math.dist(p, near_m) > radius_m:
            continue
        if best is None or o.confidence > best[0]:
            best = (float(o.confidence), p)
    return best[1] if best else None


def grasp_orientation(normal, finger_axis_tool) -> list[float]:
    """툴 z = −n (패드 정면), 손가락 축(툴) 이 base ẑ 를 향하게 하는 ZYZ [deg]. 순수 함수.
    손가락 축이 회전축(ẑ × n)과 직교 → 공이 손가락 사이에서 헛돌지 않는다."""
    n = _unit(normal)
    z_t = -n
    f = _unit(finger_axis_tool)
    up = np.array([0.0, 0.0, 1.0])
    up = up - z_t * (up @ z_t)                   # 툴 z 에 수직인 '위'
    up /= np.linalg.norm(up)
    # 툴 축 중 손가락 축을 up 에 맞춘다. 손가락 축이 x 면 R[:,0]=up, y 면 R[:,1]=up (z 는 z_t, 나머지는 오른손 법칙)
    if abs(f[0]) >= abs(f[1]):
        x_t = up if f[0] > 0 else -up
        y_t = np.cross(z_t, x_t)
    else:
        y_t = up if f[1] > 0 else -up
        x_t = np.cross(y_t, z_t)
    R = np.column_stack([x_t, y_t, z_t])
    return [float(v) for v in Rotation.from_matrix(R).as_euler("ZYZ", degrees=True)]


def grasp_pose(ball_m, normal, grasp_depth_m, orient) -> list[float]:
    """쥐는 자세 posx [mm·deg]: TCP = 공 중심 + n·grasp_depth. 순수 함수."""
    c, n = np.asarray(ball_m, float), _unit(normal)
    p = (c + n * grasp_depth_m) * 1000.0
    return [float(v) for v in p] + list(orient)


def along(pose6, normal, dist_m) -> list[float]:
    """pose6 에서 n 방향으로 dist_m 옮긴 posx (자세 그대로). 순수 함수."""
    n = _unit(normal)
    return [float(pose6[i] + n[i] * dist_m * 1000.0) for i in range(3)] + [float(v) for v in pose6[3:6]]


def peel_pose(grasp6, ball_m, normal, r_m, pivot_down_r, deg, retreat_m) -> list[float]:
    """회전 중심(pivot)으로 deg 만큼 돌린 TCP posx + n 방향 retreat_m. 순수 함수.
    pivot = 접촉점(c − r·n) − ẑ·r·pivot_down_r. 회전축 ẑ × n, +deg → 공 윗부분이 +n 쪽."""
    n = _unit(normal)
    a = peel_axis(n)
    c = np.asarray(ball_m, float) * 1000.0
    pivot = c - n * r_m * 1000.0 - np.array([0.0, 0.0, 1.0]) * r_m * pivot_down_r * 1000.0
    R = Rotation.from_rotvec(np.deg2rad(deg) * a)
    p0 = np.asarray(grasp6[:3], float)
    p = pivot + R.apply(p0 - pivot) + n * retreat_m * 1000.0
    rot = R * Rotation.from_euler("ZYZ", grasp6[3:6], degrees=True)
    return [float(v) for v in p] + [float(v) for v in rot.as_euler("ZYZ", degrees=True)]


def peel_schedule(step_deg, max_deg, retreat_per_step_m) -> list[tuple[float, float]]:
    """[(각도 deg, 누적 후퇴 m), ...] 스텝 목록. 순수 함수."""
    n = max(1, int(round(max_deg / step_deg)))
    return [(step_deg * k, retreat_per_step_m * k) for k in range(1, n + 1)]


class PeelWatch:
    """툴 힘 샘플로 '한계 초과' · '떼어짐(급락)' 판정. 순수 상태 기계."""

    def __init__(self, limit_n: float, drop_n: float, min_peak_n: float):
        self.limit_n, self.drop_n, self.min_peak_n = float(limit_n), float(drop_n), float(min_peak_n)
        self.peak = 0.0
        self.last = 0.0
        self.over = False
        self.detached = False

    def feed(self, f3) -> str:
        """→ 'over' | 'detached' | ''"""
        mag = float(np.linalg.norm(np.asarray(f3, float)[:3]))
        self.last = mag
        if mag >= self.limit_n:
            self.over = True
            return "over"
        if self.peak >= self.min_peak_n and self.peak - mag >= self.drop_n:
            self.detached = True
            return "detached"
        self.peak = max(self.peak, mag)
        return ""

    @property
    def drop(self) -> float:
        return max(0.0, self.peak - self.last)


def grip_ok(ok, width_mm, detected, ball_radius_m, tol_mm) -> bool:
    return bool(ok) and bool(detected) and abs(float(width_mm) - ball_radius_m * 2000.0) <= tol_mm
