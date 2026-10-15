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
    ball_class=["velcro_ball"],             # ★ 10/15 정정인: 모드 3 은 벨크로 공 모델(velcro_ball, v2)만 — 테니스공(ball)으로 대신하지 않음
    ball_min_conf=0.4,
    ball_obs_s=1.0,                         # 이 시간 동안 본 위치의 중앙값
    ball_obs_min=5,                         # 최소 관측 프레임 수
    ball_search_m=0.25,                     # 패드 중심(거치대 G 기준)에서 이 거리 안의 검출만 (다른 공 제외). 0 = 안 가림
    approach_m=0.08,                        # 공 앞(n 방향) 이만큼에서 그리퍼 열고 시작
    grasp_depth_m=0.0,                      # 쥘 때 TCP 가 공 중심에서 n 방향으로 몇 m 앞 (0 = 손가락 중심이 공 중심. 손가락 길이 보정은 실측 뒤)
    finger_axis_tool=[1.0, 0.0, 0.0],       # RG2 손가락이 닫히는 툴 축 (10/14 추정 — 실로봇 확인)
    grip_tol_mm=10.0,                       # 쥔 폭이 공 지름 ± 이 밖이면 잘못 쥔 것
    pivot_down_r=0.5,                       # (옛 수평 쥐기) 회전 중심 = 접촉점에서 아래로 r × 이 값
    # ★ 10/14 위에서 쥐기 (정정인): 수평으로 패드 정면에서 쥐는 자세는 IK 8해 모두 J5 ±168° (한계 133) → 툴 수직 아래로 내려와
    #   손가락을 패드 면과 나란히(옆, base y) 닫고, 수직축(z)으로 돌려 한쪽 옆부터 벗긴다
    top_down_zyz=[174.8, -177.0, 175.1],    # 툴 수직 아래 자세 (skills.yaml pick.tool_down_zyz, 손가락 닫힘 = base x)
    finger_rot_deg=90.0,                    # 위 자세를 base z 로 이만큼 돌려 손가락을 패드 면과 나란히 (90 → 닫힘 = base y). IK 확인 ±90 둘 다 해 4개
    grasp_below_m=0.010,                    # 쥘 때 TCP(손가락 끝)를 공 중심보다 이만큼 아래 — 손가락 패드가 공 가장 넓은 곳에
    ball_z_m=0.0,                           # 0 이 아니면 공 중심 높이를 이 값으로 (비전 z 대신). 10/14 실측 95 mm, 비전은 119 mm (+24 mm, 알려진 깊이 오차)
    pivot_side_r=0.5,                       # 수직축 회전 중심 = 접촉점에서 옆(−l)으로 r × 이 값 → 반대쪽(+l) 옆부터 벗겨짐
    lift_after_m=0.05,                      # 떼어지고 n 방향 후퇴 뒤 위로
    # ★ 10/14 실로봇: 위에서 쥐러 내려갈 때 공 보기 자세의 카메라 방향(+x 쪽)을 그대로 — 툴 아래 자세 두 후보(±90°) 중 지금 자세와 가까운 것
    keep_view_yaw=True,
    # ★ 10/14: 비전 점 = 카메라 쪽 공 표면(+ 깊이 오차) → 카메라(+x 위)에서 본 시선 방향으로 밀려 z +24 mm·x + 쪽. z 만 95 로 바꾸면 x 는
    #   카메라 쪽으로 남음 → 시선(카메라 → 비전 점)을 따라 ball_z_m 높이까지 밀어 x·y 도 같이 고침 (ray_correct)
    ray_correct=True,
    # ★ 10/15 실로봇: v2 비전 공 z 197 mm (실제 중심 ≤ 141) → 비전 높이에서 멈춰 헛쥠 (폭 28 mm). 비전 높이를 믿지 않고 닿을 때까지 내려감
    descend_extra_m=0.08,                   # 비전 쥐는 높이보다 이만큼 더 아래까지 (그 전에 닿으면 멈춤)
    contact_n=6.0,                          # 하강 중 툴 힘이 영점에서 이만큼 3 샘플 변하면 닿음 (pick_compliant 접촉 8 N 보다 조금 예민)
    descend_vel_mm_s=15.0,
    grasp_tries=3,                          # ★ 10/15 정정인: 잘못 쥐면 열고 올라가 공 보기부터 다시, 이 횟수까지
    ball_offset_m=[0.0, 0.0, 0.0],          # 위 보정 뒤 남는 고정 오차 [m, base] — 실로봇에서 쥐는 위치 보고 맞춤
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
    # ★ 10/14 실로봇: 거치대 옆(x 0.26)에서 공 보기 자세(J1 159°)로 관절 이동하면 경로 TCP 가 x 0.58 까지 휘어 workspace_limit STOP.
    #   관절 이동 전에 fkin 으로 경로를 재서 상자 밖이면 손목 먼저/팔 먼저/J1 먼저 두 단계 경로 중 안에 드는 것으로 (plan_joint_path)
    path_x_max_m=0.54,                      # 평소 관절 경로 TCP x 상한 (safety workspace_x_max 0.57 − 3 cm, observe_pose.route_x_max_m 과 같음)
    path_x_max_box_m=0.63,                  # 박스 구간 (workspace_x_max_extended_m 0.65 − 2 cm)
    path_y_abs_max_m=0.58,                  # |y| 상한 (safety ±0.60 − 2 cm)
    path_z_min_m=0.03,
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


def ball_classes(cls) -> tuple:
    return (cls,) if isinstance(cls, str) else tuple(cls)


def pick_ball(objects, cls, min_conf: float, near_m=None, radius_m: float = 0.0):
    """TrackedObjectArray 한 프레임에서 공 하나 고르기 → (x, y, z) [m] 또는 None. cls 는 이름 하나 또는 목록.
    near_m 에서 radius_m 안의 것 중 신뢰도 최고. 순수 함수."""
    for c in ball_classes(cls):                       # 목록 앞 클래스 우선 (10/14: velcro_ball(모드 3 v2 모델) → 없을 때만 ball)
        best = None
        for o in objects:
            if o.class_name != c or o.confidence < min_conf:
                continue
            p = (float(o.position.x), float(o.position.y), float(o.position.z))
            if near_m is not None and radius_m > 0 and math.dist(p, near_m) > radius_m:
                continue
            if best is None or o.confidence > best[0]:
                best = (float(o.confidence), p)
        if best:
            return best[1]
    return None


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


def joint_path_extent(q0, q1, fk, samples=20):
    """관절 직선 보간(movej 경로) 위 TCP 의 (최소 xyz, 최대 xyz) mm. fk(posj deg) → posx. fk 실패면 None. 순수 함수."""
    pts = []
    for i in range(samples + 1):
        t = i / samples
        p = fk([a + (b - a) * t for a, b in zip(q0, q1)])
        if p is None or isinstance(p, (int, float)) or len(p) < 3:
            return None
        pts.append([float(v) for v in p[:3]])
    a = np.asarray(pts)
    return a.min(axis=0).tolist(), a.max(axis=0).tolist()


def path_inside(q0, q1, fk, x_max_mm, y_abs_max_mm, z_min_mm) -> bool | None:
    """movej 경로 TCP 가 x ≤ x_max · |y| ≤ y_abs_max · z ≥ z_min 안인가. fk 실패면 None (모름)."""
    ext = joint_path_extent(q0, q1, fk)
    if ext is None:
        return None
    lo, hi = ext
    return hi[0] <= x_max_mm and max(abs(lo[1]), abs(hi[1])) <= y_abs_max_mm and lo[2] >= z_min_mm


def plan_joint_path(q0, q1, fk, x_max_mm, y_abs_max_mm, z_min_mm, vias=()):
    """q0 → q1 관절 이동 경로 계획: 바로 가는 경로가 상자 안이면 [q1], 아니면 두 단계 후보(손목 먼저 · 팔 먼저 · J1 먼저 · J1+손목 먼저)
    중 두 구간 다 안에 드는 첫 것 [mid, q1], 그래도 없으면 vias(경유 자세 목록들, 예: [[대기 자세], [관측 자세, 대기 자세]]) 중
    모든 구간이 안에 드는 첫 것 [*via, q1]. 없으면 None. fk 를 못 쓰면 [q1] (확인 없이). 순수 함수.
    10/14 실로봇: 홈 자세(J1 0°) → 공 보기(J1 159°) 는 바로·두 단계 모두 x 0.56~0.71 로 휨, 관측 → 대기 → 공 보기 는 x ≤ 0.49."""
    q0, q1 = [float(v) for v in q0], [float(v) for v in q1]
    ok = path_inside(q0, q1, fk, x_max_mm, y_abs_max_mm, z_min_mm)
    if ok is None or ok:
        return [q1]
    cands = [
        ("손목 먼저", q0[:3] + q1[3:6]),
        ("팔 먼저", q1[:3] + q0[3:6]),
        ("J1 먼저", [q1[0]] + q0[1:6]),
        ("J1+손목 먼저", [q1[0]] + q0[1:3] + q1[3:6]),
    ]
    for name, mid in cands:
        if path_inside(q0, mid, fk, x_max_mm, y_abs_max_mm, z_min_mm) and path_inside(mid, q1, fk, x_max_mm, y_abs_max_mm, z_min_mm):
            return [mid, q1]
    for seq in vias:
        pts = [q0] + [[float(v) for v in w] for w in seq] + [q1]
        if all(path_inside(a, b, fk, x_max_mm, y_abs_max_mm, z_min_mm) for a, b in zip(pts, pts[1:])):
            return pts[1:]
    return None


def lateral(normal) -> np.ndarray:
    """패드 면 안 수평 방향 l = ẑ × n (n = +x 면 +y). 위에서 쥘 때 손가락 닫힘 방향."""
    return peel_axis(normal)


def top_grasp_pose(ball_m, below_m, zyz_down, finger_rot_deg) -> list[float]:
    """위에서 쥐는 자세 posx [mm·deg]: TCP = 공 중심 바로 위 축에서 below_m 아래, 툴 수직 아래를 base z 로 finger_rot_deg 돌림. 순수 함수."""
    c = np.asarray(ball_m, float) * 1000.0
    rot = Rotation.from_euler("z", finger_rot_deg, degrees=True) * Rotation.from_euler("ZYZ", zyz_down, degrees=True)
    return [float(c[0]), float(c[1]), float(c[2] - below_m * 1000.0)] + [float(v) for v in rot.as_euler("ZYZ", degrees=True)]


def up(pose6, dist_m) -> list[float]:
    """pose6 를 base z 로 dist_m 올린 posx. 순수 함수."""
    return [float(pose6[0]), float(pose6[1]), float(pose6[2] + dist_m * 1000.0)] + [float(v) for v in pose6[3:6]]


def peel_pose_vertical(grasp6, ball_m, normal, r_m, pivot_side_r, deg, retreat_m) -> list[float]:
    """수직축(base z) 벗기기: 회전 중심 pivot = 접촉점(c − r·n) − l·r·pivot_side_r (l = ẑ × n). +deg 면 반대쪽(+l) 옆이 먼저 +n 쪽
    (패드에서 떨어지는 쪽)으로 — pivot 쪽 옆은 붙은 채 경첩. TCP 는 공과 한 몸으로 같이 돌고, n 방향 retreat_m 더. 순수 함수."""
    n = _unit(normal)
    l = lateral(n)
    c = np.asarray(ball_m, float) * 1000.0
    pivot = c - n * r_m * 1000.0 - l * r_m * pivot_side_r * 1000.0
    R = Rotation.from_rotvec(np.deg2rad(-deg) * np.array([0.0, 0.0, 1.0]) * float(np.sign(np.cross(n, l)[2]) or 1.0))
    p0 = np.asarray(grasp6[:3], float)
    p = pivot + R.apply(p0 - pivot) + n * retreat_m * 1000.0
    rot = R * Rotation.from_euler("ZYZ", grasp6[3:6], degrees=True)
    return [float(v) for v in p] + [float(v) for v in rot.as_euler("ZYZ", degrees=True)]


def slide_along_ray(meas_m, cam_m, z_true_m):
    """카메라 → 비전 점 시선을 따라 높이가 z_true_m 인 점까지 민 위치 [m]. 시선이 거의 수평이면 그대로. 순수 함수.
    비전 점은 카메라 쪽 공 표면(깊이 중앙값)이라 시선 방향으로 카메라 쪽에 치우친다 — 실측 높이를 알면 같은 시선 위에서 고칠 수 있다."""
    m, c = np.asarray(meas_m, float), np.asarray(cam_m, float)
    d = m - c
    if abs(d[2]) < 1e-6 * max(1.0, float(np.linalg.norm(d))):
        return [float(m[0]), float(m[1]), float(z_true_m)]
    t = (float(z_true_m) - m[2]) / d[2]
    p = m + t * d
    return [float(v) for v in p]


def camera_pos_m(tcp6_mm, xyz_tcp_cam_m) -> list[float]:
    """TCP posx(mm·ZYZ) 와 TCP ← 카메라 위치(m, hand_eye.yaml tcp_T_camera_optical.xyz_m) → 카메라 위치 base [m]. 순수 함수."""
    R = Rotation.from_euler("ZYZ", tcp6_mm[3:6], degrees=True)
    p = np.asarray(tcp6_mm[:3], float) / 1000.0 + R.apply(np.asarray(xyz_tcp_cam_m, float))
    return [float(v) for v in p]


def top_grasp_pose_like(ball_m, below_m, zyz_down, cur_zyz):
    """위에서 쥐는 자세 중 툴 z 둘레 방향(손가락 ±90° 두 후보)을 지금 자세(cur_zyz)와 가까운 쪽으로 — 카메라가 돌지 않게. 순수 함수."""
    R0 = Rotation.from_euler("ZYZ", cur_zyz, degrees=True)
    best = None
    for rot in (90.0, -90.0):
        g = top_grasp_pose(ball_m, below_m, zyz_down, rot)
        ang = (Rotation.from_euler("ZYZ", g[3:6], degrees=True) * R0.inv()).magnitude()
        if best is None or ang < best[0]:
            best = (ang, g)
    return best[1]
