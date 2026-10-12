"""rally_robot/skills/catch_plan.py — 모드 3 받기 계산 (ROS·DSR 없음, 단위 테스트용)
담당: 박성현

catch_ball.py 가 쓴다. 비전(rally_vision) 코드는 import 하지 않는다 — 비전과는 토픽(/rally/perception/landing, landing_info)으로만 연결.
좌표: base_link [m]. posx 는 두산 단위(mm, ZYZ deg).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

INFO_FIELDS = ("t_cross", "time_to_go", "std_u", "std_w", "n_obs", "since_release")   # landing_info data 순서 (INTERFACES 2.2)


@dataclass
class Prediction:
    point: np.ndarray        # 패드 면 교차 예측점 (base, m)
    t_cross: float           # 교차 시각 (ROS 시각, s)
    since_release: float     # 공 놓은 뒤 경과 (s)
    std: tuple[float, float]  # 평면 위 표준편차 u·w (m)
    n_obs: int


def parse_info(data) -> dict | None:
    """Float64MultiArray.data → dict. 길이가 모자라면 None."""
    if len(data) < len(INFO_FIELDS):
        return None
    return dict(zip(INFO_FIELDS, (float(v) for v in data[:len(INFO_FIELDS)])))


def pad_center(tcp_posx, pad_offset_tool_m) -> np.ndarray:
    """TCP posx(mm, ZYZ) + 공구 좌표 패드 오프셋(m) → 패드 중심 base [m]."""
    R = Rotation.from_euler("ZYZ", tcp_posx[3:6], degrees=True).as_matrix()
    return np.asarray(tcp_posx[:3], float) / 1000.0 + R @ np.asarray(pad_offset_tool_m, float)


def plane_axes(normal) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """패드 면 법선 → (n, u, w). u = 로봇 기준 오른쪽(−y) 수평, w = 면 안 나머지 축(투구자 쪽 아래)."""
    n = np.asarray(normal, float)
    n = n / np.linalg.norm(n)
    u = np.cross(n, [0.0, 0.0, 1.0])
    u /= np.linalg.norm(u)
    return n, u, np.cross(n, u)


@dataclass
class Intercept:
    tcp_posx: list[float]    # 목표 TCP posx (mm, ZYZ) — 자세는 대기 자세 그대로
    du: float                # 패드 면 안 이동 (m): 오른쪽 +
    dw: float                # 패드 면 안 이동 (m): 투구자 쪽 아래 +
    offset_m: float          # 대기 패드 중심 → 예측점 면 안 거리 (클리핑 전)
    clipped: bool


def intercept(ready_tcp_posx, pad_center_m, normal, predicted_m, right_max_m, updown_max_m,
              down_max_m: float | None = None) -> Intercept:
    """예측점으로 패드 중심을 옮기는 TCP 목표. TCP 자세 고정·위치만 이동 (10/11 결정).

    - 패드 면 안 두 축으로 분해해 면 수직 성분은 버린다(예측점은 이미 면 위).
    - 왼쪽(u < 0)으로는 가지 않는다: 왼쪽 공은 '환자 투구 실패'로 성공률에서 뺀다 (10/11 결정).
    - 오른쪽 right_max_m, 위 updown_max_m, 아래(+w, 투구자 쪽) down_max_m(없으면 updown_max_m) 로 자른다 (어깨 특이점·작업 공간 여유).
      아래는 대기 자세에서 약 9.4 cm 에 두산 자기충돌 정지 (10/11 실측) → skills.yaml mode3.down_max_m 0.08.
    """
    _, u, w = plane_axes(normal)
    d = np.asarray(predicted_m, float) - np.asarray(pad_center_m, float)
    du0, dw0 = float(d @ u), float(d @ w)
    du = min(max(du0, 0.0), right_max_m)
    dw = min(max(dw0, -updown_max_m), updown_max_m if down_max_m is None else down_max_m)
    move_mm = (du * u + dw * w) * 1000.0
    tcp = [float(ready_tcp_posx[i]) + float(move_mm[i]) for i in range(3)] + [float(v) for v in ready_tcp_posx[3:6]]
    return Intercept(tcp_posx=tcp, du=du, dw=dw, offset_m=float(np.hypot(du0, dw0)),
                     clipped=(abs(du - du0) > 1e-9 or abs(dw - dw0) > 1e-9))


def quality_ok(std: tuple[float, float], n_obs: int, std_max_m: float, n_obs_min: int) -> bool:
    """출발 시점 예측을 믿을 수 있나 (정정인 #128 후속 2). 표준편차 두 축 모두 std_max_m 안 · 관측 수 n_obs_min 이상."""
    return max(float(std[0]), float(std[1])) <= std_max_m and int(n_obs) >= n_obs_min


def clamp_cross(t_cross: float, t_ref: float, max_after_s: float) -> float:
    """이상한 교차 시각(예: 10 s 뒤)으로 그 자리에서 오래 기다리지 않게 t_ref + max_after_s 로 자른다 (정정인 #128 후속 1)."""
    return min(float(t_cross), float(t_ref) + float(max_after_s))


def hold_run(times, gap_s: float) -> tuple[float, int]:
    """패드 위 공 관측 시각 목록(오름차순)에서 **마지막 관측으로 끝나는 연속 구간**(관측 사이 틈 ≤ gap_s) 의 (길이 s, 관측 수).
    비전 판정용: 측면 카메라가 패드 위 공을 몇 초 연속 봤나. 관측이 없으면 (0, 0)."""
    if not len(times):
        return 0.0, 0
    j = len(times) - 1
    while j > 0 and times[j] - times[j - 1] <= gap_s:
        j -= 1
    return float(times[-1] - times[j]), len(times) - j


def pad_caught(times, now: float, hold_s: float, gap_s: float) -> bool:
    """now 기준으로 '잡음': 마지막 관측이 gap_s 안이고 그 연속 구간이 hold_s 이상."""
    if not len(times) or now - times[-1] > gap_s:
        return False
    return hold_run(times, gap_s)[0] >= hold_s
