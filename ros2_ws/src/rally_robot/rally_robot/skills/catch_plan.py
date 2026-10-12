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


def intercept(ready_tcp_posx, pad_center_m, normal, predicted_m, right_max_m, updown_max_m) -> Intercept:
    """예측점으로 패드 중심을 옮기는 TCP 목표. TCP 자세 고정·위치만 이동 (10/11 결정).

    - 패드 면 안 두 축으로 분해해 면 수직 성분은 버린다(예측점은 이미 면 위).
    - 왼쪽(u < 0)으로는 가지 않는다: 왼쪽 공은 '환자 투구 실패'로 성공률에서 뺀다 (10/11 결정).
    - 오른쪽 right_max_m, 위아래 ±updown_max_m 로 자른다 (어깨 특이점·작업 공간 여유).
    """
    _, u, w = plane_axes(normal)
    d = np.asarray(predicted_m, float) - np.asarray(pad_center_m, float)
    du0, dw0 = float(d @ u), float(d @ w)
    du = min(max(du0, 0.0), right_max_m)
    dw = min(max(dw0, -updown_max_m), updown_max_m)
    move_mm = (du * u + dw * w) * 1000.0
    tcp = [float(ready_tcp_posx[i]) + float(move_mm[i]) for i in range(3)] + [float(v) for v in ready_tcp_posx[3:6]]
    return Intercept(tcp_posx=tcp, du=du, dw=dw, offset_m=float(np.hypot(du0, dw0)),
                     clipped=(abs(du - du0) > 1e-9 or abs(dw - dw0) > 1e-9))


def impact_detected(samples, baseline_n: float, t_from: float, t_to: float, threshold_n: float) -> tuple[bool, float]:
    """힘 기록 [(t, |F|), ...] 에서 [t_from, t_to] 안에 |F| − baseline 이 threshold 를 넘는 샘플이 있는지 → (맞음, 최대 증가 N).

    10/11 정지 시험: 공이 맞을 때 |F| 4.9~15.3 N (정지 중 0.9 N). 툴 힘은 실제 10 Hz 갱신이라 충격 1~2 샘플.
    한계: 로봇이 감속하는 동안의 관성 힘도 커서, 도착 전 구간은 넘겨주지 말 것. 붙음/튕김은 힘으로 구분 못 함 → 비전 판정 예정.
    """
    peak = 0.0
    for t, f in samples:
        if t_from <= t <= t_to:
            peak = max(peak, float(f) - baseline_n)
    return peak >= threshold_n, peak
