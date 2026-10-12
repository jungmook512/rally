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


def contact_window_closed(now_ros: float, t_cross_ros: float, after_s: float) -> bool:
    """받기 창(contact_expected, 힘 한계 130 N 자리)을 닫을 때인가: 교차 시각 + after_s 가 지나면 참 (#144 조건 "창 약 1 s")."""
    return float(now_ros) >= float(t_cross_ros) + float(after_s)


def judge_pad(pad_obs, now_mono, now_ros, sleep, tick, t_from_mono: float, t_cross_ros: float, cfg: dict,
              contact_close=None) -> tuple[bool, float, list[float]]:
    """비전 판정 루프 (ROS·로봇 없이 돌릴 수 있게 시계·관측·콜백을 주입).
    pad_obs(t_from_mono) → (관측 시각 목록 오름차순(monotonic), 위치 목록) · now_mono()/now_ros() 시계 · sleep(s) · tick() 은 매 바퀴 (check_stop 자리).
    contact_close() 는 받기 창이 닫히는 시각(교차 + contact_window_after_cross_s)에 **한 번** 부른다 — 판정이 끝나기 전에도 끈다.
    판정: t_from_mono 뒤 관측이 pad_hold_s 연속(틈 ≤ pad_gap_s)이면 caught. 교차 + judge_timeout_s 안에 못 채우면 missed.
    → (caught, 연속 관측 시간 s, 패드 위 공 위치 평균 또는 nan)"""
    deadline = max(t_cross_ros, now_ros()) + cfg["judge_timeout_s"]
    t_close = t_cross_ros + cfg["contact_window_after_cross_s"]
    period, best, closed = 1.0 / cfg["judge_poll_hz"], 0.0, contact_close is None
    nan3 = [float("nan")] * 3
    try:
        while True:
            tick()
            if not closed and contact_window_closed(now_ros(), t_cross_ros, cfg["contact_window_after_cross_s"]):
                contact_close()
                closed = True
            now = now_mono()
            times, pts = pad_obs(t_from_mono)
            run_s, n = hold_run(times, cfg["pad_gap_s"])
            best = max(best, run_s)
            if pad_caught(times, now, cfg["pad_hold_s"], cfg["pad_gap_s"]):
                return True, run_s, np.mean(np.array(pts[-n:]), axis=0).tolist()
            if now_ros() > deadline:
                actual = np.mean(np.array(pts[-n:]), axis=0).tolist() if n else nan3
                return False, best, actual
            sleep(min(period, max(0.0, t_close - now_ros())) if not closed else period)
    finally:
        if not closed:
            contact_close()          # 중단(STOP·취소)으로 빠져나가도 창은 닫힌다 (catch_ball 의 finally 가 한 번 더 끈다)


def should_resend(planned_m, new_pred_m, since_release: float, n_sent: int, cfg: dict) -> bool:
    """비행 중 갱신(MODE3_PLAN ③): 출발 뒤 새 예측점이 처음 계획한 예측점에서 resend_threshold_m(3 cm) 넘게 벗어났고,
    놓은 뒤 resend_until_release_s 안이며, 아직 resend_max 번 안 보냈으면 amovel 을 한 번 더. resend_enabled 가 꺼져 있으면 항상 거짓.
    (실측 #130: 0.15 s 뒤 재전송은 첫 동작 위에 얹혀 속도가 명령값을 넘음 → 기본 꺼짐, 정정인 안전 확인 뒤 켬)"""
    if not cfg.get("resend_enabled", False) or n_sent >= int(cfg.get("resend_max", 1)):
        return False
    if since_release > float(cfg.get("resend_until_release_s", 0.30)):
        return False
    d = float(np.linalg.norm(np.asarray(new_pred_m, float) - np.asarray(planned_m, float)))
    return d > float(cfg.get("resend_threshold_m", 0.03))
