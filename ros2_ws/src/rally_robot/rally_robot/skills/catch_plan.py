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
              down_max_m: float | None = None, reach_max_m: float | None = None, overshoot_m: float = 0.0) -> Intercept:
    """예측점으로 패드 중심을 옮기는 TCP 목표. TCP 자세 고정·위치만 이동 (10/11 결정).

    - 패드 면 안 두 축으로 분해해 면 수직 성분은 버린다(예측점은 이미 면 위).
    - 왼쪽(u < 0)으로는 가지 않는다: 왼쪽 공은 '환자 투구 실패'로 성공률에서 뺀다 (10/11 결정).
    - 오른쪽 right_max_m, 위 updown_max_m, 아래(+w, 투구자 쪽) down_max_m(없으면 updown_max_m) 로 자른다 (어깨 특이점·작업 공간 여유).
      아래는 대기 자세에서 약 9.4 cm 에 두산 자기충돌 정지 (10/11 실측) → skills.yaml mode3.down_max_m 0.08.
    - overshoot_m: 10/13 정정인 — 목표를 이동 방향으로 이만큼 더 둔다. 로봇은 목표 앞에서 감속하므로 예측점에 정확히 서려 하면
      예측점 근처를 느리게 지나 공보다 늦는다. 목표를 뒤로 밀면 예측점은 속도가 붙은 채 지나간다 (450 mm/s·가속 2000 에서 마지막 5 cm:
      정지 0.22 s → 통과 0.11 s). 패드 지름 ~15 cm 라 5 cm 더 가 있어도 받는다. 한계(오른쪽·위·아래·reach)는 그 뒤에 다시 적용.
    - reach_max_m: 이동 거리(두 축 합) 상한 — 넘으면 방향은 그대로 두고 길이만 줄인다. 10/13 실로봇: 오른쪽 32 · 위 35 cm (합 47 cm)
      대각선 끝이 두산 NOT REACHABLE(1206) → not_started. 순기구학 계산으로 오른쪽~위 사이 모든 방향 40 cm 까지 도달 가능.
    """
    _, u, w = plane_axes(normal)
    d = np.asarray(predicted_m, float) - np.asarray(pad_center_m, float)
    du0, dw0 = float(d @ u), float(d @ w)
    du = min(max(du0, 0.0), right_max_m)
    dw = min(max(dw0, -updown_max_m), updown_max_m if down_max_m is None else down_max_m)
    r = float(np.hypot(du, dw))
    clipped = abs(du - du0) > 1e-9 or abs(dw - dw0) > 1e-9           # '잘림' 표시는 한계에 걸린 것만 (더 가기 전에 판정)
    if overshoot_m > 0 and r > 1e-3:                                     # 실제 이동 방향(잘린 뒤)으로 더 간 뒤 한계를 다시 적용
        du, dw = du + overshoot_m * du / r, dw + overshoot_m * dw / r
        du = min(max(du, 0.0), right_max_m)
        dw = min(max(dw, -updown_max_m), updown_max_m if down_max_m is None else down_max_m)
        r = float(np.hypot(du, dw))
    if reach_max_m is not None and r > reach_max_m > 0:
        du, dw = du * reach_max_m / r, dw * reach_max_m / r
        clipped = True
    move_mm = (du * u + dw * w) * 1000.0
    tcp = [float(ready_tcp_posx[i]) + float(move_mm[i]) for i in range(3)] + [float(v) for v in ready_tcp_posx[3:6]]
    return Intercept(tcp_posx=tcp, du=du, dw=dw, offset_m=float(np.hypot(du0, dw0)),
                     clipped=clipped)


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
    if since_release > float(cfg.get("resend_until_release_s", 0.45)):
        return False
    d = float(np.linalg.norm(np.asarray(new_pred_m, float) - np.asarray(planned_m, float)))
    return d > float(cfg.get("resend_threshold_m", 0.03))


class ContactWindow:
    """받기 창(contact_expected · catch_window 130 N)을 **이동과 무관한 타이머**로 닫는다 (10/12 밤 실측 #163 결함: 닫는 코드가
    판정 루프 안에 있어 요격 이동(0.5~0.8 s)이 끝나야 돌았고, 창이 1.3~2.4 s 로 #144 조건 "약 1 s" 를 넘었다).
    on_close 는 한 번만 불린다 — 타이머·판정 루프·finally 가 각각 불러도 처음 것만 실행."""

    def __init__(self, on_close, timer_factory=None):
        import threading
        self._on_close, self._timer_factory = on_close, (timer_factory or threading.Timer)
        self._lock = threading.Lock()
        self._timer = None
        self.closed = False
        self.closed_by: str | None = None

    def schedule_close(self, delay_s: float) -> None:
        """delay_s 뒤(0 이하면 바로)에 닫는다. 다시 부르면 이전 타이머를 취소하고 새로 잰다."""
        with self._lock:
            if self.closed:
                return
            if self._timer is not None:
                self._timer.cancel()
            self._timer = self._timer_factory(max(0.0, float(delay_s)), lambda: self.close("timer"))
            self._timer.daemon = True
            self._timer.start()

    def close(self, who: str = "") -> bool:
        """닫는다. 처음 부른 쪽만 on_close 를 실행하고 True."""
        with self._lock:
            if self.closed:
                return False
            self.closed, self.closed_by = True, who
            if self._timer is not None:
                self._timer.cancel()
        self._on_close()
        return True

    def cancel(self) -> None:
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()


def window_close_delay(t_cross_ros: float, now_ros: float, cross_max_after_s: float, after_cross_s: float,
                       opened_for_s: float, window_max_s: float) -> float:
    """받기 창을 지금부터 몇 s 뒤에 닫을지. 설계 = 교차(clamp_cross 로 자름) + after_cross_s, 단 창을 연 뒤 window_max_s 를 넘지 않게
    (#171 리뷰: cross_max_after_s 로만 자르면 최대 약 2.3 s — #144 "창 약 1 s"). 0 이하면 바로 닫는다."""
    by_cross = clamp_cross(t_cross_ros, now_ros, cross_max_after_s) + after_cross_s - now_ros
    return max(0.0, min(by_cross, window_max_s - opened_for_s))


def move_time_s(dist_m: float, vel_mps: float, acc_mps2: float) -> float:
    """정지 → 정지 직선 이동 시간 (사다리꼴 속도, 짧으면 삼각형). amovel 의 vel·acc 와 같은 단위 [m/s, m/s²]."""
    d, v, a = max(0.0, float(dist_m)), float(vel_mps), float(acc_mps2)
    if d <= 0.0:
        return 0.0
    if d <= v * v / a:                                   # 최고 속도에 못 닿음 — 삼각형
        return 2.0 * (d / a) ** 0.5
    return v / a + d / v


def max_dist_in(t_s: float, vel_mps: float, acc_mps2: float) -> float:
    """t_s 안에 정지 → 정지로 갈 수 있는 최대 거리 [m] (move_time_s 의 역)."""
    t, v, a = max(0.0, float(t_s)), float(vel_mps), float(acc_mps2)
    if t <= 2.0 * v / a:
        return a * t * t / 4.0
    return v * (t - v / a)
