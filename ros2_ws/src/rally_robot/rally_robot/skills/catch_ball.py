"""rally_robot/skills/catch_ball.py — 던진 벨크로 공을 패드로 받기 (CatchBall, 모드 3)
담당: 박성현 (요격) · 정정인 (판정 뒤 로직 after_catch(), 리뷰)

비전과는 토픽으로만 연결한다 (rally_vision 코드 import 없음):
  /rally/perception/landing       PointStamped        패드 면 교차 예측점 (base_link)
  /rally/perception/landing_info  Float64MultiArray   [교차 시각, 남은 시간, 표준편차 u·w, 관측 수, 놓은 뒤 경과] (INTERFACES 2.2)

단계
1. ready     : 패드를 쥐고(없으면 닫아서 확인) 대기 자세 posj 로 관절 이동. 힘 영점
2. tracking  : 이 goal 이후 첫 예측이 오면 투구 시작 → contact_expected ON (공이 맞아도 nudge STOP 안 나게, 힘 한계 25 N 은 그대로 —
               10/11 정지 시험에서 맞을 때마다 nudge STOP). 공 놓은 뒤 decide_after_s(0.16 s, 오차 중앙 약 3 cm) 까지 최신 예측을 기다림
3. intercept : 예측점으로 패드 중심을 옮기는 TCP 위치로 amovel 한 번 (자세 고정, 왼쪽 안 감, 오른쪽·위아래 15 cm 까지 — catch_plan.intercept).
               대기 패드 중심에서 catch_radius 밖이면 움직이지 않음 (out_of_radius)
4. judge     : 도착 뒤 ~ 예측 교차 시각 + hold_after_impact_s 동안 툴 힘 증가로 명중 판정. 붙음/튕김 구분은 비전 판정이 생기면 바꾼다
5. after     : after_catch() — 정정인 담당 로직 (이 파일의 after_catch 함수, 지금은 아무것도 안 함)
6. return    : 대기 자세 TCP 로 복귀
결과: 끝까지 돌면 succeed (caught 참/거짓), 안전·타임아웃·예외면 abort.

goal 필드: catch_radius 사용 (0 이하면 제한 없음). ready_pose·lead_time_s 는 쓰지 않는다 — 대기 자세는 skills.yaml mode3.ready_posj,
           출발 시점은 decide_after_s (예측점 자체가 교차점이라 선행 시간 보정이 따로 필요 없음).
설정: skills.yaml mode3 (없으면 DEFAULTS). 값의 근거는 docs/mode3/MODE3_OVERVIEW.md.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

import numpy as np
from geometry_msgs.msg import Point, PointStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

from rally_interfaces.action import CatchBall
from rally_robot.dsr_helpers import SkillAbort, force_mag, move_checked, movej_checked
from rally_robot.skills.catch_plan import Prediction, impact_detected, intercept, pad_center, parse_info

LANDING_TOPIC = "/rally/perception/landing"
LANDING_INFO_TOPIC = "/rally/perception/landing_info"

DEFAULTS = dict(
    ready_posj=[89.8, 30.0, -121.7, 90.2, -80.2, 88.3],   # 10/11 정정인 결정 (패드 면 45°)
    pad_offset_tool_m=[-0.053, -0.048, 0.001],          # TCP → 패드 중심, 공구 좌표 (10/11 줄자)
    pad_normal=[0.7071, 0.0, 0.7071],                   # 패드 면 법선 (투구자 쪽 45° 위), base
    decide_after_s=0.16,
    right_max_m=0.15,
    updown_max_m=0.15,
    catch_vel_mm_s=450.0,                               # 위아래 730 mm/s 는 관절 속도 121 % 로 불가 (10/11 계산)
    catch_acc_mm_s2=1000.0,                             # 10/10 실측: 20 cm 출발 148 ms · 도착 0.85 s
    return_vel_mm_s=150.0,
    return_acc_mm_s2=300.0,
    ready_joint_vel_deg_s=30.0,
    ready_joint_acc_deg_s2=60.0,
    flight_timeout_s=20.0,
    landing_max_age_s=0.2,
    impact_force_n=4.0,                                 # 10/11 맞을 때 |F| 4.9~15.3 N, 정지 0.9 N
    hold_after_impact_s=0.3,
    force_poll_hz=20.0,
)


class LandingListener:
    """비전 예측 최신값. 콜백은 node.cb_state 그룹이라 스킬 실행 스레드와 따로 돈다."""

    def __init__(self, node):
        self._lock = threading.Lock()
        self._point = None
        self._pred: Prediction | None = None
        self._rx = None                       # 마지막 예측 수신 monotonic
        node.create_subscription(PointStamped, LANDING_TOPIC, self._on_point, qos_profile_sensor_data, callback_group=node.cb_state)
        node.create_subscription(Float64MultiArray, LANDING_INFO_TOPIC, self._on_info, qos_profile_sensor_data,
                                 callback_group=node.cb_state)

    def _on_point(self, msg: PointStamped):
        if msg.header.frame_id != "base_link":          # base 변환 못 한 좌표로는 로봇을 움직이지 않는다 (INTERFACES #66)
            return
        with self._lock:
            self._point = np.array([msg.point.x, msg.point.y, msg.point.z])

    def _on_info(self, msg: Float64MultiArray):
        info = parse_info(msg.data)
        with self._lock:
            if info is None or self._point is None:     # landing 다음에 info 가 온다 — 짝이 되는 점이 있어야 함
                return
            self._pred = Prediction(point=self._point.copy(), t_cross=info["t_cross"], since_release=info["since_release"],
                                    std=(info["std_u"], info["std_w"]), n_obs=int(info["n_obs"]))
            self._rx = time.monotonic()

    def latest(self, newer_than: float = 0.0, max_age_s: float = 0.2) -> Prediction | None:
        """newer_than(monotonic) 뒤에 받았고 max_age_s 안인 예측. 없으면 None."""
        with self._lock:
            if self._pred is None or self._rx <= newer_than or time.monotonic() - self._rx > max_age_s:
                return None
            return self._pred


# ------------------------------------------------------------------ 판정 뒤 · 복귀 전 (정정인 담당)
@dataclass
class CatchContext:
    caught: bool                      # 힘으로 본 명중 여부 (붙음/튕김 구분은 아직 없음)
    impact_peak_n: float              # 충격 때 힘 증가 최대 (N)
    ready_posj: list[float]           # 대기 자세 관절 (deg)
    intercept_posx: list[float]       # 요격 위치 TCP posx (mm, ZYZ) — 지금 로봇이 있는 곳
    landing_predicted_m: list[float]  # 마지막 예측 교차점 (base, m)
    cfg: dict                         # skills.yaml mode3 + DEFAULTS
    skip_return: bool = False         # True 면 대기 자세로 돌아가지 않음
    extra: dict = field(default_factory=dict)


def after_catch(node, goal_handle, ctx: CatchContext) -> None:
    """판정 직후·대기 자세 복귀 전에 한 번 불린다. TODO(정정인): 이 함수 안만 채우면 된다 (요격 코드와 따로 떨어져 있어 병합 충돌 없음).

    할 수 있는 것: dsr_helpers.move_checked · movej_checked 이동, node.gripper, goal_handle.publish_feedback
    지킬 것: 단계 사이 node.check_stop(goal_handle) · 순응은 compliance() 컨텍스트 · 접촉 예정이면 node.set_contact(True)
             (끝나면 False, _execute 의 finally 가 한 번 더 끈다) · 실패는 SkillAbort 로 올리면 _execute 가 abort·정리
    복귀가 필요 없는 흐름이면 ctx.skip_return = True.
    """
    return None


def _point(p) -> Point:
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


def execute(node):
    landing = LandingListener(node)

    def _execute(goal_handle):
        from DSR_ROBOT2 import get_current_posx, get_tool_force
        goal = goal_handle.request
        cfg = {**DEFAULTS, **(node.cfg.get("mode3") or {})}
        fb, res = CatchBall.Feedback(), CatchBall.Result()
        log = node.get_logger()

        def ros_now() -> float:
            return node.get_clock().now().nanoseconds * 1e-9

        def phase(name, pred: Prediction | None = None):
            node.check_stop(goal_handle)
            fb.phase = name
            if pred is not None:
                fb.current_prediction = _point(pred.point)
                fb.time_to_impact_s = float(pred.t_cross - ros_now())
            goal_handle.publish_feedback(fb)

        def force_now() -> float:
            return force_mag(get_tool_force())

        def hold_pad():
            if node.holding_now():
                return
            _, grip_force = node.gripper_cfg("pad")
            ok, _, detected = node.gripper(0.0, grip_force)
            if not (ok and detected):
                raise SkillAbort("no_pad", stopped=True)

        def wait_release(t_goal: float, timeout_s: float) -> Prediction:
            while time.monotonic() - t_goal < timeout_s:
                node.check_stop(goal_handle)
                pred = landing.latest(newer_than=t_goal, max_age_s=cfg["landing_max_age_s"])
                if pred is not None:
                    return pred
                time.sleep(0.005)
            raise SkillAbort("timeout", stopped=True)

        def wait_decide(pred: Prediction, t_goal: float) -> Prediction:
            """decide_after_s 까지 최신 예측으로 갱신. 예측이 끊기면 마지막 것."""
            while pred.since_release < cfg["decide_after_s"]:
                node.check_stop(goal_handle)
                time.sleep(0.005)
                newer = landing.latest(newer_than=t_goal, max_age_s=cfg["landing_max_age_s"])
                if newer is None:
                    break
                pred = newer
            return pred

        try:
            node.check_controller()
            phase("ready")
            hold_pad()
            movej_checked(node, cfg["ready_posj"], cfg["ready_joint_vel_deg_s"], cfg["ready_joint_acc_deg_s2"])
            ready = [float(v) for v in get_current_posx()[0]]
            pad0 = pad_center(ready, cfg["pad_offset_tool_m"])
            baseline = float(np.median([force_now() for _ in range(5)]))
            t_goal = time.monotonic()
            timeout_s = goal.timeout_s if goal.timeout_s > 0 else cfg["flight_timeout_s"]
            log.info(f"[catch_ball] 대기 — 패드 중심 {np.round(pad0, 3).tolist()} m, 힘 영점 {baseline:.1f} N, 반경 {goal.catch_radius:.2f} m")

            pred = wait_release(t_goal, timeout_s)
            t_detect = time.monotonic()
            node.set_contact(True)                             # 공이 날아오는 동안만 nudge 끔 (힘 한계 25 N 은 그대로)
            phase("tracking", pred)
            pred = wait_decide(pred, t_goal)
            plan = intercept(ready, pad0, cfg["pad_normal"], pred.point, cfg["right_max_m"], cfg["updown_max_m"])
            res.landing_predicted = _point(pred.point)

            if goal.catch_radius > 0 and plan.offset_m > goal.catch_radius:
                res.failure_reason = "out_of_radius"
                log.info(f"[catch_ball] 예측점이 대기 위치에서 {plan.offset_m:.2f} m — 반경 밖, 움직이지 않음")
            else:
                phase("intercept", pred)
                res.reaction_time_s = float(time.monotonic() - t_detect)
                log.info(f"[catch_ball] 요격 오른쪽 {plan.du * 100:.1f} · 아래 {plan.dw * 100:.1f} cm"
                         f"{' (잘림)' if plan.clipped else ''}, 놓은 뒤 {pred.since_release:.2f} s, 교차까지 {pred.t_cross - ros_now():.2f} s")
                move_checked(node, plan.tcp_posx, cfg["catch_vel_mm_s"], cfg["catch_acc_mm_s2"])
                res.robot_travel_mm = float(np.hypot(plan.du, plan.dw) * 1000.0)

            # 판정: 도착 뒤부터 교차 시각 + hold 까지 힘 증가 (도착 전 감속 관성은 빼려고 도착 뒤만)
            t_from = ros_now()
            last = landing.latest(newer_than=t_goal, max_age_s=1.0) or pred
            t_to = max(last.t_cross, t_from) + cfg["hold_after_impact_s"]
            samples, period = [], 1.0 / cfg["force_poll_hz"]
            while ros_now() < t_to:
                node.check_stop(goal_handle)
                samples.append((ros_now(), force_now()))
                time.sleep(period)
            hit, peak = impact_detected(samples, baseline, t_from, t_to, cfg["impact_force_n"])
            res.caught = bool(hit)
            res.landing_predicted = _point(last.point)
            res.prediction_error_mm = float("nan")             # 실제 착지점은 비전 판정(pad_hit_judge)이 생기면 채움
            if not hit and not res.failure_reason:
                res.failure_reason = "missed"
            phase("caught" if hit else "missed", last)
            log.info(f"[catch_ball] {'명중' if hit else '놓침'} (힘 증가 최대 {peak:.1f} N)")

            ctx = CatchContext(caught=res.caught, impact_peak_n=peak, ready_posj=list(cfg["ready_posj"]),
                               intercept_posx=[float(v) for v in get_current_posx()[0]],
                               landing_predicted_m=[float(v) for v in last.point], cfg=cfg)
            after_catch(node, goal_handle, ctx)
            node.set_contact(False)

            if not ctx.skip_return:
                node.check_stop(goal_handle)
                move_checked(node, ready, cfg["return_vel_mm_s"], cfg["return_acc_mm_s2"])
            goal_handle.succeed()
            return res
        except SkillAbort as e:
            res.failure_reason = e.reason
            if e.reason == "canceled":
                goal_handle.canceled()
                log.info("[catch_ball] 취소됨")
            else:
                goal_handle.abort()
                log.error(f"[catch_ball] 중단: {e.reason} (정지 확인 {e.stopped})")
            return res
        except Exception as e:  # noqa: BLE001
            res.failure_reason = f"error:{e}"
            goal_handle.abort()
            log.error(f"[catch_ball] 예외: {e}")
            return res
        finally:
            node.set_contact(False)
    return _execute
