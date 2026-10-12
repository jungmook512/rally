"""rally_robot/skills/catch_ball.py — 던진 벨크로 공을 패드로 받기 (CatchBall, 모드 3)
담당: 박성현 (요격·판정) · 정정인 (패드 집기 pickup_pad(), 판정 뒤 로직 after_catch(), 리뷰)

비전과는 토픽으로만 연결한다 (rally_vision 코드 import 없음):
  /rally/perception/landing       PointStamped        패드 면 교차 예측점 (base_link)
  /rally/perception/landing_info  Float64MultiArray   [교차 시각, 남은 시간, 표준편차 u·w, 관측 수, 놓은 뒤 경과] (INTERFACES 2.2)
  /rally/perception/ball_on_pad   PointStamped        측면 카메라가 패드 중심 주변에서 본 공 (base_link). 보일 때만 프레임마다

단계
1. ready     : pickup_pad() — 패드 집기 (정정인 자리, 거치대 교시가 있으면 거치대에서 집어 대기 자세까지) → 대기 자세 posj 로 관절 이동
2. tracking  : 이 goal 이후 첫 예측이 오면 투구 시작 → **받기 창** contact_expected ON (공이 맞아도 nudge STOP 안 나게. 힘 한계 130 N 도
               이 창에만 걸릴 예정 — #144). 공 놓은 뒤 decide_after_s(0.16 s, 오차 중앙 약 3 cm) 까지 최신 예측을 기다림
3. intercept : 예측 품질(std·n_obs)이 기준 안이고 안전이 NORMAL 이면, 예측점으로 패드 중심을 옮기는 TCP 위치로 amovel 한 번
               (자세 고정, 왼쪽 안 감, 오른쪽·위 15 cm·아래 8 cm 까지 — catch_plan.intercept).
               대기 패드 중심에서 catch_radius 밖이면 움직이지 않음 (out_of_radius) · 품질 미달이면 low_confidence · SLOW 면 slow_at_catch 중단
4. judge     : 받기 창은 **교차 시각 + contact_window_after_cross_s(0.5 s) 에 닫는다** (투구 감지 ~ 교차 + 0.5 s ≈ 1.2 s, #144 "창 약 1 s").
               닫기는 결정 시점에 건 **타이머**(catch_plan.ContactWindow)가 한다 — 요격 이동이 길어도 창은 제때 닫힌다 (10/12 밤 실측 1.3~2.4 s 결함 수정).
               판정은 창이 닫힌 뒤에도 계속 기다린다 — **비전 판정**: 측면 카메라의 ball_on_pad 가 pad_hold_s(3 s) 연속(틈 ≤ pad_gap_s) 들어오면 caught.
               교차 시각 + judge_timeout_s 안에 없으면 missed. 힘 판정은 쓰지 않는다 (10/12 결정: 패드 끝 튕김·이동 관성과 구분 불가)
5. after     : after_catch() — 정정인 자리. 지금은 대기 자세 → 패드를 거치대에 꽂고 빠짐 (공 떼기·공 거치대는 아직)
6. return    : 대기 자세 TCP 로 복귀 (after_catch 가 skip_return 을 켜면 생략)
결과: 끝까지 돌면 succeed (caught 참/거짓), 안전·타임아웃·예외면 abort.

goal 필드: catch_radius 사용 (0 이하면 제한 없음). ready_pose·lead_time_s 는 쓰지 않는다 — 대기 자세는 skills.yaml mode3.ready_posj,
           출발 시점은 decide_after_s (예측점 자체가 교차점이라 선행 시간 보정이 따로 필요 없음).
설정: skills.yaml mode3 (없으면 DEFAULTS). 값의 근거는 docs/mode3/MODE3_OVERVIEW.md · MODE3_PLAN.md 4.1.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np
from geometry_msgs.msg import Point, PointStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

from rally_interfaces.action import CatchBall
from rally_robot.dsr_helpers import SkillAbort, move_checked, movej_checked, stop_robot
from rally_robot.skills.catch_plan import ContactWindow, Prediction, clamp_cross, intercept, judge_pad, pad_center, parse_info, quality_ok

LANDING_TOPIC = "/rally/perception/landing"
LANDING_INFO_TOPIC = "/rally/perception/landing_info"
BALL_ON_PAD_TOPIC = "/rally/perception/ball_on_pad"

DEFAULTS = dict(
    ready_posj=[89.8, 30.0, -121.7, 90.2, -80.2, 88.3],   # 10/11 정정인 결정 (패드 면 45°)
    pad_offset_tool_m=[-0.0175, -0.1043, 0.0285],       # TCP → 패드 중심, 공구 좌표 (10/12 정동휘 실측 #138, z 는 #155 보정값 0.0285 — skills.yaml 과 같게. yaml 이 있으면 yaml 이 이김)
    pad_normal=[0.697, -0.123, 0.706],                  # 패드 면 법선 base (위로 45° + 그리퍼 −10° 회전, #138)
    decide_after_s=0.16,
    decide_max_s=0.40,                                  # 결정 시점 예측이 품질 미달이면 이때까지 더 기다림 (10/12 밤)
    right_max_m=0.15,
    updown_max_m=0.15,
    down_max_m=0.08,                                    # 아래(투구자 쪽): 대기 자세 −9.4 cm 에 자기충돌 (10/11 실측)
    catch_vel_mm_s=450.0,                               # 위아래 730 mm/s 는 관절 속도 121 % 로 불가 (10/11 계산)
    catch_acc_mm_s2=1000.0,                             # 10/10 실측: 20 cm 출발 148 ms · 도착 0.85 s
    return_vel_mm_s=150.0,
    return_acc_mm_s2=300.0,
    ready_joint_vel_deg_s=30.0,
    ready_joint_acc_deg_s2=60.0,
    flight_timeout_s=20.0,
    landing_max_age_s=0.2,
    gate_std_max_m=0.10,                                # 출발 시점 예측 표준편차(u·w 큰 쪽) 상한 — 넘으면 안 움직임 (정정인 #128 후속 2)
    gate_n_obs_min=4,                                   #   관측 수 하한
    cross_max_after_s=1.5,                              # 교차 시각이 이보다 뒤로 나오면 그만큼 기다리지 않음 (정정인 #128 후속 1)
    pad_hold_s=3.0,                                     # 비전 판정: 패드 위 공이 이 시간 연속 보이면 caught (10/12 결정)
    pad_gap_s=0.3,                                      #   연속으로 치는 관측 틈 상한 (측면 60 Hz, 가림·검출 실패 여유)
    judge_timeout_s=5.0,                                #   교차 시각 뒤 이 시간 안에 못 채우면 missed
    judge_poll_hz=20.0,
    contact_window_after_cross_s=0.5,                   # 받기 창(contact_expected) 닫는 시각 = 교차 + 이 값 (#144: 130 N 은 이 창에만. 판정은 창 뒤에도 계속)
)


class LandingListener:
    """비전 예측 최신값 + 패드 위 공 관측 시각. 콜백은 node.cb_state 그룹이라 스킬 실행 스레드와 따로 돈다."""

    def __init__(self, node):
        self._lock = threading.Lock()
        self._point = None
        self._pred: Prediction | None = None
        self._rx = None                       # 마지막 예측 수신 monotonic
        self._pad: deque = deque(maxlen=2000)  # (수신 monotonic, base 3D) — 패드 위 공
        node.create_subscription(PointStamped, LANDING_TOPIC, self._on_point, qos_profile_sensor_data, callback_group=node.cb_state)
        node.create_subscription(Float64MultiArray, LANDING_INFO_TOPIC, self._on_info, qos_profile_sensor_data,
                                 callback_group=node.cb_state)
        node.create_subscription(PointStamped, BALL_ON_PAD_TOPIC, self._on_pad, qos_profile_sensor_data, callback_group=node.cb_state)

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

    def _on_pad(self, msg: PointStamped):
        if msg.header.frame_id != "base_link":
            return
        with self._lock:
            self._pad.append((time.monotonic(), np.array([msg.point.x, msg.point.y, msg.point.z])))

    def latest(self, newer_than: float = 0.0, max_age_s: float = 0.2) -> Prediction | None:
        """newer_than(monotonic) 뒤에 받았고 max_age_s 안인 예측. 없으면 None."""
        with self._lock:
            if self._pred is None or self._rx <= newer_than or time.monotonic() - self._rx > max_age_s:
                return None
            return self._pred

    def pad_obs(self, newer_than: float) -> tuple[list[float], list[np.ndarray]]:
        """newer_than(monotonic) 뒤의 패드 위 공 관측 (시각 목록, 점 목록)."""
        with self._lock:
            rows = [(t, p) for t, p in self._pad if t > newer_than]
        return [t for t, _ in rows], [p for _, p in rows]


# ------------------------------------------------------------------ 정정인 자리 ① 패드 집기 (ready 전)
def pickup_pad(node, goal_handle, cfg: dict) -> None:
    """goal 시작 직후, 대기 자세로 가기 전에 한 번 불린다. TODO(정정인): 패드 거치대에서 패드를 집는 동작.

    계획한 순서 (10/12 박성현): ① 그리퍼 열기 → ② 패드 거치대의 그리퍼 손잡이 위치로 이동 → ③ 그리퍼 닫기(패드 쥠)
      → ④ 수직으로 몇 cm 들기 (값 미정) → 돌아가면 _execute 가 대기 자세(ready_posj)로 관절 이동.
    티칭 포인트는 코드가 아니라 skills.yaml mode3 에 (예: pad_rack_posx, pad_rack_lift_m).
    할 수 있는 것: dsr_helpers.move_checked · movej_checked, node.gripper, goal_handle.publish_feedback.
    지킬 것: 단계 사이 node.check_stop(goal_handle) · 실패는 SkillAbort 로.
    지금 동작: 이미 쥐고 있으면 통과. skills.yaml mode3.pad_rack_*_posj(정정인 10/12 교시)가 있으면 거치대에서 집어
      대기 자세까지 (rally_robot/pad_rack.pick_pad — A → 열기 → G → 쥐기·폭 확인 → 홈 따라 순응으로 U → 팔 먼저·손목 나중으로 대기 자세).
      교시가 없으면 예전처럼 닫아서 쥐었는지 확인 (없으면 no_pad 중단).
    """
    if node.holding_now():
        return
    from rally_robot import pad_rack
    if pad_rack.taught(node):
        pad_rack.pick_pad(node, node.get_logger(), lambda: node.check_stop(goal_handle))
        return
    _, grip_force = node.gripper_cfg("pad")
    ok, _, detected = node.gripper(0.0, grip_force)
    if not (ok and detected):
        raise SkillAbort("no_pad", stopped=True)


# ------------------------------------------------------------------ 정정인 자리 ② 판정 뒤 · 복귀 전
@dataclass
class CatchContext:
    caught: bool                      # 비전으로 본 '패드 위에 공이 pad_hold_s 연속 있음'
    pad_obs_s: float                  # 패드 위 공이 연속으로 보인 시간 (s)
    ready_posj: list[float]           # 대기 자세 관절 (deg)
    intercept_posx: list[float]       # 요격 위치 TCP posx (mm, ZYZ) — 지금 로봇이 있는 곳
    landing_predicted_m: list[float]  # 마지막 예측 교차점 (base, m)
    landing_actual_m: list[float]     # 패드 위 공 위치 평균 (base, m). 못 봤으면 nan
    cfg: dict                         # skills.yaml mode3 + DEFAULTS
    skip_return: bool = False         # True 면 대기 자세로 돌아가지 않음
    extra: dict = field(default_factory=dict)


def after_catch(node, goal_handle, ctx: CatchContext) -> None:
    """판정 직후·대기 자세 복귀 전에 한 번 불린다. TODO(정정인): 이 함수 안만 채우면 된다 (요격 코드와 따로 떨어져 있어 병합 충돌 없음).

    계획한 순서 (10/12 박성현, ctx.caught 일 때): ① 패드 거치대 위로 이동 → ② 수직 하강 → ③ 패드를 거치대에 두고 그리퍼 열기
      → ④ 벨크로(패드 찍찍이) 자리로 관절 이동 → ⑤ YOLO 로 공 인식(/rally/perception/objects 의 velcro_ball) → 집어서 잡아 뜯기
      → ⑥ 공 거치대에 두기. 놓쳤으면(caught 거짓) 패드만 거치대에 두거나 그대로 복귀 — 정하기.
    할 수 있는 것: dsr_helpers.move_checked · movej_checked 이동, node.gripper, goal_handle.publish_feedback
    지킬 것: 단계 사이 node.check_stop(goal_handle) · 순응은 compliance() 컨텍스트 · 접촉 예정이면 node.set_contact(True)
             (끝나면 False, _execute 의 finally 가 한 번 더 끈다) · 실패는 SkillAbort 로 올리면 _execute 가 abort·정리
    복귀가 필요 없는 흐름이면 ctx.skip_return = True.

    지금 동작 (정정인 10/12): mode3.pad_place_after_catch 이고 거치대 교시가 있으면 — 대기 자세로 관절 복귀(요격 자리에서 U 로
      바로 가면 TCP 가 작업 공간 밖으로 휨) → pad_rack.place_pad (대기 자세 → 손목 먼저·팔 나중 → U → 순응 꽂기 → 놓고 빠짐)
      → skip_return. ④~⑥ 공 떼기·공 거치대는 아직 (패드에 공이 붙은 채 거치대에 꽂힘).
      10/12 밤 정정인 "놓쳤는데 왜 꽂음": 놓쳤으면(caught 거짓) 꽂지 않고 패드를 쥔 채 대기 자세로 복귀 (pad_place_only_if_caught).
    """
    from rally_robot import pad_rack
    from rally_robot.dsr_helpers import movej_checked
    if not (ctx.cfg.get("pad_place_after_catch", True) and pad_rack.taught(node)):
        return None
    if ctx.cfg.get("pad_place_only_if_caught", True) and not ctx.caught:
        node.get_logger().info("[catch_ball] 놓침 — 패드를 쥔 채 대기 자세로 (거치대에 꽂지 않음)")
        return None
    node.check_stop(goal_handle)
    jv, ja = pad_rack.joint_speeds(node)
    movej_checked(node, ctx.ready_posj, jv, ja)
    pad_rack.place_pad(node, node.get_logger(), lambda: node.check_stop(goal_handle))
    ctx.skip_return = True
    return None


def _point(p) -> Point:
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


def execute(node):
    landing = LandingListener(node)

    def _execute(goal_handle):
        from DSR_ROBOT2 import get_current_posx
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

        def wait_release(t_goal: float, timeout_s: float) -> Prediction:
            while time.monotonic() - t_goal < timeout_s:
                node.check_stop(goal_handle)
                pred = landing.latest(newer_than=t_goal, max_age_s=cfg["landing_max_age_s"])
                if pred is not None:
                    return pred
                time.sleep(0.005)
            raise SkillAbort("timeout", stopped=True)

        def wait_decide(pred: Prediction, t_goal: float) -> Prediction:
            """decide_after_s 까지 최신 예측으로 갱신. 그때 예측 품질이 기준(gate_std_max_m·gate_n_obs_min) 미달이면
            decide_max_s 까지 더 기다린다 — 10/12 밤 실로봇 16회: 0.25 s 시점 예측(관측 5·std 12~14 cm)이 38 cm 빗나가
            '반경 밖'으로 안 움직였는데 그 뒤 예측은 좋아졌다. 예측이 끊기면 마지막 것."""
            while True:
                node.check_stop(goal_handle)
                time.sleep(0.005)
                newer = landing.latest(newer_than=t_goal, max_age_s=cfg["landing_max_age_s"])
                if newer is None:
                    break
                pred = newer
                if pred.since_release < cfg["decide_after_s"]:
                    continue
                if quality_ok(pred.std, pred.n_obs, cfg["gate_std_max_m"], cfg["gate_n_obs_min"]) or pred.since_release >= cfg["decide_max_s"]:
                    break
            if pred.since_release > cfg["decide_after_s"] + 0.02:
                log.info(f"[catch_ball] 예측 품질 기다림 → 놓은 뒤 {pred.since_release:.2f} s 에 결정 (std {pred.std[0] * 100:.1f}·{pred.std[1] * 100:.1f} cm, 관측 {pred.n_obs})")
            return pred

        t_open = {"mono": None}

        def close_window():
            node.set_contact(False)
            node.set_catch_window(False)                       # 힘 한계 130 N → 25 N (#144)
            opened = f"{time.monotonic() - t_open['mono']:.2f} s" if t_open["mono"] is not None else "-"
            log.info(f"[catch_ball] 받기 창 닫음 ({window.closed_by or '?'}, 열린 뒤 {opened}; 설계 = 교차 + {cfg['contact_window_after_cross_s']:.2f} s)")

        window = ContactWindow(close_window)

        def judge(t_from_mono: float, t_cross_ros: float) -> tuple[bool, float, list[float]]:
            """비전 판정 (catch_plan.judge_pad). 받기 창은 타이머가 교차 + contact_window_after_cross_s 에 닫는다 — 여기서는 타이머가
            아직이면(이동이 빨리 끝남) 같은 시각에 닫고, 이미 닫혔으면 아무것도 안 한다. 판정은 창과 무관하게 계속."""
            return judge_pad(landing.pad_obs, time.monotonic, ros_now, time.sleep, lambda: node.check_stop(goal_handle),
                             t_from_mono, t_cross_ros, cfg, contact_close=lambda: window.close("judge"))

        try:
            node.check_controller()
            phase("ready")
            pickup_pad(node, goal_handle, cfg)
            movej_checked(node, cfg["ready_posj"], cfg["ready_joint_vel_deg_s"], cfg["ready_joint_acc_deg_s2"])
            ready = [float(v) for v in get_current_posx()[0]]
            pad0 = pad_center(ready, cfg["pad_offset_tool_m"])
            t_goal = time.monotonic()
            timeout_s = goal.timeout_s if goal.timeout_s > 0 else cfg["flight_timeout_s"]
            log.info(f"[catch_ball] 대기 — 패드 중심 {np.round(pad0, 3).tolist()} m, 반경 {goal.catch_radius:.2f} m")

            pred = wait_release(t_goal, timeout_s)
            t_detect = time.monotonic()
            node.set_contact(True)                             # 받기 창 열기 — nudge 끔. 교차 + 0.5 s 에 타이머가 닫는다
            node.set_catch_window(True)                        # 받기 창 동안만 힘 한계 catch_window_force_limit_n (130 N, #144)
            t_open["mono"] = time.monotonic()
            phase("tracking", pred)
            pred = wait_decide(pred, t_goal)
            # 받기 창 닫기 타이머: 결정 시점 예측의 교차 시각 + 0.5 s. 요격 이동(move_checked, 0.5~0.8 s)이 끝나기를 기다리지 않는다
            t_close = clamp_cross(pred.t_cross, ros_now(), cfg["cross_max_after_s"]) + cfg["contact_window_after_cross_s"]
            window.schedule_close(t_close - ros_now())
            plan = intercept(ready, pad0, cfg["pad_normal"], pred.point, cfg["right_max_m"], cfg["updown_max_m"], cfg["down_max_m"])
            res.landing_predicted = _point(pred.point)

            if goal.catch_radius > 0 and plan.offset_m > goal.catch_radius:
                res.failure_reason = "out_of_radius"
                d = [float(a - b) for a, b in zip(pred.point, pad0)]
                log.info(f"[catch_ball] 예측점이 대기 위치에서 {plan.offset_m:.2f} m — 반경 밖, 움직이지 않음 · 예측 교차점 "
                         f"({pred.point[0]:.3f}, {pred.point[1]:.3f}, {pred.point[2]:.3f}) m, 패드 중심 ({pad0[0]:.3f}, {pad0[1]:.3f}, {pad0[2]:.3f}) m, "
                         f"차이 x {d[0] * 100:+.0f} y {d[1] * 100:+.0f} z {d[2] * 100:+.0f} cm (std {pred.std[0] * 100:.1f}·{pred.std[1] * 100:.1f} cm, 관측 {pred.n_obs})")
            elif not quality_ok(pred.std, pred.n_obs, cfg["gate_std_max_m"], cfg["gate_n_obs_min"]):
                res.failure_reason = "low_confidence"
                log.info(f"[catch_ball] 예측 품질 미달 (std {pred.std[0] * 100:.1f}·{pred.std[1] * 100:.1f} cm, 관측 {pred.n_obs}) — 움직이지 않음")
            else:
                node.check_stop(goal_handle)
                if node.speed_scale() < 1.0:                   # 감속된 이동은 늦게 도착해 의미 없음 (PushBall slow_at_strike 와 같은 규칙)
                    raise SkillAbort(f"slow_at_catch (안전 SLOW {node.safety_reason()})", stopped=True)
                phase("intercept", pred)
                res.reaction_time_s = float(time.monotonic() - t_detect)
                log.info(f"[catch_ball] 요격 오른쪽 {plan.du * 100:.1f} · 아래 {plan.dw * 100:.1f} cm"
                         f"{' (잘림)' if plan.clipped else ''}, 놓은 뒤 {pred.since_release:.2f} s, 교차까지 {pred.t_cross - ros_now():.2f} s")
                move_checked(node, plan.tcp_posx, cfg["catch_vel_mm_s"], cfg["catch_acc_mm_s2"])
                res.robot_travel_mm = float(np.hypot(plan.du, plan.dw) * 1000.0)

            # 판정 (비전): 투구 감지 뒤의 패드 위 공 관측. 교차 시각은 지금 기준 cross_max_after_s 로 자름
            last = landing.latest(newer_than=t_goal, max_age_s=1.0) or pred
            t_cross = clamp_cross(last.t_cross, ros_now(), cfg["cross_max_after_s"])
            caught, held_s, actual = judge(t_detect, t_cross)
            res.caught = bool(caught)
            res.landing_predicted = _point(last.point)
            res.landing_actual = _point(actual)
            res.prediction_error_mm = float(np.linalg.norm(np.array(actual) - last.point) * 1000.0) if np.isfinite(actual[0]) else float("nan")
            if caught:
                res.failure_reason = ""                        # 안 움직였어도(반경 밖·저신뢰) 공이 패드에 왔으면 잡음. 이동 여부는 robot_travel_mm
            elif not res.failure_reason:
                res.failure_reason = "missed"
            phase("caught" if caught else "missed", last)
            log.info(f"[catch_ball] {'잡음' if caught else '놓침'} (패드 위 공 연속 {held_s:.1f} s, 사유 {res.failure_reason or '-'})")

            ctx = CatchContext(caught=res.caught, pad_obs_s=held_s, ready_posj=list(cfg["ready_posj"]),
                               intercept_posx=[float(v) for v in get_current_posx()[0]],
                               landing_predicted_m=[float(v) for v in last.point], landing_actual_m=[float(v) for v in actual], cfg=cfg)
            after_catch(node, goal_handle, ctx)

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
            stop_robot(node.dsr, log, f"catch_ball 예외 {type(e).__name__}")   # 움직이는 중 예외면 먼저 세운다 (정정인 #128 후속 4)
            goal_handle.abort()
            log.error(f"[catch_ball] 예외: {e}")
            return res
        finally:
            window.close("finally")                            # 아직 안 닫혔으면 여기서 (중단·예외 포함). 닫혔으면 아무것도 안 함
            window.cancel()
            node.set_contact(False)
            node.set_catch_window(False)
    return _execute
