"""rally_robot/skills/catch_ball.py — 던진 벨크로 공을 패드로 받기 (CatchBall, 모드 3)
담당: 박성현 (요격·판정) · 정정인 (패드 집기 pickup_pad(), 판정 뒤 로직 after_catch(), 리뷰)

비전과는 토픽으로만 연결한다 (rally_vision 코드 import 없음):
  /rally/perception/landing       PointStamped        패드 면 교차 예측점 (base_link)
  /rally/perception/landing_info  Float64MultiArray   [교차 시각, 남은 시간, 표준편차 u·w, 관측 수, 놓은 뒤 경과] (INTERFACES 2.2)
  /rally/perception/ball_on_pad   PointStamped        측면 카메라가 패드 중심 주변에서 본 공 (base_link). 보일 때만 프레임마다

단계
1. ready     : pickup_pad() — 패드 집기 (정정인 자리, 거치대 교시가 있으면 거치대에서 집어 대기 자세까지) → 대기 자세 posj 로 관절 이동
2. tracking  : 이 goal 이후 첫 예측이 오면 투구 시작 → **받기 창** contact_expected ON (공이 맞아도 nudge STOP 안 나게. 힘 한계 130 N 도
               이 창에만 걸릴 예정 — #144). 공 놓은 뒤 decide_after_s(0.25 s, 10/12 밤 실투구 기준) 까지 최신 예측을 기다림
   **정책 모드 (decide_mode: policy, 기본 rule)**: 놓은 뒤 policy_start_s~policy_max_s 동안 policy_hz 로 학습한 정책(ONNX,
               catch_policy.py)이 출발 시점과 예측점 보정을 정한다. 출발이 안 나오거나 모델을 못 읽으면 규칙 결정으로 폴백. 아래 3 이후는 같다.
3. intercept : 예측 품질(std·n_obs)이 기준 안이고 안전이 NORMAL 이면, 예측점으로 패드 중심을 옮기는 TCP 위치로 amovel 한 번
               (자세 고정, 왼쪽 안 감, 오른쪽·위 15 cm·아래 8 cm 까지 — catch_plan.intercept).
               **비행 중 갱신 (MODE3_PLAN ③, 기본 꺼짐 resend_enabled)**: 이동 중 새 예측이 처음 예측점에서 resend_threshold_m(3 cm) 넘게
               벗어나면 놓은 뒤 resend_until_release_s(0.45 s) 안에 한 번(resend_max) 새 요격점으로 amovel 을 다시 보낸다 (멈추지 않음,
               catch_plan.should_resend · dsr_helpers.move_checked retarget). 실측 #130: 재전송은 첫 동작 위에 얹혀 속도가 명령값을 넘을 수
               있어(0.52 > 0.45 m/s) 정정인 안전 확인 전엔 켜지 않는다.
               대기 패드 중심에서 catch_radius 밖이면 움직이지 않음 (out_of_radius) · 품질 미달이면 low_confidence · SLOW 면 slow_at_catch 중단
4. judge     : 받기 창은 **교차 시각 + contact_window_after_cross_s(0.5 s) 에 닫는다** (투구 감지 ~ 교차 + 0.5 s ≈ 1.2 s, #144 "창 약 1 s").
               닫기는 결정 시점에 건 **타이머**(catch_plan.ContactWindow)가 한다 — 요격 이동이 길어도 창은 제때 닫힌다 (10/12 밤 실측 1.3~2.4 s 결함 수정).
               교차가 늦게 예측돼도 창은 연 뒤 contact_window_max_s(1.5 s) 를 넘지 않는다 (#171 리뷰 — safety_monitor catch_window_max_s 2 s 가 뒷받침).
               판정은 창이 닫힌 뒤에도 계속 기다린다 — **비전 판정**: 측면 카메라의 ball_on_pad 가 pad_hold_s(1 s) 연속(틈 ≤ pad_gap_s) 들어오면 caught.
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
from rally_robot.skills.catch_policy import OnnxPolicy, decide_with_policy, find_policy, policy_target
from rally_robot.skills.catch_plan import (ContactWindow, Prediction, clamp_cross, intercept, judge_pad, max_dist_in, move_time_s, pad_center,
                                              parse_info, quality_ok,
                                              should_resend, window_close_delay)

LANDING_TOPIC = "/rally/perception/landing"
LANDING_INFO_TOPIC = "/rally/perception/landing_info"
BALL_ON_PAD_TOPIC = "/rally/perception/ball_on_pad"

DEFAULTS = dict(
    ready_posj=[89.8, 30.0, -121.7, 90.2, -80.2, 88.3],   # 10/11 정정인 결정 (패드 면 45°)
    pad_offset_tool_m=[-0.0175, -0.1043, 0.0285],       # TCP → 패드 중심, 공구 좌표 (10/12 정동휘 실측 #138, z 는 #155 보정값 0.0285 — skills.yaml 과 같게. yaml 이 있으면 yaml 이 이김)
    pad_normal=[0.697, -0.123, 0.706],                  # 패드 면 법선 base (위로 45° + 그리퍼 −10° 회전, #138)
    decide_after_s=0.25,                                # 10/12 밤 0.16 → 0.25 (skills.yaml 과 같게 — 근거는 yaml 주석)
    decide_max_s=0.40,                                  # 결정 시점 예측이 품질 미달이면 이때까지 더 기다림 (10/12 밤)
    decide_ttg_s=0.0,                                   # >0 이면 예측 교차까지 이만큼 남을 때까지 더 보고 결정 (10/13, 0 = 끔)
    ball_removal_wait_s=0.0,                            # 받은 뒤 대기 자세에서 접촉 예정 켜고 기다림 (공 떼기, 10/13). 0 = 안 기다림
    right_max_m=0.15,
    updown_max_m=0.15,
    down_max_m=0.08,                                    # 아래(투구자 쪽): 대기 자세 −9.4 cm 에 자기충돌 (10/11 실측)
    reach_max_m=0.35,                                   # 두 축 합 이동 상한 — 대각선(오른쪽 32·위 35 cm)은 두산 NOT REACHABLE (10/13)
    catch_overshoot_m=0.0,                              # 목표를 이동 방향으로 이만큼 더 (감속 구간을 예측점 뒤로, 10/13 정정인). catch_plan.intercept
    window_start_lag_s=0.15,                            # 받기 창 안에 끝나는지 볼 때 더하는 출발 지연 (10/10 실측 amovel 출발 148 ms)
    window_min_move_m=0.02,                             # 창 안에 갈 수 있는 거리가 이보다 짧으면 움직이지 않음 (too_late)
    resend_min_interval_s=0.3,                          # 비행 중 갱신: 직전 amovel 뒤 최소 이 간격 (10/13: 0.05 s 뒤 재전송이 첫 동작에 겹쳐 J1 150 °/s 초과·외력 103 N 보호 정지)
    catch_vel_mm_s=450.0,                               # 위아래 730 mm/s 는 관절 속도 121 % 로 불가 (10/11 계산)
    catch_acc_mm_s2=2000.0,                             # #144 승인값 (받기 창 130 N, #165 와 함께). 1000 은 10/10 실측 20 cm 도착 0.85 s
    return_vel_mm_s=150.0,
    return_acc_mm_s2=300.0,
    ready_joint_vel_deg_s=30.0,
    ready_joint_acc_deg_s2=60.0,
    flight_timeout_s=20.0,
    landing_max_age_s=0.2,
    gate_std_max_m=0.10,                                # 출발 시점 예측 표준편차(u·w 큰 쪽) 상한 — 넘으면 안 움직임 (정정인 #128 후속 2)
    gate_n_obs_min=4,                                   #   관측 수 하한
    cross_max_after_s=1.5,                              # 교차 시각이 이보다 뒤로 나오면 그만큼 기다리지 않음 (정정인 #128 후속 1)
    pad_hold_s=1.0,                                     # 비전 판정: 패드 위 공이 이 시간 연속 보이면 caught (10/12 밤 3.0 → 1.0)
    pad_gap_s=0.3,                                      #   연속으로 치는 관측 틈 상한 (측면 60 Hz, 가림·검출 실패 여유)
    judge_timeout_s=5.0,                                #   교차 시각 뒤 이 시간 안에 못 채우면 missed
    judge_poll_hz=20.0,
    resend_enabled=False,                               # 비행 중 갱신 ③: 이동 중 예측이 바뀌면 amovel 한 번 더 (#130 속도 초과 안전 확인 뒤 켬)
    resend_threshold_m=0.03,                            #   처음 예측점에서 이만큼 넘게 벗어나면
    resend_until_release_s=0.45,                        #   놓은 뒤 이 시각까지만 (결정 0.25~0.40 s 직후만 의미, 그 뒤는 가도 늦음)
    resend_max=1,                                       #   최대 횟수
    decide_mode="rule",                                 # rule | policy — policy 면 학습한 정책(ONNX)이 출발 시점·보정을 정함 (catch_policy.py, 기본 rule)
    policy_path="",                                     #   비면 /models · ~/rally/models · models 에서 policy_stage1.onnx
    policy_hz=50.0,                                     #   정책 호출 주기 (시뮬 dt 0.02 s)
    policy_start_s=0.10,                                #   놓은 뒤 이 시각부터 부름 (시뮬 첫 결정)
    policy_max_s=0.40,                                  #   여기까지 출발이 안 나오면 규칙 결정으로 폴백
    policy_std_source="sim_table",                      #   obs 오차 칸: sim_table(학습 표, 기본) | landing_info(칼만 std — 학습 분포와 달라 늦게 출발)
    policy_corr_scale_m=0.05,                           #   action[1:3] × 이 값 = 보정 (시뮬과 같게)
    policy_corr_max_m=0.10,                             #   보정 상한
    contact_window_max_s=1.5,                           # 받기 창 절대 상한 — 연 뒤 이 시간이면 교차 예측과 무관하게 닫음 (#171 리뷰. safety_monitor 상한 2 s 보다 짧게)
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
    # 10/13 실로봇: 터미널 2 재시작 직후엔 명령 기록이 없고 grip 비트도 0 (패드를 쥔 폭 26.4 mm 인데) → 빈손으로 보고
    # 대기 자세에서 거치대로 집으러 가다 workspace_limit STOP. 폭이 뭔가 쥔 폭이면 패드 힘으로 한 번 다시 쥐어 확인 (팔은 안 움직임)
    st = node.gripper_state
    open_mm, grip_force = node.gripper_cfg("pad")
    if st is not None and 20.0 <= st[1] < open_mm - 5.0:
        ok, width, detected = node.gripper(0.0, grip_force)
        node.get_logger().info(f"[catch_ball] 명령 기록 없이 폭 {st[1]:.1f} mm — 다시 쥠: 폭 {width:.1f} mm, 물체 {detected}")
        if ok and detected:
            return
        node.gripper(open_mm, grip_force)                    # 빈손이면 열어 두고 거치대에서 집음
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


def _load_policy(node, cfg0: dict):
    """decide_mode == policy 면 ONNX 정책을 한 번 읽는다. 못 읽으면 None (경고, 규칙으로 돈다)."""
    if str(cfg0.get("decide_mode", "rule")) != "policy":
        return None
    path = find_policy(str(cfg0.get("policy_path", "") or ""))
    if not path:
        node.get_logger().error("[catch_ball] decide_mode=policy 인데 정책 파일 없음 (models/policy_stage1.onnx) — 규칙 결정으로 돎")
        return None
    try:
        pol = OnnxPolicy(path)
    except Exception as e:  # noqa: BLE001 — onnxruntime 없음·파일 깨짐
        node.get_logger().error(f"[catch_ball] 정책 {path} 를 못 읽음 ({type(e).__name__}: {e}) — 규칙 결정으로 돎. 로봇 PC: pip install onnxruntime")
        return None
    node.get_logger().info(f"[catch_ball] 정책 모드 — {path}")
    return pol


def execute(node):
    landing = LandingListener(node)
    policy = _load_policy(node, {**DEFAULTS, **(node.cfg.get("mode3") or {})})

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
                if pred.since_release >= cfg["decide_max_s"]:
                    break
                # 10/13 오후: 높은 포물선(비행 0.9~1.4 s)은 놓은 뒤 0.25 s 예측이 14~36 cm 빗나감. 예측 교차까지 decide_ttg_s 남을 때까지
                # 더 보고 결정하면 3~6 cm (bag 1511, 4 투구), 빠른 투구는 0.25~0.31 s 에 이미 그 조건이라 거의 그대로 (bag 0738, 1~7 cm)
                ttg_ok = cfg["decide_ttg_s"] <= 0 or pred.t_cross - ros_now() <= cfg["decide_ttg_s"]
                if quality_ok(pred.std, pred.n_obs, cfg["gate_std_max_m"], cfg["gate_n_obs_min"]) and ttg_ok:
                    break
            if pred.since_release > cfg["decide_after_s"] + 0.02:
                log.info(f"[catch_ball] 예측 품질·남은 시간 기다림 → 놓은 뒤 {pred.since_release:.2f} s 에 결정 (std {pred.std[0] * 100:.1f}·{pred.std[1] * 100:.1f} cm, "
                         f"관측 {pred.n_obs}, 교차까지 {pred.t_cross - ros_now():.2f} s)")
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
            target = None                                      # 정책 모드에서 출발이 나오면 보정한 점, 아니면 예측점 그대로
            if policy is not None and cfg["decide_mode"] == "policy":
                pred, action, calls = decide_with_policy(
                    lambda: landing.latest(newer_than=t_goal, max_age_s=cfg["landing_max_age_s"]), policy, pad0, cfg["pad_normal"],
                    ros_now, time.sleep, lambda: node.check_stop(goal_handle), cfg, pred,
                    on_error=lambda e: log.error(f"[catch_ball] 정책 실행 오류 {type(e).__name__}: {e} — 이 투구는 규칙 결정으로"))
                if action is not None:
                    target = policy_target(pred.point, action, cfg["pad_normal"], cfg["policy_corr_scale_m"], cfg["policy_corr_max_m"])
                    log.info(f"[catch_ball] 정책 출발 — 놓은 뒤 {pred.since_release:.2f} s ({calls}회 호출), 보정 "
                             f"u {float(np.clip(action[1] * cfg['policy_corr_scale_m'], -cfg['policy_corr_max_m'], cfg['policy_corr_max_m'])) * 100:+.1f} · "
                             f"w {float(np.clip(action[2] * cfg['policy_corr_scale_m'], -cfg['policy_corr_max_m'], cfg['policy_corr_max_m'])) * 100:+.1f} cm")
                else:
                    log.info(f"[catch_ball] 정책이 놓은 뒤 {pred.since_release:.2f} s 까지 출발 안 함 ({calls}회 호출) — 규칙 결정으로")
                    pred = wait_decide(pred, t_goal)
            else:
                pred = wait_decide(pred, t_goal)
            # 받기 창 닫기 타이머: 결정 시점 예측의 교차 시각 + 0.5 s. 요격 이동(move_checked, 0.5~0.8 s)이 끝나기를 기다리지 않는다
            close_in = window_close_delay(pred.t_cross, ros_now(), cfg["cross_max_after_s"], cfg["contact_window_after_cross_s"],
                                          time.monotonic() - t_open["mono"], cfg["contact_window_max_s"])
            window.schedule_close(close_in)
            t_close = time.monotonic() + close_in
            v_mps, a_mps2 = cfg["catch_vel_mm_s"] / 1000.0, cfg["catch_acc_mm_s2"] / 1000.0

            def window_reach(now: float) -> float:
                """지금 출발해 받기 창이 닫히기 전에 멈출 수 있는 최대 거리 [m]. 창이 닫힌 뒤 감속 관성(59~73 N)이 힘 한계 25 N 을 넘어
                force_limit STOP 이 나지 않게 (#184 리뷰 — 창 길이는 #144 승인 범위라 늘리지 않는다)."""
                return max_dist_in(t_close - now - cfg["window_start_lag_s"], v_mps, a_mps2)

            reach_now = window_reach(time.monotonic())
            plan = intercept(ready, pad0, cfg["pad_normal"], pred.point if target is None else target,
                             cfg["right_max_m"], cfg["updown_max_m"], cfg["down_max_m"], min(cfg["reach_max_m"], reach_now), cfg["catch_overshoot_m"])
            if reach_now < cfg["reach_max_m"]:
                log.info(f"[catch_ball] 받기 창이 {close_in:.2f} s 뒤 닫힘 → 이동 최대 {reach_now * 100:.0f} cm (창 안에 멈추게)")
            res.landing_predicted = _point(pred.point)

            if goal.catch_radius > 0 and plan.offset_m > goal.catch_radius:
                res.failure_reason = "out_of_radius"
                node.set_catch_window(False)                   # 안 움직이면 130 N 필요 없음 — 바로 25 N (#165 리뷰). nudge 끔은 유지
                d = [float(a - b) for a, b in zip(pred.point, pad0)]
                log.info(f"[catch_ball] 예측점이 대기 위치에서 {plan.offset_m:.2f} m — 반경 밖, 움직이지 않음 · 예측 교차점 "
                         f"({pred.point[0]:.3f}, {pred.point[1]:.3f}, {pred.point[2]:.3f}) m, 패드 중심 ({pad0[0]:.3f}, {pad0[1]:.3f}, {pad0[2]:.3f}) m, "
                         f"차이 x {d[0] * 100:+.0f} y {d[1] * 100:+.0f} z {d[2] * 100:+.0f} cm (std {pred.std[0] * 100:.1f}·{pred.std[1] * 100:.1f} cm, 관측 {pred.n_obs})")
            elif reach_now < cfg["window_min_move_m"] and float(np.hypot(plan.du, plan.dw)) > 0.0:
                res.failure_reason = "too_late"
                node.set_catch_window(False)                   # 안 움직임 — 바로 25 N
                log.info(f"[catch_ball] 받기 창 안에 {cfg['window_min_move_m'] * 100:.0f} cm 도 못 감 (창 {close_in:.2f} s 뒤 닫힘) — 움직이지 않음")
            elif not quality_ok(pred.std, pred.n_obs, cfg["gate_std_max_m"], cfg["gate_n_obs_min"]):
                res.failure_reason = "low_confidence"
                node.set_catch_window(False)                   # 위와 같음
                log.info(f"[catch_ball] 예측 품질 미달 (std {pred.std[0] * 100:.1f}·{pred.std[1] * 100:.1f} cm, 관측 {pred.n_obs}) — 움직이지 않음")
            else:
                node.check_stop(goal_handle)
                if node.speed_scale() < 1.0:                   # 감속된 이동은 늦게 도착해 의미 없음 (PushBall slow_at_strike 와 같은 규칙)
                    raise SkillAbort(f"slow_at_catch (안전 SLOW {node.safety_reason()})", stopped=True)
                phase("intercept", pred)
                res.reaction_time_s = float(time.monotonic() - t_detect)
                log.info(f"[catch_ball] 요격 오른쪽 {plan.du * 100:.1f} · 아래 {plan.dw * 100:.1f} cm"
                         f"{' (잘림)' if plan.clipped else ''}, 놓은 뒤 {pred.since_release:.2f} s, 교차까지 {pred.t_cross - ros_now():.2f} s")
                sent = {"n": 0, "plan": plan, "pred": pred, "t_cmd": time.monotonic()}

                def retarget():
                    """비행 중 갱신 ③ (기본 꺼짐): 새 예측이 처음 예측점에서 3 cm 넘게 벗어나면 새 요격점으로 한 번 더."""
                    newer = landing.latest(newer_than=t_goal, max_age_s=cfg["landing_max_age_s"])
                    if newer is None:
                        return None
                    if newer.since_release != why["last_t"]:                 # 10/12 밤 진단: 재전송이 안 될 때 왜 안 되는지 (새 예측마다 한 번)
                        why["last_t"] = newer.since_release
                        why["seen"] += 1
                        d = float(np.linalg.norm(np.asarray(newer.point) - np.asarray(pred.point)))
                        if newer.since_release > float(cfg.get("resend_until_release_s", 0.30)):
                            why["late"] += 1
                        elif d <= float(cfg.get("resend_threshold_m", 0.03)):
                            why["near"] += 1
                        elif not quality_ok(newer.std, newer.n_obs, cfg["gate_std_max_m"], cfg["gate_n_obs_min"]):
                            why["quality"] += 1
                    # 10/13: 마지막으로 보낸 예측과 비교 (처음 예측이 아니라) · 교차 직전(resend_min_ttg_s 안)엔 안 보냄
                    # · 직전 amovel 뒤 resend_min_interval_s 안엔 안 보냄 (겹치면 속도가 더해짐 — 10/12 측정 0.15 s 겹침·0.30 s 안 겹침, 10/13 보호 정지)
                    if newer.t_cross - ros_now() < float(cfg.get("resend_min_ttg_s", 0.0)):
                        return None
                    if time.monotonic() - sent["t_cmd"] < cfg["resend_min_interval_s"]:
                        why["interval"] += 1
                        return None
                    if not should_resend(sent["pred"].point, newer.point, newer.since_release, sent["n"], cfg):
                        return None
                    if not quality_ok(newer.std, newer.n_obs, cfg["gate_std_max_m"], cfg["gate_n_obs_min"]):
                        return None
                    new_plan = intercept(ready, pad0, cfg["pad_normal"], newer.point, cfg["right_max_m"], cfg["updown_max_m"], cfg["down_max_m"], cfg["reach_max_m"], cfg["catch_overshoot_m"])
                    if goal.catch_radius > 0 and new_plan.offset_m > goal.catch_radius:
                        why["radius"] += 1
                        return None
                    # 지금 자리에서 새 목표까지 받기 창 안에 멈출 수 있을 때만 (정지 출발로 보수적으로 계산, #184 리뷰)
                    cur_rt = get_current_posx()[0]
                    d_rt = float(np.linalg.norm(np.asarray(new_plan.tcp_posx[:3]) - np.asarray(cur_rt[:3]))) / 1000.0
                    if d_rt > window_reach(time.monotonic()):
                        why["window"] += 1
                        return None
                    sent["n"] += 1
                    sent["plan"], sent["pred"], sent["t_cmd"] = new_plan, newer, time.monotonic()
                    log.info(f"[catch_ball] 비행 중 갱신 {sent['n']}회 — 예측 {np.linalg.norm(newer.point - pred.point) * 100:.1f} cm 이동, "
                             f"놓은 뒤 {newer.since_release:.2f} s, 새 요격 오른쪽 {new_plan.du * 100:.1f} · 아래 {new_plan.dw * 100:.1f} cm")
                    return new_plan.tcp_posx

                why = dict(last_t=None, seen=0, late=0, near=0, quality=0, radius=0, interval=0, window=0)
                # 정책 보정점으로 출발했으면 재전송 안 함 — retarget 은 보정 없는 예측점으로 덮어쓴다 (#175 리뷰 2)
                resend = cfg["resend_enabled"] and target is None
                if cfg["resend_enabled"] and target is not None:
                    log.info("[catch_ball] 정책 보정점으로 출발 — 비행 중 갱신(resend) 끔")
                move_checked(node, plan.tcp_posx, cfg["catch_vel_mm_s"], cfg["catch_acc_mm_s2"],
                             retarget=retarget if resend else None)
                if resend:
                    log.info(f"[catch_ball] 비행 중 갱신 {sent['n']}회 — 이동 중 새 예측 {why['seen']}개: 창(놓은 뒤 {cfg['resend_until_release_s']:.2f} s) 지남 {why['late']} · "
                             f"3 cm 안 {why['near']} · 품질 미달 {why['quality']} · 반경 밖 {why['radius']} · 간격 {cfg['resend_min_interval_s']:.2f} s 안 {why['interval']}"
                             f" · 받기 창 안에 못 멈춤 {why['window']}")
                plan, pred = sent["plan"], sent["pred"]
                res.landing_predicted = _point(pred.point)
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
            # 10/13 실로봇: 받은 공을 대기 자세에서 떼면 힘 급변이 nudge STOP 으로 잡힘(3회). 대기 자세에 멈춘 채 '접촉 예정' 을 켜고
            # ball_removal_wait_s 동안 기다린다 — nudge 판정만 쉬고 힘 한계 25 N 은 그대로 (safety_monitor 의 기존 contact_expected 규칙)
            # 10/13: 측면 판정이 패드 한가운데 붙은 공을 못 봐 '놓침'으로 나오는 경우가 있어(패드가 측면 카메라에 거의 옆면) 받았든 아니든
            # 투구가 있었으면 기다린다 — 사람이 붙은 공을 떼는 동안. 10/13 실로봇: 품질 미달로 안 움직였는데 공이 패드에 붙어
            # 다음 시도 대기 중에 떼다가 nudge STOP → 이동 여부와 상관없이 (투구가 없으면 위에서 timeout 으로 끝나 여기 안 옴)
            if not ctx.skip_return and cfg["ball_removal_wait_s"] > 0:
                node.set_contact(True)
                log.info(f"[catch_ball] 공이 붙어 있으면 떼 주세요 — {cfg['ball_removal_wait_s']:.0f} s (접촉 예정: nudge 만 쉼, 힘 한계 그대로, 로봇 정지)")
                try:
                    t_rm = time.monotonic()
                    while time.monotonic() - t_rm < cfg["ball_removal_wait_s"]:
                        node.check_stop(goal_handle)
                        time.sleep(0.1)
                finally:
                    node.set_contact(False)
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
