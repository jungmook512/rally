"""rally_brain/rally_brain/trees/mode1_push.py — 모드 1 공 굴려주기: 시도 1회 서브트리
담당: 조정묵 (10/8 회의 결정 반영 초안: 정정인 — 리뷰 부탁)

  모드 1 시도 (Guard — 도중에 멈추면 interrupted 기록)
   └─ Sequence
       ├─ 손 보기 자세     /rally/robot/go_hand_view — 툴을 환자 쪽으로 기울여 RealSense 가 환자 손을 본다 (이미 그 자세면 바로)
       └─ Selector
           ├─ 손 없음       손이 끝내 안 보이면 notes=no_hand 기록 → 시도 끝 (steps.no_hand)
           └─ 진행 (Sequence)
               ├─ 시작       레인 = 손바닥 y 에 가장 가까운 레인 (회의 결정 2, 무작위 아님)
               │             목표 거리 = 난이도로 (결정 3): 처음엔 환자 가까이, 잘 받으면 한 칸씩 로봇 쪽으로
               ├─ PushBall   target + push_speed 0 → 로봇이 레인 레시피로 속도. 로봇은 친 뒤 손 보기 자세에서 끝남
               ├─ 받기 확인  환자 손이 쥐었다(grip_closure ≥ catch_grab_closure) 폈다(≤ catch_release_closure) → 잘 받음 (결정 4)
               │             catch_timeout_s 안에 없으면 not_caught. 이 결과가 시도의 성공/실패 → 규칙 레이어가 난이도 조절
               ├─ 손 빼기    손이 중앙선에서 hand_clear_m 밖으로 갈 때까지 (안전감시 SLOW/STOP 에 안 걸리게). 넘으면 안내 후 그냥 진행
               ├─ ReturnBall 레인 안 공을 집어 거치대 맨 윗부분에 (로봇이 닿는 곳에 올 때까지 기다림, out_of_reach 면 굴려 달라고 안내)
               │             실패해도 시도 결과는 그대로 — 운영자에게 거치대에 넣어 달라고 안내
               └─ 기록       받기 결과로 한 번
  시도 수 = skills.yaml mode1.balls_per_cycle (session_tree.TRIAL_COUNTS). 공을 돌려놓으니 거치대 공 수와 상관없이 계속 가능.

목표 거리 (#84): 레인 입구에서 d = brain_target_start_m − 난이도 × brain_target_step_m, brain_target_range_m 으로 자름.
  하한 0.27 m: 공이 중앙선에 너무 가까이 서면 환자가 잡으러 올 때 손이 중앙선 15 cm 안 → 안전감시 STOP (safety.yaml stop_distance_m).
  레인에 레시피가 없으면 고정 속도 brain_push_speed_mps, target 비움.
손 위치: SceneContext.hand (HandState, header.frame_id = base_link 일 때만 레인 고르기에 씀. 아니면 무작위 + 경고).
TODO(조정묵): 받기 판정 임계값·시간은 실제 손 인식 데이터로 조정. 공을 쥔 손이 주먹(GESTURE_FIST → 비상정지)으로 잡히는지 정동휘와 확인.
"""
from __future__ import annotations

import math
import random
import time
from typing import TYPE_CHECKING, Any

import py_trees
from std_srvs.srv import Trigger

from rally_brain.behaviours import Do, Guard, ServiceCall, WaitUntil
from rally_brain.trees.steps import HAND_DEFAULTS, TrialState, no_hand, reason_code, skill_step
from rally_interfaces.action import PushBall, ReturnBall
from rally_interfaces.msg import TrialResult

if TYPE_CHECKING:
    from rally_brain.trees.session_tree import Session

ACTION = "/rally/skill/push_ball"
RETURN_ACTION = "/rally/skill/return_ball"
HAND_VIEW_SERVICE = "/rally/robot/go_hand_view"
DEFAULTS: dict[str, Any] = {
    **HAND_DEFAULTS,
    "brain_push_speed_mps": 0.3, "balls_per_cycle": 4, "push_speed_range": [0.10, 0.60],
    "brain_target_start_m": 0.40, "brain_target_step_m": 0.05, "brain_target_range_m": [0.27, 0.45],
    # 받기 확인 (손 쥐었다 폄)
    "catch_timeout_s": 20.0, "catch_grab_closure": 0.6, "catch_grab_hold_s": 0.3, "catch_release_closure": 0.3,
    "catch_prompt": "공을 잡았다가 놓아 주세요",
    # 손 빼기 → 공 회수
    "hand_clear_m": 0.25, "hand_clear_wait_s": 8.0, "hand_clear_prompt": "손을 잠깐 빼 주세요",
    "return_tries": 2, "return_timeout_s": 20.0,
    "out_of_reach_prompt": "공을 로봇 쪽으로 굴려 주세요",
    "return_fail_prompt": "공을 거치대에 올려 주세요",
}


def mode1_config(skills_cfg: dict) -> dict[str, Any]:
    cfg = {**DEFAULTS, **(skills_cfg.get("mode1") or {})}
    cfg["lane_count"] = len(cfg.get("lanes") or []) or 3
    return cfg


def trials_per_cycle(skills_cfg: dict) -> int:
    return max(1, int(mode1_config(skills_cfg)["balls_per_cycle"]))


def next_push_speed(session: Session, lane: int, cfg: dict[str, Any]) -> float:
    """레시피가 없을 때의 고정 속도 [m/s] (push_speed_range 로 자름)."""
    lo, hi = cfg["push_speed_range"]
    return min(max(float(cfg["brain_push_speed_mps"]), lo), hi)


def lane_recipe(cfg: dict[str, Any], lane: int) -> dict[str, float] | None:
    """skills.yaml mode1.lanes[lane].recipe {decel_mps2, d0_m} — 없으면 None (고정 속도로)."""
    lanes = cfg.get("lanes") or []
    r = lanes[lane].get("recipe") if 0 <= lane < len(lanes) else None
    return r if r and r.get("decel_mps2") else None


def target_distance(level: int, cfg: dict[str, Any]) -> float:
    """난이도 → 레인 입구에서 공이 멈출 거리 [m]. 난이도가 오를수록 짧게(환자에게서 멀리)."""
    lo, hi = cfg["brain_target_range_m"]
    return min(max(cfg["brain_target_start_m"] - level * cfg["brain_target_step_m"], lo), hi)


def target_point(cfg: dict[str, Any], lane: int, d: float) -> tuple[float, float]:
    """레인 입구 + 레인 방향 d → base_link (x, y)."""
    ln = cfg["lanes"][lane]
    a = math.radians(float(ln.get("dir_deg", 0.0)))
    return float(ln["start"][0]) + d * math.cos(a), float(ln["start"][1]) + d * math.sin(a)


def recipe_speed(recipe: dict[str, float], d: float, cfg: dict[str, Any]) -> float:
    """로봇과 같은 식 v = √(2a(d − d0)) — 기록용 예상 속도 (push_speed_range 로 자름)."""
    lo, hi = cfg["push_speed_range"]
    v = math.sqrt(max(2.0 * float(recipe["decel_mps2"]) * (d - float(recipe.get("d0_m", 0.0))), 0.0))
    return min(max(v, lo), hi)


def lane_for_hand(cfg: dict[str, Any], hand: Any) -> int | None:
    """손바닥 y 에 가장 가까운 레인 (회의 결정 2). 손이 없거나 base_link 좌표가 아니면 None."""
    lanes = cfg.get("lanes") or []
    if hand is None or not hand.detected or hand.header.frame_id != "base_link" or not lanes:
        return None
    y = hand.palm_pose.position.y
    if not math.isfinite(y):
        return None
    return min(range(len(lanes)), key=lambda i: abs(float(lanes[i]["start"][1]) - y))


def push_goal(lane: int, speed: float, target: tuple[float, float] | None = None) -> PushBall.Goal:
    # target 있음 → push_speed 0: 로봇이 레인 레시피로 속도. target 없음 → 그 속도로. 접촉 임계 0 = skills.yaml 기본
    if target is None:
        return PushBall.Goal(lane=lane, push_speed=float(speed))
    goal = PushBall.Goal(lane=lane, push_speed=0.0)
    goal.target.header.frame_id = "base_link"
    goal.target.point.x, goal.target.point.y = float(target[0]), float(target[1])
    return goal


def measured_error_mm(result: PushBall.Result) -> float:
    p = result.stop_position
    return result.error_mm if (p.x, p.y, p.z) != (0.0, 0.0, 0.0) else math.nan


def trial_from_result(lane: int, speed: float, result: PushBall.Result | None, notes: str,
                      success: bool = False, target_d: float = math.nan, level: int = 0,
                      lane_by_hand: bool = False, returned: bool = False) -> TrialResult:
    trial = TrialResult(mode=TrialResult.MODE_PUSH_BALL, success=success, notes=notes,
                        error_mm=measured_error_mm(result) if result is not None else math.nan,
                        param_keys=["lane", "push_speed", "target_d_m", "level"],
                        param_values=[float(lane), float(speed), float(target_d), float(level)])
    keys, values = ["lane_by_hand", "ball_returned"], [float(lane_by_hand), float(returned)]
    if result is not None:
        p = result.stop_position
        keys += ["contact_force_peak", "stop_x_m", "stop_y_m"]
        values += [float(result.contact_force_peak), float(p.x), float(p.y)]
    trial.metric_keys, trial.metric_values = keys, values
    return trial


def random_lane(cfg: dict[str, Any], rng: random.Random) -> int:
    return rng.randrange(cfg["lane_count"])


class CatchWatch:
    """손이 쥐었다(grip ≥ grab, hold_s 이상) 폈다(grip ≤ release) → caught. update(hand, now) 를 매 틱 부른다."""

    def __init__(self, cfg: dict[str, Any]):
        self.cfg, self.grab_since, self.grabbed, self.caught = cfg, None, False, False

    def update(self, hand: Any, now: float) -> bool:
        if hand is None or not hand.detected:
            self.grab_since = None if not self.grabbed else self.grab_since
            return self.caught
        g = float(hand.grip_closure)
        if not self.grabbed:
            if g >= self.cfg["catch_grab_closure"]:
                self.grab_since = self.grab_since or now
                if now - self.grab_since >= self.cfg["catch_grab_hold_s"]:
                    self.grabbed = True
            else:
                self.grab_since = None
        elif g <= self.cfg["catch_release_closure"]:
            self.caught = True
        return self.caught


def hand_clear(session: Session, cfg: dict[str, Any]) -> bool:
    """손이 안 보이거나 중앙선에서 hand_clear_m 밖 (NaN = 거리 모름 → 아직 안 빠진 것으로)."""
    ctx = session.get_context()
    if ctx is None or not ctx.hand.detected:
        return True
    d = ctx.hand.distance_to_centerline
    return math.isfinite(d) and d >= cfg["hand_clear_m"]


def build_trial(session: Session, rng: random.Random | None = None) -> py_trees.behaviour.Behaviour:
    cfg = mode1_config(session.skills_cfg)
    state = TrialState(session)
    client = session.action_client(PushBall, ACTION)
    return_client = session.action_client(ReturnBall, RETURN_ACTION)
    log = session.node.get_logger()
    rng = rng or random.Random()

    def start() -> None:
        state.start()
        ctx = session.get_context()
        lane = lane_for_hand(cfg, ctx.hand if ctx is not None else None)
        by_hand = lane is not None
        if lane is None:
            lane = random_lane(cfg, rng)
            log.warn(f"[모드 1] 손 위치(base_link)를 못 받아 레인 {lane} 무작위")
        level = session.difficulty.get(TrialResult.MODE_PUSH_BALL, 0)
        recipe = lane_recipe(cfg, lane)
        if recipe:
            d = target_distance(level, cfg)
            target = target_point(cfg, lane, d)
            speed = recipe_speed(recipe, d, cfg)
            log.info(f"[모드 1] 손 → 레인 {lane}{'' if by_hand else '(무작위)'} · 난이도 {level} → 입구에서 {d * 100:.0f} cm "
                     f"(예상 {speed:.2f} m/s, 레시피)")
        else:
            d, target = math.nan, None
            speed = next_push_speed(session, lane, cfg)
            log.info(f"[모드 1] 레인 {lane} · 속도 {speed:.2f} m/s (레시피 없음 → 고정 속도)")
        state.data.update(lane=lane, speed=speed, target=target, target_d=d, level=level, by_hand=by_hand,
                          push=None, caught=None, returned=False, notes="", watch=CatchWatch(cfg))

    def record() -> None:
        d = state.data
        caught = bool(d.get("caught"))
        notes = d.get("notes") or ("" if caught else "not_caught")
        state.finish(trial_from_result(d["lane"], d["speed"], d.get("push"), notes, caught,
                                       d["target_d"], d["level"], d["by_hand"], d["returned"]))

    def on_push_fail(result: PushBall.Result | None, reason: str, _k: int) -> None:
        log.warn(f"[모드 1] 밀기 실패: {reason}")
        state.data.update(push=result, notes=reason_code(reason))
        record()

    def on_interrupt(reason: str) -> None:
        state.data["notes"] = reason_code(reason)
        record()

    push = skill_step(state, "밀기", client, lambda: push_goal(state.data["lane"], state.data["speed"], state.data["target"]),
                      on_success=lambda r, _k: state.data.update(push=r),
                      on_fail=on_push_fail, on_interrupt=on_interrupt)

    # ── 받기 확인 ──
    def catch_tick() -> bool:
        ctx = session.get_context()
        return state.data["watch"].update(ctx.hand if ctx is not None else None, time.monotonic())

    def caught() -> None:
        state.data["caught"] = True
        log.info("[모드 1] 환자가 공을 받았다 놓음 → 잘 받음")

    def not_caught() -> bool:
        state.data["caught"] = False
        log.info(f"[모드 1] {cfg['catch_timeout_s']:.0f} s 안에 받기(쥐었다 폄) 없음")
        return True                    # 시도는 계속 — 공 회수로

    catch = py_trees.composites.Selector("받기 확인", memory=True, children=[
        py_trees.composites.Sequence("받음", memory=True, children=[
            WaitUntil("쥐었다 폄 대기", catch_tick, max(0.1, cfg["catch_timeout_s"])), Do("잘 받음", caught)]),
        Do("못 받음", not_caught),
    ])

    # ── 손 빼기 ──
    clear = py_trees.composites.Selector("손 빼기", memory=True, children=[
        WaitUntil("손 빠짐 대기", lambda: hand_clear(session, cfg), max(0.1, cfg["hand_clear_wait_s"] / 2.0)),
        py_trees.composites.Sequence("안내 후 대기", memory=True, children=[
            Do("손 빼기 안내", lambda: session.say(cfg["hand_clear_prompt"])),
            py_trees.composites.Selector("다시 대기", memory=True, children=[
                WaitUntil("손 빠짐 대기 2", lambda: hand_clear(session, cfg), max(0.1, cfg["hand_clear_wait_s"] / 2.0)),
                Do("그냥 진행", lambda: log.warn("[모드 1] 손이 아직 가까움 — 안전감시가 로봇을 늦추거나 멈춘다")),
            ]),
        ]),
    ])

    # ── 공 회수 ──
    prompted = {"t": None}

    def on_return_feedback(fb: Any) -> None:
        if fb.phase == "out_of_reach" and prompted["t"] is None:
            prompted["t"] = True
            session.say(cfg["out_of_reach_prompt"])

    def returned(_r: Any, _k: int) -> None:
        state.data["returned"] = True

    def return_failed(_r: Any, reason: str, _k: int) -> None:
        log.warn(f"[모드 1] 공 회수 실패: {reason} — 운영자에게 거치대에 넣어 달라고 안내")
        session.say(cfg["return_fail_prompt"])

    ret = skill_step(state, "공 회수", return_client,
                     lambda: ReturnBall.Goal(lane=int(state.data["lane"]), timeout_s=float(cfg["return_timeout_s"])),
                     tries=int(cfg["return_tries"]),
                     on_success=returned, on_fail=return_failed, on_interrupt=on_interrupt,
                     can_retry=lambda r: reason_code(r) in ("ball_out_of_reach", "no_ball_in_lane"),
                     on_retry=lambda _r, _k: session.say(cfg["out_of_reach_prompt"]),
                     on_feedback=on_return_feedback)

    def run_return() -> bool:
        prompted["t"] = None
        return True

    def on_stop() -> None:
        if state.started and not state.recorded:
            if state.data.get("caught") is not None:
                record()                       # 받기까지 끝났으면 그 결과로 (회수 도중 멈춤)
            else:
                state.data["notes"] = "interrupted"
                record()

    # 손 보기 자세 — 서비스 클라이언트가 없으면(테스트) 건너뜀. 실패(STOP 등)해도 손 대기로 넘어감 — 손이 안 보이면 no_hand
    view_steps: list[py_trees.behaviour.Behaviour] = []
    if session.service_client is not None:
        view_steps.append(py_trees.composites.Selector("손 보기 자세", memory=True, children=[
            ServiceCall("손 보기 자세로", session.service_client(Trigger, HAND_VIEW_SERVICE), Trigger.Request, timeout_s=30.0),
            Do("손 보기 실패", lambda: log.warn("[모드 1] 손 보기 자세로 못 감 — 지금 자세에서 손 대기")),
        ]))

    proceed = py_trees.composites.Sequence("진행", memory=True, children=[
        Do("시작", start), push,
        Do("받기 안내", lambda: session.say(cfg["catch_prompt"]) if cfg["catch_prompt"] else None),
        catch, clear, Do("회수 준비", run_return),
        py_trees.composites.Selector("공 회수", memory=True, children=[ret, Do("회수 실패 무시", lambda: None)]),
        Do("기록", record),
    ])
    return Guard("모드 1 시도", py_trees.composites.Sequence("모드 1", memory=True, children=[
        *view_steps,
        py_trees.composites.Selector("손 확인", memory=True, children=[
            no_hand(session, cfg, TrialResult.MODE_PUSH_BALL),
            py_trees.composites.Selector("진행 또는 끝", memory=True, children=[proceed, Do("끝", lambda: not state.fatal)]),
        ]),
    ]), on_stop)
