"""모드 1: 신선한 손 → 레인 목표 → 실측 레시피 속도 → PushBall → 시도 기록.

중앙선은 table.yaml. 목표와 양수 속도를 함께 전달하며 LLM은 수치를 정하지 않는다.
손 미검출은 제한 시간 대기 후 no_hand, 레시피/도달 범위 오류는 goal 없이 기록한다.
스킬 success는 타격 성공일 뿐 환자의 잡기 성공이 아니다. 자동 난이도 학습에 쓰지 않는다.
"""
from __future__ import annotations

import math
import random
from typing import TYPE_CHECKING, Any

import py_trees

from std_srvs.srv import Trigger
from rally_brain.config import load_safety
from rally_brain.behaviours import Do, Guard, ServiceCall, WaitUntil
from rally_brain.trees.steps import HAND_DEFAULTS, TrialState, no_hand, reason_code, skill_step
from rally_interfaces.action import PushBall, ReturnBall
from rally_interfaces.msg import HandState, TrialResult

if TYPE_CHECKING:
    from rally_brain.trees.session_tree import Session

ACTION = "/rally/skill/push_ball"
RETURN_ACTION = "/rally/skill/return_ball"
HAND_VIEW_SERVICE = "/rally/robot/go_hand_view"
DEFAULTS: dict[str, Any] = {**HAND_DEFAULTS, "balls_per_cycle": 4, "push_speed_range": [0.05, 0.35],
                           "hand_max_age_s": 1.0, "patient_margin_m": 0.05}


def mode1_config(skills_cfg: dict) -> dict[str, Any]:
    cfg = {**DEFAULTS, **(skills_cfg.get("mode1") or {})}
    safety = load_safety()
    cfg["patient_margin_m"] = max(float(cfg["patient_margin_m"]), float(safety["stop_distance_m"]) + 0.05)
    cfg["hand_clear_m"] = float(safety["slow_distance_m"])
    cfg["lane_count"] = len(cfg.get("lanes") or []) or 3
    return cfg


def trials_per_cycle(skills_cfg: dict) -> int:
    return max(1, int(mode1_config(skills_cfg)["balls_per_cycle"]))


def valid_hand(hand: HandState, now_ns: int, max_age_s: float) -> bool:
    stamp = hand.header.stamp.sec * 1_000_000_000 + hand.header.stamp.nanosec
    p = hand.palm_pose.position
    return (hand.detected and hand.header.frame_id == "base_link" and stamp > 0
            and 0 <= now_ns - stamp <= max_age_s * 1e9
            and all(math.isfinite(v) for v in (p.x, p.y, p.z)))


def lane_geometry(lane_cfg: dict) -> tuple[float, float, float, float]:
    try:
        sx, sy = map(float, lane_cfg["start"][:2])
        angle = math.radians(float(lane_cfg["dir_deg"]))
        dx, dy = math.cos(angle), math.sin(angle)
        if not all(math.isfinite(v) for v in (sx, sy, dx, dy)) or dx <= 1e-6:
            raise ValueError
        return sx, sy, dx, dy
    except (KeyError, TypeError, ValueError, OverflowError) as e:
        raise ValueError("invalid_lane") from e


def reach_target(hand: HandState, lane_cfg: dict, reach_m: float, centerline_x: float,
                 margin_m: float = 0.05) -> tuple[float, float] | None:
    """손 위치를 레인 축에 투영한 뒤 reach만큼 앞을 목표로 한다. 중앙선+여유보다 환자 쪽으로 제한."""
    p = hand.palm_pose.position
    if (not hand.detected or hand.header.frame_id != "base_link"
            or not all(math.isfinite(v) for v in (p.x, p.y, p.z, reach_m, centerline_x, margin_m))
            or reach_m < 0 or margin_m < 0.05):
        return None
    sx, sy, dx, dy = lane_geometry(lane_cfg)
    hand_d = (p.x - sx) * dx + (p.y - sy) * dy
    minimum = (centerline_x + margin_m - sx) / dx
    if hand_d < minimum:
        return None
    d = max(hand_d - reach_m, minimum)
    return sx + d * dx, sy + d * dy


def plan_push(hand: HandState, lane: int, level: int, cfg: dict, table: dict) -> dict:
    """검증된 거리·속도·중앙선·손 앞의 교집합에서만 목표를 만든다. 오류면 goal 금지."""
    try:
        ln = cfg["lanes"][lane]
        recipe = ln.get("recipe")
        if not recipe:
            raise ValueError("no_recipe")
        a, d0, rmse = (float(recipe[k]) for k in ("decel_mps2", "d0_m", "rmse_m"))
        dmin, dmax = map(float, recipe["distance_range_m"])
        measured_lo, measured_hi = map(float, recipe["speed_range_mps"])
        allowed_lo, allowed_hi = map(float, cfg["push_speed_range"])
        brain_lo, brain_hi = map(float, cfg["brain_speed_range_mps"])
        center = float(table["centerline_x_m"])
        margin = float(cfg["patient_margin_m"])
        factor = float(cfg["reach_rmse_multiplier"])
        values = (a, d0, rmse, dmin, dmax, measured_lo, measured_hi, allowed_lo, allowed_hi, brain_lo, brain_hi, center, margin, factor)
        if (not all(math.isfinite(v) for v in values) or a <= 0 or rmse <= 0 or factor <= 0
                or not 0 <= dmin < dmax or not 0 < measured_lo < measured_hi
                or not 0 < allowed_lo < allowed_hi or not 0 < brain_lo < brain_hi or margin < 0.05):
            raise ValueError("invalid_recipe")
        sx, sy, dx, dy = lane_geometry(ln)
        reach = (max(0, level) + 1) * factor * rmse
        target = reach_target(hand, ln, reach, center, margin)
        if target is None:
            raise ValueError("invalid_hand_target")
        p = hand.palm_pose.position
        hand_d = (p.x - sx) * dx + (p.y - sy) * dy
        vlo, vhi = max(measured_lo, allowed_lo, brain_lo), min(measured_hi, allowed_hi, brain_hi)
        if vlo > vhi:
            raise ValueError("unreachable_target")
        lo = max(dmin, d0 + vlo * vlo / (2 * a), (center + margin - sx) / dx)
        hi = min(dmax, d0 + vhi * vhi / (2 * a), hand_d)
        if lo > hi:
            raise ValueError("unreachable_target")
        wanted = (target[0] - sx) * dx + (target[1] - sy) * dy
        d = min(max(wanted, lo), hi)
        speed = min(max(math.sqrt(2 * a * (d - d0)), vlo), vhi)
        return {"lane": lane, "speed": speed, "target": (sx + d * dx, sy + d * dy),
                "target_d": d, "level": max(0, level), "reach_requested": reach,
                "reach_actual": hand_d - d, "hand_stamp_ns": hand.header.stamp.sec * 1_000_000_000 + hand.header.stamp.nanosec}
    except (KeyError, TypeError, IndexError, OverflowError) as e:
        raise ValueError("invalid_recipe") from e


def push_goal(lane: int, speed: float, target: tuple[float, float]) -> PushBall.Goal:
    goal = PushBall.Goal(lane=lane, push_speed=float(speed))
    goal.target.header.frame_id = "base_link"
    goal.target.point.x, goal.target.point.y = target
    return goal


def measured_error_mm(result: PushBall.Result) -> float:
    p = result.stop_position
    return result.error_mm if all(math.isfinite(v) for v in (p.x, p.y, p.z)) and (p.x, p.y, p.z) != (0, 0, 0) else math.nan


def trial_from_result(plan: dict, result: PushBall.Result | None, notes: str, success: bool = False) -> TrialResult:
    keys = ["lane", "push_speed", "target_d_m", "level", "reach_requested_m", "reach_actual_m", "target_x_m", "target_y_m"]
    target = plan.get("target", (math.nan, math.nan))
    values = [plan.get("lane", -1), plan.get("speed", math.nan), plan.get("target_d", math.nan),
              plan.get("level", 0), plan.get("reach_requested", math.nan), plan.get("reach_actual", math.nan), *target]
    trial = TrialResult(mode=TrialResult.MODE_PUSH_BALL, success=success, notes=notes,
                        error_mm=measured_error_mm(result) if result is not None else math.nan,
                        param_keys=keys, param_values=[float(v) for v in values])
    if result is not None:
        p = result.stop_position
        measured = all(math.isfinite(v) for v in (p.x, p.y, p.z)) and (p.x, p.y, p.z) != (0, 0, 0)
        trial.metric_keys = ["contact_force_peak", "stop_x_m", "stop_y_m"]
        trial.metric_values = [float(result.contact_force_peak), float(p.x) if measured else math.nan,
                               float(p.y) if measured else math.nan]
    return trial


def lane_for_hand(cfg: dict[str, Any], hand: Any) -> int | None:
    """손바닥 y 에 가장 가까운 레인 (회의 결정 2). 손이 없거나 base_link 좌표가 아니면 None."""
    lanes = cfg.get("lanes") or []
    if hand is None or not hand.detected or hand.header.frame_id != "base_link" or not lanes:
        return None
    y = hand.palm_pose.position.y
    if not math.isfinite(y):
        return None
    return min(range(len(lanes)), key=lambda i: abs(float(lanes[i]["start"][1]) - y))


class CatchWatch:
    """새 센서 표본으로 펼침→쥠 유지→펼침 확인. 공 접촉 증거가 아닌 후보 신호."""
    def __init__(self, cfg: dict[str, Any], after_ns: int):
        self.cfg, self.last_stamp = cfg, after_ns
        self.open_seen = self.grabbed = self.caught = False
        self.grab_since = None

    def update(self, hand: Any, now_ns: int) -> bool:
        if hand is None or not valid_hand(hand, now_ns, self.cfg["hand_max_age_s"]):
            self.open_seen = self.grabbed = False
            self.grab_since = None
            return False
        stamp = hand.header.stamp.sec * 1_000_000_000 + hand.header.stamp.nanosec
        if stamp <= self.last_stamp:
            return self.caught
        if stamp - self.last_stamp > self.cfg["hand_max_age_s"] * 1e9:
            self.open_seen = self.grabbed = False
            self.grab_since = None
        self.last_stamp = stamp
        g = float(hand.grip_closure)
        if not math.isfinite(g) or not 0 <= g <= 1:
            self.open_seen = self.grabbed = False
            self.grab_since = None
            return False
        if g <= self.cfg["catch_release_closure"]:
            if self.grabbed:
                self.caught = True
            self.open_seen = True
            self.grab_since = None
        elif self.open_seen and g >= self.cfg["catch_grab_closure"]:
            if self.grab_since is None:
                self.grab_since = stamp
            if stamp - self.grab_since >= self.cfg["catch_grab_hold_s"] * 1e9:
                self.grabbed = True
        elif not self.grabbed:
            self.grab_since = None
        return self.caught


def hand_clear(session: Session, cfg: dict[str, Any]) -> bool:
    """신선한 base_link 손의 거리로만 회수 허용. 미검출/NaN은 이탈 확인이 아니다."""
    ctx = session.get_context()
    if ctx is None or not session.safe():
        return False
    h = ctx.hand
    return (valid_hand(h, session.node.get_clock().now().nanoseconds, cfg["hand_max_age_s"])
            and math.isfinite(h.distance_to_centerline) and h.distance_to_centerline >= cfg["hand_clear_m"])


def build_trial(session: Session, rng: random.Random | None = None) -> py_trees.behaviour.Behaviour:
    cfg = mode1_config(session.skills_cfg)
    state = TrialState(session)
    client = session.action_client(PushBall, ACTION)
    return_client = session.action_client(ReturnBall, RETURN_ACTION)
    log = session.node.get_logger()
    rng = rng or random.Random()

    def hand_ready() -> bool:
        ctx = session.get_context()
        return ctx is not None and valid_hand(ctx.hand, session.node.get_clock().now().nanoseconds, cfg["hand_max_age_s"])

    def start() -> bool:
        state.start()
        state.data.update(caught=None, returned=False, notes="", push=None)
        if not session.safe() or not hand_ready():
            state.data["notes"] = "no_hand"
            record()
            return False
        lane = lane_for_hand(cfg, session.get_context().hand)
        try:
            if lane is None:
                raise ValueError("invalid_lane")
            plan = plan_push(session.get_context().hand, lane, session.difficulty.get(1, 0), cfg, session.table_cfg)
        except ValueError as e:
            state.data["notes"] = str(e)
            record()
            return False
        state.data.update(plan)
        log.info(f"[모드 1] 손 기준 레인 {lane} · 목표 {plan['target_d']:.3f} m · 속도 {plan['speed']:.3f} m/s")
        return True

    def record() -> None:
        d = state.data
        caught = bool(d.get("caught"))
        notes = d.get("notes") or ("" if caught else "not_caught")
        trial = trial_from_result(d, d.get("push"), notes, caught)
        trial.metric_keys.extend(["lane_by_hand", "ball_returned", "catch_gesture_detected"])
        trial.metric_values.extend([float("lane" in d), float(d.get("returned", False)), float(caught)])
        state.finish(trial)

    def on_push_fail(result: PushBall.Result | None, reason: str, _k: int) -> None:
        log.warn(f"[모드 1] 밀기 실패: {reason}")
        state.data.update(push=result, notes=reason_code(reason))
        record()

    def on_interrupt(reason: str) -> None:
        state.data["notes"] = reason_code(reason)
        record()

    push = skill_step(state, "밀기", client, lambda: push_goal(state.data["lane"], state.data["speed"], state.data["target"]),
                      on_success=lambda r, _k: state.data.update(push=r),
                      can_send=lambda: session.safe() and hand_ready() and
                          0 <= session.node.get_clock().now().nanoseconds - state.data["hand_stamp_ns"] <= cfg["hand_max_age_s"] * 1e9,
                      on_fail=on_push_fail, on_interrupt=on_interrupt)

    # ── 받기 확인 ──
    def catch_tick() -> bool:
        ctx = session.get_context()
        return state.data["watch"].update(ctx.hand if ctx is not None else None, session.node.get_clock().now().nanoseconds)

    def caught() -> None:
        state.data["caught"] = True
        log.info("[모드 1] 펼침→쥠 유지→펼침 관측 (받기 후보)")

    def not_caught() -> bool:
        state.data["caught"] = False
        log.info(f"[모드 1] {cfg['catch_timeout_s']:.0f} s 안에 받기(쥐었다 폄) 없음")
        return True                    # 시도는 계속 — 공 회수로

    catch = py_trees.composites.Selector("받기 확인", memory=True, children=[
        py_trees.composites.Sequence("받음", memory=True, children=[
            WaitUntil("쥐었다 폄 대기", catch_tick, max(0.1, cfg["catch_timeout_s"])), Do("잘 받음", caught)]),
        Do("못 받음", not_caught),
    ])

    # 손 이탈 대기는 세션을 끝내지 않는다. 안내 후에도 새 관측을 기다린다.
    clear = py_trees.composites.Selector("손 빼기", memory=True, children=[
        WaitUntil("손 빠짐 대기", lambda: hand_clear(session, cfg), max(0.1, cfg["hand_clear_wait_s"])),
        py_trees.composites.Sequence("안내 후 계속 대기", memory=True, children=[
            Do("손 빼기 안내", lambda: session.say(cfg["hand_clear_prompt"])),
            WaitUntil("손 이탈 확인 대기", lambda: hand_clear(session, cfg)),
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
        state.data["notes"] = "return_" + reason_code(reason)
        state.fatal = True

    ret = skill_step(state, "공 회수", return_client,
                     lambda: ReturnBall.Goal(lane=int(state.data["lane"]), timeout_s=float(cfg["return_timeout_s"])),
                     tries=int(cfg["return_tries"]),
                     can_send=lambda: hand_clear(session, cfg), wait_for_precondition=True,
                     on_success=returned, on_fail=return_failed, on_interrupt=on_interrupt,
                     can_retry=lambda r: reason_code(r) in ("ball_out_of_reach", "no_ball_in_lane"),
                     on_retry=lambda _r, _k: session.say(cfg["out_of_reach_prompt"]),
                     on_feedback=on_return_feedback)

    def run_return() -> bool:
        prompted["t"] = None
        return True

    def on_stop() -> None:
        if state.started and not state.recorded:
            state.data["notes"] = "interrupted"
            record()

    # 자세 이동 실패/timeout에서 계속하지 않는다. 진행 중에는 모드 전환도 막는다.
    def view_ready() -> None:
        state.started = False
        session.trial_active = False

    view_steps = []
    if session.service_client is not None:
        def view_failed() -> bool:
            state.data.update(caught=False, returned=False, notes="hand_view_failed")
            record()
            state.fatal = True
            return False
        view_steps = [Do("자세 이동 시작", state.start), py_trees.composites.Selector("손 보기 자세", memory=True, children=[
            ServiceCall("손 보기 자세로", session.service_client(Trigger, HAND_VIEW_SERVICE), Trigger.Request, timeout_s=30.0),
            Do("손 보기 실패", view_failed),
        ]), Do("손 대기 준비", view_ready)]

    def begin_watch() -> None:
        state.data["watch"] = CatchWatch(cfg, session.node.get_clock().now().nanoseconds)

    recover = py_trees.composites.Sequence("손 이탈 후 회수", memory=True, children=[
        clear, Do("회수 준비", run_return),
        py_trees.composites.Selector("회수 결과 처리", memory=True, children=[
            ret, Do("회수 실패 기록으로", lambda: None),
        ]),
    ])
    proceed = py_trees.composites.Sequence("진행", memory=True, children=[
        Do("시작", start), push, Do("받기 관측 시작", begin_watch),
        Do("받기 안내", lambda: session.say(cfg["catch_prompt"]) if cfg["catch_prompt"] else None),
        catch, recover, Do("기록", record), Do("다음 시도 가능", lambda: not state.fatal),
    ])
    return Guard("모드 1 시도", py_trees.composites.Sequence("모드 1", memory=True, children=[
        *view_steps,
        py_trees.composites.Selector("손 확인", memory=True, children=[
            no_hand(session, cfg, TrialResult.MODE_PUSH_BALL, hand_ready),
            py_trees.composites.Selector("진행 또는 끝", memory=True, children=[proceed, Do("끝", lambda: not state.fatal)]),
        ]),
    ]), on_stop)
