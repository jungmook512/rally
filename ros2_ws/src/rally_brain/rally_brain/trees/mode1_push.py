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

from rally_brain.behaviours import Do, Guard
from rally_brain.trees.steps import HAND_DEFAULTS, TrialState, no_hand, reason_code, skill_step
from rally_interfaces.action import PushBall
from rally_interfaces.msg import HandState, TrialResult

if TYPE_CHECKING:
    from rally_brain.trees.session_tree import Session

ACTION = "/rally/skill/push_ball"
DEFAULTS: dict[str, Any] = {**HAND_DEFAULTS, "balls_per_cycle": 4, "push_speed_range": [0.05, 0.35],
                           "hand_max_age_s": 1.0, "patient_margin_m": 0.05}


def mode1_config(skills_cfg: dict) -> dict[str, Any]:
    cfg = {**DEFAULTS, **(skills_cfg.get("mode1") or {})}
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


def random_lane(cfg: dict[str, Any], rng: random.Random) -> int:
    return rng.randrange(cfg["lane_count"])


def build_trial(session: Session, rng: random.Random | None = None) -> py_trees.behaviour.Behaviour:
    cfg = mode1_config(session.skills_cfg)
    state = TrialState(session)
    client = session.action_client(PushBall, ACTION)
    log = session.node.get_logger()
    rng = rng or random.Random()

    def hand_ready() -> bool:
        ctx = session.get_context()
        return (ctx is not None and valid_hand(ctx.hand, session.node.get_clock().now().nanoseconds,
                                               cfg["hand_max_age_s"]))

    def record(result: PushBall.Result | None, notes: str, success: bool = False) -> None:
        state.finish(trial_from_result(state.data, result, notes, success))

    def prepare() -> bool:
        state.start()
        lane = random_lane(cfg, rng)
        state.data.update(lane=lane, level=session.difficulty.get(TrialResult.MODE_PUSH_BALL, 0))
        if not session.safe() or not hand_ready():
            record(None, "no_hand")
            return False
        try:
            state.data = plan_push(session.get_context().hand, lane, state.data["level"], cfg, session.table_cfg)
        except ValueError as e:
            reason = str(e)
            log.warn(f"[모드 1] 목표 계산 거절: {reason}")
            record(None, reason)
            return False
        log.info(f"[모드 1] 레인 {lane} · 난이도 {state.data['level']} · 목표 {state.data['target_d']:.3f} m · 속도 {state.data['speed']:.3f} m/s")
        return True

    def can_send() -> bool:
        # 서버 대기 중 오래된 목표를 보내지 않는다. 동작 시작 뒤 카메라가 움직일 때는 취소 조건으로 쓰지 않음.
        now = session.node.get_clock().now().nanoseconds
        return (session.safe() and hand_ready()
                and 0 <= now - state.data["hand_stamp_ns"] <= cfg["hand_max_age_s"] * 1e9)

    push = skill_step(state, "밀기", client,
                      lambda: push_goal(state.data["lane"], state.data["speed"], state.data["target"]),
                      can_send=can_send,
                      on_success=lambda r, _k: record(r, "", True),
                      on_fail=lambda r, reason, _k: record(r, reason_code(reason)),
                      on_interrupt=lambda reason: record(None, reason_code(reason)))

    def on_stop() -> None:
        if state.started and not state.recorded:
            record(None, "interrupted")

    return Guard("모드 1 시도", py_trees.composites.Selector("모드 1", memory=True, children=[
        no_hand(session, cfg, TrialResult.MODE_PUSH_BALL, hand_ready),
        py_trees.composites.Sequence("밀기", memory=True, children=[Do("목표 계산", prepare), push]),
        Do("끝", lambda: not state.fatal),
    ]), on_stop)
