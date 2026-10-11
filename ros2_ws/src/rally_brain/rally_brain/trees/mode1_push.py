"""모드 1: 레인별 고정 목표 → 손 시작 위치·NORMAL 대기 → PushBall → 받기·회수·기록.

중앙선은 table.yaml. 목표와 양수 속도를 함께 전달하며 LLM은 수치를 정하지 않는다.
손 미검출은 제한 시간 대기 후 no_hand, 레시피/도달 범위 오류는 goal 없이 기록한다.
받기 후보는 실제 공 접촉 증거가 아니다. 자동 난이도 학습에 쓰지 않는다.
"""
from __future__ import annotations

import math
import random
import time
from typing import TYPE_CHECKING, Any

import py_trees

from std_srvs.srv import Trigger
from rally_brain.config import load_safety
from rally_brain.behaviours import Do, Guard, ServiceCall, WaitUntil
from rally_brain.trees.steps import HAND_DEFAULTS, TrialState, no_hand, reason_code, skill_step
from rally_interfaces.action import PushBall, ReturnBall
from rally_interfaces.msg import HandState, TrialResult, SafetyStatus

if TYPE_CHECKING:
    from rally_brain.trees.session_tree import Session

ACTION = "/rally/skill/push_ball"
RETURN_ACTION = "/rally/skill/return_ball"
HAND_VIEW_SERVICE = "/rally/robot/go_hand_view"
DEFAULTS: dict[str, Any] = {**HAND_DEFAULTS, "balls_per_cycle": 4, "push_speed_range": [0.05, 0.35],
                           "hand_max_age_s": 1.0, "patient_margin_m": 0.05,
                           "hand_clear_m": 0.20, "hand_clear_log_s": 2.0,
                           # 레인 결정: 손바닥이 hand_hold_tol_m 안에서 hand_hold_s 머물러야 (10/11 실로봇: 지나가는 손 한 프레임으로 레인 0 → 치기)
                           "hand_hold_s": 1.5, "hand_hold_tol_m": 0.04,
                           # 정지 위치: 받기 관측 중 손 보기 자세 카메라(SceneContext.objects)로, 손이 쥐기 전까지만 잰다
                           # still_s 짧게: 공은 로봇이 손 보기 자세에 오기 전에 이미 멈춰 있다 (10/8 시뮬: 치고 ≈ 4.6 s 뒤 도착)
                           "stop_still_s": 0.3, "stop_tol_m": 0.01, "stop_lane_halfwidth_m": 0.07,
                           "stop_min_along_m": 0.05, "stop_z_max_m": 0.10,
                           "pickup_class": "ball", "pickup_min_conf": 0.6, "vision_offset_m": [0.0, 0.0, 0.0]}


def mode1_config(skills_cfg: dict) -> dict[str, Any]:
    cfg = {**DEFAULTS, **(skills_cfg.get("mode1") or {})}
    safety = load_safety()
    cfg["patient_margin_m"] = max(float(cfg["patient_margin_m"]), float(safety["stop_distance_m"]) + 0.05)
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


def plan_push(hand: HandState, lane: int, level: int, cfg: dict, table: dict) -> dict:
    """레인별 고정 목표를 실측·속도·중앙선 범위로 제한. 난이도는 필요한 손 시작 거리."""
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
        p = hand.palm_pose.position
        hand_d = (p.x - sx) * dx + (p.y - sy) * dy
        vlo, vhi = max(measured_lo, allowed_lo, brain_lo), min(measured_hi, allowed_hi, brain_hi)
        if vlo > vhi:
            raise ValueError("unreachable_target")
        lo = max(dmin, d0 + vlo * vlo / (2 * a), (center + margin - sx) / dx)
        hi = min(dmax, d0 + vhi * vhi / (2 * a))
        if lo > hi:
            raise ValueError("unreachable_target")
        wanted = float(ln["target_d_m"])
        if not math.isfinite(wanted) or wanted < 0:
            raise ValueError("invalid_target")
        d = min(max(wanted, lo), hi)
        speed = min(max(math.sqrt(2 * a * (d - d0)), vlo), vhi)
        return {"lane": lane, "speed": speed, "target": (sx + d * dx, sy + d * dy),
                "target_d": d, "level": max(0, level), "reach_requested": reach,
                "reach_actual": hand_d - d, "hand_start_d": d + reach, "hand_stamp_ns": hand.header.stamp.sec * 1_000_000_000 + hand.header.stamp.nanosec}
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
    """plan["stop"] = brain 이 잰 정지 위치 (base m) 가 있으면 그것으로 error_mm·stop_x/y, 없으면 로봇 결과로 (0 이면 못 잼 → NaN, #69)."""
    keys = ["lane", "push_speed", "target_d_m", "level", "reach_requested_m", "reach_actual_m", "target_x_m", "target_y_m", "hand_start_d_m"]
    target = plan.get("target", (math.nan, math.nan))
    values = [plan.get("lane", -1), plan.get("speed", math.nan), plan.get("target_d", math.nan),
              plan.get("level", 0), plan.get("reach_requested", math.nan), plan.get("reach_actual", math.nan), *target, plan.get("hand_start_d", math.nan)]
    trial = TrialResult(mode=TrialResult.MODE_PUSH_BALL, success=success, notes=notes,
                        error_mm=measured_error_mm(result) if result is not None else math.nan,
                        param_keys=keys, param_values=[float(v) for v in values])
    stop = plan.get("stop")
    if stop is not None:
        trial.error_mm = math.dist(stop, target) * 1000.0 if all(map(math.isfinite, target)) else math.nan
    if result is not None:
        p = result.stop_position
        measured = all(math.isfinite(v) for v in (p.x, p.y, p.z)) and (p.x, p.y, p.z) != (0, 0, 0)
        if stop is None and measured:
            stop = (p.x, p.y)
        trial.metric_keys = ["contact_force_peak"]
        trial.metric_values = [float(result.contact_force_peak)]
    stop = stop if stop is not None else (math.nan, math.nan)
    trial.metric_keys = list(trial.metric_keys) + ["stop_x_m", "stop_y_m", "stop_d_m"]
    trial.metric_values = [*trial.metric_values, float(stop[0]), float(stop[1]), float(plan.get("stop_d", math.nan))]
    return trial


def lane_ball(cfg: dict[str, Any], objects: Any, lane: int) -> tuple[float, float, float] | None:
    """SceneContext.objects → 그 레인 통로 안에서 가장 멀리 간 공 (입구에서 거리, x, y) — 비전 보정(vision_offset_m) 뒤."""
    lanes = cfg.get("lanes") or []
    if objects is None or not 0 <= lane < len(lanes) or objects.header.frame_id not in ("base_link", ""):
        return None
    try:
        sx, sy, dx, dy = lane_geometry(lanes[lane])
    except ValueError:
        return None
    ox, oy, _ = (float(v) for v in cfg["vision_offset_m"])
    best = None
    for o in objects.objects:
        if o.class_name != cfg["pickup_class"] or o.confidence < cfg["pickup_min_conf"] or o.position.z > cfg["stop_z_max_m"]:
            continue
        x, y = o.position.x + ox, o.position.y + oy
        along = (x - sx) * dx + (y - sy) * dy
        lat = -(x - sx) * dy + (y - sy) * dx
        if along >= cfg["stop_min_along_m"] and abs(lat) <= cfg["stop_lane_halfwidth_m"] and (best is None or along > best[0]):
            best = (along, x, y)
    return best


class StopWatch:
    """add(t, p) — still_s 동안 tol_m 안에서 그대로면 그 위치 (트래커 is_moving 은 아직 안 믿음)."""

    def __init__(self, still_s: float, tol_m: float):
        self.still_s, self.tol_m, self.ref, self.since = still_s, tol_m, None, 0.0

    def add(self, t: float, p: tuple[float, float] | None) -> tuple[float, float] | None:
        if p is None:
            self.ref = None
            return None
        if self.ref is None or math.dist(p, self.ref) > self.tol_m:
            self.ref, self.since = p, t
            return None
        return self.ref if t - self.since >= self.still_s else None


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
    """새 센서 표본으로 쥠 유지(catch_grab_hold_s)→펼침 확인. 공 접촉 증거가 아닌 후보 신호.
    처음 펼침은 요구하지 않는다 — 10/8 실로봇: 손이 공을 위에서 움켜쥔 채 화면에 들어와 '펼침 먼저'를 못 봐서 받기를 놓침."""
    def __init__(self, cfg: dict[str, Any], after_ns: int):
        self.cfg, self.last_stamp = cfg, after_ns
        self.grabbed = self.caught = False
        self.grab_since = None

    def update(self, hand: Any, now_ns: int) -> bool:
        if hand is None or not valid_hand(hand, now_ns, self.cfg["hand_max_age_s"]):
            self.grabbed = False
            self.grab_since = None
            return False
        stamp = hand.header.stamp.sec * 1_000_000_000 + hand.header.stamp.nanosec
        if stamp <= self.last_stamp:
            return self.caught
        if stamp - self.last_stamp > self.cfg["hand_max_age_s"] * 1e9:
            self.grabbed = False
            self.grab_since = None
        self.last_stamp = stamp
        g = float(hand.grip_closure)
        if not math.isfinite(g) or not 0 <= g <= 1:
            self.grabbed = False
            self.grab_since = None
            return False
        if g <= self.cfg["catch_release_closure"]:
            if self.grabbed:
                self.caught = True
            self.grab_since = None
        elif g >= self.cfg["catch_grab_closure"]:
            if self.grab_since is None:
                self.grab_since = stamp
            if stamp - self.grab_since >= self.cfg["catch_grab_hold_s"] * 1e9:
                self.grabbed = True
        elif not self.grabbed:
            self.grab_since = None
        return self.caught


def hand_clear_block(session: Session, cfg: dict[str, Any]) -> str | None:
    """회수를 막는 이유 (None = 회수 가능). 신선한 base_link 손의 거리로만 허용 — 미검출/NaN은 이탈 확인이 아니다.
    기준 hand_clear_m 은 skills.yaml mode1 (safety.yaml slow_distance_m 과 별개, #104 에서 SLOW 거리가 바뀌어도 그대로)."""
    ctx = session.get_context()
    if ctx is None:
        return "상황판 없음"
    if not session.safe():
        return "안전 STOP"
    h = ctx.hand
    now_ns = session.node.get_clock().now().nanoseconds
    if not h.detected:
        return "손 안 보임"
    if h.header.frame_id != "base_link":
        return f"손 좌표가 base_link 아님({h.header.frame_id or '없음'})"
    if not valid_hand(h, now_ns, cfg["hand_max_age_s"]):
        return "손 관측이 오래됨·좌표 없음"
    d = h.distance_to_centerline
    if not math.isfinite(d):
        return "손 거리 모름(NaN)"
    if d < cfg["hand_clear_m"]:
        return f"손 거리 {d:.2f} m < {cfg['hand_clear_m']:.2f} m"
    return None


def hand_clear(session: Session, cfg: dict[str, Any]) -> bool:
    return hand_clear_block(session, cfg) is None


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
        state.data.update(caught=None, returned=False, notes="", push=None, stop=None, stop_d=math.nan,
                          hand_hold=StopWatch(cfg["hand_hold_s"], cfg["hand_hold_tol_m"]), logged_lane=None)
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
        log.info(f"[모드 1] 손 확인 (지금 레인 {lane}) — 손이 {cfg['hand_hold_s']:.1f} s 머물면 그 레인으로")
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
        state.fatal = True  # 공/그리퍼 상태가 불명확하므로 다음 타격 금지. 운영자 확인 필요.
        record()

    def on_interrupt(reason: str) -> None:
        state.data["notes"] = reason_code(reason)
        record()

    def ready_to_push() -> bool:
        ctx = session.get_context()
        ready = False
        prompt = cfg["start_wait_prompt"]
        p = ctx.hand.palm_pose.position if hand_ready() else None
        pos = None if p is None else (p.x, p.y)
        held = pos if cfg["hand_hold_s"] <= 0 else state.data["hand_hold"].add(time.monotonic(), pos)
        if held is not None and ctx.safety.level == SafetyStatus.LEVEL_NORMAL:
            try:
                lane = lane_for_hand(cfg, ctx.hand)
                plan = plan_push(ctx.hand, lane, session.difficulty.get(1, 0), cfg, session.table_cfg)
                state.data.update(plan)
                ready = plan["reach_actual"] >= plan["reach_requested"]
                if ready and state.data.get("logged_lane") != lane:   # ready_to_push 는 대기·전송 확인에서 두 번 불린다
                    state.data["logged_lane"] = lane
                    log.info(f"[모드 1] 손이 {cfg['hand_hold_s']:.1f} s 머묾 → 레인 {lane} · 목표 {plan['target_d']:.3f} m · "
                             f"속도 {plan['speed']:.3f} m/s")
                else:
                    prompt = cfg["hand_start_prompt"]
            except ValueError:
                # 최초 설정 오류는 start에서 기록. 대기 중 잘못된 입력은 명령으로 쓰지 않는다.
                ready = False
        if not ready and time.monotonic() >= state.data.get("next_start_prompt", 0):
            session.say(prompt)
            state.data["next_start_prompt"] = time.monotonic() + cfg["hand_clear_wait_s"]
        return ready

    push = skill_step(state, "밀기", client, lambda: push_goal(state.data["lane"], state.data["speed"], state.data["target"]),
                      on_success=lambda r, _k: state.data.update(push=r),
                      can_send=ready_to_push, wait_for_precondition=True,
                      on_fail=on_push_fail, on_interrupt=on_interrupt)

    # ── 받기 확인 ──
    def catch_tick() -> bool:
        ctx, d = session.get_context(), state.data
        if d.get("stop") is None and not d["watch"].grabbed and ctx is not None:   # 손이 쥐기 전까지만 정지 위치를 잰다
            b = lane_ball(cfg, ctx.objects, d["lane"])
            got = d["stop_watch"].add(time.monotonic(), (b[1], b[2]) if b else None)
            if got:
                d["stop"], d["stop_d"] = got, b[0]
                log.info(f"[모드 1] 공 정지 — 입구에서 {b[0] * 100:.1f} cm, 목표 {d['target_d'] * 100:.0f} cm · "
                         f"오차 {math.dist(got, d['target']) * 1000:.0f} mm")
        return d["watch"].update(ctx.hand if ctx is not None else None, session.node.get_clock().now().nanoseconds)

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
    # 기다리는 이유는 바뀔 때와 hand_clear_log_s 마다 로그 — 실기에서 회수가 안 될 때 원인(미검출·거리 등)을 보려고
    clear_log = {"reason": None, "t": 0.0}

    def clear_check() -> bool:
        reason = hand_clear_block(session, cfg)
        now = time.monotonic()
        if reason is None:
            if clear_log["reason"] is not None:
                log.info("[모드 1] 손 이탈 확인 — 회수 진행")
            clear_log["reason"] = None
            return True
        if reason != clear_log["reason"] or now - clear_log["t"] >= cfg["hand_clear_log_s"]:
            log.info(f"[모드 1] 회수 대기: {reason}")
            clear_log.update(reason=reason, t=now)
        return False

    clear = py_trees.composites.Selector("손 빼기", memory=True, children=[
        WaitUntil("손 빠짐 대기", clear_check, max(0.1, cfg["hand_clear_wait_s"])),
        py_trees.composites.Sequence("안내 후 계속 대기", memory=True, children=[
            Do("손 빼기 안내", lambda: session.say(cfg["hand_clear_prompt"])),
            WaitUntil("손 이탈 확인 대기", clear_check),
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
                     can_send=clear_check, wait_for_precondition=True,
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
        state.data.update(watch=CatchWatch(cfg, session.node.get_clock().now().nanoseconds),
                          stop=None, stop_d=math.nan, stop_watch=StopWatch(cfg["stop_still_s"], cfg["stop_tol_m"]))

    recover = py_trees.composites.Sequence("손 이탈 후 회수", memory=True, children=[
        clear, Do("회수 준비", run_return),
        py_trees.composites.Selector("회수 결과 처리", memory=True, children=[
            ret, Do("회수 실패 기록으로", lambda: None),
        ]),
    ])
    proceed = py_trees.composites.Sequence("진행", memory=True, children=[
        Do("시작", start), WaitUntil("손 시작 위치·NORMAL 대기", ready_to_push), push, Do("받기 관측 시작", begin_watch),
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
