"""rally_brain/rally_brain/trees/mode1_push.py — 모드 1 공 굴려주기: 시도 1회 서브트리
담당: 조정묵

  모드 1 시도 (Guard — 도중에 멈추면 interrupted 기록)
   └─ Selector
       ├─ 밀기 (Sequence)
       │   ├─ 시작          레인 무작위로 고르기 (환자가 다음 레인을 미리 알 수 없게, #75 리뷰), 목표 거리·속도 정하기
       │   └─ PushBall      로봇이 거치대 공을 비전으로 집고(x 가장 작은 공) 레인 입구에 놓고 친다
       │                    끝나면 기록: 성공 = 스킬 success, notes = 실패 이유 앞 낱말
       └─ 끝                 밀기가 실패로 기록됐으면 시도는 끝 (서버 없음이면 세션 실패)
  시도 수 = skills.yaml mode1.balls_per_cycle (거치대 공 개수, 기본 4) — session_tree.TRIAL_COUNTS.
  재시도는 없다: 실패 이유가 거치대 빔(no_ball_detected)·집기 실패 등 로봇 쪽이라 다시 해도 같다.

목표 거리 (10/8, 레시피 실측 후 #57):
  공이 멈출 자리 = 레인 입구에서 레인 방향으로 d. 클수록 환자 가까이 (환자 쪽이 +x).
  d = brain_target_start_m − 난이도 × brain_target_step_m, brain_target_range_m 으로 자름 (레시피를 잰 범위 밖은 외삽이라).
  처음(난이도 0)은 환자 가까이 멈추고, 잘 받으면(규칙 레이어 3연속 성공 → 어렵게) 한 칸씩 로봇 쪽으로 — 환자가 더 뻗어야 한다.
  goal = target(그 점, base_link) + push_speed 0 → 속도는 로봇이 레인 레시피(skills.yaml mode1.lanes[].recipe)로 정한다.
  기록용 push_speed 는 같은 레시피로 계산한 예상값. 레인에 레시피가 없으면 예전처럼 고정 속도 brain_push_speed_mps, target 비움.
  TODO(조정묵): '잘 받음' 판정 — 지금 성공 = 스킬 success(로봇이 쳤다). 환자가 공을 잡았는지(손 보기 자세 비전·손 인식)가 들어오면 그걸로.
  stop_position·error_mm 은 아직 로봇이 안 채운다 → 정지 위치가 (0,0,0) 이면 error_mm 을 NaN 으로 기록
  (0 mm 로 넣으면 "완벽한 시도"로 학습된다. #69: 못 잰 값 = NaN).
TODO(Day 5): next_push_speed 를 llm_advisor(SuggestParams) 로 — 오차가 채워진 뒤. 손 대기(mode5 와 같은 방식).
"""
from __future__ import annotations

import math
import random
from typing import TYPE_CHECKING, Any

import py_trees

from rally_brain.behaviours import Do, Guard
from rally_brain.trees.steps import TrialState, reason_code, skill_step
from rally_interfaces.action import PushBall
from rally_interfaces.msg import TrialResult

if TYPE_CHECKING:
    from rally_brain.trees.session_tree import Session

ACTION = "/rally/skill/push_ball"
DEFAULTS: dict[str, Any] = {"brain_push_speed_mps": 0.3, "balls_per_cycle": 4, "push_speed_range": [0.10, 0.60],
                            "brain_target_start_m": 0.40, "brain_target_step_m": 0.05, "brain_target_range_m": [0.15, 0.45]}


def mode1_config(skills_cfg: dict) -> dict[str, Any]:
    cfg = {**DEFAULTS, **(skills_cfg.get("mode1") or {})}
    cfg["lane_count"] = len(cfg.get("lanes") or []) or 3
    return cfg


def trials_per_cycle(skills_cfg: dict) -> int:
    return max(1, int(mode1_config(skills_cfg)["balls_per_cycle"]))


def next_push_speed(session: Session, lane: int, cfg: dict[str, Any]) -> float:
    """다음 밀기 속도 [m/s]. 지금은 고정값을 push_speed_range 로 자른 것. LLM 학습 루프가 붙을 자리."""
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
                      success: bool = False, target_d: float = math.nan, level: int = 0) -> TrialResult:
    trial = TrialResult(mode=TrialResult.MODE_PUSH_BALL, success=success, notes=notes,
                        error_mm=measured_error_mm(result) if result is not None else math.nan,
                        param_keys=["lane", "push_speed", "target_d_m", "level"],
                        param_values=[float(lane), float(speed), float(target_d), float(level)])
    if result is not None:
        p = result.stop_position
        trial.metric_keys = ["contact_force_peak", "stop_x_m", "stop_y_m"]
        trial.metric_values = [float(result.contact_force_peak), float(p.x), float(p.y)]
    return trial


def random_lane(cfg: dict[str, Any], rng: random.Random) -> int:
    return rng.randrange(cfg["lane_count"])


def build_trial(session: Session, rng: random.Random | None = None) -> py_trees.behaviour.Behaviour:
    cfg = mode1_config(session.skills_cfg)
    state = TrialState(session)
    client = session.action_client(PushBall, ACTION)
    log = session.node.get_logger()
    rng = rng or random.Random()

    def start() -> None:
        state.start()
        lane = random_lane(cfg, rng)
        level = session.difficulty.get(TrialResult.MODE_PUSH_BALL, 0)
        recipe = lane_recipe(cfg, lane)
        if recipe:
            d = target_distance(level, cfg)
            target = target_point(cfg, lane, d)
            speed = recipe_speed(recipe, d, cfg)
            log.info(f"[모드 1] 레인 {lane} · 난이도 {level} → 입구에서 {d * 100:.0f} cm 에 멈추게 (예상 {speed:.2f} m/s, 레시피)")
        else:
            d, target = math.nan, None
            speed = next_push_speed(session, lane, cfg)
            log.info(f"[모드 1] 레인 {lane} · 속도 {speed:.2f} m/s (레시피 없음 → 고정 속도)")
        state.data.update(lane=lane, speed=speed, target=target, target_d=d, level=level)

    def goal() -> PushBall.Goal:
        return push_goal(state.data["lane"], state.data["speed"], state.data["target"])

    def record(result: PushBall.Result | None, notes: str, success: bool = False) -> None:
        state.finish(trial_from_result(state.data["lane"], state.data["speed"], result, notes, success,
                                       state.data["target_d"], state.data["level"]))

    def on_fail(result: PushBall.Result | None, reason: str, _k: int) -> None:
        log.warn(f"[모드 1] 밀기 실패: {reason}")
        record(result, reason_code(reason))

    push = skill_step(state, "밀기", client, goal,
                      on_success=lambda r, _k: record(r, "", True),
                      on_fail=on_fail,
                      on_interrupt=lambda reason: record(None, reason_code(reason)))

    def on_stop() -> None:
        if state.started and not state.recorded:
            record(None, "interrupted")

    return Guard("모드 1 시도", py_trees.composites.Selector("모드 1", memory=True, children=[
        py_trees.composites.Sequence("밀기", memory=True, children=[Do("시작", start), push]),
        Do("끝", lambda: not state.fatal),
    ]), on_stop)
