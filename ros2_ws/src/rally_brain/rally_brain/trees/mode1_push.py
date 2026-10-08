"""rally_brain/rally_brain/trees/mode1_push.py — 모드 1 공 굴려주기: 시도 1회 서브트리
담당: 조정묵

  모드 1 시도 (Guard — 도중에 멈추면 interrupted 기록)
   └─ Selector
       ├─ 밀기 (Sequence)
       │   ├─ 시작          레인 고르기(0 → 1 → 2 → 0 …), 속도 정하기
       │   └─ PushBall      로봇이 거치대 공을 비전으로 집고(x 가장 작은 공) 레인 입구에 놓고 친다
       │                    끝나면 기록: 성공 = 스킬 success, notes = 실패 이유 앞 낱말
       └─ 끝                 밀기가 실패로 기록됐으면 시도는 끝 (서버 없음이면 세션 실패)
  시도 수 = skills.yaml mode1.balls_per_cycle (거치대 공 개수, 기본 4) — session_tree.TRIAL_COUNTS.
  재시도는 없다: 실패 이유가 거치대 빔(no_ball_detected)·집기 실패 등 로봇 쪽이라 다시 해도 같다.

지금(10/8, #42 요청): 손 대기 없음, 속도 고정 brain_push_speed_mps(기본 0.3 m/s), target 비움.
  target 만 주고 속도 0 → 레인 레시피로 속도 (로봇 쪽, 레시피 실측 전엔 no_recipe 실패) 는 레시피가 생기면.
  stop_position·error_mm 은 아직 로봇이 안 채운다 → 정지 위치가 (0,0,0) 이면 error_mm 을 NaN 으로 기록
  (0 mm 로 넣으면 "완벽한 시도"로 학습된다. #69: 못 잰 값 = NaN).
TODO(Day 5): next_push_speed 를 llm_advisor(SuggestParams) 로 — 오차가 채워진 뒤. 손 대기(mode5 와 같은 방식).
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

import py_trees

from rally_brain.behaviours import Do, Guard
from rally_brain.trees.steps import TrialState, reason_code, skill_step
from rally_interfaces.action import PushBall
from rally_interfaces.msg import TrialResult

if TYPE_CHECKING:
    from rally_brain.trees.session_tree import Session

ACTION = "/rally/skill/push_ball"
DEFAULTS: dict[str, Any] = {"brain_push_speed_mps": 0.3, "balls_per_cycle": 4, "push_speed_range": [0.10, 0.60]}


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


def push_goal(lane: int, speed: float) -> PushBall.Goal:
    # target 비움 + push_speed > 0 → 로봇은 그 속도로. 접촉 임계 0 = skills.yaml 기본
    return PushBall.Goal(lane=lane, push_speed=float(speed))


def measured_error_mm(result: PushBall.Result) -> float:
    p = result.stop_position
    return result.error_mm if (p.x, p.y, p.z) != (0.0, 0.0, 0.0) else math.nan


def trial_from_result(lane: int, speed: float, result: PushBall.Result | None, notes: str,
                      success: bool = False) -> TrialResult:
    trial = TrialResult(mode=TrialResult.MODE_PUSH_BALL, success=success, notes=notes,
                        error_mm=measured_error_mm(result) if result is not None else math.nan,
                        param_keys=["lane", "push_speed"], param_values=[float(lane), float(speed)])
    if result is not None:
        p = result.stop_position
        trial.metric_keys = ["contact_force_peak", "stop_x_m", "stop_y_m"]
        trial.metric_values = [float(result.contact_force_peak), float(p.x), float(p.y)]
    return trial


def build_trial(session: Session) -> py_trees.behaviour.Behaviour:
    cfg = mode1_config(session.skills_cfg)
    state = TrialState(session)
    client = session.action_client(PushBall, ACTION)
    log = session.node.get_logger()
    counter = {"n": 0}   # 이 모드 블록에서 몇 번째 공인지 → 레인

    def start() -> None:
        state.start()
        lane = counter["n"] % cfg["lane_count"]
        counter["n"] += 1
        speed = next_push_speed(session, lane, cfg)
        state.data.update(lane=lane, speed=speed)
        log.info(f"[모드 1] 레인 {lane} · 속도 {speed:.2f} m/s")

    def goal() -> PushBall.Goal:
        return push_goal(state.data["lane"], state.data["speed"])

    def record(result: PushBall.Result | None, notes: str, success: bool = False) -> None:
        state.finish(trial_from_result(state.data["lane"], state.data["speed"], result, notes, success))

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
