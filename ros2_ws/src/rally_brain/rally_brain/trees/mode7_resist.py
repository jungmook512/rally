"""rally_brain/rally_brain/trees/mode7_resist.py — 모드 7 저항 밀기(팔씨름형): 시도 1회 서브트리
담당: 조정묵

  모드 7 시도 (Guard — 도중에 멈추면 interrupted 기록)
   └─ Selector
       ├─ 손 없음            모드 5 와 같은 손 대기 (steps.no_hand) — 로봇이 손목을 내밀기 전에 환자가 있는지
       └─ 진행 (Selector)
           ├─ 밀기 (Sequence)
           │   ├─ 시작        목표 힘 = target_force_start_n × (1 + level_step_ratio)^난이도, target_force_range_n 으로 자름
           │   └─ ResistPush  로봇이 손목을 내밀고 환자가 미는 힘을 버팀 (10/12: 손잡이 없음) → 목표 힘을 hold_time_s 유지하면 져준다 → 기록 + TTS
           └─ 끝              실패도 기록했으면 시도는 끝 (서버 없음이면 세션 실패)
  난이도는 rule_layer 가 정한다 (3연속 성공 → +1, 2연속 실패 → -1, brain.difficulty_range). 이 트리는 단계 → 힘 변환만.
  재시도 없음: 실패 이유(timeout·force_limit·released·safety_stop)가 환자의 결과이거나 안전이라 다시 할 이유가 없다.
  로봇은 버티거나 져주기만 한다 — brain 은 숫자만 보낸다 (CLAUDE.md 4절, rally_robot/skills/resist_push.py).
  handle_pose 는 비워 보낸다 → 로봇이 제시 자세를 정한다 (정정인 확인 필요).

TODO(Day 6): 시작값 = 지난 세션 최대 힘의 60 % (brain → DB 읽기가 생길 때, Graph RAG 와 같이). 지금은 target_force_start_n.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

import py_trees
from geometry_msgs.msg import PoseStamped

from rally_brain.behaviours import Do, Guard
from rally_brain.trees.steps import HAND_DEFAULTS, TrialState, no_hand, reason_code, skill_step
from rally_interfaces.action import ResistPush
from rally_interfaces.msg import TrialResult

if TYPE_CHECKING:
    from rally_brain.trees.session_tree import Session

ACTION = "/rally/skill/resist_push"
DEFAULTS: dict[str, Any] = {
    **HAND_DEFAULTS,
    "stiffness_default": 700.0, "target_force_start_n": 5.0, "target_force_range_n": [3.0, 12.0],
    "level_step_ratio": 0.10, "hold_time_s": 2.0, "yield_distance_m": 0.05, "force_limit_n": 15.0, "timeout_s": 20.0,
}
# 결과별 TTS. 성공은 버틴 시간을 넣어 말한다. 안전 정지·중단 때는 말하지 않는다 (안전 안내는 다른 곳에서)
PROMPTS: dict[str, str] = {
    "success": "좋아요, {hold:.0f}초 버텼어요",
    "timeout": "조금만 더 힘을 줘 볼까요",
    "released": "로봇 손목을 끝까지 밀어 주세요",
    "force_limit": "힘을 조금 빼고 다시 해 볼게요",
}


def mode7_config(skills_cfg: dict) -> dict[str, Any]:
    return {**DEFAULTS, **(skills_cfg.get("mode7") or {})}


def target_force(level: int, cfg: dict[str, Any]) -> float:
    lo, hi = cfg["target_force_range_n"]
    value = float(cfg["target_force_start_n"]) * (1.0 + float(cfg["level_step_ratio"])) ** level
    return round(min(max(value, float(lo)), float(hi)), 2)


def resist_goal(target_n: float, cfg: dict[str, Any]) -> ResistPush.Goal:
    return ResistPush.Goal(handle_pose=PoseStamped(), stiffness=float(cfg["stiffness_default"]),
                           target_force_n=float(target_n), hold_time_s=float(cfg["hold_time_s"]),
                           yield_distance_m=float(cfg["yield_distance_m"]), force_limit_n=float(cfg["force_limit_n"]),
                           timeout_s=float(cfg["timeout_s"]))


def trial_from_result(goal: ResistPush.Goal, result: ResistPush.Result | None, notes: str) -> TrialResult:
    trial = TrialResult(mode=TrialResult.MODE_RESIST, success=bool(result is not None and result.success), notes=notes,
                        param_keys=["stiffness", "target_force_n"],
                        param_values=[float(goal.stiffness), float(goal.target_force_n)])
    trial.metric_keys = ["peak_force_n", "hold_time_achieved_s", "force_smoothness"]
    trial.metric_values = ([float(result.peak_force_n), float(result.hold_time_achieved_s), float(result.force_smoothness)]
                           if result is not None else [math.nan] * 3)
    return trial


def feedback_text(result: ResistPush.Result | None, reason: str) -> str | None:
    if result is not None and result.success:
        return PROMPTS["success"].format(hold=result.hold_time_achieved_s)
    return PROMPTS.get(reason)


def build_trial(session: Session) -> py_trees.behaviour.Behaviour:
    cfg = mode7_config(session.skills_cfg)
    state = TrialState(session)
    client = session.action_client(ResistPush, ACTION)
    log = session.node.get_logger()

    def start() -> None:
        state.start()
        level = session.difficulty.get(TrialResult.MODE_RESIST, 0)
        state.data["goal"] = resist_goal(target_force(level, cfg), cfg)
        log.info(f"[모드 7] 난이도 {level} → 목표 {state.data['goal'].target_force_n:.1f} N")
        if cfg.get("push_prompt"):
            session.say(cfg["push_prompt"])   # 시작 안내 (#119). 결과 문구는 PROMPTS

    def record(result: ResistPush.Result | None, reason: str) -> None:
        state.finish(trial_from_result(state.data["goal"], result, reason_code(reason)))
        text = feedback_text(result, reason_code(reason))
        if text:
            session.say(text)

    def on_fail(result: ResistPush.Result | None, reason: str, _k: int) -> None:
        log.warn(f"[모드 7] 실패: {reason}")
        record(result, reason)

    push = skill_step(state, "저항 밀기", client, lambda: state.data["goal"],
                      on_success=lambda r, _k: record(r, ""), on_fail=on_fail,
                      on_interrupt=lambda reason: state.finish(trial_from_result(state.data["goal"], None,
                                                                                 reason_code(reason))))

    def on_stop() -> None:
        if state.started and not state.recorded:
            state.finish(trial_from_result(state.data["goal"], None, "interrupted"))

    return Guard("모드 7 시도", py_trees.composites.Selector("모드 7", memory=True, children=[
        no_hand(session, cfg, TrialResult.MODE_RESIST),
        py_trees.composites.Selector("진행", memory=True, children=[
            py_trees.composites.Sequence("밀기", memory=True, children=[Do("시작", start), push]),
            Do("끝", lambda: not state.fatal),
        ]),
    ]), on_stop)
