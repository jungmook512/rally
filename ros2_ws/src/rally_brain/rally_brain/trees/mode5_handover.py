"""rally_brain/rally_brain/trees/mode5_handover.py — 모드 5 건네고 받기: 시도 1회 서브트리
담당: 조정묵

  모드 5 시도 (Sequence)
   ├─ 손 대기        안전 STOP 아니고 손이 보일 때까지 (WaitUntil)
   ├─ 시작 시각
   └─ 건네기 (Selector)
       ├─ 건네기 + 기록   Handover GIVE → 결과로 TrialResult
       └─ 실패 기록       스킬이 abort·취소됐으면 실패 TrialResult (goal 을 못 보냈으면 FAILURE → 세션 실패)
  세션이 스킬 도중 중단되면(안전 STOP·취소) 실패 TrialResult notes=interrupted

지금(Day 2): Handover GIVE 한 번. 물체는 로봇이 이미 쥐고 있다고 가정.
TODO(Day 3): PickCompliant 로 집기 → GIVE → RECEIVE, 실패 분기(손 결측 → 대기, 파지 실패 → 재시도 1회), 기울기 단계
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

import py_trees

from rally_brain.behaviours import Do, SkillCall, WaitUntil
from rally_interfaces.action import Handover
from rally_interfaces.msg import TrialResult

if TYPE_CHECKING:
    from rally_brain.trees.session_tree import Session

ACTION = "/rally/skill/handover"
OBJECT_CLASS = "rod"


def handover_goal() -> Handover.Goal:
    # 힘 임계·타임아웃 0 = skill_server 가 skills.yaml handover.default_* 사용
    return Handover.Goal(direction=Handover.Goal.DIR_GIVE, object_class=OBJECT_CLASS, tilt_deg=0.0, follow_hand=False)


def trial_from_result(result: Handover.Result, duration_s: float) -> TrialResult:
    return TrialResult(
        mode=TrialResult.MODE_HANDOVER, success=result.success, duration_s=duration_s,
        param_keys=["direction", "tilt_deg"], param_values=[float(Handover.Goal.DIR_GIVE), 0.0],
        metric_keys=["wrist_angle_at_grasp_deg", "handover_time_s"],
        metric_values=[result.wrist_angle_at_grasp_deg, result.handover_time_s],
        notes=result.failure_reason)


def build_trial(session: Session) -> py_trees.behaviour.Behaviour:
    state = {"t0": 0.0}
    client = session.action_client(Handover, ACTION)

    def mark_start() -> None:
        state["t0"] = time.monotonic()

    def on_result(result: Handover.Result) -> None:
        session.record(trial_from_result(result, time.monotonic() - state["t0"]))

    def record_failure(notes: str) -> None:
        session.record(TrialResult(mode=TrialResult.MODE_HANDOVER, success=False,
                                   duration_s=time.monotonic() - state["t0"], notes=notes))

    skill = SkillCall("건네기 GIVE", session.node, client, handover_goal, on_result,
                      on_interrupt=lambda: record_failure("interrupted"))

    def record_skill_failure() -> bool:
        if not skill.goal_sent:
            return False
        record_failure(f"skill_{skill.feedback_message}")
        return True

    give = py_trees.composites.Selector("건네기", memory=True, children=[skill, Do("실패 기록", record_skill_failure)])
    return py_trees.composites.Sequence("모드 5 시도", memory=True, children=[
        WaitUntil("손 대기", lambda: session.safe() and session.hand_visible()),
        Do("시작 시각", mark_start),
        give,
    ])
