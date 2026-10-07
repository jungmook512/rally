"""rally_brain/rally_brain/trees/mode5_handover.py — 모드 5 건네고 받기: 시도 1회 서브트리
담당: 조정묵

  모드 5 시도 (Selector)
   ├─ 손 없음 (Sequence)
   │   ├─ Inverter(손 대기)   hand_wait_s 씩 hand_wait_tries 번, 사이마다 TTS 안내 (손이 보이면 이 가지는 FAILURE)
   │   └─ 손 없음 기록         끝내 안 보이면 실패 TrialResult notes=no_hand → 다음 시도로
   └─ 건네기 (Sequence)
       ├─ 시작 시각
       └─ 시도들 (Selector) ─ 1회차 … (max_retries + 1)회차, 각각:
           ├─ 회차 시작          앞 회차가 goal 을 못 보냈으면(서버 없음) FAILURE → 세션 실패
           ├─ Selector(Handover GIVE, abort 결과 받기)
           └─ 판정               성공 → 기록 / retry_reasons 실패이고 회차가 남았으면 안내 후 FAILURE(→ 다음 회차) / 아니면 실패 기록
  세션이 스킬 도중 중단되면(안전 STOP·취소) 실패 TrialResult notes=interrupted
  재시도는 시도 1회 안의 일이다 — TrialResult 는 마지막 회차 결과 하나, metrics 에 attempts (no_hand 는 0).
  no_hand 는 성공률·개선율·난이도 규칙에서 빠진다 (metrics.yaml exclude_notes, session_tree.NOT_PERFORMANCE).
  설정: skills.yaml mode5 (없으면 DEFAULTS).

지금(Day 3): Handover GIVE. 물체는 로봇이 이미 쥐고 있다고 가정. 그래서 재시도 대상은 지금은 GIVE 의
timeout(환자가 안 잡음)이고, todo 의 "파지 실패 재시도"(PickCompliant 로 집기)는 집기 단계가 트리에 들어올 때 같은 방식으로.
TODO: PickCompliant 로 집기 → GIVE → RECEIVE, 기울기 단계
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

import py_trees

from rally_brain.behaviours import Do, SkillCall, WaitUntil
from rally_interfaces.action import Handover
from rally_interfaces.msg import TrialResult

if TYPE_CHECKING:
    from rally_brain.trees.session_tree import Session

ACTION = "/rally/skill/handover"
OBJECT_CLASS = "rod"
DEFAULTS: dict[str, Any] = {
    "hand_wait_s": 10.0, "hand_wait_tries": 2, "hand_prompt": "손을 앞으로 내밀어 주세요",
    "retry_reasons": ["timeout", "no_object"], "max_retries": 1, "retry_prompt": "한 번 더 해 볼게요",
}


def mode5_config(skills_cfg: dict) -> dict[str, Any]:
    return {**DEFAULTS, **(skills_cfg.get("mode5") or {})}


def handover_goal() -> Handover.Goal:
    # 힘 임계·타임아웃 0 = skill_server 가 skills.yaml handover.default_* 사용
    return Handover.Goal(direction=Handover.Goal.DIR_GIVE, object_class=OBJECT_CLASS, tilt_deg=0.0, follow_hand=False)


def trial_from_result(result: Handover.Result, duration_s: float, attempts: int = 1) -> TrialResult:
    return TrialResult(
        mode=TrialResult.MODE_HANDOVER, success=result.success, duration_s=duration_s,
        param_keys=["direction", "tilt_deg"], param_values=[float(Handover.Goal.DIR_GIVE), 0.0],
        metric_keys=["wrist_angle_at_grasp_deg", "handover_time_s", "attempts"],
        metric_values=[result.wrist_angle_at_grasp_deg, result.handover_time_s, float(attempts)],
        notes=result.failure_reason)


def should_retry(reason: str, attempt: int, cfg: dict) -> bool:
    """attempt: 방금 끝난 회차 (1부터)."""
    return reason in cfg["retry_reasons"] and attempt <= cfg["max_retries"]


def hand_wait(session: Session, cfg: dict) -> py_trees.behaviour.Behaviour:
    """손이 보이면 SUCCESS. hand_wait_s 씩 hand_wait_tries 번 기다리고, 사이마다 안내. 끝내 안 보이면 FAILURE."""
    def seen() -> bool:
        return session.safe() and session.hand_visible()

    tries = max(1, int(cfg["hand_wait_tries"]))
    children: list[py_trees.behaviour.Behaviour] = [WaitUntil("손 대기 1", seen, cfg["hand_wait_s"])]
    for k in range(2, tries + 1):
        children.append(py_trees.composites.Sequence(f"안내 후 대기 {k}", memory=True, children=[
            Do("손 안내", lambda: session.say(cfg["hand_prompt"])),
            WaitUntil(f"손 대기 {k}", seen, cfg["hand_wait_s"]),
        ]))
    return py_trees.composites.Selector("손 대기", memory=True, children=children)


def build_trial(session: Session) -> py_trees.behaviour.Behaviour:
    cfg = mode5_config(session.skills_cfg)
    state: dict[str, Any] = {"t0": 0.0, "attempt": 0, "result": None, "fatal": False, "recorded": False,
                             "wait_t0": 0.0}
    client = session.action_client(Handover, ACTION)
    log = session.node.get_logger()

    def elapsed() -> float:
        return time.monotonic() - state["t0"]

    def mark_start() -> None:
        state.update(t0=time.monotonic(), attempt=0, result=None, fatal=False, recorded=False)
        session.begin_trial()   # 여기부터 기록까지 모드 전환 안 함 (손 대기 중엔 됨)

    def record_failure(notes: str) -> None:
        session.record(TrialResult(mode=TrialResult.MODE_HANDOVER, success=False, duration_s=elapsed(),
                                   metric_keys=["attempts"], metric_values=[float(max(1, state["attempt"]))],
                                   notes=notes))

    def mark_wait() -> None:
        state["wait_t0"] = time.monotonic()

    def record_no_hand() -> None:
        waited = time.monotonic() - state["wait_t0"]
        log.warn(f"[모드 5] 손이 {waited:.0f} s 동안 안 보임 — 이번 시도 건너뜀")
        session.record(TrialResult(mode=TrialResult.MODE_HANDOVER, success=False, duration_s=waited,
                                   metric_keys=["attempts"], metric_values=[0.0], notes="no_hand"))

    def attempt(k: int) -> py_trees.behaviour.Behaviour:
        def on_result(result: Handover.Result) -> None:
            # 성공은 여기서 바로 기록 — 중단(STOP·취소) 직전에 끝난 시도는 terminate 가 이것만 부르고
            # judge 는 돌지 않는다 (#64 리뷰 1, #41 과 같은 버그). 실패는 judge 가 재시도를 정한다
            state["result"] = result
            if result.success:
                session.record(trial_from_result(result, elapsed(), k))
                state["recorded"] = True

        def on_interrupt(reason: str) -> None:
            # 중단 시점에 abort 결과가 와 있으면 실제 실패 이유(safety_stop 등)로 (#64 리뷰 2)
            if reason != "interrupted" and skill.result is not None and skill.result.failure_reason:
                reason = skill.result.failure_reason
            elif reason != "interrupted":
                reason = f"skill_{reason}"
            record_failure(reason)

        skill = SkillCall(f"건네기 GIVE {k}", session.node, client, handover_goal,
                          on_result=on_result, on_interrupt=on_interrupt)

        def start() -> bool:
            if state["fatal"]:
                return False
            state.update(attempt=k, result=None)
            return True

        def take_failed_result() -> None:
            # abort·거절이어도 결과 메시지는 skill.result 에. goal 을 못 보냈으면(서버 없음) 시도가 아니다
            state["result"] = skill.result
            if not skill.goal_sent:
                state["fatal"] = True

        def judge() -> bool:
            if state["fatal"]:
                return False
            if state["recorded"]:                 # 성공 — on_result 에서 이미 기록
                return True
            result = state["result"]
            reason = result.failure_reason if result is not None else f"skill_{skill.feedback_message}"
            if should_retry(reason, k, cfg):
                log.info(f"[모드 5] {k}회차 실패({reason}) — 다시 시도")
                session.say(cfg["retry_prompt"])
                return False
            if result is not None:
                session.record(trial_from_result(result, elapsed(), k))
            else:
                record_failure(reason)
            return True

        return py_trees.composites.Sequence(f"{k}회차", memory=True, children=[
            Do("회차 시작", start),
            py_trees.composites.Selector("건네기", memory=True, children=[
                skill, Do("abort 결과 받기", take_failed_result)]),
            Do("판정", judge),
        ])

    attempts = [attempt(k) for k in range(1, int(cfg["max_retries"]) + 2)]
    return py_trees.composites.Selector("모드 5 시도", memory=True, children=[
        py_trees.composites.Sequence("손 없음", memory=True, children=[
            Do("대기 시작", mark_wait),
            py_trees.decorators.Inverter("손 보이면 넘어감", hand_wait(session, cfg)),
            Do("손 없음 기록", record_no_hand),
        ]),
        py_trees.composites.Sequence("건네기", memory=True, children=[
            Do("시작 시각", mark_start),
            py_trees.composites.Selector("시도들", memory=True, children=attempts),
        ]),
    ])
