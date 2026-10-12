"""rally_brain/rally_brain/trees/steps.py — 모드 서브트리가 같이 쓰는 조각: 시도 1회 상태, 재시도 있는 스킬 단계
담당: 조정묵

  TrialState   시도 1회의 상태. 기록(session.record)은 시작한 시도마다 정확히 한 번 — finish 가 지킨다.
  skill_step   스킬 하나를 tries 회까지 (Selector of 회차). 회차마다:
                 회차 시작 → Selector(SkillCall, abort 결과 받기) → 판정
               성공이면 SUCCESS. 실패면 can_retry 이고 회차가 남았으면 다음 회차, 아니면 on_fail(기록) 후 FAILURE.
               goal 을 못 보냈으면(서버 없음) state.fatal → FAILURE (재시도 안 함, 세션 실패).
               성공 결과는 SkillCall.on_result 에서 바로 on_success 로 — 중단(STOP·취소) 직전에 끝난 스킬은
               terminate 가 on_result 만 부르고 판정은 돌지 않기 때문 (#64 리뷰 1).
  no_hand      손 대기: hand_wait_s 씩 hand_wait_tries 번, 사이마다 hand_prompt 안내. 끝내 안 보이면 notes=no_hand 기록
               (이 가지가 SUCCESS = 시도 끝). 손이 보이면 FAILURE → 모드 트리의 Selector 가 다음 가지(진행)로.
  모드 트리는 단계들을 Sequence 로 잇고 Guard 로 감싼다: 단계 사이에 멈추면 on_stop 이 interrupted 로 기록.
"""
from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import py_trees
from rclpy.action import ActionClient

from rally_brain.behaviours import Do, SkillCall, WaitUntil
from rally_interfaces.msg import TrialResult

if TYPE_CHECKING:
    from rally_brain.trees.session_tree import Session


def reason_code(reason: str) -> str:
    """스킬 failure_reason 의 앞 낱말 — "no_recipe (설명…)" → "no_recipe". notes 에는 이것만 (설명은 로그에)."""
    return reason.split(" ", 1)[0].split("(", 1)[0] if reason else ""


@dataclass
class TrialState:
    session: Session
    t0: float = 0.0
    started: bool = False      # 로봇 동작을 시작했는지 (손 대기 뒤). 시작 안 한 시도는 interrupted 로 기록하지 않는다
    recorded: bool = False
    fatal: bool = False        # 스킬 서버 없음 등 — 시도가 아니라 세션 문제
    goals_sent: int = 0
    data: dict[str, Any] = field(default_factory=dict)   # 모드별 값 (고른 레인·각도, 단계 결과 등)

    def start(self) -> None:
        self.t0 = time.monotonic()
        self.started, self.recorded, self.fatal, self.goals_sent = True, False, False, 0
        self.data = {}
        self.session.begin_trial()   # 여기부터 기록까지 모드 전환 안 함

    def elapsed(self) -> float:
        return time.monotonic() - self.t0

    def finish(self, trial: TrialResult) -> None:
        """시작한 시도를 한 번만 기록. 두 번째부터는 무시."""
        if not self.started or self.recorded:
            return
        self.recorded = True
        self.started = False
        if trial.duration_s <= 0:
            trial.duration_s = self.elapsed()
        self.session.record(trial)


def skill_step(state: TrialState, name: str, client: ActionClient, make_goal: Callable[[], Any], *,
               tries: int = 1,
               can_send: Callable[[], bool] = lambda: True,
               wait_for_precondition: bool = False,
               on_success: Callable[[Any, int], None],
               on_fail: Callable[[Any | None, str, int], None],
               on_interrupt: Callable[[str], None],
               can_retry: Callable[[str], bool] = lambda _: False,
               on_retry: Callable[[str, int], None] = lambda *_: None,
               reason_of: Callable[[Any, bool], str] = lambda r, _aborted: getattr(r, "failure_reason", ""),
               on_feedback: Callable[[Any], None] | None = None,
               hand_resume: Callable[[str, list[str]], str | None] | None = None,
               succeeded: Callable[[Any], bool] = lambda r: bool(r.success),
               ) -> py_trees.behaviour.Behaviour:
    """on_success(result, k) / on_fail(result 또는 None, reason, k) / on_retry(reason, k): k = 방금 끝난 회차 (1부터).
    reason_of(result, aborted): 실패 이유. 빈 문자열이면 skill_<상태> (예: skill_aborted — 이유 없는 abort 는 안전 정지일 수 있다).
    succeeded(result): 끝난(SUCCEEDED) goal 의 결과를 성공으로 볼지. 기본 result.success — success 필드가 없는 결과
    (CatchBall: caught)는 끝까지 돈 것 자체를 시도 결과로 받고 판정은 on_success 쪽에서 한다."""
    session = state.session
    log = session.node.get_logger()
    step = {"done": False, "ok": False}

    def attempt(k: int) -> py_trees.behaviour.Behaviour:
        def on_result(result: Any) -> None:
            if succeeded(result):
                step["ok"] = True
                on_success(result, k)

        def interrupted(reason: str) -> None:
            # 중단 시점에 abort 결과가 와 있으면 실제 실패 이유(safety_stop 등)로 (#64 리뷰 2)
            if reason != "interrupted":
                reason = (reason_of(skill.result, True) if skill.result is not None else "") or f"skill_{reason}"
            on_interrupt(reason)

        skill = SkillCall(f"{name} {k}", session.node, client, make_goal, on_result=on_result,
                          on_interrupt=interrupted, on_feedback=on_feedback, can_send=can_send,
                          wait_for_precondition=wait_for_precondition,
                          hand_resume=hand_resume, resume_ok=session.robot_resume_ok)

        def begin() -> bool:
            if k == 1:
                step.update(done=False, ok=False)
            return not (state.fatal or step["done"] or state.recorded)

        def take_failed() -> None:
            # abort·거절이어도 결과 메시지는 skill.result 에. goal 을 못 보냈으면(서버 없음) 시도가 아니다
            if not skill.goal_sent and skill.feedback_message != "precondition_failed":
                state.fatal = True

        def judge() -> bool:
            if skill.goal_sent:
                state.goals_sent += 1
            if state.fatal:
                return False
            if step["ok"]:
                step["done"] = True
                return True
            result = skill.result
            aborted = skill.status != py_trees.common.Status.SUCCESS
            reason = (reason_of(result, aborted) if result is not None else "") or f"skill_{skill.feedback_message or 'failed'}"
            if k < tries and can_retry(reason) and session.safe():
                log.info(f"[{name}] {k}회차 실패({reason}) — 다시 시도")
                on_retry(reason, k)
                return False
            step["done"] = True
            on_fail(result, reason, k)
            return False

        return py_trees.composites.Sequence(f"{name} {k}회차", memory=True, children=[
            Do("회차 시작", begin),
            py_trees.composites.Selector(name, memory=True, children=[skill, Do("abort 결과 받기", take_failed)]),
            Do("판정", judge),
        ])

    return py_trees.composites.Selector(name, memory=True, children=[attempt(k) for k in range(1, max(1, tries) + 1)])


HAND_DEFAULTS: dict[str, Any] = {"hand_wait_s": 10.0, "hand_wait_tries": 2, "hand_prompt": "손을 앞으로 내밀어 주세요"}


def hand_wait(session: Session, cfg: dict, condition: Callable[[], bool] | None = None) -> py_trees.behaviour.Behaviour:
    """손이 보이면 SUCCESS. hand_wait_s 씩 hand_wait_tries 번 기다리고, 사이마다 안내. 끝내 안 보이면 FAILURE."""
    def seen() -> bool:
        return session.safe() and (condition() if condition is not None else session.hand_visible())

    tries = max(1, int(cfg["hand_wait_tries"]))
    children: list[py_trees.behaviour.Behaviour] = [WaitUntil("손 대기 1", seen, cfg["hand_wait_s"])]
    for k in range(2, tries + 1):
        children.append(py_trees.composites.Sequence(f"안내 후 대기 {k}", memory=True, children=[
            Do("손 안내", lambda: session.say(cfg["hand_prompt"])),
            WaitUntil(f"손 대기 {k}", seen, cfg["hand_wait_s"]),
        ]))
    return py_trees.composites.Selector("손 대기", memory=True, children=children)


def no_hand(session: Session, cfg: dict, mode: int, condition: Callable[[], bool] | None = None) -> py_trees.behaviour.Behaviour:
    """손이 끝내 안 보이면 실패 TrialResult(notes=no_hand, attempts 0, duration = 기다린 시간) 를 남기고 SUCCESS."""
    wait = {"t0": 0.0}

    def mark() -> None:
        wait["t0"] = time.monotonic()

    def record() -> None:
        waited = time.monotonic() - wait["t0"]
        session.node.get_logger().warn(f"[모드 {mode}] 손이 {waited:.0f} s 동안 안 보임 — 이번 시도 건너뜀")
        session.record(TrialResult(mode=mode, success=False, duration_s=waited,
                                   metric_keys=["attempts"], metric_values=[0.0], notes="no_hand"))

    return py_trees.composites.Sequence("손 없음", memory=True, children=[
        Do("대기 시작", mark),
        py_trees.decorators.Inverter("손 보이면 넘어감", hand_wait(session, cfg, condition)),
        Do("손 없음 기록", record),
    ])
