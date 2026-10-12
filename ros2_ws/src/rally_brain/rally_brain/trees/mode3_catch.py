"""rally_brain/rally_brain/trees/mode3_catch.py — 모드 3 캐치볼(던진 공을 패드로 받기, 챌린지): 시도 1회 서브트리
담당: 조정묵

  모드 3 시도 (Guard — 도중에 멈추면 interrupted 기록)
   └─ Selector
       ├─ 받기 (Sequence)
       │   ├─ 시작       throw_prompt 안내 ("공을 던져 주세요")
       │   └─ CatchBall  로봇이 패드를 들고 대기 자세 → 공을 놓은 뒤 예측점으로 한 번 이동 → 측면 카메라로 판정(패드 위 공 3 s, #147) → 복귀
       └─ 끝             실패를 기록했거나 시도를 건너뛰었으면 시도는 끝 (받기 스킬 서버 없음도 건너뜀 — 세션 실패 아님)
  범위 (docs/mode3/MODE3_PLAN.md, 10/12): CatchBall 결과 기록까지. 패드에서 공 떼기·환자에게 돌려주기는 로봇 after_catch()
  보류 — 그동안 운영자가 공을 떼어 건넨다. 다음 시도의 timeout_s 에 그 시간이 들어간다.
  goal 은 catch_radius·timeout_s 만 쓴다 (스킬이 ready_pose·lead_time_s 를 쓰지 않음, #128). 난이도 레시피 없음 — 고정 값.
  재시도 없음: 실패 이유(timeout·out_of_radius·missed·안전)가 환자 투구 결과이거나 안전이라 같은 시도를 다시 할 이유가 없다.
  timeout(공을 안 던짐)은 시도 기록만 하고(notes=timeout) 연속 timeout_end_after 번이면 모드 3 남은 시도를 건너뛴다.
  받기 스킬 서버가 없으면 시도를 시작하지 않고 건너뛴다 (챌린지 — 세션 실패로 만들지 않음, 기록 없음).
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

import py_trees
from geometry_msgs.msg import Point

from rally_brain.behaviours import Do, Guard
from rally_brain.trees.steps import TrialState, reason_code, skill_step
from rally_interfaces.action import CatchBall
from rally_interfaces.msg import TrialResult

if TYPE_CHECKING:
    from rally_brain.trees.session_tree import Session

ACTION = "/rally/skill/catch_ball"
DEFAULTS: dict[str, Any] = {
    "catch_radius_m": 0.35,        # 대기 패드 중심에서 이 밖이면 로봇이 움직이지 않음 (out_of_radius)
    "trial_timeout_s": 30.0,       # 공을 던지기를 기다리는 시간 (운영자가 공을 떼어 건네는 시간 포함)
    "timeout_end_after": 2,        # 연속 이만큼 안 던지면(timeout) 모드 3 남은 시도 건너뜀
    "throw_prompt": "준비되면 패드를 향해 공을 던져 주세요",
}
# 결과별 TTS. 안전 정지·중단 때는 말하지 않는다 (안전 안내는 다른 곳에서)
PROMPTS: dict[str, str] = {
    "success": "잘 던졌어요, 받았어요",
    "missed": "아쉬워요, 다시 던져 볼까요",
    "out_of_radius": "조금 더 패드 쪽으로 던져 볼까요",
}


def mode3_config(skills_cfg: dict) -> dict[str, Any]:
    return {**DEFAULTS, **(skills_cfg.get("mode3") or {})}


def catch_goal(cfg: dict[str, Any]) -> CatchBall.Goal:
    """ready_pose·lead_time_s 는 스킬이 쓰지 않아 비워 보낸다 (대기 자세는 로봇 skills.yaml mode3.ready_posj)."""
    return CatchBall.Goal(ready_pose=Point(), catch_radius=float(cfg["catch_radius_m"]), lead_time_s=0.0,
                          timeout_s=float(cfg["trial_timeout_s"]))


def _finite(v: float) -> float:
    return float(v) if math.isfinite(v) else math.nan


def trial_from_result(goal: CatchBall.Goal, result: CatchBall.Result | None, notes: str) -> TrialResult:
    """success = 패드 위 공을 측면 카메라가 3 s 연속 본 것 (#147, 힘 판정 없음). error_mm = 예측 오차 (로봇이 패드 위 공 위치로 계산, 못 재면 NaN)."""
    caught = bool(result is not None and result.caught)
    trial = TrialResult(mode=TrialResult.MODE_CATCH_PAD, success=caught, notes=notes,
                        error_mm=_finite(result.prediction_error_mm) if result is not None else math.nan,
                        param_keys=["catch_radius"], param_values=[float(goal.catch_radius)])
    trial.metric_keys = ["robot_travel_mm", "reaction_time_s"]
    trial.metric_values = ([_finite(result.robot_travel_mm), _finite(result.reaction_time_s)]
                           if result is not None else [math.nan] * 2)
    return trial


def feedback_text(result: CatchBall.Result | None, reason: str) -> str | None:
    if result is not None and result.caught:
        return PROMPTS["success"]
    return PROMPTS.get(reason)


def build_trial(session: Session) -> py_trees.behaviour.Behaviour:
    cfg = mode3_config(session.skills_cfg)
    state = TrialState(session)
    client = session.action_client(CatchBall, ACTION)
    log = session.node.get_logger()

    def timeouts_in_a_row() -> int:
        n = 0
        for t in reversed([t for t in session.trials if t.mode == TrialResult.MODE_CATCH_PAD]):
            if t.notes != "timeout":
                break
            n += 1
        return n

    def start() -> bool:
        """False = 이 시도를 건너뜀 (받기 스킬 서버 없음, 또는 연속으로 안 던짐). 기록 없이 끝."""
        if not client.server_is_ready():
            # 챌린지 모드 — 받기 스킬이 안 떠 있으면 세션을 실패시키지 않고 모드 3 만 건너뜀 (readiness.yaml 의 세션 조건 밖)
            log.warn(f"[모드 3] {ACTION} 서버 없음 — 이 시도 건너뜀")
            return False
        if timeouts_in_a_row() >= int(cfg["timeout_end_after"]):
            log.warn(f"[모드 3] 연속 {cfg['timeout_end_after']}번 공을 안 던짐 — 남은 시도 건너뜀")
            return False
        state.start()
        state.data["goal"] = catch_goal(cfg)
        log.info(f"[모드 3] 받기 — 반경 {cfg['catch_radius_m']:.2f} m, 기다림 {cfg['trial_timeout_s']:.0f} s")
        if cfg.get("throw_prompt"):
            session.say(cfg["throw_prompt"])
        return True

    def record(result: CatchBall.Result | None, reason: str) -> None:
        code = reason_code(reason)
        state.finish(trial_from_result(state.data["goal"], result, code))
        text = feedback_text(result, code)
        if text:
            session.say(text)

    def on_success(result: CatchBall.Result, _k: int) -> None:
        # 끝까지 돈 경우 (명중 참/거짓). 빗나감·반경 밖은 failure_reason 에
        log.info(f"[모드 3] {'명중' if result.caught else '놓침'}"
                 + (f" — {result.failure_reason}" if result.failure_reason else "")
                 + f", 이동 {result.robot_travel_mm:.0f} mm")
        record(result, result.failure_reason if not result.caught else "")

    def on_fail(result: CatchBall.Result | None, reason: str, _k: int) -> None:
        log.warn(f"[모드 3] 실패: {reason}")
        record(result, reason)

    catch = skill_step(state, "공 받기", client, lambda: state.data["goal"],
                       on_success=on_success, on_fail=on_fail,
                       succeeded=lambda _r: True,   # 끝까지 돌았으면 명중 참/거짓 모두 이 시도의 결과 (on_success 가 판정)
                       on_interrupt=lambda reason: state.finish(trial_from_result(state.data["goal"], None,
                                                                                  reason_code(reason))))

    def on_stop() -> None:
        if state.started and not state.recorded:
            state.finish(trial_from_result(state.data["goal"], None, "interrupted"))

    return Guard("모드 3 시도", py_trees.composites.Selector("모드 3", memory=True, children=[
        py_trees.composites.Sequence("받기", memory=True, children=[Do("시작", start), catch]),
        Do("끝", lambda: not state.fatal),
    ]), on_stop)
