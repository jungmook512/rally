"""rally_brain/rally_brain/trees/mode3_catch.py — 모드 3 캐치볼(던진 공을 패드로 받기, 챌린지): 시도 1회 서브트리
담당: 조정묵

  모드 3 시도 (Guard — 도중에 멈추면 interrupted 기록)
   └─ Selector
       ├─ 받기 (Sequence)
       │   ├─ 노출 대기  손목 카메라 노출 고정이 적용될 때까지 (최대 exposure_wait_s, 넘으면 그냥 진행 — 공이 번질 수 있음, #139 박성현)
       │   ├─ 시작       goal 준비 (회차 세기)
       │   └─ 받기 중 (Parallel)
       │       ├─ 투구 안내  goal 을 보낸 뒤 throw_prompt_delay_s(3 s) 지나서 throw_prompt ("공을 던져 주세요")
       │       │             (10/15: 로봇은 준비 완료를 알려 주지 않아 시간으로. 스킬이 먼저 끝나면 말하지 않음)
       │       └─ CatchBall  로봇이 패드를 들고 대기 자세 → 공을 놓은 뒤 예측점으로 한 번 이동 → 측면 카메라로 판정(패드 위 공 3 s, #147) → 복귀
       └─ 끝             실패를 기록했거나 시도를 건너뛰었으면 시도는 끝 (받기 스킬 서버 없음도 건너뜀 — 세션 실패 아님)
  결과 안내 (skills.yaml mode3.*_prompt): 받음 success_prompt · 놓침 missed_prompt · 반경 밖 out_of_radius_prompt.
  모드 3 의 마지막 시도(session.trials_planned)에서 놓침·반경 밖이면 last_fail_prompt ("아쉬워요" — 바로 done_prompt 가 이어지므로
  '다시 던져 볼까요' 를 빼고, 10/15 조정묵). timeout·안전 정지·중단은 말하지 않는다.
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

from rally_brain import hand_stop
from rally_brain.behaviours import Do, Guard, WaitUntil
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
    # 투구 안내 시점 (10/16 조정묵: 대기 자세에 선 뒤). 로봇은 대기 자세 도착을 알리지 않으므로(feedback ready 는 집기 전) brain 이 관절값으로 본다:
    #   관절 6개가 모두 ready_posj(skills.yaml mode3) 에서 throw_prompt_pose_tol_deg 안에 throw_prompt_settle_s 동안 머물면 (그리퍼가 빈 걸 알면 아직).
    #   관절값이 안 오면 그리퍼 상태로 — 패드를 쥔 것이 보인 뒤 throw_prompt_after_grip_s(거치대 → 대기 자세 이동 시간), 처음부터 쥐고 있었으면
    #   (직전 놓침) goal 뒤 throw_prompt_delay_s. 그리퍼도 모르면 delay_s. 늦어도 goal 뒤 throw_prompt_max_s. 그 전에 스킬이 끝나면 안내 없음
    "throw_prompt_pose_tol_deg": 3.0,
    "throw_prompt_settle_s": 1.0,
    "throw_prompt_delay_s": 3.0,
    "throw_prompt_after_grip_s": 8.0,
    "throw_prompt_max_s": 25.0,
    "exposure_wait_s": 3.0,       # goal 전 손목 노출 고정 적용을 기다리는 최대 시간
    # 결과별 TTS (빈 문자열이면 말하지 않음). 안전 정지·중단·timeout 때는 말하지 않는다 (안전 안내는 다른 곳에서)
    "success_prompt": "잘 던졌어요, 받았어요",
    "missed_prompt": "아쉬워요, 다시 던져 볼까요",
    "out_of_radius_prompt": "조금 더 패드 쪽으로 던져 볼까요",
    "last_fail_prompt": "아쉬워요",   # 마지막 시도의 놓침·반경 밖 — '다시 던져 볼까요' 없이
}
FAIL_PROMPT_KEYS = {"missed": "missed_prompt", "out_of_radius": "out_of_radius_prompt"}


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


def result_text(cfg: dict[str, Any], caught: bool, reason: str, last: bool = False) -> str | None:
    """결과 안내 문구. 마지막 시도(last)의 놓침·반경 밖은 last_fail_prompt. 빈 문구면 None (말하지 않음)."""
    if caught:
        text = cfg["success_prompt"]
    elif reason in FAIL_PROMPT_KEYS:
        text = cfg["last_fail_prompt"] if last else cfg[FAIL_PROMPT_KEYS[reason]]
    else:
        text = ""
    return str(text or "") or None


def feedback_text(cfg: dict[str, Any], result: CatchBall.Result | None, reason: str, last: bool = False) -> str | None:
    return result_text(cfg, result is not None and bool(result.caught), reason, last)


def build_trial(session: Session) -> py_trees.behaviour.Behaviour:
    cfg = mode3_config(session.skills_cfg)
    state = TrialState(session)
    client = session.action_client(CatchBall, ACTION)
    log = session.node.get_logger()
    turn = {"k": 0}   # 이 모드 블록에서 몇 번째 시도인지 — 건너뛴 시도도 세어 Repeat 회차와 같게 (트리를 다시 만들면 0 부터)
    said = {"result": False}                      # 결과 안내를 판정 피드백에서 이미 했는가 (결과 때 되풀이하지 않게)
    judged = {"phase": None, "moved": False}      # feedback (executor 스레드가 씀): 판정 caught/missed · 요격 이동(intercept) 여부
    hold = {"at_start": None, "since": None}      # 투구 안내: goal 때 쥐고 있었나 · 쥔 것이 처음 보인 시각(state.elapsed 기준)
    pose = {"since": None}                        # 투구 안내: 관절이 대기 자세 허용 오차 안에 들어온 시각

    def is_last() -> bool:
        """모드 3 의 계획된 마지막 시도인가 (session_tree 가 trials_planned 에 적음. 없으면 — 테스트 등 — 마지막으로 보지 않음)."""
        planned = session.trials_planned.get(TrialResult.MODE_CATCH_PAD)
        return planned is not None and turn["k"] >= int(planned)

    def timeouts_in_a_row() -> int:
        n = 0
        for t in reversed([t for t in session.trials if t.mode == TrialResult.MODE_CATCH_PAD]):
            if t.notes != "timeout":
                break
            n += 1
        return n

    def start() -> bool:
        """False = 이 시도를 건너뜀 (받기 스킬 서버 없음, 또는 연속으로 안 던짐). 기록 없이 끝."""
        turn["k"] += 1
        if not client.server_is_ready():
            # 챌린지 모드 — 받기 스킬이 안 떠 있으면 세션을 실패시키지 않고 모드 3 만 건너뜀 (readiness.yaml 의 세션 조건 밖)
            log.warn(f"[모드 3] {ACTION} 서버 없음 — 이 시도 건너뜀")
            return False
        if timeouts_in_a_row() >= int(cfg["timeout_end_after"]):
            log.warn(f"[모드 3] 연속 {cfg['timeout_end_after']}번 공을 안 던짐 — 남은 시도 건너뜀")
            return False
        state.start()
        state.data["goal"] = catch_goal(cfg)
        said["result"] = False
        judged.update(phase=None, moved=False)
        hold.update(at_start=None, since=None)
        pose["since"] = None
        log.info(f"[모드 3] 받기 — 반경 {cfg['catch_radius_m']:.2f} m, 기다림 {cfg['trial_timeout_s']:.0f} s"
                 + (" (마지막 시도)" if is_last() else ""))
        return True

    def record(result: CatchBall.Result | None, reason: str) -> None:
        code = reason_code(reason)
        state.finish(trial_from_result(state.data["goal"], result, code))
        text = None if said["result"] else feedback_text(cfg, result, code, is_last())   # 판정 피드백에서 이미 말했으면 생략
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

    # 손 근접 STOP (10/15): 대기 자세에서 공을 기다리는 동안(phase ready 만)이면 손이 빠진 뒤 같은 goal 다시 — 패드는 이미 쥐고 있어
    # 집기를 건너뛰고 대기 자세로 간다. 비행·요격 중이면 그 시도는 놓친 것 — bt_runner 가 취소하고 다음 시도 (복구 대기)
    hand_reasons = list(hand_stop.config(session.skills_cfg)["hand_stop_wait_reasons"])

    def catch_hand_resume(reason: str, phases: list[str]) -> str | None:
        return "resend" if reason_code(reason) in hand_reasons and set(phases) <= {"ready"} else None

    # 결과 안내 (10/16 조정묵): 판정 피드백(caught/missed) 이 오는 순간 — 결과는 패드 꽂기·공 떼기까지 끝나야 와서 20 s 넘게 늦다.
    # 놓쳤는데 로봇이 안 움직였으면(intercept 없음 — 반경 밖·저신뢰) 이유를 모르니 결과(바로 옴)에서 이유에 맞는 문구로
    def catch_feedback(fb: Any) -> None:
        if fb.phase == "intercept":
            judged["moved"] = True
        elif fb.phase in ("caught", "missed"):
            judged["phase"] = fb.phase

    def judgment_text() -> str | None:
        if judged["phase"] == "caught":
            return result_text(cfg, True, "", is_last())
        if judged["phase"] == "missed" and judged["moved"]:
            return result_text(cfg, False, "missed", is_last())
        return None

    def say_judgment() -> bool:
        text = judgment_text()
        if text:
            said["result"] = True
            session.say(text)
        return True

    catch = skill_step(state, "공 받기", client, lambda: state.data["goal"],
                       on_success=on_success, on_fail=on_fail, hand_resume=catch_hand_resume, on_feedback=catch_feedback,
                       succeeded=lambda _r: True,   # 끝까지 돌았으면 명중 참/거짓 모두 이 시도의 결과 (on_success 가 판정)
                       on_interrupt=lambda reason: state.finish(trial_from_result(state.data["goal"], None,
                                                                                  reason_code(reason))))

    # 투구 안내 (10/16 조정묵: 대기 자세에 선 뒤). 로봇은 패드를 집고 대기 자세에 선 것을 알려 주지 않는다(feedback "ready" 는 집기 전에 옴) —
    # brain 이 관절값(/dsr01/joint_states)을 ready_posj 와 허용 오차(throw_prompt_pose_tol_deg)로 비교해 settle_s 동안 안에 머물면 안내.
    # 일정한 값이 아니라 범위로 보는 이유: 관절 도착값이 교시값과 조금씩 다르다 (사용자 결정 10/16). 그리퍼가 빈 걸 알면(패드 집기 전) 아직.
    # 관절값이 안 오면 그리퍼 상태로: 거치대에서 집으면 쥔 것이 보인 뒤 throw_prompt_after_grip_s, goal 때 이미 쥐고 있었으면(직전 놓침)
    # throw_prompt_delay_s, 그리퍼도 모르면(로봇 노드 없음·테스트) delay_s. 늦어도 throw_prompt_max_s.
    # 받기 스킬과 나란히 돌리고, 스킬이 먼저 끝나면(집기 실패·안전 정지·바로 abort) Parallel 이 안내 가지를 멈춰 말하지 않는다.
    delay, after_grip, max_s, tol, settle = (float(cfg[k]) for k in (
        "throw_prompt_delay_s", "throw_prompt_after_grip_s", "throw_prompt_max_s", "throw_prompt_pose_tol_deg", "throw_prompt_settle_s"))
    ready_posj = [float(v) for v in (cfg.get("ready_posj") or [])]

    def at_ready_pose() -> bool | None:
        """관절 6개가 모두 대기 자세 허용 오차 안인가. 관절값·교시값이 없으면 None (다른 기준으로)."""
        q = session.joint_deg()
        if q is None or len(q) < 6 or len(ready_posj) < 6:
            return None
        return all(abs(a - b) <= tol for a, b in zip(q[:6], ready_posj[:6]))

    def throw_ready() -> bool:
        holding = session.gripper_holding()
        t = state.elapsed()
        if t >= max_s:
            return True
        at = at_ready_pose()
        if at is not None:
            if at and holding is not False:
                pose["since"] = t if pose["since"] is None else pose["since"]
            else:
                pose["since"] = None
            return pose["since"] is not None and t - pose["since"] >= settle
        if hold["at_start"] is None:
            hold["at_start"] = holding
        if holding is None or hold["at_start"] is True:
            return t >= delay
        if holding and hold["since"] is None:
            hold["since"] = t
        return hold["since"] is not None and t - hold["since"] >= after_grip

    branches: list[py_trees.behaviour.Behaviour] = []
    if cfg.get("throw_prompt"):
        branches.append(py_trees.composites.Sequence("투구 안내", memory=True, children=[
            WaitUntil("투구 안내 대기", throw_ready),
            Do("투구 안내", lambda: session.say(str(cfg["throw_prompt"]))),
        ]))
    branches.append(py_trees.composites.Sequence("판정 안내", memory=True, children=[
        WaitUntil("판정 대기", lambda: judgment_text() is not None),
        Do("판정 안내", say_judgment),
    ]))
    # catch 를 뒤에 두어 대시보드 tip 이 스킬을 가리키게
    catching = py_trees.composites.Parallel(
        "받기 중", policy=py_trees.common.ParallelPolicy.SuccessOnSelected([catch]), children=[*branches, catch])

    # 자동 노출 첫 프레임은 공이 번져 검출이 흔들린다 — 고정이 적용된 뒤 goal. 못 바꾸면(카메라 없음·거절) 기다린 뒤 그냥 진행
    exposure_wait = py_trees.decorators.FailureIsSuccess(
        "노출 대기", WaitUntil("노출 고정 대기", session.exposure_fixed, timeout_s=float(cfg["exposure_wait_s"])))

    def on_stop() -> None:
        if state.started and not state.recorded:
            state.finish(trial_from_result(state.data["goal"], None, "interrupted"))

    return Guard("모드 3 시도", py_trees.composites.Selector("모드 3", memory=True, children=[
        py_trees.composites.Sequence("받기", memory=True, children=[exposure_wait, Do("시작", start), catching]),
        Do("끝", lambda: not state.fatal),
    ]), on_stop)
