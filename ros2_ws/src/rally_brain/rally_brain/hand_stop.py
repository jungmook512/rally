"""rally_brain/rally_brain/hand_stop.py — STOP 뒤 세션 유지: 손 근접 STOP 대기 (#98) + 복구 대기 (10/15)
담당: 조정묵

① 손 근접 STOP 대기 (#98 4번, INTERFACES 9절 1단계)
  손 근접 STOP(비래치, safety_monitor 가 손이 빠지면 스스로 푼다)이 로봇이 서 있는 단계에서 걸리면 세션을 끝내지 않고 기다린다.
  - 대상 사유: skills.yaml brain.hand_stop_wait_reasons (정확한 문자열 목록, 접두사 허용 없음)
  - 대상 모드: brain.hand_stop_wait_modes (10/15 부터 1·3·5·7 모두)
  - 로봇이 움직이는 중(스킬 goal 을 보냄·자세 이동 서비스 호출 중)이면, 그 블록이 받은 진행 단계로 보아 다시 보내도 되는
    경우(hand_decision → resend/done)만 같은 시도를 이어 간다 (모드 1: 손 보기 이동·PushBall 놓기 전·ReturnBall 공 찾기는 다시 보냄,
    ReturnBall 손 보기 단계는 끝난 것으로. 모드 5: 접근·제시 단계. 모드 3: 대기 자세(ready)). 아니면 ② 복구 대기
  - 기다리는 동안 트리를 tick 하지 않는다 (새 동작 없음). NORMAL 이 hand_stop_resume_hold_s 동안 이어지면 같은 시도를 이어 간다
    (SLOW 는 재진행 조건이 아님). 이어 갈 때 대기 블록의 시간 기준을 기다린 만큼 뒤로 민다 — 멈춘 시간 때문에 바로 timeout 나지 않게
  - 안전 상태 끊김(no_safety_status, brain.stop_wait_reasons_no_cancel)은 래치가 아니라 어느 모드든 같은 방식으로 기다리되
    스킬은 취소하지 않는다 (로봇 PC 의 safety_monitor 가 실제 정지를 맡고 있고, 상태가 돌아오면 그대로 이어 간다)

② 복구 대기 (Recovery, 10/15 조정묵: "웬만한 건 세션 종료가 아니라 실패 처리 후 다음 시도")
  그 밖의 STOP(힘 한계·툭 치기·작업 공간·비상어 등 래치 STOP)과 로봇 동작 중 이어 갈 수 없는 단계의 손 STOP 은 세션을 끝내지 않는다:
  bt_runner 가 트리를 멈춰(돌던 스킬 취소, 그 시도는 interrupted 기록) → 안전이 NORMAL 로 resume_hold_s 이어질 때까지 기다리고
  (래치 STOP 은 운영자가 reset 해야 풀림, recovery_prompt 로 부탁) → 모드 1·5 에서 그리퍼에 물건이 있으면 사람이 뺄 때까지
  (clear_gripper_prompt. 다음 스킬이 시작하자마자 그리퍼를 열어 물건이 떨어지므로) → 같은 모드의 남은 시도부터 트리를 다시 만든다
  (모드 시작의 관측 자세 이동이 자세를 정리한다). recovery_max_s·clear_gripper_max_s 를 넘기면 그때만 세션 종료.
  Brain 은 STOP 을 낮추거나 reset 하지 않는다 (안전 규칙 그대로). 취소·상황판 끊김은 bt_runner 가 따로 다룬다.
"""
from __future__ import annotations

from collections.abc import Iterable

import py_trees

from rally_brain.behaviours import ServiceCall, SkillCall
from rally_interfaces.msg import SafetyStatus, TrialResult

DEFAULTS = {
    "hand_stop_wait_reasons": ["hand_near_robot", "hand_near_centerline", "hand_lost_near"],
    "hand_stop_wait_modes": [TrialResult.MODE_PUSH_BALL, TrialResult.MODE_CATCH_PAD, TrialResult.MODE_HANDOVER, TrialResult.MODE_RESIST],
    "hand_stop_resume_hold_s": 1.0,
    "hand_stop_prompt": "손을 조금 떼어 주세요. 손이 빠지면 이어서 할게요",
    "hand_stop_prompt_s": 10.0,
    # 복구 대기 (10/15)
    "stop_wait_reasons_no_cancel": ["no_safety_status"],   # 안전 상태 끊김: 어느 모드든 기다리고 스킬도 취소하지 않음 (안내 없음, 로그만)
    "recovery_prompt": "로봇이 서 있어요. 안전을 확인하고 리셋해 주세요",   # 래치 STOP — 운영자 reset 을 기다리는 동안
    "recovery_prompt_s": 15.0,                                   # 복구 안내 간격
    "recovery_max_s": 300.0,                                     # 이만큼 기다려도 NORMAL 이 안 되면 세션 종료
    "clear_gripper_prompt": "로봇 손에 있는 물건을 빼서 제자리에 놓아 주세요",
    "clear_gripper_modes": [TrialResult.MODE_PUSH_BALL, TrialResult.MODE_HANDOVER],   # 그리퍼 비우기를 확인하는 모드 (3 은 패드, 7 은 손잡이를 쥔 게 정상)
    "clear_gripper_max_s": 120.0,                                # 비워지길 이만큼 기다려도 안 비면 세션 종료
}


def config(skills_cfg: dict) -> dict:
    brain = skills_cfg.get("brain") or {}
    return {k: brain.get(k, v) for k, v in DEFAULTS.items()}


def waitable(reason: str, mode: int | None, cfg: dict) -> bool:
    """이 STOP 을 같은 시도를 이어 갈 목적으로 기다려도 되는가 (모드·사유만). 로봇이 움직이는지는 robot_busy 로 따로.
    아니면 bt_runner 가 복구 대기(Recovery)로 간다."""
    if reason in cfg["stop_wait_reasons_no_cancel"]:
        return True
    return mode in cfg["hand_stop_wait_modes"] and reason in cfg["hand_stop_wait_reasons"]


def no_cancel(reason: str, cfg: dict) -> bool:
    """이 STOP 은 돌던 스킬을 취소하지 않고 기다린다 (안전 상태 끊김 — 로봇은 로봇 PC 의 safety_monitor 가 지킨다)."""
    return reason in cfg["stop_wait_reasons_no_cancel"]


def stop_prompt(reason: str, cfg: dict) -> str:
    """대기 중 말할 안내. 손 근접이면 손 안내, 상태 끊김이면 말하지 않음(빈 문자열), 그 밖(래치 STOP)은 reset 부탁."""
    if reason in cfg["hand_stop_wait_reasons"]:
        return str(cfg["hand_stop_prompt"])
    if reason in cfg["stop_wait_reasons_no_cancel"]:
        return ""
    return str(cfg["recovery_prompt"])


def robot_busy(behaviours: Iterable[py_trees.behaviour.Behaviour]) -> bool:
    """로봇에 명령이 나가 있는 중: 스킬 goal 을 보냈고 결과를 아직 처리 안 함, 또는 자세 이동 서비스 응답 대기.
    precondition 을 기다리는 SkillCall(goal 전)은 움직이는 중이 아니다."""
    for b in behaviours:
        if b.status != py_trees.common.Status.RUNNING:
            continue
        if isinstance(b, SkillCall) and b.goal_sent:
            return True
        if isinstance(b, ServiceCall) and b.request_sent:
            return True
    return False


def busy_resumable(behaviours: Iterable[py_trees.behaviour.Behaviour], reason: str, cfg: dict | None = None) -> bool:
    """로봇에 명령이 나가 있는 블록이 모두 이 손 STOP 뒤 이어 갈 수 있는가 (다시 보냄 또는 끝난 것으로).
    취소하지 않는 사유(no_cancel)면 항상 참 — 명령은 그대로 두고 상태가 돌아오길 기다린다."""
    if cfg is not None and no_cancel(reason, cfg):
        return True
    for b in behaviours:
        if b.status != py_trees.common.Status.RUNNING:
            continue
        sent = (isinstance(b, SkillCall) and b.goal_sent) or (isinstance(b, ServiceCall) and b.request_sent)
        if sent and b.hand_decision(reason) not in ("resend", "done"):
            return False
    return True


def shift_timers(behaviours: Iterable[py_trees.behaviour.Behaviour], dt: float) -> None:
    """실행 중 블록의 시작 시각을 dt 만큼 뒤로 — 기다린 시간을 대기·timeout 계산에서 뺀다."""
    for b in behaviours:
        if b.status == py_trees.common.Status.RUNNING and hasattr(b, "shift_time"):
            b.shift_time(dt)


class HandStopWait:
    """대기 한 번의 상태. bt_runner tick 스레드에서만 쓴다."""

    def __init__(self, reason: str, now: float, cfg: dict):
        self.reason = reason
        self.cfg = cfg
        self.started = now
        self.normal_since: float | None = None
        self.last_prompt = now

    def stop_seen(self, reason: str) -> None:
        """대기 중 손 STOP 이 다시 옴 (tick 사이의 짧은 STOP 포함) → NORMAL 유지 시간을 처음부터."""
        self.reason = reason
        self.normal_since = None

    def update(self, level: int, now: float) -> bool:
        """최신 안전 레벨로 갱신. NORMAL 이 resume_hold_s 이어졌으면 True (이어 가도 됨)."""
        if level != SafetyStatus.LEVEL_NORMAL:
            self.normal_since = None
            return False
        if self.normal_since is None:
            self.normal_since = now
        return now - self.normal_since >= float(self.cfg["hand_stop_resume_hold_s"])

    def prompt_due(self, now: float) -> bool:
        if now - self.last_prompt >= float(self.cfg["hand_stop_prompt_s"]):
            self.last_prompt = now
            return True
        return False


class Recovery:
    """복구 대기 한 번의 상태 (10/15). bt_runner tick 스레드에서만 쓴다. 트리는 이미 멈춘 상태.
    단계: safety(NORMAL 이 resume_hold_s 이어질 때까지 — 래치 STOP 은 운영자 reset) → gripper(모드 1·5: 그리퍼가 빌 때까지) → done."""

    def __init__(self, reason: str, now: float, cfg: dict, check_gripper: bool):
        self.reason, self.cfg, self.started = reason, cfg, now
        self.check_gripper = check_gripper
        self.stage = "safety"
        self.stage_since = now
        self.normal_since: float | None = None
        self.last_prompt = float("-inf")       # 처음 prompt() 는 바로 말함

    def stop_seen(self, reason: str) -> None:
        """대기 중 STOP 이 다시 옴 → 사유 갱신, NORMAL 유지 시간을 처음부터 (그리퍼 단계였으면 안전 단계로 되돌림)."""
        self.reason = reason
        self.normal_since = None
        if self.stage == "gripper":
            self.stage, self.stage_since, self.last_prompt = "safety", self.stage_since, float("-inf")

    def update(self, level: int, now: float, gripper_empty: bool | None) -> str:
        """최신 안전 레벨·그리퍼 상태로 갱신. "wait" | "done" | "timeout". gripper_empty None = 모름(그리퍼 상태 없음) → 비운 것으로."""
        if now - self.started > float(self.cfg["recovery_max_s"]):
            return "timeout"
        if self.stage == "safety":
            if level != SafetyStatus.LEVEL_NORMAL:
                self.normal_since = None
                return "wait"
            self.normal_since = self.normal_since or now
            if now - self.normal_since < float(self.cfg["hand_stop_resume_hold_s"]):
                return "wait"
            self.stage, self.stage_since, self.last_prompt = "gripper", now, float("-inf")
        if self.stage == "gripper":
            if level != SafetyStatus.LEVEL_NORMAL:          # 비우는 동안 다시 STOP (손이 로봇 가까이) → 안전 단계부터
                self.stage, self.normal_since = "safety", None
                return "wait"
            if not self.check_gripper or gripper_empty is not False:
                self.stage = "done"
                return "done"
            if now - self.stage_since > float(self.cfg["clear_gripper_max_s"]):
                return "timeout"
            return "wait"
        return "done"

    def prompt(self, now: float) -> str | None:
        """지금 말할 안내 (간격 recovery_prompt_s). 없으면 None."""
        if now - self.last_prompt < float(self.cfg["recovery_prompt_s"]):
            return None
        self.last_prompt = now
        text = str(self.cfg["clear_gripper_prompt"]) if self.stage == "gripper" else stop_prompt(self.reason, self.cfg)
        return text or None
