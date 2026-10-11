"""rally_brain/rally_brain/hand_stop.py — 모드 1 손 근접 STOP 대기 (#98 4번, INTERFACES 9절 1단계)
담당: 조정묵

손 근접 STOP(비래치, safety_monitor 가 손이 빠지면 스스로 푼다)이 로봇이 서 있는 단계에서 걸리면 세션을 끝내지 않고 기다린다.
  - 대상 사유: skills.yaml brain.hand_stop_wait_reasons (정확한 문자열 목록, 접두사 허용 없음). 그 밖의 STOP 은 기존대로 세션 종료
  - 대상 모드: 모드 1 만
  - 로봇이 움직이는 중(스킬 goal 을 보냄·자세 이동 서비스 호출 중)이면 기다리지 않고 세션 종료 — 공·그리퍼 상태를 몰라 자동 재진행 금지
  - 기다리는 동안 트리를 tick 하지 않는다 (새 동작 없음). NORMAL 이 hand_stop_resume_hold_s 동안 이어지면 같은 시도를 이어 간다
    (SLOW 는 재진행 조건이 아님). 이어 갈 때 대기 블록의 시간 기준을 기다린 만큼 뒤로 민다 — 멈춘 시간 때문에 바로 timeout 나지 않게
  - Brain 은 STOP 을 낮추거나 reset 하지 않는다. 대기 중 다른 STOP·취소·상황판 끊김은 bt_runner 가 기존대로 끝낸다
"""
from __future__ import annotations

from collections.abc import Iterable

import py_trees

from rally_brain.behaviours import ServiceCall, SkillCall
from rally_interfaces.msg import SafetyStatus, TrialResult

DEFAULTS = {
    "hand_stop_wait_reasons": ["hand_near_robot", "hand_near_centerline", "hand_lost_near"],
    "hand_stop_wait_modes": [TrialResult.MODE_PUSH_BALL],
    "hand_stop_resume_hold_s": 1.0,
    "hand_stop_prompt": "손을 조금 떼어 주세요. 손이 빠지면 이어서 할게요",
    "hand_stop_prompt_s": 10.0,
}


def config(skills_cfg: dict) -> dict:
    brain = skills_cfg.get("brain") or {}
    return {k: brain.get(k, v) for k, v in DEFAULTS.items()}


def waitable(reason: str, mode: int | None, cfg: dict) -> bool:
    """이 STOP 을 기다려도 되는가 (모드·사유만). 로봇이 움직이는지는 robot_busy 로 따로."""
    return mode in cfg["hand_stop_wait_modes"] and reason in cfg["hand_stop_wait_reasons"]


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
