"""rally_brain/rally_brain/rule_layer.py — 규칙 레이어: 시도가 끝날 때마다 다음 행동 하나를 고른다
담당: 조정묵

판단 순서는 규칙 → LLM (rally_brain/CLAUDE.md). 답이 분명하면 여기서 끝내고, 애매하면 ask_llm 으로 넘긴다.
행동은 skills.yaml brain.actions 안에서만 고른다. 위에서부터 처음 걸리는 규칙 하나:

  1. 안전 STOP                                  → wait
  2. 음성 "힘들어"(tired, 강한 피로 신호)          → suggest_rest
  3. 음성 harder / easier                        → harder / easier
  4. 같은 모드 success_streak 연속 성공            → harder
  5. 같은 모드 failure_streak 연속 실패            → easier
  6. 약한·중간 피로 신호 llm_signal_count 개 이상   → ask_llm   (신호 강도는 ontology.yaml fatigue_signals)
  7. 그 외                                       → continue

연속 횟수는 그 모드의 난이도가 마지막으로 바뀐 뒤의 시도만 센다 (호출하는 쪽이 잘라서 넘김).
안전 SLOW 는 다루지 않는다 — skill_server 가 speed_override 로 이미 낮춘다 (기획서 5.3).
"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any

from rally_brain import metrics
from rally_brain.knowledge import ontology
from rally_interfaces.msg import SafetyStatus, SceneContext, TrialResult

DEFAULTS: dict[str, Any] = {
    "actions": ["continue", "harder", "easier", "suggest_rest", "wait", "ask_llm"],
    "success_streak": 3,
    "failure_streak": 2,
    "llm_signal_count": 2,
    "success_drop_rate": 0.4,
    "slowing_ratio": 1.5,
    "difficulty_range": [-3, 3],
}
MIN_TRIALS_FOR_TREND = 3


@dataclass(frozen=True)
class Decision:
    action: str
    reason: str                                 # 한 줄, 로그·대시보드·TTS 용
    signals: tuple[str, ...] = field(default=())  # 걸린 피로 신호 (ontology fatigue_signals 키)


def rule_config(skills_cfg: dict[str, Any]) -> dict[str, Any]:
    return {**DEFAULTS, **(skills_cfg.get("brain") or {})}


def tail_streak(trials: Sequence[TrialResult], success: bool) -> int:
    """마지막 시도와 같은 모드에서, 끝에서부터 success 가 연속된 횟수."""
    if not trials:
        return 0
    mode = trials[-1].mode
    n = 0
    for t in reversed(trials):
        if t.mode != mode or t.success != success:
            break
        n += 1
    return n


def fatigue_signals(ctx: SceneContext | None, trials: Sequence[TrialResult], cfg: dict[str, Any]) -> list[str]:
    """음성 tired 를 뺀 피로 신호들 (ontology.yaml fatigue_signals 키)."""
    signals = []
    if ctx is not None and ctx.body.compensation_detected:
        signals.append("compensation_repeated")   # 1차: 지금 보이면. TODO: 최근 시도 중 반복 횟수로
    if len(trials) >= MIN_TRIALS_FOR_TREND:
        if metrics.success_rate(trials) <= cfg["success_drop_rate"]:   # 성공률 정의는 metrics.yaml 한 곳
            signals.append("success_drop")
        earlier = [t.duration_s for t in trials[:-1] if t.duration_s > 0]
        if earlier and trials[-1].duration_s >= cfg["slowing_ratio"] * (sum(earlier) / len(earlier)):
            signals.append("slowing")
    return signals


def decide(ctx: SceneContext | None, trials: Sequence[TrialResult], intent: str | None,
           cfg: dict[str, Any], onto: dict[str, Any] | None = None) -> Decision:
    """trials: 이번 세션에서 이 모드의 난이도가 마지막으로 바뀐 뒤의 시도. intent: 아직 처리 안 한 음성 intent."""
    onto = onto or ontology.load()

    def make(action: str, reason: str, signals: Sequence[str] = ()) -> Decision:
        if action not in cfg["actions"]:
            raise ValueError(f"skills.yaml brain.actions 에 없는 행동: {action}")
        return Decision(action, reason, tuple(signals))

    if ctx is not None and ctx.safety.level == SafetyStatus.LEVEL_STOP:
        return make("wait", f"안전 정지({ctx.safety.reason or '사유 없음'}) → 대기")
    if intent == "tired":
        return make("suggest_rest", '"힘들어" 음성 → 휴식 제안', ["voice_tired"])
    if intent in ("harder", "easier"):
        return make(intent, f"음성 요청 → {'어렵게' if intent == 'harder' else '쉽게'}")
    if (n := tail_streak(trials, True)) >= cfg["success_streak"]:
        return make("harder", f"{n}연속 성공 → 난이도 올림")
    if (n := tail_streak(trials, False)) >= cfg["failure_streak"]:
        return make("easier", f"{n}연속 실패 → 난이도 내림")

    signals = fatigue_signals(ctx, trials, cfg)
    weak = [s for s in signals if onto["fatigue_signals"][s]["strength"] in ("medium", "weak")]
    if len(weak) >= cfg["llm_signal_count"]:
        names = ", ".join(ontology.label("fatigue_signals", s, onto) for s in weak)
        return make("ask_llm", f"애매한 피로 신호 {len(weak)}개({names}) → LLM 판단", weak)
    return make("continue", "특이 사항 없음 → 계속", signals)


def apply_difficulty(level: int, action: str, cfg: dict[str, Any]) -> int:
    """harder/easier 를 난이도 단계에 반영 (difficulty_range 안으로)."""
    lo, hi = cfg["difficulty_range"]
    step = {"harder": 1, "easier": -1}.get(action, 0)
    return max(lo, min(hi, level + step))
