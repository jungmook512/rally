"""rally_brain/rally_brain/rule_layer.py — 규칙 레이어: 시도가 끝날 때마다 다음 행동 하나를 고른다
담당: 조정묵

판단 순서는 규칙 → LLM (rally_brain/CLAUDE.md). 답이 분명하면 여기서 끝내고, 애매하면 ask_llm 으로 넘긴다.
규칙은 "어떤 계기가 일어났나"(cue)만 정하고, 그 계기에 맞는 행동은 ontology 그래프(Cue —triggers→ Action)에서 찾는다.
단 안전 STOP 은 코드에 wait 로 고정한다 — 데이터 파일을 고쳐서 안전 판단이 바뀌면 안 된다 (#60 리뷰 1, CLAUDE.md 4절).
행동은 skills.yaml brain.actions 안에서만 고른다. 위에서부터 처음 걸리는 계기 하나:

  1. 안전 STOP                                  → wait (코드 고정, 그래프는 이름에만)
  2. 음성 "힘들어"                               cue tired            → (그래프) suggest_rest
  3. 음성 harder / easier                        cue harder / easier  → (그래프) harder / easier
  4. 같은 모드 success_streak 연속 성공            cue success_streak   → (그래프) harder
  5. 같은 모드 failure_streak 연속 실패            cue failure_streak   → (그래프) easier
  6. 애매한 신호 llm_signal_count 개 이상          → ask_llm (규칙으로 정하지 않음)
     (보상 동작 반복 · 성공률 하락 · 시도 시간 증가 — 각각은 근거가 약해 혼자서는 행동을 정하지 않는다)
     보상 동작은 팔을 뻗는 동안에만 나오므로(#83) 판단 순간의 상황판이 아니라 시도별 기록(compensation_history)으로 센다.
  7. 그 외                                       → continue

연속 횟수는 그 모드의 난이도가 마지막으로 바뀐 뒤의 시도만 센다 (호출하는 쪽이 잘라서 넘김).
그래프는 bt_runner 가 켜질 때 check_graph 로 미리 검사한다 — 문제가 있으면 노드가 뜨지 않는다 (세션 도중 예외 방지).
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
    "compensation_window": 5,
    "compensation_min_trials": 3,
    "difficulty_range": [-3, 3],
}
MIN_TRIALS_FOR_TREND = 3
GRAPH_CUES = ("tired", "harder", "easier", "success_streak", "failure_streak")   # 행동을 그래프에서 찾는 계기


@dataclass(frozen=True)
class Decision:
    action: str
    reason: str                                 # 한 줄, 로그·대시보드·TTS 용
    signals: tuple[str, ...] = field(default=())  # 걸린 신호 (voice_tired · SIGNAL_LABELS 키)


def rule_config(skills_cfg: dict[str, Any]) -> dict[str, Any]:
    return {**DEFAULTS, **(skills_cfg.get("brain") or {})}


def check_graph(cfg: dict[str, Any], graph=None) -> list[str]:
    """규칙 레이어가 쓸 그래프 검사. 문제 목록 (빈 목록이면 정상). bt_runner 가 시작할 때 부른다.
    ontology.validate() + GRAPH_CUES 마다 행동이 있고 그 행동이 skills.yaml brain.actions 안에 있는지."""
    try:
        graph = graph if graph is not None else ontology.load()
    except Exception as e:  # noqa: BLE001 — 파일 없음·깨짐
        return [f"ontology 를 읽지 못함: {e}"]
    problems = ontology.validate(graph)
    for cue in GRAPH_CUES:
        action = ontology.action_for_cue(cue, graph)
        if action is None:
            problems.append(f"계기 {cue!r} → 행동 연결 없음")
        elif action not in cfg["actions"]:
            problems.append(f"계기 {cue!r} → {action!r} 는 skills.yaml brain.actions 에 없음")
    return problems


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


SIGNAL_LABELS = {"compensation": "보상 동작 반복", "success_drop": "성공률 하락", "slowing": "시도 시간 증가"}


def fatigue_signals(trials: Sequence[TrialResult], cfg: dict[str, Any], performance: bool = True,
                    compensation_history: Sequence[bool] = ()) -> list[str]:
    """애매한 신호들 (SIGNAL_LABELS 키). 음성 tired 는 여기 없음 — 그건 확실한 계기라 규칙 2.
    compensation_history: 이 모드의 시도마다 보상 동작이 한 번이라도 보였는지 (오래된 것 → 최근)."""
    signals = []
    recent = list(compensation_history)[-int(cfg["compensation_window"]):]
    if sum(recent) >= int(cfg["compensation_min_trials"]):
        signals.append("compensation")
    if len(trials) >= MIN_TRIALS_FOR_TREND:
        if performance and metrics.success_rate(trials) <= cfg["success_drop_rate"]:   # 성공률 정의는 metrics.yaml 한 곳
            signals.append("success_drop")
        earlier = [t.duration_s for t in trials[:-1] if t.duration_s > 0]
        if earlier and trials[-1].duration_s >= cfg["slowing_ratio"] * (sum(earlier) / len(earlier)):
            signals.append("slowing")
    return signals


def decide(ctx: SceneContext | None, trials: Sequence[TrialResult], intent: str | None,
           cfg: dict[str, Any], graph=None, *, performance: bool = True,
           fatigue_history: Sequence[TrialResult] | None = None,
           compensation_history: Sequence[bool] = ()) -> Decision:
    """trials: 이번 세션에서 이 모드의 난이도가 마지막으로 바뀐 뒤의 시도. intent: 아직 처리 안 한 음성 intent.
    compensation_history: fatigue_signals 참고."""
    graph = graph or ontology.load()

    def make(action: str, reason: str, signals: Sequence[str] = ()) -> Decision:
        if action not in cfg["actions"]:
            raise ValueError(f"skills.yaml brain.actions 에 없는 행동: {action}")
        return Decision(action, reason, tuple(signals))

    def from_cue(cue: str, why: str, signals: Sequence[str] = ()) -> Decision:
        action = ontology.action_for_cue(cue, graph)
        if action is None:
            raise ValueError(f"ontology 에 계기 {cue!r} → 행동 연결이 없음")
        return make(action, f"{why} → {ontology.label(action, graph)}", signals)

    if ctx is not None and ctx.safety.level == SafetyStatus.LEVEL_STOP:
        return make("wait", f"안전 정지({ctx.safety.reason or '사유 없음'}) → {ontology.label('wait', graph)}")
    if intent in ("tired", "harder", "easier"):
        return from_cue(intent, f"{ontology.label(intent, graph)}", ["voice_tired"] if intent == "tired" else [])
    if performance and (n := tail_streak(trials, True)) >= cfg["success_streak"]:
        return from_cue("success_streak", f"{n}연속 성공")
    if performance and (n := tail_streak(trials, False)) >= cfg["failure_streak"]:
        return from_cue("failure_streak", f"{n}연속 실패")

    signals = fatigue_signals(trials if fatigue_history is None else fatigue_history, cfg,
                              performance=performance, compensation_history=compensation_history)
    if len(signals) >= cfg["llm_signal_count"]:
        names = ", ".join(SIGNAL_LABELS[s] for s in signals)
        return make("ask_llm", f"애매한 신호 {len(signals)}개({names}) → LLM 판단", signals)
    return make("continue", "특이 사항 없음 → 계속", signals)


def apply_difficulty(level: int, action: str, cfg: dict[str, Any]) -> int:
    """harder/easier 를 난이도 단계에 반영 (difficulty_range 안으로)."""
    lo, hi = cfg["difficulty_range"]
    step = {"harder": 1, "easier": -1}.get(action, 0)
    return max(lo, min(hi, level + step))
