"""rally_brain/rally_brain/metrics.py — metrics.yaml 의 지표를 파이썬으로 계산 (규칙 레이어·LLM·리포트 공용)
담당: 조정묵

같은 지표의 DB 버전은 rally_db/schema_003_metrics.sql. 정의가 바뀌면 둘 다 바꾼다.
"""
from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from functools import lru_cache
from typing import Any

import yaml

from rally_brain.config import config_path
from rally_interfaces.msg import TrialResult


@lru_cache(maxsize=1)
def load() -> dict[str, Any]:
    path = config_path("metrics.yaml")
    if path is None:
        raise FileNotFoundError("rally_bringup/config/metrics.yaml 을 찾지 못함")
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def key_value(trial: TrialResult, cfg: dict[str, Any] | None = None) -> float | None:
    """모드별 대표 값 (metrics.yaml key_values). 없으면 None."""
    spec = (cfg or load())["key_values"].get(trial.mode)
    if spec is None:
        return None
    if spec["source"] == "field":
        value = float(getattr(trial, spec["name"]))
    else:
        pairs = dict(zip(trial.metric_keys, trial.metric_values))
        value = float(pairs[spec["name"]]) if spec["name"] in pairs else math.nan
    return value if math.isfinite(value) else None   # 못 잰 값(NaN, #69) = 없음


def counted(trials: Iterable[TrialResult], cfg: dict[str, Any] | None = None) -> list[TrialResult]:
    """지표에 넣는 시도만 (exclude_notes 빼고: no_hand · interrupted)."""
    exclude = set((cfg or load())["exclude_notes"])
    return [t for t in trials if t.notes not in exclude]


def success_rate(trials: Sequence[TrialResult], cfg: dict[str, Any] | None = None) -> float:
    """최근 window 회 성공 비율 (exclude_notes 시도는 빼고). trials 는 같은 모드라고 가정. 시도가 없으면 0."""
    cfg = cfg or load()
    recent = counted(trials, cfg)[-cfg["metrics"]["success_rate"]["window"]:]
    return sum(t.success for t in recent) / len(recent) if recent else 0.0


def improvement(trials: Sequence[TrialResult], cfg: dict[str, Any] | None = None) -> float | None:
    """성공한 시도의 처음 k 회 대비 마지막 k 회 대표 값 개선율 (양수 = 나아짐). 같은 모드라고 가정. 계산 불가면 None."""
    cfg = cfg or load()
    k = cfg["metrics"]["improvement"]["k"]
    trials = [t for t in counted(trials, cfg) if t.success]
    if len(trials) < 2 * k:
        return None
    spec = cfg["key_values"].get(trials[0].mode)
    values = [key_value(t, cfg) for t in trials]
    if spec is None or any(v is None for v in values):
        return None
    first, last = sum(values[:k]) / k, sum(values[-k:]) / k
    if first == 0:
        return None
    return (first - last) / first if spec["better"] == "lower" else (last - first) / first


def fatigue_score(signals: Iterable[str], cfg: dict[str, Any] | None = None) -> float:
    """신호(voice_tired · rule_layer.SIGNAL_LABELS 키) → 0~1. 모르는 신호는 0."""
    weights = (cfg or load())["metrics"]["fatigue_score"]["weights"]
    return float(min(1.0, sum(weights.get(s, 0.0) for s in set(signals))))
