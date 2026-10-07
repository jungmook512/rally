"""rally_brain/rally_brain/knowledge/ontology.py — ontology.yaml 읽기 + 사람이 읽는 문장 만들기
담당: 조정묵

  load()                                    ontology.yaml (패키지 안에 같이 설치됨)
  label(kind, key)                          한국어 이름 — 대시보드·리포트용 (없으면 key 그대로)
  describe_compensation(mode, comp_type)    LLM 맥락 문장. 가정(verified: false)이면 "(가정)" 표시
  check_range_refs(onto, skills_cfg)        range_ref 가 skills.yaml 의 [min, max], choices_ref 가 목록인지 — 문제 목록
"""
from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml

ONTOLOGY_PATH = Path(__file__).with_name("ontology.yaml")


@lru_cache(maxsize=1)
def load(path: str | None = None) -> dict[str, Any]:
    with open(path or ONTOLOGY_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f)


def label(kind: str, key: str | int, onto: dict | None = None) -> str:
    """kind: modes · compensations · fatigue_signals · skills · body_parts."""
    entry = (onto or load()).get(kind, {}).get(key)
    if isinstance(entry, str):
        return entry
    if isinstance(entry, dict) and "name" in entry:
        return entry["name"]
    return str(key)


def _assumed(entry: dict) -> str:
    return "" if entry.get("verified", False) else " (가정)"


def describe_compensation(mode: int, comp_type: str, onto: dict | None = None) -> str:
    """예: '모드 1 공 굴리기(팔 뻗기) 중 몸통 기울이기 관찰 — 팔 대신 … 뜻 (가정). 피로 신호: 보상 동작 반복(중간)'"""
    onto = onto or load()
    comp = onto["compensations"].get(comp_type)
    if comp is None:
        return ""
    m = onto["modes"].get(mode)
    where = f"모드 {mode} {m['name']}({m['motion']['name']}) 중 " if m else ""
    signal = onto["fatigue_signals"][comp["signal"]]
    strength = {"strong": "강함", "medium": "중간", "weak": "약함"}[signal["strength"]]
    return (f"{where}{comp['name']} 관찰 — {comp['meaning']['text']}{_assumed(comp['meaning'])}. "
            f"피로 신호: {signal['name']}({strength})")


def _lookup(cfg: dict, ref: str) -> tuple[bool, Any]:
    node: Any = cfg
    for part in ref.split("."):
        if not isinstance(node, dict) or part not in node:
            return False, None
        node = node[part]
    return True, node


def _is_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def check_range_refs(onto: dict, skills_cfg: dict) -> list[str]:
    """ontology 의 range_ref·choices_ref 가 skills.yaml 에서 올바른지. 문제 목록 (빈 목록이면 정상).
    range_ref 는 숫자 2개 [min, max] (min ≤ max), choices_ref 는 비어 있지 않은 목록이어야 한다."""
    problems = []
    for skill, s in onto["skills"].items():
        for param, p in (s.get("params") or {}).items():
            where = f"{skill}.{param}"
            if (ref := p.get("range_ref")) is not None:
                found, v = _lookup(skills_cfg, ref)
                if not found:
                    problems.append(f"{where} → {ref} 없음")
                elif not (isinstance(v, list) and len(v) == 2 and all(map(_is_number, v)) and v[0] <= v[1]):
                    problems.append(f"{where} → {ref} = {v!r} 는 [min, max] 범위가 아님")
            if (ref := p.get("choices_ref")) is not None:
                found, v = _lookup(skills_cfg, ref)
                if not (found and isinstance(v, list) and v):
                    problems.append(f"{where} → {ref} 는 비어 있지 않은 목록이어야 함")
    return problems
