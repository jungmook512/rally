"""rally_db/rally_db/embeddings.py — 시도 1회 → 숫자 벡터 (trials.feature_vec), 비슷한 예전 시도 찾기용
담당: 조정묵

  load_spec()               rally_bringup/config/embeddings.yaml (+ skills.yaml 의 range_ref) → Spec
  trial_vector(row, spec)   convert.trial_to_row 의 dict → dim 개 숫자 (0~1). 벡터를 안 만드는 시도면 None
  python3 -m rally_db.embeddings --backfill [--all]   벡터가 없는(--all: 전부) trials 를 다시 계산

같은 모드끼리만 비교한다 (repository.find_similar_trials 가 mode 로 거른다).
거리는 L2(유클리드): 각 칸이 0~1 로 맞춰져 있어서 "값이 얼마나 다른가"를 그대로 잰다.
코사인은 방향만 보므로 [0.1, 0.1] 과 [0.9, 0.9] 를 같다고 본다 — 이 용도엔 맞지 않는다.
ROS 를 import 하지 않는다 (pytest 로 확인).
"""
from __future__ import annotations

import argparse
import logging
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

SRC_CONFIG = Path(__file__).resolve().parents[2] / "rally_bringup" / "config"
SOURCES = ("field", "param", "metric")


@dataclass(frozen=True)
class Feature:
    source: str
    name: str
    lo: float
    hi: float

    def scale(self, value: float) -> float:
        if math.isnan(value):
            return 0.0
        return min(1.0, max(0.0, (value - self.lo) / (self.hi - self.lo)))


@dataclass(frozen=True)
class Spec:
    version: int
    dim: int
    common: tuple[Feature, ...]
    modes: dict[int, tuple[Feature, ...]]
    skip_notes: frozenset[str]

    def features(self, mode: int) -> tuple[Feature, ...] | None:
        if mode not in self.modes:
            return None
        return self.common + self.modes[mode]


def config_path(name: str) -> Path:
    """설치된 rally_bringup share → 소스 트리 순."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("rally_bringup")) / "config" / name
        if path.exists():
            return path
    except Exception:  # noqa: BLE001 — ROS 환경 밖(단위 테스트)
        pass
    return SRC_CONFIG / name


def _read(name: str) -> dict[str, Any]:
    with open(config_path(name), encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _lookup(cfg: dict, ref: str) -> Any:
    node: Any = cfg
    for part in ref.split("."):
        if not isinstance(node, dict) or part not in node:
            raise ValueError(f"range_ref {ref} 가 skills.yaml 에 없음")
        node = node[part]
    return node


def _feature(entry: dict, skills: dict, where: str) -> Feature:
    if entry.get("source") not in SOURCES:
        raise ValueError(f"{where}: source 는 {SOURCES} 중 하나")
    rng = _lookup(skills, entry["range_ref"]) if "range_ref" in entry else entry.get("range")
    if not (isinstance(rng, list) and len(rng) == 2 and all(isinstance(v, (int, float)) for v in rng)
            and rng[0] < rng[1]):
        raise ValueError(f"{where}: 범위 {rng!r} 는 [min, max] (min < max) 여야 함")
    return Feature(entry["source"], entry["name"], float(rng[0]), float(rng[1]))


def load_spec(spec_cfg: dict | None = None, skills: dict | None = None) -> Spec:
    """설정이 틀리면 ValueError — 조용히 이상한 벡터를 저장하지 않는다."""
    spec_cfg = spec_cfg if spec_cfg is not None else _read("embeddings.yaml")
    skills = skills if skills is not None else _read("skills.yaml")
    dim = int(spec_cfg["dim"])
    common = tuple(_feature(e, skills, f"common[{i}]") for i, e in enumerate(spec_cfg.get("common") or []))
    modes = {}
    for mode, entries in (spec_cfg.get("modes") or {}).items():
        feats = tuple(_feature(e, skills, f"modes.{mode}[{i}]") for i, e in enumerate(entries or []))
        if len(common) + len(feats) > dim:
            raise ValueError(f"모드 {mode}: 값 {len(common) + len(feats)} 개 > dim {dim}")
        names = [f.name for f in common + feats]
        if len(set(names)) != len(names):
            raise ValueError(f"모드 {mode}: 같은 이름이 두 번 있음 {names}")
        modes[int(mode)] = feats
    return Spec(int(spec_cfg.get("version", 1)), dim, common, modes, frozenset(spec_cfg.get("skip_notes") or []))


def _value(row: dict[str, Any], f: Feature) -> float:
    if f.source == "field":
        v = row.get(f.name)
    else:
        v = (row.get("params" if f.source == "param" else "metrics") or {}).get(f.name)
    return 0.0 if v is None else float(v)


def trial_vector(row: dict[str, Any], spec: Spec) -> list[float] | None:
    """row: convert.trial_to_row 결과 (또는 같은 키를 가진 DB 한 줄).
    설정에 없는 모드이거나 skip_notes 시도(no_hand 등)면 None — 그 시도는 검색에 안 나온다."""
    notes = (row.get("decision") or {}).get("notes", "")
    feats = spec.features(int(row["mode"]))
    if feats is None or notes in spec.skip_notes:
        return None
    vec = [f.scale(_value(row, f)) for f in feats]
    return vec + [0.0] * (spec.dim - len(vec))


def backfill(repo, spec: Spec, everything: bool = False) -> tuple[int, int]:
    """trials 의 feature_vec 를 다시 계산. (갱신한 줄, 벡터 없음으로 둔 줄)."""
    updated = skipped = 0
    for row in repo.trials_for_embedding(everything):
        vec = trial_vector(row, spec)
        repo.set_feature_vec(row["id"], vec)
        if vec is None:
            skipped += 1
        else:
            updated += 1
    return updated, skipped


def main() -> None:
    ap = argparse.ArgumentParser(description="trials.feature_vec 다시 계산 (embeddings.yaml 을 바꾼 뒤)")
    ap.add_argument("--backfill", action="store_true", required=True)
    ap.add_argument("--all", action="store_true", help="벡터가 있는 줄도 전부 다시 (기본: 없는 줄만)")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    from rally_db.repository import Repository
    spec = load_spec()
    repo = Repository()
    try:
        updated, skipped = backfill(repo, spec, args.all)
    finally:
        repo.close()
    logging.info(f"embeddings v{spec.version}: {updated} 줄 계산, {skipped} 줄 벡터 없음")


if __name__ == "__main__":
    main()
