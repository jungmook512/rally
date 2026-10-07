"""rally_brain/rally_brain/config.py — rally_bringup/config/*.yaml 읽기 (설치된 share → 소스 트리 순)"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

SRC_CONFIG = Path(__file__).resolve().parents[2] / "rally_bringup" / "config"


def config_path(name: str) -> Path | None:
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory("rally_bringup")) / "config" / name)
    except Exception:  # noqa: BLE001 — ROS 환경 밖(단위 테스트)이면 소스 트리
        pass
    candidates.append(SRC_CONFIG / name)
    return next((c for c in candidates if c.exists()), None)


def load_skills() -> dict[str, Any]:
    """skills.yaml 전체. 못 찾으면 빈 dict (호출하는 쪽이 기본값을 쓴다)."""
    path = config_path("skills.yaml")
    if path is None:
        return {}
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}
