"""brain 트리가 부르는 로봇 Trigger 서비스(/rally/robot/go_*)는 모두 세션 시작 조건(readiness.yaml robot)에 있어야 한다.
없으면 서비스가 안 떠 있을 때 세션이 시작된 뒤 첫 호출에서 기다리다 실패한다 (#132 리뷰)."""
import re
from pathlib import Path

import yaml

from rally_brain.config import config_path

BRAIN = Path(__file__).resolve().parents[1] / "rally_brain"


def robot_services() -> set[str]:
    cfg = yaml.safe_load(config_path("readiness.yaml").read_text(encoding="utf-8"))
    return {s["name"] for s in cfg["robot"].get("services", [])}


def test_robot_services_called_by_brain_are_in_readiness():
    called = set()
    for path in BRAIN.rglob("*.py"):
        called |= set(re.findall(r'"(/rally/robot/go_[a-z_]+)"', path.read_text(encoding="utf-8")))
    assert "/rally/robot/go_hand_view" in called          # 검사 자체가 서비스를 찾는지
    missing = called - robot_services()
    assert not missing, f"readiness.yaml robot.services 에 없음: {sorted(missing)}"


def test_pose_services_listed():
    assert {"/rally/robot/go_observe_pose", "/rally/robot/go_pick_view"} <= robot_services()
