"""rally_brain/rally_brain/trees/session_tree.py — 세션 트리: 모드들을 순서대로, 모드마다 시도 N회
담당: 조정묵

  세션 (Sequence)
   ├─ 모드 1 × N (Repeat) ─ 시도 서브트리 (trees/mode1_push.py)
   ├─ 모드 5 × N (Repeat) ─ 시도 서브트리 (trees/mode5_handover.py)
   └─ ...
시도 서브트리는 build_trial(session) → Behaviour 를 내놓는다. 한 번 SUCCESS = 시도 1회 끝 (성공·실패는 TrialResult 에).
아직 서브트리가 없는 모드는 로그만 남기고 건너뛴다.
"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

import py_trees
from rclpy.action import ActionClient
from rclpy.node import Node

from rally_brain.behaviours import Do
from rally_brain.trees import mode5_handover
from rally_interfaces.msg import SafetyStatus, SceneContext, TrialResult

MODE_NAMES = {1: "공 굴리기", 3: "캐치볼", 5: "건네고 받기", 7: "팔씨름"}

TRIAL_BUILDERS: dict[int, Callable[["Session"], py_trees.behaviour.Behaviour]] = {
    TrialResult.MODE_HANDOVER: mode5_handover.build_trial,
}


@dataclass
class Session:
    """시도 서브트리가 쓰는 세션 상태와 도구. bt_runner 가 만든다."""
    node: Node
    session_id: str
    get_context: Callable[[], SceneContext | None]
    publish: Callable[[TrialResult], None]
    action_client: Callable[[type, str], ActionClient]
    trials: list[TrialResult] = field(default_factory=list)

    def safe(self) -> bool:
        ctx = self.get_context()
        return ctx is not None and ctx.safety.level != SafetyStatus.LEVEL_STOP

    def hand_visible(self) -> bool:
        ctx = self.get_context()
        return ctx is not None and ctx.hand.detected

    def record(self, trial: TrialResult) -> None:
        """session_id·trial_index·stamp 를 채워 발행. 모든 시도는 여기를 거친다 (rally_brain/CLAUDE.md)."""
        trial.session_id = self.session_id
        trial.trial_index = len(self.trials) + 1
        trial.header.stamp = self.node.get_clock().now().to_msg()
        self.trials.append(trial)
        self.publish(trial)
        self.node.get_logger().info(
            f"시도 {trial.trial_index} (모드 {trial.mode}) {'성공' if trial.success else '실패'}"
            + (f" — {trial.notes}" if trial.notes else ""))

    def success_rate(self) -> float:
        return sum(t.success for t in self.trials) / len(self.trials) if self.trials else 0.0


def build_session_tree(session: Session, modes: list[int], trials_per_mode: int) -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence("세션", memory=True)
    for mode in modes:
        name = f"모드 {mode} {MODE_NAMES.get(mode, '?')}"
        builder = TRIAL_BUILDERS.get(mode)
        if builder is None:
            root.add_child(Do(f"{name} 건너뜀",
                              lambda n=name: session.node.get_logger().warn(f"{n} — 시도 서브트리가 아직 없어 건너뜀")))
            continue
        root.add_child(py_trees.decorators.Repeat(f"{name} × {trials_per_mode}", builder(session), trials_per_mode))
    return root
