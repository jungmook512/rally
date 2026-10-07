"""rally_brain/rally_brain/trees/session_tree.py — 세션 트리: 모드들을 순서대로, 모드마다 시도 N회
담당: 조정묵

  세션 (Sequence)
   ├─ 모드 1 × N (Repeat) ─ 시도 서브트리 (trees/mode1_push.py)
   ├─ 모드 5 × N (Repeat) ─ 시도 서브트리 (trees/mode5_handover.py)
   └─ ...
시도 서브트리는 build_trial(session) → Behaviour 를 내놓는다. 한 번 SUCCESS = 시도 1회 끝 (성공·실패는 TrialResult 에).
아직 서브트리가 없는 모드는 로그만 남기고 건너뛴다.
모드마다 맨 앞에서 session.current_mode 를 기록한다 — 모드 전환(SetMode·음성)은 modes_after_switch 로 남은 순서를 다시 만든다.
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
NOT_PERFORMANCE = {"no_hand"}   # 환자가 자리에 없었던 시도 — 기록은 하되 난이도 규칙에는 안 셈

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
    skills_cfg: dict = field(default_factory=dict)                  # skills.yaml 전체 (모드별 설정)
    say: Callable[[str], None] = lambda text: None                  # TTS 안내 (기다리지 않음)
    after_trial: Callable[["Session", TrialResult], None] | None = None   # 규칙 레이어 (bt_runner)
    trials: list[TrialResult] = field(default_factory=list)
    difficulty: dict[int, int] = field(default_factory=dict)        # 모드 → 난이도 단계 (0 = 시작)
    level_changed_at: dict[int, int] = field(default_factory=dict)  # 모드 → 난이도가 바뀐 시점의 len(trials)
    modes: list[int] = field(default_factory=list)                  # 지금 트리의 모드 순서 (전환하면 바뀜)
    current_mode: int | None = None                                 # 지금 진행 중인 모드
    trial_active: bool = False                                      # 로봇이 시도 중 (begin_trial ~ record). 모드 전환은 이때 안 함

    def safe(self) -> bool:
        ctx = self.get_context()
        return ctx is not None and ctx.safety.level != SafetyStatus.LEVEL_STOP

    def hand_visible(self) -> bool:
        ctx = self.get_context()
        return ctx is not None and ctx.hand.detected

    def begin_trial(self) -> None:
        """시도 서브트리가 로봇 동작을 시작할 때 부른다 (손 대기 뒤). record 까지는 모드 전환을 미룬다."""
        self.trial_active = True

    def record(self, trial: TrialResult) -> None:
        """session_id·trial_index·stamp 를 채워 발행. 모든 시도는 여기를 거친다 (rally_brain/CLAUDE.md)."""
        self.trial_active = False
        trial.session_id = self.session_id
        trial.trial_index = len(self.trials) + 1
        trial.header.stamp = self.node.get_clock().now().to_msg()
        self.trials.append(trial)
        self.publish(trial)
        self.node.get_logger().info(
            f"시도 {trial.trial_index} (모드 {trial.mode}) {'성공' if trial.success else '실패'}"
            + (f" — {trial.notes}" if trial.notes else ""))
        if self.after_trial is not None:
            self.after_trial(self, trial)

    def trials_for_rules(self, mode: int) -> list[TrialResult]:
        """이 모드에서 난이도가 마지막으로 바뀐 뒤의 시도 (규칙 레이어의 연속 횟수·추세 계산용). no_hand 는 뺀다."""
        return [t for t in self.trials[self.level_changed_at.get(mode, 0):]
                if t.mode == mode and t.notes not in NOT_PERFORMANCE]

    def set_difficulty(self, mode: int, level: int) -> None:
        if level != self.difficulty.get(mode, 0):
            self.difficulty[mode] = level
            self.level_changed_at[mode] = len(self.trials)

    def success_rate(self) -> float:
        return sum(t.success for t in self.trials) / len(self.trials) if self.trials else 0.0


def modes_after_switch(modes: list[int], current: int | None, target: int | None) -> list[int]:
    """모드 전환 뒤 남은 순서. target None = 다음 모드로. 고른 모드부터 원래 순서대로 이어 간다.
    예) [1, 5, 7] 에서 5 진행 중: None → [7], 7 → [7], 1 → [1, 5, 7]. 순서에 없던 모드는 그 모드 뒤에 원래 남은 순서."""
    after_current = modes[modes.index(current) + 1:] if current in modes else list(modes)
    if target is None:
        return after_current
    if target in modes:
        return modes[modes.index(target):]
    return [target] + [m for m in after_current if m != target]


def build_session_tree(session: Session, modes: list[int], trials_per_mode: int) -> py_trees.behaviour.Behaviour:
    session.modes = list(modes)
    root = py_trees.composites.Sequence("세션", memory=True)
    for mode in modes:
        name = f"모드 {mode} {MODE_NAMES.get(mode, '?')}"

        def enter(m: int = mode, n: str = name) -> None:
            session.current_mode = m
            session.node.get_logger().info(f"{n} 시작")

        builder = TRIAL_BUILDERS.get(mode)
        if builder is None:
            root.add_child(Do(f"{name} 건너뜀",
                              lambda n=name: session.node.get_logger().warn(f"{n} — 시도 서브트리가 아직 없어 건너뜀")))
            continue
        root.add_child(py_trees.composites.Sequence(name, memory=True, children=[
            Do(f"{name} 시작", enter),
            py_trees.decorators.Repeat(f"{name} × {trials_per_mode}", builder(session), trials_per_mode),
        ]))
    return root
