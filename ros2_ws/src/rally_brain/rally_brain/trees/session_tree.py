"""rally_brain/rally_brain/trees/session_tree.py — 세션 트리: 모드들을 순서대로, 모드마다 시도 N회
담당: 조정묵

  세션 (Sequence)
   ├─ 자세 기준 잡기 — 안내 → /rally/perception/body/reset_baseline → 기준 잡는 동안 대기 (#83). 세션에 한 번.
   │    body_pose 가 없거나 실패해도 경고만 남기고 넘어간다 (보상 동작 감지만 빠짐). 값은 skills.yaml brain.body_baseline_*
   ├─ 모드 1 × 공 개수 (Repeat) ─ 시도 서브트리 (trees/mode1_push.py) — 시도 수는 TRIAL_COUNTS (거치대 공 4개 = 한 사이클)
   ├─ 모드 5 × N (Repeat) ─ 시도 서브트리 (trees/mode5_handover.py)
   ├─ 모드 7 × N (Repeat) ─ 시도 서브트리 (trees/mode7_resist.py)
   └─ ...
시도 서브트리는 build_trial(session) → Behaviour 를 내놓는다. 한 번 SUCCESS = 시도 1회 끝 (성공·실패는 TrialResult 에).
아직 서브트리가 없는 모드는 로그만 남기고 건너뛴다.
모드마다 맨 앞에서 session.current_mode 를 기록한다 — 모드 전환(SetMode·음성)은 modes_after_switch 로 남은 순서를 다시 만든다.
"""
from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any
from dataclasses import dataclass, field

import py_trees
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from rally_brain.config import load_table
from rally_brain.behaviours import Do, ServiceCall, WaitUntil
from rally_brain.trees import mode1_push, mode5_handover, mode7_resist
from rally_interfaces.msg import SafetyStatus, SceneContext, TrialResult

OBSERVE_SERVICE = "/rally/robot/go_observe_pose"
MODE_NAMES = {1: "공 굴리기", 3: "캐치볼", 5: "건네고 받기", 7: "팔씨름"}
BODY_BASELINE_SERVICE = "/rally/perception/body/reset_baseline"
BODY_BASELINE_DEFAULTS = {"body_baseline_prompt": "기준 자세를 잡을게요. 바르게 서서 잠깐 가만히 있어 주세요.",
                          "body_baseline_prompt_wait_s": 2.5, "body_baseline_hold_s": 3.5}
NOT_PERFORMANCE = {"no_hand"}   # 환자가 자리에 없었던 시도 — 기록은 하되 난이도 규칙에는 안 셈

TRIAL_BUILDERS: dict[int, Callable[["Session"], py_trees.behaviour.Behaviour]] = {
    TrialResult.MODE_PUSH_BALL: mode1_push.build_trial,
    TrialResult.MODE_HANDOVER: mode5_handover.build_trial,
    TrialResult.MODE_RESIST: mode7_resist.build_trial,
}
# 모드 시도 수가 장비로 정해지는 경우 (skills.yaml → 횟수). 여기 없는 모드는 trials_per_mode
TRIAL_COUNTS: dict[int, Callable[[dict], int]] = {
    TrialResult.MODE_PUSH_BALL: mode1_push.trials_per_cycle,
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
    service_client: Callable[[type, str], Any] | None = None        # 서비스 클라이언트 (예: go_hand_view). None 이면 그 단계 건너뜀(테스트)
    after_trial: Callable[["Session", TrialResult], None] | None = None   # 규칙 레이어 (bt_runner)
    trials: list[TrialResult] = field(default_factory=list)
    difficulty: dict[int, int] = field(default_factory=dict)        # 모드 → 난이도 단계 (0 = 시작)
    level_changed_at: dict[int, int] = field(default_factory=dict)  # 모드 → 난이도가 바뀐 시점의 len(trials)
    modes: list[int] = field(default_factory=list)                  # 지금 트리의 모드 순서 (전환하면 바뀜)
    current_mode: int | None = None                                 # 지금 진행 중인 모드
    trial_active: bool = False                                      # 로봇이 시도 중 (begin_trial ~ record). 모드 전환은 이때 안 함
    allow_trial: Callable[[], bool] = lambda: True                   # 시도 전체(손 대기 포함) 시작 전 휴식 관문
    robot_resume_ok: Callable[[], bool] = lambda: True               # 손 근접 STOP 으로 멈춘 명령을 다시 보내도 되는가 (#98, bt_runner)
    body_baseline_done: bool = False                                # 자세 기준 잡기를 지났음 (모드 전환으로 트리를 새로 만들어도 다시 안 함)

    table_cfg: dict = field(default_factory=load_table)             # 중앙선 원본: table.yaml

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
        if mode == TrialResult.MODE_PUSH_BALL:
            return []  # 손동작은 받기 후보 신호. 실물 판정 검증 전 자동 난이도 금지.
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


def elapsed(seconds: float) -> Callable[[], bool]:
    """처음 부른 때부터 seconds 가 지나면 True — WaitUntil 조건용 (블록이 다시 시작되면 새로 만든다)."""
    t0: list[float] = []

    def check() -> bool:
        now = time.monotonic()
        if not t0:
            t0.append(now)
        return now - t0[0] >= seconds
    return check


def body_baseline(session: Session) -> py_trees.behaviour.Behaviour:
    """세션 시작 때 몸 자세 기준 잡기 (#83). 안내 → 자세 잡을 시간 → reset_baseline → 기준 잡는 동안 대기.
    서비스가 없거나 실패하면 경고만 — body_pose 는 보상 동작 감지용이라 세션을 막지 않는다.
    훈련 팔을 바꾸면 다시 불러야 하지만, brain 은 훈련 팔을 모르므로 지금은 운영자(확인 창 b)가 한다."""
    cfg = {**BODY_BASELINE_DEFAULTS, **(session.skills_cfg.get("brain") or {})}
    log = session.node.get_logger()

    def warn_failed() -> None:
        log.warn("[자세 기준] reset_baseline 실패·없음 — 보상 동작 값은 NaN 으로 남을 수 있음 (세션은 계속)")

    def done() -> None:
        session.body_baseline_done = True

    call = py_trees.composites.Selector("기준 호출", memory=True, children=[
        ServiceCall("reset_baseline", session.service_client(Trigger, BODY_BASELINE_SERVICE), Trigger.Request,
                    timeout_s=3.0, server_timeout_s=2.0),
        Do("기준 호출 실패 — 계속", warn_failed),
    ])
    hold = py_trees.composites.Sequence("기준 잡는 동안", memory=True, children=[
        call, WaitUntil("기준 잡는 중", elapsed(float(cfg["body_baseline_hold_s"]))),
    ])
    return py_trees.composites.Sequence("자세 기준 잡기", memory=True, children=[
        Do("자세 안내", lambda: session.say(str(cfg["body_baseline_prompt"]))),
        WaitUntil("자세 잡을 시간", elapsed(float(cfg["body_baseline_prompt_wait_s"]))),
        hold, Do("자세 기준 끝", done),
    ])


def build_session_tree(session: Session, modes: list[int], trials_per_mode: int) -> py_trees.behaviour.Behaviour:
    session.modes = list(modes)
    root = py_trees.composites.Sequence("세션", memory=True)
    if session.service_client is not None and not session.body_baseline_done:
        root.add_child(body_baseline(session))
    for mode in modes:
        name = f"모드 {mode} {MODE_NAMES.get(mode, '?')}"

        def enter(m: int = mode, n: str = name) -> None:
            if session.current_mode != m:
                session.current_mode = m
                session.node.get_logger().info(f"{n} 시작")

        builder = TRIAL_BUILDERS.get(mode)
        if builder is None:
            root.add_child(Do(f"{name} 건너뜀",
                              lambda n=name: session.node.get_logger().warn(f"{n} — 시도 서브트리가 아직 없어 건너뜀")))
            continue
        count = TRIAL_COUNTS[mode](session.skills_cfg) if mode in TRIAL_COUNTS else trials_per_mode
        trial = py_trees.composites.Sequence(name, memory=True, children=[
            WaitUntil("휴식 대기", session.allow_trial),
            Do(f"{name} 시작", enter), builder(session),
        ])
        repeat = py_trees.decorators.Repeat(f"{name} × {count}", trial, count)
        if session.service_client is None:              # 테스트 — 자세 이동 없음
            root.add_child(repeat)
            continue
        # 10/11: 모든 모드는 관측 자세에서 시작 (테이블·중앙선·환자 손이 화면에 들어옴). 못 가면 세션 실패
        root.add_child(py_trees.composites.Sequence(name, memory=True, children=[
            ServiceCall(f"{name} — 관측 자세로", session.service_client(Trigger, OBSERVE_SERVICE), Trigger.Request,
                        timeout_s=40.0),
            repeat]))
    return root
