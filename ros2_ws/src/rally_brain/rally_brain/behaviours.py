"""rally_brain/rally_brain/behaviours.py — 행동 트리 리프 블록 (모드 서브트리가 조립해서 쓴다)
담당: 조정묵

  WaitUntil  조건이 참이 될 때까지 RUNNING (timeout_s 넘으면 FAILURE)
  SkillCall  스킬 action 을 비동기로 보내고 결과가 오면 SUCCESS — 로봇이 움직이는 동안 트리·안전 감시가 막히지 않는다.
             결과 메시지는 on_result 로 넘긴다 (스킬의 success=false 도 SUCCESS: 시도는 끝났고 판정은 기록 쪽에서).
             서버 없음·goal 거절·abort·취소는 FAILURE. 트리가 중단되면(terminate) 진행 중 goal 을 취소하고 on_interrupt 호출
             (중단된 시도도 TrialResult 로 남기기 위해).
  Do         함수 한 번 실행 후 SUCCESS (TrialResult 발행 등). 함수가 False 를 돌려주면 FAILURE
"""
from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

import py_trees
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

Status = py_trees.common.Status
GOAL_STATUS_NAMES = {GoalStatus.STATUS_ABORTED: "aborted", GoalStatus.STATUS_CANCELED: "canceled"}


class WaitUntil(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, condition: Callable[[], bool], timeout_s: float = 0.0):
        super().__init__(name)
        self.condition = condition
        self.timeout_s = timeout_s   # 0 = 무한 대기
        self._t0 = 0.0

    def initialise(self) -> None:
        self._t0 = time.monotonic()

    def update(self) -> Status:
        if self.condition():
            return Status.SUCCESS
        if self.timeout_s > 0 and time.monotonic() - self._t0 > self.timeout_s:
            self.feedback_message = f"{self.timeout_s:.0f} s 초과"
            return Status.FAILURE
        return Status.RUNNING


class SkillCall(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node, client: ActionClient, make_goal: Callable[[], Any],
                 on_result: Callable[[Any], None], on_interrupt: Callable[[], None] | None = None,
                 server_timeout_s: float = 3.0):
        super().__init__(name)
        self.node = node
        self.client = client
        self.make_goal = make_goal
        self.on_result = on_result
        self.on_interrupt = on_interrupt
        self.server_timeout_s = server_timeout_s
        self._send_future = None
        self._result_future = None
        self._goal_handle = None
        self._t0 = 0.0

    def initialise(self) -> None:
        self._send_future = self._result_future = self._goal_handle = None
        self._t0 = time.monotonic()

    @property
    def goal_sent(self) -> bool:
        """이번 실행에서 goal 을 보냈는지 (실패 시 시도로 기록할지 판단용)."""
        return self._send_future is not None

    def update(self) -> Status:
        if self._send_future is None:
            if not self.client.server_is_ready():
                if time.monotonic() - self._t0 > self.server_timeout_s:
                    self.feedback_message = f"스킬 서버 없음 ({self.server_timeout_s:.0f} s)"
                    self.node.get_logger().error(f"[{self.name}] {self.feedback_message}")
                    return Status.FAILURE
                return Status.RUNNING
            self._send_future = self.client.send_goal_async(self.make_goal())
            return Status.RUNNING
        if self._goal_handle is None:
            if not self._send_future.done():
                return Status.RUNNING
            self._goal_handle = self._send_future.result()
            if not self._goal_handle.accepted:
                self.feedback_message = "goal 거절"
                self.node.get_logger().warn(f"[{self.name}] goal 거절")
                return Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
        if not self._result_future.done():
            return Status.RUNNING
        response = self._result_future.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = GOAL_STATUS_NAMES.get(response.status, f"status_{response.status}")
            self.node.get_logger().warn(f"[{self.name}] 스킬 {self.feedback_message}")
            return Status.FAILURE
        self.on_result(response.result)
        return Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        # 실행 중에 트리가 중단되면(세션 취소·안전 정지) 로봇 스킬도 취소
        if new_status == Status.INVALID and self._goal_handle is not None and self._result_future is not None \
                and not self._result_future.done():
            self.node.get_logger().info(f"[{self.name}] 중단 — goal 취소 요청")
            self._goal_handle.cancel_goal_async()
            if self.on_interrupt is not None:
                self.on_interrupt()


class Do(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, fn: Callable[[], bool | None]):
        super().__init__(name)
        self.fn = fn

    def update(self) -> Status:
        return Status.FAILURE if self.fn() is False else Status.SUCCESS
