"""rally_brain/rally_brain/behaviours.py — 행동 트리 리프 블록 (모드 서브트리가 조립해서 쓴다)
담당: 조정묵

  WaitUntil  조건이 참이 될 때까지 RUNNING (timeout_s 넘으면 FAILURE)
  SkillCall  스킬 action 을 비동기로 보내고 결과가 오면 SUCCESS — 로봇이 움직이는 동안 트리·안전 감시가 막히지 않는다.
             결과 메시지는 on_result 로 넘긴다 (스킬의 success=false 도 SUCCESS: 시도는 끝났고 판정은 기록 쪽에서).
             서버 없음·goal 거절·abort·취소는 FAILURE. abort 여도 결과 메시지(failure_reason 등)는 .result 에 남는다. 트리가 중단되면(terminate) goal 을 보낸 시도는 반드시 기록하고
             (이미 끝났으면 그 결과, 아니면 on_interrupt), 아직 돌고 있거나 수락 전이면 취소한다.
  Do         함수 한 번 실행 후 SUCCESS (TrialResult 발행 등). 함수가 False 를 돌려주면 FAILURE
  Guard      시도 서브트리를 감싸, 트리가 도중에 멈추면(INVALID) on_stop 을 부른다. 자식이 먼저 멈춰
             SkillCall.on_interrupt 가 실제 이유로 기록할 기회를 먼저 갖는다 — on_stop 은 그래도 기록이 안 된 경우
             (스킬과 스킬 사이, 중간 스킬이 성공한 직후 등)를 위한 것이라 기록 여부는 on_stop 쪽이 확인한다
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
                 on_result: Callable[[Any], None], on_interrupt: Callable[[str], None] | None = None,
                 server_timeout_s: float = 3.0, can_send: Callable[[], bool] = lambda: True):
        super().__init__(name)
        self.node = node
        self.client = client
        self.make_goal = make_goal
        self.on_result = on_result
        self.on_interrupt = on_interrupt   # 인자: interrupted | aborted | canceled ...
        self.can_send = can_send
        self.server_timeout_s = server_timeout_s
        self._reset()

    def _reset(self) -> None:
        self._send_future = None
        self._result_future = None
        self._goal_handle = None
        self._handled = False          # 결과(또는 거절)를 update 에서 처리했는지
        self.result = None             # 마지막 결과 메시지 (abort 포함) — 재시도 판단용
        self._cancel_on_accept = False  # 수락 전에 중단됨 → 수락되자마자 취소
        self._t0 = time.monotonic()

    def initialise(self) -> None:
        self._reset()

    @property
    def goal_sent(self) -> bool:
        """이번 실행에서 goal 을 보냈는지 (실패 시 시도로 기록할지 판단용)."""
        return self._send_future is not None

    def _on_goal_response(self, future) -> None:
        # executor 스레드. 수락 전에 중단됐으면 수락되자마자 취소 (리뷰 #41-2)
        goal_handle = future.result()
        if self._cancel_on_accept and goal_handle is not None and goal_handle.accepted:
            self.node.get_logger().info(f"[{self.name}] 중단 뒤 수락됨 — 바로 취소")
            goal_handle.cancel_goal_async()

    def update(self) -> Status:
        if self._send_future is None:
            if not self.client.server_is_ready():
                if time.monotonic() - self._t0 > self.server_timeout_s:
                    self.feedback_message = f"스킬 서버 없음 ({self.server_timeout_s:.0f} s)"
                    self.node.get_logger().error(f"[{self.name}] {self.feedback_message}")
                    return Status.FAILURE
                return Status.RUNNING
            if not self.can_send():
                self.feedback_message = "precondition_failed"
                return Status.FAILURE
            self._send_future = self.client.send_goal_async(self.make_goal())
            self._send_future.add_done_callback(self._on_goal_response)
            return Status.RUNNING
        if self._goal_handle is None:
            if not self._send_future.done():
                return Status.RUNNING
            self._goal_handle = self._send_future.result()
            if not self._goal_handle.accepted:
                self._handled = True
                self.feedback_message = "goal 거절"
                self.node.get_logger().warn(f"[{self.name}] goal 거절")
                return Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
        if not self._result_future.done():
            return Status.RUNNING
        self._handled = True
        response = self._result_future.result()
        self.result = response.result
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = GOAL_STATUS_NAMES.get(response.status, f"status_{response.status}")
            self.node.get_logger().warn(f"[{self.name}] 스킬 {self.feedback_message}")
            return Status.FAILURE
        self.on_result(response.result)
        return Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        """트리가 중단되면(세션 취소·안전 정지): goal 을 보냈고 결과를 아직 처리 안 했으면 반드시 기록하고,
        스킬이 아직 돌고 있으면 취소한다. 실로봇에선 skill_server 가 STOP 을 먼저 보고 abort 해서
        중단 시점에 결과가 이미 와 있는 경우가 흔하다 (리뷰 #41-1)."""
        if new_status != Status.INVALID or not self.goal_sent or self._handled:
            return
        self._handled = True
        if self._result_future is None and self._send_future.done():
            goal_handle = self._send_future.result()
            if goal_handle is not None and goal_handle.accepted:
                self._goal_handle = goal_handle
                self._result_future = goal_handle.get_result_async()
        if self._result_future is not None and self._result_future.done():
            response = self._result_future.result()
            self.result = response.result
            if response.status == GoalStatus.STATUS_SUCCEEDED:
                self.on_result(response.result)           # 중단 직전에 끝난 시도는 실제 결과로
            else:
                self._interrupted(GOAL_STATUS_NAMES.get(response.status, f"status_{response.status}"))
            return
        if self._goal_handle is not None:
            self.node.get_logger().info(f"[{self.name}] 중단 — goal 취소 요청")
            self._goal_handle.cancel_goal_async()
        else:
            self._cancel_on_accept = True                  # 아직 수락 전 → _on_goal_response 에서 취소
            if self._send_future.done():                   # 그 사이 응답이 왔으면 직접
                self._on_goal_response(self._send_future)
        self._interrupted("interrupted")

    def _interrupted(self, reason: str) -> None:
        if self.on_interrupt is not None:
            self.on_interrupt(reason)


class Do(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, fn: Callable[[], bool | None]):
        super().__init__(name)
        self.fn = fn

    def update(self) -> Status:
        return Status.FAILURE if self.fn() is False else Status.SUCCESS


class Guard(py_trees.composites.Sequence):
    def __init__(self, name: str, child: py_trees.behaviour.Behaviour, on_stop: Callable[[], None]):
        super().__init__(name, memory=True, children=[child])
        self.on_stop = on_stop

    def terminate(self, new_status: Status) -> None:
        # Composite.stop 은 자식을 먼저 멈춘 뒤 terminate 를 부른다 (py_trees 2.x)
        if new_status == Status.INVALID:
            self.on_stop()
