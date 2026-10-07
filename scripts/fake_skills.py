#!/usr/bin/env python3
"""가짜 스킬 서버 + 가짜 상황판 — 로봇·카메라 없이 bt_runner 흐름 확인용.

    python3 scripts/fake_skills.py                    # handover 1 s 뒤 성공, 손 보임·안전 NORMAL 상황판 5 Hz
    python3 scripts/fake_skills.py --fail-every 3     # 3번째마다 success=false (failure_reason=timeout)
    python3 scripts/fake_skills.py --duration 5       # 스킬 하나에 5 s

context_builder 를 같이 띄우지 않는다 — 이 스크립트가 /rally/brain/context 를 직접 낸다.
안전 정지 흉내 (다른 터미널, 새 토픽을 만들지 않으려고 파라미터로):
    ros2 param set /rally_fake_skills stop true     (false 로 해제)
"""
from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult

from rally_interfaces.action import Handover
from rally_interfaces.msg import SafetyStatus, SceneContext


class FakeSkills(Node):
    def __init__(self, duration_s: float, fail_every: int):
        super().__init__("rally_fake_skills")
        self.duration_s = duration_s
        self.fail_every = fail_every
        self.count = 0
        self.stop = False
        group = ReentrantCallbackGroup()
        ActionServer(self, Handover, "/rally/skill/handover", self._handover,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.declare_parameter("stop", False)
        self.add_on_set_parameters_callback(self._on_params)
        self._ctx_pub = self.create_publisher(SceneContext, "/rally/brain/context", qos_profile_sensor_data)
        self.create_timer(0.2, self._publish_context)
        self.get_logger().info(f"가짜 handover {duration_s} s" + (f", {fail_every}번째마다 실패" if fail_every else ""))

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "stop":
                self.stop = bool(p.value)
                self.get_logger().warn("안전 STOP 흉내" if self.stop else "안전 NORMAL 복귀")
        return SetParametersResult(successful=True)

    def _publish_context(self) -> None:
        ctx = SceneContext()
        ctx.header.stamp = self.get_clock().now().to_msg()
        ctx.safety.level = SafetyStatus.LEVEL_STOP if self.stop else SafetyStatus.LEVEL_NORMAL
        ctx.safety.reason = "fake_stop" if self.stop else ""
        ctx.hand.detected = True
        self._ctx_pub.publish(ctx)

    def _handover(self, goal_handle) -> Handover.Result:
        self.count += 1
        n = self.count
        self.get_logger().info(f"handover #{n} 시작 (dir={goal_handle.request.direction})")
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.duration_s:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f"handover #{n} 취소됨")
                return Handover.Result(success=False, failure_reason="canceled")
            if self.stop:
                goal_handle.abort()
                self.get_logger().info(f"handover #{n} abort (STOP)")
                return Handover.Result(success=False, failure_reason="safety_stop")
            time.sleep(0.05)
        ok = not (self.fail_every and n % self.fail_every == 0)
        goal_handle.succeed()
        self.get_logger().info(f"handover #{n} {'성공' if ok else '실패'}")
        return Handover.Result(success=ok, wrist_angle_at_grasp_deg=12.5, handover_time_s=self.duration_s,
                               failure_reason="" if ok else "timeout")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=1.0)
    ap.add_argument("--fail-every", type=int, default=0)
    args = ap.parse_args()
    rclpy.init()
    node = FakeSkills(args.duration, args.fail_every)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
