#!/usr/bin/env python3
"""가짜 스킬 서버 + 가짜 상황판 — 로봇·카메라 없이 bt_runner 흐름 확인용.

    python3 scripts/fake_skills.py                    # 스킬(push_ball·pick_compliant·handover·resist_push) 1 s 뒤 성공, 손 보임·막대 1개·안전 NORMAL 상황판 5 Hz
    python3 scripts/fake_skills.py --fail-every 3     # 3번째마다 실패 (실제 스킬처럼 abort + failure_reason)
    python3 scripts/fake_skills.py --fail-every 1 --fail-reason no_object   # 매번 실패, 이유 지정
    python3 scripts/fake_skills.py --duration 5       # 스킬 하나에 5 s

context_builder 를 같이 띄우지 않는다 — 이 스크립트가 /rally/brain/context 를 직접 낸다.
안전 정지 흉내 (다른 터미널, 새 토픽을 만들지 않으려고 파라미터로):
    ros2 param set /rally_fake_skills stop true     (false 로 해제)
손 숨기기 (모드 5 손 대기 확인):
    ros2 param set /rally_fake_skills hand false    (true 로 다시 보임)
/rally/voice/speak 도 흉내 낸다 — 안내 문구가 이 스크립트 로그에 찍힌다.
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

from rally_interfaces.action import Handover, PickCompliant, PushBall, ResistPush
from rally_interfaces.msg import SafetyStatus, SceneContext, TrackedObject
from rally_interfaces.srv import Speak


class FakeSkills(Node):
    def __init__(self, duration_s: float, fail_every: int, fail_reason: str):
        super().__init__("rally_fake_skills")
        self.duration_s = duration_s
        self.fail_every = fail_every
        self.fail_reason = fail_reason
        self.count = 0
        self.stop = False
        self.hand = True
        group = ReentrantCallbackGroup()
        for kind, name in ((Handover, "handover"), (PickCompliant, "pick_compliant"), (PushBall, "push_ball"),
                           (ResistPush, "resist_push")):
            ActionServer(self, kind, f"/rally/skill/{name}", lambda h, k=kind, n=name: self._skill(h, k, n),
                         cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.create_service(Speak, "/rally/voice/speak", self._speak, callback_group=group)
        self.declare_parameter("stop", False)
        self.declare_parameter("hand", True)
        self.add_on_set_parameters_callback(self._on_params)
        self._ctx_pub = self.create_publisher(SceneContext, "/rally/brain/context", qos_profile_sensor_data)
        self.create_timer(0.2, self._publish_context)
        self.get_logger().info(f"가짜 스킬 {duration_s} s" + (f", {fail_every}번째마다 실패" if fail_every else ""))

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "stop":
                self.stop = bool(p.value)
                self.get_logger().warn("안전 STOP 흉내" if self.stop else "안전 NORMAL 복귀")
            elif p.name == "hand":
                self.hand = bool(p.value)
                self.get_logger().warn("손 보임" if self.hand else "손 숨김")
        return SetParametersResult(successful=True)

    def _publish_context(self) -> None:
        ctx = SceneContext()
        ctx.header.stamp = self.get_clock().now().to_msg()
        ctx.safety.level = SafetyStatus.LEVEL_STOP if self.stop else SafetyStatus.LEVEL_NORMAL
        ctx.safety.reason = "fake_stop" if self.stop else ""
        ctx.hand.detected = self.hand
        ctx.objects.header.frame_id = "base_link"
        rod = TrackedObject(class_name="rod", confidence=0.9)   # 모드 5 집기용 (거치 자리 x 0.30, y 0.20)
        rod.header.frame_id = "base_link"
        rod.position.x, rod.position.y = 0.30, 0.20
        ctx.objects.objects = [rod]
        self._ctx_pub.publish(ctx)

    def _speak(self, request: Speak.Request, response: Speak.Response) -> Speak.Response:
        self.get_logger().info(f"[TTS] {request.text}")
        response.done = True
        return response

    def _skill(self, goal_handle, kind, name):
        self.count += 1
        n = self.count
        g = goal_handle.request
        detail = {"handover": lambda: f"dir={g.direction} tilt={g.tilt_deg:+.0f}°",
                  "pick_compliant": lambda: f"xy=({g.target.pose.position.x:.2f}, {g.target.pose.position.y:.2f})",
                  "push_ball": lambda: f"lane={g.lane} speed={g.push_speed:.2f}",
                  "resist_push": lambda: f"target={g.target_force_n:.1f} N stiffness={g.stiffness:.0f}"}[name]()
        self.get_logger().info(f"{name} #{n} 시작 ({detail})")
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.duration_s:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f"{name} #{n} 취소됨")
                return self._result(kind, False, "canceled")
            if self.stop:
                goal_handle.abort()
                self.get_logger().info(f"{name} #{n} abort (STOP)")
                return self._result(kind, False, "safety_stop")
            time.sleep(0.05)
        ok = not (self.fail_every and n % self.fail_every == 0)
        # 실제 스킬처럼 실패는 abort (결과 메시지에 failure_reason)
        goal_handle.succeed() if ok else goal_handle.abort()
        self.get_logger().info(f"{name} #{n} {'성공' if ok else f'실패 ({self.fail_reason})'}")
        return self._result(kind, ok, "" if ok else self.fail_reason)

    def _result(self, kind, ok: bool, reason: str):
        if kind is Handover:
            return Handover.Result(success=ok, wrist_angle_at_grasp_deg=12.5, handover_time_s=self.duration_s,
                                   failure_reason=reason)
        if kind is PushBall:
            return PushBall.Result(success=ok, contact_force_peak=3.0, failure_reason=reason)   # 정지 위치는 아직 안 채움 (#42 와 같게)
        if kind is ResistPush:
            return ResistPush.Result(success=ok, peak_force_n=8.0 if ok else 4.0, hold_time_achieved_s=2.0 if ok else 0.5,
                                     force_smoothness=0.1, failure_reason=reason)
        return PickCompliant.Result(success=ok, grip_width_mm=20.0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=1.0)
    ap.add_argument("--fail-every", type=int, default=0)
    ap.add_argument("--fail-reason", default="timeout")
    args = ap.parse_args()
    rclpy.init()
    node = FakeSkills(args.duration, args.fail_every, args.fail_reason)
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
