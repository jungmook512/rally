"""rally_brain/rally_brain/context_builder.py — 토픽 구독 → SceneContext 5 Hz 발행 (/rally/brain/context)
담당: 조정묵

구독                                 보관
  /rally/safety/status      Reliable    최신 1개 (stale_safety_s 넘게 안 오면 STOP 으로 간주, reason=no_safety_status)
  /rally/perception/hand    BestEffort  최신 1개 (stale_s 넘으면 빈 메시지 = detected False)
  /rally/perception/body    BestEffort  최신 1개 (stale_s 넘으면 빈 메시지)
  /rally/perception/objects BestEffort  최신 1개 (stale_s 넘으면 빈 배열)
  /rally/voice/intent       Reliable    마지막 1개를 intent_hold_s 동안만 싣는다 (같은 발화 = 같은 header.stamp 는 다시 와도 유지 시간을 늘리지 않음.
                                        5 Hz 로 같은 last_intent 가 여러 번 실리므로 받는 쪽은 header.stamp 로 한 번만 처리)
  /rally/brain/trial_result Reliable    최근 recent_n 개 (session_id 가 바뀌면 비움)
발행
  /rally/brain/context      BestEffort  rate_hz (기본 5 Hz)

신선도는 수신 시각(이 노드 시계) 기준이다. header.stamp 는 PC 간 시계 차가 있어 그대로 넘기기만 한다.
session_id · mode · trial_index 는 마지막 TrialResult 에서 가져온다 — bt_runner 가 생기면 그쪽 상태로 바꾼다.
recent_success_rate 는 마지막 시도와 같은 모드만으로 metrics.success_rate (metrics.yaml — 규칙 레이어와 같은 정의).
fatigue_score 는 같은 모드 최근 시도로 rule_layer.fatigue_signals (+ 음성 tired) → metrics.fatigue_score (0~1, 표시용).
  보상 동작은 시도 사이에 body.compensation_detected 가 한 번이라도 보였는지를 그 시도의 기록으로 남겨 센다 (bt_runner 와 같은 방식).
  보상 동작 반복은 모드를 가리지 않고 세션의 최근 시도로 센다 (10/15 조정묵: 몸 신호라 모드 무관). 성공률·시간은 같은 모드만.
  규칙 레이어는 난이도가 바뀐 뒤의 시도만 보므로 신호가 조금 다를 수 있다 — 휴식 결정은 규칙 레이어가 한다. robot_state 는 STOP 이면 STOPPED, 아니면 IDLE
(스킬 실행 상태 토픽이 아직 없다).
"""
from __future__ import annotations

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from rally_brain import metrics, rule_layer
from rally_brain.config import load_skills
from rally_interfaces.msg import (
    BodyPose,
    HandState,
    SafetyStatus,
    SceneContext,
    TrackedObjectArray,
    TrialResult,
    VoiceIntent,
)

NO_SAFETY_REASON = "no_safety_status"


def is_fresh(received_ns: int | None, now_ns: int, max_age_s: float) -> bool:
    """수신 기록이 있고 max_age_s 안이면 True."""
    return received_ns is not None and now_ns - received_ns <= max_age_s * 1e9


def same_utterance(a: VoiceIntent, b: VoiceIntent) -> bool:
    """header.stamp 가 같으면 같은 발화. stamp 가 비어 있으면(0) 판별하지 않는다."""
    sa, sb = a.header.stamp, b.header.stamp
    if sa.sec == 0 and sa.nanosec == 0:
        return False
    return (sa.sec, sa.nanosec) == (sb.sec, sb.nanosec)


def robot_state_for(safety_level: int) -> int:
    return SceneContext.ROBOT_STOPPED if safety_level == SafetyStatus.LEVEL_STOP else SceneContext.ROBOT_IDLE


class ContextBuilder(Node):
    def __init__(self):
        super().__init__("rally_context_builder")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("recent_n", 5)
        self.declare_parameter("stale_s", 1.0)            # 손·몸·물체
        # 안전 상태(100 Hz)가 이만큼 안 오면 STOP 으로 보고 세션을 끝낸다. 0.5 → 3.0 s (10/14): 와이파이로 0.8 s~1.5 s 넘는 끊김이 나
        # 세션이 끊김. 진짜 STOP 은 로봇 PC safety_monitor 가 스스로 즉시 멈추므로 brain 이 0.5 s 만에 반응할 필요가 없다
        self.declare_parameter("stale_safety_s", 3.0)
        self.declare_parameter("intent_hold_s", 3.0)
        p = self.get_parameter
        self.rate_hz = float(p("rate_hz").value)
        self.stale_s = float(p("stale_s").value)
        self.stale_safety_s = float(p("stale_safety_s").value)
        self.intent_hold_s = float(p("intent_hold_s").value)
        self._rule_cfg = rule_layer.rule_config(load_skills())

        # (메시지, 수신 시각 ns)
        self._safety: tuple[SafetyStatus, int] | None = None
        self._hand: tuple[HandState, int] | None = None
        self._body: tuple[BodyPose, int] | None = None
        self._objects: tuple[TrackedObjectArray, int] | None = None
        self._intent: tuple[VoiceIntent, int] | None = None
        self._trials: deque[TrialResult] = deque(maxlen=int(p("recent_n").value))
        self._trial_comp: deque[bool] = deque(maxlen=self._trials.maxlen)   # _trials 와 같은 순서, 시도별 보상 동작 여부
        self._comp_seen = False                                              # 지난 시도 뒤로 보상 동작이 보였나
        self._last_stale_warn: bool | None = None

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.create_subscription(SafetyStatus, "/rally/safety/status", self._keep("_safety"), reliable)
        self.create_subscription(HandState, "/rally/perception/hand", self._keep("_hand"), qos_profile_sensor_data)
        self.create_subscription(BodyPose, "/rally/perception/body", self._on_body, qos_profile_sensor_data)
        self.create_subscription(
            TrackedObjectArray, "/rally/perception/objects", self._keep("_objects"), qos_profile_sensor_data)
        self.create_subscription(VoiceIntent, "/rally/voice/intent", self._on_intent, reliable)
        self.create_subscription(TrialResult, "/rally/brain/trial_result", self._on_trial, reliable)
        self._pub = self.create_publisher(SceneContext, "/rally/brain/context", qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate_hz, self._publish)
        self.get_logger().info(
            f"rally_context_builder 시작 — {self.rate_hz:.0f} Hz, 최근 시도 {self._trials.maxlen}개, "
            f"stale {self.stale_s} s / 안전 {self.stale_safety_s} s")

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _keep(self, attr: str):
        def callback(msg) -> None:
            setattr(self, attr, (msg, self._now_ns()))
        return callback

    def _on_intent(self, msg: VoiceIntent) -> None:
        """같은 발화(header.stamp 가 같은 메시지)가 다시 오면 무시한다 — intent_hold_s 유지 시간을 다시 늘리지 않는다."""
        if self._intent is not None and same_utterance(self._intent[0], msg):
            return
        self._intent = (msg, self._now_ns())

    def _on_body(self, msg: BodyPose) -> None:
        self._body = (msg, self._now_ns())
        self._comp_seen = self._comp_seen or msg.compensation_detected

    def _on_trial(self, msg: TrialResult) -> None:
        if self._trials and self._trials[-1].session_id != msg.session_id:
            self._trials.clear()
            self._trial_comp.clear()
        self._trials.append(msg)
        self._trial_comp.append(self._comp_seen)
        self._comp_seen = False

    def _fresh(self, kept: tuple | None, max_age_s: float, now_ns: int):
        """신선하면 메시지, 아니면 None."""
        if kept is not None and is_fresh(kept[1], now_ns, max_age_s):
            return kept[0]
        return None

    def build(self) -> SceneContext:
        now_ns = self._now_ns()
        ctx = SceneContext()
        ctx.header.stamp = self.get_clock().now().to_msg()
        ctx.header.frame_id = "base_link"

        safety = self._fresh(self._safety, self.stale_safety_s, now_ns)
        if safety is None:
            safety = SafetyStatus(level=SafetyStatus.LEVEL_STOP, reason=NO_SAFETY_REASON, speed_override=0.0)
            safety.header.stamp = ctx.header.stamp
        ctx.safety = safety
        ctx.robot_state = robot_state_for(safety.level)
        ctx.hand = self._fresh(self._hand, self.stale_s, now_ns) or HandState()
        ctx.body = self._fresh(self._body, self.stale_s, now_ns) or BodyPose()
        ctx.objects = self._fresh(self._objects, self.stale_s, now_ns) or TrackedObjectArray()
        ctx.last_intent = self._fresh(self._intent, self.intent_hold_s, now_ns) or VoiceIntent()

        ctx.recent_trials = list(self._trials)
        same_mode = [t for t in self._trials if t.mode == self._trials[-1].mode] if self._trials else []
        ctx.recent_success_rate = metrics.success_rate(same_mode)
        if self._trials:
            last = self._trials[-1]
            ctx.session_id = last.session_id
            ctx.mode = last.mode
            ctx.trial_index = last.trial_index
        comp = list(self._trial_comp)                                   # 모드 무관 — 세션의 시도 순서대로 (10/15)
        signals = rule_layer.fatigue_signals(metrics.counted(same_mode), self._rule_cfg, compensation_history=comp)
        if ctx.last_intent.intent == "tired":
            signals.append("voice_tired")
        ctx.fatigue_score = metrics.fatigue_score(signals)
        return ctx

    def _publish(self) -> None:
        ctx = self.build()
        no_safety = ctx.safety.reason == NO_SAFETY_REASON
        if no_safety != self._last_stale_warn:   # 바뀔 때만 한 줄
            if no_safety:
                self.get_logger().warn(f"/rally/safety/status 가 {self.stale_safety_s} s 넘게 안 옴 — STOP 으로 간주")
            elif self._last_stale_warn is not None:
                self.get_logger().info("/rally/safety/status 수신 재개")
            self._last_stale_warn = no_safety
        self._pub.publish(ctx)


def main(args=None):
    rclpy.init(args=args)
    node = ContextBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
