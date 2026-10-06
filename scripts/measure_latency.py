"""노드 간 지연 측정: header.stamp 대비 수신 시각 차이를 토픽별로 집계
담당: 조정묵

각 노드는 센서 원본의 header.stamp 를 그대로 넘기므로(CLAUDE.md), "수신 시각 - stamp" = 센서 촬영부터 그 토픽까지의 지연이다.
  /camera/color/image_raw    → 카메라 드라이버 지연
  /rally/perception/objects  → 카메라 → 인식 결과 (NFR-01 ≤ 60 ms)
  /rally/perception/landing  → 웹캠 → 착지 예측 (NFR-01 ≤ 40 ms, 모드 3 lead_time_s 근거)

시계: 발행한 PC 와 이 스크립트를 돌리는 PC 의 시계가 다르면 그 차이가 지연에 섞인다.
시간이 중요한 경로는 로봇·인식 PC 안에서 끝나므로 **로봇·인식 PC 에서 실행**한다. 지연이 음수로 나오면 시계 차이 경고.

사용 (ROS 환경 source 후, 로봇·인식 PC 는 scripts/env_robot_pc.sh 도):
  python3 scripts/measure_latency.py                          # 기본 토픽 10 s
  python3 scripts/measure_latency.py --duration 30 --csv data/latency/run1.csv
  python3 scripts/measure_latency.py --only /rally/perception/objects --only /camera/color/image_raw
  python3 scripts/measure_latency.py --topic /my/topic:geometry_msgs/msg/PointStamped:50   # 이름:타입[:목표 ms]
"""
import argparse
import csv
import os
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

# (토픽, 타입, NFR-01 목표 ms 또는 None) — docs/INTERFACES.md 2절, 기획서 NFR-01
DEFAULT_TOPICS = [
    ("/camera/color/image_raw", "sensor_msgs/msg/Image", None),
    ("/camera/aligned_depth_to_color/image_raw", "sensor_msgs/msg/Image", None),
    ("/webcam/image_raw", "sensor_msgs/msg/Image", None),
    ("/rally/perception/objects", "rally_interfaces/msg/TrackedObjectArray", 60.0),
    ("/rally/perception/landing", "geometry_msgs/msg/PointStamped", 40.0),
    ("/rally/perception/hand", "rally_interfaces/msg/HandState", None),
    ("/rally/perception/body", "rally_interfaces/msg/BodyPose", None),
    ("/rally/robot/wrench", "geometry_msgs/msg/WrenchStamped", None),
    ("/rally/safety/status", "rally_interfaces/msg/SafetyStatus", None),
    ("/rally/brain/context", "rally_interfaces/msg/SceneContext", None),
]
CLOCK_SKEW_WARN_MS = -1.0


class LatencyProbe(Node):
    def __init__(self, topics):
        super().__init__("rally_measure_latency")
        self.samples = {name: [] for name, _, _ in topics}   # (수신 시각 ns, stamp ns, 지연 ms)
        for name, msg_type, _ in topics:
            # 센서 QoS(BestEffort) 구독은 Reliable·BestEffort 발행 모두에서 받는다
            self.create_subscription(get_message(msg_type), name,
                                     lambda msg, n=name: self._on_msg(n, msg), qos_profile_sensor_data)

    def _on_msg(self, name, msg):
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns == 0:
            return   # stamp 를 안 채운 발행자 — 지연을 잴 수 없음 (요약에서 따로 표시)
        self.samples[name].append((now_ns, stamp_ns, (now_ns - stamp_ns) / 1e6))


def parse_topic(spec):
    parts = spec.split(":")
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError(f"'{spec}' — 이름:타입[:목표 ms] 형식")
    return parts[0], parts[1], float(parts[2]) if len(parts) == 3 else None


def percentile(values, q):
    s = sorted(values)
    return s[min(len(s) - 1, int(round(q * (len(s) - 1))))]


def summarize(topics, samples, duration):
    rows = []
    for name, _, target in topics:
        lat = [s[2] for s in samples[name]]
        if not lat:
            rows.append((name, "수신 없음", "", "", "", "", "", ""))
            continue
        hz = len(lat) / duration
        p95 = percentile(lat, 0.95)
        verdict = "" if target is None else (f"OK ≤{target:.0f}" if p95 <= target else f"초과 >{target:.0f}")
        rows.append((name, str(len(lat)), f"{hz:.1f}", f"{statistics.mean(lat):.1f}",
                     f"{statistics.median(lat):.1f}", f"{p95:.1f}", f"{max(lat):.1f}", verdict))
    header = ("topic", "n", "Hz", "mean ms", "p50 ms", "p95 ms", "max ms", "NFR-01 (p95)")
    widths = [max(len(str(r[i])) for r in [header, *rows]) for i in range(len(header))]
    lines = ["  ".join(str(c).ljust(w) for c, w in zip(r, widths)) for r in [header, *rows]]
    return "\n".join([lines[0], "-" * len(lines[0]), *lines[1:]])


def parse_args():
    """명령어 뒤 옵션 읽기 → (옵션, 잴 토픽 목록)"""
    parser = argparse.ArgumentParser(description="header.stamp 기준 토픽 지연 측정")
    parser.add_argument("--duration", type=float, default=10.0, help="측정 시간 [s]")
    parser.add_argument("--topic", type=parse_topic, action="append", default=[],
                        help="추가 토픽 이름:타입[:목표 ms] (여러 번 가능)")
    parser.add_argument("--only", action="append", default=[], help="기본 목록 중 이 토픽만 (여러 번 가능)")
    parser.add_argument("--csv", default=None, help="샘플별 CSV 저장 경로")
    opts = parser.parse_args()

    unknown = set(opts.only) - {t[0] for t in DEFAULT_TOPICS}
    if unknown:
        parser.error(f"--only 에 기본 목록에 없는 토픽: {', '.join(sorted(unknown))} (--topic 으로 추가)")
    topics = [t for t in DEFAULT_TOPICS if not opts.only or t[0] in opts.only] + opts.topic
    return opts, topics


def collect(node, duration):
    """duration 동안 메시지 받기. Ctrl+C 로 일찍 끝내면 실제로 잰 시간을 돌려준다"""
    end = time.monotonic() + duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        duration -= max(0.0, end - time.monotonic())
    return max(duration, 1e-3)


def warn_clock_skew(node):
    """지연이 음수인 토픽이 있으면 시계 차이 경고"""
    skewed = [n for n, s in node.samples.items() if s and min(x[2] for x in s) < CLOCK_SKEW_WARN_MS]
    if skewed:
        node.get_logger().warn(f"지연이 음수인 토픽: {', '.join(skewed)} — 발행 PC 와 시계가 다릅니다. "
                               "같은 PC 에서 재거나 두 PC 시계를 맞추세요 (chrony)")


def save_csv(path, samples):
    """샘플 하나당 한 줄: topic, 수신 시각, stamp, 지연 ms"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["topic", "recv_ns", "stamp_ns", "latency_ms"])
        for name, rows in samples.items():
            w.writerows((name, *r) for r in rows)
    return sum(len(s) for s in samples.values())


def main():
    opts, topics = parse_args()                       # 1. 옵션 읽기

    rclpy.init()
    node = LatencyProbe(topics)                       # 2. 토픽 구독
    node.get_logger().info(f"{len(topics)}개 토픽, {opts.duration:.0f} s 측정 시작")
    duration = collect(node, opts.duration)           # 3. 정한 시간 동안 측정

    print(summarize(topics, node.samples, duration))  # 4. 결과 표 + 시계 경고
    warn_clock_skew(node)
    if opts.csv:                                      # 5. CSV 저장 (선택)
        rows = save_csv(opts.csv, node.samples)
        node.get_logger().info(f"CSV 저장: {opts.csv} ({rows} 행)")

    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
