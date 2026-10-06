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


def main():
    parser = argparse.ArgumentParser(description="header.stamp 기준 토픽 지연 측정")
    parser.add_argument("--duration", type=float, default=10.0, help="측정 시간 [s]")
    parser.add_argument("--topic", type=parse_topic, action="append", default=[],
                        help="추가 토픽 이름:타입[:목표 ms] (여러 번 가능)")
    parser.add_argument("--only", action="append", default=[], help="기본 목록 중 이 토픽만 (여러 번 가능)")
    parser.add_argument("--csv", default=None, help="샘플별 CSV 저장 경로")
    opts = parser.parse_args()

    topics = [t for t in DEFAULT_TOPICS if not opts.only or t[0] in opts.only] + opts.topic
    unknown = set(opts.only) - {t[0] for t in DEFAULT_TOPICS}
    if unknown:
        parser.error(f"--only 에 기본 목록에 없는 토픽: {', '.join(sorted(unknown))} (--topic 으로 추가)")

    rclpy.init()
    node = LatencyProbe(topics)
    log = node.get_logger()
    log.info(f"{len(topics)}개 토픽, {opts.duration:.0f} s 측정 시작")
    end = time.monotonic() + opts.duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        opts.duration -= max(0.0, end - time.monotonic())

    print(summarize(topics, node.samples, max(opts.duration, 1e-3)))
    skewed = [n for n, s in node.samples.items() if s and min(x[2] for x in s) < CLOCK_SKEW_WARN_MS]
    if skewed:
        log.warn(f"지연이 음수인 토픽: {', '.join(skewed)} — 발행 PC 와 시계가 다릅니다. "
                 "같은 PC 에서 재거나 두 PC 시계를 맞추세요 (chrony)")

    if opts.csv:
        os.makedirs(os.path.dirname(os.path.abspath(opts.csv)), exist_ok=True)
        with open(opts.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["topic", "recv_ns", "stamp_ns", "latency_ms"])
            for name, rows in node.samples.items():
                w.writerows((name, *r) for r in rows)
        log.info(f"CSV 저장: {opts.csv} ({sum(len(s) for s in node.samples.values())} 행)")

    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
