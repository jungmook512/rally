"""rally_bringup/wait_ready.py — readiness.yaml 그룹이 준비될 때까지 기다린다. 준비되면 0, 시간 초과면 1 로 끝난다.

launch 단계 관문:   ros2 run rally_bringup wait_ready --group safety --timeout 10
시연 전 점검(1회):  ros2 run rally_bringup wait_ready --group session --once
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from rally_bringup.readiness import ReadinessChecker

REPORT_EVERY_S = 2.0


def main(args=None):
    parser = argparse.ArgumentParser(description="readiness.yaml 그룹 준비 대기")
    parser.add_argument("--group", action="append", required=True, help="확인할 그룹 (여러 번 가능)")
    parser.add_argument("--timeout", type=float, default=30.0, help="최대 대기 시간 [s]")
    parser.add_argument("--once", action="store_true", help="기다리지 않고 짧게 한 번 확인 (토픽 수신 대기 2 s)")
    parser.add_argument("--config", default=None, help="readiness.yaml 경로 (기본: rally_bringup share)")
    opts = parser.parse_args(rclpy.utilities.remove_ros_args(args if args is not None else sys.argv)[1:])

    rclpy.init(args=args)
    node = Node("rally_wait_ready")
    log = node.get_logger()
    checker = ReadinessChecker(node, opts.group, opts.config)
    groups = ", ".join(opts.group)
    timeout = 2.0 if opts.once else opts.timeout

    start = time.monotonic()
    last_report = start
    missing = checker.missing()
    try:
        while missing and time.monotonic() - start < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            missing = checker.missing()
            if not opts.once and time.monotonic() - last_report >= REPORT_EVERY_S:
                log.info(f"[{groups}] 대기 중 ({time.monotonic() - start:.0f}/{timeout:.0f} s): {'; '.join(missing)}")
                last_report = time.monotonic()
    except KeyboardInterrupt:
        pass

    if missing:
        log.error(f"[{groups}] 준비 안 됨 ({timeout:.0f} s): {'; '.join(missing)}")
    else:
        log.info(f"[{groups}] 준비 완료 ({time.monotonic() - start:.1f} s)")
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(1 if missing else 0)


if __name__ == "__main__":
    main()
