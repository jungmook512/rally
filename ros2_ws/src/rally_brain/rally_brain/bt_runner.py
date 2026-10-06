"""rally_brain/rally_brain/bt_runner.py — py_trees 세션 트리 실행 (모드 1 → (3) → 5 → 7)
담당: 조정묵

지금(Day 1): 세션 시작 조건만 — readiness.yaml 의 session 그룹(안전·로봇·인식·음성)을 1 s 마다 검사해
준비 상태가 바뀔 때 로그를 남긴다. 준비 안 되면 세션을 시작하지 않는다 (NFR-08).
TODO(Day 2): 세션 트리 골격, action client 래퍼, TrialResult 발행 / 도중에 준비가 깨지면 세션 중단 + 관측 자세 대기
"""
import rclpy
from rclpy.node import Node

from rally_bringup.readiness import ReadinessChecker

CHECK_PERIOD_S = 1.0


class BtRunner(Node):
    def __init__(self):
        super().__init__("rally_bt_runner")
        self.readiness = ReadinessChecker(self, ["session"])
        self.session_ready = False
        self._last_missing: list[str] | None = None
        self.create_timer(CHECK_PERIOD_S, self._check_ready)
        self.get_logger().info("bt_runner 대기 — 세션 시작 조건(session 그룹) 확인 중")

    def _check_ready(self):
        missing = self.readiness.missing()
        self.session_ready = not missing
        if missing != self._last_missing:
            if missing:
                self.get_logger().warn(f"세션 시작 불가 — 준비 안 됨: {'; '.join(missing)}")
            else:
                self.get_logger().info("세션 시작 조건 충족")
            self._last_missing = missing


def main(args=None):
    rclpy.init(args=args)
    node = BtRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
