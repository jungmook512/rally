"""rally_db/rally_db/recorder_node.py — TrialResult·시계열 → PostgreSQL/Timescale, 이벤트 → Redis
담당: 조정묵
TODO(Day 2): TrialResult → trials, WrenchStamped → ts_wrench, HandState → ts_hand, 이벤트 → events, Redis rally:state · rally:events
"""
import rclpy
from rclpy.node import Node


class Recorder(Node):
    def __init__(self):
        super().__init__("rally_recorder")
        self.get_logger().info("rally_recorder 골격 — 아직 하는 일 없음")


def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
