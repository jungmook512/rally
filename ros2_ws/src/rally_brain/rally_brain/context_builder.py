"""rally_brain/rally_brain/context_builder.py — 토픽 구독 → SceneContext 5 Hz 발행 (/rally/brain/context)
담당: 조정묵
TODO(Day 2): 손·몸·물체·음성·안전·최근 시도 구독 → SceneContext. 가짜 발행기로 먼저 테스트
"""
import rclpy
from rclpy.node import Node


class ContextBuilder(Node):
    def __init__(self):
        super().__init__("rally_context_builder")
        self.get_logger().info("rally_context_builder 골격 — 아직 하는 일 없음")


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
