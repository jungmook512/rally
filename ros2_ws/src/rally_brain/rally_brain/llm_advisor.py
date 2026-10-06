"""rally_brain/rally_brain/llm_advisor.py — SuggestParams 서버 (/rally/llm/suggest_params). 파라미터는 config/llm.yaml
담당: 조정묵
TODO(Day 4): 비동기 호출, 3 s 타임아웃, pydantic 스키마 검증 → skills.yaml 범위 클램핑 → 실패 시 규칙 기본값 (네 단계 필수)
"""
import rclpy
from rclpy.node import Node


class LlmAdvisor(Node):
    def __init__(self):
        super().__init__("rally_llm_advisor")
        self.get_logger().info("rally_llm_advisor 골격 — 아직 하는 일 없음")


def main(args=None):
    rclpy.init(args=args)
    node = LlmAdvisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
