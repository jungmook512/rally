"""rally_vision/segmenter_node.py — YOLO-Seg → TrackedObjectArray (/rally/perception/objects, 30 Hz)
담당: 박성현
선생님 detection.py 와의 차이
- 서비스 요청 시 1회가 아니라 토픽으로 계속 발행
- 박스 중심 대신 **마스크 픽셀 깊이 중앙값**으로 3D 중심 (배경 깊이 오염 제거)
- 마스크 PCA 주축 → yaw, mask_area
- camera → base_link 변환은 tf2 (camera_link static tf = T_gripper2camera.npy 를 rally_bringup 이 발행)
  ※ 초기엔 선생님 방식(get_current_posx + npy 직접 곱)으로 해도 됨 — rally_robot.dsr_helpers.transform_to_base
재사용: rally_vision.realsense.ImgNode (구독), rally_vision.yolo.YoloModel._aggregate_detections (프레임 집계 아이디어)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rally_interfaces.msg import TrackedObject, TrackedObjectArray


class SegmenterNode(Node):
    def __init__(self):
        super().__init__("rally_segmenter")
        self.declare_parameter("model_path", "")          # resource/yolov8n-seg_rally.pt (TensorRT .engine 권장)
        self.declare_parameter("conf", 0.5)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(TrackedObjectArray, "/rally/perception/objects", qos)
        # TODO(박성현): ImgNode 구독 → ultralytics seg 추론 → 마스크별 3D 중심·yaw → 발행


def main(args=None):
    rclpy.init(args=args); n = SegmenterNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
