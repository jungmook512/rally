"""rally_bringup/launch/pc2_perception.launch.py — 로봇·인식 PC 추론 컨테이너 (역할 pc2)

compose 서비스마다 part 를 하나씩 준다:
  vision    → realsense2_camera(/camera/color/image_raw, 정렬 깊이) + camera_tf · segmenter · tracker · flight_predictor
  human     → usb_cam(/webcam/image_raw) + hand_pose · body_pose
  voice     → intent_node · tts_node
  all       → 셋 다 (컨테이너 없이 한 터미널에서 확인할 때)

아직 실행 등록이 안 된 노드나 이 환경에 없는 패키지는 경고만 남기고 건너뛴다 — 담당자가 노드를 채우는 동안에도 launch 는 돈다.
캘리브레이션(calibration 노드)은 필요할 때 따로: ros2 run rally_vision calibration
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARTS = {
    "vision": [("rally_vision", "camera_tf"),                 # hand_eye.yaml → base_link→camera_link tf (손 노드·segmenter 가 조회)
               ("rally_vision", "segmenter"), ("rally_vision", "tracker"), ("rally_vision", "flight_predictor")],
    "human": [("rally_human", "hand_pose"), ("rally_human", "body_pose")],
    "voice": [("rally_voice", "intent_node"), ("rally_voice", "tts_node")],
}


def _has_executable(package: str, executable: str) -> bool:
    try:
        return os.path.isfile(os.path.join(get_package_prefix(package), "lib", package, executable))
    except PackageNotFoundError:
        return False


def _has_package(package: str) -> bool:
    try:
        get_package_prefix(package)
        return True
    except PackageNotFoundError:
        return False


def _nodes(package: str, executables: list[str]):
    actions = []
    for exe in executables:
        if _has_executable(package, exe):
            actions.append(Node(package=package, executable=exe, output="screen"))
        else:
            actions.append(LogInfo(msg=f"[pc2] {package}/{exe} 실행 등록 없음 → 건너뜀 (setup.py console_scripts 확인)"))
    return actions


def _realsense(context):
    if not _has_package("realsense2_camera"):
        return [LogInfo(msg="[pc2] realsense2_camera 패키지 없음 → 카메라 건너뜀 (vision 컨테이너에서 실행하세요)")]
    rs_launch = os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(rs_launch), launch_arguments={
        # 토픽을 /camera/color/image_raw 로 (선생님 코드·INTERFACES 2.1). 드라이버 기본값은 /camera/camera/...
        "camera_namespace": "",
        "camera_name": "camera",
        "align_depth.enable": "true",
    }.items())]


def _webcam(context):
    if not _has_package("usb_cam"):
        return [LogInfo(msg="[pc2] usb_cam 패키지 없음 → 측면 웹캠 건너뜀 (hand_pose 컨테이너에서 실행하세요)")]
    return [Node(package="usb_cam", executable="usb_cam_node_exe", name="rally_webcam", output="screen",
                 parameters=[{
                     "video_device": LaunchConfiguration("webcam_device").perform(context),
                     "framerate": 60.0,                       # 모드 3 비행 추적 60 Hz (INTERFACES 2.1)
                     "image_width": 640, "image_height": 480,
                     "pixel_format": "mjpeg2rgb",             # 60 fps 는 대부분 MJPEG 에서만 나온다
                     "frame_id": "webcam_link",
                 }],
                 remappings=[("image_raw", "/webcam/image_raw"), ("camera_info", "/webcam/camera_info")])]


def _launch_part(context):
    part = LaunchConfiguration("part").perform(context)
    parts = list(PARTS) if part == "all" else [part]
    if any(p not in PARTS for p in parts):
        raise ValueError(f"part:={part} — vision | human | voice | all 중 하나")
    actions = [LogInfo(msg=f"[pc2] part={part}")]
    for p in parts:
        if p == "vision":
            actions += _realsense(context)
        if p == "human":
            actions += _webcam(context)
        for package in {pkg for pkg, _ in PARTS[p]}:
            actions += _nodes(package, [exe for pkg, exe in PARTS[p] if pkg == package])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("part", default_value="all", description="vision | human | voice | all"),
        DeclareLaunchArgument("webcam_device", default_value="/dev/video0", description="측면 웹캠 장치"),
        OpaqueFunction(function=_launch_part),
    ])
