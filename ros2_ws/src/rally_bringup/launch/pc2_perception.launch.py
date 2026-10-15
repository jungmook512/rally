"""rally_bringup/launch/pc2_perception.launch.py — 로봇·인식 PC 추론 컨테이너 (역할 pc2)

compose 서비스마다 part 를 하나씩 준다:
  vision    → RealSense 두 대 (config/cameras.yaml, 둘 다 848×480 @60 fps, 시리얼로 구분)
                손목 /camera/color/image_raw + 정렬 깊이 — 노출 평소 자동, 모드 3 동안만 고정 (brain 이 바꿈)
                측면 /side/side/color/image_raw (컬러만) — 노출 항상 고정 (set_camera_params 가 뜬 뒤 넣음). side_camera:=false 면 안 띄움
              + camera_tf · segmenter · tracker · flight_predictor
              + web_video_server (포트 8080) — 대시보드 '카메라' 칸이 MJPEG 로 받아 본다 (video_server:=false 면 안 띄움)
  human     → usb_cam(/webcam/image_raw) + hand_pose · body_pose
  voice     → intent_node · tts_node (10/14 부터 서버 PC compose pc3 에서 띄움 — 파일 이름은 역할 이름 그대로)
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

from rally_bringup import camera_params

PARTS = {
    "vision": [("rally_vision", "camera_tf"),                 # hand_eye.yaml → base_link→camera_link tf (손 노드·segmenter 가 조회)
               ("rally_vision", "segmenter"), ("rally_vision", "tracker"), ("rally_vision", "flight_predictor")],
    "human": [("rally_human", "hand_pose"), ("rally_human", "body_pose")],
    "voice": [("rally_voice", "intent_node"), ("rally_voice", "tts_node")],
}

# 노드별 파라미터 (launch 인자에서). hand_pose 기본 토픽은 RealSense 를 직접 띄울 때의 /camera/camera/... 라서,
# 이 launch 의 RealSense(camera_namespace "", INTERFACES 2.1 의 /camera/color/...) 에 맞춰 넘긴다.
def _params(context, executable: str) -> list[dict]:
    if executable == "hand_pose":
        return [{"color_topic": "/camera/color/image_raw",
                 "depth_topic": "/camera/aligned_depth_to_color/image_raw",
                 "info_topic": "/camera/color/camera_info"}]
    if executable == "body_pose":
        return [{"arm": LaunchConfiguration("body_arm").perform(context)}]   # 시연은 왼팔 (10/8 결정, 웹캠이 환자 왼쪽)
    if executable == "segmenter":
        # 10/14: compose 경로도 scripts/vision_local.sh 의 HAND_EYE 와 같은 선택을 할 수 있게. none = hand_eye.yaml 안 쓰고 선생님 npy
        # (#66 과 같은 좌표 — mode1.vision_offset_m 이 10/7 그 좌표 기준으로 맞춰져 있어, yaml 과 겹치면 약 1 cm 어긋남)
        he = LaunchConfiguration("segmenter_hand_eye").perform(context).strip()
        params = {"hand_eye_path": he} if he else {}
        conf = LaunchConfiguration("segmenter_conf").perform(context).strip()
        if conf:                                   # 비우면 segmenter 기본값(0.5)
            params["conf"] = float(conf)
        models = LaunchConfiguration("segmenter_models").perform(context).strip()
        if models:                                 # 쉼표 목록, /models 기준 이름 (앞이 겹칠 때 우선). 비우면 segmenter 기본 찾기(공 + 컵·막대)
            params["model_path"] = ",".join(m if m.startswith("/") else f"/models/{m}" for m in (x.strip() for x in models.split(",")) if m)
        return [params] if params else []
    return []


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


def _nodes(context, package: str, executables: list[str]):
    actions = []
    for exe in executables:
        if _has_executable(package, exe):
            actions.append(Node(package=package, executable=exe, output="screen", parameters=_params(context, exe)))
        else:
            actions.append(LogInfo(msg=f"[pc2] {package}/{exe} 실행 등록 없음 → 건너뜀 (setup.py console_scripts 확인)"))
    return actions


def _realsense(context):
    if not _has_package("realsense2_camera"):
        return [LogInfo(msg="[pc2] realsense2_camera 패키지 없음 → 카메라 건너뜀 (vision 컨테이너에서 실행하세요)")]
    rs_launch = os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")
    cfg = camera_params.load()

    def include(cam: dict, depth: bool):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(rs_launch),
                                        launch_arguments=camera_params.rs_launch_args(cam, depth).items())

    # 손목: 토픽 /camera/color/image_raw (선생님 코드·INTERFACES 2.1). 두 대가 꽂혀 있으므로 시리얼 필수 — 없으면 아무 쪽이나 열린다
    actions = [include(cfg["wrist"], depth=True)]
    side = cfg.get("side") or {}
    if side.get("enabled", True) and LaunchConfiguration("side_camera").perform(context).lower() in ("true", "1"):
        actions += [include(side, depth=False),
                    Node(package="rally_bringup", executable="set_camera_params", output="screen",
                         arguments=["--node", side["node"], "--fixed"])]
    else:
        actions.append(LogInfo(msg="[pc2] 측면 RealSense 끔 (side_camera:=false 또는 cameras.yaml side.enabled) — 모드 3 예측 안 됨"))
    return actions


def _video_server(context):
    """대시보드 카메라 화면용. 영상은 이 PC 안에서 압축돼 브라우저로만 간다 (DDS 로 원본을 서버 PC 에 보내지 않음, 저장 안 함)."""
    if LaunchConfiguration("video_server").perform(context).lower() not in ("true", "1"):
        return []
    if not _has_package("web_video_server"):
        return [LogInfo(msg="[pc2] web_video_server 패키지 없음 → 대시보드 카메라 화면 없음 (vision 컨테이너 다시 빌드)")]
    return [Node(package="web_video_server", executable="web_video_server", name="rally_web_video", output="screen",
                 parameters=[{"port": 8080, "address": "0.0.0.0", "server_threads": 2, "ros_threads": 2}])]


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
            actions += _realsense(context) + _video_server(context)
        if p == "human":
            actions += _webcam(context)
        for package in {pkg for pkg, _ in PARTS[p]}:
            actions += _nodes(context, package, [exe for pkg, exe in PARTS[p] if pkg == package])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("part", default_value="all", description="vision | human | voice | all"),
        DeclareLaunchArgument("side_camera", default_value="true", description="측면 RealSense (모드 3) 띄우기"),
        DeclareLaunchArgument("video_server", default_value="true", description="대시보드 카메라 화면용 web_video_server (포트 8080)"),
        DeclareLaunchArgument("webcam_device", default_value="/dev/video0", description="측면 웹캠 장치"),
        DeclareLaunchArgument("body_arm", default_value="left", description="body_pose 뻗는 팔: left | right | auto (시연은 왼팔)"),
        DeclareLaunchArgument("segmenter_hand_eye", default_value="",
                              description="segmenter hand_eye_path: 비우면 hand_eye.yaml, none = 옛 T_gripper2camera.npy (#66 좌표, vision_offset_m 기준)"),
        DeclareLaunchArgument("segmenter_conf", default_value="",
                              description="segmenter 검출 신뢰도 하한: 비우면 0.5. 모드 5 받기(손에 든 막대 0.25~0.5)는 0.25 — 테이블 집기는 brain 이 0.5 로 다시 거름"),
        DeclareLaunchArgument("segmenter_models", default_value="",
                              description="segmenter 모델 쉼표 목록 (/models 안 이름, 앞이 우선). 비우면 공 + 컵·막대. "
                                          "모드 3 벨크로 공은 velcro_ball_yolo11n-seg_1024_rot_v2.pt (10/15)"),
        OpaqueFunction(function=_launch_part),
    ])
