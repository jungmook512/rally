"""rally_bringup/launch/pc1_robot.launch.py — 로봇·인식 PC 네이티브 로봇 노드 (역할 pc1)

먼저: source scripts/env_robot_pc.sh → dsr_bringup2 (~/cobot2_ws) 기동 → 이 launch.
순서 (docs/INTERFACES.md 5절):
  1. safety_monitor (safety.yaml)
  2. wait_ready --group safety : /rally/safety/status 가 실제로 발행되는지 확인
     성공 → 3 / 실패 → 로봇을 움직이는 노드를 켜지 않고 launch 종료
  3. force_publisher, gripper_node, skill_server
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory("rally_bringup"), "config")
    safety_wait_s = LaunchConfiguration("safety_wait_s")

    safety_monitor = Node(package="rally_robot", executable="safety_monitor", output="screen",
                          parameters=[os.path.join(config, "safety.yaml"),
                                      {"hand_stop_mode": ParameterValue(LaunchConfiguration("hand_stop"), value_type=str)}])
    wait_safety = Node(package="rally_bringup", executable="wait_ready", output="screen",
                       arguments=["--group", "safety", "--timeout", safety_wait_s])
    robot_nodes = [
        Node(package="rally_robot", executable="force_publisher", output="screen"),
        Node(package="rally_robot", executable="gripper_node", output="screen"),   # IP 는 dsr_helpers.TOOLCHARGER_IP
        Node(package="rally_robot", executable="skill_server", output="screen",
             parameters=[os.path.join(config, "safety.yaml")]),   # status_timeout_s (#14)
    ]

    def after_safety(event, _context):
        if event.returncode == 0:
            return [LogInfo(msg="[pc1] 안전 감시 확인 → 로봇 노드 기동"), *robot_nodes]
        return [LogInfo(msg="[pc1] 안전 감시 발행 확인 실패 → 로봇 노드를 켜지 않고 종료. "
                            "safety_monitor 로그와 env_robot_pc.sh(DDS 설정)를 확인하세요"),
                EmitEvent(event=Shutdown(reason="safety_monitor not ready"))]

    return LaunchDescription([
        DeclareLaunchArgument("hand_stop", default_value="slow",
                              description="손 근접 STOP: slow(기본, 10/12~ — SLOW 로 낮춤) | stop | off(개발 시험 전용, 시연 금지). "
                                          "리허설 결과로 slow/stop 확정 (#145)"),
        DeclareLaunchArgument("safety_wait_s", default_value="10.0",
                              description="safety_monitor 발행을 기다리는 최대 시간 [s]"),
        safety_monitor,
        wait_safety,
        RegisterEventHandler(OnProcessExit(target_action=wait_safety, on_exit=after_safety)),
    ])
