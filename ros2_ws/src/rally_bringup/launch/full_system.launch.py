"""rally_bringup/launch/full_system.launch.py — 전체 기동 순서 안내 + 준비 상태 1회 점검

물리 PC 2대라 한 launch 로 전부 띄울 수 없다. 아래 순서로 각 PC 에서 띄운 뒤 이 launch 로 점검한다.
  0. 서버 PC      cd docker && docker compose --profile pc3 up -d discovery db redis
  1. 로봇·인식 PC source scripts/env_robot_pc.sh → dsr_bringup2 → ros2 launch rally_bringup pc1_robot.launch.py
  2. 로봇·인식 PC cd docker && docker compose --profile pc2 up -d
  3. 서버 PC      cd docker && docker compose --profile pc3 up -d brain dashboard

점검: ros2 launch rally_bringup full_system.launch.py   (session 그룹, 시연 직전에도 이걸로)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ORDER = (
    "[full_system] 기동 순서 — "
    "0) 서버 PC: compose pc3 discovery·db·redis → "
    "1) 로봇·인식 PC: env_robot_pc.sh → dsr_bringup2 → pc1_robot.launch → "
    "2) 로봇·인식 PC: compose pc2 → "
    "3) 서버 PC: compose pc3 brain·dashboard"
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("group", default_value="session", description="readiness.yaml 점검 그룹"),
        LogInfo(msg=ORDER),
        Node(package="rally_bringup", executable="wait_ready", output="screen",
             arguments=["--group", LaunchConfiguration("group"), "--once"]),
    ])
