"""rally_bringup/launch/pc3_brain.launch.py — 서버 PC brain 컨테이너 (역할 pc3)

compose: discovery · db · redis (healthcheck) 가 먼저 뜬 뒤 brain 컨테이너가 이 launch 를 실행한다.
  recorder         TrialResult·시계열 → DB, 이벤트 → Redis
  context_builder  SceneContext 5 Hz
  llm_advisor      SuggestParams 서버 (config/llm.yaml)
  bt_runner        세션 트리. readiness.yaml session 그룹이 준비돼야 세션 시작 (NFR-08)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("rally_bringup"), "config")
    return LaunchDescription([
        Node(package="rally_db", executable="recorder", output="screen"),
        Node(package="rally_brain", executable="context_builder", output="screen"),
        Node(package="rally_brain", executable="llm_advisor", output="screen",
             parameters=[os.path.join(config, "llm.yaml")]),
        Node(package="rally_brain", executable="bt_runner", output="screen"),
    ])
