"""선생님 코드 기준선 한 번에: detection(/get_3d_position) + get_keyword(/get_keyword) + legacy_pick_and_place
PC 1에서 dsr_bringup2 와 realsense2_camera 가 먼저 떠 있어야 함. Day 1~2 확인용.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="rally_vision", executable="detection", output="screen"),
        Node(package="rally_voice", executable="get_keyword", output="screen"),
        Node(package="rally_robot", executable="legacy_pick_and_place", output="screen"),
    ])
