"""armpi_vr_bridge 단독 실행 (개발·시험용). 전체 로봇 실행은 armpi_bringup 의 launch 를 쓴다.

  ros2 launch armpi_vr_bridge vr_bridge.launch.py [params_file:=/경로/vr_bridge.yaml]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("armpi_vr_bridge"), "config", "vr_bridge.yaml")
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="armpi_vr_bridge 매개변수 YAML"),
        Node(
            package="armpi_vr_bridge",
            executable="armpi_vr_bridge_node",
            name="armpi_vr_bridge",
            output="screen",
            parameters=[params_file],
            # 종료 시 SIGINT 를 받아 정지 명령 3회를 발행한 뒤 끝난다.
            sigterm_timeout="5",
        ),
    ])
