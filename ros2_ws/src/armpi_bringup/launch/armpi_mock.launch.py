"""ArmPi Pro 가짜 하드웨어 실행(Mac/Docker, 카메라 없음): 제어기(mock) + UDP 수신기 + rosbridge.

  ros2 launch armpi_bringup armpi_mock.launch.py [rosbridge:=false] [params_file:=...]

실제 바퀴·서보·GPIO 를 쓰지 않는다. 하드웨어 프로필은 시험용 값(config/test_hardware_profile.yaml)을 쓴다. 가짜 서보는 명령 위치로 시간에 맞춰 선형 이동하고, 가짜 배터리는 7.4 V.
가짜 장치 상태: /armpi_control/hw_state, 가짜 입력: /armpi_control/mock_cmd ("gpio <핀> 0" 등, README 참고).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("armpi_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "armpi.yaml")),
        DeclareLaunchArgument("hardware_profile", default_value=os.path.join(share, "config", "test_hardware_profile.yaml")),
        DeclareLaunchArgument("vr_bridge", default_value="true"),
        DeclareLaunchArgument("rosbridge", default_value="true"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "armpi.launch.py")),
            launch_arguments={
                "params_file": LaunchConfiguration("params_file"),
                "mock": "true",
                "hardware_profile": LaunchConfiguration("hardware_profile"),
                "vr_bridge": LaunchConfiguration("vr_bridge"),
                "camera": "false",
                "web_video": "false",
                "rosbridge": LaunchConfiguration("rosbridge"),
                "rosbridge_port": LaunchConfiguration("rosbridge_port"),
            }.items(),
        ),
    ])
