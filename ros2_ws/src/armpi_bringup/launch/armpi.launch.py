"""ArmPi Pro 실기 실행: 제어기 + UDP 수신기 + 카메라 + MJPEG 스트림 + rosbridge.

  ros2 launch armpi_bringup armpi.launch.py
  ros2 launch armpi_bringup armpi.launch.py mock:=true camera:=false      # 가짜 하드웨어

인자
  params_file   매개변수 YAML (기본: armpi_bringup/config/armpi.yaml)
  hardware_profile 하드웨어 프로필 YAML. 비우면 mock:=true 는 config/test_hardware_profile.yaml(시험용 값),
                실제 하드웨어는 config/hardware_profile.yaml(없으면 제어 노드가 fault 로 시작하고 장치를 열지 않음)
  mock          true 면 가짜 하드웨어(바퀴·서보·GPIO 를 쓰지 않음)
  vr_bridge     UDP 수신기(5001/5002) 실행 여부
  camera        usb_cam 실행 여부, video_device / pixel_format / image_width / image_height / framerate
  camera_brightness     카메라 밝기(v4l2, 이 카메라 범위 −128..127, 카메라 기본 0). usb_cam 0.8 은 따로 주지 않으면 50 으로
                        올려 화면이 하얗게 뜬다(2026-11-04 실기). −1 이면 usb_cam 에 맡김
  camera_power_line_hz  전원 주파수 깜빡임 보정: 60(한국, 기본) / 50 / disabled / 빈 값(건드리지 않음).
                        usb_cam 에 이 매개변수가 없어 시작할 때 v4l2-ctl 로 한 번 적용한다(카메라를 다시 꽂으면
                        udev 규칙 99-armpi-hw.rules 가 60 Hz 로 다시 맞춘다)
  web_video     web_video_server(HTTP 8080) 실행 여부, web_video_port
  rosbridge     rosbridge_websocket 실행 여부, rosbridge_port
  dds_profile   Fast DDS XML(FASTRTPS_DEFAULT_PROFILES_FILE). 기본 config/fastdds_armpi.xml(대용량 영상용 버퍼·공유 메모리).
                "" 이면 Fast DDS 기본값
설치되지 않은 패키지(usb_cam, web_video_server, rosbridge_server, armpi_vr_bridge)는 경고만 남기고 건너뛴다.
카메라 스트림: http://<IP>:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed
"""

import os
import shutil

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable, Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _have(pkg):
    try:
        return get_package_share_directory(pkg)
    except PackageNotFoundError:
        return None


def _profile_files(context):
    """하드웨어 프로필 파일 목록(0개 또는 1개). armpi_hardware/hardware_profile.hpp 참고."""
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    path = lc("hardware_profile")
    if not path:
        share = get_package_share_directory("armpi_bringup")
        mock = lc("mock").lower() in ("1", "true", "yes", "on")
        path = os.path.join(share, "config", "test_hardware_profile.yaml" if mock else "hardware_profile.yaml")
    return [path] if os.path.isfile(path) else []


def _control(context, *args, **kwargs):
    profile = _profile_files(context)
    msg = ("[armpi_bringup] 하드웨어 프로필: " + profile[0]) if profile else \
        "[armpi_bringup] 경고: 하드웨어 프로필 파일 없음 → 실제 하드웨어 모드는 fault 로 시작(장치 열지 않음)"
    return [
        LogInfo(msg=msg),
        Node(
            package="armpi_control", executable="armpi_control_node", name="armpi_control", output="screen",
            parameters=[LaunchConfiguration("params_file"), *profile,
                        {"hardware.mock": ParameterValue(LaunchConfiguration("mock"), value_type=bool)}],
            sigterm_timeout="5",  # 종료 시 바퀴 0 (서보 토크는 유지)
            # 제어 노드가 죽으면 launch 전체를 끝낸다 -> systemd armpi.service 의 ExecStopPost 가 모터 보드에 바퀴 0 을 쓰고
            # Restart=always 로 다시 띄운다(제어기 없이 수신기만 살아 있는 상태를 만들지 않음).
            on_exit=Shutdown(reason="armpi_control 종료"),
        ),
    ]


def _optional(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    on = lambda n: lc(n).lower() in ("1", "true", "yes", "on")  # noqa: E731
    params_file = lc("params_file")
    profile = _profile_files(context)
    actions = []

    if on("vr_bridge"):
        if _have("armpi_vr_bridge"):
            actions.append(Node(
                package="armpi_vr_bridge", executable="armpi_vr_bridge_node", name="armpi_vr_bridge",
                output="screen", parameters=[params_file, *profile],
                sigterm_timeout="5",  # SIGINT 를 받으면 정지 명령 3회 발행 뒤 끝남
            ))
        else:
            actions.append(LogInfo(msg="[armpi_bringup] 경고: armpi_vr_bridge 패키지 없음 → UDP 수신기 건너뜀"))

    if on("camera"):
        if _have("usb_cam"):
            actions.extend(_camera_power_line(lc("video_device"), lc("camera_power_line_hz")))
            actions.append(Node(
                package="usb_cam", executable="usb_cam_node_exe", name="usb_cam", namespace="usb_cam",
                output="screen", respawn=True, respawn_delay=3.0,
                parameters=[{
                    "video_device": lc("video_device"),
                    "image_width": int(lc("image_width")),
                    "image_height": int(lc("image_height")),
                    "framerate": float(lc("framerate")),
                    "pixel_format": lc("pixel_format"),
                    "io_method": "mmap",
                    "frame_id": "camera_link",
                    "camera_name": "armpi_camera",
                    "brightness": int(lc("camera_brightness")),
                }],
            ))
        else:
            actions.append(LogInfo(msg="[armpi_bringup] 경고: usb_cam 패키지 없음 → 카메라 건너뜀"))

    if on("web_video"):
        if _have("web_video_server"):
            actions.append(Node(
                package="web_video_server", executable="web_video_server", name="web_video_server",
                output="screen", respawn=True, respawn_delay=3.0,
                parameters=[{"port": int(lc("web_video_port")), "address": "0.0.0.0"}],
            ))
        else:
            actions.append(LogInfo(msg="[armpi_bringup] 경고: web_video_server 패키지 없음 → MJPEG 스트림 건너뜀"))

    if on("rosbridge"):
        share = _have("rosbridge_server")
        xml = os.path.join(share, "launch", "rosbridge_websocket_launch.xml") if share else None
        if xml and os.path.exists(xml):
            actions.append(IncludeLaunchDescription(
                AnyLaunchDescriptionSource(xml), launch_arguments={"port": lc("rosbridge_port")}.items()))
        else:
            actions.append(LogInfo(msg="[armpi_bringup] 경고: rosbridge_server 패키지 없음 → rosbridge 건너뜀"))
    return actions


_POWER_LINE = {"disabled": 0, "50": 1, "60": 2}


def _camera_power_line(device, hz):
    # usb_cam 0.8 에 전원 주파수 매개변수가 없어 v4l2-ctl 로 한 번 적용한다(실패해도 카메라는 그대로 켠다).
    hz = hz.strip().lower()
    if not hz:
        return []
    if hz not in _POWER_LINE:
        return [LogInfo(msg="[armpi_bringup] 경고: camera_power_line_hz 는 60/50/disabled/빈 값 중 하나: " + hz)]
    tool = shutil.which("v4l2-ctl")
    if not tool:
        return [LogInfo(msg="[armpi_bringup] 경고: v4l2-ctl 없음 → 카메라 전원 주파수 설정 건너뜀")]
    return [ExecuteProcess(cmd=[tool, "-d", device, "-c", "power_line_frequency=%d" % _POWER_LINE[hz]],
                           name="camera_power_line", output="screen"),
            LogInfo(msg="[armpi_bringup] 카메라 전원 주파수 보정: %s (%s)" % (hz, device))]


def _dds_profile(context):
    # 노드를 띄우기 전에 환경 변수를 정한다(이 launch 가 띄우는 모든 노드에 적용).
    path = LaunchConfiguration("dds_profile").perform(context)
    if not path:
        return [LogInfo(msg="[armpi_bringup] Fast DDS 기본 설정 사용(dds_profile 빈 값)")]
    if not os.path.isfile(path):
        return [LogInfo(msg="[armpi_bringup] 경고: dds_profile 파일 없음 → Fast DDS 기본 설정: " + path)]
    return [SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", path),
            LogInfo(msg="[armpi_bringup] Fast DDS 설정: " + path)]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("armpi_bringup"), "config", "armpi.yaml")
    default_dds = os.path.join(get_package_share_directory("armpi_bringup"), "config", "fastdds_armpi.xml")
    args = [
        DeclareLaunchArgument("params_file", default_value=default_params, description="매개변수 YAML"),
        DeclareLaunchArgument("mock", default_value="false", description="가짜 하드웨어"),
        DeclareLaunchArgument("hardware_profile", default_value="",
                              description="하드웨어 프로필 YAML(비우면 mock 은 시험용, 실제는 config/hardware_profile.yaml)"),
        DeclareLaunchArgument("vr_bridge", default_value="true"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),
        # usb_cam 0.8: YUYV 카메라를 rgb8 로 변환해야 compressed(JPEG) 전송이 된다
        DeclareLaunchArgument("pixel_format", default_value="yuyv2rgb"),
        DeclareLaunchArgument("image_width", default_value="640"),
        DeclareLaunchArgument("image_height", default_value="480"),
        DeclareLaunchArgument("framerate", default_value="15.0"),
        DeclareLaunchArgument("camera_brightness", default_value="0",
                              description="카메라 밝기(카메라 기본 0). usb_cam 0.8 기본 50 은 화면이 하얗게 뜸"),
        DeclareLaunchArgument("camera_power_line_hz", default_value="60",
                              description="전원 주파수 깜빡임 보정 60/50/disabled/빈 값(한국 60 Hz)"),
        DeclareLaunchArgument("web_video", default_value="true"),
        DeclareLaunchArgument("web_video_port", default_value="8080"),
        DeclareLaunchArgument("rosbridge", default_value="true"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument("dds_profile", default_value=default_dds,
                              description="Fast DDS XML(대용량 영상용). 빈 값이면 Fast DDS 기본값"),
    ]
    return LaunchDescription(args + [
        OpaqueFunction(function=_dds_profile),
        LogInfo(msg=["[armpi_bringup] 매개변수: ", LaunchConfiguration("params_file"),
                     ", mock=", LaunchConfiguration("mock")]),
        OpaqueFunction(function=_control),
        GroupAction([OpaqueFunction(function=_optional)]),
    ])
