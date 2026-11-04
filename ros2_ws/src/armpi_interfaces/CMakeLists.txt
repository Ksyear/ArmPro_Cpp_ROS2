cmake_minimum_required(VERSION 3.8)
project(armpi_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 정의는 docs/02_ROS2_INTERFACES.md 의 ArmTarget.msg 와 같아야 한다.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmTarget.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
