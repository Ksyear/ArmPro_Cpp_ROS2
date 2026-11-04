cmake_minimum_required(VERSION 3.8)
project(armpi_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(armpi_interfaces REQUIRED)
find_package(armpi_kinematics REQUIRED)
find_package(armpi_hardware REQUIRED)
find_package(armpi_common REQUIRED)

# 제어 논리(ROS 비의존): 단위 시험 대상
add_library(armpi_control_core STATIC
  src/safety.cpp
  src/chassis_controller.cpp
  src/arm_controller.cpp
  src/boot_check.cpp
  src/buzzer_pattern.cpp
)
set_target_properties(armpi_control_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(armpi_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(armpi_control_core PUBLIC armpi_hardware::armpi_hardware armpi_kinematics::armpi_kinematics
  armpi_common::armpi_common)

# ROS 2 노드
add_executable(armpi_control_node
  src/main.cpp
  src/control_node.cpp          # 매개변수·준비·종료
  src/control_node_safety.cpp   # 비상정지·fault·바퀴 쓰기·부저(시작 알림 포함)
  src/control_node_inputs.cpp   # 구독 콜백·서비스
  src/control_node_timers.cpp   # 주기 작업
  src/control_node_status.cpp   # 상태 JSON·발행
)
target_link_libraries(armpi_control_node armpi_control_core)
ament_target_dependencies(armpi_control_node rclcpp std_msgs std_srvs geometry_msgs sensor_msgs armpi_interfaces)

install(TARGETS armpi_control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS test/integration/test_control_integration.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(name safety chassis arm_controller boot_check buzzer_pattern)
    ament_add_gtest(test_ctl_${name} test/test_${name}.cpp TIMEOUT 120)
    target_link_libraries(test_ctl_${name} armpi_control_core)
  endforeach()
  # 노드 통합 시험(가짜 하드웨어 + ROS 토픽)은 armpi_bringup/test/dev_build_and_test.sh 가 설치 공간에서 따로 실행한다.
endif()

ament_package()
