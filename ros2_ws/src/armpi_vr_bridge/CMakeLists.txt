cmake_minimum_required(VERSION 3.8)
project(armpi_vr_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(armpi_interfaces REQUIRED)
find_package(armpi_kinematics REQUIRED)
find_package(armpi_common REQUIRED)
find_package(armpi_protocol REQUIRED)
find_package(Threads REQUIRED)

# 수신기 계층(순수 C++, ROS 비의존): 세션·검색·CIDR·UDP 서버. 패킷 형식은 armpi_protocol, JSON 은 armpi_common.
add_library(armpi_vr_bridge_core STATIC
  src/net.cpp
  src/session.cpp
  src/rx_stats.cpp
  src/discovery.cpp
  src/udp_server.cpp
)
set_target_properties(armpi_vr_bridge_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(armpi_vr_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(armpi_vr_bridge_core PUBLIC armpi_protocol::armpi_protocol armpi_common::armpi_common
  armpi_kinematics::armpi_kinematics Threads::Threads)

# ROS 2 노드
add_executable(armpi_vr_bridge_node src/bridge_node.cpp src/main.cpp)
target_link_libraries(armpi_vr_bridge_node armpi_vr_bridge_core)
ament_target_dependencies(armpi_vr_bridge_node rclcpp std_msgs geometry_msgs armpi_interfaces)

install(TARGETS armpi_vr_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS test/integration/test_bridge_integration.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(name session discovery source_filter udp_server)
    ament_add_gtest(test_${name} test/test_${name}.cpp TIMEOUT 120)
    target_link_libraries(test_${name} armpi_vr_bridge_core)
  endforeach()
  # 노드 통합 시험(UDP ↔ ROS 토픽)은 scripts/armpi_dev_build_and_test.sh 가 설치 공간에서 따로 실행한다.
endif()

ament_package()
