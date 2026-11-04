cmake_minimum_required(VERSION 3.8)
project(armpi_web_gateway)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(armpi_common REQUIRED)
find_package(armpi_protocol REQUIRED)
find_package(Threads REQUIRED)

# 웹 조종기 핵심(ROS 비의존): 조종권·세션·명령(Controller), HTTP 경로(WebApi), HTTP 서버
add_library(armpi_web_gateway_core STATIC
  src/input.cpp
  src/controller.cpp
  src/http.cpp
  src/web_api.cpp
  src/http_server.cpp
)
set_target_properties(armpi_web_gateway_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(armpi_web_gateway_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(armpi_web_gateway_core PUBLIC armpi_protocol::armpi_protocol armpi_common::armpi_common
  Threads::Threads)

# ROS 2 노드: rclcpp 어댑터 + 조립
add_executable(armpi_web_gateway_node src/ros_robot_link.cpp src/main.cpp)
target_link_libraries(armpi_web_gateway_node armpi_web_gateway_core)
ament_target_dependencies(armpi_web_gateway_node rclcpp std_msgs std_srvs sensor_msgs ament_index_cpp)

install(TARGETS armpi_web_gateway_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY www DESTINATION share/${PROJECT_NAME})
install(PROGRAMS test/integration/test_gateway_e2e.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(name input controller web_api http)
    ament_add_gtest(test_gw_${name} test/test_${name}.cpp TIMEOUT 120)
    target_link_libraries(test_gw_${name} armpi_web_gateway_core)
    target_compile_definitions(test_gw_${name} PRIVATE ARMPI_WWW_DIR="${CMAKE_CURRENT_SOURCE_DIR}/www")
  endforeach()
  # 종단 시험(가짜 하드웨어 + 수신기 + 게이트웨이 HTTP)은 armpi_bringup/test/dev_build_and_test.sh 가 실행한다.
endif()

ament_package()
