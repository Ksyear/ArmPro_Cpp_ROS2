cmake_minimum_required(VERSION 3.8)
project(armpi_power)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 종료 정책(ROS 비의존): 한 번만 실행하는 래치 + 실행 방법(systemctl / 시험용 dry-run)
add_library(armpi_power_core STATIC src/power_policy.cpp)
set_target_properties(armpi_power_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(armpi_power_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(armpi_power_node src/power_node.cpp)
target_link_libraries(armpi_power_node armpi_power_core)
ament_target_dependencies(armpi_power_node rclcpp std_msgs)

install(TARGETS armpi_power_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS test/integration/test_power_e2e.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_power_policy test/test_power_policy.cpp TIMEOUT 60)
  target_link_libraries(test_power_policy armpi_power_core)
  # KEY2 → 종료 요청 → dry-run 종단 시험은 armpi_bringup/test/dev_build_and_test.sh 가 실행한다.
endif()
ament_package()
