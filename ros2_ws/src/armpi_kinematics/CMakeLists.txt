cmake_minimum_required(VERSION 3.8)
project(armpi_kinematics)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)

add_library(armpi_kinematics STATIC src/arm_kinematics.cpp src/reachability.cpp)
set_target_properties(armpi_kinematics PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(armpi_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS armpi_kinematics EXPORT export_armpi_kinematics
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_armpi_kinematics HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_arm_kinematics test/test_arm_kinematics.cpp)
  target_link_libraries(test_arm_kinematics armpi_kinematics)
  ament_add_gtest(test_reachability test/test_reachability.cpp)
  target_link_libraries(test_reachability armpi_kinematics)
endif()
ament_package()
