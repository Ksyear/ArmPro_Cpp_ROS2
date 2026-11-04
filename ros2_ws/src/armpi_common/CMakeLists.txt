cmake_minimum_required(VERSION 3.8)
project(armpi_common)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)

# 공용 라이브러리(ROS 비의존): JSON
add_library(armpi_common STATIC src/json.cpp)
set_target_properties(armpi_common PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(armpi_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS armpi_common EXPORT export_armpi_common
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_armpi_common HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(name json_strict json_compat)
    ament_add_gtest(test_${name} test/test_${name}.cpp TIMEOUT 120)
    target_link_libraries(test_${name} armpi_common)
  endforeach()
endif()
ament_package()
