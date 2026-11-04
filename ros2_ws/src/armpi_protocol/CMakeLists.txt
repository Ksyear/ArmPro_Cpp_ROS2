cmake_minimum_required(VERSION 3.8)
project(armpi_protocol)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(armpi_common REQUIRED)

# Protocol v3: 형식(서버·클라이언트 공용, ROS 비의존) + 클라이언트 세션(UDP)
add_library(armpi_protocol STATIC
  src/protocol.cpp       # 패킷 해석·ACK 작성(수신기)
  src/client_codec.cpp   # 패킷 작성·ACK 해석(클라이언트)
  src/client.cpp         # 클라이언트 세션 상태 기계
  src/udp_link.cpp       # connect 된 UDP 송수신
)
set_target_properties(armpi_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(armpi_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(armpi_protocol PUBLIC armpi_common::armpi_common)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS armpi_protocol EXPORT export_armpi_protocol
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_armpi_protocol HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(armpi_common)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(name protocol client_codec client)
    ament_add_gtest(test_${name} test/test_${name}.cpp TIMEOUT 120)
    target_link_libraries(test_${name} armpi_protocol)
  endforeach()
endif()
ament_package()
