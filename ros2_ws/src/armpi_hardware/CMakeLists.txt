cmake_minimum_required(VERSION 3.8)
project(armpi_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

# libgpiod v1.6 C++ 바인딩(Ubuntu 22.04: libgpiod-dev). 없으면 실제 GPIO 없이 빌드(가짜 하드웨어만 동작).
find_package(PkgConfig QUIET)
set(ARMPI_HAVE_GPIOD OFF)
if(PkgConfig_FOUND)
  pkg_check_modules(GPIODCXX IMPORTED_TARGET libgpiodcxx)
  if(GPIODCXX_FOUND)
    set(ARMPI_HAVE_GPIOD ON)
  endif()
endif()
if(ARMPI_HAVE_GPIOD)
  message(STATUS "armpi_hardware: libgpiodcxx ${GPIODCXX_VERSION} 사용")
else()
  message(WARNING "armpi_hardware: libgpiodcxx 없음 → 실제 GPIO(서보 방향·부저·버튼) 사용 불가. 'apt install libgpiod-dev' 후 다시 빌드")
endif()

add_library(armpi_hardware STATIC
  src/conversions.cpp
  src/hardware_profile.cpp
  src/servo_protocol.cpp
  src/linux_i2c_bus.cpp
  src/linux_serial_port.cpp
  src/gpiod_gpio.cpp
  src/devices.cpp
  src/mock.cpp
  src/hardware.cpp
)
set_target_properties(armpi_hardware PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(armpi_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(ARMPI_HAVE_GPIOD)
  target_compile_definitions(armpi_hardware PRIVATE ARMPI_HAVE_GPIOD=1)
  # 정적 라이브러리이므로 소비자도 libgpiodcxx 를 링크해야 한다(아래 ament_export_dependencies 대신 직접 링크 정보 전달)
  target_link_libraries(armpi_hardware PUBLIC ${GPIODCXX_LINK_LIBRARIES})
  target_include_directories(armpi_hardware PRIVATE ${GPIODCXX_INCLUDE_DIRS})
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS armpi_hardware EXPORT export_armpi_hardware
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_armpi_hardware HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(name conversions servo_protocol devices)
    ament_add_gtest(test_hw_${name} test/test_${name}.cpp TIMEOUT 60)
    target_link_libraries(test_hw_${name} armpi_hardware)
    target_include_directories(test_hw_${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  endforeach()
endif()

ament_package()
