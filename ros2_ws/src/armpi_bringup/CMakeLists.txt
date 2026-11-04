cmake_minimum_required(VERSION 3.8)
project(armpi_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # 설정 파일 검사·통합·종단 시험은 test/dev_build_and_test.sh 가 개발 컨테이너에서 실행한다.
endif()

ament_package()
