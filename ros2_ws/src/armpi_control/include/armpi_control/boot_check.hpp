// 부팅 점검: 지금 켜진 커널·부팅 설정이 이 이미지의 것인지 확인한다(ROS 비의존, 판정은 순수 함수).
//
// 로봇에 예전 시스템이 든 SD 카드가 꽂혀 있으면 라즈베리파이는 SD 카드의 부팅 파일(커널·cmdline)로 켜지고,
// 운영체제 본체만 새 USB 것을 쓸 수 있다. 그러면 커널 모듈이 맞지 않아 무선·카메라 드라이버가 올라오지 않는다.
// 이미지 매니페스트(/opt/armpi/manifests/stage20.env: kernel=, partuuid_root=)와 실제 값을 비교해 경고한다.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace armpi_control {

struct BootFacts {
  std::string kernel_release;   // uname -r
  bool modules_dir_exists{false};  // /lib/modules/<kernel_release>
  std::string cmdline;          // /proc/cmdline
  bool manifest_found{false};
  std::map<std::string, std::string> manifest;  // stage20.env 의 키=값
};

// 경고 문장 목록(한국어). 매니페스트가 없으면(이미지가 아닌 개발 환경) 빈 목록.
std::vector<std::string> boot_warnings(const BootFacts& facts);

// 실제 시스템에서 읽는다. 경로는 시험에서 바꿀 수 있다.
BootFacts read_boot_facts(const std::string& manifest_path = "/opt/armpi/manifests/stage20.env",
                          const std::string& modules_root = "/lib/modules",
                          const std::string& cmdline_path = "/proc/cmdline");

// "키=값" 줄 해석(# 주석·빈 줄 무시)
std::map<std::string, std::string> parse_env(const std::string& text);

}  // namespace armpi_control
