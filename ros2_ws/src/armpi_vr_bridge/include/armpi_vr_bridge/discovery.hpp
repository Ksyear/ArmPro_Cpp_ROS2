// LAN 검색 응답 (UDP 5002). docs/01_PROTOCOL_V3_SPEC.md §1 [v3 변경].
//   요청: ARMPI_DISCOVER/1|<32 hex nonce(소문자)>
//   응답: ARMPI_HERE/1|<nonce>|<32 hex id>|<URL 인코딩 이름>|<제어 포트>|3  (출발 포트 = 검색 포트)
// 검색은 제어 소켓을 열거나 소유권을 바꾸지 않는다. 순수 로직(소켓은 udp_server.hpp).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace armpi_vr_bridge {

constexpr std::string_view kDiscoverRequestPrefix = "ARMPI_DISCOVER/1|";
constexpr std::string_view kDiscoverReplyPrefix = "ARMPI_HERE/1";
constexpr uint16_t kDefaultDiscoveryPort = 5002;

// SHA-256 16진수(소문자 64자)
std::string sha256_hex(std::string_view data);

// 기기 식별자: SHA-256("armpi-discovery-v1:" + machine-id) 앞 32 hex.
// DHCP·Wi-Fi 가 바뀌어도 유지되고 machine-id 원문·원문 해시를 노출하지 않는다.
std::string device_id_from(std::string_view identity);
// /etc/machine-id 를 읽는다. 없거나 비면 hostname 을 쓴다.
std::string device_id(const std::string& machine_id_path = "/etc/machine-id");

// Python urllib.parse.quote(name, safe="") 와 같다(A-Z a-z 0-9 _ . - ~ 외 %XX 대문자).
std::string url_quote(std::string_view text);

// 이름: 유효한 UTF-8, 1~32 코드포인트, 공백만으로 이루어지지 않음, 제어 문자(U+0000-001F, U+007F-009F) 없음.
bool valid_device_name(std::string_view name);

// 형식이 맞는 요청이면 응답 문자열, 아니면 nullopt (속도 제한 없음)
std::optional<std::string> make_discovery_reply(std::string_view packet, const std::string& name,
                                                const std::string& identity, uint16_t control_port);

// 형식 검사 + 초당 최대 응답 수 제한(1초 창).
class DiscoveryResponder {
 public:
  // 잘못된 이름·식별자·포트·제한값은 std::invalid_argument
  DiscoveryResponder(std::string name, std::string identity, uint16_t control_port, int max_replies_per_second = 20);

  std::optional<std::string> handle(std::string_view packet, double now);
  uint64_t replies_sent() const { return replies_sent_; }
  uint64_t rate_limited() const { return rate_limited_; }
  const std::string& name() const { return name_; }
  const std::string& identity() const { return identity_; }

 private:
  std::string name_;
  std::string identity_;
  uint16_t control_port_;
  int max_replies_;
  bool window_started_{false};
  double window_start_{0.0};
  int replies_in_window_{0};
  uint64_t replies_sent_{0};
  uint64_t rate_limited_{0};
};

}  // namespace armpi_vr_bridge
