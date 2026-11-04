// IPv4 주소·송신 주소(Endpoint)·allowed_sources CIDR 필터. 소켓 비의존 순수 로직.
#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace armpi_vr_bridge {

// 엄격한 점 4개 10진 표기(a.b.c.d, 각 0~255, 앞자리 0 불가). 성공 시 호스트 바이트 순서 값.
bool parse_ipv4(std::string_view text, uint32_t& out);
std::string ipv4_to_string(uint32_t ip);

struct Endpoint {
  uint32_t ip{0};  // 호스트 바이트 순서
  uint16_t port{0};

  bool operator==(const Endpoint& o) const { return ip == o.ip && port == o.port; }
  bool operator!=(const Endpoint& o) const { return !(*this == o); }
  std::string to_string() const;  // "a.b.c.d:port"
  // 시험·로그 편의. 잘못된 주소면 std::invalid_argument.
  static Endpoint from(const std::string& dotted, uint16_t port);
};

struct Cidr {
  uint32_t network{0};
  uint32_t mask{0};
  int prefix{0};
  bool contains(uint32_t ip) const { return (ip & mask) == network; }
  std::string to_string() const;
};

// "a.b.c.d/n" 또는 "a.b.c.d"(= /32). 잘못된 항목은 std::invalid_argument.
// 호스트 비트가 켜진 항목(192.168.2.5/24)은 네트워크 주소로 맞춘다(192.168.2.0/24).
Cidr parse_cidr(std::string_view text);

// [v3] allowed_sources. 빈 목록 = 모두 허용. 목록에 없는 주소는 응답 없이 버린다.
class SourceFilter {
 public:
  SourceFilter() = default;
  // 빈 문자열 항목은 무시한다(YAML 에서 빈 배열 대신 [""] 를 쓸 때 대비). 잘못된 항목은 예외.
  explicit SourceFilter(const std::vector<std::string>& entries);

  bool allows(uint32_t ip) const;
  bool allow_all() const { return entries_.empty(); }
  std::string describe() const;  // 로그용
  const std::vector<Cidr>& entries() const { return entries_; }

 private:
  std::vector<Cidr> entries_;
};

}  // namespace armpi_vr_bridge
