#include "armpi_vr_bridge/net.hpp"

#include <stdexcept>

namespace armpi_vr_bridge {

namespace {
bool parse_uint(std::string_view s, unsigned max, unsigned& out) {
  if (s.empty() || s.size() > 3) return false;
  if (s.size() > 1 && s[0] == '0') return false;  // 앞자리 0 (8진수 해석 혼동 방지)
  unsigned v = 0;
  for (const char c : s) {
    if (c < '0' || c > '9') return false;
    v = v * 10 + static_cast<unsigned>(c - '0');
  }
  if (v > max) return false;
  out = v;
  return true;
}
}  // namespace

bool parse_ipv4(std::string_view text, uint32_t& out) {
  uint32_t value = 0;
  std::size_t start = 0;
  for (int part = 0; part < 4; ++part) {
    const std::size_t dot = text.find('.', start);
    const bool last = part == 3;
    if (last != (dot == std::string_view::npos)) return false;
    const std::string_view piece = text.substr(start, last ? std::string_view::npos : dot - start);
    unsigned octet = 0;
    if (!parse_uint(piece, 255, octet)) return false;
    value = (value << 8) | octet;
    start = dot + 1;
  }
  out = value;
  return true;
}

std::string ipv4_to_string(uint32_t ip) {
  return std::to_string((ip >> 24) & 0xFF) + "." + std::to_string((ip >> 16) & 0xFF) + "." +
         std::to_string((ip >> 8) & 0xFF) + "." + std::to_string(ip & 0xFF);
}

std::string Endpoint::to_string() const { return ipv4_to_string(ip) + ":" + std::to_string(port); }

Endpoint Endpoint::from(const std::string& dotted, uint16_t port) {
  Endpoint e;
  if (!parse_ipv4(dotted, e.ip)) throw std::invalid_argument("잘못된 IPv4 주소: " + dotted);
  e.port = port;
  return e;
}

std::string Cidr::to_string() const { return ipv4_to_string(network) + "/" + std::to_string(prefix); }

Cidr parse_cidr(std::string_view text) {
  const std::size_t slash = text.find('/');
  const std::string_view addr = text.substr(0, slash);
  uint32_t ip = 0;
  if (!parse_ipv4(addr, ip)) {
    throw std::invalid_argument("allowed_sources 항목의 IPv4 주소가 잘못되었습니다: '" + std::string(text) + "'");
  }
  unsigned prefix = 32;
  if (slash != std::string_view::npos) {
    const std::string_view p = text.substr(slash + 1);
    if (!parse_uint(p, 32, prefix)) {
      throw std::invalid_argument("allowed_sources 항목의 접두 길이(0~32)가 잘못되었습니다: '" +
                                  std::string(text) + "'");
    }
  }
  Cidr c;
  c.prefix = static_cast<int>(prefix);
  c.mask = prefix == 0 ? 0u : (0xFFFFFFFFu << (32 - prefix));
  c.network = ip & c.mask;
  return c;
}

SourceFilter::SourceFilter(const std::vector<std::string>& entries) {
  for (const auto& e : entries) {
    if (e.empty()) continue;
    entries_.push_back(parse_cidr(e));
  }
}

bool SourceFilter::allows(uint32_t ip) const {
  if (entries_.empty()) return true;
  for (const auto& c : entries_) {
    if (c.contains(ip)) return true;
  }
  return false;
}

std::string SourceFilter::describe() const {
  if (entries_.empty()) return "모두 허용";
  std::string s;
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    if (i) s += ", ";
    s += entries_[i].to_string();
  }
  return s;
}

}  // namespace armpi_vr_bridge
