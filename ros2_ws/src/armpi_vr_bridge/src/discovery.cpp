#include "armpi_vr_bridge/discovery.hpp"

#include <unistd.h>

#include <array>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace armpi_vr_bridge {

// ── SHA-256 (FIPS 180-4). 외부 라이브러리 없이 식별자 계산에만 쓴다. ──
namespace {

constexpr std::array<uint32_t, 64> kK = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

inline uint32_t rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

}  // namespace

std::string sha256_hex(std::string_view data) {
  uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                   0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  std::vector<unsigned char> msg(data.begin(), data.end());
  const uint64_t bit_len = static_cast<uint64_t>(data.size()) * 8;
  msg.push_back(0x80);
  while (msg.size() % 64 != 56) msg.push_back(0x00);
  for (int i = 7; i >= 0; --i) msg.push_back(static_cast<unsigned char>((bit_len >> (i * 8)) & 0xFF));

  for (std::size_t off = 0; off < msg.size(); off += 64) {
    uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
      w[i] = (static_cast<uint32_t>(msg[off + 4 * i]) << 24) | (static_cast<uint32_t>(msg[off + 4 * i + 1]) << 16) |
             (static_cast<uint32_t>(msg[off + 4 * i + 2]) << 8) | static_cast<uint32_t>(msg[off + 4 * i + 3]);
    }
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; ++i) {
      const uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const uint32_t ch = (e & f) ^ (~e & g);
      const uint32_t t1 = hh + S1 + ch + kK[i] + w[i];
      const uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t t2 = S0 + maj;
      hh = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
    h[5] += f;
    h[6] += g;
    h[7] += hh;
  }
  char out[65];
  for (int i = 0; i < 8; ++i) std::snprintf(out + i * 8, 9, "%08x", h[i]);
  return std::string(out, 64);
}

std::string device_id_from(std::string_view identity) {
  return sha256_hex(std::string("armpi-discovery-v1:") + std::string(identity)).substr(0, 32);
}

std::string device_id(const std::string& machine_id_path) {
  std::string identity;
  std::ifstream f(machine_id_path);
  if (f) {
    std::stringstream ss;
    ss << f.rdbuf();
    identity = ss.str();
    // Python str.strip() 처럼 앞뒤 공백·개행 제거
    const auto first = identity.find_first_not_of(" \t\r\n\v\f");
    const auto last = identity.find_last_not_of(" \t\r\n\v\f");
    identity = first == std::string::npos ? std::string() : identity.substr(first, last - first + 1);
  }
  if (identity.empty()) {
    char host[256] = {0};
    if (::gethostname(host, sizeof(host) - 1) == 0) identity = host;
  }
  return device_id_from(identity);
}

std::string url_quote(std::string_view text) {
  static const char* hex = "0123456789ABCDEF";
  std::string out;
  for (const char ch : text) {
    const auto c = static_cast<unsigned char>(ch);
    const bool unreserved = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                            c == '_' || c == '.' || c == '-' || c == '~';
    if (unreserved) {
      out.push_back(ch);
    } else {
      out.push_back('%');
      out.push_back(hex[c >> 4]);
      out.push_back(hex[c & 0x0F]);
    }
  }
  return out;
}

namespace {

// 유효한 UTF-8 을 코드포인트로 푼다. 잘못되면 false.
bool decode_utf8(std::string_view s, std::vector<uint32_t>& out) {
  const auto* p = reinterpret_cast<const unsigned char*>(s.data());
  std::size_t i = 0;
  while (i < s.size()) {
    const unsigned char c = p[i];
    uint32_t cp;
    std::size_t len;
    if (c < 0x80) {
      cp = c;
      len = 1;
    } else if (c >= 0xC2 && c <= 0xDF) {
      cp = c & 0x1F;
      len = 2;
    } else if (c >= 0xE0 && c <= 0xEF) {
      cp = c & 0x0F;
      len = 3;
    } else if (c >= 0xF0 && c <= 0xF4) {
      cp = c & 0x07;
      len = 4;
    } else {
      return false;
    }
    if (i + len > s.size()) return false;
    for (std::size_t k = 1; k < len; ++k) {
      if ((p[i + k] & 0xC0) != 0x80) return false;
      cp = (cp << 6) | (p[i + k] & 0x3F);
    }
    if ((len == 3 && cp < 0x800) || (len == 4 && cp < 0x10000) || (cp >= 0xD800 && cp <= 0xDFFF) ||
        cp > 0x10FFFF) {
      return false;
    }
    out.push_back(cp);
    i += len;
  }
  return true;
}

bool is_space_cp(uint32_t cp) {
  return cp == 0x20 || (cp >= 0x09 && cp <= 0x0D) || (cp >= 0x1C && cp <= 0x1F) || cp == 0x85 || cp == 0xA0 ||
         cp == 0x1680 || (cp >= 0x2000 && cp <= 0x200A) || cp == 0x2028 || cp == 0x2029 || cp == 0x202F ||
         cp == 0x205F || cp == 0x3000;
}

bool is_lower_hex32(std::string_view s) {
  if (s.size() != 32) return false;
  for (const char c : s) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  return true;
}

}  // namespace

bool valid_device_name(std::string_view name) {
  std::vector<uint32_t> cps;
  if (!decode_utf8(name, cps)) return false;
  if (cps.empty() || cps.size() > 32) return false;
  bool only_space = true;
  for (const uint32_t cp : cps) {
    if (cp < 32 || (cp >= 127 && cp <= 159)) return false;
    if (!is_space_cp(cp)) only_space = false;
  }
  return !only_space;
}

std::optional<std::string> make_discovery_reply(std::string_view packet, const std::string& name,
                                                const std::string& identity, uint16_t control_port) {
  if (packet.size() != kDiscoverRequestPrefix.size() + 32) return std::nullopt;
  if (packet.substr(0, kDiscoverRequestPrefix.size()) != kDiscoverRequestPrefix) return std::nullopt;
  const std::string_view nonce = packet.substr(kDiscoverRequestPrefix.size());
  if (!is_lower_hex32(nonce)) return std::nullopt;
  std::string reply(kDiscoverReplyPrefix);
  reply += "|";
  reply += nonce;
  reply += "|";
  reply += identity;
  reply += "|";
  reply += url_quote(name);
  reply += "|";
  reply += std::to_string(control_port);
  reply += "|3";
  return reply;
}

DiscoveryResponder::DiscoveryResponder(std::string name, std::string identity, uint16_t control_port,
                                       int max_replies_per_second)
    : name_(std::move(name)), identity_(std::move(identity)), control_port_(control_port),
      max_replies_(max_replies_per_second) {
  if (!valid_device_name(name_)) throw std::invalid_argument("검색 이름은 출력 가능한 1~32자여야 합니다");
  if (!is_lower_hex32(identity_)) throw std::invalid_argument("검색 식별자는 소문자 16진수 32자여야 합니다");
  if (control_port_ == 0) throw std::invalid_argument("제어 포트는 1~65535 여야 합니다");
  if (max_replies_ < 1) throw std::invalid_argument("초당 최대 검색 응답 수는 1 이상이어야 합니다");
}

std::optional<std::string> DiscoveryResponder::handle(std::string_view packet, double now) {
  auto reply = make_discovery_reply(packet, name_, identity_, control_port_);
  if (!reply) return std::nullopt;
  if (!window_started_ || now - window_start_ >= 1.0) {
    window_started_ = true;
    window_start_ = now;
    replies_in_window_ = 0;
  }
  if (replies_in_window_ >= max_replies_) {
    ++rate_limited_;
    return std::nullopt;
  }
  ++replies_in_window_;
  ++replies_sent_;
  return reply;
}

}  // namespace armpi_vr_bridge
