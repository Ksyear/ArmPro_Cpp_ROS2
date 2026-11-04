// allowed_sources CIDR 필터 단위 시험 ([v3] docs/01_PROTOCOL_V3_SPEC.md §1 보안).
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "armpi_vr_bridge/net.hpp"

using namespace armpi_vr_bridge;

namespace {
uint32_t ip(const std::string& s) {
  uint32_t v = 0;
  EXPECT_TRUE(parse_ipv4(s, v)) << s;
  return v;
}
}  // namespace

TEST(Ipv4, StrictDottedDecimal) {
  uint32_t v = 0;
  EXPECT_TRUE(parse_ipv4("192.168.2.2", v));
  EXPECT_EQ(v, 0xC0A80202u);
  EXPECT_EQ(ipv4_to_string(v), "192.168.2.2");
  EXPECT_TRUE(parse_ipv4("0.0.0.0", v));
  EXPECT_TRUE(parse_ipv4("255.255.255.255", v));
  for (const char* bad : {"", "1.2.3", "1.2.3.4.5", "256.1.1.1", "01.2.3.4", "1.2.3.-4", "1..2.3", "a.b.c.d",
                          " 1.2.3.4", "1.2.3.4 ", "1.2.3.4/24", "0x7f.0.0.1", "1234.1.1.1"}) {
    EXPECT_FALSE(parse_ipv4(bad, v)) << bad;
  }
}

TEST(Endpoint, FormatAndCompare) {
  const Endpoint a = Endpoint::from("10.0.0.5", 5001);
  EXPECT_EQ(a.to_string(), "10.0.0.5:5001");
  EXPECT_EQ(a, Endpoint::from("10.0.0.5", 5001));
  EXPECT_NE(a, Endpoint::from("10.0.0.5", 5002));
  EXPECT_THROW(Endpoint::from("10.0.0", 1), std::invalid_argument);
}

TEST(SourceFilter, EmptyListAllowsAll) {
  SourceFilter f;
  EXPECT_TRUE(f.allow_all());
  EXPECT_TRUE(f.allows(ip("8.8.8.8")));
  SourceFilter g(std::vector<std::string>{});
  EXPECT_TRUE(g.allows(ip("192.168.2.10")));
  SourceFilter h(std::vector<std::string>{""});  // YAML [""] 대비: 빈 항목 무시 = 모두 허용
  EXPECT_TRUE(h.allow_all());
}

TEST(SourceFilter, CidrMatching) {
  SourceFilter f({"192.168.2.0/24", "10.42.0.5", "172.16.0.0/12"});
  EXPECT_FALSE(f.allow_all());
  EXPECT_TRUE(f.allows(ip("192.168.2.0")));
  EXPECT_TRUE(f.allows(ip("192.168.2.255")));
  EXPECT_FALSE(f.allows(ip("192.168.3.1")));
  EXPECT_TRUE(f.allows(ip("10.42.0.5")));  // 접두 길이 없음 = /32
  EXPECT_FALSE(f.allows(ip("10.42.0.6")));
  EXPECT_TRUE(f.allows(ip("172.31.255.255")));
  EXPECT_FALSE(f.allows(ip("172.32.0.0")));
  EXPECT_FALSE(f.allows(ip("127.0.0.1")));
  EXPECT_EQ(f.describe(), "192.168.2.0/24, 10.42.0.5/32, 172.16.0.0/12");
}

TEST(SourceFilter, PrefixZeroAndThirtyTwo) {
  SourceFilter all({"0.0.0.0/0"});
  EXPECT_TRUE(all.allows(ip("1.2.3.4")));
  SourceFilter one({"127.0.0.1/32"});
  EXPECT_TRUE(one.allows(ip("127.0.0.1")));
  EXPECT_FALSE(one.allows(ip("127.0.0.2")));
}

TEST(SourceFilter, HostBitsAreMaskedToNetwork) {
  const Cidr c = parse_cidr("192.168.2.77/24");
  EXPECT_EQ(c.to_string(), "192.168.2.0/24");
  SourceFilter f({"192.168.2.77/24"});
  EXPECT_TRUE(f.allows(ip("192.168.2.1")));
}

TEST(SourceFilter, InvalidEntriesThrow) {
  for (const char* bad : {"300.1.1.1/8", "1.2.3.4/33", "abc", "1.2.3/24", "1.2.3.4/", "1.2.3.4/-1", " 1.2.3.4",
                          "1.2.3.4/24/1", "1.2.3.4/024", "::1", "1.2.3.4/x"}) {
    EXPECT_THROW(SourceFilter(std::vector<std::string>{bad}), std::invalid_argument) << bad;
  }
}
