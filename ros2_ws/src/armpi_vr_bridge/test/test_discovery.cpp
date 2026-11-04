// 검색 응답 형식·식별자·속도 제한 단위 시험 (docs/01_PROTOCOL_V3_SPEC.md §1).
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "armpi_vr_bridge/discovery.hpp"

using namespace armpi_vr_bridge;

namespace {
const std::string kNonce = "0123456789abcdef0123456789abcdef";
const std::string kId = std::string(32, 'a');

std::vector<std::string> split(const std::string& s, char d) {
  std::vector<std::string> out;
  std::size_t start = 0;
  while (true) {
    const auto p = s.find(d, start);
    out.push_back(s.substr(start, p == std::string::npos ? std::string::npos : p - start));
    if (p == std::string::npos) break;
    start = p + 1;
  }
  return out;
}
}  // namespace

TEST(Sha256, KnownVectors) {
  EXPECT_EQ(sha256_hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(sha256_hex("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(sha256_hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  EXPECT_EQ(sha256_hex(std::string(1000000, 'a')),
            "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(DeviceId, StableDomainSeparatedHash) {
  const std::string id = device_id_from("0123456789abcdef0123456789abcdef");
  EXPECT_EQ(id.size(), 32u);
  EXPECT_EQ(id, sha256_hex("armpi-discovery-v1:0123456789abcdef0123456789abcdef").substr(0, 32));
  EXPECT_NE(id, sha256_hex("0123456789abcdef0123456789abcdef").substr(0, 32));  // 원문 해시를 노출하지 않음
  EXPECT_EQ(id, device_id_from("0123456789abcdef0123456789abcdef"));
}

TEST(DeviceId, ReadsMachineIdFileAndStripsWhitespace) {
  const std::string path = ::testing::TempDir() + "armpi_machine_id_test";
  {
    std::ofstream f(path);
    f << "  feedfacefeedfacefeedfacefeedface\n";
  }
  EXPECT_EQ(device_id(path), device_id_from("feedfacefeedfacefeedfacefeedface"));
  std::remove(path.c_str());
  // 파일이 없으면 hostname 기반(형식만 확인)
  const std::string fallback = device_id(path + "_missing");
  EXPECT_EQ(fallback.size(), 32u);
}

TEST(UrlQuote, MatchesPythonQuoteSafeEmpty) {
  EXPECT_EQ(url_quote("armpi-pro_1.2~x"), "armpi-pro_1.2~x");
  EXPECT_EQ(url_quote("armpi pro"), "armpi%20pro");
  EXPECT_EQ(url_quote("a/b|c%d"), "a%2Fb%7Cc%25d");
  EXPECT_EQ(url_quote("로봇"), "%EB%A1%9C%EB%B4%87");
}

TEST(DeviceName, Validation) {
  EXPECT_TRUE(valid_device_name("armpi"));
  EXPECT_TRUE(valid_device_name(std::string(32, 'n')));
  std::string korean32;
  for (int i = 0; i < 32; ++i) korean32 += "가";
  EXPECT_TRUE(valid_device_name(korean32));  // 바이트가 아니라 문자 수
  EXPECT_FALSE(valid_device_name(korean32 + "가"));
  EXPECT_FALSE(valid_device_name(""));
  EXPECT_FALSE(valid_device_name("   "));
  EXPECT_FALSE(valid_device_name(std::string(33, 'n')));
  EXPECT_FALSE(valid_device_name("bad\nname"));
  EXPECT_FALSE(valid_device_name("bad\x7f"));
  EXPECT_FALSE(valid_device_name("bad\xc2\x85"));  // U+0085 (C1 제어 문자)
  EXPECT_FALSE(valid_device_name("\xff"));
}

TEST(DiscoveryReply, FormatMatchesSpec) {
  const auto reply = make_discovery_reply("ARMPI_DISCOVER/1|" + kNonce, "armpi pro", kId, 5001);
  ASSERT_TRUE(reply.has_value());
  EXPECT_EQ(split(*reply, '|'), (std::vector<std::string>{"ARMPI_HERE/1", kNonce, kId, "armpi%20pro", "5001", "3"}));
  EXPECT_GE(reply->size(), 80u);  // VR 쪽 파서의 80~640 byte 조건
  EXPECT_LE(reply->size(), 640u);
  for (const char c : *reply) EXPECT_TRUE(c >= 0x20 && c < 0x7f);  // 출력 가능한 ASCII
}

TEST(DiscoveryReply, LongestNameStillWithinLimit) {
  std::string name;
  for (int i = 0; i < 32; ++i) name += "\xf0\x9f\x9a\x80";  // 4바이트 문자 32개
  const auto reply = make_discovery_reply("ARMPI_DISCOVER/1|" + kNonce, name, kId, 65535);
  ASSERT_TRUE(reply.has_value());
  EXPECT_LE(reply->size(), 640u);
}

TEST(DiscoveryReply, MalformedRequestsIgnored) {
  for (const std::string& bad :
       {std::string("OTHERBOT_DISCOVER/1|") + kNonce, std::string("ARMPI_DISCOVER/2|") + kNonce,
        std::string("ARMPI_DISCOVER/1|") + kNonce.substr(1), std::string("ARMPI_DISCOVER/1|") + kNonce + "0",
        std::string("ARMPI_DISCOVER/1|") + kNonce + "\n", std::string("ARMPI_DISCOVER/1|0123456789ABCDEF0123456789abcdef"),
        std::string("ARMPI_DISCOVER/1|0123456789abcdef0123456789abcdeg"), std::string("armpi_discover/1|") + kNonce,
        std::string(""), std::string("ARMPI_DISCOVER/1|")}) {
    EXPECT_FALSE(make_discovery_reply(bad, "armpi", kId, 5001).has_value()) << bad;
  }
}

TEST(DiscoveryResponder, RateLimitPerOneSecondWindow) {
  DiscoveryResponder r("armpi", kId, 5001, 20);
  const std::string req = "ARMPI_DISCOVER/1|" + kNonce;
  int replies = 0;
  for (int i = 0; i < 25; ++i) {
    if (r.handle(req, 10.0 + i * 0.01)) ++replies;
  }
  EXPECT_EQ(replies, 20);
  EXPECT_EQ(r.rate_limited(), 5u);
  EXPECT_FALSE(r.handle("garbage", 10.5).has_value());  // 형식 오류는 제한 계산에 넣지 않음
  EXPECT_TRUE(r.handle(req, 11.0).has_value());          // 새 1초 창
  EXPECT_EQ(r.replies_sent(), 21u);
}

TEST(DiscoveryResponder, ConstructorValidation) {
  EXPECT_THROW(DiscoveryResponder("", kId, 5001), std::invalid_argument);
  EXPECT_THROW(DiscoveryResponder("armpi", "XYZ", 5001), std::invalid_argument);
  EXPECT_THROW(DiscoveryResponder("armpi", std::string(32, 'A'), 5001), std::invalid_argument);
  EXPECT_THROW(DiscoveryResponder("armpi", kId, 0), std::invalid_argument);
  EXPECT_THROW(DiscoveryResponder("armpi", kId, 5001, 0), std::invalid_argument);
  EXPECT_NO_THROW(DiscoveryResponder("armpi", kId, 5001));
}
