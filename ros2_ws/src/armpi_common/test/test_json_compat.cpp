// ROS 문자열 토픽용 설정(lenient_topic_options) 시험: 예전 armpi_control/mini_json 시험을 옮겼다.
// estop_request, vr_bridge state 형식, 고정 소수 출력.
#include <gtest/gtest.h>

#include <cmath>

#include "armpi_common/json.hpp"

namespace json = armpi_common::json;

TEST(JsonTopic, EstopRequest) {
  json::Value v;
  std::string err;
  const auto opt = json::lenient_topic_options();
  ASSERT_TRUE(json::parse("{\"source\":\"udp\",\"latch\":true}", v, err, opt)) << err;
  ASSERT_TRUE(v.is_object());
  EXPECT_EQ(v.find("source")->string, "udp");
  EXPECT_TRUE(v.find("latch")->boolean);
  EXPECT_EQ(v.find("missing"), nullptr);
}

TEST(JsonTopic, BridgeStateWithNullAndNested) {
  const std::string s =
      "{\"owner\":null,\"owner_addr\":null,\"base_state\":\"awaiting_neutral\",\"arm_state\":\"idle\","
      "\"emergency_stop\":false,\"packets\":{\"received\":12,\"rate_hz\":19.5,\"reasons\":{\"ok\":3}}}";
  json::Value v;
  std::string err;
  const auto opt = json::lenient_topic_options();
  ASSERT_TRUE(json::parse(s, v, err, opt)) << err;
  EXPECT_TRUE(v.find("owner")->is_null());
  EXPECT_EQ(v.find("arm_state")->string, "idle");
  EXPECT_DOUBLE_EQ(v.find("packets")->find("rate_hz")->number, 19.5);
  ASSERT_TRUE(json::parse("{\"owner\":\"d8b27f10-09a8-4c21-b649-1e7058d46b72\"}", v, err, opt));
  EXPECT_EQ(v.find("owner")->string, "d8b27f10-09a8-4c21-b649-1e7058d46b72");
}

TEST(JsonTopic, StringsAndNumbers) {
  json::Value v;
  std::string err;
  const auto opt = json::lenient_topic_options();
  ASSERT_TRUE(json::parse("[\"a\\\"b\\\\c\\n\", \"\\u00e9\\ud83d\\ude00\", -1.5e2, 0, 1e-3]", v, err, opt)) << err;
  EXPECT_EQ(v.array[0].string, "a\"b\\c\n");
  EXPECT_EQ(v.array[1].string, "\xC3\xA9\xF0\x9F\x98\x80");
  EXPECT_DOUBLE_EQ(v.array[2].number, -150.0);
  EXPECT_DOUBLE_EQ(v.array[4].number, 0.001);
}

TEST(JsonTopic, RejectsInvalid) {
  json::Value v;
  std::string err;
  const auto opt = json::lenient_topic_options();
  EXPECT_FALSE(json::parse("", v, err, opt));
  EXPECT_FALSE(json::parse("{", v, err, opt));
  EXPECT_FALSE(json::parse("{\"a\":1,}", v, err, opt));
  EXPECT_FALSE(json::parse("{\"a\":NaN}", v, err, opt));
  EXPECT_FALSE(json::parse("{\"a\":01}", v, err, opt));
  EXPECT_FALSE(json::parse("{\"a\":1} x", v, err, opt));
  EXPECT_FALSE(json::parse("{a:1}", v, err, opt));
  EXPECT_FALSE(json::parse("[[[[[[[[[[[[[[[[[[[[1]]]]]]]]]]]]]]]]]]]", v, err, json::lenient_topic_options(65536, 8)));
  EXPECT_FALSE(json::parse(std::string(100, ' '), v, err, json::lenient_topic_options(50)));
}

TEST(JsonTopic, Writers) {
  EXPECT_EQ(json::quote("a\"b\\c\n\x01"), "\"a\\\"b\\\\c\\n\\u0001\"");
  EXPECT_EQ(json::fixed(1.23456, 3), "1.235");
  EXPECT_EQ(json::fixed(-0.00001, 4), "0.0000");
  EXPECT_EQ(json::fixed(NAN, 4), "null");
  EXPECT_EQ(json::boolean(true), "true");
  // 쓴 것을 다시 읽을 수 있어야 함
  json::Value v;
  std::string err;
  const auto opt = json::lenient_topic_options();
  ASSERT_TRUE(json::parse("{\"k\":" + json::quote("값 \"x\"") + ",\"n\":" + json::fixed(-2.5, 4) + "}", v, err, opt));
  EXPECT_EQ(v.find("k")->string, "값 \"x\"");
  EXPECT_DOUBLE_EQ(v.find("n")->number, -2.5);
}

TEST(JsonTopic, DuplicateKeysFirstWinsOnlyInTopicMode) {
  json::Value v;
  std::string err;
  ASSERT_TRUE(json::parse("{\"a\":1,\"a\":2}", v, err, json::lenient_topic_options())) << err;
  EXPECT_DOUBLE_EQ(v.find("a")->number, 1.0);
  EXPECT_FALSE(json::parse("{\"a\":1,\"a\":2}", v, err));  // 기본(Protocol v3)은 거부
}

TEST(JsonTopic, NonFiniteNumberRejectedOnlyInTopicMode) {
  json::Value v;
  std::string err;
  EXPECT_FALSE(json::parse("{\"a\":1e400}", v, err, json::lenient_topic_options()));
  ASSERT_TRUE(json::parse("{\"a\":1e400}", v, err)) << err;  // 기본은 받되 유한하지 않음으로 표시
  EXPECT_FALSE(v.find("a")->number_finite);
}

TEST(JsonTopic, FixedAndShortestWriters) {
  EXPECT_EQ(json::fixed(0.10470001, 4), "0.1047");
  EXPECT_EQ(json::fixed(-1.0263, 4), "-1.0263");
  EXPECT_EQ(json::number(0.1), "0.1");
  EXPECT_EQ(json::boolean(false), "false");
}
