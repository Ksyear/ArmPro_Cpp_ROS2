// 엄격한 JSON 파서 단위 시험. 기대값은 Python json.loads(json.loads + 중복 키 거부 + NaN 거부)과 같다.
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "armpi_common/json.hpp"

using armpi_common::json::parse;
using armpi_common::json::ParseOptions;
using armpi_common::json::Type;
using armpi_common::json::Value;

namespace {
bool ok(const std::string& s, Value* out = nullptr) {
  Value v;
  std::string err;
  const bool r = parse(s, v, err);
  if (out) *out = v;
  return r;
}
}  // namespace

TEST(StrictJson, AcceptsObjectWithWhitespaceAndAnyFieldOrder) {
  Value v;
  ASSERT_TRUE(ok(" \t\r\n{\"b\": 1 ,\n \"a\" :[true,false,null,\"x\"], \"c\":{}}\n ", &v));
  ASSERT_EQ(v.type, Type::kObject);
  ASSERT_EQ(v.object.size(), 3u);
  EXPECT_EQ(v.object[0].first, "b");  // 입력 순서 유지
  ASSERT_NE(v.find("a"), nullptr);
  EXPECT_EQ(v.find("a")->array.size(), 4u);
  EXPECT_EQ(v.find("missing"), nullptr);
}

TEST(StrictJson, DuplicateKeysRejectedAtAnyDepth) {
  EXPECT_FALSE(ok("{\"a\":1,\"a\":1}"));
  EXPECT_FALSE(ok("{\"a\":{\"b\":1,\"b\":2}}"));
  EXPECT_FALSE(ok("{\"a\":[{\"b\":1,\"b\":2}]}"));
  // escape 로 쓴 같은 키도 중복이다
  EXPECT_FALSE(ok("{\"a\":1,\"\\u0061\":2}"));
}

TEST(StrictJson, IntegerAndFloatTokensAreDistinguished) {
  Value v;
  ASSERT_TRUE(ok("{\"i\":2,\"f\":2.0,\"e\":2e0,\"n\":-0,\"E\":1E-05}", &v));
  EXPECT_EQ(v.find("i")->type, Type::kInteger);
  EXPECT_TRUE(v.find("i")->int_fits);
  EXPECT_EQ(v.find("i")->integer, 2);
  EXPECT_EQ(v.find("f")->type, Type::kFloat);
  EXPECT_EQ(v.find("e")->type, Type::kFloat);
  EXPECT_EQ(v.find("n")->type, Type::kInteger);
  EXPECT_EQ(v.find("n")->integer, 0);
  EXPECT_DOUBLE_EQ(v.find("E")->number, 1e-05);
}

TEST(StrictJson, NonStandardConstantsRejected) {
  for (const char* s : {"{\"a\":NaN}", "{\"a\":Infinity}", "{\"a\":-Infinity}", "{\"a\":nan}", "{\"a\":inf}",
                        "{\"a\":True}", "{\"a\":undefined}", "{'a':1}"}) {
    EXPECT_FALSE(ok(s)) << s;
  }
}

TEST(StrictJson, MalformedNumbersRejected) {
  for (const char* s : {"{\"a\":01}", "{\"a\":1.}", "{\"a\":.5}", "{\"a\":+1}", "{\"a\":-}", "{\"a\":1e}",
                        "{\"a\":1e+}", "{\"a\":0x10}", "{\"a\":1_000}", "{\"a\":--1}"}) {
    EXPECT_FALSE(ok(s)) << s;
  }
}

TEST(StrictJson, StructureErrorsRejected) {
  for (const char* s : {"", "{", "}", "{\"a\":1,}", "{\"a\":1}x", "{\"a\":1}{}", "{\"a\" 1}", "{a:1}",
                        "{\"a\":[1,2,]}", "{\"a\":[1 2]}", "{\"a\":\"unterminated}", "{1:2}"}) {
    EXPECT_FALSE(ok(s)) << s;
  }
}

TEST(StrictJson, Utf8Validation) {
  EXPECT_TRUE(ok("{\"이름\":\"로봇 ✓ 🚀\"}"));
  EXPECT_FALSE(ok(std::string("{\"a\":\"\xff\"}")));            // 잘못된 바이트
  EXPECT_FALSE(ok(std::string("{\"a\":\"\xc0\xaf\"}")));        // overlong
  EXPECT_FALSE(ok(std::string("{\"a\":\"\xe0\x80\xaf\"}")));    // overlong 3바이트
  EXPECT_FALSE(ok(std::string("{\"a\":\"\xed\xa0\x80\"}")));    // UTF-8 로 쓴 서로게이트
  EXPECT_FALSE(ok(std::string("{\"a\":\"\xe2\x82\"}")));        // 잘린 문자
  EXPECT_FALSE(ok(std::string("{\"a\":\"\xf4\x90\x80\x80\"}")));  // U+10FFFF 초과
  EXPECT_FALSE(ok(std::string("\xef\xbb\xbf{\"a\":1}")));       // BOM 은 JSON 공백이 아님
}

TEST(StrictJson, StringEscapesAndControlCharacters) {
  Value v;
  ASSERT_TRUE(ok("{\"a\":\"\\\"\\\\\\/\\b\\f\\n\\r\\t\\u00e9\\ud83d\\ude80\"}", &v));
  EXPECT_EQ(v.find("a")->string, "\"\\/\b\f\n\r\t\xc3\xa9\xf0\x9f\x9a\x80");
  EXPECT_FALSE(ok("{\"a\":\"\\x41\"}"));
  EXPECT_FALSE(ok("{\"a\":\"\\u12\"}"));
  EXPECT_FALSE(ok(std::string("{\"a\":\"tab\there\"}")));  // 날 제어 문자
  // Python 처럼 짝 없는 서로게이트 escape 는 허용(비교용 U+FFFD)
  ASSERT_TRUE(ok("{\"a\":\"\\ud800\"}", &v));
  EXPECT_EQ(v.find("a")->string, "\xef\xbf\xbd");
}

TEST(StrictJson, DepthLimit) {
  std::string deep;
  for (int i = 0; i < 40; ++i) deep += "[";
  for (int i = 0; i < 40; ++i) deep += "]";
  EXPECT_FALSE(ok("{\"a\":" + deep + "}"));
  EXPECT_TRUE(ok("{\"a\":[[[[1]]]]}"));
}

TEST(StrictJson, SizeLimitAndEmpty) {
  Value v;
  std::string err;
  EXPECT_FALSE(parse("", v, err));
  std::string big = "{\"a\":\"" + std::string(3000, 'x') + "\"}";
  EXPECT_FALSE(parse(big, v, err));
  ParseOptions large;
  large.max_bytes = 65536;
  EXPECT_TRUE(parse(big, v, err, large));
}

TEST(StrictJson, HugeIntegersAndExponents) {
  Value v;
  ASSERT_TRUE(ok("{\"big\":100000000000000000000000000000,\"max\":9223372036854775807,"
                 "\"over\":1e400,\"under\":1e-400,\"nunder\":-1e-400}",
                 &v));
  EXPECT_EQ(v.find("big")->type, Type::kInteger);
  EXPECT_FALSE(v.find("big")->int_fits);
  EXPECT_TRUE(v.find("big")->number_finite);  // 1e29 는 double 로 표현 가능
  EXPECT_TRUE(v.find("max")->int_fits);
  EXPECT_FALSE(v.find("over")->number_finite);
  EXPECT_TRUE(v.find("under")->number_finite);  // Python: float("1e-400") == 0.0
  EXPECT_EQ(v.find("under")->number, 0.0);
  EXPECT_TRUE(std::signbit(v.find("nunder")->number));
  std::string digits(400, '9');
  ASSERT_TRUE(ok("{\"a\":" + digits + "}", &v));
  EXPECT_FALSE(v.find("a")->int_fits);
  EXPECT_FALSE(v.find("a")->number_finite);
}

TEST(StrictJson, QuoteAndNumberSerialization) {
  using armpi_common::json::number;
  using armpi_common::json::quote;
  EXPECT_EQ(quote("a\"b\\c\n\x01"), "\"a\\\"b\\\\c\\n\\u0001\"");
  EXPECT_EQ(number(0.1), "0.1");
  EXPECT_EQ(number(-0.18), "-0.18");
  EXPECT_EQ(number(NAN), "null");
  EXPECT_EQ(number(INFINITY), "null");
  // 직렬화 결과를 다시 읽으면 같은 값
  Value v;
  ASSERT_TRUE(ok("{\"a\":" + number(0.123456789012345678) + "}", &v));
  EXPECT_EQ(v.find("a")->number, 0.123456789012345678);
}
