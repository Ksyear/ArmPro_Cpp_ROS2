// 입력 해석 시험: 파이썬 float()·int()·bool()·화면 토큰 규칙과 같은 결과
#include <gtest/gtest.h>

#include "armpi_web_gateway/input.hpp"

using namespace armpi_web_gateway;

namespace {
json::Value P(const std::string& s) {
  json::Value v;
  std::string e;
  EXPECT_TRUE(json::parse(s, v, e, json::lenient_topic_options())) << e;
  return v;
}
}  // namespace

TEST(Input, PythonConversions) {
  EXPECT_FALSE(pyconv::to_float(nullptr));
  const auto v = P("[1, 2.5, true, \" -3e1 \", \"0x10\", \"abc\", null, [1]]");
  EXPECT_DOUBLE_EQ(*pyconv::to_float(&v.array[0]), 1.0);
  EXPECT_DOUBLE_EQ(*pyconv::to_float(&v.array[1]), 2.5);
  EXPECT_DOUBLE_EQ(*pyconv::to_float(&v.array[2]), 1.0);
  EXPECT_DOUBLE_EQ(*pyconv::to_float(&v.array[3]), -30.0);
  EXPECT_FALSE(pyconv::to_float(&v.array[4]));
  EXPECT_FALSE(pyconv::to_float(&v.array[5]));
  EXPECT_FALSE(pyconv::to_float(&v.array[6]));
  EXPECT_FALSE(pyconv::to_float(&v.array[7]));
  const auto i = P("[1.9, -1.9, \" 2 \", \"1.0\", false]");
  EXPECT_EQ(*pyconv::to_int(&i.array[0]), 1);
  EXPECT_EQ(*pyconv::to_int(&i.array[1]), -1);
  EXPECT_EQ(*pyconv::to_int(&i.array[2]), 2);
  EXPECT_FALSE(pyconv::to_int(&i.array[3]));
  EXPECT_EQ(*pyconv::to_int(&i.array[4]), 0);
  const auto t = P("[0, 0.0, 2, \"\", \"x\", [], {}, {\"a\":1}, null, false]");
  const bool expect[] = {false, false, true, false, true, false, false, true, false, false};
  for (size_t k = 0; k < t.array.size(); ++k) EXPECT_EQ(pyconv::truthy(&t.array[k]), expect[k]) << k;
}

TEST(Input, ScreenIdAndName) {
  const auto v = P("[\"phone_1-A\", 42, true, 1.5, \"bad id\", \"" + std::string(65, 'a') + "\", null]");
  EXPECT_EQ(screen_id(&v.array[0]), "phone_1-A");
  EXPECT_EQ(screen_id(&v.array[1]), "42");
  EXPECT_EQ(screen_id(&v.array[2]), "True");
  EXPECT_EQ(screen_id(&v.array[3]), "api");
  EXPECT_EQ(screen_id(&v.array[4]), "api");
  EXPECT_EQ(screen_id(&v.array[5]), "api");
  EXPECT_EQ(screen_id(nullptr), "api");
  const auto n = P("[\"  내 폰\\u0007\\n \", \"" + std::string(50, 'x') + "\", 0, \"\\u200b숨김\"]");
  EXPECT_EQ(screen_name(&n.array[0]), "내 폰");
  EXPECT_EQ(screen_name(&n.array[1]), std::string(40, 'x'));
  EXPECT_EQ(screen_name(&n.array[2]), "");
  EXPECT_EQ(screen_name(&n.array[3]), "숨김");
}

TEST(Input, ParseInputClampsAndDefaults) {
  const auto in = parse_input(P("{\"f\":3,\"s\":\"-0.5\",\"t\":null,\"held\":1,\"focused\":\"yes\","
                                "\"arm\":{\"x\":5,\"y\":-1.7,\"z\":\"1\",\"p\":\"x\"}}"));
  EXPECT_DOUBLE_EQ(in.f, 1.0);
  EXPECT_DOUBLE_EQ(in.s, -0.5);
  EXPECT_DOUBLE_EQ(in.t, 0.0);
  EXPECT_TRUE(in.held);
  EXPECT_TRUE(in.focused);
  EXPECT_EQ(in.arm, (std::array<int, 5>{1, -1, 1, 0, 0}));
  const auto none = parse_input(P("{\"arm\":[1,2]}"));
  EXPECT_FALSE(none.any_arm());
  EXPECT_FALSE(none.held);
  EXPECT_EQ(input_counter(P("{\"n\":7}")), 7);
  EXPECT_FALSE(input_counter(P("{\"n\":7.0}")));
  EXPECT_FALSE(input_counter(P("{\"n\":true}")));
}
