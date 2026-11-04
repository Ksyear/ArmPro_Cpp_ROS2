// 화면 요청 JSON 해석. 파이썬 조종기(clients/mac_controller/armpi_mac_controller.py)의 float()·int()·bool()
// 변환 규칙을 그대로 따라 같은 입력에 같은 결과를 낸다.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "armpi_common/json.hpp"

namespace armpi_web_gateway {

namespace json = armpi_common::json;

// 브라우저 입력 한 번(/api/input)
struct InputState {
  double f{0.0};  // 앞뒤 −1..1
  double s{0.0};  // 옆 −1..1
  double t{0.0};  // 회전 −1..1
  bool held{false};
  bool focused{false};
  std::array<int, 5> arm{0, 0, 0, 0, 0};  // x, y, z, pitch, gripper 방향 −1/0/1
  bool any_arm() const { return arm[0] || arm[1] || arm[2] || arm[3] || arm[4]; }
};

namespace pyconv {
std::optional<double> to_float(const json::Value* v);   // float(x): 숫자·불리언·숫자 문자열
std::optional<int64_t> to_int(const json::Value* v);    // int(x): 정수·실수(0 쪽으로 버림)·불리언·정수 문자열
bool truthy(const json::Value* v);                      // bool(x): null·false·0·빈 문자열/배열/객체는 거짓
}  // namespace pyconv

// 화면 토큰: [A-Za-z0-9_-]{1,64} 가 아니면 "api"
std::string screen_id(const json::Value* v);
// 화면 이름: 출력 가능한 문자만, 앞뒤 공백 제거, 40자
std::string screen_name(const json::Value* v);
// set_input 의 축·팔 방향·held·focused
InputState parse_input(const json::Value& req);
// "n"(입력 번호): JSON 정수만, 그 밖은 nullopt
std::optional<int64_t> input_counter(const json::Value& req);

}  // namespace armpi_web_gateway
