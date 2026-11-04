// 엄격한 JSON 해석기·작성 도우미 (RFC 8259 문법, 외부 의존 없음). ArmPi 노드들이 같이 쓴다.
//
// 외부 라이브러리를 쓰지 않는 이유:
//  - 정수 토큰(`2`)과 실수 토큰(`2.0`, `2e0`)을 어휘 단계에서 구분해야 한다(Protocol v3).
//  - 모든 깊이에서 중복 키를 거부해야 한다(Protocol v3).
//  - NaN·Infinity 같은 비표준 상수를 거부하고, 2^53-1 을 넘는 정수도 정확히 판별해야 한다.
//  - Python json.loads(+ object_pairs_hook 중복 키 검사)와 같은 수락/거부 결과를 내야 한다.
//
// 쓰는 곳과 옵션
//  - armpi_protocol(UDP 패킷·ACK): 기본값(2048 B, 중복 키 거부).
//  - armpi_control(ROS 문자열 토픽): kLenientTopic(64 KiB, 중복 키는 첫 값, 유한하지 않은 숫자 거부).
//    예전 armpi_control/mini_json 과 같은 수락 범위다.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace armpi_common::json {

enum class Type { kNull, kBool, kInteger, kFloat, kString, kArray, kObject };

struct Value {
  Type type{Type::kNull};
  bool boolean{false};
  // kInteger: int64 범위 안이면 int_fits=true 이고 integer 에 값이 있다.
  bool int_fits{false};
  int64_t integer{0};
  // kInteger·kFloat 의 double 값. 표현 불가(1e400 등)면 number_finite=false.
  double number{0.0};
  bool number_finite{true};
  std::string string;
  std::vector<Value> array;
  // 입력 순서 유지. 중복 키를 허용한 경우 같은 키가 여러 번 있을 수 있다.
  std::vector<std::pair<std::string, Value>> object;

  // 객체가 아니거나 키가 없으면 nullptr. 같은 키가 여럿이면 첫 번째.
  const Value* find(std::string_view key) const;

  bool is_null() const { return type == Type::kNull; }
  bool is_bool() const { return type == Type::kBool; }
  bool is_number() const { return type == Type::kInteger || type == Type::kFloat; }
  bool is_string() const { return type == Type::kString; }
  bool is_array() const { return type == Type::kArray; }
  bool is_object() const { return type == Type::kObject; }
};

struct ParseOptions {
  std::size_t max_bytes{2048};
  int max_depth{16};
  bool reject_duplicate_keys{true};
  bool reject_non_finite{false};  // true: 1e400 처럼 double 로 표현할 수 없는 숫자를 오류로
};

// ROS 문자열 토픽(내부 상태 JSON)용 관대한 설정.
inline ParseOptions lenient_topic_options(std::size_t max_bytes = 65536, int max_depth = 16) {
  ParseOptions o;
  o.max_bytes = max_bytes;
  o.max_depth = max_depth;
  o.reject_duplicate_keys = false;
  o.reject_non_finite = true;
  return o;
}

// 성공하면 true. 실패하면 error 에 한국어 설명을 넣고 false.
bool parse(std::string_view text, Value& out, std::string& error, const ParseOptions& options = {});

// 엄격한 UTF-8 검사(overlong·서로게이트·U+10FFFF 초과 거부). Python bytes.decode("utf-8") 과 같다.
bool valid_utf8(std::string_view text);

// ── 작성 ──

// JSON 문자열 리터럴로 직렬화(따옴표 포함).
std::string quote(std::string_view text);

// 유한한 double 을 가장 짧은 왕복 가능 표기로. 유한하지 않으면 "null".
std::string number(double value);

// 소수점 아래 decimals 자리 고정 표기. −0.000 은 0.000 으로. 유한하지 않으면 "null".
std::string fixed(double value, int decimals);

inline std::string boolean(bool b) { return b ? "true" : "false"; }

}  // namespace armpi_common::json
