#include "armpi_web_gateway/input.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <vector>

namespace armpi_web_gateway {

namespace {

std::string trim_ascii_space(const std::string& s) {
  const auto b = s.find_first_not_of(" \t\n\r\f\v");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\n\r\f\v");
  return s.substr(b, e - b + 1);
}

// Python float(str): 앞뒤 공백 허용, 16진수 불가, 전체가 숫자여야 함
std::optional<double> parse_float_text(const std::string& raw) {
  const std::string s = trim_ascii_space(raw);
  if (s.empty() || s.find_first_of("xX") != std::string::npos) return std::nullopt;
  errno = 0;
  char* end = nullptr;
  const double d = std::strtod(s.c_str(), &end);
  if (end != s.c_str() + s.size()) return std::nullopt;
  return d;
}

// Python int(str): 앞뒤 공백, 부호 허용, 10진 정수만
std::optional<int64_t> parse_int_text(const std::string& raw) {
  const std::string s = trim_ascii_space(raw);
  size_t i = (!s.empty() && (s[0] == '+' || s[0] == '-')) ? 1 : 0;
  if (i >= s.size()) return std::nullopt;
  for (size_t k = i; k < s.size(); ++k) {
    if (s[k] < '0' || s[k] > '9') return std::nullopt;
  }
  errno = 0;
  const long long v = std::strtoll(s.c_str(), nullptr, 10);
  if (errno == ERANGE) return std::nullopt;
  return static_cast<int64_t>(v);
}

// UTF-8 → 코드 포인트(잘못된 바이트는 버림: 입력은 이미 엄격한 JSON 해석을 통과했다)
std::vector<uint32_t> decode_utf8(const std::string& s) {
  std::vector<uint32_t> out;
  for (size_t i = 0; i < s.size();) {
    const auto c = static_cast<unsigned char>(s[i]);
    int len = c < 0x80 ? 1 : (c >> 5) == 0x6 ? 2 : (c >> 4) == 0xE ? 3 : (c >> 3) == 0x1E ? 4 : 0;
    if (len == 0 || i + len > s.size()) {
      ++i;
      continue;
    }
    uint32_t cp = len == 1 ? c : len == 2 ? (c & 0x1F) : len == 3 ? (c & 0x0F) : (c & 0x07);
    for (int k = 1; k < len; ++k) cp = (cp << 6) | (static_cast<unsigned char>(s[i + k]) & 0x3F);
    out.push_back(cp);
    i += len;
  }
  return out;
}

void append_utf8(std::string& out, uint32_t cp) {
  if (cp < 0x80) {
    out.push_back(static_cast<char>(cp));
  } else if (cp < 0x800) {
    out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else if (cp < 0x10000) {
    out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else {
    out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  }
}

// Python str.isprintable() 의 근사: 제어 문자·서식 문자·줄/문단 구분·공백이 아닌 구분 문자·사용자 영역을 뺀다.
bool printable(uint32_t cp) {
  if (cp < 0x20 || (cp >= 0x7F && cp <= 0xA0) || cp == 0xAD) return false;
  if ((cp >= 0x600 && cp <= 0x605) || cp == 0x1680 || (cp >= 0x2000 && cp <= 0x200F)) return false;
  if ((cp >= 0x2028 && cp <= 0x202F) || (cp >= 0x205F && cp <= 0x2064) || cp == 0x3000 || cp == 0xFEFF) return false;
  if ((cp >= 0xD800 && cp <= 0xF8FF) || cp > 0x10FFFF) return false;
  return true;
}

// Python str(value) 중 이 화면에 나올 수 있는 것: 문자열·정수·불리언
std::optional<std::string> py_str(const json::Value* v) {
  if (v == nullptr || v->is_null()) return std::nullopt;
  if (v->is_string()) return v->string;
  if (v->is_bool()) return std::string(v->boolean ? "True" : "False");
  if (v->type == json::Type::kInteger && v->int_fits) return std::to_string(v->integer);
  return std::nullopt;
}

}  // namespace

namespace pyconv {

std::optional<double> to_float(const json::Value* v) {
  if (v == nullptr) return std::nullopt;
  if (v->is_bool()) return v->boolean ? 1.0 : 0.0;
  if (v->type == json::Type::kInteger) return v->int_fits ? static_cast<double>(v->integer) : v->number;
  if (v->type == json::Type::kFloat) return v->number_finite ? v->number : HUGE_VAL;
  if (v->is_string()) return parse_float_text(v->string);
  return std::nullopt;
}

std::optional<int64_t> to_int(const json::Value* v) {
  if (v == nullptr) return std::nullopt;
  if (v->is_bool()) return v->boolean ? 1 : 0;
  if (v->type == json::Type::kInteger) return v->int_fits ? std::optional<int64_t>(v->integer) : std::nullopt;
  if (v->type == json::Type::kFloat) {
    if (!v->number_finite || !std::isfinite(v->number) || std::fabs(v->number) > 9.2e18) return std::nullopt;
    return static_cast<int64_t>(std::trunc(v->number));
  }
  if (v->is_string()) return parse_int_text(v->string);
  return std::nullopt;
}

bool truthy(const json::Value* v) {
  if (v == nullptr) return false;
  switch (v->type) {
    case json::Type::kNull: return false;
    case json::Type::kBool: return v->boolean;
    case json::Type::kInteger:
    case json::Type::kFloat: return v->number != 0.0 || (v->type == json::Type::kInteger && v->integer != 0);
    case json::Type::kString: return !v->string.empty();
    case json::Type::kArray: return !v->array.empty();
    case json::Type::kObject: return !v->object.empty();
  }
  return false;
}

}  // namespace pyconv

std::string screen_id(const json::Value* v) {
  const auto s = py_str(v);
  if (!s || s->empty() || s->size() > 64) return "api";
  for (const char c : *s) {
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
    if (!ok) return "api";
  }
  return *s;
}

std::string screen_name(const json::Value* v) {
  if (!pyconv::truthy(v)) return "";  // str(value or "")
  const auto s = py_str(v);
  if (!s) return "";
  std::vector<uint32_t> cps;
  for (uint32_t cp : decode_utf8(*s)) {
    if (printable(cp)) cps.push_back(cp);
  }
  size_t b = 0, e = cps.size();
  while (b < e && cps[b] == ' ') ++b;
  while (e > b && cps[e - 1] == ' ') --e;
  std::string out;
  for (size_t i = b; i < e && i - b < 40; ++i) append_utf8(out, cps[i]);
  return out;
}

InputState parse_input(const json::Value& req) {
  InputState in;
  auto axis = [&](const char* key) {
    const json::Value* v = req.find(key);
    if (v == nullptr) return 0.0;  // req.get(key, 0.0)
    const auto d = pyconv::to_float(v);
    if (!d || !std::isfinite(*d)) return 0.0;
    return std::max(-1.0, std::min(1.0, *d));
  };
  in.f = axis("f");
  in.s = axis("s");
  in.t = axis("t");
  const json::Value* arm = req.find("arm");
  const char* keys[5] = {"x", "y", "z", "p", "g"};
  for (int i = 0; i < 5; ++i) {
    int64_t d = 0;
    if (arm != nullptr && arm->is_object()) {
      const json::Value* v = arm->find(keys[i]);
      if (v != nullptr) d = pyconv::to_int(v).value_or(0);
    }
    in.arm[i] = static_cast<int>(std::max<int64_t>(-1, std::min<int64_t>(1, d)));
  }
  in.held = pyconv::truthy(req.find("held"));
  in.focused = pyconv::truthy(req.find("focused"));
  return in;
}

std::optional<int64_t> input_counter(const json::Value& req) {
  const json::Value* n = req.find("n");
  if (n == nullptr || n->type != json::Type::kInteger || !n->int_fits) return std::nullopt;
  return n->integer;
}

}  // namespace armpi_web_gateway
