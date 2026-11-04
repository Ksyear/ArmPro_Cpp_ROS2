#include "armpi_common/json.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <set>
#include <system_error>

namespace armpi_common::json {

const Value* Value::find(std::string_view key) const {
  for (const auto& kv : object) {
    if (kv.first == key) return &kv.second;
  }
  return nullptr;
}

bool valid_utf8(std::string_view text) {
  const auto* s = reinterpret_cast<const unsigned char*>(text.data());
  const std::size_t n = text.size();
  std::size_t i = 0;
  while (i < n) {
    const unsigned char c = s[i];
    if (c < 0x80) {
      ++i;
      continue;
    }
    std::size_t len = 0;
    uint32_t cp = 0;
    if (c >= 0xC2 && c <= 0xDF) {
      len = 2;
      cp = c & 0x1F;
    } else if (c >= 0xE0 && c <= 0xEF) {
      len = 3;
      cp = c & 0x0F;
    } else if (c >= 0xF0 && c <= 0xF4) {
      len = 4;
      cp = c & 0x07;
    } else {
      return false;  // 0x80-0xC1(연속/overlong 시작), 0xF5 이상
    }
    if (i + len > n) return false;
    for (std::size_t k = 1; k < len; ++k) {
      const unsigned char cc = s[i + k];
      if ((cc & 0xC0) != 0x80) return false;
      cp = (cp << 6) | (cc & 0x3F);
    }
    if ((len == 3 && cp < 0x800) || (len == 4 && cp < 0x10000)) return false;  // overlong
    if (cp >= 0xD800 && cp <= 0xDFFF) return false;                            // 서로게이트
    if (cp > 0x10FFFF) return false;
    i += len;
  }
  return true;
}

namespace {

struct Failure {
  const char* message;
  std::size_t offset;
};

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

bool is_digit(char c) { return c >= '0' && c <= '9'; }

class Parser {
 public:
  Parser(std::string_view s, const ParseOptions& options) : s_(s), opt_(options) {}

  Value document() {
    skip_ws();
    Value v = value(0);
    skip_ws();
    if (pos_ != s_.size()) fail("JSON 값 뒤에 남은 문자가 있습니다");
    return v;
  }

 private:
  [[noreturn]] void fail(const char* message) const { throw Failure{message, pos_}; }

  bool eof() const { return pos_ >= s_.size(); }
  char peek() const { return s_[pos_]; }

  void skip_ws() {
    // JSON 공백은 공백·탭·LF·CR 네 가지뿐이다(BOM 불가).
    while (!eof()) {
      const char c = peek();
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        ++pos_;
      } else {
        break;
      }
    }
  }

  void expect_literal(std::string_view lit) {
    if (s_.substr(pos_, lit.size()) != lit) fail("알 수 없는 리터럴(NaN·Infinity 등 불가)");
    pos_ += lit.size();
  }

  Value value(int depth) {
    if (eof()) fail("값이 필요합니다");
    const char c = peek();
    Value v;
    switch (c) {
      case '{':
        return object(depth + 1);
      case '[':
        return array(depth + 1);
      case '"':
        v.type = Type::kString;
        v.string = string();
        return v;
      case 't':
        expect_literal("true");
        v.type = Type::kBool;
        v.boolean = true;
        return v;
      case 'f':
        expect_literal("false");
        v.type = Type::kBool;
        v.boolean = false;
        return v;
      case 'n':
        expect_literal("null");
        v.type = Type::kNull;
        return v;
      default:
        if (c == '-' || is_digit(c)) return number();
        fail("알 수 없는 값(NaN·Infinity·작은따옴표 등 불가)");
    }
  }

  Value object(int depth) {
    if (depth > opt_.max_depth) fail("중첩이 너무 깊습니다");
    ++pos_;  // '{'
    Value v;
    v.type = Type::kObject;
    std::set<std::string> keys;
    skip_ws();
    if (!eof() && peek() == '}') {
      ++pos_;
      return v;
    }
    while (true) {
      skip_ws();
      if (eof() || peek() != '"') fail("객체 키는 문자열이어야 합니다");
      std::string key = string();
      if (!keys.insert(key).second && opt_.reject_duplicate_keys) fail("중복 JSON 키");
      skip_ws();
      if (eof() || peek() != ':') fail("':' 가 필요합니다");
      ++pos_;
      skip_ws();
      Value member = value(depth);
      v.object.emplace_back(std::move(key), std::move(member));
      skip_ws();
      if (eof()) fail("객체가 닫히지 않았습니다");
      if (peek() == ',') {
        ++pos_;
        continue;
      }
      if (peek() == '}') {
        ++pos_;
        return v;
      }
      fail("',' 또는 '}' 가 필요합니다");
    }
  }

  Value array(int depth) {
    if (depth > opt_.max_depth) fail("중첩이 너무 깊습니다");
    ++pos_;  // '['
    Value v;
    v.type = Type::kArray;
    skip_ws();
    if (!eof() && peek() == ']') {
      ++pos_;
      return v;
    }
    while (true) {
      skip_ws();
      v.array.push_back(value(depth));
      skip_ws();
      if (eof()) fail("배열이 닫히지 않았습니다");
      if (peek() == ',') {
        ++pos_;
        continue;
      }
      if (peek() == ']') {
        ++pos_;
        return v;
      }
      fail("',' 또는 ']' 가 필요합니다");
    }
  }

  uint32_t hex4() {
    if (pos_ + 4 > s_.size()) fail("\\u 뒤에 16진수 4자리가 필요합니다");
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
      const char c = s_[pos_++];
      v <<= 4;
      if (c >= '0' && c <= '9') {
        v |= static_cast<uint32_t>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        v |= static_cast<uint32_t>(c - 'a' + 10);
      } else if (c >= 'A' && c <= 'F') {
        v |= static_cast<uint32_t>(c - 'A' + 10);
      } else {
        fail("\\u 뒤에 16진수 4자리가 필요합니다");
      }
    }
    return v;
  }

  std::string string() {
    ++pos_;  // '"'
    std::string out;
    while (true) {
      if (eof()) fail("문자열이 닫히지 않았습니다");
      const char c = s_[pos_];
      if (c == '"') {
        ++pos_;
        return out;
      }
      if (static_cast<unsigned char>(c) < 0x20) fail("문자열 안 제어 문자");
      if (c != '\\') {
        out.push_back(c);  // UTF-8 은 parse() 앞에서 전체 검사했다
        ++pos_;
        continue;
      }
      ++pos_;
      if (eof()) fail("잘못된 escape");
      const char e = s_[pos_++];
      switch (e) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
          uint32_t cp = hex4();
          if (cp >= 0xD800 && cp <= 0xDBFF && s_.substr(pos_, 2) == "\\u") {
            const std::size_t save = pos_;
            pos_ += 2;
            const uint32_t lo = hex4();
            if (lo >= 0xDC00 && lo <= 0xDFFF) {
              cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
            } else {
              pos_ = save;  // 짝이 아니면 두 번째 \u 는 다음 반복에서 따로 처리
            }
          }
          // Python 은 짝 없는 서로게이트를 허용한다. 비교용 값으로만 쓰므로 U+FFFD 로 바꾼다.
          if (cp >= 0xD800 && cp <= 0xDFFF) cp = 0xFFFD;
          append_utf8(out, cp);
          break;
        }
        default:
          fail("잘못된 escape");
      }
    }
  }

  Value number() {
    const std::size_t start = pos_;
    bool is_float = false;
    if (peek() == '-') ++pos_;
    if (eof() || !is_digit(peek())) fail("'-' 뒤에 숫자가 필요합니다");
    if (peek() == '0') {
      ++pos_;
      if (!eof() && is_digit(peek())) fail("앞자리 0 은 허용되지 않습니다");
    } else {
      while (!eof() && is_digit(peek())) ++pos_;
    }
    if (!eof() && peek() == '.') {
      is_float = true;
      ++pos_;
      if (eof() || !is_digit(peek())) fail("소수점 뒤에 숫자가 필요합니다");
      while (!eof() && is_digit(peek())) ++pos_;
    }
    if (!eof() && (peek() == 'e' || peek() == 'E')) {
      is_float = true;
      ++pos_;
      if (!eof() && (peek() == '+' || peek() == '-')) ++pos_;
      if (eof() || !is_digit(peek())) fail("지수에 숫자가 필요합니다");
      while (!eof() && is_digit(peek())) ++pos_;
    }
    const std::string_view tok = s_.substr(start, pos_ - start);
    Value v;
    v.type = is_float ? Type::kFloat : Type::kInteger;
    const char* first = tok.data();
    const char* last = tok.data() + tok.size();
    if (!is_float) {
      int64_t iv = 0;
      const auto r = std::from_chars(first, last, iv);
      v.int_fits = (r.ec == std::errc() && r.ptr == last);
      v.integer = v.int_fits ? iv : 0;
    }
    double dv = 0.0;
    const auto r = std::from_chars(first, last, dv, std::chars_format::general);
    if (r.ec == std::errc() && r.ptr == last && std::isfinite(dv)) {
      v.number = dv;
      v.number_finite = true;
    } else if (r.ec == std::errc::result_out_of_range) {
      // 아주 작은 값(1e-400)은 Python 처럼 ±0.0, 아주 큰 값(1e400)은 유한하지 않음으로 둔다.
      if (decimal_order(tok) < 0) {
        v.number = (tok.front() == '-') ? -0.0 : 0.0;
        v.number_finite = true;
      } else {
        v.number = 0.0;
        v.number_finite = false;
      }
    } else {
      v.number = 0.0;
      v.number_finite = false;
    }
    if (!v.number_finite && opt_.reject_non_finite) fail("double 로 표현할 수 없는 숫자");
    return v;
  }

  // 토큰의 10진 자릿수 차수(첫 유효 숫자 위치 + 지수). 범위 초과 시 큰지 작은지 판정에만 쓴다.
  static long long decimal_order(std::string_view tok) {
    std::size_t i = 0;
    if (i < tok.size() && tok[i] == '-') ++i;
    std::size_t int_begin = i;
    while (i < tok.size() && is_digit(tok[i])) ++i;
    std::string_view int_part = tok.substr(int_begin, i - int_begin);
    std::string_view frac_part;
    if (i < tok.size() && tok[i] == '.') {
      ++i;
      const std::size_t fb = i;
      while (i < tok.size() && is_digit(tok[i])) ++i;
      frac_part = tok.substr(fb, i - fb);
    }
    long long exp = 0;
    if (i < tok.size() && (tok[i] == 'e' || tok[i] == 'E')) {
      ++i;
      bool neg = false;
      if (i < tok.size() && (tok[i] == '+' || tok[i] == '-')) {
        neg = tok[i] == '-';
        ++i;
      }
      for (; i < tok.size(); ++i) {
        if (exp < 1000000000LL) exp = exp * 10 + (tok[i] - '0');
      }
      if (neg) exp = -exp;
    }
    if (!(int_part.size() == 1 && int_part[0] == '0')) {
      return static_cast<long long>(int_part.size()) - 1 + exp;
    }
    for (std::size_t k = 0; k < frac_part.size(); ++k) {
      if (frac_part[k] != '0') return -static_cast<long long>(k) - 1 + exp;
    }
    return -1;  // 0 (범위 초과가 날 수 없음)
  }

  std::string_view s_;
  const ParseOptions& opt_;
  std::size_t pos_{0};
};

}  // namespace

bool parse(std::string_view text, Value& out, std::string& error, const ParseOptions& options) {
  if (text.empty()) {
    error = "빈 데이터";
    return false;
  }
  if (text.size() > options.max_bytes) {
    error = "크기 제한 초과";
    return false;
  }
  if (!valid_utf8(text)) {
    error = "UTF-8 이 아닙니다";
    return false;
  }
  try {
    Parser p(text, options);
    out = p.document();
    return true;
  } catch (const Failure& f) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), " (위치 %zu)", f.offset);
    error = std::string(f.message) + buf;
    return false;
  }
}

std::string quote(std::string_view text) {
  std::string out;
  out.reserve(text.size() + 2);
  out.push_back('"');
  for (const char ch : text) {
    const auto c = static_cast<unsigned char>(ch);
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out.push_back(ch);
        }
    }
  }
  out.push_back('"');
  return out;
}

std::string number(double value) {
  if (!std::isfinite(value)) return "null";
  char buf[64];
  const auto r = std::to_chars(buf, buf + sizeof(buf), value);
  if (r.ec != std::errc()) return "null";
  return std::string(buf, r.ptr);
}

std::string fixed(double value, int decimals) {
  if (!std::isfinite(value)) return "null";
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
  std::string s(buf);
  // −0.0000 처럼 0 으로만 이뤄진 음수 표기는 부호를 뗀다.
  if (!s.empty() && s[0] == '-' && s.find_first_not_of("-0.") == std::string::npos) s.erase(0, 1);
  return s;
}

}  // namespace armpi_common::json
