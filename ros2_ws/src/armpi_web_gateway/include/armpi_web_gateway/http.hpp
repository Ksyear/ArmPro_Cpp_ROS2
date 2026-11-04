// 아주 작은 HTTP/1.1 요청·응답 형식(웹 조종기 전용). 소켓과 분리해 단위 시험한다.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <utility>

namespace armpi_web_gateway {

constexpr size_t kMaxHeadBytes = 16384;
constexpr size_t kMaxBodyBytes = 8192;

struct HttpRequest {
  std::string method;                          // GET, HEAD, POST ...
  std::string path;                            // ? 앞
  std::string query;                           // ? 뒤(디코딩 전)
  std::map<std::string, std::string> headers;  // 이름은 소문자
  std::string body;
  bool body_too_large{false};                  // Content-Length 가 kMaxBodyBytes 초과(본문은 읽지 않음)
  std::string client_addr;                     // 요청한 IP
  std::string header(const std::string& lower_name) const;
};

struct HttpResponse {
  int code{200};
  std::string content_type{"text/plain; charset=utf-8"};
  std::string body;
  bool cache{false};                                     // max-age=86400, 아니면 no-store
  std::map<std::string, std::string> extra_headers;      // Location 등
  bool head_only{false};                                  // HEAD: 본문 생략
  std::string serialize() const;
};

const char* reason_phrase(int code);

// 요청 머리(요청줄 + 헤더, 빈 줄 앞까지)를 해석한다. 틀리면 nullopt.
std::optional<HttpRequest> parse_request_head(const std::string& head);

// Host 헤더 → (이름 소문자, 포트). 형식이 틀리면 nullopt. 포트가 없으면 second = nullopt.
std::optional<std::pair<std::string, std::optional<int>>> split_host(const std::string& value);
bool is_ip_literal(const std::string& name);

// query 에서 key 의 첫 값(%xx·+ 디코딩). 없으면 nullopt.
std::optional<std::string> query_param(const std::string& query, const std::string& key);

std::string to_lower(std::string s);
std::string trim(const std::string& s);

}  // namespace armpi_web_gateway
