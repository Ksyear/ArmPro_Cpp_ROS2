#include "armpi_web_gateway/http.hpp"

#include <arpa/inet.h>

#include <algorithm>
#include <cctype>
#include <sstream>

namespace armpi_web_gateway {

std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

std::string trim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

std::string HttpRequest::header(const std::string& lower_name) const {
  const auto it = headers.find(lower_name);
  return it == headers.end() ? std::string() : it->second;
}

const char* reason_phrase(int code) {
  switch (code) {
    case 200: return "OK";
    case 302: return "Found";
    case 400: return "Bad Request";
    case 403: return "Forbidden";
    case 404: return "Not Found";
    case 409: return "Conflict";
    case 413: return "Payload Too Large";
    case 501: return "Not Implemented";
    case 503: return "Service Unavailable";
    default: return "Unknown";
  }
}

std::string HttpResponse::serialize() const {
  std::ostringstream o;
  o << "HTTP/1.1 " << code << " " << reason_phrase(code) << "\r\n";
  o << "Content-Type: " << content_type << "\r\n";
  o << "Content-Length: " << body.size() << "\r\n";
  o << "Cache-Control: " << (cache ? "max-age=86400" : "no-store") << "\r\n";
  for (const auto& kv : extra_headers) o << kv.first << ": " << kv.second << "\r\n";
  o << "Connection: close\r\n\r\n";
  if (!head_only) o << body;
  return o.str();
}

std::optional<HttpRequest> parse_request_head(const std::string& head) {
  HttpRequest r;
  std::istringstream in(head);
  std::string line;
  if (!std::getline(in, line)) return std::nullopt;
  if (!line.empty() && line.back() == '\r') line.pop_back();
  std::istringstream rl(line);
  std::string target, version;
  if (!(rl >> r.method >> target >> version) || version.rfind("HTTP/1.", 0) != 0 || target.empty() ||
      target[0] != '/') {
    return std::nullopt;
  }
  const auto q = target.find('?');
  r.path = target.substr(0, q);
  r.query = q == std::string::npos ? "" : target.substr(q + 1);
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) break;
    const auto c = line.find(':');
    if (c == std::string::npos || c == 0) return std::nullopt;
    const std::string name = to_lower(trim(line.substr(0, c)));
    if (r.headers.count(name) == 0) r.headers[name] = trim(line.substr(c + 1));  // 같은 이름은 첫 값
  }
  return r;
}

std::optional<std::pair<std::string, std::optional<int>>> split_host(const std::string& value) {
  const std::string h = to_lower(trim(value));
  if (h.empty()) return std::nullopt;
  std::string name, port;
  bool has_port = false;
  if (h[0] == '[') {  // [IPv6]:port
    const auto end = h.find(']');
    if (end == std::string::npos) return std::nullopt;
    name = h.substr(1, end - 1);
    const std::string rest = h.substr(end + 1);
    if (!rest.empty() && rest[0] != ':') return std::nullopt;
    if (!rest.empty()) {
      has_port = true;
      port = rest.substr(1);
    }
  } else {
    const auto c = h.rfind(':');
    if (c == std::string::npos) {
      name = h;
    } else {
      name = h.substr(0, c);
      port = h.substr(c + 1);
      has_port = true;
    }
  }
  if (!has_port) return std::make_pair(name, std::optional<int>());
  if (port.empty() || port.size() > 5 || !std::all_of(port.begin(), port.end(), ::isdigit)) return std::nullopt;
  return std::make_pair(name, std::optional<int>(std::stoi(port)));
}

bool is_ip_literal(const std::string& name) {
  unsigned char buf[16];
  return ::inet_pton(AF_INET, name.c_str(), buf) == 1 || ::inet_pton(AF_INET6, name.c_str(), buf) == 1;
}

namespace {
std::string url_decode(const std::string& s) {
  std::string out;
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '+') {
      out.push_back(' ');
    } else if (s[i] == '%' && i + 2 < s.size() && std::isxdigit(static_cast<unsigned char>(s[i + 1])) &&
               std::isxdigit(static_cast<unsigned char>(s[i + 2]))) {
      out.push_back(static_cast<char>(std::stoi(s.substr(i + 1, 2), nullptr, 16)));
      i += 2;
    } else {
      out.push_back(s[i]);
    }
  }
  return out;
}
}  // namespace

std::optional<std::string> query_param(const std::string& query, const std::string& key) {
  size_t i = 0;
  while (i <= query.size()) {
    const auto amp = query.find_first_of("&;", i);
    const std::string part = query.substr(i, amp == std::string::npos ? std::string::npos : amp - i);
    const auto eq = part.find('=');
    if (eq != std::string::npos && url_decode(part.substr(0, eq)) == key) {
      const std::string v = url_decode(part.substr(eq + 1));
      if (!v.empty()) return v;  // parse_qs 는 빈 값을 버린다
    }
    if (amp == std::string::npos) break;
    i = amp + 1;
  }
  return std::nullopt;
}

}  // namespace armpi_web_gateway
