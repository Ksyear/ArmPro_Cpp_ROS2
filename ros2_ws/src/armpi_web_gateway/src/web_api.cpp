#include "armpi_web_gateway/web_api.hpp"

#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <tuple>

namespace armpi_web_gateway {

namespace {

struct StaticFile {
  const char* file;
  const char* type;
  bool cache;
};

const std::map<std::string, StaticFile>& static_files() {
  static const std::map<std::string, StaticFile> m = {
      {"/", {"index.html", "text/html; charset=utf-8", false}},
      {"/index.html", {"index.html", "text/html; charset=utf-8", false}},
      {"/mobile", {"mobile.html", "text/html; charset=utf-8", false}},
      {"/mobile.html", {"mobile.html", "text/html; charset=utf-8", false}},
      {"/manifest.webmanifest", {"manifest.webmanifest", "application/manifest+json; charset=utf-8", false}},
      {"/icon-192.png", {"icon-192.png", "image/png", true}},
      {"/icon-512.png", {"icon-512.png", "image/png", true}},
      {"/apple-touch-icon.png", {"icon-192.png", "image/png", true}},
      {"/favicon.ico", {"icon-192.png", "image/png", true}},
  };
  return m;
}

bool read_file(const std::string& path, std::string* out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  *out = ss.str();
  return true;
}

HttpResponse plain(int code, const std::string& text) {
  HttpResponse r;
  r.code = code;
  r.body = text;
  return r;
}

}  // namespace

WebApi::WebApi(Controller& controller, WebApiOptions options) : ctl_(controller), opt_(std::move(options)) {
  host_names_ = {"127.0.0.1", "localhost", "::1"};
  if (opt_.public_bind) {
    // 0.0.0.0 등으로 열었을 때: IP 주소로 연 화면과 이 기기의 이름(armpi-ros2.local 등)으로 연 화면을 받는다.
    const std::string hn = to_lower(opt_.hostname);
    if (!hn.empty()) {
      const std::string first = hn.substr(0, hn.find('.'));
      host_names_.insert({hn, first, first + ".local"});
    }
    for (const auto& h : opt_.extra_hosts) host_names_.insert(to_lower(h));
  }
}

bool WebApi::host_ok(const std::string& host_header) const {
  // DNS rebinding 방지: 이름이 IP 주소이거나 이 기기의 이름일 때만 받는다(공격자 도메인 이름은 거부).
  const auto hp = split_host(host_header);
  if (!hp) return false;
  const auto& [name, port] = *hp;
  const bool port_ok = port ? *port == opt_.http_port : opt_.http_port == 80;
  if (!port_ok) return false;
  return host_names_.count(name) > 0 || is_ip_literal(name);
}

HttpResponse WebApi::json_response(const std::string& body, int code) {
  HttpResponse r;
  r.code = code;
  r.content_type = "application/json; charset=utf-8";
  r.body = body;
  return r;
}

HttpResponse WebApi::handle(const HttpRequest& req) {
  HttpResponse r;
  if (req.method == "GET" || req.method == "HEAD") {
    r = get(req);
    r.head_only = req.method == "HEAD";
  } else if (req.method == "POST") {
    r = post(req);
  } else {
    r = plain(501, "501 지원하지 않는 메서드");
  }
  return r;
}

HttpResponse WebApi::get(const HttpRequest& req) {
  if (!host_ok(req.header("host"))) return json_response("{\"ok\":false,\"error\":\"host 거부\"}", 403);
  static const std::regex kMobileUa("Android|iPhone|iPad|iPod|Mobile", std::regex::icase);
  if (req.path == "/" && std::regex_search(req.header("user-agent"), kMobileUa)) {
    // 휴대폰 브라우저로 주소만 치고 들어오면 터치 화면으로 보낸다(PC 화면은 /index.html).
    HttpResponse r = plain(302, "");
    r.extra_headers["Location"] = "/mobile";
    return r;
  }
  const auto& files = static_files();
  if (const auto it = files.find(req.path); it != files.end()) {
    HttpResponse r;
    if (!read_file(opt_.www_dir + "/" + it->second.file, &r.body)) return plain(404, "404 없음");
    r.content_type = it->second.type;
    r.cache = it->second.cache;
    return r;
  }
  if (req.path == "/api/config") return json_response(opt_.config_json);
  if (req.path == "/api/state") {
    const auto sid = query_param(req.query, "screen");
    json::Value v;
    v.type = json::Type::kString;
    v.string = sid.value_or("");
    return json_response(ctl_.state_json(sid ? screen_id(&v) : "api"));
  }
  return plain(404, "404 없음");
}

HttpResponse WebApi::post(const HttpRequest& req) {
  // 다른 웹사이트가 이 조종기로 명령을 보내지 못하게(CSRF): JSON 본문 + 같은 출처(Origin = http://<Host>)만.
  const auto origin = req.headers.find("origin");
  const std::string ctype = to_lower(trim(req.header("content-type").substr(0, req.header("content-type").find(';'))));
  const bool same_origin =
      origin == req.headers.end() || to_lower(origin->second) == "http://" + to_lower(trim(req.header("host")));
  if (!host_ok(req.header("host")) || !same_origin || ctype != "application/json") {
    return json_response("{\"ok\":false,\"error\":\"요청 출처·형식 거부\"}", 403);
  }
  if (req.body_too_large) return json_response("{\"ok\":false,\"error\":\"bad json: 본문이 너무 큼\"}", 400);
  json::Value body;
  std::string err;
  const std::string text = req.body.empty() ? "{}" : req.body;
  if (!json::parse(text, body, err, json::lenient_topic_options(kMaxBodyBytes))) {
    return json_response("{\"ok\":false,\"error\":" + json::quote("bad json: " + err) + "}", 400);
  }
  if (!body.is_object()) return json_response("{\"ok\":false,\"error\":\"bad json: 객체 아님\"}", 400);
  const Screen s{screen_id(body.find("screen")), screen_name(body.find("screen_name")), req.client_addr};
  if (req.path == "/api/input") return json_response(ctl_.set_input(body, s));
  if (req.path == "/api/cmd") {
    const CommandResult r = ctl_.command(body, s);
    return json_response(r.to_json(), r.http_code());
  }
  return plain(404, "404 없음");
}

}  // namespace armpi_web_gateway
