// WebApi 시험: Host 검사, CSRF(Origin·Content-Type), 화면 파일, /api 경로와 응답 코드.
#include <gtest/gtest.h>

#include "armpi_web_gateway/web_api.hpp"
#include "fakes.hpp"

using namespace gwtest;
using armpi_web_gateway::HttpRequest;
using armpi_web_gateway::WebApi;
using armpi_web_gateway::WebApiOptions;

namespace {

struct ApiRig {
  Rig rig;
  std::unique_ptr<WebApi> api;
  ApiRig() {
    WebApiOptions o;
    o.http_port = 18765;
    o.public_bind = true;
    o.hostname = "armpi-ros2";
    o.extra_hosts = {"robot.lab"};
    o.www_dir = ARMPI_WWW_DIR;
    o.config_json = "{\"robot\":\"127.0.0.1\"}";
    api = std::make_unique<WebApi>(*rig.ctl, o);
  }
  static HttpRequest get(const std::string& path, const std::string& host = "192.168.0.5:18765") {
    HttpRequest r;
    r.method = "GET";
    const auto q = path.find('?');
    r.path = path.substr(0, q);
    r.query = q == std::string::npos ? "" : path.substr(q + 1);
    r.headers["host"] = host;
    r.client_addr = "192.168.0.10";
    return r;
  }
  static HttpRequest post(const std::string& path, const std::string& body, const std::string& host = "192.168.0.5:18765") {
    HttpRequest r = get(path, host);
    r.method = "POST";
    r.headers["content-type"] = "application/json";
    r.headers["origin"] = "http://" + host;
    r.body = body;
    return r;
  }
};

}  // namespace

TEST(WebApi, HostCheck) {
  ApiRig a;
  EXPECT_TRUE(a.api->host_ok("127.0.0.1:18765"));
  EXPECT_TRUE(a.api->host_ok("ARMPI-ROS2.local:18765"));
  EXPECT_TRUE(a.api->host_ok("armpi-ros2:18765"));
  EXPECT_TRUE(a.api->host_ok("robot.lab:18765"));
  EXPECT_TRUE(a.api->host_ok("[::1]:18765"));
  EXPECT_TRUE(a.api->host_ok("10.1.2.3:18765"));
  EXPECT_FALSE(a.api->host_ok("evil.example:18765"));  // DNS rebinding
  EXPECT_FALSE(a.api->host_ok("127.0.0.1:80"));
  EXPECT_FALSE(a.api->host_ok("127.0.0.1"));            // 포트 생략은 80 일 때만
  EXPECT_FALSE(a.api->host_ok(""));
  EXPECT_EQ(a.api->handle(ApiRig::get("/api/config", "evil.example:18765")).code, 403);
}

TEST(WebApi, StaticFilesAndMobileRedirect) {
  ApiRig a;
  auto r = ApiRig::get("/");
  r.headers["user-agent"] = "Mozilla/5.0 (Linux; Android 13; SM-G781N)";
  const auto red = a.api->handle(r);
  EXPECT_EQ(red.code, 302);
  EXPECT_EQ(red.extra_headers.at("Location"), "/mobile");
  const auto m = a.api->handle(ApiRig::get("/mobile"));
  EXPECT_EQ(m.code, 200);
  EXPECT_EQ(m.content_type, "text/html; charset=utf-8");
  EXPECT_NE(m.body.find("<html"), std::string::npos);
  const auto icon = a.api->handle(ApiRig::get("/favicon.ico"));
  EXPECT_EQ(icon.content_type, "image/png");
  EXPECT_TRUE(icon.cache);
  auto head = ApiRig::get("/index.html");
  head.method = "HEAD";
  const auto h = a.api->handle(head);
  EXPECT_TRUE(h.head_only);
  EXPECT_EQ(h.serialize().find("<html"), std::string::npos);  // 본문 없음
  EXPECT_EQ(a.api->handle(ApiRig::get("/nothing")).code, 404);
}

TEST(WebApi, ConfigAndState) {
  ApiRig a;
  EXPECT_EQ(a.api->handle(ApiRig::get("/api/config")).body, "{\"robot\":\"127.0.0.1\"}");
  const auto s = a.api->handle(ApiRig::get("/api/state?screen=abc_1"));
  EXPECT_EQ(s.code, 200);
  const auto v = Rig::parse(s.body);
  ASSERT_TRUE(v.is_object());
  EXPECT_FALSE(v.find("control")->find("you")->boolean);
}

TEST(WebApi, PostRequiresSameOriginJson) {
  ApiRig a;
  auto bad_origin = ApiRig::post("/api/input", "{}");
  bad_origin.headers["origin"] = "http://evil.example";
  EXPECT_EQ(a.api->handle(bad_origin).code, 403);
  auto form = ApiRig::post("/api/input", "{}");
  form.headers["content-type"] = "text/plain";
  EXPECT_EQ(a.api->handle(form).code, 403);
  auto no_origin = ApiRig::post("/api/input", "{\"screen\":\"A\",\"held\":true,\"focused\":true}");
  no_origin.headers.erase("origin");  // 같은 기기 도구(curl 등)는 Origin 이 없다
  EXPECT_EQ(a.api->handle(no_origin).body, "{\"ok\":true,\"control\":true}");
  auto charset = ApiRig::post("/api/input", "{}");
  charset.headers["content-type"] = "Application/JSON; charset=utf-8";
  EXPECT_EQ(a.api->handle(charset).code, 200);
}

TEST(WebApi, PostBodyErrorsAndCommandCodes) {
  ApiRig a;
  auto big = ApiRig::post("/api/cmd", "");
  big.body_too_large = true;
  EXPECT_EQ(a.api->handle(big).code, 400);
  EXPECT_EQ(a.api->handle(ApiRig::post("/api/cmd", "{bad")).code, 400);
  const auto arr = a.api->handle(ApiRig::post("/api/cmd", "[1]"));
  EXPECT_EQ(arr.code, 400);
  EXPECT_NE(arr.body.find("객체 아님"), std::string::npos);
  EXPECT_EQ(a.api->handle(ApiRig::post("/api/cmd", "{\"cmd\":\"nope\"}")).code, 400);
  EXPECT_EQ(a.api->handle(ApiRig::post("/api/cmd", "{\"cmd\":\"take_control\",\"screen\":\"A\"}")).code, 200);
  EXPECT_EQ(a.api->handle(ApiRig::post("/api/cmd", "{\"cmd\":\"clear\",\"screen\":\"B\"}")).code, 409);
  EXPECT_EQ(a.api->handle(ApiRig::post("/api/cmd", "{\"cmd\":\"estop\",\"screen\":\"B\"}")).code, 200);
  EXPECT_EQ(a.api->handle(ApiRig::post("/api/other", "{}")).code, 404);
  auto put = ApiRig::get("/api/cmd");
  put.method = "PUT";
  EXPECT_EQ(a.api->handle(put).code, 501);
}
