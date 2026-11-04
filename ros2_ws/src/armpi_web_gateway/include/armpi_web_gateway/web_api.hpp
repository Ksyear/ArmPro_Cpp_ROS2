// 웹 조종기 HTTP 경로(파이썬 조종기 make_handler 와 같은 규칙). 소켓과 분리돼 있어 요청 객체로 바로 시험한다.
//  GET  /, /index.html, /mobile, /mobile.html, 아이콘·manifest   화면 파일(www)
//  GET  /api/config, /api/state?screen=<토큰>
//  POST /api/input, /api/cmd                                      JSON 본문, 같은 출처만
// 보안: Host 검사(DNS rebinding 방지), POST 는 Origin = http://<Host> 이고 Content-Type: application/json 일 때만(CSRF).
#pragma once

#include <set>
#include <string>
#include <vector>

#include "armpi_web_gateway/controller.hpp"
#include "armpi_web_gateway/http.hpp"

namespace armpi_web_gateway {

struct WebApiOptions {
  int http_port{18765};
  bool public_bind{true};                // 127.* 가 아닌 주소로 열었는가
  std::string hostname;                  // 이 기기 이름(public 일 때 Host 로 허용)
  std::vector<std::string> extra_hosts;  // 추가로 받을 Host 이름(armpi.local 등)
  std::string www_dir;                   // 화면 파일 폴더
  std::string config_json;               // /api/config 응답
};

class WebApi {
 public:
  WebApi(Controller& controller, WebApiOptions options);
  HttpResponse handle(const HttpRequest& req);
  bool host_ok(const std::string& host_header) const;

 private:
  HttpResponse get(const HttpRequest& req);
  HttpResponse post(const HttpRequest& req);
  static HttpResponse json_response(const std::string& body, int code = 200);

  Controller& ctl_;
  WebApiOptions opt_;
  std::set<std::string> host_names_;
};

}  // namespace armpi_web_gateway
