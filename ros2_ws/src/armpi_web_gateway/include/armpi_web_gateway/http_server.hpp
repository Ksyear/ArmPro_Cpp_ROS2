// 작은 HTTP/1.1 서버(POSIX 소켓, 외부 라이브러리 없음). 연결마다 요청 1개를 처리하고 닫는다(Connection: close).
// 처리 내용은 Handler(WebApi::handle)가 정한다. 동시 연결 수와 송수신 시간을 제한한다.
#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>

#include "armpi_web_gateway/http.hpp"

namespace armpi_web_gateway {

class HttpServer {
 public:
  using Handler = std::function<HttpResponse(const HttpRequest&)>;
  HttpServer(std::string bind_addr, int port, Handler handler, int max_connections = 32, int io_timeout_s = 5);
  ~HttpServer();
  HttpServer(const HttpServer&) = delete;
  HttpServer& operator=(const HttpServer&) = delete;

  bool start(std::string* error);  // bind·listen 실패면 false
  void stop();                     // 받기 중지, 처리 중인 연결은 최대 2 s 기다림
  int port() const { return port_; }  // port 0 으로 열었으면 실제 포트

 private:
  void accept_loop();
  void serve(int fd, std::string peer);

  std::string bind_addr_;
  int port_;
  Handler handler_;
  int max_connections_;
  int io_timeout_s_;
  int listen_fd_{-1};
  int wake_pipe_[2]{-1, -1};
  std::atomic<bool> running_{false};
  std::atomic<int> active_{0};
  std::thread accept_thread_;
};

}  // namespace armpi_web_gateway
