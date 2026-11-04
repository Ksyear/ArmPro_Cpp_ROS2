#include "armpi_web_gateway/http_server.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>

namespace armpi_web_gateway {

namespace {

bool send_all(int fd, const std::string& data) {
  size_t off = 0;
  while (off < data.size()) {
    const ssize_t n = ::send(fd, data.data() + off, data.size() - off, MSG_NOSIGNAL);
    if (n <= 0) {
      if (n < 0 && errno == EINTR) continue;
      return false;
    }
    off += static_cast<size_t>(n);
  }
  return true;
}

HttpResponse make_response(int code, const std::string& body, const char* type = "text/plain; charset=utf-8") {
  HttpResponse r;
  r.code = code;
  r.content_type = type;
  r.body = body;
  return r;
}

}  // namespace

HttpServer::HttpServer(std::string bind_addr, int port, Handler handler, int max_connections, int io_timeout_s)
    : bind_addr_(std::move(bind_addr)),
      port_(port),
      handler_(std::move(handler)),
      max_connections_(max_connections),
      io_timeout_s_(io_timeout_s) {}

HttpServer::~HttpServer() { stop(); }

bool HttpServer::start(std::string* error) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port_));
  if (::inet_pton(AF_INET, bind_addr_.c_str(), &addr.sin_addr) != 1) {
    if (error) *error = "IPv4 주소가 아님: " + bind_addr_;
    return false;
  }
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (listen_fd_ < 0) {
    if (error) *error = std::string("socket: ") + std::strerror(errno);
    return false;
  }
  const int one = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || ::listen(listen_fd_, 32) != 0) {
    if (error) *error = std::string("bind/listen ") + bind_addr_ + ":" + std::to_string(port_) + ": " + std::strerror(errno);
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  socklen_t len = sizeof(addr);
  if (::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len) == 0) port_ = ntohs(addr.sin_port);
  if (::pipe2(wake_pipe_, O_CLOEXEC) != 0) {
    if (error) *error = std::string("pipe: ") + std::strerror(errno);
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  running_ = true;
  accept_thread_ = std::thread([this] { accept_loop(); });
  return true;
}

void HttpServer::stop() {
  if (!running_.exchange(false)) return;
  const char b = 1;
  if (::write(wake_pipe_[1], &b, 1) < 0) {
  }
  if (accept_thread_.joinable()) accept_thread_.join();
  ::close(listen_fd_);
  listen_fd_ = -1;
  ::close(wake_pipe_[0]);
  ::close(wake_pipe_[1]);
  // 처리 중인 연결(분리 스레드)이 끝나기를 잠시 기다린다(각 연결은 송수신 제한 시간이 있다).
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (active_ > 0 && std::chrono::steady_clock::now() < until) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void HttpServer::accept_loop() {
  while (running_) {
    pollfd fds[2] = {{listen_fd_, POLLIN, 0}, {wake_pipe_[0], POLLIN, 0}};
    if (::poll(fds, 2, -1) < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (fds[1].revents) break;
    if (!(fds[0].revents & POLLIN)) continue;
    sockaddr_in peer{};
    socklen_t len = sizeof(peer);
    const int fd = ::accept4(listen_fd_, reinterpret_cast<sockaddr*>(&peer), &len, SOCK_CLOEXEC);
    if (fd < 0) continue;
    if (active_ >= max_connections_) {  // 너무 많으면 바로 닫는다
      ::close(fd);
      continue;
    }
    char ip[INET_ADDRSTRLEN] = {0};
    ::inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
    ++active_;
    std::thread([this, fd, p = std::string(ip)] {
      serve(fd, p);
      --active_;
    }).detach();
  }
}

void HttpServer::serve(int fd, std::string peer) {
  timeval tv{io_timeout_s_, 0};
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  std::string buf;
  size_t head_end = std::string::npos;
  char chunk[4096];
  while (head_end == std::string::npos) {
    const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
    if (n <= 0) {
      ::close(fd);
      return;
    }
    buf.append(chunk, static_cast<size_t>(n));
    head_end = buf.find("\r\n\r\n");
    if (head_end == std::string::npos && buf.size() > kMaxHeadBytes) {
      send_all(fd, make_response(400, "400 요청 머리가 너무 큼").serialize());
      ::close(fd);
      return;
    }
  }
  auto req = parse_request_head(buf.substr(0, head_end + 2));
  if (!req) {
    send_all(fd, make_response(400, "400 잘못된 요청").serialize());
    ::close(fd);
    return;
  }
  req->client_addr = peer;
  std::string body = buf.substr(head_end + 4);
  long length = 0;
  const std::string cl = req->header("content-length");
  if (!cl.empty()) {
    char* end = nullptr;
    length = std::strtol(cl.c_str(), &end, 10);
    if (*end != '\0' || length < 0) length = -1;
  }
  if (length < 0) {
    send_all(fd, make_response(400, "400 Content-Length 오류").serialize());
    ::close(fd);
    return;
  }
  if (static_cast<size_t>(length) > kMaxBodyBytes) {
    req->body_too_large = true;  // 본문은 읽지 않고 처리기가 400 을 낸다
  } else {
    while (body.size() < static_cast<size_t>(length)) {
      const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
      if (n <= 0) {
        ::close(fd);
        return;
      }
      body.append(chunk, static_cast<size_t>(n));
    }
    req->body = body.substr(0, static_cast<size_t>(length));
  }
  HttpResponse res;
  try {
    res = handler_(*req);
  } catch (const std::exception&) {
    res = make_response(503, "{\"ok\":false,\"error\":\"처리 오류\"}", "application/json; charset=utf-8");
  }
  send_all(fd, res.serialize());
  ::shutdown(fd, SHUT_WR);
  ::close(fd);
}

}  // namespace armpi_web_gateway
