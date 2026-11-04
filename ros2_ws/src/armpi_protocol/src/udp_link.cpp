#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "armpi_protocol/datagram_link.hpp"

namespace armpi_protocol {

std::unique_ptr<UdpLink> UdpLink::open(const std::string& host, int port, std::string* error) {
  if (port < 1 || port > 65535) {
    if (error) *error = "포트 범위 오류";
    return nullptr;
  }
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  addrinfo* res = nullptr;
  const int rc = ::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res);
  if (rc != 0 || res == nullptr) {
    if (error) *error = std::string("주소 해석 실패: ") + ::gai_strerror(rc);
    return nullptr;
  }
  sockaddr_in addr{};
  std::memcpy(&addr, res->ai_addr, sizeof(addr));
  ::freeaddrinfo(res);
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    if (error) *error = std::string("socket: ") + std::strerror(errno);
    return nullptr;
  }
  if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
      ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK) != 0) {
    if (error) *error = std::string("connect: ") + std::strerror(errno);
    ::close(fd);
    return nullptr;
  }
  char ip[INET_ADDRSTRLEN] = {0};
  ::inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));
  return std::unique_ptr<UdpLink>(new UdpLink(fd, std::string(ip) + ":" + std::to_string(port)));
}

UdpLink::~UdpLink() {
  if (fd_ >= 0) ::close(fd_);
}

bool UdpLink::send(const std::string& data, std::string* error) {
  const ssize_t n = ::send(fd_, data.data(), data.size(), 0);
  if (n == static_cast<ssize_t>(data.size())) return true;
  if (error) *error = n < 0 ? std::strerror(errno) : "일부만 송신";
  return false;
}

std::optional<std::string> UdpLink::receive(std::string* error) {
  char buf[4096];
  const ssize_t n = ::recv(fd_, buf, sizeof(buf), 0);
  if (n >= 0) return std::string(buf, static_cast<size_t>(n));
  if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR && error) *error = std::strerror(errno);
  return std::nullopt;
}

}  // namespace armpi_protocol
