#include "armpi_vr_bridge/udp_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <system_error>

namespace armpi_vr_bridge {

double monotonic_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

namespace {

int open_udp(const std::string& bind_address, uint16_t port, uint16_t& bound) {
  uint32_t ip = 0;
  if (!parse_ipv4(bind_address, ip)) throw std::invalid_argument("bind_address 가 잘못되었습니다: " + bind_address);
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) throw std::system_error(errno, std::generic_category(), "UDP 소켓 생성 실패");
  // SO_REUSEADDR 금지: Linux 에서 두 서버가 같은 UDP 포트를 나눠 받게 된다.
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(ip);
  addr.sin_port = htons(port);
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    const int e = errno;
    ::close(fd);
    throw std::system_error(e, std::generic_category(),
                            "UDP " + bind_address + ":" + std::to_string(port) + " bind 실패");
  }
  sockaddr_in got{};
  socklen_t len = sizeof(got);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&got), &len) == 0) {
    bound = ntohs(got.sin_port);
  } else {
    bound = port;
  }
  return fd;
}

sockaddr_in to_sockaddr(const Endpoint& e) {
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(e.ip);
  a.sin_port = htons(e.port);
  return a;
}

Endpoint from_sockaddr(const sockaddr_in& a) { return Endpoint{ntohl(a.sin_addr.s_addr), ntohs(a.sin_port)}; }

// 정지·비상정지·fault·차단·만료는 WARN, 나머지(조종자 획득·중립 확인 등 일상 전이)는 INFO.
LogLevel event_level(const std::string& e) {
  for (const char* key : {"만료", "비상정지", "fault", "차단", "지연", "정지", "오류"}) {
    if (e.find(key) != std::string::npos) return LogLevel::kWarn;
  }
  return LogLevel::kInfo;
}

std::string join(const std::vector<std::string>& v) {
  std::string s;
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i) s += "; ";
    s += v[i];
  }
  return s;
}

}  // namespace

// ───────────────────────── UdpServer ─────────────────────────

UdpServer::UdpServer(const SessionConfig& session_config, const ServerConfig& config, SourceFilter filter,
                     ServerCallbacks callbacks, ClockFn clock)
    : session_(session_config), cfg_(config), filter_(std::move(filter)), cb_(std::move(callbacks)),
      clock_(std::move(clock)) {
  if (!(cfg_.poll_interval_s > 0.0) || !(cfg_.stall_timeout_s > cfg_.poll_interval_s)) {
    throw std::invalid_argument("poll_interval 은 0보다 크고 rx_stall_timeout 보다 작아야 합니다");
  }
}

UdpServer::~UdpServer() { shutdown(); }

void UdpServer::start() {
  if (running_.load()) return;
  fd_ = open_udp(cfg_.bind_address, cfg_.port, bound_port_);
  rx_exited_ = false;
  heartbeat_ = clock_();
  running_ = true;
  rx_thread_ = std::thread(&UdpServer::rx_loop, this);
  monitor_thread_ = std::thread(&UdpServer::monitor_loop, this);
  log(LogLevel::kInfo, "Protocol v3 UDP 수신: " + cfg_.bind_address + ":" + std::to_string(bound_port_) +
                           " (allowed_sources: " + filter_.describe() + ")");
}

void UdpServer::shutdown() {
  const bool was_running = running_.exchange(false);
  if (rx_thread_.joinable()) rx_thread_.join();
  if (monitor_thread_.joinable()) monitor_thread_.join();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  if (was_running) {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      session_.force_stop();
    }
    if (cb_.on_stop) {
      try {
        cb_.on_stop("서버 종료");
      } catch (...) {
      }
    }
  }
}

std::vector<std::string> UdpServer::check_timeouts() {
  std::vector<std::string> ev;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    ev = session_.check_timeouts(clock_());
  }
  if (!ev.empty()) {
    for (const auto& e : ev) log(event_level(e), e);
    if (cb_.on_stop) {
      try {
        cb_.on_stop(join(ev));
      } catch (const std::exception& e) {
        log(LogLevel::kError, std::string("정지 출력 콜백 실패: ") + e.what());
      }
    }
  }
  return ev;
}

std::vector<std::string> UdpServer::set_control_state(const ControlState& state) {
  std::vector<std::string> ev;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    ev = session_.set_control_state(state, clock_());
  }
  if (!ev.empty()) {
    for (const auto& e : ev) log(event_level(e), e);
    if (cb_.on_stop) {
      try {
        cb_.on_stop(join(ev));
      } catch (const std::exception& e) {
        log(LogLevel::kError, std::string("정지 출력 콜백 실패: ") + e.what());
      }
    }
  }
  return ev;
}

RxSummary UdpServer::stats() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return stats_.summary(clock_());
}

void UdpServer::rx_loop() {
  std::vector<char> buf(65536);  // UDP 최대 크기. 2048 초과 여부를 잘리지 않은 길이로 판정한다.
  const int poll_ms = std::max(1, static_cast<int>(std::lround(cfg_.poll_interval_s * 1000.0)));
  while (running_.load()) {
    heartbeat_ = clock_();
    // 한 패킷의 예외가 이 스레드를 끝내면 watchdog 도 멈추므로 반복마다 보호한다.
    try {
      pollfd pfd{fd_, POLLIN, 0};
      const int n = ::poll(&pfd, 1, poll_ms);
      if (!running_.load()) break;
      if (n < 0) {
        if (errno != EINTR) {
          log(LogLevel::kWarn, std::string("UDP poll 오류: ") + std::strerror(errno));
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      } else if (n > 0) {
        sockaddr_in src{};
        socklen_t len = sizeof(src);
        const ssize_t got =
            ::recvfrom(fd_, buf.data(), buf.size(), MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&src), &len);
        if (got >= 0) {
          const Endpoint from = from_sockaddr(src);
          if (!filter_.allows(from.ip)) {
            std::lock_guard<std::mutex> lk(mutex_);
            stats_.count_filtered();
          } else {
            handle(std::string_view(buf.data(), static_cast<std::size_t>(got)), from);
          }
        } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          log(LogLevel::kWarn, std::string("UDP 수신 오류: ") + std::strerror(errno));
        }
      }
    } catch (const std::exception& e) {
      log(LogLevel::kError, std::string("패킷 처리 중 예기치 않은 오류: ") + e.what());
      stop_session("패킷 처리 오류", false);
    } catch (...) {
      log(LogLevel::kError, "패킷 처리 중 예기치 않은 오류");
      stop_session("패킷 처리 오류", false);
    }
    guarded_check();
  }
  rx_exited_ = true;
}

void UdpServer::monitor_loop() {
  const double period = std::min(0.1, cfg_.stall_timeout_s / 3.0);
  bool stalled = false;
  while (running_.load()) {
    std::this_thread::sleep_for(std::chrono::duration<double>(period));
    if (!running_.load()) break;  // 정상 종료 중
    const double age = clock_() - heartbeat_.load();
    if (rx_exited_.load()) {
      // 수신 스레드가 끝났으면 재시작 전까지 이동을 막는다(fault latch).
      if (!stalled) {
        stalled = true;
        log(LogLevel::kError, "수신 스레드 종료 감지; fault latch");
        stop_session("수신 스레드 종료", true);
      }
    } else if (age > cfg_.stall_timeout_s) {
      // 일시 지연(CPU 부족 등): 정지 후 새 중립을 요구하고 계속 운영한다.
      if (!stalled) {
        stalled = true;
        char buf[96];
        std::snprintf(buf, sizeof(buf), "수신 루프 %.3fs 지연; 안전 정지", age);
        log(LogLevel::kError, buf);
        stop_session("수신 루프 지연", false);
      }
    } else {
      stalled = false;
    }
  }
}

void UdpServer::handle(std::string_view data, const Endpoint& from) {
  HandleResult r;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    const double now = clock_();
    r = session_.handle_datagram(data, from, now);
    stats_.observe(now, data.size(), r.ack, r.arm_adjust);
  }
  // 안전 우선: 비상정지 요청 → 출력 → ACK 순서.
  if (r.estop_change && cb_.on_estop) {
    try {
      cb_.on_estop(*r.estop_change);
    } catch (const std::exception& e) {
      log(LogLevel::kError, std::string("비상정지 요청 전달 실패: ") + e.what());
    }
  }
  if (cb_.on_packet) {
    try {
      cb_.on_packet(r);
    } catch (const std::exception& e) {
      log(LogLevel::kError, std::string("출력 콜백 실패: ") + e.what());
    }
  }
  send_to(from, encode_ack(r.ack));
  for (const auto& e : r.events) log(event_level(e), e);
  if (!r.ack.accepted) {
    const double now = clock_();
    if (now - last_reject_log_ >= 1.0) {  // 고주파 거부 로그 제한(초당 1회)
      last_reject_log_ = now;
      log(LogLevel::kWarn, "v3 패킷 거부 " + from.to_string() + ": " + to_string(r.ack.reason) + " (" +
                               (r.detail.empty() ? "-" : r.detail) + ")");
    }
  }
}

void UdpServer::guarded_check() {
  try {
    check_timeouts();
  } catch (const std::exception& e) {
    log(LogLevel::kError, std::string("watchdog 검사 오류: ") + e.what());
    stop_session("watchdog 검사 오류", false);
  } catch (...) {
    stop_session("watchdog 검사 오류", false);
  }
}

void UdpServer::stop_session(const std::string& why, bool fault) {
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (fault) {
      session_.enter_fault(why);
    } else {
      session_.force_stop();
    }
  }
  if (cb_.on_stop) {
    try {
      cb_.on_stop(why);
    } catch (...) {
    }
  }
}

void UdpServer::send_to(const Endpoint& to, const std::string& payload) {
  if (fd_ < 0) return;
  const sockaddr_in a = to_sockaddr(to);
  // 실패(버퍼 부족 등)는 무시한다. 클라이언트는 ACK timeout 으로 스스로 멈춘다.
  (void)::sendto(fd_, payload.data(), payload.size(), MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&a),
                 sizeof(a));
}

void UdpServer::log(LogLevel level, const std::string& message) const {
  if (cb_.log) {
    try {
      cb_.log(level, message);
    } catch (...) {
    }
  }
}

// ───────────────────────── DiscoveryServer ─────────────────────────

DiscoveryServer::DiscoveryServer(DiscoveryResponder responder, const DiscoveryServerConfig& config,
                                 SourceFilter filter, LogFn log, ClockFn clock)
    : responder_(std::move(responder)), cfg_(config), filter_(std::move(filter)), log_(std::move(log)),
      clock_(std::move(clock)) {}

DiscoveryServer::~DiscoveryServer() { shutdown(); }

void DiscoveryServer::start() {
  if (running_.load()) return;
  fd_ = open_udp(cfg_.bind_address, cfg_.port, bound_port_);
  running_ = true;
  thread_ = std::thread(&DiscoveryServer::loop, this);
}

void DiscoveryServer::shutdown() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void DiscoveryServer::loop() {
  std::vector<char> buf(2048);
  while (running_.load()) {
    try {
      pollfd pfd{fd_, POLLIN, 0};
      const int n = ::poll(&pfd, 1, 100);
      if (!running_.load()) break;
      if (n <= 0) continue;
      sockaddr_in src{};
      socklen_t len = sizeof(src);
      const ssize_t got =
          ::recvfrom(fd_, buf.data(), buf.size(), MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&src), &len);
      if (got < 0) continue;
      const Endpoint from = from_sockaddr(src);
      if (!filter_.allows(from.ip)) {
        ++filtered_;
        continue;
      }
      const auto reply = responder_.handle(std::string_view(buf.data(), static_cast<std::size_t>(got)), clock_());
      if (!reply) continue;
      // 같은 소켓으로 보내므로 응답 출발 포트 = 검색 포트(5002).
      if (::sendto(fd_, reply->data(), reply->size(), MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&src),
                   sizeof(src)) >= 0) {
        ++replies_sent_;
      }
    } catch (const std::exception& e) {
      if (log_) log_(LogLevel::kWarn, std::string("검색 응답 오류: ") + e.what());
    }
  }
}

}  // namespace armpi_vr_bridge
