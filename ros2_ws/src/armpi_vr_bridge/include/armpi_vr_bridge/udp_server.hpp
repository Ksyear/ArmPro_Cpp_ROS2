// Protocol v3 UDP 서버(5001)와 검색 응답 서버(5002). ROS 비의존(POSIX 소켓 + std::thread).
//
// - 데이터그램 1개 = UTF-8 JSON 객체 1개. ACK 는 같은 소켓으로 보낸 주소·포트에 응답한다.
// - 수신 스레드 하나가 패킷 처리와 watchdog 검사를 한다. 세션은 mutex 로 보호하고,
//   콜백(출력 발행·estop 요청)은 lock 밖에서 부른다(콜백이 막혀도 감시 스레드가 세션을 멈출 수 있게).
// - 감시 스레드는 수신 루프가 stall_timeout(기본 0.5 s) 넘게 돌지 않으면 정지시키고(새 중립 요구),
//   수신 스레드가 끝나 버렸으면 fault latch 한다.
// - allowed_sources 에 없는 송신 주소의 패킷은 응답 없이 버린다.
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "armpi_vr_bridge/discovery.hpp"
#include "armpi_vr_bridge/net.hpp"
#include "armpi_vr_bridge/rx_stats.hpp"
#include "armpi_vr_bridge/session.hpp"

namespace armpi_vr_bridge {

enum class LogLevel { kInfo, kWarn, kError };
using LogFn = std::function<void(LogLevel, const std::string&)>;
using ClockFn = std::function<double()>;

// std::chrono::steady_clock 초
double monotonic_seconds();

struct ServerConfig {
  std::string bind_address{"0.0.0.0"};
  uint16_t port{5001};  // 0 = 임의 포트(시험용)
  double poll_interval_s{0.02};
  double stall_timeout_s{0.5};
};

struct ServerCallbacks {
  // 패킷 처리 직후(수신 스레드, 세션 lock 밖). 출력 발행에 쓴다.
  std::function<void(const HandleResult&)> on_packet;
  // estop_request 발행(true = latch, false = 해제 요청). on_packet 보다 먼저 부른다.
  std::function<void(bool)> on_estop;
  // 시간 만료·감시 정지 등으로 출력이 바뀌었을 수 있을 때(수신·감시·외부 스레드).
  std::function<void(const std::string&)> on_stop;
  LogFn log;
};

class UdpServer {
 public:
  UdpServer(const SessionConfig& session_config, const ServerConfig& config, SourceFilter filter,
            ServerCallbacks callbacks, ClockFn clock = monotonic_seconds);
  ~UdpServer();
  UdpServer(const UdpServer&) = delete;
  UdpServer& operator=(const UdpServer&) = delete;

  // 소켓을 열고 스레드를 시작한다. bind 실패 시 std::system_error (SO_REUSEADDR 없음: 중복 실행이 드러난다).
  void start();
  // 스레드를 멈추고 소켓을 닫은 뒤 세션을 정지시킨다(새 중립 요구). 여러 번 불러도 된다.
  void shutdown();
  bool running() const { return running_.load(); }
  uint16_t bound_port() const { return bound_port_; }

  // 세션에 lock 을 잡고 접근한다(조회·외부 입력).
  template <typename F>
  auto with_session(F&& f) {
    std::lock_guard<std::mutex> lk(mutex_);
    return f(session_);
  }

  // 외부(ROS 타이머)에서 만료 검사. 만료가 있으면 on_stop 도 부른다.
  std::vector<std::string> check_timeouts();
  // armpi_control 상태 반영. 전이가 있으면 on_stop 도 부른다.
  std::vector<std::string> set_control_state(const ControlState& state);

  RxSummary stats() const;
  double now() const { return clock_(); }

 private:
  void rx_loop();
  void monitor_loop();
  void handle(std::string_view data, const Endpoint& from);
  void guarded_check();
  void stop_session(const std::string& why, bool fault);
  void send_to(const Endpoint& to, const std::string& payload);
  void log(LogLevel level, const std::string& message) const;

  Session session_;
  mutable std::mutex mutex_;  // session_, stats_
  RxStats stats_;
  ServerConfig cfg_;
  SourceFilter filter_;
  ServerCallbacks cb_;
  ClockFn clock_;
  int fd_{-1};
  uint16_t bound_port_{0};
  std::atomic<bool> running_{false};
  std::atomic<double> heartbeat_{0.0};
  std::atomic<bool> rx_exited_{false};
  std::thread rx_thread_;
  std::thread monitor_thread_;
  double last_reject_log_{-1e9};  // 수신 스레드 전용
};

struct DiscoveryServerConfig {
  std::string bind_address{"0.0.0.0"};
  uint16_t port{kDefaultDiscoveryPort};  // 0 = 임의 포트(시험용)
};

class DiscoveryServer {
 public:
  DiscoveryServer(DiscoveryResponder responder, const DiscoveryServerConfig& config, SourceFilter filter,
                  LogFn log = {}, ClockFn clock = monotonic_seconds);
  ~DiscoveryServer();
  DiscoveryServer(const DiscoveryServer&) = delete;
  DiscoveryServer& operator=(const DiscoveryServer&) = delete;

  void start();  // bind 실패 시 std::system_error
  void shutdown();
  uint16_t bound_port() const { return bound_port_; }
  uint64_t replies_sent() const { return replies_sent_.load(); }
  uint64_t filtered() const { return filtered_.load(); }
  const std::string& name() const { return responder_.name(); }
  const std::string& identity() const { return responder_.identity(); }

 private:
  void loop();

  DiscoveryResponder responder_;  // 루프 스레드 전용
  DiscoveryServerConfig cfg_;
  SourceFilter filter_;
  LogFn log_;
  ClockFn clock_;
  int fd_{-1};
  uint16_t bound_port_{0};
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> replies_sent_{0};
  std::atomic<uint64_t> filtered_{0};
  std::thread thread_;
};

}  // namespace armpi_vr_bridge
