// UDP 서버·검색 서버 시험 (127.0.0.1 루프백, 실제 시간). ROS 없이 실행한다.
#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "armpi_common/json.hpp"
#include "armpi_vr_bridge/udp_server.hpp"

using namespace armpi_vr_bridge;
namespace json = armpi_common::json;
using namespace std::chrono_literals;

namespace {

const std::string kCid = "d8b27f10-09a8-4c21-b649-1e7058d46b72";

std::string cmd_json(int64_t seq, bool active, double f) {
  return "{\"protocol\":3,\"type\":\"command\",\"client_id\":\"" + kCid + "\",\"sequence\":" + std::to_string(seq) +
         ",\"control_active\":" + (active ? "true" : "false") + ",\"forward\":" + json::number(f) +
         ",\"strafe\":0,\"turn\":0}";
}

std::string ev_json(const std::string& type, int64_t seq) {
  return "{\"protocol\":3,\"type\":\"" + type + "\",\"client_id\":\"" + kCid + "\",\"sequence\":" +
         std::to_string(seq) + "}";
}

SessionConfig session_cfg() {
  SessionConfig c;
  c.workspace = {0.0, 0.30, -0.25, 0.25, -0.05, 0.35};
  c.kinematics.base_height_m = 0.10;
  c.kinematics.upper_arm_m = 0.10;
  c.kinematics.forearm_m = 0.095;
  c.kinematics.hand_m = 0.12;
  return c;
}

ServerConfig loopback() {
  ServerConfig s;
  s.bind_address = "127.0.0.1";
  s.port = 0;
  return s;
}

class Client {
 public:
  explicit Client(uint16_t server_port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ::bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local));
    server_.sin_family = AF_INET;
    server_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    server_.sin_port = htons(server_port);
  }
  ~Client() { ::close(fd_); }
  void send(const std::string& data) {
    ::sendto(fd_, data.data(), data.size(), 0, reinterpret_cast<sockaddr*>(&server_), sizeof(server_));
  }
  std::optional<std::string> recv(double timeout_s, uint16_t* from_port = nullptr) {
    pollfd p{fd_, POLLIN, 0};
    if (::poll(&p, 1, static_cast<int>(timeout_s * 1000)) <= 0) return std::nullopt;
    char buf[4096];
    sockaddr_in src{};
    socklen_t len = sizeof(src);
    const ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&src), &len);
    if (n < 0) return std::nullopt;
    if (from_port) *from_port = ntohs(src.sin_port);
    return std::string(buf, static_cast<std::size_t>(n));
  }
  // kCid 의 ACK 가 올 때까지(다른 ACK 는 건너뜀). seq >= 0 이면 그 번호의 ACK 만.
  std::optional<json::Value> ack_for_cid(double timeout_s, int64_t seq = -1) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
      const auto m = recv(0.05);
      if (!m) continue;
      json::Value v;
      std::string err;
      if (!json::parse(*m, v, err) || !v.find("client_id") || v.find("client_id")->string != kCid) continue;
      if (seq >= 0 && v.find("sequence")->integer != seq) continue;
      return v;
    }
    return std::nullopt;
  }

 private:
  int fd_;
  sockaddr_in server_{};
};

struct Recorder {
  std::mutex m;
  std::vector<std::string> stops;
  std::vector<bool> estops;
  int packets{0};
  std::atomic<bool> block_next{false};
  std::atomic<bool> blocking{false};

  ServerCallbacks callbacks() {
    ServerCallbacks cb;
    cb.on_packet = [this](const HandleResult&) {
      {
        std::lock_guard<std::mutex> lk(m);
        ++packets;
      }
      if (block_next.exchange(false)) {  // 수신 스레드 정지(교착·CPU 부족) 흉내
        blocking = true;
        std::this_thread::sleep_for(800ms);
        blocking = false;
      }
    };
    cb.on_estop = [this](bool latch) {
      std::lock_guard<std::mutex> lk(m);
      estops.push_back(latch);
    };
    cb.on_stop = [this](const std::string& why) {
      std::lock_guard<std::mutex> lk(m);
      stops.push_back(why);
    };
    return cb;
  }
  bool has_stop(const std::string& needle) {
    std::lock_guard<std::mutex> lk(m);
    for (const auto& s : stops) {
      if (s.find(needle) != std::string::npos) return true;
    }
    return false;
  }
};

bool wait_until(const std::function<bool()>& pred, double timeout_s) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return pred();
}

}  // namespace

TEST(UdpServer, AckRoundTripFromSameSocket) {
  Recorder rec;
  UdpServer srv(session_cfg(), loopback(), SourceFilter(), rec.callbacks());
  srv.start();
  ASSERT_NE(srv.bound_port(), 0);
  Client c(srv.bound_port());
  c.send(cmd_json(1, false, 0.0));
  uint16_t from = 0;
  const auto m = c.recv(1.0, &from);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(from, srv.bound_port());  // 같은 소켓으로 응답
  EXPECT_EQ(*m, "{\"protocol\":3,\"type\":\"ack\",\"client_id\":\"" + kCid +
                    "\",\"sequence\":1,\"accepted\":true,\"reason\":\"ok\",\"emergency_stop\":false,"
                    "\"state\":\"idle\"}");
  EXPECT_TRUE(wait_until([&] { std::lock_guard<std::mutex> lk(rec.m); return rec.packets == 1; }, 1.0));
  srv.shutdown();
}

TEST(UdpServer, OversizeAndEmptyDatagramsGetNullIdInvalidAck) {
  Recorder rec;
  UdpServer srv(session_cfg(), loopback(), SourceFilter(), rec.callbacks());
  srv.start();
  Client c(srv.bound_port());
  std::string big = cmd_json(1, false, 0.0);
  big.insert(1, std::string(3000, ' '));  // 유효한 JSON 이지만 2048 byte 초과
  c.send(big);
  auto m = c.recv(1.0);
  ASSERT_TRUE(m.has_value());
  EXPECT_NE(m->find("\"client_id\":null,\"sequence\":-1,\"accepted\":false,\"reason\":\"invalid_packet\""),
            std::string::npos)
      << *m;
  c.send("");
  m = c.recv(1.0);
  ASSERT_TRUE(m.has_value());
  EXPECT_NE(m->find("invalid_packet"), std::string::npos);
}

TEST(UdpServer, GarbageFloodDoesNotKillReceiver) {
  Recorder rec;
  UdpServer srv(session_cfg(), loopback(), SourceFilter(), rec.callbacks());
  srv.start();
  Client c(srv.bound_port());
  std::mt19937 rng(42);
  for (int i = 0; i < 2000; ++i) {
    std::string junk(static_cast<std::size_t>(i % 3000 + 1), '\0');
    for (auto& ch : junk) ch = static_cast<char>(rng() & 0xFF);
    c.send(junk);
  }
  std::this_thread::sleep_for(500ms);
  std::optional<json::Value> ack;
  for (int attempt = 1; attempt <= 5 && !ack; ++attempt) {
    c.send(cmd_json(attempt, false, 0.0));
    ack = c.ack_for_cid(0.5);
  }
  ASSERT_TRUE(ack.has_value());
  EXPECT_TRUE(ack->find("accepted")->boolean);
  EXPECT_TRUE(srv.running());
  EXPECT_GE(srv.stats().reasons["invalid_packet"], 100u);
}

TEST(UdpServer, SecondServerOnSamePortFailsVisibly) {
  Recorder rec;
  UdpServer srv(session_cfg(), loopback(), SourceFilter(), rec.callbacks());
  srv.start();
  ServerConfig same = loopback();
  same.port = srv.bound_port();
  UdpServer other(session_cfg(), same, SourceFilter(), rec.callbacks());
  EXPECT_THROW(other.start(), std::system_error);
}

TEST(UdpServer, AllowedSourcesDropsSilently) {
  Recorder rec;
  UdpServer srv(session_cfg(), loopback(), SourceFilter({"10.0.0.0/8"}), rec.callbacks());
  srv.start();
  Client c(srv.bound_port());
  c.send(cmd_json(1, false, 0.0));
  EXPECT_FALSE(c.recv(0.3).has_value());  // 응답 없음
  EXPECT_GE(srv.stats().filtered, 1u);
  EXPECT_EQ(srv.stats().packets, 0u);
  std::lock_guard<std::mutex> lk(rec.m);
  EXPECT_EQ(rec.packets, 0);
}

TEST(UdpServer, AllowedSourcesPermitsListedAddress) {
  Recorder rec;
  UdpServer srv(session_cfg(), loopback(), SourceFilter({"127.0.0.0/8"}), rec.callbacks());
  srv.start();
  Client c(srv.bound_port());
  c.send(cmd_json(1, false, 0.0));
  EXPECT_TRUE(c.recv(1.0).has_value());
}

TEST(UdpServer, ReceiveLoopWatchdogStopsWithinTimeout) {
  Recorder rec;
  UdpServer srv(session_cfg(), loopback(), SourceFilter(), rec.callbacks());
  srv.start();
  srv.set_control_state(ControlState{false, false, "", true});
  Client c(srv.bound_port());
  c.send(cmd_json(1, false, 0.0));
  ASSERT_TRUE(c.recv(1.0));
  c.send(cmd_json(2, true, 1.0));
  ASSERT_TRUE(c.recv(1.0));
  const auto t_last = std::chrono::steady_clock::now();
  EXPECT_NEAR(srv.with_session([](Session& s) { return s.base_output().vx; }), 0.10, 1e-12);
  ASSERT_TRUE(wait_until([&] { return rec.has_stop("watchdog"); }, 1.5));
  const double latency = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_last).count();
  EXPECT_GE(latency, 0.38);
  EXPECT_LE(latency, 0.60);  // 0.4 s watchdog + poll 주기(0.02 s) + 여유
  EXPECT_EQ(srv.with_session([](Session& s) { return s.base_state(); }), ChannelState::kTimeout);
  EXPECT_TRUE(srv.with_session([](Session& s) { return s.base_output().is_zero(); }));
}

TEST(UdpServer, StallMonitorStopsWhileReceiveThreadIsBlocked) {
  Recorder rec;
  UdpServer srv(session_cfg(), loopback(), SourceFilter(), rec.callbacks());
  srv.start();
  srv.set_control_state(ControlState{false, false, "", true});
  Client c(srv.bound_port());
  c.send(cmd_json(1, false, 0.0));
  ASSERT_TRUE(c.recv(1.0));
  rec.block_next = true;  // 다음 패킷 처리 콜백에서 0.8 s 멈춤
  c.send(cmd_json(2, true, 0.5));
  ASSERT_TRUE(wait_until([&] { return rec.blocking.load(); }, 1.0));
  ASSERT_TRUE(wait_until([&] { return rec.has_stop("수신 루프 지연"); }, 1.0));
  EXPECT_TRUE(rec.blocking.load());  // 수신 스레드가 아직 막혀 있는 동안 정지했다
  EXPECT_TRUE(srv.with_session([](Session& s) { return s.base_output().is_zero(); }));
  EXPECT_EQ(srv.with_session([](Session& s) { return s.base_state(); }), ChannelState::kAwaitingNeutral);
  ASSERT_TRUE(wait_until([&] { return !rec.blocking.load(); }, 2.0));
  // 회복 후에도 계속 동작(새 중립 필요)
  c.send(cmd_json(3, true, 0.5));
  const auto ack = c.ack_for_cid(1.0, 3);
  ASSERT_TRUE(ack.has_value());
  EXPECT_EQ(ack->find("reason")->string, "neutral_required");
}

TEST(UdpServer, EstopRequestCallbackAndAck) {
  Recorder rec;
  UdpServer srv(session_cfg(), loopback(), SourceFilter(), rec.callbacks());
  srv.start();
  Client c(srv.bound_port());
  c.send(cmd_json(1, false, 0.0));
  ASSERT_TRUE(c.recv(1.0));
  c.send(ev_json("emergency_stop", 2));
  const auto ack = c.ack_for_cid(1.0);
  ASSERT_TRUE(ack.has_value());
  EXPECT_TRUE(ack->find("emergency_stop")->boolean);
  EXPECT_EQ(ack->find("state")->string, "emergency_stop");
  c.send(ev_json("clear_emergency", 3));
  ASSERT_TRUE(c.ack_for_cid(1.0).has_value());
  std::lock_guard<std::mutex> lk(rec.m);
  EXPECT_EQ(rec.estops, (std::vector<bool>{true, false}));
}

TEST(UdpServer, ShutdownForcesStop) {
  Recorder rec;
  UdpServer srv(session_cfg(), loopback(), SourceFilter(), rec.callbacks());
  srv.start();
  srv.set_control_state(ControlState{false, false, "", true});
  Client c(srv.bound_port());
  c.send(cmd_json(1, false, 0.0));
  c.recv(1.0);
  c.send(cmd_json(2, true, 0.5));
  c.recv(1.0);
  srv.shutdown();
  EXPECT_FALSE(srv.running());
  EXPECT_TRUE(rec.has_stop("서버 종료"));
  EXPECT_TRUE(srv.with_session([](Session& s) { return s.base_output().is_zero(); }));
  srv.shutdown();  // 두 번 불러도 안전
}

TEST(DiscoveryServer, RoundTripFromBoundPort) {
  const std::string id(32, 'b');
  DiscoveryServerConfig cfg;
  cfg.bind_address = "127.0.0.1";
  cfg.port = 0;
  DiscoveryServer srv(DiscoveryResponder("armpi test", id, 5001), cfg, SourceFilter());
  srv.start();
  Client c(srv.bound_port());
  const std::string nonce = "00112233445566778899aabbccddeeff";
  c.send("ARMPI_DISCOVER/1|" + nonce);
  uint16_t from = 0;
  const auto m = c.recv(1.0, &from);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(from, srv.bound_port());  // 응답 출발 포트 = 검색 포트
  EXPECT_EQ(*m, "ARMPI_HERE/1|" + nonce + "|" + id + "|armpi%20test|5001|3");
  c.send("OTHERBOT_DISCOVER/1|" + nonce);  // 다른 로봇 계열 요청은 무시
  EXPECT_FALSE(c.recv(0.3).has_value());
  EXPECT_EQ(srv.replies_sent(), 1u);
}

TEST(DiscoveryServer, AllowedSourcesFilter) {
  DiscoveryServerConfig cfg;
  cfg.bind_address = "127.0.0.1";
  cfg.port = 0;
  DiscoveryServer srv(DiscoveryResponder("armpi", std::string(32, 'c'), 5001), cfg, SourceFilter({"10.0.0.0/8"}));
  srv.start();
  Client c(srv.bound_port());
  c.send("ARMPI_DISCOVER/1|00112233445566778899aabbccddeeff");
  EXPECT_FALSE(c.recv(0.3).has_value());
  EXPECT_TRUE(wait_until([&] { return srv.filtered() >= 1; }, 1.0));
}
