// Protocol v3 클라이언트 세션(차체·팔 두 채널). protocol_v3/armpi_v3_client.py 의 ArmPiV3Client 를 C++ 로 옮겼다.
//
// - 채널마다 중립 확인 → 무장(armed) → 활성. ACK 의 state 는 그 패킷이 속한 채널 상태다.
// - 자동 재무장: 사용자가 조종 의도를 유지하는 동안 통신 끊김·watchdog 때문에 풀린 채널은
//   "중립 송신 → 로봇의 중립 확인(ACK state=idle) → 활성"을 다시 밟는다.
//   비상정지·해제·fault·not_owner·형식 오류 같은 '강한' 차단 뒤에는 의도를 한 번 내려야 다시 무장한다.
// - 팔 목표 거부(workspace_limit·ik_unreachable)는 채널을 풀지 않고 목표를 마지막 수락값으로 되돌린다.
// - emergency_stop 이벤트는 ACK 를 받을 때까지 tick 마다(50 ms, 매번 새 sequence) 다시 보낸다.
//
// 스레드: 모든 공개 메서드는 내부 재귀 mutex 로 보호된다. 여러 호출을 한 덩어리로 묶을 때는 mutex() 를 잡는다.
// 시험: 송수신(DatagramLink)·시계·UUID 생성기를 바꿔 끼울 수 있다.
#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "armpi_protocol/client_codec.hpp"
#include "armpi_protocol/datagram_link.hpp"

namespace armpi_protocol {

constexpr double kClientAckTimeoutS = 0.30;   // 사양 4절
constexpr double kClientSendPeriodS = 0.05;   // 채널마다 20 Hz
constexpr double kEstopRetryPeriodS = 0.05;

struct ClientOptions {
  double ack_timeout_s{kClientAckTimeoutS};
  double ack_max_age_s{-1.0};  // 음수면 ack_timeout_s 와 같다
  ArmTarget home{kDefaultHome};
  bool send_arm{true};
  std::function<double()> clock;                    // 단조 시계(초). 비우면 steady_clock
  std::function<void(const std::string&)> log;      // 비우면 기록 안 함
  std::function<std::string()> new_uuid;            // 비우면 무작위 UUIDv4
  // 팔 목표를 바꿀 때마다(set/move) 거친다. wanted = 새로 원하는 목표, current = 지금 목표.
  // 로봇과 같은 보정 규칙으로 닿는 자세에 맞춰 두면(armpi_web_gateway) 버튼으로 움직인 목표가 닿지 않는 곳으로
  // 앞서 나가지 않는다. 비우면 그대로.
  std::function<ArmTarget(const ArmTarget& wanted, const ArmTarget& current)> arm_target_filter;
};

// 채널 하나(차체 또는 팔)의 클라이언트 쪽 무장 상태
struct ChannelStatus {
  bool intent{false};             // 사용자가 지금 조종하려 함
  bool armed{false};              // 활성 패킷을 보내도 됨
  bool neutral_confirmed{false};
  int64_t neutral_after{0};       // 이 번호보다 뒤에 보낸 중립만 확인으로 인정
  bool needs_release{false};      // 강한 차단 뒤: 의도를 한 번 내려야 재무장
  std::string server_state{"no_ack"};
  std::string server_reason{"no_ack"};
  std::optional<double> last_ack_time;
  int blocks{0};
  int auto_rearms{0};
  std::string last_block_reason;
};

class V3Client {
 public:
  V3Client(std::unique_ptr<DatagramLink> link, ClientOptions options);

  // 50 ms 마다 1회: ACK 비우기 → 링크 검사 → 의도 반영 → (비상정지 재전송) → 차체·팔 패킷 1개씩
  void tick(bool base_intent, double forward, double strafe, double turn, bool arm_intent);
  void poll();  // ACK 만 비운다

  void emergency_stop();   // 즉시 송신, ACK 까지 tick 마다 재전송
  bool clear_emergency();  // retired 면 false
  void disconnect();       // 중립 2개 + disconnect. 이 UUID 는 다시 못 쓴다
  void new_session();      // 새 UUID 로 다시 시작(비상정지 표시는 유지, 로봇 ACK 가 다시 알려 줌)

  // 팔 목표(부분 지정). pitch 는 −π..π, gripper 는 0..1 로 자른다. accepted=true 면 마지막 수락값도 바꾼다.
  ArmTarget set_arm_target(std::optional<double> x, std::optional<double> y, std::optional<double> z,
                           std::optional<double> pitch, std::optional<double> gripper, bool accepted = false);
  ArmTarget set_arm_target(const ArmTarget& t, bool accepted = false);
  ArmTarget move_arm_target(double dx, double dy, double dz, double dpitch, double dgripper);

  // ── 조회 ──
  std::recursive_mutex& mutex() const { return mu_; }
  bool retired() const;
  bool pending_estop() const;
  ChannelStatus base() const;
  ChannelStatus arm() const;
  ArmTarget arm_target() const;
  void set_last_error(const std::string& text);
  // Python ArmPiV3Client.snapshot() 과 같은 키의 JSON 객체
  std::string snapshot_json() const;
  std::string client_id() const;

 private:
  struct Sent {
    double time;
    PacketType kind;
    bool arm_channel;
    bool neutral;
    std::optional<ArmTarget> target;
  };
  struct Reject {
    Reason reason;
    std::optional<ArmTarget> target;
    double time;
    int64_t sequence;
  };

  double now() const;
  void log(const std::string& msg) const;
  void new_session_locked(bool keep_emergency);
  int64_t next_seq();
  void send_locked(const std::string& data, int64_t seq, PacketType kind, bool arm_channel, bool neutral,
                   std::optional<ArmTarget> target);
  void send_command_locked(double forward, double strafe, double turn);
  void send_arm_locked();
  void send_event_locked(PacketType kind);
  bool can_drive_locked() const;
  void block_locked(ChannelStatus& ch, bool hard, const std::string& why);
  void block_all_locked(bool hard, const std::string& why);
  void update_intent_locked(ChannelStatus& ch, bool intent);
  void poll_locked();
  void on_ack_locked(const AckMessage& ack, double now);
  void check_link_locked(double now);

  mutable std::recursive_mutex mu_;
  std::unique_ptr<DatagramLink> link_;
  ClientOptions opt_;

  // 통계
  std::deque<double> rtts_;  // 최근 200개
  std::optional<double> last_rtt_;
  uint64_t packets_sent_{0}, acks_received_{0}, bad_acks_{0}, late_acks_{0}, null_acks_{0}, send_errors_{0},
      link_drops_{0}, sessions_started_{0};
  std::string last_error_;

  // 세션
  std::string client_id_;
  int64_t sequence_{0};
  int64_t last_ack_sequence_{-1};
  std::optional<double> last_ack_time_;
  bool connected_{false}, faulted_{false}, retired_{false}, emergency_{false}, pending_estop_{false};
  double last_estop_send_{-1e18};
  int64_t estop_acked_seq_{-1};
  std::map<int64_t, Sent> sent_;  // 최근 512개
  ChannelStatus base_, arm_;
  ArmTarget arm_target_;
  std::optional<ArmTarget> arm_last_accepted_;
  std::optional<Reject> arm_last_reject_;
};

// 무작위 UUIDv4 (소문자, 하이픈 포함 36자)
std::string random_uuid_v4();

}  // namespace armpi_protocol
