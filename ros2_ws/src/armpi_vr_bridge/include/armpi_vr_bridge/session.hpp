// Protocol v3 세션 상태기계 (docs/01_PROTOCOL_V3_SPEC.md §3, docs/02_ROS2_INTERFACES.md 수신기 규칙).
//
// - ROS·소켓·스레드 비의존. 호출자가 monotonic 시각(now, 초)을 넘기고 모든 I/O 를 담당한다.
// - 세션(소유권·sequence·비상정지·retire·lease)은 하나, 채널 상태(arming·watchdog)는 차체/팔 따로.
// - 스레드 안전하지 않다. 호출자(UdpServer)가 mutex 로 보호한다.
#pragma once

#include <cstddef>
#include <cstdint>
#include <list>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "armpi_kinematics/arm_kinematics.hpp"
#include "armpi_vr_bridge/net.hpp"
#include "armpi_vr_bridge/protocol.hpp"

namespace armpi_vr_bridge {

// 차체 목표 (m/s, m/s, rad/s). REP-103: vx 앞, vy 왼쪽, wz 반시계.
struct BaseTarget {
  double vx{0.0}, vy{0.0}, wz{0.0};
  bool is_zero() const { return vx == 0.0 && vy == 0.0 && wz == 0.0; }
  bool operator==(const BaseTarget& o) const { return vx == o.vx && vy == o.vy && wz == o.wz; }
  bool operator!=(const BaseTarget& o) const { return !(*this == o); }
};

// 팔 출력. active=false 면 제어기는 현재 위치를 유지한다(토크 유지). 값은 마지막으로 수락한 목표.
struct ArmOutput {
  bool active{false};
  bool has_goal{false};
  double x{0.0}, y{0.0}, z{0.0}, pitch{0.0}, gripper{0.0};
  bool operator==(const ArmOutput& o) const {
    return active == o.active && has_goal == o.has_goal && x == o.x && y == o.y && z == o.z &&
           pitch == o.pitch && gripper == o.gripper;
  }
  bool operator!=(const ArmOutput& o) const { return !(*this == o); }
};

// /armpi/control_state 의 내용 (armpi_control 이 발행)
struct ControlState {
  bool estop{false};
  bool fault{false};
  std::string fault_reason;
  bool udp_allowed{false};
};

struct SessionConfig {
  double base_watchdog_s{0.4};
  double arm_watchdog_s{0.4};
  double lease_s{1.0};
  double control_state_max_age_s{2.0};
  // 라이브러리 기본값(보수값, 단위 시험 기준). 로봇에서는 노드 매개변수(armpi.yaml)가 덮어쓴다
  double max_vx_m_s{0.10};
  double max_vy_m_s{0.10};
  double max_wz_rad_s{0.60};
  std::size_t retired_limit{1024};
  bool accept_estop_from_any_client{true};
  armpi_kinematics::Workspace workspace{};
  armpi_kinematics::Params kinematics{};

  // 잘못된 값이면 std::invalid_argument (한국어 설명)
  void validate() const;
};

struct HandleResult {
  Ack ack;
  Channel channel{Channel::kBase};
  std::optional<bool> estop_change;  // true = latch 요청, false = 해제 요청 (/armpi/estop_request)
  bool arm_accepted{false};          // 수락된 arm_command → ArmTarget 1회 발행
  std::vector<std::string> events;   // 상태 전이(로그용)
  std::string detail;                // 거부 사유 설명(로그용)
};

struct SessionSnapshot {
  std::optional<std::string> owner;
  std::optional<std::string> owner_addr;
  ChannelState base_state{ChannelState::kAwaitingNeutral};
  ChannelState arm_state{ChannelState::kAwaitingNeutral};
  bool emergency_stop{false};
  bool udp_estop_latched{false};
  bool controller_estop{false};
  std::optional<std::string> fault;
  bool udp_permitted{false};
  std::optional<std::string> blocked_reason;
  bool control_state_received{false};
  std::optional<double> control_state_age_s;
  int64_t last_sequence{-1};
  Reason last_reason{Reason::kOk};
  BaseTarget base_target;
  ArmOutput arm_output;
  std::size_t retired_count{0};
};

class Session {
 public:
  explicit Session(const SessionConfig& config);

  HandleResult handle_datagram(std::string_view data, const Endpoint& from, double now);
  HandleResult handle_packet(const Packet& packet, const Endpoint& from, double now);

  // watchdog·lease·control_state 나이 만료를 검사한다. 정지해야 할 이유 목록을 돌려준다.
  std::vector<std::string> check_timeouts(double now);

  // armpi_control 의 상태 반영. 상태 전이 목록을 돌려준다.
  std::vector<std::string> set_control_state(const ControlState& state, double now);

  // 외부 정지(수신 루프 지연·종료 등). 소유권은 유지하고 두 채널 모두 새 중립을 요구한다.
  void force_stop();
  // 재시작 전까지 활성 명령을 거부하는 내부 fault latch(수신 스레드 종료 등).
  void enter_fault(const std::string& reason);

  ChannelState state(Channel channel) const;
  ChannelState base_state() const { return state(Channel::kBase); }
  ChannelState arm_state() const { return state(Channel::kArm); }
  bool emergency_stop() const;
  std::optional<std::string> fault() const;
  const std::optional<std::string>& owner() const { return owner_; }
  bool udp_permitted() const { return gate_permitted_; }
  BaseTarget base_output() const { return base_.target; }
  ArmOutput arm_output() const;
  SessionSnapshot snapshot(double now) const;
  const SessionConfig& config() const { return cfg_; }

 private:
  struct ChannelCore {
    bool neutral_confirmed{false};
    bool timed_out{false};
    bool active{false};
    std::optional<double> last_rx;
    void reset() {
      neutral_confirmed = false;
      timed_out = false;
      active = false;
      last_rx.reset();
    }
  };
  struct BaseChannel : ChannelCore {
    BaseTarget target;
  };
  struct ArmChannel : ChannelCore {
    ArmOutput goal;  // goal.active 는 쓰지 않는다(ChannelCore::active 사용)
  };

  HandleResult handle_current(const Packet& p, const Endpoint& from, double now);
  HandleResult apply(const Packet& p, double now);
  HandleResult apply_base(const Packet& p, double now);
  HandleResult apply_arm(const Packet& p, double now);

  void evaluate_gate(double now, std::vector<std::string>& events);
  ChannelState channel_state(const ChannelCore& ch) const;
  void reset_base();
  void reset_arm();
  void reset_all();
  void release_owner();
  void retire(const std::string& session_id);
  bool is_retired(const std::string& session_id) const;

  Ack make_ack(const std::optional<std::string>& client_id, int64_t sequence, bool accepted, Reason reason,
               Channel channel);
  HandleResult accept(const Packet& p, std::optional<std::string> event = std::nullopt);
  HandleResult reject(const std::optional<std::string>& client_id, int64_t sequence, Reason reason,
                      Channel channel, std::string detail);

  SessionConfig cfg_;
  armpi_kinematics::ArmKinematics kin_;

  std::optional<std::string> owner_;
  std::optional<Endpoint> owner_addr_;
  int64_t last_sequence_{-1};
  double last_owner_activity_{0.0};
  // lease 만료로 풀린 직전 조종자(UUID, 마지막 sequence). 다른 UUID 가 인수하기 전까지만 재획득 가능.
  std::optional<std::pair<std::string, int64_t>> previous_owner_;
  // disconnect 했거나 교체된 세션(최대 retired_limit, 가장 오래된 것부터 잊음)
  std::list<std::string> retired_order_;
  std::unordered_map<std::string, std::list<std::string>::iterator> retired_index_;

  BaseChannel base_;
  ArmChannel arm_;

  bool udp_estop_{false};                  // UDP 로 걸린 latch
  std::optional<std::string> internal_fault_;
  bool cs_received_{false};
  double cs_rx_time_{0.0};
  ControlState cs_;
  bool gate_permitted_{false};             // 마지막으로 평가한 게이트(시작 시 차단)
  std::optional<std::string> blocked_reason_{"control_state 미수신"};
  Reason last_reason_{Reason::kOk};
};

}  // namespace armpi_vr_bridge
