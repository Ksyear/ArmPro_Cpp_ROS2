// 클라이언트 쪽 Protocol v3: 패킷 작성(command·arm_command·이벤트)과 로봇 ACK 엄격 해석.
// 기준: docs/01_PROTOCOL_V3_SPEC.md, 참조 구현 protocol_v3/armpi_v3_client.py(parse_ack, _send_*).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "armpi_protocol/protocol.hpp"

namespace armpi_protocol {

// 팔 목표(손끝 m, pitch rad, gripper 0~1)
struct ArmTarget {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double pitch{0.0};
  double gripper{0.0};
};

// 기본 자세(서보 기본값)의 FK. Python DEFAULT_HOME 과 같다.
constexpr ArmTarget kDefaultHome{0.1047, 0.0, 0.0753, -1.0263, 0.0};

// 필드 순서는 사양 2절과 Python 참조 구현(OrderedDict)과 같다.
std::string encode_command(std::string_view client_id, int64_t sequence, bool control_active, double forward,
                           double strafe, double turn);
std::string encode_arm_command(std::string_view client_id, int64_t sequence, bool control_active,
                               const ArmTarget& target);
// type: kEmergencyStop·kClearEmergency·kDisconnect
std::string encode_event(PacketType type, std::string_view client_id, int64_t sequence);

struct AckMessage {
  std::optional<std::string> client_id;  // 식별 불가 패킷의 ACK 이면 nullopt(JSON null)
  int64_t sequence{-1};
  bool accepted{false};
  Reason reason{Reason::kInvalidPacket};
  bool emergency_stop{false};
  ChannelState state{ChannelState::kAwaitingNeutral};
};

// 로봇 ACK 1개를 엄격하게 해석한다. 형식이 틀리면 nullopt.
// 정확히 8필드, 값은 문자열·불리언·정수만(실수 금지), 중복 키 금지, 문자열은 [A-Za-z0-9_-] 만.
std::optional<AckMessage> parse_ack(std::string_view data);

std::optional<Reason> reason_from_string(std::string_view text);
std::optional<ChannelState> state_from_string(std::string_view text);

}  // namespace armpi_protocol
