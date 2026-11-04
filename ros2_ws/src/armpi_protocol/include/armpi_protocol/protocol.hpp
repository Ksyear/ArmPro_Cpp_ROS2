// ArmPi Protocol v3 패킷 형식 (docs/01_PROTOCOL_V3_SPEC.md §2). ROS·소켓 비의존.
// 로봇(armpi_vr_bridge 수신기)과 C++ 클라이언트(armpi_web_gateway)가 같이 쓴다.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace armpi_protocol {

constexpr int kProtocolVersion = 3;
constexpr std::size_t kMaxPacketBytes = 2048;
constexpr int64_t kMaxSequence = 9007199254740991LL;  // 2^53 - 1 (JS/double 안전 정수)

enum class PacketType { kCommand, kArmCommand, kEmergencyStop, kClearEmergency, kDisconnect };

enum class Reason {
  kOk,
  kInvalidPacket,
  kUnsupportedProtocol,
  kOutOfOrder,
  kNotOwner,
  kEmergencyStop,
  kNeutralRequired,
  kFault,
  kWorkspaceLimit,  // [v3 신규]
  kIkUnreachable,   // [v3 신규]
};

enum class ChannelState { kAwaitingNeutral, kIdle, kActive, kTimeout, kEmergencyStop, kFault };

// [v3] 채널: command·event → 차체, arm_command → 팔
enum class Channel { kBase, kArm };

const char* to_string(PacketType t);
const char* to_string(Reason r);
const char* to_string(ChannelState s);
const char* to_string(Channel c);

struct Packet {
  PacketType type{PacketType::kCommand};
  std::string client_id;   // 받은 문자열 그대로(ACK 에 되돌림)
  std::string session_id;  // 비교용 소문자 UUID
  int64_t sequence{0};
  bool control_active{false};
  // command
  double forward{0.0}, strafe{0.0}, turn{0.0};
  // arm_command
  double x{0.0}, y{0.0}, z{0.0}, pitch{0.0}, gripper{0.0};

  Channel channel() const { return type == PacketType::kArmCommand ? Channel::kArm : Channel::kBase; }
  // command: control_active=false 이거나 세 축이 모두 0. arm_command: control_active=false.
  bool is_neutral() const;
};

struct PacketError {
  Reason reason{Reason::kInvalidPacket};
  std::string message;
  std::optional<std::string> client_id;  // 해석 가능했으면 되돌릴 값, 아니면 null
  int64_t sequence{-1};
  Channel channel{Channel::kBase};  // type 이 "arm_command" 였으면 팔 채널 상태로 ACK
};

struct ParseResult {
  std::optional<Packet> packet;
  PacketError error;
  bool ok() const { return packet.has_value(); }
};

// 엄격한 v3 datagram 하나를 해석한다. 누락·추가·중복·자료형·범위 오류는 거부한다.
ParseResult parse_packet(std::string_view data);

// 하이픈 포함 36자 표준 UUID(대소문자 무시)만 허용하고 nil UUID 는 거부. 소문자로 돌려준다.
std::optional<std::string> canonical_uuid(std::string_view value);

struct Ack {
  std::optional<std::string> client_id;
  int64_t sequence{-1};
  bool accepted{false};
  Reason reason{Reason::kInvalidPacket};
  bool emergency_stop{false};
  ChannelState state{ChannelState::kAwaitingNeutral};
};

// 정확히 8필드, 필드 순서 고정: protocol,type,client_id,sequence,accepted,reason,emergency_stop,state
std::string encode_ack(const Ack& ack);

}  // namespace armpi_protocol
