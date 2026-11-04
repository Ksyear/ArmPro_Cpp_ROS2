#include "armpi_protocol/client_codec.hpp"

#include <array>
#include <initializer_list>

#include "armpi_common/json.hpp"

namespace armpi_protocol {

namespace json = armpi_common::json;

namespace {

std::string head(std::string_view type, std::string_view client_id, int64_t sequence) {
  std::string out;
  out.reserve(220);
  out += "{\"protocol\":3,\"type\":";
  out += json::quote(type);
  out += ",\"client_id\":";
  out += json::quote(client_id);
  out += ",\"sequence\":";
  out += std::to_string(sequence);
  return out;
}

bool safe_string(std::string_view s) {
  for (const char c : s) {
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
    if (!ok) return false;
  }
  return true;
}

}  // namespace

std::string encode_command(std::string_view client_id, int64_t sequence, bool control_active, double forward,
                           double strafe, double turn) {
  std::string out = head("command", client_id, sequence);
  out += ",\"control_active\":";
  out += json::boolean(control_active);
  out += ",\"forward\":" + json::number(forward);
  out += ",\"strafe\":" + json::number(strafe);
  out += ",\"turn\":" + json::number(turn);
  out += "}";
  return out;
}

std::string encode_arm_command(std::string_view client_id, int64_t sequence, bool control_active,
                               const ArmTarget& t) {
  std::string out = head("arm_command", client_id, sequence);
  out += ",\"control_active\":";
  out += json::boolean(control_active);
  out += ",\"x\":" + json::number(t.x);
  out += ",\"y\":" + json::number(t.y);
  out += ",\"z\":" + json::number(t.z);
  out += ",\"pitch\":" + json::number(t.pitch);
  out += ",\"gripper\":" + json::number(t.gripper);
  out += "}";
  return out;
}

std::string encode_event(PacketType type, std::string_view client_id, int64_t sequence) {
  return head(to_string(type), client_id, sequence) + "}";
}

std::optional<Reason> reason_from_string(std::string_view text) {
  for (Reason r : {Reason::kOk, Reason::kInvalidPacket, Reason::kUnsupportedProtocol, Reason::kOutOfOrder,
                   Reason::kNotOwner, Reason::kEmergencyStop, Reason::kNeutralRequired, Reason::kFault,
                   Reason::kWorkspaceLimit, Reason::kIkUnreachable}) {
    if (text == to_string(r)) return r;
  }
  return std::nullopt;
}

std::optional<ChannelState> state_from_string(std::string_view text) {
  for (ChannelState s : {ChannelState::kAwaitingNeutral, ChannelState::kIdle, ChannelState::kActive,
                         ChannelState::kTimeout, ChannelState::kEmergencyStop, ChannelState::kFault}) {
    if (text == to_string(s)) return s;
  }
  return std::nullopt;
}

std::optional<AckMessage> parse_ack(std::string_view data) {
  if (data.empty() || data.size() > kMaxPacketBytes) return std::nullopt;
  json::Value root;
  std::string err;
  json::ParseOptions opt;
  opt.max_bytes = kMaxPacketBytes;
  if (!json::parse(data, root, err, opt) || !root.is_object()) return std::nullopt;
  static constexpr std::array<const char*, 8> kKeys = {"protocol", "type",   "client_id",      "sequence",
                                                       "accepted", "reason", "emergency_stop", "state"};
  if (root.object.size() != kKeys.size()) return std::nullopt;
  for (const char* k : kKeys) {
    if (root.find(k) == nullptr) return std::nullopt;  // 크기가 같고 중복이 없으므로 추가 키도 없다
  }
  const json::Value* proto = root.find("protocol");
  if (proto->type != json::Type::kInteger || !proto->int_fits || proto->integer != kProtocolVersion) {
    return std::nullopt;
  }
  const json::Value* type = root.find("type");
  if (!type->is_string() || type->string != "ack") return std::nullopt;

  AckMessage ack;
  const json::Value* cid = root.find("client_id");
  if (cid->is_string()) {
    if (!safe_string(cid->string)) return std::nullopt;
    ack.client_id = cid->string;
  } else if (!cid->is_null()) {
    return std::nullopt;
  }
  const json::Value* seq = root.find("sequence");
  if (seq->type != json::Type::kInteger || !seq->int_fits || seq->integer < -1 || seq->integer > kMaxSequence) {
    return std::nullopt;
  }
  ack.sequence = seq->integer;
  const json::Value* acc = root.find("accepted");
  const json::Value* est = root.find("emergency_stop");
  if (!acc->is_bool() || !est->is_bool()) return std::nullopt;
  ack.accepted = acc->boolean;
  ack.emergency_stop = est->boolean;
  const json::Value* reason = root.find("reason");
  const json::Value* state = root.find("state");
  if (!reason->is_string() || !state->is_string()) return std::nullopt;
  const auto r = reason_from_string(reason->string);
  const auto s = state_from_string(state->string);
  if (!r || !s) return std::nullopt;
  ack.reason = *r;
  ack.state = *s;
  return ack;
}

}  // namespace armpi_protocol
