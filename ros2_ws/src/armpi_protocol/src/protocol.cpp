#include "armpi_protocol/protocol.hpp"

#include <cmath>
#include <initializer_list>

#include "armpi_common/json.hpp"

namespace armpi_protocol {

namespace json = armpi_common::json;

const char* to_string(PacketType t) {
  switch (t) {
    case PacketType::kCommand: return "command";
    case PacketType::kArmCommand: return "arm_command";
    case PacketType::kEmergencyStop: return "emergency_stop";
    case PacketType::kClearEmergency: return "clear_emergency";
    case PacketType::kDisconnect: return "disconnect";
  }
  return "unknown";
}

const char* to_string(Reason r) {
  switch (r) {
    case Reason::kOk: return "ok";
    case Reason::kInvalidPacket: return "invalid_packet";
    case Reason::kUnsupportedProtocol: return "unsupported_protocol";
    case Reason::kOutOfOrder: return "out_of_order";
    case Reason::kNotOwner: return "not_owner";
    case Reason::kEmergencyStop: return "emergency_stop";
    case Reason::kNeutralRequired: return "neutral_required";
    case Reason::kFault: return "fault";
    case Reason::kWorkspaceLimit: return "workspace_limit";
    case Reason::kIkUnreachable: return "ik_unreachable";
  }
  return "unknown";
}

const char* to_string(ChannelState s) {
  switch (s) {
    case ChannelState::kAwaitingNeutral: return "awaiting_neutral";
    case ChannelState::kIdle: return "idle";
    case ChannelState::kActive: return "active";
    case ChannelState::kTimeout: return "timeout";
    case ChannelState::kEmergencyStop: return "emergency_stop";
    case ChannelState::kFault: return "fault";
  }
  return "unknown";
}

const char* to_string(Channel c) { return c == Channel::kArm ? "arm" : "base"; }

bool Packet::is_neutral() const {
  if (type == PacketType::kArmCommand) return !control_active;
  return !control_active || (forward == 0.0 && strafe == 0.0 && turn == 0.0);
}

std::optional<std::string> canonical_uuid(std::string_view value) {
  if (value.size() != 36) return std::nullopt;
  std::string out(value);
  bool all_zero = true;
  for (std::size_t i = 0; i < out.size(); ++i) {
    char& c = out[i];
    if (i == 8 || i == 13 || i == 18 || i == 23) {
      if (c != '-') return std::nullopt;
      continue;
    }
    if (c >= 'A' && c <= 'F') c = static_cast<char>(c - 'A' + 'a');
    const bool hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    if (!hex) return std::nullopt;
    if (c != '0') all_zero = false;
  }
  if (all_zero) return std::nullopt;  // nil UUID
  return out;
}

namespace {

using json::Type;
using json::Value;

bool valid_sequence(const Value* v) {
  return v != nullptr && v->type == Type::kInteger && v->int_fits && v->integer >= 0 &&
         v->integer <= kMaxSequence;
}

// -1~1 유한한 JSON 숫자만. 불리언·문자열·null 은 거부(자료형이 다르다).
bool unit_axis(const Value* v, double& out) {
  if (v == nullptr) return false;
  if (v->type == Type::kInteger) {
    if (!v->int_fits || v->integer < -1 || v->integer > 1) return false;
    out = static_cast<double>(v->integer);
    return true;
  }
  if (v->type == Type::kFloat) {
    if (!v->number_finite || !(v->number >= -1.0 && v->number <= 1.0)) return false;
    out = v->number;
    return true;
  }
  return false;
}

// 유한한 JSON 숫자(범위 제한 없음). 범위는 세션(workspace)이 판정한다.
bool finite_number(const Value* v, double& out) {
  if (v == nullptr || !v->is_number()) return false;
  if (v->type == Type::kInteger && v->int_fits) {
    out = static_cast<double>(v->integer);
    return true;
  }
  if (!v->number_finite) return false;
  out = v->number;
  return std::isfinite(out);
}

bool ranged_number(const Value* v, double lo, double hi, double& out) {
  return finite_number(v, out) && out >= lo && out <= hi;
}

bool exact_fields(const Value& obj, std::initializer_list<const char*> required) {
  if (obj.object.size() != required.size()) return false;
  for (const char* key : required) {
    if (obj.find(key) == nullptr) return false;
  }
  return true;  // 크기가 같고 필수 키가 모두 있으며 중복이 없으므로 추가 키도 없다
}

}  // namespace

ParseResult parse_packet(std::string_view data) {
  ParseResult res;
  auto& err = res.error;
  auto reject = [&](Reason reason, std::string message) {
    err.reason = reason;
    err.message = std::move(message);
    return res;
  };

  if (data.empty() || data.size() > kMaxPacketBytes) {
    return reject(Reason::kInvalidPacket, "패킷 크기가 올바르지 않습니다");
  }
  Value root;
  std::string jerr;
  json::ParseOptions opt;
  opt.max_bytes = kMaxPacketBytes;
  if (!json::parse(data, root, jerr, opt)) {
    return reject(Reason::kInvalidPacket, "JSON 디코딩 실패: " + jerr);
  }
  if (root.type != Type::kObject) {
    return reject(Reason::kInvalidPacket, "JSON 객체만 허용됩니다");
  }

  const Value* type_v = root.find("type");
  if (type_v != nullptr && type_v->type == Type::kString && type_v->string == "arm_command") {
    err.channel = Channel::kArm;
  }
  std::optional<std::string> session_id;
  const Value* cid = root.find("client_id");
  if (cid != nullptr && cid->type == Type::kString) {
    session_id = canonical_uuid(cid->string);
    if (session_id) err.client_id = cid->string;
  }
  const Value* seq = root.find("sequence");
  if (valid_sequence(seq)) err.sequence = seq->integer;

  // 검사 순서: protocol → type → 필드 집합 → 값.
  const Value* proto = root.find("protocol");
  if (proto == nullptr || proto->type != Type::kInteger) {
    return reject(Reason::kInvalidPacket, "protocol은 정수여야 합니다");
  }
  if (!proto->int_fits || proto->integer != kProtocolVersion) {
    return reject(Reason::kUnsupportedProtocol, "지원하지 않는 protocol");
  }
  if (type_v == nullptr || type_v->type != Type::kString) {
    return reject(Reason::kInvalidPacket, "type은 문자열이어야 합니다");
  }

  Packet p;
  const std::string& t = type_v->string;
  bool fields_ok = false;
  if (t == "command") {
    p.type = PacketType::kCommand;
    fields_ok = exact_fields(root, {"protocol", "type", "client_id", "sequence", "control_active",
                                    "forward", "strafe", "turn"});
  } else if (t == "arm_command") {
    p.type = PacketType::kArmCommand;
    fields_ok = exact_fields(root, {"protocol", "type", "client_id", "sequence", "control_active",
                                    "x", "y", "z", "pitch", "gripper"});
  } else if (t == "emergency_stop" || t == "clear_emergency" || t == "disconnect") {
    p.type = t == "emergency_stop"    ? PacketType::kEmergencyStop
             : t == "clear_emergency" ? PacketType::kClearEmergency
                                      : PacketType::kDisconnect;
    fields_ok = exact_fields(root, {"protocol", "type", "client_id", "sequence"});
  } else {
    return reject(Reason::kInvalidPacket, "알 수 없는 type");
  }
  if (!fields_ok) return reject(Reason::kInvalidPacket, "필드 불일치(누락 또는 추가)");
  if (!session_id) return reject(Reason::kInvalidPacket, "client_id는 표준 UUID 문자열이어야 합니다");
  if (err.sequence < 0) return reject(Reason::kInvalidPacket, "sequence는 0~2^53-1 정수여야 합니다");

  p.client_id = *err.client_id;
  p.session_id = *session_id;
  p.sequence = err.sequence;

  if (p.type == PacketType::kCommand || p.type == PacketType::kArmCommand) {
    const Value* ca = root.find("control_active");
    if (ca->type != Type::kBool) return reject(Reason::kInvalidPacket, "control_active는 불리언이어야 합니다");
    p.control_active = ca->boolean;
  }
  if (p.type == PacketType::kCommand) {
    if (!unit_axis(root.find("forward"), p.forward) || !unit_axis(root.find("strafe"), p.strafe) ||
        !unit_axis(root.find("turn"), p.turn)) {
      return reject(Reason::kInvalidPacket, "forward/strafe/turn은 -1~1 유한한 숫자여야 합니다");
    }
  } else if (p.type == PacketType::kArmCommand) {
    if (!finite_number(root.find("x"), p.x) || !finite_number(root.find("y"), p.y) ||
        !finite_number(root.find("z"), p.z)) {
      return reject(Reason::kInvalidPacket, "x/y/z는 유한한 숫자여야 합니다");
    }
    if (!ranged_number(root.find("pitch"), -M_PI, M_PI, p.pitch)) {
      return reject(Reason::kInvalidPacket, "pitch는 -pi~pi 유한한 숫자여야 합니다");
    }
    if (!ranged_number(root.find("gripper"), 0.0, 1.0, p.gripper)) {
      return reject(Reason::kInvalidPacket, "gripper는 0~1 유한한 숫자여야 합니다");
    }
  }
  res.packet = std::move(p);
  return res;
}

std::string encode_ack(const Ack& ack) {
  std::string out;
  out.reserve(200);
  out += "{\"protocol\":3,\"type\":\"ack\",\"client_id\":";
  out += ack.client_id ? json::quote(*ack.client_id) : std::string("null");
  out += ",\"sequence\":";
  out += std::to_string(ack.sequence);
  out += ",\"accepted\":";
  out += ack.accepted ? "true" : "false";
  out += ",\"reason\":\"";
  out += to_string(ack.reason);
  out += "\",\"emergency_stop\":";
  out += ack.emergency_stop ? "true" : "false";
  out += ",\"state\":\"";
  out += to_string(ack.state);
  out += "\"}";
  return out;
}

}  // namespace armpi_protocol
