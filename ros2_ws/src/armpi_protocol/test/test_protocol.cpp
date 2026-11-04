// Protocol v3 패킷 파서·ACK 형식 단위 시험 (docs/01_PROTOCOL_V3_SPEC.md §2).
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "armpi_protocol/protocol.hpp"

using namespace armpi_protocol;

namespace {

const std::string kCid = "d8b27f10-09a8-4c21-b649-1e7058d46b72";

// 필드(키 → JSON 리터럴) 순서를 유지하는 간단한 패킷 빌더
using Fields = std::vector<std::pair<std::string, std::string>>;

std::string build(const Fields& f) {
  std::string s = "{";
  for (std::size_t i = 0; i < f.size(); ++i) {
    if (i) s += ",";
    s += "\"" + f[i].first + "\":" + f[i].second;
  }
  return s + "}";
}

Fields command_fields() {
  return {{"protocol", "3"},      {"type", "\"command\""},        {"client_id", "\"" + kCid + "\""},
          {"sequence", "123"},    {"control_active", "true"},     {"forward", "0.6"},
          {"strafe", "0.0"},      {"turn", "-0.3"}};
}

Fields arm_fields() {
  return {{"protocol", "3"},        {"type", "\"arm_command\""}, {"client_id", "\"" + kCid + "\""},
          {"sequence", "124"},      {"control_active", "true"},  {"x", "0.15"},
          {"y", "0.0"},             {"z", "0.12"},               {"pitch", "-1.5708"},
          {"gripper", "0.0"}};
}

Fields event_fields(const std::string& type) {
  return {{"protocol", "3"}, {"type", "\"" + type + "\""}, {"client_id", "\"" + kCid + "\""}, {"sequence", "7"}};
}

Fields with(Fields f, const std::string& key, const std::string& value) {
  for (auto& kv : f) {
    if (kv.first == key) {
      kv.second = value;
      return f;
    }
  }
  f.emplace_back(key, value);
  return f;
}

Fields without(Fields f, const std::string& key) {
  for (auto it = f.begin(); it != f.end(); ++it) {
    if (it->first == key) {
      f.erase(it);
      break;
    }
  }
  return f;
}

Reason reason_of(const std::string& data) {
  const ParseResult r = parse_packet(data);
  return r.ok() ? Reason::kOk : r.error.reason;
}

}  // namespace

// ───── 정상 예시 ─────

TEST(Protocol, SpecCommandExample) {
  const auto r = parse_packet(build(command_fields()));
  ASSERT_TRUE(r.ok()) << r.error.message;
  const Packet& p = *r.packet;
  EXPECT_EQ(p.type, PacketType::kCommand);
  EXPECT_EQ(p.client_id, kCid);
  EXPECT_EQ(p.sequence, 123);
  EXPECT_TRUE(p.control_active);
  EXPECT_DOUBLE_EQ(p.forward, 0.6);
  EXPECT_DOUBLE_EQ(p.strafe, 0.0);
  EXPECT_DOUBLE_EQ(p.turn, -0.3);
  EXPECT_EQ(p.channel(), Channel::kBase);
  EXPECT_FALSE(p.is_neutral());
}

TEST(Protocol, SpecArmCommandExample) {
  const auto r = parse_packet(build(arm_fields()));
  ASSERT_TRUE(r.ok()) << r.error.message;
  const Packet& p = *r.packet;
  EXPECT_EQ(p.type, PacketType::kArmCommand);
  EXPECT_DOUBLE_EQ(p.x, 0.15);
  EXPECT_DOUBLE_EQ(p.z, 0.12);
  EXPECT_DOUBLE_EQ(p.pitch, -1.5708);
  EXPECT_DOUBLE_EQ(p.gripper, 0.0);
  EXPECT_EQ(p.channel(), Channel::kArm);
}

TEST(Protocol, EventsParse) {
  for (const auto& [name, type] : std::map<std::string, PacketType>{{"emergency_stop", PacketType::kEmergencyStop},
                                                                     {"clear_emergency", PacketType::kClearEmergency},
                                                                     {"disconnect", PacketType::kDisconnect}}) {
    const auto r = parse_packet(build(event_fields(name)));
    ASSERT_TRUE(r.ok()) << name;
    EXPECT_EQ(r.packet->type, type);
    EXPECT_EQ(r.packet->channel(), Channel::kBase);
  }
}

TEST(Protocol, NeutralDefinitions) {
  auto p = parse_packet(build(with(with(with(command_fields(), "forward", "0"), "strafe", "0"), "turn", "-0.0")));
  ASSERT_TRUE(p.ok());
  EXPECT_TRUE(p.packet->is_neutral());  // 활성이면서 세 축 0
  p = parse_packet(build(with(command_fields(), "control_active", "false")));
  EXPECT_TRUE(p.packet->is_neutral());
  p = parse_packet(build(with(command_fields(), "strafe", "0.2")));
  EXPECT_FALSE(p.packet->is_neutral());
  p = parse_packet(build(with(arm_fields(), "control_active", "false")));
  EXPECT_TRUE(p.packet->is_neutral());
  p = parse_packet(build(arm_fields()));
  EXPECT_FALSE(p.packet->is_neutral());
}

TEST(Protocol, FieldOrderAndWhitespaceFree) {
  const std::string raw = "  {\"turn\": -0.3, \"strafe\":0, \"forward\": 0.6, \"control_active\": true, "
                          "\"sequence\": 1,\n \"client_id\": \"" + kCid + "\", \"type\": \"command\", \"protocol\": 3}  ";
  const auto r = parse_packet(raw);
  ASSERT_TRUE(r.ok());
  EXPECT_DOUBLE_EQ(r.packet->turn, -0.3);
}

// ───── protocol·type ─────

TEST(Protocol, ProtocolField) {
  EXPECT_EQ(reason_of(build(with(command_fields(), "protocol", "2"))), Reason::kUnsupportedProtocol);
  EXPECT_EQ(reason_of(build(with(command_fields(), "protocol", "4"))), Reason::kUnsupportedProtocol);
  EXPECT_EQ(reason_of(build(with(command_fields(), "protocol", "100000000000000000000000000000"))),
            Reason::kUnsupportedProtocol);
  EXPECT_EQ(reason_of(build(with(command_fields(), "protocol", "\"3\""))), Reason::kInvalidPacket);
  EXPECT_EQ(reason_of(build(with(command_fields(), "protocol", "true"))), Reason::kInvalidPacket);
  EXPECT_EQ(reason_of(build(with(command_fields(), "protocol", "3.0"))), Reason::kInvalidPacket);
  EXPECT_EQ(reason_of(build(with(command_fields(), "protocol", "null"))), Reason::kInvalidPacket);
  EXPECT_EQ(reason_of(build(without(command_fields(), "protocol"))), Reason::kInvalidPacket);
  // protocol 검사가 필드 집합 검사보다 먼저다
  EXPECT_EQ(reason_of(build({{"protocol", "2"}, {"type", "\"command\""}})), Reason::kUnsupportedProtocol);
}

TEST(Protocol, TypeField) {
  for (const char* t : {"\"drive\"", "null", "3", "\"Command\"", "\"ack\"", "[\"command\"]"}) {
    EXPECT_EQ(reason_of(build(with(command_fields(), "type", t))), Reason::kInvalidPacket) << t;
  }
}

// ───── 필드 집합 ─────

TEST(Protocol, EachMissingCommandFieldRejected) {
  for (const auto& kv : command_fields()) {
    const Reason r = reason_of(build(without(command_fields(), kv.first)));
    EXPECT_EQ(r, Reason::kInvalidPacket) << kv.first;
  }
}

TEST(Protocol, EachMissingArmFieldRejected) {
  for (const auto& kv : arm_fields()) {
    EXPECT_EQ(reason_of(build(without(arm_fields(), kv.first))), Reason::kInvalidPacket) << kv.first;
  }
}

TEST(Protocol, ExtraFieldsRejected) {
  EXPECT_EQ(reason_of(build(with(command_fields(), "extra", "1"))), Reason::kInvalidPacket);
  EXPECT_EQ(reason_of(build(with(arm_fields(), "roll", "0"))), Reason::kInvalidPacket);
  EXPECT_EQ(reason_of(build(with(event_fields("emergency_stop"), "forward", "0"))), Reason::kInvalidPacket);
  // v2 형식(strafe 없음)과 섞인 필드 집합
  EXPECT_EQ(reason_of(build(without(command_fields(), "strafe"))), Reason::kInvalidPacket);
  EXPECT_EQ(reason_of(build(with(with(command_fields(), "type", "\"arm_command\""), "x", "0"))),
            Reason::kInvalidPacket);
}

TEST(Protocol, DuplicateKeyRejected) {
  std::string raw = build(command_fields());
  raw.insert(1, "\"protocol\":3,");
  EXPECT_EQ(reason_of(raw), Reason::kInvalidPacket);
}

// ───── 값 자료형·범위 ─────

TEST(Protocol, ControlActiveMustBeBool) {
  for (const char* v : {"1", "0", "\"true\"", "null", "1.0"}) {
    EXPECT_EQ(reason_of(build(with(command_fields(), "control_active", v))), Reason::kInvalidPacket) << v;
    EXPECT_EQ(reason_of(build(with(arm_fields(), "control_active", v))), Reason::kInvalidPacket) << v;
  }
}

TEST(Protocol, AxesRejectBoolStringNullArrayAndOutOfRange) {
  for (const char* axis : {"forward", "strafe", "turn"}) {
    for (const char* v : {"true", "false", "\"0.5\"", "null", "[0.1]", "{\"v\":1}", "1.0000001", "-1.5", "2",
                          "-2", "100000000000000000000000000000", "1e400", "NaN", "Infinity"}) {
      EXPECT_EQ(reason_of(build(with(command_fields(), axis, v))), Reason::kInvalidPacket) << axis << "=" << v;
    }
  }
}

TEST(Protocol, AxesBoundaryAndExponentAccepted) {
  auto r = parse_packet(build(with(with(with(command_fields(), "forward", "1"), "strafe", "-1"), "turn", "1E-05")));
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.packet->forward, 1.0);
  EXPECT_EQ(r.packet->strafe, -1.0);
  EXPECT_DOUBLE_EQ(r.packet->turn, 1e-05);
  r = parse_packet(build(with(command_fields(), "forward", "1e-400")));  // Python: 0.0
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.packet->forward, 0.0);
}

TEST(Protocol, ArmCoordinatesMustBeFiniteNumbers) {
  for (const char* axis : {"x", "y", "z"}) {
    for (const char* v : {"true", "\"0.1\"", "null", "[0]", "NaN", "1e400", "-1e400"}) {
      EXPECT_EQ(reason_of(build(with(arm_fields(), axis, v))), Reason::kInvalidPacket) << axis << "=" << v;
    }
  }
  // 범위 밖 좌표는 형식상 유효(작업 공간 판정은 세션의 workspace_limit)
  auto r = parse_packet(build(with(with(arm_fields(), "x", "5"), "z", "-100.5")));
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.packet->x, 5.0);
  r = parse_packet(build(with(arm_fields(), "y", "100000000000000000000000000000")));
  ASSERT_TRUE(r.ok());  // double 로 표현 가능한 큰 정수
}

TEST(Protocol, PitchRange) {
  EXPECT_TRUE(parse_packet(build(with(arm_fields(), "pitch", "3.141592653589793"))).ok());
  EXPECT_TRUE(parse_packet(build(with(arm_fields(), "pitch", "-3.141592653589793"))).ok());
  EXPECT_TRUE(parse_packet(build(with(arm_fields(), "pitch", "0"))).ok());
  for (const char* v : {"3.1416", "-3.1416", "4", "true", "\"0\"", "null"}) {
    EXPECT_EQ(reason_of(build(with(arm_fields(), "pitch", v))), Reason::kInvalidPacket) << v;
  }
}

TEST(Protocol, GripperRange) {
  EXPECT_TRUE(parse_packet(build(with(arm_fields(), "gripper", "0"))).ok());
  EXPECT_TRUE(parse_packet(build(with(arm_fields(), "gripper", "1"))).ok());
  EXPECT_TRUE(parse_packet(build(with(arm_fields(), "gripper", "0.5"))).ok());
  for (const char* v : {"-0.01", "1.01", "2", "-1", "true", "\"1\"", "null"}) {
    EXPECT_EQ(reason_of(build(with(arm_fields(), "gripper", v))), Reason::kInvalidPacket) << v;
  }
}

TEST(Protocol, SequenceRules) {
  for (const char* v : {"true", "1.0", "2.0", "1e3", "\"1\"", "-1", "9007199254740992", "null"}) {
    EXPECT_EQ(reason_of(build(with(command_fields(), "sequence", v))), Reason::kInvalidPacket) << v;
  }
  EXPECT_TRUE(parse_packet(build(with(command_fields(), "sequence", "9007199254740991"))).ok());
  EXPECT_TRUE(parse_packet(build(with(command_fields(), "sequence", "0"))).ok());
}

TEST(Protocol, ClientIdRules) {
  const std::vector<std::string> bad = {
      "123", "\"not-a-uuid\"", "\"00000000-0000-0000-0000-000000000000\"", "\"D8B27F1009A84C21B6491E7058D46B72\"",
      "\"{" + kCid + "}\"", "null", "\"d8b27f10-09a8-4c21-b649-1e7058d46b7g\"",
      "\"d8b27f10_09a8-4c21-b649-1e7058d46b72\"", "\"urn:uuid:" + kCid + "\""};
  for (const std::string& v : bad) {
    EXPECT_EQ(reason_of(build(with(command_fields(), "client_id", v))), Reason::kInvalidPacket) << v;
  }
  // 대문자 UUID 는 허용, 비교용 session_id 는 소문자, ACK 에는 받은 그대로
  const std::string upper = "D8B27F10-09A8-4C21-B649-1E7058D46B72";
  const auto r = parse_packet(build(with(command_fields(), "client_id", "\"" + upper + "\"")));
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.packet->client_id, upper);
  EXPECT_EQ(r.packet->session_id, kCid);
}

TEST(Protocol, MalformedDatagramsRejectedWithoutCrash) {
  const std::vector<std::string> cases = {
      "", "{", "[]", "null", "\xff\xfe", "{\"a\":1}", std::string(3000, 'x'),
      std::string(1000, '{') + std::string(1000, '}'), "\"string\"", "3"};
  for (const auto& c : cases) {
    const auto r = parse_packet(c);
    EXPECT_FALSE(r.ok());
    EXPECT_EQ(r.error.reason, Reason::kInvalidPacket);
    EXPECT_FALSE(r.error.client_id.has_value());
    EXPECT_EQ(r.error.sequence, -1);
  }
  // 2048 byte 경계: 정확히 2048 은 크기 검사 통과, 2049 는 거부
  std::string pad = build(with(command_fields(), "turn", "-0.3"));
  pad.insert(1, std::string(2048 - pad.size(), ' '));
  ASSERT_EQ(pad.size(), 2048u);
  EXPECT_TRUE(parse_packet(pad).ok());
  EXPECT_FALSE(parse_packet(" " + pad).ok());
}

TEST(Protocol, EchoIdentifiersOnRejection) {
  // client_id 누락 → null, sequence 는 되돌림
  auto r = parse_packet(build(without(command_fields(), "client_id")));
  EXPECT_FALSE(r.error.client_id.has_value());
  EXPECT_EQ(r.error.sequence, 123);
  // sequence 누락 → client_id 는 되돌리고 sequence -1
  r = parse_packet(build(without(command_fields(), "sequence")));
  ASSERT_TRUE(r.error.client_id.has_value());
  EXPECT_EQ(*r.error.client_id, kCid);
  EXPECT_EQ(r.error.sequence, -1);
  // 지원하지 않는 protocol 도 식별자를 되돌린다
  r = parse_packet(build(with(command_fields(), "protocol", "2")));
  EXPECT_EQ(r.error.reason, Reason::kUnsupportedProtocol);
  EXPECT_EQ(r.error.client_id.value_or(""), kCid);
  EXPECT_EQ(r.error.sequence, 123);
}

TEST(Protocol, RejectedArmPacketReportsArmChannel) {
  const auto r = parse_packet(build(with(arm_fields(), "gripper", "5")));
  EXPECT_EQ(r.error.channel, Channel::kArm);
  const auto r2 = parse_packet(build(with(command_fields(), "turn", "5")));
  EXPECT_EQ(r2.error.channel, Channel::kBase);
}

TEST(Protocol, CanonicalUuid) {
  EXPECT_EQ(canonical_uuid(kCid).value_or(""), kCid);
  EXPECT_FALSE(canonical_uuid("00000000-0000-0000-0000-000000000000"));
  EXPECT_FALSE(canonical_uuid(kCid.substr(1)));
}

TEST(Protocol, AckEncodingExactFieldsAndOrder) {
  Ack a;
  a.client_id = kCid;
  a.sequence = 124;
  a.accepted = true;
  a.reason = Reason::kOk;
  a.emergency_stop = false;
  a.state = ChannelState::kActive;
  EXPECT_EQ(encode_ack(a), "{\"protocol\":3,\"type\":\"ack\",\"client_id\":\"" + kCid +
                               "\",\"sequence\":124,\"accepted\":true,\"reason\":\"ok\","
                               "\"emergency_stop\":false,\"state\":\"active\"}");
  Ack n;  // 식별 불가 패킷
  n.reason = Reason::kInvalidPacket;
  EXPECT_EQ(encode_ack(n), "{\"protocol\":3,\"type\":\"ack\",\"client_id\":null,\"sequence\":-1,"
                           "\"accepted\":false,\"reason\":\"invalid_packet\",\"emergency_stop\":false,"
                           "\"state\":\"awaiting_neutral\"}");
}

TEST(Protocol, ReasonAndStateStrings) {
  EXPECT_STREQ(to_string(Reason::kWorkspaceLimit), "workspace_limit");
  EXPECT_STREQ(to_string(Reason::kIkUnreachable), "ik_unreachable");
  EXPECT_STREQ(to_string(Reason::kNeutralRequired), "neutral_required");
  EXPECT_STREQ(to_string(ChannelState::kAwaitingNeutral), "awaiting_neutral");
  EXPECT_STREQ(to_string(ChannelState::kEmergencyStop), "emergency_stop");
  EXPECT_STREQ(to_string(ChannelState::kTimeout), "timeout");
}
