// 클라이언트 쪽 인코딩·ACK 해석 시험. 서버 쪽 parse_packet·encode_ack 와 왕복이 맞는지도 확인한다.
#include <gtest/gtest.h>

#include <string>

#include "armpi_protocol/client_codec.hpp"

using namespace armpi_protocol;

namespace {
const std::string kCid = "d8b27f10-09a8-4c21-b649-1e7058d46b72";

std::string ack_text(const std::string& body) { return "{" + body + "}"; }
const std::string kGood =
    "\"protocol\":3,\"type\":\"ack\",\"client_id\":\"" + kCid +
    "\",\"sequence\":7,\"accepted\":true,\"reason\":\"ok\",\"emergency_stop\":false,\"state\":\"idle\"";
}  // namespace

TEST(ClientCodec, CommandRoundTripsThroughServerParser) {
  const auto r = parse_packet(encode_command(kCid, 5, true, 0.5, -1.0, 0.25));
  ASSERT_TRUE(r.ok()) << r.error.message;
  EXPECT_EQ(r.packet->type, PacketType::kCommand);
  EXPECT_EQ(r.packet->sequence, 5);
  EXPECT_TRUE(r.packet->control_active);
  EXPECT_DOUBLE_EQ(r.packet->strafe, -1.0);
  EXPECT_EQ(encode_command(kCid, 1, false, 0, 0, 0),
            "{\"protocol\":3,\"type\":\"command\",\"client_id\":\"" + kCid +
                "\",\"sequence\":1,\"control_active\":false,\"forward\":0,\"strafe\":0,\"turn\":0}");
}

TEST(ClientCodec, ArmCommandAndEventsRoundTrip) {
  const auto r = parse_packet(encode_arm_command(kCid, 9, false, kDefaultHome));
  ASSERT_TRUE(r.ok()) << r.error.message;
  EXPECT_DOUBLE_EQ(r.packet->x, 0.1047);
  EXPECT_DOUBLE_EQ(r.packet->pitch, -1.0263);
  for (PacketType t : {PacketType::kEmergencyStop, PacketType::kClearEmergency, PacketType::kDisconnect}) {
    const auto e = parse_packet(encode_event(t, kCid, 3));
    ASSERT_TRUE(e.ok()) << to_string(t) << ": " << e.error.message;
    EXPECT_EQ(e.packet->type, t);
  }
}

TEST(ClientCodec, ParsesServerAck) {
  Ack a;
  a.client_id = kCid;
  a.sequence = 12;
  a.accepted = false;
  a.reason = Reason::kIkUnreachable;
  a.emergency_stop = true;
  a.state = ChannelState::kActive;
  const auto m = parse_ack(encode_ack(a));
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(*m->client_id, kCid);
  EXPECT_EQ(m->sequence, 12);
  EXPECT_EQ(m->reason, Reason::kIkUnreachable);
  EXPECT_EQ(m->state, ChannelState::kActive);
  EXPECT_TRUE(m->emergency_stop);
  a.client_id.reset();
  a.sequence = -1;
  const auto n = parse_ack(encode_ack(a));
  ASSERT_TRUE(n.has_value());
  EXPECT_FALSE(n->client_id.has_value());
}

TEST(ClientCodec, RejectsMalformedAcks) {
  EXPECT_TRUE(parse_ack(ack_text(kGood)).has_value());
  EXPECT_FALSE(parse_ack(ack_text(kGood + ",\"extra\":1")));                                  // 추가 키
  EXPECT_FALSE(parse_ack(ack_text(kGood + ",\"state\":\"idle\"")));                         // 중복 키
  EXPECT_FALSE(parse_ack("{\"protocol\":3,\"type\":\"ack\",\"client_id\":\"" + kCid +
                         "\",\"sequence\":7.0,\"accepted\":true,\"reason\":\"ok\",\"emergency_stop\":false,"
                         "\"state\":\"idle\"}"));                                                // 실수
  EXPECT_FALSE(parse_ack(
      "{\"protocol\":2,\"type\":\"ack\",\"client_id\":null,\"sequence\":7,\"accepted\":true,\"reason\":\"ok\","
      "\"emergency_stop\":false,\"state\":\"idle\"}"));                                          // 다른 버전
  EXPECT_FALSE(parse_ack(
      "{\"protocol\":3,\"type\":\"ack\",\"client_id\":\"a b\",\"sequence\":7,\"accepted\":true,\"reason\":\"ok\","
      "\"emergency_stop\":false,\"state\":\"idle\"}"));                                          // 허용 밖 문자
  EXPECT_FALSE(parse_ack(
      "{\"protocol\":3,\"type\":\"ack\",\"client_id\":null,\"sequence\":7,\"accepted\":true,\"reason\":\"maybe\","
      "\"emergency_stop\":false,\"state\":\"idle\"}"));                                          // 모르는 reason
  EXPECT_FALSE(parse_ack(
      "{\"protocol\":3,\"type\":\"ack\",\"client_id\":null,\"sequence\":-2,\"accepted\":true,\"reason\":\"ok\","
      "\"emergency_stop\":false,\"state\":\"idle\"}"));                                          // 범위
  EXPECT_FALSE(parse_ack(""));
}
