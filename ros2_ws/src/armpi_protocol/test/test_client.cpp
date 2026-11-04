// V3Client(클라이언트 세션) 시험: 가짜 데이터그램 연결 + 가짜 시계로 Python ArmPiV3Client 와 같은 상태 전이를 확인한다.
#include <gtest/gtest.h>

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "armpi_common/json.hpp"
#include "armpi_protocol/client.hpp"

using namespace armpi_protocol;
namespace json = armpi_common::json;

namespace {

const std::string kCid = "d8b27f10-09a8-4c21-b649-1e7058d46b72";

struct FakeLink : DatagramLink {
  std::vector<std::string>* sent;
  std::deque<std::string>* inbox;
  FakeLink(std::vector<std::string>* s, std::deque<std::string>* i) : sent(s), inbox(i) {}
  bool send(const std::string& data, std::string*) override {
    sent->push_back(data);
    return true;
  }
  std::optional<std::string> receive(std::string*) override {
    if (inbox->empty()) return std::nullopt;
    std::string d = inbox->front();
    inbox->pop_front();
    return d;
  }
  std::string peer() const override { return "127.0.0.1:5001"; }
};

std::string ack(int64_t seq, bool accepted, const std::string& reason, bool estop, const std::string& state,
                const std::string& cid = kCid) {
  return "{\"protocol\":3,\"type\":\"ack\",\"client_id\":\"" + cid + "\",\"sequence\":" + std::to_string(seq) +
         ",\"accepted\":" + (accepted ? "true" : "false") + ",\"reason\":\"" + reason +
         "\",\"emergency_stop\":" + (estop ? "true" : "false") + ",\"state\":\"" + state + "\"}";
}

json::Value parse(const std::string& s) {
  json::Value v;
  std::string err;
  EXPECT_TRUE(json::parse(s, v, err)) << err << " : " << s;
  return v;
}

class ClientTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ClientOptions o;
    o.clock = [this] { return t; };
    o.new_uuid = [] { return kCid; };
    client = std::make_unique<V3Client>(std::make_unique<FakeLink>(&sent, &inbox), o);
  }
  // 마지막으로 보낸 패킷
  json::Value last(size_t back = 0) { return parse(sent.at(sent.size() - 1 - back)); }
  int64_t seq_of(size_t back = 0) { return last(back).find("sequence")->integer; }
  // 차체·팔 중립을 보내고 둘 다 idle 로 확인받아 무장 가능 상태로
  void confirm_neutral(bool base_intent, bool arm_intent) {
    client->tick(base_intent, 0, 0, 0, arm_intent);
    inbox.push_back(ack(seq_of(1), true, "ok", false, "idle"));  // command
    inbox.push_back(ack(seq_of(0), true, "ok", false, "idle"));  // arm_command
    t += 0.01;
    client->poll();
  }

  double t{100.0};
  std::vector<std::string> sent;
  std::deque<std::string> inbox;
  std::unique_ptr<V3Client> client;
};

}  // namespace

TEST_F(ClientTest, NeutralFirstThenArmsAndDrives) {
  client->tick(true, 0.5, 0, 0, false);
  ASSERT_EQ(sent.size(), 2u);  // command + arm_command
  EXPECT_FALSE(last(1).find("control_active")->boolean);
  EXPECT_EQ(last(1).find("type")->string, "command");
  EXPECT_EQ(last(0).find("type")->string, "arm_command");
  inbox.push_back(ack(seq_of(1), true, "ok", false, "idle"));
  inbox.push_back(ack(seq_of(0), true, "ok", false, "idle"));
  t += 0.05;
  client->tick(true, 0.5, -2.0, 0, false);  // strafe −2 → −1 로 자름
  const auto cmd = last(1);
  EXPECT_TRUE(cmd.find("control_active")->boolean);
  EXPECT_DOUBLE_EQ(cmd.find("forward")->number, 0.5);
  EXPECT_DOUBLE_EQ(cmd.find("strafe")->number, -1.0);
  EXPECT_TRUE(client->base().armed);
  EXPECT_FALSE(last(0).find("control_active")->boolean);  // 팔 의도 없음 → 팔은 중립
}

TEST_F(ClientTest, AckTimeoutBlocksThenAutoRearms) {
  confirm_neutral(true, false);
  t += 0.04;
  client->tick(true, 1, 0, 0, false);
  ASSERT_TRUE(client->base().armed);
  t += 0.31;  // ACK 없음
  client->tick(true, 1, 0, 0, false);
  EXPECT_FALSE(client->base().armed);
  EXPECT_EQ(client->base().last_block_reason, "ACK 300 ms 없음");
  EXPECT_FALSE(last(1).find("control_active")->boolean);
  // 의도를 유지한 채 중립 확인을 다시 받으면 자동 재무장
  inbox.push_back(ack(seq_of(1), true, "ok", false, "idle"));
  t += 0.01;
  client->tick(true, 1, 0, 0, false);
  EXPECT_TRUE(client->base().armed);
  EXPECT_EQ(client->base().auto_rearms, 1);
  EXPECT_NE(client->snapshot_json().find("\"link_drops\":1"), std::string::npos);
}

TEST_F(ClientTest, NeutralSentBeforeBlockIsNotAConfirmation) {
  confirm_neutral(true, false);
  t += 0.04;
  client->tick(true, 1, 0, 0, false);  // 활성
  const int64_t old_neutral_seq = 1;   // 첫 tick 의 중립
  inbox.push_back(ack(seq_of(1), false, "neutral_required", false, "timeout"));  // watchdog
  t += 0.01;
  client->poll();
  EXPECT_FALSE(client->base().armed);
  // 차단 전에 보낸 옛 중립의 늦은 ACK 는 확인으로 치지 않는다(이미 지난 번호라 무시)
  inbox.push_back(ack(old_neutral_seq, true, "ok", false, "idle"));
  client->poll();
  EXPECT_FALSE(client->base().neutral_confirmed);
}

TEST_F(ClientTest, EmergencyStopRetriesUntilAcked) {
  confirm_neutral(true, true);
  client->emergency_stop();
  EXPECT_EQ(last().find("type")->string, "emergency_stop");
  const int64_t first = seq_of();
  t += 0.05;
  client->tick(true, 1, 0, 0, true);
  // 재전송(새 번호) + 중립 차체·팔
  ASSERT_GE(sent.size(), 3u);
  EXPECT_EQ(last(2).find("type")->string, "emergency_stop");
  EXPECT_GT(last(2).find("sequence")->integer, first);
  EXPECT_FALSE(last(1).find("control_active")->boolean);
  inbox.push_back(ack(last(2).find("sequence")->integer, true, "ok", true, "emergency_stop"));
  client->poll();
  EXPECT_FALSE(client->pending_estop());
  EXPECT_TRUE(client->base().needs_release);  // 의도 중이었으므로 손을 한 번 떼야 함
  const size_t n = sent.size();
  t += 0.05;
  client->tick(true, 1, 0, 0, true);
  EXPECT_EQ(sent.size(), n + 2);  // 더 이상 재전송하지 않음
}

TEST_F(ClientTest, ArmTargetRejectRevertsToLastAccepted) {
  confirm_neutral(false, true);
  t += 0.04;
  client->set_arm_target(0.20, 0.0, 0.10, -0.5, 0.3);
  client->tick(false, 0, 0, 0, true);
  ASSERT_TRUE(last(0).find("control_active")->boolean);
  inbox.push_back(ack(seq_of(0), true, "ok", false, "active"));
  t += 0.01;
  client->poll();
  client->move_arm_target(0.5, 0, 0, 0, 0);  // 작업 공간 밖
  t += 0.04;
  client->tick(false, 0, 0, 0, true);
  inbox.push_back(ack(seq_of(0), false, "workspace_limit", false, "active"));
  t += 0.01;
  client->poll();
  EXPECT_DOUBLE_EQ(client->arm_target().x, 0.20);
  EXPECT_TRUE(client->arm().armed);  // 채널은 풀지 않는다
  EXPECT_NE(client->snapshot_json().find("\"arm_last_reject\":{\"reason\":\"workspace_limit\""), std::string::npos);
}

TEST_F(ClientTest, NotOwnerIsHardBlockOnBothChannels) {
  confirm_neutral(true, true);
  t += 0.04;
  client->tick(true, 1, 0, 0, true);
  inbox.push_back(ack(seq_of(1), false, "not_owner", false, "idle"));
  t += 0.01;
  client->poll();
  EXPECT_FALSE(client->base().armed);
  EXPECT_TRUE(client->base().needs_release);
  EXPECT_TRUE(client->arm().needs_release);
}

TEST_F(ClientTest, DisconnectRetiresAndStopsSending) {
  confirm_neutral(true, false);
  client->disconnect();
  EXPECT_EQ(last().find("type")->string, "disconnect");
  EXPECT_FALSE(last(2).find("control_active")->boolean);
  EXPECT_TRUE(client->retired());
  const size_t n = sent.size();
  t += 0.05;
  client->tick(true, 1, 0, 0, false);
  EXPECT_EQ(sent.size(), n);
  EXPECT_FALSE(client->clear_emergency());
}

TEST_F(ClientTest, IgnoresOtherClientAndBadAcks) {
  client->tick(true, 0, 0, 0, false);
  inbox.push_back(ack(seq_of(1), true, "ok", false, "idle", "11111111-2222-4333-8444-555555555555"));
  inbox.push_back("{\"protocol\":3}");
  t += 0.01;
  client->poll();
  EXPECT_FALSE(client->base().neutral_confirmed);
  EXPECT_NE(client->snapshot_json().find("\"bad_acks\":1"), std::string::npos);
}

TEST_F(ClientTest, SnapshotHasPythonKeys) {
  const auto v = parse(client->snapshot_json());
  for (const char* k : {"robot", "client_id", "sequence", "connected", "emergency", "estop_pending", "faulted",
                        "retired", "base", "arm", "arm_target", "arm_last_accepted", "arm_last_reject", "ack_ms_last",
                        "ack_ms_p50", "ack_ms_max", "packets_sent", "acks", "bad_acks", "late_acks", "null_acks",
                        "send_errors", "link_drops", "last_error", "ack_timeout_s"}) {
    EXPECT_NE(v.find(k), nullptr) << k;
  }
  EXPECT_EQ(v.object.size(), 25u);
  EXPECT_EQ(v.find("base")->object.size(), 9u);
}

TEST(ClientUuid, RandomV4Format) {
  const std::string u = random_uuid_v4();
  ASSERT_EQ(u.size(), 36u);
  EXPECT_EQ(u[14], '4');
  EXPECT_TRUE(canonical_uuid(u).has_value());
  EXPECT_NE(u, random_uuid_v4());
}
