// Protocol v3 세션 상태기계 단위 시험. 시계를 직접 넘겨(now) 시간 의존 규칙을 결정적으로 확인한다.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "armpi_vr_bridge/session.hpp"
#include "armpi_common/json.hpp"

using namespace armpi_vr_bridge;
namespace json = armpi_common::json;

namespace {

const std::string kCid = "d8b27f10-09a8-4c21-b649-1e7058d46b72";
const std::string kOther = "0f0e0d0c-0b0a-4908-8706-050403020100";

SessionConfig default_cfg() {
  SessionConfig c;  // 기본: watchdog 0.4/0.4, lease 1.0, control_state 2.0, 0.10/0.10/0.60
  c.workspace = {0.0, 0.30, -0.25, 0.25, -0.05, 0.35};
  c.kinematics.base_height_m = 0.10;  // 시험용 placeholder 치수
  c.kinematics.upper_arm_m = 0.10;
  c.kinematics.forearm_m = 0.095;
  c.kinematics.hand_m = 0.12;
  return c;
}

std::string num(double v) { return json::number(v); }

std::string cmd_json(int64_t seq, bool active, double f, double s, double t, const std::string& cid = kCid) {
  return "{\"protocol\":3,\"type\":\"command\",\"client_id\":\"" + cid + "\",\"sequence\":" + std::to_string(seq) +
         ",\"control_active\":" + (active ? "true" : "false") + ",\"forward\":" + num(f) + ",\"strafe\":" + num(s) +
         ",\"turn\":" + num(t) + "}";
}

std::string arm_json(int64_t seq, bool active, double x, double y, double z, double pitch, double grip,
                     const std::string& cid = kCid) {
  return "{\"protocol\":3,\"type\":\"arm_command\",\"client_id\":\"" + cid + "\",\"sequence\":" +
         std::to_string(seq) + ",\"control_active\":" + (active ? "true" : "false") + ",\"x\":" + num(x) +
         ",\"y\":" + num(y) + ",\"z\":" + num(z) + ",\"pitch\":" + num(pitch) + ",\"gripper\":" + num(grip) + "}";
}

std::string ev_json(const std::string& type, int64_t seq, const std::string& cid = kCid) {
  return "{\"protocol\":3,\"type\":\"" + type + "\",\"client_id\":\"" + cid + "\",\"sequence\":" +
         std::to_string(seq) + "}";
}

const Endpoint kAddr = Endpoint::from("192.168.0.10", 50000);
const Endpoint kOtherAddr = Endpoint::from("192.168.0.11", 40000);

// 가짜 시계 하네스. auto_control=true 면 패킷마다 정상 control_state 를 먼저 넣는다(게이트 허용 유지).
struct H {
  std::unique_ptr<Session> s;
  double now{100.0};
  int64_t seq{0};
  bool auto_control{true};
  ControlState cs{false, false, "", true};

  explicit H(bool control = true, const SessionConfig& cfg = default_cfg()) : auto_control(control) {
    s = std::make_unique<Session>(cfg);
  }
  HandleResult send(const std::string& data, const Endpoint& a = kAddr, double dt = 0.05) {
    now += dt;
    if (auto_control) s->set_control_state(cs, now);
    return s->handle_datagram(data, a, now);
  }
  HandleResult base(bool active, double f = 0, double st = 0, double t = 0, double dt = 0.05) {
    return send(cmd_json(++seq, active, f, st, t), kAddr, dt);
  }
  HandleResult base_neutral(double dt = 0.05) { return base(false, 0, 0, 0, dt); }
  HandleResult arm(bool active, double x = 0.15, double y = 0.0, double z = 0.05, double pitch = -M_PI / 2,
                   double grip = 0.5, double dt = 0.05) {
    return send(arm_json(++seq, active, x, y, z, pitch, grip), kAddr, dt);
  }
  HandleResult arm_neutral(double dt = 0.05) { return arm(false, 0, 0, 0, 0, 0, dt); }
  HandleResult event(const std::string& type, const Endpoint& a = kAddr, const std::string& cid = kCid) {
    return send(ev_json(type, ++seq, cid), a);
  }
  std::vector<std::string> tick(double dt) {
    now += dt;
    return s->check_timeouts(now);
  }
};

::testing::AssertionResult AckIs(const HandleResult& r, bool accepted, Reason reason,
                               std::optional<ChannelState> state = std::nullopt) {
  if (r.ack.accepted != accepted || r.ack.reason != reason || (state && r.ack.state != *state)) {
    return ::testing::AssertionFailure() << "ACK " << encode_ack(r.ack) << " (기대 accepted=" << accepted
                                         << " reason=" << to_string(reason)
                                         << " state=" << (state ? to_string(*state) : "*") << ", 설명 " << r.detail
                                         << ")";
  }
  return ::testing::AssertionSuccess();
}

bool has_event(const HandleResult& r, const std::string& needle) {
  for (const auto& e : r.events) {
    if (e.find(needle) != std::string::npos) return true;
  }
  return false;
}

}  // namespace

// ───────────── 차체 채널: 중립·활성·변환 ─────────────

TEST(SessionBase, NeutralRequiredBeforeMotionAndVelocityMapping) {
  H h;
  EXPECT_TRUE(AckIs(h.base(true, 0.6), false, Reason::kNeutralRequired, ChannelState::kAwaitingNeutral));
  EXPECT_TRUE(h.s->base_output().is_zero());
  EXPECT_TRUE(AckIs(h.base_neutral(), true, Reason::kOk, ChannelState::kIdle));
  const auto r = h.base(true, 0.6, 0.2, -0.3);
  EXPECT_TRUE(AckIs(r, true, Reason::kOk, ChannelState::kActive));
  EXPECT_NEAR(h.s->base_output().vx, 0.06, 1e-12);
  EXPECT_NEAR(h.s->base_output().vy, 0.02, 1e-12);
  EXPECT_NEAR(h.s->base_output().wz, -0.18, 1e-12);
  // control_active=false 면 값이 남아 있어도 정지
  EXPECT_TRUE(AckIs(h.base(false, 0.6, 0.2, -0.3), true, Reason::kOk, ChannelState::kIdle));
  EXPECT_TRUE(h.s->base_output().is_zero());
}

TEST(SessionBase, ActiveNeutralKeepsActiveStateWithZeroTarget) {
  H h;
  h.base_neutral();
  h.base(true, 1.0);
  const auto r = h.base(true, 0.0, 0.0, 0.0);
  EXPECT_TRUE(AckIs(r, true, Reason::kOk, ChannelState::kActive));
  EXPECT_TRUE(h.s->base_output().is_zero());
}

TEST(SessionBase, StrafeIsLeftPositiveAndScaled) {
  H h;
  h.base_neutral();
  h.base(true, 0.0, 1.0, 0.0);
  EXPECT_NEAR(h.s->base_output().vy, 0.10, 1e-12);  // + 왼쪽 (REP-103)
  h.base(true, -1.0, -0.5, 1.0);
  EXPECT_NEAR(h.s->base_output().vx, -0.10, 1e-12);
  EXPECT_NEAR(h.s->base_output().vy, -0.05, 1e-12);
  EXPECT_NEAR(h.s->base_output().wz, 0.60, 1e-12);  // + 좌회전
}

// ───────────── sequence ─────────────

TEST(SessionSequence, DuplicateReorderedAndGap) {
  H h;
  EXPECT_TRUE(AckIs(h.send(cmd_json(10, false, 0, 0, 0)), true, Reason::kOk, ChannelState::kIdle));
  EXPECT_TRUE(AckIs(h.send(cmd_json(11, true, 0.5, 0, 0)), true, Reason::kOk, ChannelState::kActive));
  EXPECT_TRUE(AckIs(h.send(cmd_json(11, true, 1.0, 0, 0)), false, Reason::kOutOfOrder));  // 중복
  EXPECT_NEAR(h.s->base_output().vx, 0.05, 1e-12);
  EXPECT_TRUE(AckIs(h.send(cmd_json(9, true, -1.0, 0, 0)), false, Reason::kOutOfOrder));  // 옛 명령
  EXPECT_TRUE(AckIs(h.send(cmd_json(20, true, 0.2, 0, 0)), true, Reason::kOk));           // 유실 허용
  EXPECT_NEAR(h.s->base_output().vx, 0.02, 1e-12);
}

TEST(SessionSequence, SharedByCommandArmCommandAndEvents) {
  H h;
  EXPECT_TRUE(AckIs(h.send(cmd_json(1, false, 0, 0, 0)), true, Reason::kOk));
  EXPECT_TRUE(AckIs(h.send(arm_json(2, false, 0, 0, 0, 0, 0)), true, Reason::kOk));
  EXPECT_TRUE(AckIs(h.send(cmd_json(2, false, 0, 0, 0)), false, Reason::kOutOfOrder));
  EXPECT_TRUE(AckIs(h.send(ev_json("emergency_stop", 3)), true, Reason::kOk));
  EXPECT_TRUE(AckIs(h.send(arm_json(3, false, 0, 0, 0, 0, 0)), false, Reason::kOutOfOrder));
  EXPECT_TRUE(AckIs(h.send(arm_json(4, false, 0, 0, 0, 0, 0)), true, Reason::kOk, ChannelState::kEmergencyStop));
}

TEST(SessionSequence, Limits) {
  H h;
  EXPECT_TRUE(AckIs(h.send(cmd_json(kMaxSequence - 1, false, 0, 0, 0)), true, Reason::kOk));
  EXPECT_TRUE(AckIs(h.send(cmd_json(kMaxSequence, false, 0, 0, 0)), true, Reason::kOk));
  EXPECT_TRUE(AckIs(h.send(cmd_json(kMaxSequence + 1, false, 0, 0, 0)), false, Reason::kInvalidPacket));
}

// ───────────── 차체 watchdog ─────────────

TEST(SessionBase, WatchdogBoundaryStopsAndRequiresNewNeutral) {
  H h;
  h.base_neutral();
  h.base(true, 1.0);
  EXPECT_TRUE(h.tick(0.39).empty());
  EXPECT_EQ(h.s->base_state(), ChannelState::kActive);
  const auto ev = h.tick(0.011);  // 0.401 s (경계 직후)
  ASSERT_FALSE(ev.empty());
  EXPECT_NE(ev[0].find("watchdog"), std::string::npos);
  EXPECT_EQ(h.s->base_state(), ChannelState::kTimeout);
  EXPECT_TRUE(h.s->base_output().is_zero());
  EXPECT_TRUE(AckIs(h.base(true, 1.0, 0, 0, 0.01), false, Reason::kNeutralRequired, ChannelState::kTimeout));
  EXPECT_TRUE(AckIs(h.base_neutral(0.01), true, Reason::kOk, ChannelState::kIdle));
}

TEST(SessionBase, IdleChannelAlsoTimesOut) {
  H h;
  h.base_neutral();
  h.tick(0.41);
  EXPECT_EQ(h.s->base_state(), ChannelState::kTimeout);
}

TEST(SessionBase, ExpiryIsCheckedBeforeLatePacketIsApplied) {
  H h;
  h.base_neutral();
  h.base(true, 1.0);
  const auto r = h.base(true, 1.0, 0, 0, 0.41);  // 만료 직후 도착한 활성 명령이 만료 구간을 지우지 못한다
  EXPECT_TRUE(AckIs(r, false, Reason::kNeutralRequired, ChannelState::kTimeout));
  EXPECT_TRUE(has_event(r, "watchdog"));
  EXPECT_TRUE(h.s->base_output().is_zero());
}

TEST(SessionBase, InvalidAndRejectedPacketsDoNotRefreshWatchdog) {
  H h;
  h.base_neutral();
  h.base(true, 1.0);
  for (int i = 0; i < 4; ++i) h.send("{broken", kAddr, 0.05);  // 0.20 s: 아직 만료 전
  EXPECT_EQ(h.s->base_state(), ChannelState::kActive);
  h.send(cmd_json(h.seq, true, 1.0, 0, 0), kAddr, 0.05);  // 중복(out_of_order)
  h.send(cmd_json(h.seq + 100, true, 1.0, 0, 0, kOther), kOtherAddr, 0.05);  // 비소유자
  h.send(ev_json("emergency_stop", ++h.seq).substr(0, 10), kAddr, 0.05);     // 깨진 이벤트
  h.send("{broken", kAddr, 0.06);  // 누적 0.41 s
  EXPECT_EQ(h.s->base_state(), ChannelState::kTimeout);
  EXPECT_TRUE(h.s->base_output().is_zero());
}

// ───────────── 팔 채널 ─────────────

TEST(SessionArm, ChannelsAreIndependentAndAckReportsOwnChannel) {
  H h;
  h.base_neutral();
  EXPECT_TRUE(AckIs(h.base(true, 0.5), true, Reason::kOk, ChannelState::kActive));
  // 팔은 아직 중립 확인 전
  EXPECT_TRUE(AckIs(h.arm(true), false, Reason::kNeutralRequired, ChannelState::kAwaitingNeutral));
  const auto n = h.arm_neutral();
  EXPECT_TRUE(AckIs(n, true, Reason::kOk, ChannelState::kIdle));
  EXPECT_TRUE(n.arm_accepted);
  const auto r = h.arm(true, 0.15, 0.02, 0.05, -M_PI / 2, 0.7);
  EXPECT_TRUE(AckIs(r, true, Reason::kOk, ChannelState::kActive));
  EXPECT_TRUE(r.arm_accepted);
  const ArmOutput a = h.s->arm_output();
  EXPECT_TRUE(a.active);
  EXPECT_TRUE(a.has_goal);
  EXPECT_DOUBLE_EQ(a.x, 0.15);
  EXPECT_DOUBLE_EQ(a.y, 0.02);
  EXPECT_DOUBLE_EQ(a.z, 0.05);
  EXPECT_DOUBLE_EQ(a.pitch, -M_PI / 2);
  EXPECT_DOUBLE_EQ(a.gripper, 0.7);
  // 차체 ACK 는 차체 채널 상태
  EXPECT_TRUE(AckIs(h.base(false), true, Reason::kOk, ChannelState::kIdle));
  EXPECT_EQ(h.s->arm_state(), ChannelState::kActive);
}

TEST(SessionArm, ArmNeutralHoldsAndIgnoresCoordinates) {
  H h;
  h.arm_neutral();
  h.arm(true, 0.15, 0.0, 0.05);
  const auto r = h.arm(false, 5.0, -9.0, 7.0, 0.0, 0.0);  // 중립: 좌표 무시(작업 공간 밖이어도 수락)
  EXPECT_TRUE(AckIs(r, true, Reason::kOk, ChannelState::kIdle));
  const ArmOutput a = h.s->arm_output();
  EXPECT_FALSE(a.active);
  EXPECT_DOUBLE_EQ(a.x, 0.15);  // 마지막 목표 유지(정보용)
}

TEST(SessionArm, BasePacketsDoNotRefreshArmWatchdog) {
  H h;
  h.base_neutral();
  h.arm_neutral();
  h.arm(true);
  for (int i = 0; i < 9; ++i) h.base(true, 0.3);  // 0.45 s 동안 차체만
  EXPECT_EQ(h.s->arm_state(), ChannelState::kTimeout);
  EXPECT_FALSE(h.s->arm_output().active);
  EXPECT_TRUE(h.s->arm_output().has_goal);  // 목표 값은 남기고 active=false(현재 위치 유지)
  EXPECT_EQ(h.s->base_state(), ChannelState::kActive);
  EXPECT_EQ(h.s->owner().value_or(""), kCid);  // lease 는 차체 패킷으로 유지
}

TEST(SessionArm, WatchdogExpiryHoldsAndRequiresNeutral) {
  H h;
  h.arm_neutral();
  h.arm(true);
  EXPECT_TRUE(h.tick(0.39).empty());
  const auto ev = h.tick(0.011);
  ASSERT_FALSE(ev.empty());
  EXPECT_NE(ev[0].find("팔 watchdog"), std::string::npos);
  EXPECT_EQ(h.s->arm_state(), ChannelState::kTimeout);
  EXPECT_TRUE(AckIs(h.arm(true, 0.15, 0, 0.05, -M_PI / 2, 0.5, 0.01), false, Reason::kNeutralRequired,
                  ChannelState::kTimeout));
  EXPECT_TRUE(AckIs(h.arm_neutral(0.01), true, Reason::kOk, ChannelState::kIdle));
}

TEST(SessionArm, WorkspaceLimitKeepsPreviousGoalAndDoesNotRefreshWatchdog) {
  H h;
  h.arm_neutral();
  h.arm(true, 0.15, 0.0, 0.05, -M_PI / 2, 0.2);
  const auto r = h.arm(true, 0.31, 0.0, 0.12);  // x_max 0.30 초과
  EXPECT_TRUE(AckIs(r, false, Reason::kWorkspaceLimit, ChannelState::kActive));
  EXPECT_FALSE(r.arm_accepted);
  ArmOutput a = h.s->arm_output();
  EXPECT_TRUE(a.active);
  EXPECT_DOUBLE_EQ(a.x, 0.15);
  EXPECT_DOUBLE_EQ(a.gripper, 0.2);
  for (const auto& bad : std::vector<std::array<double, 3>>{{-0.01, 0, 0.1}, {0.1, 0.26, 0.1}, {0.1, -0.26, 0.1},
                                                             {0.1, 0, -0.06}, {0.1, 0, 0.36}}) {
    EXPECT_TRUE(AckIs(h.arm(true, bad[0], bad[1], bad[2], 0.0, 0.0, 0.05), false, Reason::kWorkspaceLimit));
  }
  // 수락 뒤 0.30 s 동안 거부만 → 아직 active, 0.11 s 더 → 만료
  EXPECT_EQ(h.s->arm_state(), ChannelState::kActive);
  h.arm(true, 0.5, 0.0, 0.12, 0.0, 0.0, 0.11);
  EXPECT_EQ(h.s->arm_state(), ChannelState::kTimeout);
}

TEST(SessionArm, IkUnreachableInsideWorkspace) {
  H h;
  h.arm_neutral();
  h.arm(true, 0.15, 0.0, 0.05);
  // 상자 안이지만 placeholder 팔 길이(0.315 m)로 닿지 않는 점
  const auto r = h.arm(true, 0.30, 0.25, 0.35, 0.0, 0.0);
  EXPECT_TRUE(AckIs(r, false, Reason::kIkUnreachable, ChannelState::kActive));
  EXPECT_DOUBLE_EQ(h.s->arm_output().x, 0.15);
}

TEST(SessionArm, JointLimitIsReportedAsIkUnreachable) {
  SessionConfig cfg = default_cfg();
  cfg.kinematics.joint_min[0] = -0.1;  // 베이스 회전을 좁게
  cfg.kinematics.joint_max[0] = 0.1;
  H h(true, cfg);
  h.arm_neutral();
  EXPECT_TRUE(AckIs(h.arm(true, 0.15, 0.0, 0.1, -M_PI / 4), true, Reason::kOk));
  EXPECT_TRUE(AckIs(h.arm(true, 0.15, 0.15, 0.1, -M_PI / 4), false, Reason::kIkUnreachable));
}

TEST(SessionArm, InvalidArmPacketAckUsesArmChannelState) {
  H h;
  h.arm_neutral();
  h.arm(true);
  const auto r = h.send(arm_json(++h.seq, true, 0.1, 0, 0.1, 0, 5.0));  // gripper 범위 밖
  EXPECT_TRUE(AckIs(r, false, Reason::kInvalidPacket, ChannelState::kActive));
  EXPECT_EQ(h.s->base_state(), ChannelState::kAwaitingNeutral);
}

// ───────────── lease·소유권·retire ─────────────

TEST(SessionOwner, LeaseExpiryReleasesBothChannelsAndSameUuidReacquiresWithHigherSequence) {
  H h;
  h.base_neutral();
  h.arm_neutral();
  h.base(true, 0.5);
  h.arm(true);
  const auto ev = h.tick(1.01);
  bool lease = false;
  for (const auto& e : ev) lease = lease || e.find("lease") != std::string::npos;
  EXPECT_TRUE(lease);
  EXPECT_FALSE(h.s->owner().has_value());
  EXPECT_TRUE(h.s->base_output().is_zero());
  EXPECT_FALSE(h.s->arm_output().active);
  EXPECT_TRUE(AckIs(h.send(cmd_json(1, false, 0, 0, 0)), false, Reason::kOutOfOrder));  // 같은 UUID 옛 번호
  EXPECT_TRUE(AckIs(h.send(cmd_json(h.seq + 1, false, 0, 0, 0)), true, Reason::kOk, ChannelState::kIdle));
  EXPECT_EQ(h.s->owner().value_or(""), kCid);
  EXPECT_EQ(h.s->arm_state(), ChannelState::kAwaitingNeutral);  // 팔도 새 중립 필요
}

TEST(SessionOwner, LeaseIsRefreshedByAnyChannel) {
  H h;
  h.arm_neutral();
  for (int i = 0; i < 30; ++i) h.arm_neutral(0.05);  // 1.5 s 동안 팔 패킷만
  EXPECT_EQ(h.s->owner().value_or(""), kCid);
}

TEST(SessionOwner, OtherUuidIsNotOwnerDuringLeaseAndOldUuidRetiredAfterTakeover) {
  H h;
  h.base_neutral();
  EXPECT_TRUE(AckIs(h.send(cmd_json(1, false, 0, 0, 0, kOther), kOtherAddr), false, Reason::kNotOwner));
  EXPECT_TRUE(AckIs(h.send(arm_json(1, false, 0, 0, 0, 0, 0, kOther), kOtherAddr), false, Reason::kNotOwner));
  h.tick(1.01);
  EXPECT_TRUE(AckIs(h.send(cmd_json(1, false, 0, 0, 0, kOther), kOtherAddr), true, Reason::kOk, ChannelState::kIdle));
  EXPECT_EQ(h.s->owner().value_or(""), kOther);
  h.tick(1.01);  // 새 조종자 lease 도 만료
  EXPECT_TRUE(AckIs(h.send(cmd_json(999, false, 0, 0, 0)), false, Reason::kNotOwner));  // 교체된 UUID 는 retire
}

TEST(SessionOwner, SameUuidFromNewSourcePortWaitsForLease) {
  H h;
  h.base_neutral();
  const Endpoint newport = Endpoint::from("192.168.0.10", 50001);
  EXPECT_TRUE(AckIs(h.send(cmd_json(++h.seq, false, 0, 0, 0), newport), false, Reason::kNotOwner));
  h.tick(1.01);
  EXPECT_TRUE(AckIs(h.send(cmd_json(++h.seq, false, 0, 0, 0), newport), true, Reason::kOk));
}

TEST(SessionOwner, DisconnectReleasesAndRetiresSession) {
  H h;
  h.base_neutral();
  h.arm_neutral();
  h.base(true, 0.5);
  h.arm(true);
  const auto r = h.event("disconnect");
  EXPECT_TRUE(AckIs(r, true, Reason::kOk));
  EXPECT_FALSE(h.s->owner().has_value());
  EXPECT_TRUE(h.s->base_output().is_zero());
  EXPECT_FALSE(h.s->arm_output().active);
  EXPECT_TRUE(AckIs(h.base_neutral(), false, Reason::kNotOwner));  // 종료된 세션의 지연 패킷
  EXPECT_TRUE(AckIs(h.send(cmd_json(1, false, 0, 0, 0, kOther), kOtherAddr), true, Reason::kOk));  // 새 UUID
}

TEST(SessionOwner, DisconnectWithoutOwnerRetiresThatUuid) {
  H h;
  EXPECT_TRUE(AckIs(h.send(ev_json("disconnect", 5, kOther), kOtherAddr), true, Reason::kOk));
  EXPECT_FALSE(h.s->owner().has_value());
  EXPECT_TRUE(AckIs(h.send(cmd_json(6, false, 0, 0, 0, kOther), kOtherAddr), false, Reason::kNotOwner));
}

TEST(SessionOwner, RetiredListIsBoundedAndForgetsOldest) {
  SessionConfig cfg = default_cfg();
  cfg.retired_limit = 3;
  H h(true, cfg);
  const std::vector<std::string> ids = {"00000000-0000-4000-8000-000000000001", "00000000-0000-4000-8000-000000000002",
                                        "00000000-0000-4000-8000-000000000003", "00000000-0000-4000-8000-000000000004"};
  for (const auto& id : ids) h.send(ev_json("disconnect", 1, id), kOtherAddr);
  EXPECT_EQ(h.s->snapshot(h.now).retired_count, 3u);
  EXPECT_TRUE(AckIs(h.send(cmd_json(2, false, 0, 0, 0, ids[1]), kOtherAddr), false, Reason::kNotOwner));
  EXPECT_TRUE(AckIs(h.send(cmd_json(2, false, 0, 0, 0, ids[0]), kOtherAddr), true, Reason::kOk));  // 잊힘
}

TEST(SessionOwner, Default1024RetiredLimit) {
  H h;
  char id[40];
  for (int i = 1; i <= 1025; ++i) {
    std::snprintf(id, sizeof(id), "00000000-0000-4000-8000-%012d", i);
    h.send(ev_json("disconnect", 1, id), kOtherAddr, 0.0);
  }
  EXPECT_EQ(h.s->snapshot(h.now).retired_count, 1024u);
  EXPECT_TRUE(AckIs(h.send(cmd_json(2, false, 0, 0, 0, "00000000-0000-4000-8000-000000000001"), kOtherAddr), true,
                  Reason::kOk));
}

// ───────────── 비상정지 ─────────────

TEST(SessionEstop, LatchStopsBothChannelsAndRejectsActiveCommands) {
  H h;
  h.base_neutral();
  h.arm_neutral();
  h.base(true, 1.0);
  h.arm(true);
  const auto r = h.event("emergency_stop");
  EXPECT_TRUE(AckIs(r, true, Reason::kOk, ChannelState::kEmergencyStop));
  ASSERT_TRUE(r.estop_change.has_value());
  EXPECT_TRUE(*r.estop_change);
  EXPECT_TRUE(r.ack.emergency_stop);
  EXPECT_TRUE(h.s->base_output().is_zero());
  EXPECT_FALSE(h.s->arm_output().active);  // 팔은 현재 위치 유지
  EXPECT_TRUE(AckIs(h.base(true, 1.0), false, Reason::kEmergencyStop, ChannelState::kEmergencyStop));
  EXPECT_TRUE(AckIs(h.arm(true), false, Reason::kEmergencyStop, ChannelState::kEmergencyStop));
  // 중립은 수락하지만 상태는 비상정지 유지
  EXPECT_TRUE(AckIs(h.base_neutral(), true, Reason::kOk, ChannelState::kEmergencyStop));
  EXPECT_TRUE(AckIs(h.arm_neutral(), true, Reason::kOk, ChannelState::kEmergencyStop));
  const auto again = h.event("emergency_stop");
  EXPECT_FALSE(again.estop_change.has_value());  // 이미 latch: 요청 재발행 없음
}

TEST(SessionEstop, NonOwnerEstopIsLatchedWithoutStealingOwnership) {
  H h;
  h.base_neutral();
  h.base(true, 1.0);
  const auto r = h.send(ev_json("emergency_stop", 1, kOther), kOtherAddr);
  EXPECT_TRUE(AckIs(r, true, Reason::kOk, ChannelState::kEmergencyStop));
  EXPECT_TRUE(r.estop_change.value_or(false));
  EXPECT_EQ(h.s->owner().value_or(""), kCid);
  EXPECT_TRUE(h.s->base_output().is_zero());
  // 비소유자의 해제는 거부
  EXPECT_TRUE(AckIs(h.send(ev_json("clear_emergency", 2, kOther), kOtherAddr), false, Reason::kNotOwner));
  EXPECT_TRUE(h.s->emergency_stop());
}

TEST(SessionEstop, NonOwnerEstopCanBeDisabled) {
  SessionConfig cfg = default_cfg();
  cfg.accept_estop_from_any_client = false;
  H h(true, cfg);
  h.base_neutral();
  EXPECT_TRUE(AckIs(h.send(ev_json("emergency_stop", 1, kOther), kOtherAddr), false, Reason::kNotOwner));
  EXPECT_FALSE(h.s->emergency_stop());
}

TEST(SessionEstop, RetiredClientEstopIsLatched) {
  H h;
  h.base_neutral();
  h.event("disconnect");  // kCid retire, 소유자 없음
  const auto r = h.event("emergency_stop");
  EXPECT_TRUE(AckIs(r, true, Reason::kOk, ChannelState::kEmergencyStop));
  EXPECT_FALSE(h.s->owner().has_value());
}

TEST(SessionEstop, LatchSurvivesOwnerChangeAndOwnerClearRequiresNeutralOnBothChannels) {
  H h;
  h.base_neutral();
  h.event("emergency_stop");
  h.tick(1.01);  // lease 만료 → 소유자 없음, latch 유지
  EXPECT_TRUE(h.s->emergency_stop());
  EXPECT_TRUE(AckIs(h.send(cmd_json(1, false, 0, 0, 0, kOther), kOtherAddr), true, Reason::kOk,
                  ChannelState::kEmergencyStop));
  const auto r = h.send(ev_json("clear_emergency", 2, kOther), kOtherAddr);
  EXPECT_TRUE(AckIs(r, true, Reason::kOk, ChannelState::kAwaitingNeutral));
  ASSERT_TRUE(r.estop_change.has_value());
  EXPECT_FALSE(*r.estop_change);
  EXPECT_FALSE(r.ack.emergency_stop);
  EXPECT_TRUE(AckIs(h.send(cmd_json(3, true, 1.0, 0, 0, kOther), kOtherAddr), false, Reason::kNeutralRequired));
  EXPECT_TRUE(AckIs(h.send(arm_json(4, true, 0.15, 0, 0.12, 0, 0, kOther), kOtherAddr), false, Reason::kNeutralRequired,
                  ChannelState::kAwaitingNeutral));
}

// ───────────── control_state 게이트 ─────────────

TEST(SessionGate, BeforeControlStateActiveCommandsAreNotOwnerButNeutralsAccepted) {
  H h(false);  // control_state 미수신
  EXPECT_TRUE(AckIs(h.base_neutral(), true, Reason::kOk, ChannelState::kIdle));
  EXPECT_TRUE(AckIs(h.base(true, 0.5), false, Reason::kNotOwner, ChannelState::kIdle));
  EXPECT_TRUE(h.s->base_output().is_zero());
  EXPECT_TRUE(AckIs(h.arm_neutral(), true, Reason::kOk, ChannelState::kIdle));
  EXPECT_TRUE(AckIs(h.arm(true), false, Reason::kNotOwner, ChannelState::kIdle));
  const auto snap = h.s->snapshot(h.now);
  EXPECT_FALSE(snap.udp_permitted);
  EXPECT_EQ(snap.blocked_reason.value_or(""), "control_state 미수신");
}

TEST(SessionGate, StaleControlStateBlocksAfterMaxAgeAndStopsActiveChannels) {
  H h(false);
  h.s->set_control_state(ControlState{false, false, "", true}, h.now);
  h.base_neutral(0.0);
  h.arm_neutral(0.0);
  EXPECT_TRUE(AckIs(h.base(true, 0.5, 0, 0, 0.0), true, Reason::kOk));
  EXPECT_TRUE(AckIs(h.arm(true, 0.15, 0, 0.05, -M_PI / 2, 0.5, 0.0), true, Reason::kOk));
  // 0.3 s 마다 패킷을 보내 watchdog 은 유지하면서 control_state 만 오래되게 한다
  for (int i = 0; i < 6; ++i) {
    h.base(true, 0.5, 0, 0, 0.3);
    h.arm(true, 0.15, 0, 0.05, -M_PI / 2, 0.5, 0.0);
  }
  ASSERT_NEAR(h.now - 100.0, 1.8, 1e-9);
  EXPECT_TRUE(AckIs(h.base(true, 0.5, 0, 0, 0.19), true, Reason::kOk));  // 1.99 s: 아직 허용
  h.tick(0.02);  // 2.01 s: 차단 → 활성 채널 정지
  EXPECT_FALSE(h.s->udp_permitted());
  EXPECT_TRUE(h.s->base_output().is_zero());
  EXPECT_FALSE(h.s->arm_output().active);
  EXPECT_TRUE(AckIs(h.base_neutral(0.01), true, Reason::kOk, ChannelState::kIdle));
  EXPECT_TRUE(AckIs(h.base(true, 0.5, 0, 0, 0.01), false, Reason::kNotOwner));
  // 새 control_state 가 오면 다시 허용
  h.s->set_control_state(ControlState{false, false, "", true}, h.now);
  EXPECT_TRUE(AckIs(h.base(true, 0.5, 0, 0, 0.01), true, Reason::kOk, ChannelState::kActive));
}

TEST(SessionGate, UdpAllowedFalseRejectsActiveAndStopsCurrentMotion) {
  H h;
  h.base_neutral();
  h.arm_neutral();
  h.base(true, 0.5);
  h.arm(true);
  h.cs.udp_allowed = false;
  EXPECT_TRUE(AckIs(h.base(true, 0.5), false, Reason::kNeutralRequired));  // 차단 순간 정지·새 중립 요구
  EXPECT_TRUE(h.s->base_output().is_zero());
  EXPECT_FALSE(h.s->arm_output().active);
  EXPECT_TRUE(AckIs(h.base_neutral(), true, Reason::kOk, ChannelState::kIdle));
  EXPECT_TRUE(AckIs(h.base(true, 0.5), false, Reason::kNotOwner, ChannelState::kIdle));
  EXPECT_TRUE(AckIs(h.arm_neutral(), true, Reason::kOk));
  EXPECT_TRUE(AckIs(h.arm(true), false, Reason::kNotOwner));
  h.cs.udp_allowed = true;
  EXPECT_TRUE(AckIs(h.base(true, 0.5), true, Reason::kOk, ChannelState::kActive));
}

TEST(SessionGate, GateRejectionsDoNotRefreshWatchdog) {
  H h;
  h.base_neutral();
  h.cs.udp_allowed = false;
  for (int i = 0; i < 9; ++i) h.base(true, 0.5);  // 0.45 s 동안 차단된 활성 명령만
  EXPECT_EQ(h.s->base_state(), ChannelState::kTimeout);
}

TEST(SessionGate, ControllerEstopMirrorsIntoSession) {
  H h;
  h.base_neutral();
  h.arm_neutral();
  h.base(true, 0.5);
  h.arm(true);
  h.cs.estop = true;
  const auto r = h.base(true, 0.5);
  EXPECT_TRUE(AckIs(r, false, Reason::kEmergencyStop, ChannelState::kEmergencyStop));
  EXPECT_TRUE(r.ack.emergency_stop);
  EXPECT_FALSE(r.estop_change.has_value());  // 제어기에서 온 상태를 다시 요청으로 보내지 않는다
  EXPECT_TRUE(h.s->base_output().is_zero());
  EXPECT_FALSE(h.s->arm_output().active);
  EXPECT_TRUE(AckIs(h.arm(true), false, Reason::kEmergencyStop, ChannelState::kEmergencyStop));
  h.cs.estop = false;
  EXPECT_TRUE(AckIs(h.base(true, 0.5), false, Reason::kNeutralRequired, ChannelState::kAwaitingNeutral));
  EXPECT_EQ(h.s->arm_state(), ChannelState::kAwaitingNeutral);
}

TEST(SessionGate, UdpClearWhileControllerEstopStillLatched) {
  H h;
  h.base_neutral();
  h.event("emergency_stop");
  h.cs.estop = true;  // 제어기도 latch 보고
  const auto r = h.event("clear_emergency");
  EXPECT_TRUE(AckIs(r, true, Reason::kOk, ChannelState::kEmergencyStop));
  EXPECT_FALSE(r.estop_change.value_or(true));  // 해제 요청은 보낸다
  EXPECT_TRUE(r.ack.emergency_stop);            // 제어기가 풀기 전까지 비상정지 유지
  h.cs.estop = false;
  EXPECT_TRUE(AckIs(h.base_neutral(), true, Reason::kOk, ChannelState::kIdle));
}

TEST(SessionGate, ControllerFaultRejectsActiveCommandsWithFault) {
  H h;
  h.base_neutral();
  h.arm_neutral();
  h.base(true, 0.5);
  h.cs.fault = true;
  h.cs.fault_reason = "servo bus timeout";
  EXPECT_TRUE(AckIs(h.base(true, 0.5), false, Reason::kFault, ChannelState::kFault));
  EXPECT_TRUE(AckIs(h.arm(true), false, Reason::kFault, ChannelState::kFault));
  EXPECT_TRUE(h.s->base_output().is_zero());
  EXPECT_NE(h.s->fault().value_or("").find("servo bus timeout"), std::string::npos);
  EXPECT_TRUE(AckIs(h.base_neutral(), true, Reason::kOk, ChannelState::kFault));
  h.cs.fault = false;
  EXPECT_TRUE(AckIs(h.base(true, 0.5), false, Reason::kNeutralRequired, ChannelState::kAwaitingNeutral));
}

TEST(SessionGate, FaultTakesPrecedenceOverEstop) {
  H h;
  h.base_neutral();
  h.cs.fault = true;
  h.cs.estop = true;
  EXPECT_TRUE(AckIs(h.base(true, 0.5), false, Reason::kFault, ChannelState::kFault));
}

// ───────────── 내부 fault·외부 정지 ─────────────

TEST(SessionInternal, InternalFaultIsPermanent) {
  H h;
  h.base_neutral();
  h.s->enter_fault("수신 스레드 종료");
  EXPECT_TRUE(AckIs(h.base(true, 0.5), false, Reason::kFault, ChannelState::kFault));
  h.cs.fault = false;  // 제어기 fault 없음 보고로 풀리지 않는다
  EXPECT_TRUE(AckIs(h.base(true, 0.5), false, Reason::kFault, ChannelState::kFault));
}

TEST(SessionInternal, ForceStopRequiresNewNeutralButKeepsOwnership) {
  H h;
  h.base_neutral();
  h.arm_neutral();
  h.base(true, 0.5);
  h.arm(true);
  h.s->force_stop();
  EXPECT_TRUE(h.s->base_output().is_zero());
  EXPECT_FALSE(h.s->arm_output().active);
  EXPECT_EQ(h.s->owner().value_or(""), kCid);
  EXPECT_TRUE(AckIs(h.base(true, 0.5), false, Reason::kNeutralRequired));
  EXPECT_TRUE(AckIs(h.arm(true), false, Reason::kNeutralRequired));
}

TEST(SessionInternal, UnsupportedProtocolAckEchoesIdentifiers) {
  H h;
  const std::string v2 = "{\"protocol\":2,\"type\":\"command\",\"client_id\":\"" + kCid +
                         "\",\"sequence\":5,\"control_active\":false,\"forward\":0,\"turn\":0}";
  const auto r = h.send(v2);
  EXPECT_TRUE(AckIs(r, false, Reason::kUnsupportedProtocol));
  EXPECT_EQ(r.ack.client_id.value_or(""), kCid);
  EXPECT_EQ(r.ack.sequence, 5);
  EXPECT_FALSE(h.s->owner().has_value());  // 소유권을 얻지 않는다
}

TEST(SessionInternal, SnapshotReportsOwnerAndStates) {
  H h;
  h.base_neutral();
  const auto s = h.s->snapshot(h.now);
  EXPECT_EQ(s.owner.value_or(""), kCid);
  EXPECT_EQ(s.owner_addr.value_or(""), "192.168.0.10:50000");
  EXPECT_EQ(s.base_state, ChannelState::kIdle);
  EXPECT_EQ(s.arm_state, ChannelState::kAwaitingNeutral);
  EXPECT_FALSE(s.emergency_stop);
  EXPECT_TRUE(s.udp_permitted);
  EXPECT_EQ(s.last_sequence, h.seq);
}

TEST(SessionConfigValidation, RejectsBadValues) {
  auto bad = [](auto mutate) {
    SessionConfig c = default_cfg();
    mutate(c);
    return c;
  };
  EXPECT_NO_THROW({
    Session ok_session(default_cfg());
    (void)ok_session;
  });
  EXPECT_THROW(Session(bad([](SessionConfig& c) { c.lease_s = 0.4; })), std::invalid_argument);
  EXPECT_THROW(Session(bad([](SessionConfig& c) { c.arm_watchdog_s = 1.5; })), std::invalid_argument);
  EXPECT_THROW(Session(bad([](SessionConfig& c) { c.base_watchdog_s = 0.0; })), std::invalid_argument);
  EXPECT_THROW(Session(bad([](SessionConfig& c) { c.max_vy_m_s = 0.0; })), std::invalid_argument);
  EXPECT_THROW(Session(bad([](SessionConfig& c) { c.max_wz_rad_s = NAN; })), std::invalid_argument);
  EXPECT_THROW(Session(bad([](SessionConfig& c) { c.workspace.x_min = 0.5; })), std::invalid_argument);
  EXPECT_THROW(Session(bad([](SessionConfig& c) { c.kinematics.upper_arm_m = 0.0; })), std::invalid_argument);
  EXPECT_THROW(Session(bad([](SessionConfig& c) { c.kinematics.j2_sign = 0.0; })), std::invalid_argument);
  EXPECT_THROW(Session(bad([](SessionConfig& c) { c.control_state_max_age_s = -1.0; })), std::invalid_argument);
  EXPECT_THROW(Session(bad([](SessionConfig& c) { c.retired_limit = 0; })), std::invalid_argument);
}
