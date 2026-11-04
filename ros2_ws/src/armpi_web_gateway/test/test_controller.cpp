// Controller 시험: 파이썬 조종기와 같은 조종권·세션·중립·명령 규칙.
#include <gtest/gtest.h>

#include "fakes.hpp"

using namespace gwtest;
using armpi_web_gateway::kOwnerTimeoutS;

TEST(Controller, OnDemandOpensSessionOnlyWhenScreenClaims) {
  Rig r;
  r.run(3);
  EXPECT_FALSE(r.ctl->session_live());
  EXPECT_TRUE(r.wire.sent.empty());  // 조종하는 화면이 없으면 아무것도 보내지 않는다
  EXPECT_EQ(r.ctl->set_input(Rig::input(true, true, 0.5), screen("A")), "{\"ok\":true,\"control\":true}");
  r.run(1);
  EXPECT_TRUE(r.ctl->session_live());
  EXPECT_EQ(r.wire.last(1).type, PacketType::kCommand);
  EXPECT_FALSE(r.wire.last(1).control_active);  // 첫 패킷은 중립
  for (int i = 0; i < 4; ++i) {
    r.ctl->set_input(Rig::input(true, true, 0.5), screen("A"));
    r.run(1);
  }
  EXPECT_TRUE(r.wire.last(1).control_active);   // 중립 확인 뒤 활성
  EXPECT_DOUBLE_EQ(r.wire.last(1).forward, 0.5);
}

TEST(Controller, InputGapOver300msGoesNeutral) {
  Rig r;
  for (int i = 0; i < 5; ++i) {
    r.ctl->set_input(Rig::input(true, true, 1.0), screen("A"));
    r.run(1);
  }
  ASSERT_TRUE(r.wire.last(1).control_active);
  r.run(7);  // 0.35 s 입력 없음
  EXPECT_FALSE(r.wire.last(1).control_active);
}

TEST(Controller, OwnerTimeoutReleasesAndClosesSession) {
  Rig r;
  r.ctl->set_input(Rig::input(true, true), screen("A"));
  r.run(1);
  ASSERT_EQ(r.ctl->owner_id(), "A");
  r.t += kOwnerTimeoutS + 0.1;
  r.run(1);
  EXPECT_FALSE(r.ctl->owner_id().has_value());
  EXPECT_FALSE(r.ctl->session_live());
  EXPECT_EQ(r.wire.last().type, PacketType::kDisconnect);  // 다른 조종기(VR 등)가 쓸 수 있게
  EXPECT_TRUE(r.client->retired());
  // 다시 잡으면 새 UUID 로 세션
  const std::string old = r.client->client_id();
  r.ctl->set_input(Rig::input(true, true), screen("A"));
  r.run(1);
  EXPECT_NE(r.client->client_id(), old);
}

TEST(Controller, OtherScreenIsRefusedUntilTakeControl) {
  Rig r;
  r.ctl->set_input(Rig::input(true, true), screen("A"));
  EXPECT_EQ(r.ctl->set_input(Rig::input(true, true), screen("B")), "{\"ok\":true,\"control\":false}");
  const auto refused = r.ctl->command(Rig::cmd("arm_home_target"), screen("B"));
  EXPECT_FALSE(refused.ok);
  EXPECT_EQ(refused.http_code(), 409);
  EXPECT_NE(refused.to_json().find("다른 기기가 조종 중입니다: 폰 A(192.168.0.10)"), std::string::npos);
  const auto taken = r.ctl->command(Rig::cmd("take_control"), screen("B"));
  EXPECT_TRUE(taken.ok);
  EXPECT_EQ(r.ctl->owner_id(), "B");
  EXPECT_EQ(r.ctl->set_input(Rig::input(true, true), screen("A")), "{\"ok\":true,\"control\":false}");
}

TEST(Controller, UnfocusedOrIdleScreenDoesNotClaim) {
  Rig r;
  EXPECT_EQ(r.ctl->set_input(Rig::input(true, false), screen("A")), "{\"ok\":true,\"control\":false}");
  EXPECT_EQ(r.ctl->set_input(Rig::input(false, true), screen("A")), "{\"ok\":true,\"control\":false}");
  EXPECT_FALSE(r.ctl->owner_id().has_value());
  EXPECT_EQ(r.ctl->set_input(Rig::input(false, true, 0, 1), screen("A")), "{\"ok\":true,\"control\":true}");  // 팔 방향
}

TEST(Controller, EstopFromAnyScreenOpensSessionAndLatches) {
  Rig r;
  r.ctl->set_input(Rig::input(true, true), screen("A"));
  r.run(1);
  const auto res = r.ctl->command(Rig::cmd("estop"), screen("B"));  // 조종권 없는 화면
  EXPECT_TRUE(res.ok);
  EXPECT_EQ(r.wire.count(PacketType::kEmergencyStop), 1);
  // 세션이 없을 때도: 조종권이 풀린 뒤 다른 화면의 비상정지
  Rig q;
  q.ctl->command(Rig::cmd("estop"), screen("C"));
  EXPECT_TRUE(q.ctl->session_live());
  EXPECT_EQ(q.wire.count(PacketType::kEmergencyStop), 1);
  q.run(1);
  EXPECT_FALSE(q.client->pending_estop());  // ACK 받음
  q.t += 2.1;
  q.run(1);
  EXPECT_FALSE(q.ctl->session_live());  // 조종권 없으면 2 s 뒤 닫음
}

TEST(Controller, EstopWithoutControlIsEstopOnlySessionNotAConnection) {
  // 사용자 보고(2026-11-04): 연결이 없는데 비상정지를 누르면 화면에 "로봇 연결"이 떴다.
  // 비상정지는 그대로 보내되, 세션 이유를 estop_only 로 알려 화면이 연결로 보이지 않게 한다.
  Rig q;
  EXPECT_EQ(q.ctl->session_reason(), "none");
  EXPECT_TRUE(q.ctl->command(Rig::cmd("estop"), screen("C")).ok);
  EXPECT_EQ(q.wire.count(PacketType::kEmergencyStop), 1);  // 연결이 없어도 비상정지는 간다
  EXPECT_EQ(q.ctl->session_reason(), "estop_only");
  EXPECT_FALSE(q.ctl->owner_id().has_value());             // 조종권은 생기지 않는다
  auto v = Rig::parse(q.ctl->state_json("C"));
  EXPECT_EQ(v.find("session")->find("reason")->string, "estop_only");
  EXPECT_TRUE(v.find("session")->find("live")->boolean);
  EXPECT_TRUE(v.find("control")->find("owner")->is_null());
  EXPECT_FALSE(v.find("control")->find("you")->boolean);
  q.run(1);
  EXPECT_EQ(q.ctl->session_reason(), "estop_only");         // ACK 뒤에도 2 s 동안은 비상정지용
  q.t += 2.1;
  q.run(1);
  EXPECT_EQ(q.ctl->session_reason(), "none");               // 2 s 뒤 닫힘
  v = Rig::parse(q.ctl->state_json("C"));
  EXPECT_FALSE(v.find("session")->find("live")->boolean);
  // 조종권을 가진 화면이 있으면 user — 그 상태의 비상정지는 원래 연결이므로 user 그대로
  q.ctl->set_input(Rig::input(true, true), screen("A"));
  q.run(1);
  EXPECT_EQ(q.ctl->session_reason(), "user");
  q.ctl->command(Rig::cmd("estop"), screen("B"));
  EXPECT_EQ(q.ctl->session_reason(), "user");
  EXPECT_EQ(q.ctl->owner_id().value_or(""), "A");
  q.run(1);  // 비상정지 ACK 받음
  // 조종권을 놓으면 linger 도 지워져 닫히는 중(closing) → 다음 루프에서 none
  q.ctl->command(Rig::cmd("release_control"), screen("A"));
  EXPECT_EQ(q.ctl->session_reason(), "closing");  // release_control 은 linger 를 지운다 → 다음 루프에서 닫힘
  q.run(1);
  EXPECT_EQ(q.ctl->session_reason(), "none");
  // always 모드는 항상 always
  Rig a(false);
  EXPECT_EQ(a.ctl->session_reason(), "always");
  a.ctl->command(Rig::cmd("estop"), screen("C"));
  EXPECT_EQ(a.ctl->session_reason(), "always");
}

TEST(Controller, StaleInputCounterIsDropped) {
  Rig r;
  EXPECT_EQ(r.ctl->set_input(Rig::input(true, true, 0, 0, 5), screen("A")), "{\"ok\":true,\"control\":true}");
  EXPECT_EQ(r.ctl->set_input(Rig::input(true, true, 0, 0, 4), screen("A")),
            "{\"ok\":true,\"control\":true,\"stale\":true}");
}

TEST(Controller, IdleReleaseFreesRobotForOtherControllers) {
  Rig r(true, 5.0);
  r.ctl->set_input(Rig::input(true, true), screen("A"));
  for (int i = 0; i < 110; ++i) {  // 5.5 s 동안 화면은 살아 있지만 조작 없음
    r.ctl->set_input(Rig::input(false, true), screen("A"));
    r.run(1);
  }
  EXPECT_FALSE(r.ctl->owner_id().has_value());
  EXPECT_FALSE(r.ctl->session_live());
  // 조작 없이 포커스만으로는 다시 잡지 못한다
  EXPECT_EQ(r.ctl->set_input(Rig::input(false, true), screen("A")), "{\"ok\":true,\"control\":false}");
}

TEST(Controller, AlwaysModeKeepsSession) {
  Rig r(false);
  EXPECT_TRUE(r.ctl->session_live());
  r.run(2);
  EXPECT_EQ(r.wire.count(PacketType::kCommand), 2);
}

TEST(Controller, ArmEnableSyncsTargetToRobotAndMovesWithInput) {
  Rig r;
  r.ctl->command(Rig::cmd("arm_enable", ",\"on\":true"), screen("A"));
  EXPECT_TRUE(r.ctl->arm_enabled());
  EXPECT_DOUBLE_EQ(r.client->arm_target().x, 0.15);
  for (int i = 0; i < 6; ++i) {
    r.ctl->set_input(Rig::input(false, true, 0, 1), screen("A"));
    r.run(1);
  }
  EXPECT_GT(r.client->arm_target().x, 0.15);
  EXPECT_TRUE(r.wire.last(0).control_active);
}

TEST(Controller, CommandsAndErrors) {
  Rig r;
  EXPECT_EQ(r.ctl->command(Rig::cmd("nope"), screen("A")).http_code(), 400);
  EXPECT_EQ(r.ctl->command(Rig::cmd("arm_target", ",\"x\":\"abc\""), screen("A")).error, "x 숫자 아님");
  EXPECT_TRUE(r.ctl->command(Rig::cmd("arm_target", ",\"x\":0.2,\"gripper\":5"), screen("A")).ok);
  EXPECT_DOUBLE_EQ(r.client->arm_target().gripper, 1.0);
  EXPECT_TRUE(r.ctl->command(Rig::cmd("robot_reset"), screen("A")).ok);
  EXPECT_EQ(r.robot.calls.back(), "/armpi/reset_estop");
  r.robot.services_ready = false;
  EXPECT_EQ(r.ctl->command(Rig::cmd("robot_estop"), screen("B")).http_code(), 503);
}

TEST(Controller, JointDebugOnlyWithoutSession) {
  Rig r;
  const std::string pos = ",\"positions\":[0,0,0,0,0,9]";
  // 세션이 없으면 허용(로봇 owner 도 null)
  EXPECT_TRUE(r.ctl->command(Rig::cmd("joint_debug", pos), screen("A")).ok);
  ASSERT_EQ(r.robot.joint_debug.size(), 1u);
  EXPECT_DOUBLE_EQ(r.robot.joint_debug[0][5], armpi_web_gateway::kJointLimitRad);  // 잘림
  r.ctl->set_input(Rig::input(true, true), screen("A"));
  r.run(1);  // 세션 열림
  EXPECT_FALSE(r.ctl->command(Rig::cmd("joint_debug", pos), screen("A")).ok);
  EXPECT_EQ(r.ctl->command(Rig::cmd("joint_debug", ",\"positions\":[1]"), screen("A")).error,
            "조종 세션이 있으면 관절 디버그 불가(먼저 '연결 끊기')");
}

TEST(Controller, StateJsonHasPythonKeys) {
  Rig r;
  r.ctl->set_input(Rig::input(true, true), screen("A"));
  r.run(1);
  const auto v = Rig::parse(r.ctl->state_json("A"));
  for (const char* k : {"udp", "arm_enabled", "input", "rosbridge", "joint_debug_allowed", "loop_overruns", "control",
                        "session", "events"}) {
    EXPECT_NE(v.find(k), nullptr) << k;
  }
  EXPECT_TRUE(v.find("control")->find("you")->boolean);
  EXPECT_EQ(v.find("control")->find("owner")->find("short")->string, "A");
  EXPECT_EQ(v.find("session")->find("mode")->string, "on-demand");
  EXPECT_EQ(v.find("session")->find("reason")->string, "user");
  EXPECT_FALSE(v.find("events")->array.empty());
}
