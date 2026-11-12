// 팔: 보간·속도 제한·관절 속도 제한·home·watchdog(유지)·비상정지·IK/작업 공간 거부·joint_debug·fault 시험.
// 가짜 서보 버스(시계 주입)를 쓴다. 장치 값은 시험용 프로필(test_hardware_profile(), 실제 로봇 값 아님).
#include <gtest/gtest.h>

#include <cmath>

#include "armpi_control/arm_controller.hpp"
#include "armpi_hardware/hardware_profile.hpp"
#include "armpi_hardware/mock.hpp"

using namespace armpi_control;
namespace ah = armpi_hardware;
namespace ak = armpi_kinematics;

namespace {
constexpr double kD = M_PI / 180.0;

const ah::HardwareProfile kProf = ah::test_hardware_profile();

ArmConfig default_cfg() {
  ArmConfig c;
  auto& p = c.kinematics;
  p.base_height_m = 0.064605;  // 링크 길이: 공개 기본값
  p.upper_arm_m = 0.10048;
  p.forearm_m = 0.094714;
  p.hand_m = 0.16331;
  p.j1_sign = kProf.joint_sign[0];
  p.j2_sign = kProf.joint_sign[1];
  p.j3_sign = kProf.joint_sign[2];
  p.j4_sign = kProf.joint_sign[3];
  p.j1_offset = kProf.joint_offset[0];
  p.j2_offset = kProf.joint_offset[1];
  p.j3_offset = kProf.joint_offset[2];
  p.j4_offset = kProf.joint_offset[3];
  p.min_wrist_height_m = 0.0;
  p.min_elbow_bend_rad = 0.05;
  for (int i = 0; i < 4; ++i) {
    p.joint_min[i] = kProf.joint_min[i];
    p.joint_max[i] = kProf.joint_max[i];
  }
  c.workspace = ak::Workspace{0.05, 0.32, -0.25, 0.25, -0.08, 0.32};
  c.servo_map = kProf.servo_map;
  c.gripper = kProf.gripper;
  c.gripper_servo_min = kProf.gripper_servo_min;
  c.gripper_servo_max = kProf.gripper_servo_max;
  c.home_servo_values = kProf.home_servo_values;
  return c;
}

// 시험용 home 서보값 → 관절각(시험 안의 독립 계산)
double home_joint(int i) {
  const int sid = kProf.servo_map.servo_ids[i];
  return kProf.servo_map.directions[i] * (kProf.home_servo_values[sid - 1] - 500) * 0.24 * kD;
}
ak::Pose home_pose() {
  ak::ArmKinematics kin(default_cfg().kinematics);
  return kin.forward(ak::JointAngles{home_joint(0), home_joint(1), home_joint(2), home_joint(3)});
}

struct Fixture {
  double t{1000.0};
  std::shared_ptr<ah::MockServoBus> bus;
  std::unique_ptr<ArmController> arm;
  explicit Fixture(ArmConfig cfg = default_cfg()) {
    bus = std::make_shared<ah::MockServoBus>([this] { return t; });
    arm = std::make_unique<ArmController>(cfg, bus);
  }
  // 50 Hz 로 n 틱
  void run(int n, bool inhibit = false) {
    for (int i = 0; i < n; ++i) {
      t += 0.02;
      arm->tick(t, 0.02, inhibit);
    }
  }
};

double dist(const ArmPose& a, const ArmPose& b) { return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2)); }
}  // namespace

TEST(Interpolate, LinearPitchGripperLimits) {
  ArmRateLimits lim;  // 0.10 m/s, 1.0 rad/s, 1.0/s
  ArmPose cur{0.10, 0.0, 0.10, 0.0, 0.0};
  ArmPose goal{0.20, 0.0, 0.10, -1.0, 1.0};
  ArmPose n = interpolate_step(cur, goal, lim, 0.02);
  EXPECT_NEAR(n.x, 0.102, 1e-12);       // 0.10 m/s × 0.02 s
  EXPECT_NEAR(n.pitch, -0.02, 1e-12);   // 1 rad/s × 0.02
  EXPECT_NEAR(n.gripper, 0.02, 1e-12);
  // 대각선: 직선 거리로 제한
  ArmPose g2{0.13, 0.04, 0.10, 0.0, 0.0};
  n = interpolate_step(cur, g2, lim, 0.02);
  EXPECT_NEAR(dist(n, cur), 0.002, 1e-12);
  EXPECT_NEAR((n.y - cur.y) / (n.x - cur.x), 0.04 / 0.03, 1e-9);
  // 가까우면 목표 그대로
  ArmPose g3{0.1005, 0.0, 0.10, 0.001, 0.001};
  n = interpolate_step(cur, g3, lim, 0.02);
  EXPECT_TRUE(same_pose(n, g3));
  EXPECT_TRUE(same_pose(interpolate_step(cur, goal, lim, 0.0), cur));
}

TEST(Interpolate, StepJoints) {
  Joints5 a{0, 0, 0, 0, 0}, b{1.0, -1.0, 0.01, 0, 0};
  const auto s = step_joints(a, b, 0.04);
  EXPECT_NEAR(s[0], 0.04, 1e-12);
  EXPECT_NEAR(s[1], -0.04, 1e-12);
  EXPECT_NEAR(s[2], 0.01, 1e-12);
  EXPECT_EQ(step_joints(a, b, 0.0), b);  // 제한 없음
}

TEST(ArmController, ValidateDefaultConfig) {
  Fixture f;
  std::string why;
  EXPECT_TRUE(f.arm->validate(&why)) << why;
  ArmConfig bad = default_cfg();
  bad.workspace.x_min = home_pose().x + 0.02;  // home 이 작업 공간 밖
  Fixture g(bad);
  EXPECT_FALSE(g.arm->validate(&why));
}

TEST(ArmController, HomeSendsBootPoseAndInitializesFromFk) {
  Fixture f;
  EXPECT_FALSE(f.arm->initialized());
  std::string msg;
  ASSERT_TRUE(f.arm->home(f.t, &msg)) << msg;
  EXPECT_TRUE(f.arm->initialized());
  EXPECT_TRUE(f.arm->busy(f.t));
  for (int id = 1; id <= 6; ++id) {
    EXPECT_EQ(f.bus->goal(id), kProf.home_servo_values[id - 1]) << id;
    EXPECT_EQ(f.arm->commanded_servo()[id], kProf.home_servo_values[id - 1]);
  }
  EXPECT_EQ(f.bus->writes(), 6u);
  // 보간 상태 = 켤 때 자세의 FK, 그리퍼 0(열림 값과 같음)
  const ak::Pose want = home_pose();
  const ArmPose p = f.arm->reported_pose();
  EXPECT_NEAR(p.x, want.x, 1e-9);
  EXPECT_NEAR(p.y, want.y, 1e-9);
  EXPECT_NEAR(p.z, want.z, 1e-9);
  EXPECT_NEAR(p.pitch, want.pitch, 1e-9);
  EXPECT_DOUBLE_EQ(p.gripper, 0.0);
  // 이동 시간(1.5 s) 동안 가짜 서보는 선형으로 움직이고 끝나면 도착
  f.t += 1.6;
  const int id_j3 = kProf.servo_map.servo_ids[2];
  EXPECT_NEAR(f.bus->position(id_j3), kProf.home_servo_values[id_j3 - 1], 1e-9);
  // 관절 이름 순서: joint1..5, r_joint
  const auto j = f.arm->reported_joint_positions();
  for (int i = 0; i < 4; ++i) EXPECT_NEAR(j[i], home_joint(i), 1e-12) << i;
}

TEST(ArmController, IgnoresTargetsBeforeHomeAndWhileHoming) {
  Fixture f;
  ArmPose t{0.15, 0.0, 0.10, -0.5, 0.0};
  EXPECT_EQ(f.arm->on_target(true, t, f.t, false), TargetResult::kIgnoredUninitialized);
  std::string m;
  f.arm->home(f.t, &m);
  EXPECT_EQ(f.arm->on_target(true, t, f.t + 0.5, false), TargetResult::kIgnoredBusy);
  f.t += 1.6;
  f.arm->tick(f.t, 0.02, false);
  EXPECT_EQ(f.arm->mode(), ArmMode::kHold);
  EXPECT_EQ(f.arm->on_target(true, t, f.t, false), TargetResult::kAccepted);
}

TEST(ArmController, TracksTargetAtLimitedSpeedAndReachesIt) {
  Fixture f;
  std::string m;
  f.arm->home(f.t, &m);
  f.t += 1.6;
  f.arm->tick(f.t, 0.02, false);
  const ArmPose start = f.arm->reported_pose();
  ArmPose goal{0.15, 0.03, 0.10, -0.6, 0.5};
  ASSERT_EQ(f.arm->on_target(true, goal, f.t, false), TargetResult::kAccepted);
  const uint64_t w0 = f.bus->writes();
  f.run(1);
  EXPECT_GT(f.bus->writes(), w0);  // 서보 명령 나감
  // 한 틱 뒤 손끝 이동은 0.002 m 정도(관절 양자화 0.24° 때문에 약간 다를 수 있음)
  EXPECT_LT(dist(f.arm->reported_pose(), start), 0.004);
  // 20 Hz 로 목표를 계속 보내며 3 s → 도착
  for (int k = 0; k < 150; ++k) {
    if (k % 2 == 0) f.arm->on_target(true, goal, f.t, false);
    f.run(1);
  }
  const ArmPose p = f.arm->reported_pose();
  EXPECT_LT(dist(p, goal), 0.0025) << p.x << " " << p.y << " " << p.z;  // 서보 1단위(0.24°) 양자화 오차
  EXPECT_NEAR(p.pitch, goal.pitch, 0.01);
  EXPECT_NEAR(p.gripper, 0.5, 0.005);
  EXPECT_FALSE(f.arm->moving(f.t));
  // 손목 회전(서보 2)은 500 고정
  EXPECT_EQ(f.arm->commanded_servo()[2], 500);
  EXPECT_EQ(f.bus->goal(2), 500);
  // 토크 해제 명령은 한 번도 없음
  EXPECT_EQ(f.bus->unload_commands(), 0u);
}

TEST(ArmController, WatchdogHoldsAndStopsSendingCommands) {
  Fixture f;
  std::string m;
  f.arm->home(f.t, &m);
  f.t += 1.6;
  f.arm->tick(f.t, 0.02, false);
  ArmPose goal{0.20, 0.0, 0.05, -0.8, 0.0};
  ASSERT_EQ(f.arm->on_target(true, goal, f.t, false), TargetResult::kAccepted);
  f.run(10);  // 0.2 s 진행
  EXPECT_EQ(f.arm->mode(), ArmMode::kTracking);
  f.run(25);  // 마지막 목표 뒤 0.7 s → 0.6 s 넘음 → 유지
  EXPECT_EQ(f.arm->mode(), ArmMode::kHold);
  EXPECT_FALSE(f.arm->channel_active(f.t));
  const uint64_t w = f.bus->writes();
  const auto held = f.arm->commanded_servo();
  f.run(50);
  EXPECT_EQ(f.bus->writes(), w);  // 새 명령 없음
  EXPECT_EQ(f.arm->commanded_servo(), held);
  for (int id = 1; id <= 6; ++id) EXPECT_TRUE(f.bus->torque(id));  // 토크 유지
  EXPECT_EQ(f.bus->unload_commands(), 0u);
  // 멈춘 위치는 목표에 못 미친 중간 지점
  EXPECT_GT(dist(f.arm->reported_pose(), goal), 0.01);
}

TEST(ArmController, InactiveAndEstopHold) {
  Fixture f;
  std::string m;
  f.arm->home(f.t, &m);
  f.t += 1.6;
  f.arm->tick(f.t, 0.02, false);
  ArmPose goal{0.20, 0.05, 0.10, -0.5, 1.0};
  f.arm->on_target(true, goal, f.t, false);
  f.run(5);
  EXPECT_EQ(f.arm->on_target(false, goal, f.t, false), TargetResult::kHold);
  EXPECT_EQ(f.arm->mode(), ArmMode::kHold);
  uint64_t w = f.bus->writes();
  f.run(10);
  EXPECT_EQ(f.bus->writes(), w);
  // 다시 active → 추적, 비상정지(inhibit) → 즉시 유지, 목표는 무시
  EXPECT_EQ(f.arm->on_target(true, goal, f.t, false), TargetResult::kAccepted);
  f.run(3);
  w = f.bus->writes();
  f.run(1, /*inhibit=*/true);
  EXPECT_EQ(f.arm->mode(), ArmMode::kHold);
  EXPECT_EQ(f.bus->writes(), w);
  EXPECT_EQ(f.arm->on_target(true, goal, f.t, true), TargetResult::kIgnoredInhibit);
  f.run(10, true);
  EXPECT_EQ(f.bus->writes(), w);
}

TEST(ArmController, RejectsOutsideWorkspaceAndUnreachable) {
  Fixture f;
  std::string m;
  f.arm->home(f.t, &m);
  f.t += 1.6;
  EXPECT_EQ(f.arm->on_target(true, ArmPose{0.40, 0.0, 0.1, 0.0, 0.0}, f.t, false), TargetResult::kRejectedWorkspace);
  EXPECT_EQ(f.arm->on_target(true, ArmPose{0.02, 0.0, 0.1, 0.0, 0.0}, f.t, false), TargetResult::kRejectedWorkspace);
  // 상자 안이지만 IK 해 없음(아래를 보며 높이 0.3)
  EXPECT_EQ(f.arm->on_target(true, ArmPose{0.10, 0.0, 0.30, -M_PI / 2, 0.0}, f.t, false), TargetResult::kRejectedIk);
  EXPECT_EQ(f.arm->on_target(true, ArmPose{NAN, 0.0, 0.1, 0.0, 0.0}, f.t, false), TargetResult::kRejectedInvalid);
  // 거부된 목표는 watchdog 을 먹이지 않음
  EXPECT_FALSE(f.arm->channel_active(f.t));
}

TEST(ArmController, IkFailureDuringInterpolationHoldsLastValid) {
  // 시작 a 와 목표 b 는 둘 다 풀리지만 그 사이 직선(pitch 0)은 팔꿈치 한계(±120°) 밖이라 풀리지 않는다.
  // (미리 찾은 예: a 의 joint3 = −119.96°)
  Fixture f;
  std::string m;
  f.arm->home(f.t, &m);
  f.t += 1.6;
  f.arm->tick(f.t, 0.02, false);
  ak::ArmKinematics kin(default_cfg().kinematics);
  const ArmPose a{0.26, 0.0, 0.05, 0.0, 0.0}, b{0.26, 0.0, 0.08, 0.0, 0.0}, c{0.30, 0.0, 0.05, 0.0, 0.0};
  const auto ra = kin.inverse(ak::Pose{a.x, a.y, a.z, a.pitch});
  ASSERT_EQ(ra.status, ak::IkStatus::kOk);
  ASSERT_EQ(kin.inverse(ak::Pose{b.x, b.y, b.z, b.pitch}).status, ak::IkStatus::kOk);
  ASSERT_NE(kin.inverse(ak::Pose{a.x, a.y, 0.065, a.pitch}).status, ak::IkStatus::kOk);  // 중간은 해 없음
  // 관절 직접 지정으로 a 에 정확히 놓는다
  ASSERT_TRUE(f.arm->joint_debug({"joint1", "joint2", "joint3", "joint4"},
                                 {ra.joints.j1, ra.joints.j2, ra.joints.j3, ra.joints.j4}, f.t, &m)) << m;
  f.t += 5.0;
  f.arm->tick(f.t, 0.02, false);
  const auto at_a = f.arm->commanded_servo();
  ASSERT_EQ(f.arm->on_target(true, b, f.t, false), TargetResult::kAccepted);
  for (int k = 0; k < 50; ++k) {
    f.arm->on_target(true, b, f.t, false);
    f.run(1);
  }
  EXPECT_TRUE(f.arm->ik_blocked());
  EXPECT_EQ(f.arm->commanded_servo(), at_a);  // 마지막 유효 자세(a) 유지
  bool warned = false;
  for (const auto& e : f.arm->take_events()) warned |= e.level == ArmEvent::kWarn && e.text.find("IK 실패") != std::string::npos;
  EXPECT_TRUE(warned);
  // 풀리는 경로의 새 목표 → 다시 움직여 도착
  for (int k = 0; k < 100; ++k) {
    f.arm->on_target(true, c, f.t, false);
    f.run(1);
  }
  EXPECT_FALSE(f.arm->ik_blocked());
  EXPECT_LT(dist(f.arm->reported_pose(), c), 0.003);
}

TEST(ArmController, NearestPolicyContinuesThroughBlockedInterpolation) {
  // 위 시험과 같은 a → b(사이 직선이 pitch 0 으로는 안 풀림). nearest 정책이면 pitch 를 바꿔 지나가 b 위치에 도착한다.
  // b 에서 pitch 0.02–0.10 rad 은 팔꿈치 한계라 pitch 0 까지는 못 넘어간다(시험용 프로필 값으로 미리 계산).
  // 그때는 떨지 않고 가장 가까운 자세에서 멈춰야 한다.
  ArmConfig cfg = default_cfg();
  cfg.target_policy = ak::TargetPolicy::kNearest;
  Fixture f(cfg);
  std::string m;
  f.arm->home(f.t, &m);
  f.t += 1.6;
  f.arm->tick(f.t, 0.02, false);
  ak::ArmKinematics kin(cfg.kinematics);
  const ArmPose a{0.26, 0.0, 0.05, 0.0, 0.0}, b{0.26, 0.0, 0.08, 0.0, 0.7};
  const auto ra = kin.inverse(ak::Pose{a.x, a.y, a.z, a.pitch});
  ASSERT_EQ(ra.status, ak::IkStatus::kOk);
  ASSERT_NE(kin.inverse(ak::Pose{a.x, a.y, 0.065, a.pitch}).status, ak::IkStatus::kOk);
  ASSERT_TRUE(f.arm->joint_debug({"joint1", "joint2", "joint3", "joint4"},
                                 {ra.joints.j1, ra.joints.j2, ra.joints.j3, ra.joints.j4}, f.t, &m)) << m;
  f.t += 5.0;
  f.arm->tick(f.t, 0.02, false);
  ASSERT_EQ(f.arm->on_target(true, b, f.t, false), TargetResult::kAccepted);
  bool ever_blocked = false;
  for (int k = 0; k < 150; ++k) {
    f.arm->on_target(true, b, f.t, false);
    f.run(1);
    ever_blocked |= f.arm->ik_blocked();
    // 보정한 자세도 항상 상자 안·IK 해 있음
    const ArmPose p = f.arm->reported_pose();
    ASSERT_TRUE(ak::inside(cfg.workspace, ak::Pose{p.x, p.y, p.z, p.pitch})) << k;
  }
  (void)ever_blocked;
  EXPECT_LT(dist(f.arm->reported_pose(), b), 0.003);              // 위치는 도착
  EXPECT_LT(std::fabs(f.arm->reported_pose().pitch - b.pitch), 0.15);  // pitch 는 닿는 가장 가까운 값(약 0.12)
  EXPECT_NEAR(f.arm->reported_pose().gripper, b.gripper, 0.02);
  // 멈춘 뒤 서보 명령이 더 바뀌지 않는다(떨림 없음)
  const auto settled = f.arm->commanded_servo();
  for (int k = 0; k < 50; ++k) {
    f.arm->on_target(true, b, f.t, false);
    f.run(1);
    ASSERT_EQ(f.arm->commanded_servo(), settled) << k;
  }
  bool noted = false;
  for (const auto& e : f.arm->take_events()) noted |= e.text.find("보간 경로 보정") != std::string::npos;
  EXPECT_TRUE(noted);
}

TEST(ArmController, JointSpeedLimitSmoothsLargeIkJumps) {
  ArmConfig cfg = default_cfg();
  cfg.max_joint_speed_rad_s = 2.0;
  cfg.limits.max_linear_speed_m_s = 10.0;  // 손끝 보간을 사실상 끄고 관절 제한만 시험
  cfg.limits.max_pitch_rate_rad_s = 100.0;
  Fixture f(cfg);
  std::string m;
  f.arm->home(f.t, &m);
  f.t += 1.6;
  f.arm->tick(f.t, 0.02, false);
  const ArmPose goal{0.10, 0.20, 0.10, -0.5, 0.0};  // 베이스 약 63° 회전
  ASSERT_EQ(f.arm->on_target(true, goal, f.t, false), TargetResult::kAccepted);
  const int s6_before = f.arm->commanded_servo()[6];
  f.run(1);
  const int s6_after = f.arm->commanded_servo()[6];
  // 2 rad/s × 0.02 s = 0.04 rad = 2.29° ≈ 9.5 단위 이하
  EXPECT_LE(std::abs(s6_after - s6_before), 10);
  EXPECT_GT(std::abs(s6_after - s6_before), 0);
  for (int k = 0; k < 100; ++k) {
    f.arm->on_target(true, goal, f.t, false);
    f.run(1);
  }
  EXPECT_LT(dist(f.arm->reported_pose(), goal), 0.003);
}

TEST(ArmController, JointDebugAppliesAndClamps) {
  Fixture f;
  std::string m;
  EXPECT_FALSE(f.arm->joint_debug({"joint1"}, {0.1}, f.t, &m));  // home 전 거부
  f.arm->home(f.t, &m);
  f.t += 1.6;
  f.arm->tick(f.t, 0.02, false);
  ASSERT_TRUE(f.arm->joint_debug({"joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"},
                                 {0.2, 0.3, -1.0, -1.2, 0.1, 2.0}, f.t, &m))
      << m;
  const auto s = f.arm->commanded_servo();
  const auto& mp = kProf.servo_map;
  EXPECT_EQ(s[mp.id(0)], ah::joint_to_servo(mp, 0, 0.2));
  EXPECT_EQ(s[mp.id(1)], ah::joint_to_servo(mp, 1, 0.3));
  EXPECT_EQ(s[mp.id(2)], ah::joint_to_servo(mp, 2, -1.0));
  EXPECT_EQ(s[mp.id(3)], ah::joint_to_servo(mp, 3, -1.2));
  EXPECT_EQ(s[mp.id(4)], ah::joint_to_servo(mp, 4, 0.1));
  EXPECT_EQ(s[mp.gripper_id()], kProf.gripper_servo_max);  // r_joint 2.0 rad → 그리퍼 상한으로 자름
  EXPECT_TRUE(f.arm->busy(f.t));
  EXPECT_EQ(f.arm->timed_kind(), "joint_debug");
  // 관절 한계 자르기: joint2 는 joint_max 로
  f.t += 5.0;
  ASSERT_TRUE(f.arm->joint_debug({"joint2"}, {3.0}, f.t, &m));
  EXPECT_EQ(f.arm->commanded_servo()[kProf.servo_map.id(1)], ah::joint_to_servo(kProf.servo_map, 1, kProf.joint_max[1]));
  EXPECT_FALSE(f.arm->joint_debug({"bogus"}, {0.0}, f.t, &m));
  EXPECT_FALSE(f.arm->joint_debug({"joint1"}, {NAN}, f.t, &m));
}

TEST(ArmController, ServoWriteFailuresBecomeFault) {
  ArmConfig cfg = default_cfg();
  cfg.fault_threshold = 3;
  Fixture f(cfg);
  std::string m;
  f.arm->home(f.t, &m);
  f.t += 1.6;
  f.arm->tick(f.t, 0.02, false);
  f.arm->on_target(true, ArmPose{0.15, 0.0, 0.10, -0.6, 0.0}, f.t, false);
  f.bus->set_fail(true);
  f.run(2);
  EXPECT_FALSE(f.arm->fault());
  f.run(1);
  EXPECT_TRUE(f.arm->fault());
  EXPECT_NE(f.arm->fault_reason().find("연속 실패"), std::string::npos);
  EXPECT_EQ(f.arm->on_target(true, ArmPose{0.15, 0.0, 0.10, -0.6, 0.0}, f.t, false), TargetResult::kIgnoredInhibit);
}

TEST(ArmController, MeasuredFeedbackAndFallback) {
  ArmConfig cfg = default_cfg();
  cfg.read_position = true;
  cfg.read_rate_hz = 5.0;
  Fixture f(cfg);
  std::string m;
  f.arm->home(f.t, &m);
  EXPECT_EQ(f.arm->position_source(), "commanded");
  for (int k = 0; k < 100; ++k) {
    f.t += 0.02;
    f.arm->poll_feedback(f.t);
  }
  EXPECT_EQ(f.arm->position_source(), "measured");
  EXPECT_EQ(f.arm->reported_servo()[4], kProf.home_servo_values[3]);  // 가짜 서보가 home 에 도착
  // 읽기 실패가 이어지면 commanded 로 되돌아감
  f.bus->set_fail(true);
  for (int k = 0; k < 200; ++k) {
    f.t += 0.02;
    f.arm->poll_feedback(f.t);
  }
  EXPECT_EQ(f.arm->position_source(), "commanded");
  EXPECT_GT(f.arm->feedback_failures(), 0u);
}
