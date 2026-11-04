#include <gtest/gtest.h>

#include <cmath>

#include "armpi_kinematics/arm_kinematics.hpp"

using armpi_kinematics::ArmKinematics;
using armpi_kinematics::IkStatus;
using armpi_kinematics::Params;
using armpi_kinematics::Pose;

namespace {
// 시험용 가상 치수(실제 ArmPi Pro 치수 아님)
Params test_params() {
  Params p;
  p.base_height_m = 0.10;
  p.upper_arm_m = 0.10;
  p.forearm_m = 0.095;
  p.hand_m = 0.12;
  return p;
}
}  // namespace

TEST(ArmKinematics, RoundTripManyPoses) {
  ArmKinematics k(test_params());
  ASSERT_TRUE(k.valid());
  int ok = 0;
  for (double x = 0.05; x <= 0.25; x += 0.02) {
    for (double y = -0.15; y <= 0.15; y += 0.05) {
      for (double z = -0.05; z <= 0.25; z += 0.03) {
        for (double pitch : {-M_PI / 2, -M_PI / 4, 0.0}) {
          Pose t{x, y, z, pitch};
          auto r = k.inverse(t);
          if (r.status != IkStatus::kOk) continue;
          ++ok;
          Pose f = k.forward(r.joints);
          EXPECT_NEAR(f.x, x, 1e-9);
          EXPECT_NEAR(f.y, y, 1e-9);
          EXPECT_NEAR(f.z, z, 1e-9);
          EXPECT_NEAR(std::remainder(f.pitch - pitch, 2 * M_PI), 0.0, 1e-9);
        }
      }
    }
  }
  EXPECT_GT(ok, 100);
}

TEST(ArmKinematics, OutOfReach) {
  ArmKinematics k(test_params());
  EXPECT_EQ(k.inverse(Pose{1.0, 0.0, 0.1, 0.0}).status, IkStatus::kOutOfReach);
}

TEST(ArmKinematics, InvalidInput) {
  ArmKinematics k(test_params());
  EXPECT_EQ(k.inverse(Pose{NAN, 0.0, 0.1, 0.0}).status, IkStatus::kInvalid);
  Params bad;  // 길이 0
  EXPECT_EQ(ArmKinematics(bad).inverse(Pose{0.1, 0, 0.1, 0}).status, IkStatus::kInvalid);
}

TEST(ArmKinematics, JointLimit) {
  Params p = test_params();
  p.joint_min[0] = -0.1;  // 베이스 회전을 좁게
  p.joint_max[0] = 0.1;
  ArmKinematics k(p);
  EXPECT_EQ(k.inverse(Pose{0.15, 0.15, 0.1, -M_PI / 4}).status, IkStatus::kJointLimit);
  EXPECT_EQ(k.inverse(Pose{0.15, 0.0, 0.1, -M_PI / 4}).status, IkStatus::kOk);
}

TEST(ArmKinematics, SignOffsetMapping) {
  Params p = test_params();
  p.j3_sign = -1.0;
  p.j2_offset = 0.3;
  ArmKinematics k(p);
  Pose t{0.15, 0.03, 0.12, -M_PI / 3};
  auto r = k.inverse(t);
  ASSERT_EQ(r.status, IkStatus::kOk);
  Pose f = k.forward(r.joints);
  EXPECT_NEAR(f.x, t.x, 1e-9);
  EXPECT_NEAR(f.z, t.z, 1e-9);
}

TEST(Workspace, Inside) {
  armpi_kinematics::Workspace w{0.0, 0.3, -0.2, 0.2, -0.05, 0.3};
  EXPECT_TRUE(armpi_kinematics::inside(w, Pose{0.1, 0.0, 0.1, 0.0}));
  EXPECT_FALSE(armpi_kinematics::inside(w, Pose{0.4, 0.0, 0.1, 0.0}));
}

// ---------------------------------------------------------------------------
// armpi.yaml 의 링크 길이(Hiwonder 공개 ArmPi FPV 값)와 시험용 관절 기준·범위(test_hardware_profile.yaml,
// 실제 로봇 값 아님)로 관절 공간을 훑어 FK → IK → FK 왕복을 확인한다.
namespace {
Params default_params() {
  Params p;
  p.base_height_m = 0.064605;
  p.upper_arm_m = 0.10048;
  p.forearm_m = 0.094714;
  p.hand_m = 0.16331;
  p.j2_offset = -1.4;  // 시험용
  p.j3_sign = -1.0;    // 시험용
  p.j4_offset = 0.1;   // 시험용
  p.min_wrist_height_m = 0.0;
  p.min_elbow_bend_rad = 0.05;
  const double d = M_PI / 180.0;
  p.joint_min[0] = -120 * d; p.joint_max[0] = 120 * d;
  p.joint_min[1] = -1.4;     p.joint_max[1] = 1.4;  // 시험용
  p.joint_min[2] = -120 * d; p.joint_max[2] = 120 * d;
  p.joint_min[3] = -1.9;     p.joint_max[3] = 1.9;  // 시험용
  return p;
}
}  // namespace

TEST(DefaultParams, ForwardInverseRoundTrip) {
  ArmKinematics k(default_params());
  ASSERT_TRUE(k.valid());
  const double d = M_PI / 180.0;
  int ok = 0;
  for (double a = -100; a <= 100; a += 25) {
    for (double b = -80; b <= 80; b += 20) {
      for (double c = -110; c <= 110; c += 20) {
        for (double e = -100; e <= 100; e += 25) {
          armpi_kinematics::JointAngles q{};
          q.j1 = a * d; q.j2 = b * d; q.j3 = c * d; q.j4 = e * d;
          const Pose t = k.forward(q);
          const auto r = k.inverse(t);
          if (r.status != IkStatus::kOk) continue;  // 손목 높이·팔꿈치 조건으로 거부되는 자세는 건너뜀
          ++ok;
          const Pose f = k.forward(r.joints);
          EXPECT_NEAR(f.x, t.x, 1e-9);
          EXPECT_NEAR(f.y, t.y, 1e-9);
          EXPECT_NEAR(f.z, t.z, 1e-9);
          EXPECT_NEAR(std::remainder(f.pitch - t.pitch, 2 * M_PI), 0.0, 1e-9);
        }
      }
    }
  }
  EXPECT_GT(ok, 300);
}

TEST(DefaultParams, BootPoseIsSolvable) {
  // 기본 자세 손끝 목표(armpi.yaml arm.home)는 풀려야 한다.
  ArmKinematics k(default_params());
  const auto r = k.inverse(Pose{0.1047, 0.0, 0.0753, -1.0263});
  EXPECT_EQ(r.status, IkStatus::kOk);
}
