// 메카넘·cmd_vel·서보값·그리퍼 변환 시험. 기대값은 식을 시험 안에서 따로 구현한 ref_*() 로 계산한다.
// 장치 값은 시험용 프로필(test_hardware_profile(), 실제 로봇 값 아님)을 쓴다.
#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "armpi_hardware/conversions.hpp"
#include "armpi_hardware/hardware_profile.hpp"

using namespace armpi_hardware;

namespace {
constexpr double kPi = 3.14159265358979323846;
double deg2rad(double d) { return d * kPi / 180.0; }
const HardwareProfile kTest = test_hardware_profile();
const MecanumParams& kMec = kTest.mecanum;
const JointServoMap& kMap = kTest.servo_map;

// 메카넘 식(시험용 독립 구현): mm/s → 펄스/10 ms, 0 쪽으로 자르기
double ref_pulse(double mm_s, const MecanumParams& p) {
  return mm_s / (kPi * p.wheel_diameter_mm) * p.pulse_per_cycle * 0.01;
}
int trunc0(double x) { return static_cast<int>(x); }
std::array<int, 4> ref_mecanum(double v, double dir_deg, double w, const MecanumParams& p) {
  const double vel = -v, ang = -w;
  const double vx = vel * std::cos(dir_deg * kPi / 180.0), vy = vel * std::sin(dir_deg * kPi / 180.0);
  const double vp = ang * (p.a_mm + p.b_mm);
  const double v1 = vy - vx + vp, v2 = vy + vx - vp, v3 = vy - vx - vp, v4 = vy + vx + vp;
  return {trunc0(ref_pulse(-v1, p)), trunc0(ref_pulse(v4, p)), trunc0(ref_pulse(v2, p)), trunc0(ref_pulse(-v3, p))};
}
// 서보값 식(시험용): round-half-even(dir·deg·1000/240 + 500), 0..1000
int ref_servo(double deg, int dir) {
  double raw = dir * deg * (1000.0 / 240.0) + 500.0;
  int v = static_cast<int>(std::nearbyint(raw));
  return v < 0 ? 0 : (v > 1000 ? 1000 : v);
}
}  // namespace

TEST(Profile, TestProfileIsComplete) {
  EXPECT_TRUE(missing_profile_items(kTest).empty());
  EXPECT_TRUE(kMec.valid());
  EXPECT_TRUE(kMap.valid());
}

TEST(Profile, EmptyProfileListsEverything) {
  const auto m = missing_profile_items(HardwareProfile{});
  EXPECT_GE(m.size(), 10u);
  // 꺼 둔 장치의 값은 묻지 않는다
  const auto m2 = missing_profile_items(HardwareProfile{}, ProfileUse{false, false, false});
  EXPECT_LT(m2.size(), m.size());
}

TEST(Profile, FillUnsetOnlyFillsBlanks) {
  HardwareProfile p;
  p.mecanum.a_mm = 123.0;
  p.buzzer_bcm = 5;
  const auto filled = fill_unset(&p, kTest);
  EXPECT_DOUBLE_EQ(p.mecanum.a_mm, 123.0);
  EXPECT_EQ(p.buzzer_bcm, 5);
  EXPECT_DOUBLE_EQ(p.mecanum.b_mm, kMec.b_mm);
  EXPECT_TRUE(missing_profile_items(p).empty());
  for (const auto& f : filled) {
    EXPECT_NE(f, "chassis.a_mm");
    EXPECT_NE(f, "buzzer.bcm");
  }
}

TEST(Mecanum, MatchesReferenceFormulaSweep) {
  int n = 0;
  for (double v = 0.0; v <= 300.0; v += 37.5) {
    for (double dir = -180.0; dir <= 360.0; dir += 22.5) {
      for (double w : {-1.2, -0.6, -0.05, 0.0, 0.05, 0.6, 1.2}) {
        EXPECT_EQ(mecanum_motor_pulses(v, dir, w, kMec), ref_mecanum(v, dir, w, kMec)) << v << " " << dir << " " << w;
        ++n;
      }
    }
  }
  EXPECT_GT(n, 1000);
}

TEST(Mecanum, SpeedConversionFormula) {
  for (double mm : {-500.0, -100.0, -1.0, 0.0, 1.0, 37.0, 100.0, 500.0}) {
    EXPECT_NEAR(speed_mm_s_to_pulse_per_10ms(mm, kMec), ref_pulse(mm, kMec), 1e-9);
  }
}

TEST(Mecanum, ForwardHundredMmPerSecond) {
  // 시험용 치수: 100/(π·100)·6000·0.01 = 19.10 → 19, 앞으로는 [+, −, −, +]
  const auto m = mecanum_motor_pulses(100.0, 90.0, 0.0, kMec);
  EXPECT_EQ(m[0], 19);
  EXPECT_EQ(m[1], -19);
  EXPECT_EQ(m[2], -19);
  EXPECT_EQ(m[3], 19);
}

TEST(Mecanum, NonFiniteOrEmptyParamsGiveZero) {
  for (int v : mecanum_motor_pulses(NAN, 90.0, 0.0, kMec)) EXPECT_EQ(v, 0);
  for (int v : mecanum_motor_pulses(100.0, 90.0, 0.0, MecanumParams{})) EXPECT_EQ(v, 0);  // 프로필 비어 있음
}

TEST(CmdVel, ToPolarBasicDirections) {
  // 앞(+x) = 90°, 왼쪽(+y) = 180°, 오른쪽(−y) = 0°, 뒤(−x) = −90°(≡ 270°)
  auto p = twist_to_polar(0.1, 0.0, 0.0, 1.0);
  EXPECT_NEAR(p.velocity_mm_s, 100.0, 1e-9);
  EXPECT_NEAR(p.direction_deg, 90.0, 1e-9);
  p = twist_to_polar(0.0, 0.1, 0.0, 1.0);
  EXPECT_NEAR(p.direction_deg, 180.0, 1e-9);
  p = twist_to_polar(0.0, -0.1, 0.0, 1.0);
  EXPECT_NEAR(p.direction_deg, 0.0, 1e-9);
  p = twist_to_polar(-0.1, 0.0, 0.0, 1.0);
  EXPECT_NEAR(std::remainder(p.direction_deg - 270.0, 360.0), 0.0, 1e-9);
  p = twist_to_polar(0.1, 0.1, 0.0, 1.0);
  EXPECT_NEAR(p.velocity_mm_s, 141.4213562373095, 1e-9);
  EXPECT_NEAR(p.direction_deg, 135.0, 1e-9);
}

TEST(CmdVel, AngularSign) {
  EXPECT_DOUBLE_EQ(twist_to_polar(0, 0, 0.5, 1.0).angular_rate_rad_s, 0.5);
  EXPECT_DOUBLE_EQ(twist_to_polar(0, 0, 0.5, -1.0).angular_rate_rad_s, -0.5);
  const auto z = twist_to_polar(0, 0, 0, 1.0);
  EXPECT_DOUBLE_EQ(z.velocity_mm_s, 0.0);
  EXPECT_DOUBLE_EQ(z.direction_deg, 90.0);
}

TEST(CmdVel, PolarSweep) {
  for (double vx = -0.3; vx <= 0.3; vx += 0.05) {
    for (double vy = -0.3; vy <= 0.3; vy += 0.05) {
      const auto p = twist_to_polar(vx, vy, 0.0, 1.0);
      const double speed = std::hypot(vx, vy) * 1000.0;
      EXPECT_NEAR(p.velocity_mm_s, speed, 1e-9);
      if (speed > 0) {
        const double dir = std::atan2(vx, -vy) * 180.0 / kPi;
        EXPECT_NEAR(std::remainder(p.direction_deg - dir, 360.0), 0.0, 1e-9) << vx << "," << vy;
      }
    }
  }
}

TEST(CmdVel, ForwardCmdVelGivesForwardPulses) {
  const auto p = twist_to_polar(0.1, 0.0, 0.0, 1.0);
  EXPECT_EQ(mecanum_motor_pulses(p.velocity_mm_s, p.direction_deg, p.angular_rate_rad_s, kMec),
            mecanum_motor_pulses(100.0, 90.0, 0.0, kMec));
}

TEST(WheelLimit, ScalesProportionally) {
  bool scaled = false;
  auto o = limit_wheel_pulses({200, -100, 50, 0}, 100, &scaled);
  EXPECT_TRUE(scaled);
  EXPECT_EQ(o[0], 100);
  EXPECT_EQ(o[1], -50);
  EXPECT_EQ(o[2], 25);
  EXPECT_EQ(o[3], 0);
  o = limit_wheel_pulses({25, -25, -25, 25}, 100, &scaled);
  EXPECT_FALSE(scaled);
  EXPECT_EQ(o[0], 25);
  EXPECT_EQ(o[1], -25);
  o = limit_wheel_pulses({1000, 0, 0, 0}, 500, &scaled);  // int8 상한 127
  EXPECT_EQ(o[0], 127);
}

TEST(ServoJoint, MapValidation) {
  EXPECT_FALSE(JointServoMap{}.valid());
  JointServoMap dup = kMap;
  dup.servo_ids[1] = dup.servo_ids[2];  // 같은 ID 두 번
  EXPECT_FALSE(dup.valid());
  JointServoMap baddir = kMap;
  baddir.directions[0] = 0;
  EXPECT_FALSE(baddir.valid());
  for (int j = 0; j < kNumJoints; ++j) EXPECT_EQ(kMap.joint_for_servo(kMap.id(j)), j);
  EXPECT_EQ(kMap.joint_for_servo(7), -1);
  EXPECT_EQ(kMap.id(-1), -1);
  EXPECT_EQ(kMap.gripper_id(), kMap.id(kGripperIndex));
}

TEST(ServoJoint, ConversionsAndDirection) {
  EXPECT_EQ(joint_to_servo(kMap, 0, 0.0), 500);
  // 방향 +1 관절과 −1 관절 각각 확인(시험용 프로필: joint2 가 −1)
  ASSERT_EQ(kMap.directions[1], -1);
  ASSERT_EQ(kMap.directions[2], 1);
  EXPECT_NEAR(servo_to_joint(kMap, 2, 625), deg2rad(30.0), 1e-12);   // (625−500)·0.24 = 30°
  EXPECT_NEAR(servo_to_joint(kMap, 1, 625), deg2rad(-30.0), 1e-12);  // 반대 방향
  EXPECT_EQ(joint_to_servo(kMap, 1, deg2rad(-30.0)), 625);
  EXPECT_EQ(joint_to_servo(kMap, 2, deg2rad(240.0)), 1000);  // 자르기
  EXPECT_EQ(joint_to_servo(kMap, 2, deg2rad(-240.0)), 0);
  EXPECT_EQ(joint_to_servo(kMap, 2, NAN), 500);
  EXPECT_EQ(joint_to_servo(JointServoMap{}, 2, 0.3), 500);  // 방향이 비어 있으면 중앙
  // 왕복: 서보값 → 각 → 서보값
  for (int j = 0; j < 6; ++j) {
    for (int v = 0; v <= 1000; v += 7) {
      EXPECT_EQ(joint_to_servo(kMap, j, servo_to_joint(kMap, j, v)), v) << j << " " << v;
    }
  }
}

TEST(ServoJoint, DegreeSweepMatchesReference) {
  int n = 0;
  for (double deg = -130.0; deg <= 130.0; deg += 0.37) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_EQ(joint_deg_to_servo(kMap, j, deg), ref_servo(deg, kMap.directions[j])) << j << " " << deg;
      ++n;
    }
  }
  EXPECT_GT(n, 2000);
}

TEST(Gripper, OpenCloseAndClamp) {
  const GripperParams g = kTest.gripper;  // 시험용: 열림 100, 닫힘 400
  EXPECT_EQ(gripper_to_servo(0.0, g), 100);
  EXPECT_EQ(gripper_to_servo(1.0, g), 400);
  EXPECT_EQ(gripper_to_servo(0.5, g), 250);
  EXPECT_EQ(gripper_to_servo(-1.0, g), 100);
  EXPECT_EQ(gripper_to_servo(2.0, g), 400);
  EXPECT_EQ(gripper_to_servo(NAN, g), 100);
  EXPECT_DOUBLE_EQ(servo_to_gripper(100, g), 0.0);
  EXPECT_DOUBLE_EQ(servo_to_gripper(400, g), 1.0);
  EXPECT_DOUBLE_EQ(servo_to_gripper(500, g), 1.0);
  EXPECT_DOUBLE_EQ(servo_to_gripper(0, g), 0.0);
  EXPECT_NEAR(servo_to_gripper(175, g), 0.25, 1e-12);
  GripperParams custom{150, 500};
  EXPECT_EQ(gripper_to_servo(1.0, custom), 500);
  EXPECT_EQ(gripper_to_servo(0.0, custom), 150);
  // 비어 있거나 같은 값이면 쓰지 않음(중앙)
  EXPECT_FALSE(GripperParams{}.valid());
  EXPECT_FALSE((GripperParams{300, 300}.valid()));
  EXPECT_EQ(gripper_to_servo(1.0, GripperParams{}), 500);
}
