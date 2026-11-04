// 차체: 가속 제한·즉시 정지·속도 상한·stamp·2차 watchdog·바퀴 펄스 시험
#include <gtest/gtest.h>

#include <cmath>

#include "armpi_control/chassis_controller.hpp"
#include "armpi_hardware/hardware_profile.hpp"

using namespace armpi_control;

TEST(Slew, AccelerationLimitedStopImmediate) {
  // 0.6 m/s², 50 Hz → 한 틱 0.012 m/s
  double v = 0.0;
  v = slew_axis(v, 0.1, 0.012);
  EXPECT_NEAR(v, 0.012, 1e-12);
  for (int i = 0; i < 20; ++i) v = slew_axis(v, 0.1, 0.012);
  EXPECT_NEAR(v, 0.1, 1e-12);
  EXPECT_EQ(slew_axis(v, 0.0, 0.012), 0.0);           // 정지 즉시
  EXPECT_NEAR(slew_axis(0.1, 0.05, 0.012), 0.05, 1e-12);  // 감속 즉시
  // 반전: 먼저 0, 같은 틱에서 반대 방향으로 한 걸음
  EXPECT_NEAR(slew_axis(0.1, -0.1, 0.012), -0.012, 1e-12);
  EXPECT_EQ(slew_axis(0.05, NAN, 0.012), 0.0);
}

TEST(Clamp, SpeedLimitsAndNonFinite) {
  ChassisConfig c;
  const auto t = clamp_twist(Twist2D{0.5, -0.5, 3.0}, c);
  EXPECT_DOUBLE_EQ(t.vx, 0.10);
  EXPECT_DOUBLE_EQ(t.vy, -0.10);
  EXPECT_DOUBLE_EQ(t.wz, 0.60);
  const auto n = clamp_twist(Twist2D{NAN, INFINITY, 0.1}, c);
  EXPECT_DOUBLE_EQ(n.vx, 0.0);
  EXPECT_DOUBLE_EQ(n.vy, 0.0);
  EXPECT_DOUBLE_EQ(n.wz, 0.1);
}

TEST(Stamp, FreshWithinHalfSecond) {
  EXPECT_TRUE(cmd_stamp_fresh(0.0, 0.5));
  EXPECT_TRUE(cmd_stamp_fresh(0.49, 0.5));
  EXPECT_TRUE(cmd_stamp_fresh(0.5, 0.5));
  EXPECT_FALSE(cmd_stamp_fresh(0.51, 0.5));
  EXPECT_FALSE(cmd_stamp_fresh(-0.51, 0.5));  // 미래 stamp 도 같은 한도
  EXPECT_FALSE(cmd_stamp_fresh(NAN, 0.5));
  EXPECT_FALSE(cmd_stamp_fresh(INFINITY, 0.5));
}

TEST(ChassisController, RampWatchdogAndForceStop) {
  ChassisConfig cfg;
  ChassisController c(cfg);
  double t = 100.0;
  const double dt = 0.02;
  // 명령 없음 → 0
  EXPECT_TRUE(c.tick(t, dt, false).is_zero());
  EXPECT_FALSE(c.on_command(Twist2D{0.1, 0.0, 0.3}, t));
  auto o = c.tick(t, dt, false);
  EXPECT_NEAR(o.vx, 0.012, 1e-12);
  EXPECT_NEAR(o.wz, 0.08, 1e-12);  // 4 rad/s² × 0.02
  for (int i = 0; i < 9; ++i) {
    t += dt;
    o = c.tick(t, dt, false);
  }
  EXPECT_NEAR(o.vx, 0.1, 1e-12);   // 0.012×10 > 0.1
  EXPECT_NEAR(o.wz, 0.3, 1e-12);   // 0.08×10 > 0.3
  // watchdog 0.6 s: 마지막 명령 t=100.0 → 100.6 까지는 유지, 넘으면 0
  o = c.tick(100.59, dt, false);
  EXPECT_FALSE(o.is_zero());
  o = c.tick(100.61, dt, false);
  EXPECT_TRUE(o.is_zero());
  EXPECT_TRUE(c.watchdog_expired(100.61));
  // 다시 명령 → 다시 가속
  c.on_command(Twist2D{0.1, 0.0, 0.0}, 101.0);
  o = c.tick(101.0, dt, false);
  EXPECT_NEAR(o.vx, 0.012, 1e-12);
  // inhibit(비상정지) → 즉시 0
  o = c.tick(101.02, dt, true);
  EXPECT_TRUE(o.is_zero());
  // force_stop 뒤에는 이전 명령을 잊는다: inhibit 이 풀려도 새 명령 전까지 0
  c.force_stop();
  o = c.tick(101.04, dt, false);
  EXPECT_TRUE(o.is_zero());
}

TEST(ChassisController, StopCommandIsImmediate) {
  ChassisController c(ChassisConfig{});
  c.on_command(Twist2D{0.1, 0.0, 0.0}, 0.0);
  for (int i = 0; i < 20; ++i) c.tick(i * 0.02, 0.02, false);
  EXPECT_NEAR(c.output().vx, 0.1, 1e-12);
  EXPECT_TRUE(c.on_command(Twist2D{}, 0.5));  // 정지 → 호출자가 즉시 출력
  EXPECT_TRUE(c.output().is_zero());
}

TEST(ChassisController, WheelPulsesAndAngularSign) {
  ChassisConfig cfg;
  cfg.mecanum = armpi_hardware::test_hardware_profile().mecanum;  // 시험용 치수(실제 로봇 값 아님)
  ChassisController c(cfg);
  auto p = c.wheel_pulses(Twist2D{0.1, 0.0, 0.0});
  EXPECT_EQ(p[0], 19);  // 100/(π·100)·6000·0.01 = 19.10
  EXPECT_EQ(p[1], -19);
  EXPECT_EQ(p[2], -19);
  EXPECT_EQ(p[3], 19);
  // 회전: angular_sign 을 뒤집으면 펄스 부호가 뒤집힌다
  const auto r1 = c.wheel_pulses(Twist2D{0.0, 0.0, 0.6});
  cfg.angular_sign = -1.0;
  ChassisController c2(cfg);
  const auto r2 = c2.wheel_pulses(Twist2D{0.0, 0.0, 0.6});
  for (int i = 0; i < 4; ++i) EXPECT_EQ(r1[i], -r2[i]);
  EXPECT_EQ(r1[0], 21);  // 시험용 치수: 0.6·(100+90)/(π·100)·6000·0.01 = 21.8 → [21, −21, 21, −21]
  EXPECT_EQ(r1[1], -21);
  // 모든 축 최대: 상한(100) 안
  bool scaled = true;
  const auto m = c.wheel_pulses(Twist2D{0.1, 0.1, 0.6}, &scaled);
  EXPECT_FALSE(scaled);
  for (int v : m) EXPECT_LE(std::abs(v), 100);
}
