// 부저 패턴 재생기 시험: 시작 알림 횟수, 울림·쉼 시간, 비상정지 우선순위.
#include <gtest/gtest.h>

#include <vector>

#include "armpi_control/buzzer_pattern.hpp"

using namespace armpi_control;

namespace {
// 20 ms 주기(노드 misc_tick 50 Hz)로 읽어 켜짐 구간(시작, 끝)을 모은다.
std::vector<std::pair<double, double>> sample(const BuzzerSequencer& s, double t0, double t1, double dt = 0.001) {
  std::vector<std::pair<double, double>> on;
  bool prev = false;
  double start = 0.0;
  for (double t = t0; t <= t1; t += dt) {
    const bool lv = s.level(t);
    if (lv && !prev) start = t;
    if (!lv && prev) on.emplace_back(start, t);
    prev = lv;
  }
  if (prev) on.emplace_back(start, t1);
  return on;
}
}  // namespace

TEST(BuzzerPattern, StartupCountRules) {
  EXPECT_EQ(startup_beep_count(false, false), 1);
  EXPECT_EQ(startup_beep_count(false, true), 2);
  EXPECT_EQ(startup_beep_count(true, false), 3);
  EXPECT_EQ(startup_beep_count(true, true), 3);
}

TEST(BuzzerPattern, TwoBeepsTimingAndOrder) {
  BuzzerSequencer s;
  ASSERT_TRUE(s.play({2, 0.1, 0.15}, 10.0, BuzzerSequencer::kStartup));
  const auto on = sample(s, 9.9, 10.6);
  ASSERT_EQ(on.size(), 2u);
  EXPECT_NEAR(on[0].first, 10.0, 0.002);
  EXPECT_NEAR(on[0].second, 10.1, 0.002);
  EXPECT_NEAR(on[1].first, 10.25, 0.002);
  EXPECT_NEAR(on[1].second, 10.35, 0.002);
  EXPECT_TRUE(s.active(10.3));
  EXPECT_FALSE(s.active(10.36));
  EXPECT_FALSE(s.level(11.0));
}

TEST(BuzzerPattern, ThreeBeepsForFault) {
  BuzzerSequencer s;
  s.play({startup_beep_count(true, false), 0.1, 0.15}, 0.0, BuzzerSequencer::kStartup);
  EXPECT_EQ(sample(s, 0.0, 1.0).size(), 3u);
}

TEST(BuzzerPattern, EstopOverridesStartupAndStartupCannotInterruptEstop) {
  BuzzerSequencer s;
  s.play({3, 0.1, 0.15}, 0.0, BuzzerSequencer::kStartup);
  // 시작 알림 중 비상정지: 바로 덮어쓴다
  EXPECT_TRUE(s.play({1, 0.5, 0.0}, 0.12, BuzzerSequencer::kEstop));
  EXPECT_TRUE(s.level(0.13));
  EXPECT_TRUE(s.level(0.6));
  EXPECT_FALSE(s.level(0.63));
  // 비상정지 울림 중에는 시작 알림이 끼어들지 못한다
  BuzzerSequencer s2;
  s2.play({1, 0.5, 0.0}, 0.0, BuzzerSequencer::kEstop);
  EXPECT_FALSE(s2.play({1, 0.1, 0.15}, 0.1, BuzzerSequencer::kStartup));
  EXPECT_TRUE(s2.level(0.4));
  // 끝난 뒤에는 다시 받는다
  EXPECT_TRUE(s2.play({1, 0.1, 0.15}, 0.6, BuzzerSequencer::kStartup));
}

TEST(BuzzerPattern, InvalidPatternIgnored) {
  BuzzerSequencer s;
  EXPECT_FALSE(s.play({0, 0.1, 0.1}, 0.0, BuzzerSequencer::kStartup));
  EXPECT_FALSE(s.play({1, 0.0, 0.1}, 0.0, BuzzerSequencer::kStartup));
  EXPECT_FALSE(s.level(0.05));
  EXPECT_FALSE(s.active(0.05));
}
