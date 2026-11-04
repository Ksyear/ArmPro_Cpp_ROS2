// 비상정지 latch 규칙, watchdog, 버튼 판정 시험
#include <gtest/gtest.h>

#include "armpi_control/safety.hpp"

using namespace armpi_control;

TEST(EstopLatch, PerSourceLatchAndRelease) {
  EstopLatch e;
  EXPECT_FALSE(e.active());
  EXPECT_TRUE(e.latch("udp"));
  EXPECT_FALSE(e.latch("udp"));  // 이미 걸림
  EXPECT_TRUE(e.latch("service"));
  EXPECT_TRUE(e.latch("button"));
  EXPECT_EQ(e.sources(), (std::vector<std::string>{"button", "service", "udp"}));
  // UDP latch:false → UDP 출처만 해제
  EXPECT_TRUE(e.release("udp"));
  EXPECT_TRUE(e.active());
  EXPECT_EQ(e.sources(), (std::vector<std::string>{"button", "service"}));
  EXPECT_FALSE(e.release("udp"));  // 이미 해제
  // reset_estop → 모두 해제
  EXPECT_EQ(e.reset_all(), (std::vector<std::string>{"button", "service"}));
  EXPECT_FALSE(e.active());
  EXPECT_TRUE(e.reset_all().empty());
}

TEST(EstopLatch, ReleasableByRequest) {
  EXPECT_TRUE(estop_source_releasable_by_request("udp"));
  EXPECT_FALSE(estop_source_releasable_by_request("service"));
  EXPECT_FALSE(estop_source_releasable_by_request("button"));
  EXPECT_TRUE(valid_estop_source("udp"));
  EXPECT_TRUE(valid_estop_source("web_ui2"));
  EXPECT_FALSE(valid_estop_source(""));
  EXPECT_FALSE(valid_estop_source("UDP"));
  EXPECT_FALSE(valid_estop_source("a b"));
  EXPECT_FALSE(valid_estop_source(std::string(33, 'a')));
}

TEST(Watchdog, NeverFedIsExpired) {
  Watchdog w(0.6);
  EXPECT_TRUE(w.expired(0.0));
  EXPECT_FALSE(w.ever_fed());
  w.feed(10.0);
  EXPECT_FALSE(w.expired(10.0));
  EXPECT_FALSE(w.expired(10.6));
  EXPECT_TRUE(w.expired(10.6001));
  w.reset();
  EXPECT_TRUE(w.expired(10.1));
}

TEST(ButtonDetector, DebouncedPressAndRelease) {
  ButtonDetector b(0.05, 2.0);
  EXPECT_EQ(b.update(false, 0.00), ButtonDetector::kNone);
  EXPECT_EQ(b.update(true, 0.02), ButtonDetector::kNone);   // 튐
  EXPECT_EQ(b.update(false, 0.04), ButtonDetector::kNone);  // 튐 끝
  EXPECT_EQ(b.update(false, 0.10), ButtonDetector::kNone);
  EXPECT_EQ(b.update(true, 0.20), ButtonDetector::kNone);
  EXPECT_EQ(b.update(true, 0.24), ButtonDetector::kNone);
  EXPECT_EQ(b.update(true, 0.26), ButtonDetector::kPressed);
  EXPECT_TRUE(b.pressed());
  EXPECT_EQ(b.update(true, 0.30), ButtonDetector::kNone);
  EXPECT_EQ(b.update(false, 0.40), ButtonDetector::kNone);
  EXPECT_EQ(b.update(false, 0.46), ButtonDetector::kReleased);
}

TEST(ButtonDetector, HoldFiresOncePerPress) {
  ButtonDetector b(0.05, 2.0);
  b.update(false, 0.0);
  int held = 0;
  for (double t = 1.0; t < 5.0; t += 0.02) {
    if (b.update(true, t) & ButtonDetector::kHeld) ++held;
  }
  EXPECT_EQ(held, 1);
  b.update(false, 5.0);
  b.update(false, 5.1);
  int held2 = 0;
  for (double t = 6.0; t < 7.5; t += 0.02) {
    if (b.update(true, t) & ButtonDetector::kHeld) ++held2;  // 1.5 s 만 누름
  }
  EXPECT_EQ(held2, 0);
}

TEST(ButtonDetector, PressedAtStartIsIgnoredUntilReleased) {
  // 시작부터 0(눌림)으로 읽히면 선로 고장일 수 있으므로 떼어질 때까지 무시
  ButtonDetector b(0.05, 2.0);
  int ev = 0;
  for (double t = 0.0; t < 3.0; t += 0.02) ev |= b.update(true, t);
  EXPECT_EQ(ev, ButtonDetector::kNone);
  EXPECT_FALSE(b.armed());
  b.update(false, 3.0);
  b.update(false, 3.1);
  EXPECT_TRUE(b.armed());
  b.update(true, 3.2);
  EXPECT_EQ(b.update(true, 3.3), ButtonDetector::kPressed);
}
