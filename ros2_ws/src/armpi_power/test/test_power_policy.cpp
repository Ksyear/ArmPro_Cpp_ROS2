// 종료 정책 시험: 한 번만 실행, 실패하면 다시 받음, dry-run 표시 파일, 명령 실행(셸 없이)
#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>

#include "armpi_power/power_policy.hpp"

using namespace armpi_power;

namespace {
struct CountingAction : PowerAction {
  int* calls;
  bool ok;
  CountingAction(int* c, bool o) : calls(c), ok(o) {}
  bool execute(const std::string&, std::string* err) override {
    ++*calls;
    if (!ok) *err = "가짜 실패";
    return ok;
  }
  std::string describe() const override { return "가짜"; }
};
}  // namespace

TEST(PowerPolicy, OnlyFirstRequestExecutes) {
  int calls = 0;
  std::vector<std::string> log;
  ShutdownLatch latch(std::make_unique<CountingAction>(&calls, true), [&](const std::string& m) { log.push_back(m); });
  EXPECT_TRUE(latch.request("토픽"));
  EXPECT_FALSE(latch.request("토픽"));
  EXPECT_EQ(calls, 1);
  EXPECT_TRUE(latch.done());
  EXPECT_NE(log.front().find("KEY2: 시스템 종료"), std::string::npos);
}

TEST(PowerPolicy, FailedExecutionCanBeRetried) {
  int calls = 0;
  ShutdownLatch latch(std::make_unique<CountingAction>(&calls, false), nullptr);
  EXPECT_FALSE(latch.request("a"));
  EXPECT_FALSE(latch.request("b"));
  EXPECT_EQ(calls, 2);
  EXPECT_FALSE(latch.done());
}

TEST(PowerPolicy, DryRunWritesMarker) {
  char dir[] = "/tmp/armpi_power_XXXXXX";
  ASSERT_NE(::mkdtemp(dir), nullptr);
  const std::string marker = std::string(dir) + "/marker";
  ShutdownLatch latch(std::make_unique<DryRunPowerAction>(marker), nullptr);
  EXPECT_FALSE(file_exists(marker));
  EXPECT_TRUE(latch.request("토픽 /armpi/shutdown_requested"));
  std::ifstream f(marker);
  std::string line;
  std::getline(f, line);
  EXPECT_EQ(line.rfind("poweroff (dry-run) 토픽 /armpi/shutdown_requested", 0), 0u);
}

TEST(PowerPolicy, CommandActionRunsWithoutShell) {
  std::string err;
  EXPECT_TRUE(CommandPowerAction({"true"}).execute("x", &err)) << err;
  EXPECT_FALSE(CommandPowerAction({"false"}).execute("x", &err));
  EXPECT_EQ(err, "종료 코드 1");
  EXPECT_FALSE(CommandPowerAction({"/no/such/armpi-cmd"}).execute("x", &err));
  EXPECT_FALSE(CommandPowerAction({}).execute("x", &err));
  // 인자는 셸로 해석되지 않는다(; 가 그대로 인자)
  EXPECT_TRUE(CommandPowerAction({"test", "a;b", "=", "a;b"}).execute("x", &err)) << err;
  EXPECT_EQ(CommandPowerAction({"systemctl", "poweroff"}).describe(), "systemctl poweroff");
}
