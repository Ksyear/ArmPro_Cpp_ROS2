// 종료 정책: 첫 요청 한 번만 시스템을 끈다. 끄는 방법(PowerAction)은 바꿔 끼운다(Strategy).
//  - CommandPowerAction: 명령 실행(기본 systemctl poweroff, 셸을 거치지 않음)
//  - DryRunPowerAction : 표시 파일만 쓴다(시험·시연용, 실제로 끄지 않음)
// 요청 출처(TriggerSource)는 노드가 정한다: 토픽(기본, 지금과 같은 동작) 또는 파일(docs/12 E2 준비).
#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace armpi_power {

class PowerAction {
 public:
  virtual ~PowerAction() = default;
  // source: 요청 출처(로그·표시용)
  virtual bool execute(const std::string& source, std::string* error) = 0;
  virtual std::string describe() const = 0;
};

class CommandPowerAction : public PowerAction {
 public:
  explicit CommandPowerAction(std::vector<std::string> argv) : argv_(std::move(argv)) {}
  bool execute(const std::string& source, std::string* error) override;  // 끝날 때까지 기다리고 종료 코드 0 이면 true
  std::string describe() const override;

 private:
  std::vector<std::string> argv_;
};

class DryRunPowerAction : public PowerAction {
 public:
  explicit DryRunPowerAction(std::string marker_path) : marker_(std::move(marker_path)) {}
  bool execute(const std::string& source, std::string* error) override;  // marker 에 "poweroff (dry-run) <출처>" 를 쓴다
  std::string describe() const override { return "dry-run(표시 파일 " + marker_ + ")"; }

 private:
  std::string marker_;
};

// 첫 요청만 실행한다(두 번째부터는 무시). 실행 실패하면 다시 받을 수 있게 풀린다.
class ShutdownLatch {
 public:
  using Log = std::function<void(const std::string&)>;
  ShutdownLatch(std::unique_ptr<PowerAction> action, Log log);
  // 실행했으면 true(이미 실행했거나 실패하면 false)
  bool request(const std::string& source);
  bool done() const;
  const PowerAction& action() const { return *action_; }

 private:
  std::unique_ptr<PowerAction> action_;
  Log log_;
  mutable std::mutex mu_;
  bool done_{false};
};

bool file_exists(const std::string& path);

}  // namespace armpi_power
