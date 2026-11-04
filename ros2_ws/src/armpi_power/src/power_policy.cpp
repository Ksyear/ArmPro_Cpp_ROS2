#include "armpi_power/power_policy.hpp"

#include <spawn.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <ctime>
#include <fstream>

extern char** environ;

namespace armpi_power {

bool file_exists(const std::string& path) {
  struct stat st {};
  return ::stat(path.c_str(), &st) == 0;
}

std::string CommandPowerAction::describe() const {
  std::string s;
  for (const auto& a : argv_) s += (s.empty() ? "" : " ") + a;
  return s;
}

bool CommandPowerAction::execute(const std::string&, std::string* error) {
  if (argv_.empty()) {
    if (error) *error = "명령이 비어 있음";
    return false;
  }
  std::vector<char*> argv;
  for (auto& a : argv_) argv.push_back(const_cast<char*>(a.c_str()));
  argv.push_back(nullptr);
  pid_t pid = 0;
  const int rc = ::posix_spawnp(&pid, argv[0], nullptr, nullptr, argv.data(), environ);  // 셸을 거치지 않음
  if (rc != 0) {
    if (error) *error = std::string("실행 실패: ") + std::strerror(rc);
    return false;
  }
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0) {
    if (errno != EINTR) {
      if (error) *error = std::string("waitpid: ") + std::strerror(errno);
      return false;
    }
  }
  if (WIFEXITED(status) && WEXITSTATUS(status) == 0) return true;
  if (error) *error = "종료 코드 " + std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : -1);
  return false;
}

bool DryRunPowerAction::execute(const std::string& source, std::string* error) {
  std::ofstream f(marker_, std::ios::trunc);
  if (!f) {
    if (error) *error = "표시 파일을 쓸 수 없음: " + marker_;
    return false;
  }
  f << "poweroff (dry-run) " << source << " " << std::time(nullptr) << "\n";
  return static_cast<bool>(f);
}

ShutdownLatch::ShutdownLatch(std::unique_ptr<PowerAction> action, Log log)
    : action_(std::move(action)), log_(std::move(log)) {}

bool ShutdownLatch::request(const std::string& source) {
  std::lock_guard<std::mutex> lk(mu_);
  if (done_) {
    if (log_) log_("종료 요청 무시(이미 실행함): " + source);
    return false;
  }
  if (log_) log_("KEY2: 시스템 종료 (" + source + ") → " + action_->describe());
  std::string err;
  if (!action_->execute(source, &err)) {
    if (log_) log_("종료 실행 실패: " + err);
    return false;
  }
  done_ = true;
  return true;
}

bool ShutdownLatch::done() const {
  std::lock_guard<std::mutex> lk(mu_);
  return done_;
}

}  // namespace armpi_power
