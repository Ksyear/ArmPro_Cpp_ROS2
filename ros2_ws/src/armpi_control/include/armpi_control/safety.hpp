// 비상정지 latch, watchdog, 버튼 판정 (ROS 비의존, 단조 시계 초 단위).
#pragma once

#include <limits>
#include <set>
#include <string>
#include <vector>

namespace armpi_control {

// 출처별 비상정지 latch.
//   - latch(출처): 그 출처 latch (어떤 출처든 걸 수 있다)
//   - release(출처): 그 출처만 해제 (예: UDP latch:false → "udp" 만)
//   - reset_all(): /armpi/reset_estop — 모든 출처 해제
class EstopLatch {
 public:
  // 새로 추가된 출처면 true
  bool latch(const std::string& source);
  // 그 출처가 걸려 있었으면 true
  bool release(const std::string& source);
  // 해제된 출처 목록(정렬)
  std::vector<std::string> reset_all();
  bool active() const { return !sources_.empty(); }
  bool has(const std::string& source) const { return sources_.count(source) > 0; }
  std::vector<std::string> sources() const { return {sources_.begin(), sources_.end()}; }

 private:
  std::set<std::string> sources_;
};

// 토픽(/armpi/estop_request)으로 해제할 수 없는 출처: 서비스·버튼 latch 는 /armpi/reset_estop 으로만 푼다.
bool estop_source_releasable_by_request(const std::string& source);
// 출처 이름 검사: 1..32자 [a-z0-9_]
bool valid_estop_source(const std::string& source);

class Watchdog {
 public:
  explicit Watchdog(double timeout_s) : timeout_(timeout_s) {}
  void feed(double now) { last_ = now; }
  void reset() { last_ = -std::numeric_limits<double>::infinity(); }
  bool expired(double now) const { return !(now - last_ <= timeout_); }  // 한 번도 안 먹였으면 만료
  double age(double now) const { return now - last_; }
  bool ever_fed() const { return last_ > -std::numeric_limits<double>::infinity(); }
  double timeout() const { return timeout_; }
  void set_timeout(double t) { timeout_ = t; }

 private:
  double timeout_;
  double last_{-std::numeric_limits<double>::infinity()};
};

// 버튼(누르면 raw_pressed=true) 판정: debounce, 누름·뗌·길게 누름(1회).
// 시작 시 이미 눌려 있으면(선로 고장·풀업 실패 등) 한 번 떼어질 때까지 누름을 무시한다.
class ButtonDetector {
 public:
  enum Event : int { kNone = 0, kPressed = 1, kReleased = 2, kHeld = 4 };
  ButtonDetector(double debounce_s, double hold_s) : debounce_(debounce_s), hold_(hold_s) {}
  int update(bool raw_pressed, double now);
  bool pressed() const { return stable_; }
  bool armed() const { return armed_; }

 private:
  double debounce_;
  double hold_;
  bool initialized_{false};
  bool stable_{false};
  bool candidate_{false};
  double candidate_since_{0.0};
  double pressed_since_{0.0};
  bool held_fired_{false};
  bool armed_{false};
};

}  // namespace armpi_control
