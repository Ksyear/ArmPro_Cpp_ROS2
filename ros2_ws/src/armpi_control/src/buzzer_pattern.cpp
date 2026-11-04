#include "armpi_control/buzzer_pattern.hpp"

#include <cmath>

namespace armpi_control {

int startup_beep_count(bool fault, bool has_warnings) {
  if (fault) return 3;
  return has_warnings ? 2 : 1;
}

std::string startup_beep_reason(bool fault, bool has_warnings) {
  if (fault) return "fault 로 시작";
  return has_warnings ? "경고 있음(부팅 점검·초기화)" : "정상 시작";
}

bool BuzzerSequencer::play(const BeepPattern& p, double now, int priority) {
  if (p.count <= 0 || !(p.on_s > 0.0)) return false;
  if (active(now) && priority < priority_) return false;
  pattern_ = p;
  if (!(pattern_.off_s >= 0.0)) pattern_.off_s = 0.0;
  start_ = now;
  priority_ = priority;
  active_ = true;
  return true;
}

bool BuzzerSequencer::active(double now) const {
  if (!active_) return false;
  const double period = pattern_.on_s + pattern_.off_s;
  const double end = start_ + (pattern_.count - 1) * period + pattern_.on_s;  // 마지막 울림이 끝나는 시각
  return now < end;
}

bool BuzzerSequencer::level(double now) const {
  if (!active(now) || now < start_) return false;
  const double period = pattern_.on_s + pattern_.off_s;
  const double t = now - start_;
  const int k = period > 0.0 ? static_cast<int>(std::floor(t / period)) : 0;
  if (k >= pattern_.count) return false;
  return (t - k * period) < pattern_.on_s;
}

}  // namespace armpi_control
