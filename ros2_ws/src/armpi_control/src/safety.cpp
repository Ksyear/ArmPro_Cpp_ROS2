#include "armpi_control/safety.hpp"

namespace armpi_control {

bool EstopLatch::latch(const std::string& source) { return sources_.insert(source).second; }

bool EstopLatch::release(const std::string& source) { return sources_.erase(source) > 0; }

std::vector<std::string> EstopLatch::reset_all() {
  std::vector<std::string> out(sources_.begin(), sources_.end());
  sources_.clear();
  return out;
}

bool estop_source_releasable_by_request(const std::string& source) {
  return source != "service" && source != "button";
}

bool valid_estop_source(const std::string& s) {
  if (s.empty() || s.size() > 32) return false;
  for (char c : s) {
    if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
  }
  return true;
}

int ButtonDetector::update(bool raw, double now) {
  int ev = kNone;
  if (!initialized_) {
    initialized_ = true;
    stable_ = candidate_ = raw;
    candidate_since_ = now;
    armed_ = !raw;  // 처음부터 눌려 있으면 떼어질 때까지 무시
    pressed_since_ = now;
    held_fired_ = raw;
    return ev;
  }
  if (raw != candidate_) {
    candidate_ = raw;
    candidate_since_ = now;
  }
  if (candidate_ != stable_ && now - candidate_since_ >= debounce_) {
    stable_ = candidate_;
    if (stable_) {
      pressed_since_ = candidate_since_;
      held_fired_ = false;
      if (armed_) ev |= kPressed;
    } else {
      if (armed_) ev |= kReleased;
      armed_ = true;
    }
  }
  if (stable_ && armed_ && !held_fired_ && now - pressed_since_ >= hold_) {
    held_fired_ = true;
    ev |= kHeld;
  }
  return ev;
}

}  // namespace armpi_control
