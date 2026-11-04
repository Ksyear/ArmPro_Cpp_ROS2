// armpi_control_node: 비상정지 latch·fault·바퀴 쓰기·부저·UDP 허용 판단 (ControlNode 의 일부. 클래스 선언은 control_node.hpp)
#include "armpi_control/control_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

#include "armpi_common/json.hpp"
#include "control_node_detail.hpp"

namespace armpi_control {

namespace json = armpi_common::json;
namespace ah = armpi_hardware;
using detail::join;

// ───────────────────────── 동작 ─────────────────────────

void ControlNode::set_fault(const std::string& reason) {
  if (fault_) return;
  fault_ = true;
  fault_reason_ = reason;
  RCLCPP_ERROR(get_logger(), "FAULT: %s (재시작 필요)", reason.c_str());
  if (chassis_) chassis_->force_stop();
  if (arm_) arm_->hold();
  write_wheels({0, 0, 0, 0}, now_s(), true);
  if (control_state_pub_) publish_control_state(true);
}

void ControlNode::write_wheels(const std::array<int8_t, 4>& in, double now, bool force) {
  if (!hw_.motors) return;
  std::array<int8_t, 4> pulses = in;
  if (inhibit()) pulses = {0, 0, 0, 0};
  const bool changed = pulses != last_pulses_ || !last_wheel_write_ok_;
  if (!force && !changed && now - last_wheel_write_ < wheel_refresh_s_) return;
  if (!force && fault() && now - last_wheel_write_ < 0.5) return;  // fault 중 정지 재시도는 0.5 s 마다
  std::string err;
  const bool ok = hw_.motors->set_speeds(pulses, &err);
  last_wheel_write_ = now;
  if (ok) {
    last_pulses_ = pulses;
    last_wheel_write_ok_ = true;
    wheel_consecutive_failures_ = 0;
    ++wheel_writes_;
    return;
  }
  last_wheel_write_ok_ = false;
  ++wheel_failures_;
  ++wheel_consecutive_failures_;
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "바퀴 모터 쓰기 실패(%d회 연속): %s",
                       wheel_consecutive_failures_, err.c_str());
  if (wheel_consecutive_failures_ >= std::max(1, fault_threshold_)) {
    set_fault("I2C 바퀴 모터(0x34) 쓰기 연속 실패 " + std::to_string(wheel_consecutive_failures_) + "회: " + err);
  }
}

void ControlNode::beep() {
  if (!buzzer_enabled_ || !buzzer_ok_ || !hw_.gpio || estop_beep_s_ <= 0.0) return;
  const double now = now_s();
  if (buzzer_seq_.play({1, estop_beep_s_, 0.0}, now, BuzzerSequencer::kEstop)) update_buzzer(now);
}

void ControlNode::startup_beep() {
  // 부저 GPIO 가 없거나 실패했으면 조용히 건너뛴다(초기화 경고에 이미 적힘).
  if (!startup_beep_enabled_ || !buzzer_enabled_ || !buzzer_ok_ || !hw_.gpio) return;
  const bool warn = !boot_warnings_.empty() || !init_warnings_.empty();
  const int n = startup_beep_count(fault(), warn);
  const double now = now_s();
  if (buzzer_seq_.play({n, startup_beep_on_s_, startup_beep_off_s_}, now, BuzzerSequencer::kStartup)) {
    RCLCPP_INFO(get_logger(), "시작 알림 부저 %d번 (%s)", n, startup_beep_reason(fault(), warn).c_str());
    update_buzzer(now);
  }
}

void ControlNode::update_buzzer(double now) {
  if (!buzzer_ok_ || !hw_.gpio) return;
  const bool want = buzzer_seq_.level(now);
  if (want == buzzer_on_) return;
  std::string e;
  if (hw_.gpio->write(buzzer_bcm_, want ? 1 : 0, &e)) buzzer_on_ = want;
}

void ControlNode::latch_estop(const std::string& source, const std::string& why) {
  const bool newly = estop_.latch(source);
  chassis_->force_stop();
  write_wheels({0, 0, 0, 0}, now_s(), true);  // 바퀴는 즉시 0
  arm_->hold();                               // 팔은 현재 명령 자세 유지(토크 유지)
  if (newly) {
    RCLCPP_WARN(get_logger(), "비상정지 latch [%s] (%s) → 차체 정지, 팔 자세 유지. 출처: %s", source.c_str(),
                why.c_str(), join(estop_.sources()).c_str());
    beep();
  }
  publish_control_state(true);
}

void ControlNode::after_estop_cleared() {
  // 해제 뒤에는 새 명령이 와야 움직인다(수신기도 두 채널 모두 새 중립을 요구)
  chassis_->force_stop();
  arm_->hold();
  write_wheels({0, 0, 0, 0}, now_s(), true);
}

void ControlNode::drain_arm_events() {
  for (const auto& e : arm_->take_events()) {
    switch (e.level) {
      case ArmEvent::kInfo: RCLCPP_INFO(get_logger(), "팔: %s", e.text.c_str()); break;
      case ArmEvent::kWarn: RCLCPP_WARN(get_logger(), "팔: %s", e.text.c_str()); break;
      case ArmEvent::kError: RCLCPP_ERROR(get_logger(), "팔: %s", e.text.c_str()); break;
    }
  }
}

bool ControlNode::bridge_fresh(double now) const { return now - bridge_rx_time_ <= bridge_state_timeout_s_; }

bool ControlNode::owner_present(double now) const { return bridge_fresh(now) && owner_.has_value(); }

bool ControlNode::udp_allowed(double now) const {
  // home·joint_debug 시간 이동 중에는 다른 경로(서비스·디버그)가 팔을 쓰고 있으므로 UDP 활성 명령을 막는다.
  return !arm_->busy(now);
}

}  // namespace armpi_control
