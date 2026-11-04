#include "armpi_control/chassis_controller.hpp"

#include <algorithm>
#include <cmath>

namespace armpi_control {

double slew_axis(double current, double target, double max_step) {
  if (!std::isfinite(target) || target == 0.0) return 0.0;
  if (!std::isfinite(current)) current = 0.0;
  if (current != 0.0 && ((current > 0.0) != (target > 0.0))) current = 0.0;  // 반전: 먼저 0
  if (std::fabs(target) <= std::fabs(current)) return target;                 // 감속은 즉시
  const double step = std::min(std::fabs(target) - std::fabs(current), std::max(0.0, max_step));
  return current + std::copysign(step, target);
}

Twist2D clamp_twist(const Twist2D& t, const ChassisConfig& c) {
  auto cl = [](double v, double m) {
    if (!std::isfinite(v)) return 0.0;
    m = std::fabs(m);
    return std::max(-m, std::min(m, v));
  };
  return Twist2D{cl(t.vx, c.max_vx_m_s), cl(t.vy, c.max_vy_m_s), cl(t.wz, c.max_wz_rad_s)};
}

bool cmd_stamp_fresh(double age_s, double max_age_s) {
  return std::isfinite(age_s) && age_s <= max_age_s && age_s >= -max_age_s;
}

ChassisController::ChassisController(const ChassisConfig& cfg) : cfg_(cfg), watchdog_(cfg.cmd_timeout_s) {}

bool ChassisController::on_command(const Twist2D& cmd, double now) {
  target_ = clamp_twist(cmd, cfg_);
  watchdog_.feed(now);
  if (target_.is_zero()) {
    out_ = Twist2D{};  // 정지는 즉시
    return true;
  }
  return false;
}

void ChassisController::force_stop() {
  target_ = Twist2D{};
  out_ = Twist2D{};
  watchdog_.reset();
}

Twist2D ChassisController::tick(double now, double dt, bool inhibit) {
  Twist2D target = target_;
  if (inhibit || watchdog_.expired(now)) target = Twist2D{};
  dt = std::max(0.0, std::min(dt, 0.2));  // 실행기 지연으로 dt 가 튀어도 한 번에 크게 가속하지 않게
  const double lin = cfg_.max_linear_accel_m_s2 * dt;
  const double ang = cfg_.max_angular_accel_rad_s2 * dt;
  out_.vx = slew_axis(out_.vx, target.vx, lin);
  out_.vy = slew_axis(out_.vy, target.vy, lin);
  out_.wz = slew_axis(out_.wz, target.wz, ang);
  return out_;
}

std::array<int8_t, 4> ChassisController::wheel_pulses(const Twist2D& out, bool* scaled) const {
  const auto p = armpi_hardware::twist_to_polar(out.vx, out.vy, out.wz, cfg_.angular_sign);
  const auto m = armpi_hardware::mecanum_motor_pulses(p.velocity_mm_s, p.direction_deg, p.angular_rate_rad_s,
                                                      cfg_.mecanum);
  return armpi_hardware::limit_wheel_pulses(m, cfg_.max_wheel_pulse, scaled);
}

}  // namespace armpi_control
