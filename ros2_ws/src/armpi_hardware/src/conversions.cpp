#include "armpi_hardware/conversions.hpp"

#include <algorithm>
#include <cmath>

namespace armpi_hardware {
namespace {
constexpr double kPi = 3.14159265358979323846;  // Python math.pi 와 같은 double

// Python int(x): 0 쪽으로 자르기. 터무니없는 값은 int 범위 안으로 먼저 제한.
int py_int(double x) {
  if (!std::isfinite(x)) return 0;
  x = std::max(-1.0e9, std::min(1.0e9, x));
  return static_cast<int>(x);
}
}  // namespace

double speed_mm_s_to_pulse_per_10ms(double speed_mm_s, const MecanumParams& p) {
  // speed / (π·바퀴 지름) · 1회전당 펄스 · 0.01 s
  return speed_mm_s / (kPi * p.wheel_diameter_mm) * p.pulse_per_cycle * 0.01;
}

std::array<int, 4> mecanum_motor_pulses(double velocity_mm_s, double direction_deg, double angular_rate_rad_s,
                                        const MecanumParams& p) {
  if (!std::isfinite(velocity_mm_s) || !std::isfinite(direction_deg) || !std::isfinite(angular_rate_rad_s) ||
      !p.valid()) {
    return {0, 0, 0, 0};
  }
  // 메카넘 바퀴 속도 식
  const double velocity = -velocity_mm_s;
  const double angular_rate = -angular_rate_rad_s;
  const double rad_per_deg = kPi / 180.0;
  const double vx = velocity * std::cos(direction_deg * rad_per_deg);
  const double vy = velocity * std::sin(direction_deg * rad_per_deg);
  const double vp = angular_rate * (p.a_mm + p.b_mm);
  const double v1 = vy - vx + vp;
  const double v2 = vy + vx - vp;
  const double v3 = vy - vx - vp;
  const double v4 = vy + vx + vp;
  return {py_int(speed_mm_s_to_pulse_per_10ms(-v1, p)), py_int(speed_mm_s_to_pulse_per_10ms(v4, p)),
          py_int(speed_mm_s_to_pulse_per_10ms(v2, p)), py_int(speed_mm_s_to_pulse_per_10ms(-v3, p))};
}

PolarVelocity twist_to_polar(double vx_m_s, double vy_m_s, double wz_rad_s, double angular_sign) {
  PolarVelocity out;
  if (!std::isfinite(vx_m_s) || !std::isfinite(vy_m_s) || !std::isfinite(wz_rad_s) || !std::isfinite(angular_sign)) {
    return out;  // 0, 90°, 0
  }
  out.velocity_mm_s = std::hypot(vx_m_s, vy_m_s) * 1000.0;
  // 방향: 0° = 오른쪽(−y), 90° = 앞(+x)
  out.direction_deg = out.velocity_mm_s > 0.0 ? std::atan2(vx_m_s, -vy_m_s) * 180.0 / kPi : 90.0;
  out.angular_rate_rad_s = angular_sign * wz_rad_s;
  return out;
}

std::array<int8_t, 4> limit_wheel_pulses(const std::array<int, 4>& pulses, int max_abs, bool* scaled) {
  max_abs = std::max(0, std::min(127, max_abs));
  int peak = 0;
  for (int v : pulses) peak = std::max(peak, std::abs(v));
  std::array<int8_t, 4> out{0, 0, 0, 0};
  const bool need = peak > max_abs;
  if (scaled) *scaled = need;
  for (int i = 0; i < 4; ++i) {
    int v = pulses[i];
    if (need) v = static_cast<int>(static_cast<double>(v) * max_abs / peak);  // 0 쪽으로 자름 → |v| ≤ max_abs
    v = std::max(-max_abs, std::min(max_abs, v));
    out[i] = static_cast<int8_t>(v);
  }
  return out;
}

int clamp_servo(int value, int lo, int hi) { return std::max(lo, std::min(hi, value)); }

bool MecanumParams::valid(std::string* why) const {
  const double v[4] = {a_mm, b_mm, wheel_diameter_mm, pulse_per_cycle};
  const char* names[4] = {"chassis.a_mm", "chassis.b_mm", "chassis.wheel_diameter_mm", "chassis.pulse_per_cycle"};
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(v[i]) || v[i] <= 0.0) {
      if (why) *why = std::string(names[i]) + " 가 비어 있거나 0 이하";
      return false;
    }
  }
  return true;
}

bool JointServoMap::valid(std::string* why) const {
  std::array<bool, kNumJoints + 1> seen{};
  for (int i = 0; i < kNumJoints; ++i) {
    const int sid = servo_ids[i];
    if (sid < 1 || sid > kNumJoints || seen[sid]) {
      if (why) *why = "arm.servo_ids 는 서보 ID 1..6 이 한 번씩 나와야 함";
      return false;
    }
    seen[sid] = true;
    if (directions[i] != 1 && directions[i] != -1) {
      if (why) *why = "arm.joint_directions 는 +1 또는 -1 이어야 함";
      return false;
    }
  }
  return true;
}

int JointServoMap::id(int joint_index) const {
  if (joint_index < 0 || joint_index >= kNumJoints) return -1;
  return servo_ids[joint_index];
}

int JointServoMap::joint_for_servo(int servo_id) const {
  for (int i = 0; i < kNumJoints; ++i) {
    if (servo_ids[i] == servo_id) return i;
  }
  return -1;
}

int joint_to_servo(const JointServoMap& m, int joint_index, double rad) {
  return joint_deg_to_servo(m, joint_index, rad * 180.0 / kPi);
}

int joint_deg_to_servo(const JointServoMap& m, int joint_index, double deg) {
  if (joint_index < 0 || joint_index >= kNumJoints || !std::isfinite(deg)) return kServoCenter;
  const int dir = m.directions[joint_index];
  if (dir != 1 && dir != -1) return kServoCenter;
  // 서보값 = round(dir·θ·(1000/240) + 500)
  const double raw = dir * deg * kServoUnitPerDeg + kServoCenter;
  const double bounded = std::max(-1.0e6, std::min(1.0e6, raw));
  return clamp_servo(static_cast<int>(std::nearbyint(bounded)));  // 기본 반올림 모드 = 짝수 쪽
}

double servo_to_joint(const JointServoMap& m, int joint_index, int value) {
  if (joint_index < 0 || joint_index >= kNumJoints) return 0.0;
  const int dir = m.directions[joint_index];
  if (dir != 1 && dir != -1) return 0.0;
  const double deg = (value - kServoCenter) * kServoDegPerUnit;
  return dir * deg * kPi / 180.0;
}

bool GripperParams::valid(std::string* why) const {
  const bool ok = open_value >= kServoMin && open_value <= kServoMax && close_value >= kServoMin &&
                  close_value <= kServoMax && open_value != close_value;
  if (!ok && why) *why = "arm.gripper.open_value/close_value 는 0..1000 의 서로 다른 값이어야 함";
  return ok;
}

int gripper_to_servo(double g, const GripperParams& p) {
  if (!p.valid()) return kServoCenter;
  if (!std::isfinite(g)) g = 0.0;
  g = std::max(0.0, std::min(1.0, g));
  const double v = p.open_value + g * (p.close_value - p.open_value);
  return clamp_servo(static_cast<int>(std::nearbyint(v)));
}

double servo_to_gripper(int value, const GripperParams& p) {
  if (!p.valid()) return 0.0;
  const double g = static_cast<double>(value - p.open_value) / (p.close_value - p.open_value);
  return std::max(0.0, std::min(1.0, g));
}

}  // namespace armpi_hardware
