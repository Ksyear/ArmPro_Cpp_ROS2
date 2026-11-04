// 차체 제어(ROS 비의존): cmd_vel 보관 → 2차 watchdog → 속도 상한 → 가속 제한 → 메카넘 바퀴 펄스.
#pragma once

#include <array>
#include <cstdint>

#include "armpi_control/safety.hpp"
#include "armpi_hardware/conversions.hpp"

namespace armpi_control {

struct Twist2D {
  double vx{0.0};  // m/s, 앞
  double vy{0.0};  // m/s, 왼쪽
  double wz{0.0};  // rad/s, 반시계
  bool is_zero() const { return vx == 0.0 && vy == 0.0 && wz == 0.0; }
  bool operator==(const Twist2D& o) const { return vx == o.vx && vy == o.vy && wz == o.wz; }
  bool operator!=(const Twist2D& o) const { return !(*this == o); }
};

struct ChassisConfig {
  // 라이브러리 기본값(보수값, 단위 시험 기준). 로봇에서는 노드 매개변수(armpi.yaml)가 덮어쓴다
  double max_vx_m_s{0.10};
  double max_vy_m_s{0.10};
  double max_wz_rad_s{0.60};
  double max_linear_accel_m_s2{0.6};    // 직선 가속 상한
  double max_angular_accel_rad_s2{4.0};  // 회전 가속 상한
  double cmd_timeout_s{0.6};             // 2차 watchdog
  double max_cmd_age_s{0.5};             // header.stamp 가 이보다 오래되면 버림
  double angular_sign{1.0};              // angular = angular_sign × wz
  int max_wheel_pulse{100};              // 넘으면 네 바퀴를 같은 비율로 줄임
  armpi_hardware::MecanumParams mecanum{};
};

// 축 하나의 가속 제한. 목표 0·감속은 즉시, 부호 반전은 먼저 0 으로(즉시) 간 뒤 가속.
double slew_axis(double current, double target, double max_step);
Twist2D clamp_twist(const Twist2D& t, const ChassisConfig& c);
// stamp 나이(현재 − stamp, 초)가 유효한지: |age| ≤ max_age (미래 stamp 도 같은 한도)
bool cmd_stamp_fresh(double age_s, double max_age_s);

class ChassisController {
 public:
  explicit ChassisController(const ChassisConfig& cfg);

  // 유효성(stamp·유한값)을 통과한 명령. 상한으로 자른 뒤 보관하고 watchdog 을 먹인다.
  // 반환: 정지 명령(모두 0)이면 true → 호출자가 즉시 출력하도록
  bool on_command(const Twist2D& cmd, double now);
  // 비상정지·fault: 출력 0, 보관한 명령도 잊는다(해제 뒤에는 새 명령이 와야 움직임)
  void force_stop();
  // 50 Hz. inhibit(비상정지·fault)이면 0. watchdog 만료면 0.
  Twist2D tick(double now, double dt, bool inhibit);

  bool watchdog_expired(double now) const { return watchdog_.expired(now); }
  bool watchdog_ever_fed() const { return watchdog_.ever_fed(); }
  const Twist2D& output() const { return out_; }
  const Twist2D& target() const { return target_; }
  // 현재 출력 → 모터 1..4 펄스(메카넘 식) + 상한 비율 축소
  std::array<int8_t, 4> wheel_pulses(const Twist2D& out, bool* scaled = nullptr) const;
  const ChassisConfig& config() const { return cfg_; }

 private:
  ChassisConfig cfg_;
  Watchdog watchdog_;
  Twist2D target_{};
  Twist2D out_{};
};

}  // namespace armpi_control
