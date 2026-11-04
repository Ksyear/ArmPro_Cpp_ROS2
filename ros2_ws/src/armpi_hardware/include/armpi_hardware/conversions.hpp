// ArmPi Pro 하드웨어 변환 순수 함수 (ROS 비의존).
// 메카넘 차체 식, 버스 서보 값 ↔ 관절각, 그리퍼 값.
// 로봇마다 다른 값(차체 치수, 관절 ↔ 서보 배정, 그리퍼 끝값)은 코드에 두지 않고 하드웨어 프로필에서 받는다
// (hardware_profile.hpp, armpi_bringup/config/hardware_profile.template.yaml).
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <string>

namespace armpi_hardware {

constexpr double kUnsetDouble = std::numeric_limits<double>::quiet_NaN();  // 프로필에서 채울 값
constexpr int kUnsetInt = -1;

// ───────────── 메카넘 차체 ─────────────

// 차체 치수(하드웨어 프로필): a, b = 차체 중심에서 바퀴까지 mm, 바퀴 지름 mm, 바퀴 1회전당 엔코더 펄스.
struct MecanumParams {
  double a_mm{kUnsetDouble};
  double b_mm{kUnsetDouble};
  double wheel_diameter_mm{kUnsetDouble};
  double pulse_per_cycle{kUnsetDouble};
  // 네 값이 모두 유한한 양수인가
  bool valid(std::string* why = nullptr) const;
};

// 바퀴 선속도 변환: mm/s → 펄스/10 ms (실수, 자르기 전)
double speed_mm_s_to_pulse_per_10ms(double speed_mm_s, const MecanumParams& p);

// 메카넘 바퀴 속도 식(극좌표 입력).
//   velocity: mm/s, direction: 도(0 = 오른쪽, 90 = 앞), angular_rate: rad/s
//   반환: 모터 1..4 펄스/10 ms. 0 쪽으로 자른다.
//   입력이 유한값이 아니거나 차체 치수가 비어 있으면 모두 0.
std::array<int, 4> mecanum_motor_pulses(double velocity_mm_s, double direction_deg, double angular_rate_rad_s,
                                        const MecanumParams& p);

// 극좌표 입력 형식
struct PolarVelocity {
  double velocity_mm_s{0.0};
  double direction_deg{90.0};
  double angular_rate_rad_s{0.0};
};

// ROS 2 cmd_vel(REP-103: vx 앞, vy 왼쪽 m/s, wz 반시계 rad/s) → 극좌표 입력.
//   velocity = hypot(vx, vy)·1000, direction = atan2(vx, −vy)(도), angular = angular_sign·wz
//   속도가 0 이면 direction 은 90(앞)으로 둔다(값에 영향 없음).
PolarVelocity twist_to_polar(double vx_m_s, double vy_m_s, double wz_rad_s, double angular_sign);

// 펄스가 ±max_abs 를 넘으면 네 바퀴를 같은 비율로 줄여 방향을 유지한다(int8 넘침 방지).
// scaled 가 주어지면 줄였는지 알려 준다.
std::array<int8_t, 4> limit_wheel_pulses(const std::array<int, 4>& pulses, int max_abs, bool* scaled = nullptr);

// ───────────── 버스 서보 ↔ 관절 ─────────────
// 서보 위치 0..1000 ↔ 0..240°, 중앙 500 (Hiwonder 버스 서보 통신 규격)

constexpr int kServoCenter = 500;
constexpr int kServoMin = 0;
constexpr int kServoMax = 1000;
constexpr double kServoDegPerUnit = 0.24;              // 240° / 1000
constexpr double kServoUnitPerDeg = 1000.0 / 240.0;    // 서보 단위 / 도

// 관절 순서: 0 joint1(몸통 회전) … 4 joint5(손목 회전), 5 r_joint(그리퍼)
constexpr int kNumJoints = 6;
constexpr int kGripperIndex = 5;
constexpr std::array<const char*, kNumJoints> kJointNames{"joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"};

// 관절 ↔ 서보 배정(하드웨어 프로필).
//   servo_ids : 관절 i 를 움직이는 서보 ID. 1..6 이 한 번씩 나와야 한다.
//   directions: +1 이면 서보값이 늘 때 관절각이 늘고, −1 이면 반대.
struct JointServoMap {
  std::array<int, kNumJoints> servo_ids{kUnsetInt, kUnsetInt, kUnsetInt, kUnsetInt, kUnsetInt, kUnsetInt};
  std::array<int, kNumJoints> directions{0, 0, 0, 0, 0, 0};
  bool valid(std::string* why = nullptr) const;
  int id(int joint_index) const;               // 범위 밖이면 −1
  int joint_for_servo(int servo_id) const;     // 없으면 −1
  int gripper_id() const { return id(kGripperIndex); }
};

int clamp_servo(int value, int lo = kServoMin, int hi = kServoMax);

// 관절각(rad) → 서보값. 500 + dir·deg·(1000/240), 반올림은 짝수 쪽(ties-to-even), 0..1000 로 자름.
// joint_index 범위 밖, 방향이 비어 있음, 값이 유한값이 아니면 500.
int joint_to_servo(const JointServoMap& m, int joint_index, double rad);
int joint_deg_to_servo(const JointServoMap& m, int joint_index, double deg);
// 서보값 → 관절각(rad) = dir·(value − 500)·0.24°
double servo_to_joint(const JointServoMap& m, int joint_index, int value);

// 그리퍼(하드웨어 프로필): 0 = 열림, 1 = 닫힘. 서보값 = round(open + g·(close − open))
struct GripperParams {
  int open_value{kUnsetInt};
  int close_value{kUnsetInt};
  bool valid(std::string* why = nullptr) const;  // 둘 다 0..1000, 서로 다름
};
int gripper_to_servo(double g, const GripperParams& p);
double servo_to_gripper(int value, const GripperParams& p);  // 0..1 로 자름

}  // namespace armpi_hardware
