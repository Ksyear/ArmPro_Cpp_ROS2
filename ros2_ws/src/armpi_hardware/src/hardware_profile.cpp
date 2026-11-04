#include "armpi_hardware/hardware_profile.hpp"

#include <cmath>

namespace armpi_hardware {
namespace {
bool unset_d(double v) { return !std::isfinite(v); }
bool unset_i(int v) { return v < 0; }
}  // namespace

std::vector<std::string> missing_profile_items(const HardwareProfile& p, const ProfileUse& use) {
  std::vector<std::string> out;
  std::string why;
  if (!p.mecanum.valid(&why)) out.push_back(why);
  if (!p.servo_map.valid(&why)) out.push_back(why);
  if (!p.gripper.valid(&why)) out.push_back(why);
  if (p.gripper_servo_min < kServoMin || p.gripper_servo_max > kServoMax || p.gripper_servo_min > p.gripper_servo_max) {
    out.push_back("arm.gripper.servo_min/servo_max 가 비어 있거나 범위 밖");
  }
  for (int v : p.home_servo_values) {
    if (v < kServoMin || v > kServoMax) {
      out.push_back("arm.home_servo_values 는 0..1000 값 6개(서보 ID1..6)여야 함");
      break;
    }
  }
  for (int i = 0; i < 4; ++i) {
    if (unset_d(p.joint_sign[i]) || p.joint_sign[i] == 0.0 || unset_d(p.joint_offset[i])) {
      out.push_back("arm.kinematics.j1..j4_sign(0 아님)·j1..j4_offset 가 비어 있음");
      break;
    }
  }
  for (int i = 0; i < 4; ++i) {
    if (unset_d(p.joint_min[i]) || unset_d(p.joint_max[i]) || !(p.joint_min[i] < p.joint_max[i])) {
      out.push_back("arm.kinematics.joint_min/joint_max 는 joint_min < joint_max 인 실수 4개여야 함");
      break;
    }
  }
  if (!p.servo_timing.valid(&why)) out.push_back(why);
  if (unset_i(p.servo_rx_con_bcm) || unset_i(p.servo_tx_con_bcm)) {
    out.push_back("hardware.servo_rx_con_bcm·servo_tx_con_bcm 가 비어 있음");
  }
  if (use.fan) {
    if (!p.expansion.valid(&why)) out.push_back(why);
    if (p.fan_motor_index < 1 || p.fan_motor_index > 4 || p.fan_speed == kUnsetSpeed) {
      out.push_back("fan.motor_index(1..4)·fan.speed 가 비어 있음");
    }
  } else if (!p.expansion.valid(&why)) {
    out.push_back(why);  // 배터리 읽기에도 확장보드 값이 필요하다
  }
  if (use.buzzer && unset_i(p.buzzer_bcm)) out.push_back("buzzer.bcm 가 비어 있음");
  if (use.buttons && (unset_i(p.key1_bcm) || unset_i(p.key2_bcm))) {
    out.push_back("buttons.key1_bcm·key2_bcm 가 비어 있음");
  }
  return out;
}

HardwareProfile test_hardware_profile() {
  // 시험용 값. 실제 로봇 값이 아니며 일부러 다르게 골랐다. 가짜 하드웨어·단위 시험 전용.
  HardwareProfile p;
  p.mecanum = MecanumParams{100.0, 90.0, 100.0, 6000.0};
  p.servo_map.servo_ids = {6, 3, 4, 5, 2, 1};      // 공개 자료: ID1 그리퍼, ID2 손목 회전, ID6 몸통 회전
  p.servo_map.directions = {1, -1, 1, 1, 1, 1};
  p.gripper = GripperParams{100, 400};
  p.gripper_servo_min = 100;
  p.gripper_servo_max = 470;
  p.home_servo_values = {100, 500, 355, 792, 128, 500};  // 서보 ID1..6
  p.joint_sign = {1.0, 1.0, -1.0, 1.0};
  p.joint_offset = {0.0, -1.4, 0.0, 0.1};
  p.joint_min = {-2.0943951023931953, -1.4, -2.0943951023931953, -1.9};
  p.joint_max = {2.0943951023931953, 1.4, 2.0943951023931953, 1.9};
  p.expansion = ExpansionBoardRegisters{0x60, 2, 40, 1};
  p.fan_motor_index = 2;
  p.fan_speed = 60;
  p.servo_rx_con_bcm = 17;
  p.servo_tx_con_bcm = 22;
  p.buzzer_bcm = 24;
  p.key1_bcm = 25;
  p.key2_bcm = 26;
  p.servo_timing.pre_write_delay_us = 400;
  p.servo_timing.post_write_delay_us = 300;
  p.servo_timing.read_turnaround_delay_us = 2500;
  p.servo_timing.read_timeout_ms = 12;
  p.servo_timing.read_attempts = 3;
  return p;
}

std::vector<std::string> fill_unset(HardwareProfile* p, const HardwareProfile& from) {
  std::vector<std::string> filled;
  auto fd = [&](double* v, double f, const char* name) {
    if (unset_d(*v)) {
      *v = f;
      filled.push_back(name);
    }
  };
  auto fi = [&](int* v, int f, const char* name) {
    if (unset_i(*v)) {
      *v = f;
      filled.push_back(name);
    }
  };
  fd(&p->mecanum.a_mm, from.mecanum.a_mm, "chassis.a_mm");
  fd(&p->mecanum.b_mm, from.mecanum.b_mm, "chassis.b_mm");
  fd(&p->mecanum.wheel_diameter_mm, from.mecanum.wheel_diameter_mm, "chassis.wheel_diameter_mm");
  fd(&p->mecanum.pulse_per_cycle, from.mecanum.pulse_per_cycle, "chassis.pulse_per_cycle");
  if (!p->servo_map.valid()) {
    p->servo_map = from.servo_map;
    filled.push_back("arm.servo_ids·arm.joint_directions");
  }
  fi(&p->gripper.open_value, from.gripper.open_value, "arm.gripper.open_value");
  fi(&p->gripper.close_value, from.gripper.close_value, "arm.gripper.close_value");
  fi(&p->gripper_servo_min, from.gripper_servo_min, "arm.gripper.servo_min");
  fi(&p->gripper_servo_max, from.gripper_servo_max, "arm.gripper.servo_max");
  bool home_unset = false;
  for (int v : p->home_servo_values) home_unset = home_unset || unset_i(v);
  if (home_unset) {
    p->home_servo_values = from.home_servo_values;
    filled.push_back("arm.home_servo_values");
  }
  const char* sn[4] = {"arm.kinematics.j1_sign", "arm.kinematics.j2_sign", "arm.kinematics.j3_sign",
                       "arm.kinematics.j4_sign"};
  const char* on[4] = {"arm.kinematics.j1_offset", "arm.kinematics.j2_offset", "arm.kinematics.j3_offset",
                       "arm.kinematics.j4_offset"};
  for (int i = 0; i < 4; ++i) {
    fd(&p->joint_sign[i], from.joint_sign[i], sn[i]);
    fd(&p->joint_offset[i], from.joint_offset[i], on[i]);
  }
  bool lim_unset = false;
  for (int i = 0; i < 4; ++i) lim_unset = lim_unset || unset_d(p->joint_min[i]) || unset_d(p->joint_max[i]);
  if (lim_unset) {
    p->joint_min = from.joint_min;
    p->joint_max = from.joint_max;
    filled.push_back("arm.kinematics.joint_min·joint_max");
  }
  fi(&p->expansion.address, from.expansion.address, "hardware.expansion_board_address");
  fi(&p->expansion.battery_register, from.expansion.battery_register, "hardware.battery_register");
  fi(&p->expansion.dc_motor_register, from.expansion.dc_motor_register, "hardware.dc_motor_register");
  if (p->expansion.dc_motor_sign != 1 && p->expansion.dc_motor_sign != -1) {
    p->expansion.dc_motor_sign = from.expansion.dc_motor_sign;
    filled.push_back("hardware.dc_motor_sign");
  }
  fi(&p->fan_motor_index, from.fan_motor_index, "fan.motor_index");
  if (p->fan_speed == kUnsetSpeed) {
    p->fan_speed = from.fan_speed;
    filled.push_back("fan.speed");
  }
  fi(&p->servo_rx_con_bcm, from.servo_rx_con_bcm, "hardware.servo_rx_con_bcm");
  fi(&p->servo_tx_con_bcm, from.servo_tx_con_bcm, "hardware.servo_tx_con_bcm");
  fi(&p->buzzer_bcm, from.buzzer_bcm, "buzzer.bcm");
  fi(&p->key1_bcm, from.key1_bcm, "buttons.key1_bcm");
  fi(&p->key2_bcm, from.key2_bcm, "buttons.key2_bcm");
  fi(&p->servo_timing.pre_write_delay_us, from.servo_timing.pre_write_delay_us, "servo.pre_write_delay_us");
  fi(&p->servo_timing.post_write_delay_us, from.servo_timing.post_write_delay_us, "servo.post_write_delay_us");
  fi(&p->servo_timing.read_turnaround_delay_us, from.servo_timing.read_turnaround_delay_us,
     "servo.read_turnaround_delay_us");
  if (p->servo_timing.read_timeout_ms <= 0) {
    p->servo_timing.read_timeout_ms = from.servo_timing.read_timeout_ms;
    filled.push_back("servo.read_timeout_ms");
  }
  if (p->servo_timing.read_attempts < 1) {
    p->servo_timing.read_attempts = from.servo_timing.read_attempts;
    filled.push_back("servo.read_attempts");
  }
  return filled;
}

}  // namespace armpi_hardware
