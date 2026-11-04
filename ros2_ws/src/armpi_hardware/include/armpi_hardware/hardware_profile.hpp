// 하드웨어 프로필: 로봇마다 다르거나 제조사 공개 자료로 확인되지 않은 장치 값.
//
// 이 값들은 코드에 기본값을 두지 않는다. 실제 로봇에서는 armpi_bringup 의 hardware_profile.yaml
// (본보기: hardware_profile.template.yaml)로 넣는다. 하나라도 비어 있으면 제어 노드는 장치를 열지 않고
// fault 로 시작한다(바퀴·서보에 아무것도 보내지 않음).
// 가짜 하드웨어와 시험은 test_hardware_profile()(시험용 값, 실제 로봇 값이 아님)으로 빈 칸을 채운다.
#pragma once

#include <array>
#include <limits>
#include <string>
#include <vector>

#include "armpi_hardware/conversions.hpp"
#include "armpi_hardware/devices.hpp"

namespace armpi_hardware {

constexpr int kUnsetSpeed = std::numeric_limits<int>::min();  // fan.speed 가 비어 있음

struct HardwareProfile {
  // 차체
  MecanumParams mecanum{};
  // 팔: 관절 ↔ 서보 배정, 그리퍼, 켤 때 보낼 서보값(서보 ID1..6 순서)
  JointServoMap servo_map{};
  GripperParams gripper{};
  int gripper_servo_min{kUnsetInt};
  int gripper_servo_max{kUnsetInt};
  std::array<int, kNumJoints> home_servo_values{kUnsetInt, kUnsetInt, kUnsetInt, kUnsetInt, kUnsetInt, kUnsetInt};
  // 팔 기구학의 관절 기준(joint = sign·기하 각도 + offset)과 관절 범위(rad), joint1..joint4
  std::array<double, 4> joint_sign{kUnsetDouble, kUnsetDouble, kUnsetDouble, kUnsetDouble};
  std::array<double, 4> joint_offset{kUnsetDouble, kUnsetDouble, kUnsetDouble, kUnsetDouble};
  std::array<double, 4> joint_min{kUnsetDouble, kUnsetDouble, kUnsetDouble, kUnsetDouble};
  std::array<double, 4> joint_max{kUnsetDouble, kUnsetDouble, kUnsetDouble, kUnsetDouble};
  // 확장보드(배터리·팬)
  ExpansionBoardRegisters expansion{};
  int fan_motor_index{kUnsetInt};
  int fan_speed{kUnsetSpeed};
  // GPIO(BCM 번호)
  int servo_rx_con_bcm{kUnsetInt};
  int servo_tx_con_bcm{kUnsetInt};
  int buzzer_bcm{kUnsetInt};
  int key1_bcm{kUnsetInt};
  int key2_bcm{kUnsetInt};
  // 버스 서보 반이중 타이밍
  ServoBusTiming servo_timing{};
};

// 어떤 장치를 쓰는지(꺼 둔 장치의 값은 필요 없다)
struct ProfileUse {
  bool fan{true};
  bool buzzer{true};
  bool buttons{true};
};

// 비어 있거나 잘못된 항목의 매개변수 이름 목록(빈 목록 = 모두 있음)
std::vector<std::string> missing_profile_items(const HardwareProfile& p, const ProfileUse& use = ProfileUse{});

// 시험용 프로필. 실제 로봇 값이 아니다(일부러 다르게 둔 값). 가짜 하드웨어·단위 시험에만 쓴다.
HardwareProfile test_hardware_profile();

// p 의 빈 칸만 from 으로 채운다. 채운 항목의 매개변수 이름을 돌려준다.
std::vector<std::string> fill_unset(HardwareProfile* p, const HardwareProfile& from);

}  // namespace armpi_hardware
