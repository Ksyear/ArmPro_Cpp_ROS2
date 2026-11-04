// 장치 묶음 만들기(실제 또는 가짜).
#pragma once

#include <memory>
#include <string>

#include "armpi_hardware/devices.hpp"
#include "armpi_hardware/hardware_profile.hpp"
#include "armpi_hardware/mock.hpp"
#include "armpi_hardware/transport.hpp"

namespace armpi_hardware {

struct HardwareConfig {
  std::string i2c_device{"/dev/i2c-1"};
  uint8_t motor_board_address{kMotorBoardAddress};
  uint8_t motor_type{3};  // 엔코더 모터 드라이버 공개 예제의 JGB 모터 종류 값
  ExpansionBoardRegisters expansion{};  // 하드웨어 프로필
  std::string servo_port{"/dev/ttyS0"};
  int servo_baud{115200};           // 버스 서보 규격 통신 속도
  ServoBusTiming servo_timing{};    // 하드웨어 프로필
  std::string gpio_chip{"gpiochip0"};
  std::string gpio_consumer{"armpi_control"};
  int servo_rx_con_bcm{-1};  // 하드웨어 프로필
  int servo_tx_con_bcm{-1};  // 하드웨어 프로필
};

struct Hardware {
  bool mock{false};
  std::shared_ptr<MotorBoard> motors;
  std::shared_ptr<ExpansionBoard> expansion;
  std::shared_ptr<ServoBus> servos;
  std::shared_ptr<Gpio> gpio;  // 부저·버튼(서보 방향 핀도 이 GPIO 로 요청됨)
  // 가짜일 때만 채워짐(시험·상태 보고용)
  std::shared_ptr<MockMotorBoard> mock_motors;
  std::shared_ptr<MockExpansionBoard> mock_expansion;
  std::shared_ptr<MockServoBus> mock_servos;
  std::shared_ptr<MockGpio> mock_gpio;
};

// expansion: 가짜 확장보드의 레지스터 값(기본: 시험용 프로필 값)
Hardware make_mock_hardware(ClockFn clock = steady_seconds, int battery_mv = 7400,
                            ExpansionBoardRegisters expansion = test_hardware_profile().expansion);

// 실제 장치 열기. 일부가 실패해도 열린 장치는 채워서 돌려주고 errors 에 이유를 모은다(빈 문자열 = 모두 성공).
// 모터 보드 init(레지스터 20)과 서보 방향 핀 초기화까지 한다.
Hardware open_real_hardware(const HardwareConfig& cfg, std::string* errors);

}  // namespace armpi_hardware
