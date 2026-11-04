#include "armpi_hardware/hardware.hpp"

namespace armpi_hardware {

Hardware make_mock_hardware(ClockFn clock, int battery_mv, ExpansionBoardRegisters expansion) {
  Hardware hw;
  hw.mock = true;
  hw.mock_motors = std::make_shared<MockMotorBoard>();
  hw.mock_expansion = std::make_shared<MockExpansionBoard>(battery_mv, expansion);
  hw.mock_servos = std::make_shared<MockServoBus>(std::move(clock));
  hw.mock_gpio = std::make_shared<MockGpio>();
  hw.motors = hw.mock_motors;
  hw.expansion = hw.mock_expansion;
  hw.servos = hw.mock_servos;
  hw.gpio = hw.mock_gpio;
  std::string ignore;
  hw.motors->init(&ignore);
  return hw;
}

Hardware open_real_hardware(const HardwareConfig& cfg, std::string* errors) {
  Hardware hw;
  hw.mock = false;
  std::string all;
  auto add = [&all](const std::string& e) {
    if (e.empty()) return;
    if (!all.empty()) all += "; ";
    all += e;
  };

  std::string e;
  auto i2c = open_linux_i2c_bus(cfg.i2c_device, &e);
  add(e);
  if (i2c) {
    auto mb = std::make_shared<I2cMotorBoard>(i2c, cfg.motor_board_address, cfg.motor_type);
    e.clear();
    if (mb->init(&e)) {
      hw.motors = mb;
    } else {
      add("모터 보드 초기화 실패: " + e);
      hw.motors = mb;  // 정지 명령 시도는 할 수 있게 남겨 둔다
    }
    hw.expansion = std::make_shared<I2cExpansionBoard>(i2c, cfg.expansion);
  }

  e.clear();
  hw.gpio = open_gpiod_gpio(cfg.gpio_chip, cfg.gpio_consumer, &e);
  add(e);

  e.clear();
  auto port = open_linux_serial_port(cfg.servo_port, cfg.servo_baud, &e);
  add(e);
  if (port && hw.gpio) {
    auto bus = std::make_shared<HalfDuplexServoBus>(port, hw.gpio, cfg.servo_rx_con_bcm, cfg.servo_tx_con_bcm,
                                                  cfg.servo_timing);
    e.clear();
    if (bus->init(&e)) {
      hw.servos = bus;
    } else {
      add("서보 방향 핀 초기화 실패: " + e);
    }
  }
  if (errors) *errors = all;
  return hw;
}

}  // namespace armpi_hardware
