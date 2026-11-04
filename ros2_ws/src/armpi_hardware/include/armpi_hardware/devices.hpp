// ArmPi Pro 장치 인터페이스와 실제 구현.
//   MotorBoard     : I2C 0x34 엔코더 모터 드라이버(바퀴 4개). 주소·레지스터는 Hiwonder 엔코더 모터 드라이버 공개 예제 기준
//   ExpansionBoard : 확장보드(배터리 전압, DC 모터 = 팬). 주소·레지스터는 하드웨어 프로필에서 받는다
//   ServoBus       : 반이중 UART 버스 서보(+ GPIO 방향 전환). 방향 핀·타이밍은 하드웨어 프로필에서 받는다
// 가짜 구현은 mock.hpp.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "armpi_hardware/transport.hpp"

namespace armpi_hardware {

// ───────────── 바퀴 모터 보드 ─────────────
class MotorBoard {
 public:
  virtual ~MotorBoard() = default;
  virtual bool init(std::string* err) = 0;
  // 모터 1..4 속도(펄스/10 ms, signed int8)
  virtual bool set_speeds(const std::array<int8_t, 4>& pulses, std::string* err) = 0;
};

constexpr uint8_t kMotorBoardAddress = 0x34;
constexpr uint8_t kMotorTypeRegister = 20;
constexpr uint8_t kMotorSpeedRegister = 51;  // 4개 한꺼번에

class I2cMotorBoard : public MotorBoard {
 public:
  I2cMotorBoard(std::shared_ptr<I2cBus> bus, uint8_t address = kMotorBoardAddress, uint8_t motor_type = 3);
  // 레지스터 20 ← [motor_type]
  bool init(std::string* err) override;
  // 레지스터 51 ← [m1, m2, m3, m4] (2의 보수 바이트)
  bool set_speeds(const std::array<int8_t, 4>& pulses, std::string* err) override;

 private:
  std::shared_ptr<I2cBus> bus_;
  uint8_t address_;
  uint8_t motor_type_;
};

// ───────────── 확장보드 ─────────────
class ExpansionBoard {
 public:
  virtual ~ExpansionBoard() = default;
  virtual std::optional<int> read_battery_mv(std::string* err) = 0;
  // 입력: index 1..4, speed −100..100. 보드에는 부호를 반전해 쓴다.
  virtual bool set_dc_motor(int index, int speed, std::string* err) = 0;
};

// 확장보드 주소·레지스터(하드웨어 프로필). −1 = 비어 있음.
//   battery_register : 이 레지스터 번호를 쓰고 2 B(little-endian mV)를 읽는다
//   dc_motor_register: DC 모터 1번 레지스터. n번은 dc_motor_register + (n − 1)
//   dc_motor_sign    : 보드에 쓸 때 속도에 곱하는 값(+1 또는 −1)
struct ExpansionBoardRegisters {
  int address{-1};
  int battery_register{-1};
  int dc_motor_register{-1};
  int dc_motor_sign{0};
  bool valid(std::string* why = nullptr) const;
};

// DC 모터 레지스터 쓰기 값(−100..100 로 자르고 dc_motor_sign 을 곱한 2의 보수 바이트).
// index 범위 밖이거나 레지스터가 비어 있으면 nullopt.
std::optional<std::pair<uint8_t, uint8_t>> dc_motor_register_write(const ExpansionBoardRegisters& regs, int index,
                                                                   int speed);

class I2cExpansionBoard : public ExpansionBoard {
 public:
  I2cExpansionBoard(std::shared_ptr<I2cBus> bus, ExpansionBoardRegisters regs);
  std::optional<int> read_battery_mv(std::string* err) override;
  bool set_dc_motor(int index, int speed, std::string* err) override;

 private:
  std::shared_ptr<I2cBus> bus_;
  ExpansionBoardRegisters regs_;
};

// ───────────── 버스 서보 ─────────────
class ServoBus {
 public:
  virtual ~ServoBus() = default;
  // MOVE_TIME_WRITE(위치 0..1000, 시간 0..30000 ms)
  virtual bool move(uint8_t id, int position, int duration_ms, std::string* err) = 0;
  virtual bool stop(uint8_t id, std::string* err) = 0;
  // LOAD_OR_UNLOAD_WRITE. 제어 노드는 load=false(토크 해제)를 쓰지 않는다.
  virtual bool set_load(uint8_t id, bool load, std::string* err) = 0;
  virtual std::optional<int> read_position(uint8_t id, std::string* err) = 0;
  virtual std::optional<int> read_voltage_mv(uint8_t id, std::string* err) = 0;
  virtual std::optional<int> read_temperature_c(uint8_t id, std::string* err) = 0;
};

// 반이중 방향 전환 타이밍(하드웨어 프로필, 마이크로초·밀리초). −1 = 비어 있음.
struct ServoBusTiming {
  int pre_write_delay_us{-1};         // 보내기 모드로 바꾼 뒤 쓰기 전 대기
  int post_write_delay_us{-1};        // 쓰기 뒤 대기
  bool drain_before_turnaround{true}; // 쓰기 뒤 tcdrain: 마지막 바이트가 다 나간 뒤 방향을 바꾸도록
  int read_turnaround_delay_us{-1};   // 받기 모드로 바꾸고 입력을 비운 뒤 대기
  int read_timeout_ms{-1};            // 응답 기다리는 시간
  int read_attempts{-1};              // 읽기 시도 횟수(1 이상)
  bool valid(std::string* why = nullptr) const;
};

using SleepUsFn = std::function<void(int)>;

// 반이중 방향: 보낼 때 TX_CON=1, RX_CON=0 / 받을 때 RX_CON=1, TX_CON=0
class HalfDuplexServoBus : public ServoBus {
 public:
  HalfDuplexServoBus(std::shared_ptr<SerialPort> port, std::shared_ptr<Gpio> gpio, int rx_con_bcm, int tx_con_bcm,
                   ServoBusTiming timing = ServoBusTiming{}, SleepUsFn sleep_us = nullptr);
  // 방향 핀을 출력으로 요청하고 RX_CON=0, TX_CON=1(보내기 모드)로 둔다.
  bool init(std::string* err);

  bool move(uint8_t id, int position, int duration_ms, std::string* err) override;
  bool stop(uint8_t id, std::string* err) override;
  bool set_load(uint8_t id, bool load, std::string* err) override;
  std::optional<int> read_position(uint8_t id, std::string* err) override;
  std::optional<int> read_voltage_mv(uint8_t id, std::string* err) override;
  std::optional<int> read_temperature_c(uint8_t id, std::string* err) override;

 private:
  bool set_tx(std::string* err);
  bool set_rx(std::string* err);
  bool write_packet_locked(const std::vector<uint8_t>& pkt, std::string* err);
  std::optional<std::vector<uint8_t>> read_locked(uint8_t id, uint8_t cmd, std::string* err);

  std::shared_ptr<SerialPort> port_;
  std::shared_ptr<Gpio> gpio_;
  int rx_con_;
  int tx_con_;
  ServoBusTiming timing_;
  SleepUsFn sleep_us_;
  std::mutex mu_;
};

}  // namespace armpi_hardware
