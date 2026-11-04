// 가짜 하드웨어(Mac/Docker 에서 실제 장치 없이 실행·시험).
//   - 가짜 서보: 명령 위치로 duration 동안 선형 이동, 토크 상태 기억
//   - 가짜 배터리: 7400 mV
//   - 가짜 GPIO: 출력 기록, 입력 수준을 시험에서 설정
// 모든 가짜 장치는 set_fail(true) 로 출력 실패를 흉내 낼 수 있다(fault 경로 시험용).
#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <map>
#include <mutex>
#include <utility>
#include <vector>

#include "armpi_hardware/devices.hpp"
#include "armpi_hardware/transport.hpp"

namespace armpi_hardware {

using ClockFn = std::function<double()>;  // 단조 시계(초)
double steady_seconds();

class MockMotorBoard : public MotorBoard {
 public:
  bool init(std::string* err) override;
  bool set_speeds(const std::array<int8_t, 4>& pulses, std::string* err) override;

  void set_fail(bool f) { fail_ = f; }
  std::array<int8_t, 4> speeds() const;
  bool initialized() const;
  uint64_t writes() const;

 private:
  mutable std::mutex mu_;
  std::array<int8_t, 4> speeds_{0, 0, 0, 0};
  bool initialized_{false};
  uint64_t writes_{0};
  std::atomic<bool> fail_{false};
};

class MockExpansionBoard : public ExpansionBoard {
 public:
  // regs: 레지스터 값 계산에 쓰는 확장보드 값(가짜에서는 시험용 프로필 값)
  explicit MockExpansionBoard(int battery_mv = 7400, ExpansionBoardRegisters regs = ExpansionBoardRegisters{0x60, 2, 40, 1})
      : battery_mv_(battery_mv), regs_(regs) {}
  std::optional<int> read_battery_mv(std::string* err) override;
  bool set_dc_motor(int index, int speed, std::string* err) override;

  void set_fail(bool f) { fail_ = f; }
  void set_battery_mv(int mv) { battery_mv_ = mv; }
  // 보드 레지스터에 쓰인 값(dc_motor_sign 을 곱한 뒤, signed). index 1..4
  int dc_motor_register_value(int index) const;

 private:
  mutable std::mutex mu_;
  std::atomic<int> battery_mv_;
  ExpansionBoardRegisters regs_;
  std::array<int, 4> dc_reg_{0, 0, 0, 0};
  std::atomic<bool> fail_{false};
};

class MockServoBus : public ServoBus {
 public:
  explicit MockServoBus(ClockFn clock = steady_seconds, int initial_position = 500);

  bool move(uint8_t id, int position, int duration_ms, std::string* err) override;
  bool stop(uint8_t id, std::string* err) override;
  bool set_load(uint8_t id, bool load, std::string* err) override;
  std::optional<int> read_position(uint8_t id, std::string* err) override;
  std::optional<int> read_voltage_mv(uint8_t id, std::string* err) override;
  std::optional<int> read_temperature_c(uint8_t id, std::string* err) override;

  void set_fail(bool f) { fail_ = f; }
  // 시험·상태 보고용
  double position(uint8_t id) const;        // 현재(선형 이동 중간값)
  int goal(uint8_t id) const;               // 마지막 명령 위치
  bool torque(uint8_t id) const;            // 토크 상태(처음에는 꺼짐, move 하면 켜짐)
  uint64_t writes() const;
  uint64_t unload_commands() const;         // set_load(false) 횟수(제어 노드는 0 이어야 함)

 private:
  struct Servo {
    double start{500.0};
    double goal{500.0};
    double t0{0.0};
    double duration{0.0};
    bool torque{false};
  };
  bool valid_id(uint8_t id, std::string* err) const;
  double position_locked(const Servo& s, double now) const;

  ClockFn clock_;
  mutable std::mutex mu_;
  std::array<Servo, 7> servos_{};  // 1..6 사용
  uint64_t writes_{0};
  uint64_t unloads_{0};
  std::atomic<bool> fail_{false};
};

class MockGpio : public Gpio {
 public:
  bool request_output(int bcm, int initial, std::string* err) override;
  bool request_input(int bcm, bool pull_up, std::string* err) override;
  bool write(int bcm, int value, std::string* err) override;
  std::optional<int> read(int bcm, std::string* err) override;

  // 시험: 입력 수준 설정(버튼 누름 = 0)
  void set_input_level(int bcm, int level);
  int output_level(int bcm) const;  // 요청되지 않았으면 −1
  // (bcm, value) 쓰기 기록(방향 전환 순서 확인용)
  std::vector<std::pair<int, int>> write_log() const;
  void clear_log();
  void set_fail(bool f) { fail_ = f; }

 private:
  mutable std::mutex mu_;
  std::map<int, int> outputs_;
  std::map<int, int> inputs_;
  std::vector<std::pair<int, int>> log_;
  std::atomic<bool> fail_{false};
};

}  // namespace armpi_hardware
