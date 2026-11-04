#include "armpi_hardware/mock.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace armpi_hardware {

double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ───────────── 가짜 모터 보드 ─────────────

bool MockMotorBoard::init(std::string* err) {
  if (fail_) {
    if (err) *err = "가짜 모터 보드: 실패 주입";
    return false;
  }
  std::lock_guard<std::mutex> lk(mu_);
  initialized_ = true;
  return true;
}

bool MockMotorBoard::set_speeds(const std::array<int8_t, 4>& pulses, std::string* err) {
  if (fail_) {
    if (err) *err = "가짜 모터 보드: 실패 주입";
    return false;
  }
  std::lock_guard<std::mutex> lk(mu_);
  speeds_ = pulses;
  ++writes_;
  return true;
}

std::array<int8_t, 4> MockMotorBoard::speeds() const {
  std::lock_guard<std::mutex> lk(mu_);
  return speeds_;
}

bool MockMotorBoard::initialized() const {
  std::lock_guard<std::mutex> lk(mu_);
  return initialized_;
}

uint64_t MockMotorBoard::writes() const {
  std::lock_guard<std::mutex> lk(mu_);
  return writes_;
}

// ───────────── 가짜 확장보드 ─────────────

std::optional<int> MockExpansionBoard::read_battery_mv(std::string* err) {
  if (fail_) {
    if (err) *err = "가짜 확장보드: 실패 주입";
    return std::nullopt;
  }
  return battery_mv_.load();
}

bool MockExpansionBoard::set_dc_motor(int index, int speed, std::string* err) {
  if (fail_) {
    if (err) *err = "가짜 확장보드: 실패 주입";
    return false;
  }
  const auto w = dc_motor_register_write(regs_, index, speed);
  if (!w) {
    if (err) *err = "DC 모터 번호 범위 밖: " + std::to_string(index);
    return false;
  }
  std::lock_guard<std::mutex> lk(mu_);
  dc_reg_[index - 1] = static_cast<int8_t>(w->second);
  return true;
}

int MockExpansionBoard::dc_motor_register_value(int index) const {
  if (index < 1 || index > 4) return 0;
  std::lock_guard<std::mutex> lk(mu_);
  return dc_reg_[index - 1];
}

// ───────────── 가짜 버스 서보 ─────────────

MockServoBus::MockServoBus(ClockFn clock, int initial_position) : clock_(clock ? std::move(clock) : ClockFn(steady_seconds)) {
  for (auto& s : servos_) {
    s.start = s.goal = static_cast<double>(std::max(0, std::min(1000, initial_position)));
  }
}

bool MockServoBus::valid_id(uint8_t id, std::string* err) const {
  if (fail_) {
    if (err) *err = "가짜 서보 버스: 실패 주입";
    return false;
  }
  if (id < 1 || id > 6) {
    if (err) *err = "가짜 서보 버스: ID 범위 밖 " + std::to_string(id);
    return false;
  }
  return true;
}

double MockServoBus::position_locked(const Servo& s, double now) const {
  if (s.duration <= 0.0 || now >= s.t0 + s.duration) return s.goal;
  if (now <= s.t0) return s.start;
  return s.start + (s.goal - s.start) * (now - s.t0) / s.duration;
}

bool MockServoBus::move(uint8_t id, int position, int duration_ms, std::string* err) {
  if (!valid_id(id, err)) return false;
  position = std::max(0, std::min(1000, position));
  duration_ms = std::max(0, std::min(30000, duration_ms));
  const double now = clock_();
  std::lock_guard<std::mutex> lk(mu_);
  Servo& s = servos_[id];
  s.start = position_locked(s, now);
  s.goal = position;
  s.t0 = now;
  s.duration = duration_ms / 1000.0;
  s.torque = true;
  ++writes_;
  return true;
}

bool MockServoBus::stop(uint8_t id, std::string* err) {
  if (!valid_id(id, err)) return false;
  const double now = clock_();
  std::lock_guard<std::mutex> lk(mu_);
  Servo& s = servos_[id];
  s.start = s.goal = position_locked(s, now);
  s.duration = 0.0;
  ++writes_;
  return true;
}

bool MockServoBus::set_load(uint8_t id, bool load, std::string* err) {
  if (!valid_id(id, err)) return false;
  const double now = clock_();
  std::lock_guard<std::mutex> lk(mu_);
  Servo& s = servos_[id];
  if (!load) {
    // 토크 해제: 그 자리에서 멈춤(실물은 중력으로 떨어질 수 있음)
    s.start = s.goal = position_locked(s, now);
    s.duration = 0.0;
    ++unloads_;
  }
  s.torque = load;
  ++writes_;
  return true;
}

std::optional<int> MockServoBus::read_position(uint8_t id, std::string* err) {
  if (!valid_id(id, err)) return std::nullopt;
  const double now = clock_();
  std::lock_guard<std::mutex> lk(mu_);
  return static_cast<int>(std::lround(position_locked(servos_[id], now)));
}

std::optional<int> MockServoBus::read_voltage_mv(uint8_t id, std::string* err) {
  if (!valid_id(id, err)) return std::nullopt;
  return 7400;
}

std::optional<int> MockServoBus::read_temperature_c(uint8_t id, std::string* err) {
  if (!valid_id(id, err)) return std::nullopt;
  return 30;
}

double MockServoBus::position(uint8_t id) const {
  if (id < 1 || id > 6) return 0.0;
  const double now = clock_();
  std::lock_guard<std::mutex> lk(mu_);
  return position_locked(servos_[id], now);
}

int MockServoBus::goal(uint8_t id) const {
  if (id < 1 || id > 6) return 0;
  std::lock_guard<std::mutex> lk(mu_);
  return static_cast<int>(std::lround(servos_[id].goal));
}

bool MockServoBus::torque(uint8_t id) const {
  if (id < 1 || id > 6) return false;
  std::lock_guard<std::mutex> lk(mu_);
  return servos_[id].torque;
}

uint64_t MockServoBus::writes() const {
  std::lock_guard<std::mutex> lk(mu_);
  return writes_;
}

uint64_t MockServoBus::unload_commands() const {
  std::lock_guard<std::mutex> lk(mu_);
  return unloads_;
}

// ───────────── 가짜 GPIO ─────────────

bool MockGpio::request_output(int bcm, int initial, std::string* err) {
  if (fail_) {
    if (err) *err = "가짜 GPIO: 실패 주입";
    return false;
  }
  std::lock_guard<std::mutex> lk(mu_);
  outputs_[bcm] = initial ? 1 : 0;
  log_.emplace_back(bcm, outputs_[bcm]);
  return true;
}

bool MockGpio::request_input(int bcm, bool pull_up, std::string* err) {
  if (fail_) {
    if (err) *err = "가짜 GPIO: 실패 주입";
    return false;
  }
  std::lock_guard<std::mutex> lk(mu_);
  if (inputs_.find(bcm) == inputs_.end()) inputs_[bcm] = pull_up ? 1 : 0;
  return true;
}

bool MockGpio::write(int bcm, int value, std::string* err) {
  if (fail_) {
    if (err) *err = "가짜 GPIO: 실패 주입";
    return false;
  }
  std::lock_guard<std::mutex> lk(mu_);
  auto it = outputs_.find(bcm);
  if (it == outputs_.end()) {
    if (err) *err = "GPIO" + std::to_string(bcm) + " 요청되지 않음";
    return false;
  }
  it->second = value ? 1 : 0;
  log_.emplace_back(bcm, it->second);
  return true;
}

std::optional<int> MockGpio::read(int bcm, std::string* err) {
  if (fail_) {
    if (err) *err = "가짜 GPIO: 실패 주입";
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lk(mu_);
  auto it = inputs_.find(bcm);
  if (it == inputs_.end()) {
    if (err) *err = "GPIO" + std::to_string(bcm) + " 요청되지 않음";
    return std::nullopt;
  }
  return it->second;
}

void MockGpio::set_input_level(int bcm, int level) {
  std::lock_guard<std::mutex> lk(mu_);
  inputs_[bcm] = level ? 1 : 0;
}

int MockGpio::output_level(int bcm) const {
  std::lock_guard<std::mutex> lk(mu_);
  auto it = outputs_.find(bcm);
  return it == outputs_.end() ? -1 : it->second;
}

std::vector<std::pair<int, int>> MockGpio::write_log() const {
  std::lock_guard<std::mutex> lk(mu_);
  return log_;
}

void MockGpio::clear_log() {
  std::lock_guard<std::mutex> lk(mu_);
  log_.clear();
}

}  // namespace armpi_hardware
