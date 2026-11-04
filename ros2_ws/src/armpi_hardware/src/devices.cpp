#include "armpi_hardware/devices.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "armpi_hardware/servo_protocol.hpp"

namespace armpi_hardware {

// ───────────── 바퀴 모터 보드 ─────────────

I2cMotorBoard::I2cMotorBoard(std::shared_ptr<I2cBus> bus, uint8_t address, uint8_t motor_type)
    : bus_(std::move(bus)), address_(address), motor_type_(motor_type) {}

bool I2cMotorBoard::init(std::string* err) {
  // 레지스터 20 ← 모터 종류
  return bus_->write(address_, {kMotorTypeRegister, motor_type_}, err);
}

bool I2cMotorBoard::set_speeds(const std::array<int8_t, 4>& p, std::string* err) {
  // 레지스터 51 ← 네 바퀴 속도(m1..m4)
  return bus_->write(address_,
                     {kMotorSpeedRegister, static_cast<uint8_t>(p[0]), static_cast<uint8_t>(p[1]),
                      static_cast<uint8_t>(p[2]), static_cast<uint8_t>(p[3])},
                     err);
}

// ───────────── 확장보드 ─────────────

bool ExpansionBoardRegisters::valid(std::string* why) const {
  auto byte = [](int v) { return v >= 0 && v <= 255; };
  const bool ok = address >= 0x01 && address <= 0x7F && byte(battery_register) && byte(dc_motor_register) &&
                  dc_motor_register <= 252 && (dc_motor_sign == 1 || dc_motor_sign == -1);
  if (!ok && why) {
    *why = "hardware.expansion_board_address(0x01..0x7F)·battery_register·dc_motor_register(0..252)·"
           "dc_motor_sign(±1) 가 비어 있거나 범위 밖";
  }
  return ok;
}

std::optional<std::pair<uint8_t, uint8_t>> dc_motor_register_write(const ExpansionBoardRegisters& regs, int index,
                                                                   int speed) {
  if (index < 1 || index > 4 || !regs.valid()) return std::nullopt;
  speed = std::max(-100, std::min(100, speed)) * regs.dc_motor_sign;
  return std::make_pair(static_cast<uint8_t>(regs.dc_motor_register + index - 1),
                        static_cast<uint8_t>(static_cast<int8_t>(speed)));
}

I2cExpansionBoard::I2cExpansionBoard(std::shared_ptr<I2cBus> bus, ExpansionBoardRegisters regs)
    : bus_(std::move(bus)), regs_(regs) {}

std::optional<int> I2cExpansionBoard::read_battery_mv(std::string* err) {
  std::string why;
  if (!regs_.valid(&why)) {
    if (err) *err = why;
    return std::nullopt;
  }
  const auto addr = static_cast<uint8_t>(regs_.address);
  if (!bus_->write(addr, {static_cast<uint8_t>(regs_.battery_register)}, err)) return std::nullopt;
  std::vector<uint8_t> b;
  if (!bus_->read(addr, 2, &b, err) || b.size() != 2) return std::nullopt;
  return static_cast<int>(b[0] | (b[1] << 8));
}

bool I2cExpansionBoard::set_dc_motor(int index, int speed, std::string* err) {
  const auto w = dc_motor_register_write(regs_, index, speed);
  if (!w) {
    if (err) *err = "DC 모터 번호 범위 밖이거나 확장보드 레지스터가 비어 있음: " + std::to_string(index);
    return false;
  }
  return bus_->write(static_cast<uint8_t>(regs_.address), {w->first, w->second}, err);
}

bool ServoBusTiming::valid(std::string* why) const {
  const bool ok = pre_write_delay_us >= 0 && post_write_delay_us >= 0 && read_turnaround_delay_us >= 0 &&
                  read_timeout_ms > 0 && read_attempts >= 1;
  if (!ok && why) {
    *why = "servo.pre_write_delay_us·post_write_delay_us·read_turnaround_delay_us(0 이상)·read_timeout_ms·"
           "read_attempts(1 이상) 가 비어 있음";
  }
  return ok;
}

// ───────────── 버스 서보 ─────────────

namespace {
void default_sleep_us(int us) {
  if (us > 0) std::this_thread::sleep_for(std::chrono::microseconds(us));
}
}  // namespace

HalfDuplexServoBus::HalfDuplexServoBus(std::shared_ptr<SerialPort> port, std::shared_ptr<Gpio> gpio, int rx_con_bcm,
                                   int tx_con_bcm, ServoBusTiming timing, SleepUsFn sleep_us)
    : port_(std::move(port)),
      gpio_(std::move(gpio)),
      rx_con_(rx_con_bcm),
      tx_con_(tx_con_bcm),
      timing_(timing),
      sleep_us_(sleep_us ? std::move(sleep_us) : SleepUsFn(default_sleep_us)) {}

bool HalfDuplexServoBus::init(std::string* err) {
  std::lock_guard<std::mutex> lk(mu_);
  if (!timing_.valid(err)) return false;
  if (rx_con_ < 0 || tx_con_ < 0) {
    if (err) *err = "hardware.servo_rx_con_bcm·servo_tx_con_bcm 가 비어 있음";
    return false;
  }
  // 초기화: RX_CON 출력 0, TX_CON 출력 1, 이어서 보내기 모드
  if (!gpio_->request_output(rx_con_, 0, err)) return false;
  if (!gpio_->request_output(tx_con_, 1, err)) return false;
  return set_tx(err);
}

bool HalfDuplexServoBus::set_tx(std::string* err) {
  // 보내기 모드: TX_CON=1 다음 RX_CON=0
  return gpio_->write(tx_con_, 1, err) && gpio_->write(rx_con_, 0, err);
}

bool HalfDuplexServoBus::set_rx(std::string* err) {
  // 받기 모드: RX_CON=1 다음 TX_CON=0
  return gpio_->write(rx_con_, 1, err) && gpio_->write(tx_con_, 0, err);
}

bool HalfDuplexServoBus::write_packet_locked(const std::vector<uint8_t>& pkt, std::string* err) {
  if (!set_tx(err)) return false;
  sleep_us_(timing_.pre_write_delay_us);
  if (!port_->write_all(pkt, err)) return false;
  if (timing_.drain_before_turnaround && !port_->drain(err)) return false;
  sleep_us_(timing_.post_write_delay_us);
  return true;
}

bool HalfDuplexServoBus::move(uint8_t id, int position, int duration_ms, std::string* err) {
  std::lock_guard<std::mutex> lk(mu_);
  return write_packet_locked(servo_proto::encode_move_time_write(id, position, duration_ms), err);
}

bool HalfDuplexServoBus::stop(uint8_t id, std::string* err) {
  std::lock_guard<std::mutex> lk(mu_);
  return write_packet_locked(servo_proto::encode_move_stop(id), err);
}

bool HalfDuplexServoBus::set_load(uint8_t id, bool load, std::string* err) {
  std::lock_guard<std::mutex> lk(mu_);
  return write_packet_locked(servo_proto::encode_load_or_unload(id, load), err);
}

std::optional<std::vector<uint8_t>> HalfDuplexServoBus::read_locked(uint8_t id, uint8_t cmd, std::string* err) {
  const int attempts = std::max(1, timing_.read_attempts);
  std::string last;
  for (int i = 0; i < attempts; ++i) {
    std::string e;
    if (!write_packet_locked(servo_proto::encode_read(id, cmd), &e)) {
      last = e;
      continue;
    }
    std::vector<uint8_t> data;
    bool ok = set_rx(&e);
    if (ok) {
      port_->flush_input();
      sleep_us_(timing_.read_turnaround_delay_us);
      // 4바이트 읽기 → 머리 0x55 0x55 확인 → 남은 길이(data[3] − 1)만큼 읽기
      ok = port_->read_exact(4, timing_.read_timeout_ms, &data, &e);
      if (ok && !(data[0] == servo_proto::kHeader && data[1] == servo_proto::kHeader)) {
        e = "응답 머리 오류";
        ok = false;
      }
      if (ok && data[3] < 3) {
        e = "응답 길이 오류";
        ok = false;
      }
      if (ok) ok = port_->read_exact(static_cast<size_t>(data[3]) - 1, timing_.read_timeout_ms, &data, &e);
    }
    std::string e2;
    if (!set_tx(&e2) && e.empty()) e = e2;  // 끝나면 항상 보내기 모드로 되돌림
    if (!ok) {
      last = e;
      continue;
    }
    servo_proto::Response r;
    const auto pe = servo_proto::parse_response(data, id, cmd, &r);
    if (pe != servo_proto::ParseError::kOk) {
      last = std::string("응답 해석 실패: ") + servo_proto::to_string(pe);
      continue;
    }
    return data;
  }
  if (err) *err = "서보 " + std::to_string(id) + " 읽기(명령 " + std::to_string(cmd) + ") 실패: " + last;
  return std::nullopt;
}

std::optional<int> HalfDuplexServoBus::read_position(uint8_t id, std::string* err) {
  std::lock_guard<std::mutex> lk(mu_);
  auto d = read_locked(id, servo_proto::kPosRead, err);
  if (!d) return std::nullopt;
  servo_proto::Response r;
  servo_proto::parse_packet(*d, &r);
  return servo_proto::decode_position(r);
}

std::optional<int> HalfDuplexServoBus::read_voltage_mv(uint8_t id, std::string* err) {
  std::lock_guard<std::mutex> lk(mu_);
  auto d = read_locked(id, servo_proto::kVinRead, err);
  if (!d) return std::nullopt;
  servo_proto::Response r;
  servo_proto::parse_packet(*d, &r);
  return servo_proto::decode_vin_mv(r);
}

std::optional<int> HalfDuplexServoBus::read_temperature_c(uint8_t id, std::string* err) {
  std::lock_guard<std::mutex> lk(mu_);
  auto d = read_locked(id, servo_proto::kTempRead, err);
  if (!d) return std::nullopt;
  servo_proto::Response r;
  servo_proto::parse_packet(*d, &r);
  return servo_proto::decode_temp_c(r);
}

}  // namespace armpi_hardware
