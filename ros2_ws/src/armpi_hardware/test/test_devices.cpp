// 장치 구현 시험: 실제 구현(I2C 모터 보드·확장보드·버스 서보)을 가짜 전송에 붙여 바이트열과 순서를 확인하고,
// 가짜(mock) 장치의 동작을 확인한다.
#include <gtest/gtest.h>

#include <deque>
#include <string>
#include <vector>

#include "armpi_hardware/devices.hpp"
#include "armpi_hardware/hardware.hpp"
#include "armpi_hardware/hardware_profile.hpp"
#include "armpi_hardware/mock.hpp"
#include "armpi_hardware/servo_protocol.hpp"

using namespace armpi_hardware;
namespace sp = armpi_hardware::servo_proto;

// 장치 값은 시험용 프로필(실제 로봇 값 아님)
static const HardwareProfile kTest = test_hardware_profile();
static const int kRx = kTest.servo_rx_con_bcm;
static const int kTx = kTest.servo_tx_con_bcm;
static const ServoBusTiming& kTiming = kTest.servo_timing;
static std::string ev_gpio(int pin, int v) { return "gpio" + std::to_string(pin) + "=" + std::to_string(v); }

namespace {

// 트랜잭션 기록용 가짜 I2C
class RecordingI2c : public I2cBus {
 public:
  struct Tx {
    bool read;
    uint8_t addr;
    std::vector<uint8_t> data;
  };
  bool write(uint8_t addr, const std::vector<uint8_t>& data, std::string* err) override {
    if (fail) {
      if (err) *err = "fail";
      return false;
    }
    log.push_back({false, addr, data});
    return true;
  }
  bool read(uint8_t addr, size_t n, std::vector<uint8_t>* out, std::string* err) override {
    if (fail) {
      if (err) *err = "fail";
      return false;
    }
    std::vector<uint8_t> d(read_data.begin(), read_data.begin() + std::min(n, read_data.size()));
    d.resize(n, 0);
    log.push_back({true, addr, d});
    if (out) *out = d;
    return true;
  }
  std::vector<Tx> log;
  std::vector<uint8_t> read_data;
  bool fail{false};
};

// 이벤트 순서(GPIO·직렬)를 한 줄로 기록하는 가짜 직렬 포트 + GPIO
struct EventLog {
  std::vector<std::string> ev;
};

class FakeSerial : public SerialPort {
 public:
  explicit FakeSerial(EventLog* log) : log_(log) {}
  bool write_all(const std::vector<uint8_t>& data, std::string*) override {
    written.push_back(data);
    log_->ev.push_back("write");
    // 응답이 준비돼 있으면 "선로"에 올린다(flush_input 뒤 읽힘)
    return true;
  }
  bool drain(std::string*) override {
    log_->ev.push_back("drain");
    return true;
  }
  void flush_input() override {
    log_->ev.push_back("flush");
    rx.clear();
    if (!responses.empty()) {  // 서보는 요청을 받은 뒤 응답하므로 flush 뒤에 도착하는 것으로 흉내
      rx.insert(rx.end(), responses.front().begin(), responses.front().end());
      responses.pop_front();
    }
  }
  bool read_exact(size_t n, int, std::vector<uint8_t>* out, std::string* err) override {
    log_->ev.push_back("read" + std::to_string(n));
    if (rx.size() < n) {
      rx.clear();
      if (err) *err = "timeout";
      return false;
    }
    out->insert(out->end(), rx.begin(), rx.begin() + n);
    rx.erase(rx.begin(), rx.begin() + n);
    return true;
  }
  std::vector<std::vector<uint8_t>> written;
  std::deque<std::vector<uint8_t>> responses;
  std::vector<uint8_t> rx;

 private:
  EventLog* log_;
};

class LoggingGpio : public MockGpio {
 public:
  explicit LoggingGpio(EventLog* log) : log_(log) {}
  bool write(int bcm, int value, std::string* err) override {
    log_->ev.push_back("gpio" + std::to_string(bcm) + "=" + std::to_string(value));
    return MockGpio::write(bcm, value, err);
  }

 private:
  EventLog* log_;
};

}  // namespace

// ───────────── I2C 장치 ─────────────

TEST(MotorBoard, InitAndSpeedBytes) {
  auto bus = std::make_shared<RecordingI2c>();
  I2cMotorBoard mb(bus);
  std::string err;
  ASSERT_TRUE(mb.init(&err));
  ASSERT_TRUE(mb.set_speeds({25, -25, -25, 25}, &err));
  ASSERT_EQ(bus->log.size(), 2u);
  EXPECT_EQ(bus->log[0].addr, 0x34);
  EXPECT_EQ(bus->log[0].data, (std::vector<uint8_t>{20, 3}));
  EXPECT_EQ(bus->log[1].addr, 0x34);
  EXPECT_EQ(bus->log[1].data, (std::vector<uint8_t>{51, 25, 0xE7, 0xE7, 25}));  // −25 = 0xE7
  bus->fail = true;
  EXPECT_FALSE(mb.set_speeds({0, 0, 0, 0}, &err));
  EXPECT_FALSE(err.empty());
}

TEST(ExpansionBoard, BatteryReadSequence) {
  auto bus = std::make_shared<RecordingI2c>();
  bus->read_data = {0xBE, 0x1C};  // 0x1CBE = 7358 mV
  I2cExpansionBoard eb(bus, kTest.expansion);
  std::string err;
  auto mv = eb.read_battery_mv(&err);
  ASSERT_TRUE(mv.has_value());
  EXPECT_EQ(*mv, 7358);
  ASSERT_EQ(bus->log.size(), 2u);
  EXPECT_FALSE(bus->log[0].read);
  EXPECT_EQ(bus->log[0].addr, kTest.expansion.address);
  EXPECT_EQ(bus->log[0].data, (std::vector<uint8_t>{static_cast<uint8_t>(kTest.expansion.battery_register)}));
  EXPECT_TRUE(bus->log[1].read);
  EXPECT_EQ(bus->log[1].data.size(), 2u);
}

TEST(ExpansionBoard, DcMotorRegisterRule) {
  // 규칙: speed 를 −100..100 로 자르고 dc_motor_sign 을 곱해 레지스터 dc_motor_register + (index − 1) 에
  // 2의 보수 바이트로 쓴다. 부호 +1·−1 두 경우 모두 확인한다.
  for (int sign : {1, -1}) {
    ExpansionBoardRegisters regs = kTest.expansion;
    regs.dc_motor_sign = sign;
    for (int index = 1; index <= 4; ++index) {
      for (int speed : {-150, -100, -99, -50, -1, 0, 1, 37, 99, 100, 150}) {
        auto bus = std::make_shared<RecordingI2c>();
        I2cExpansionBoard eb(bus, regs);
        std::string err;
        ASSERT_TRUE(eb.set_dc_motor(index, speed, &err));
        ASSERT_EQ(bus->log.size(), 1u);
        const int clamped = speed < -100 ? -100 : (speed > 100 ? 100 : speed);
        const int value = clamped * sign;
        const uint8_t want_reg = static_cast<uint8_t>(regs.dc_motor_register + (index - 1));
        const uint8_t want_val = static_cast<uint8_t>(value & 0xFF);
        EXPECT_EQ(bus->log[0].addr, regs.address);
        EXPECT_EQ(bus->log[0].data, (std::vector<uint8_t>{want_reg, want_val})) << "index " << index << " speed " << speed;
      }
    }
  }
  EXPECT_FALSE(dc_motor_register_write(kTest.expansion, 0, 10).has_value());
  EXPECT_FALSE(dc_motor_register_write(kTest.expansion, 5, 10).has_value());
  EXPECT_FALSE(dc_motor_register_write(ExpansionBoardRegisters{}, 1, 10).has_value());  // 프로필 비어 있음
}

TEST(ExpansionBoard, EmptyProfileRefusesIo) {
  auto bus = std::make_shared<RecordingI2c>();
  I2cExpansionBoard eb(bus, ExpansionBoardRegisters{});
  std::string err;
  EXPECT_FALSE(eb.read_battery_mv(&err).has_value());
  EXPECT_FALSE(err.empty());
  EXPECT_FALSE(eb.set_dc_motor(1, 50, &err));
  EXPECT_TRUE(bus->log.empty());  // 버스에 아무것도 쓰지 않음
}

// ───────────── 버스 서보 (실제 구현 + 가짜 전송) ─────────────

TEST(HalfDuplexServoBus, InitSetsWriteMode) {
  EventLog log;
  auto ser = std::make_shared<FakeSerial>(&log);
  auto gpio = std::make_shared<LoggingGpio>(&log);
  HalfDuplexServoBus bus(ser, gpio, kRx, kTx, kTiming, [](int) {});
  std::string err;
  ASSERT_TRUE(bus.init(&err)) << err;
  EXPECT_EQ(gpio->output_level(kRx), 0);  // RX_CON
  EXPECT_EQ(gpio->output_level(kTx), 1);  // TX_CON
}

TEST(HalfDuplexServoBus, InitRefusesEmptyProfile) {
  EventLog log;
  auto ser = std::make_shared<FakeSerial>(&log);
  auto gpio = std::make_shared<LoggingGpio>(&log);
  std::string err;
  HalfDuplexServoBus no_timing(ser, gpio, kRx, kTx, ServoBusTiming{}, [](int) {});
  EXPECT_FALSE(no_timing.init(&err));
  EXPECT_FALSE(err.empty());
  HalfDuplexServoBus no_pins(ser, gpio, -1, -1, kTiming, [](int) {});
  EXPECT_FALSE(no_pins.init(&err));
  EXPECT_TRUE(log.ev.empty());  // 핀을 건드리지 않음
}

TEST(HalfDuplexServoBus, MoveWritesPacketInTxMode) {
  EventLog log;
  auto ser = std::make_shared<FakeSerial>(&log);
  auto gpio = std::make_shared<LoggingGpio>(&log);
  std::vector<int> sleeps;
  HalfDuplexServoBus bus(ser, gpio, kRx, kTx, kTiming, [&](int us) { sleeps.push_back(us); });
  std::string err;
  ASSERT_TRUE(bus.init(&err));
  log.ev.clear();
  sleeps.clear();
  ASSERT_TRUE(bus.move(1, 500, 1000, &err));
  ASSERT_EQ(ser->written.size(), 1u);
  EXPECT_EQ(ser->written[0], sp::encode_move_time_write(1, 500, 1000));
  // 순서: TX_CON=1, RX_CON=0 → 쓰기 전 대기 → write → (drain) → 쓰기 뒤 대기
  const std::vector<std::string> want{ev_gpio(kTx, 1), ev_gpio(kRx, 0), "write", "drain"};
  EXPECT_EQ(log.ev, want);
  EXPECT_EQ(sleeps, (std::vector<int>{kTiming.pre_write_delay_us, kTiming.post_write_delay_us}));
}

TEST(HalfDuplexServoBus, ReadPositionHalfDuplexSequence) {
  EventLog log;
  auto ser = std::make_shared<FakeSerial>(&log);
  auto gpio = std::make_shared<LoggingGpio>(&log);
  std::vector<int> sleeps;
  HalfDuplexServoBus bus(ser, gpio, kRx, kTx, kTiming, [&](int us) { sleeps.push_back(us); });
  std::string err;
  ASSERT_TRUE(bus.init(&err));
  log.ev.clear();
  ser->responses.push_back(sp::encode(4, sp::kPosRead, {0x39, 0x03}));  // 825
  auto pos = bus.read_position(4, &err);
  ASSERT_TRUE(pos.has_value()) << err;
  EXPECT_EQ(*pos, 825);
  EXPECT_EQ(ser->written.back(), sp::encode_read(4, sp::kPosRead));
  // 요청(보내기 모드) → 받기 모드(RX_CON=1, TX_CON=0) → flush → 대기 → 4 B → LEN−1 B → 보내기 모드 복귀
  const std::vector<std::string> want{ev_gpio(kTx, 1), ev_gpio(kRx, 0), "write", "drain", ev_gpio(kRx, 1),
                                      ev_gpio(kTx, 0), "flush", "read4", "read4", ev_gpio(kTx, 1), ev_gpio(kRx, 0)};
  EXPECT_EQ(log.ev, want);
  EXPECT_EQ(sleeps, (std::vector<int>{kTiming.pre_write_delay_us, kTiming.post_write_delay_us,
                                      kTiming.read_turnaround_delay_us}));
}

TEST(HalfDuplexServoBus, ReadRetriesAndFails) {
  EventLog log;
  auto ser = std::make_shared<FakeSerial>(&log);
  auto gpio = std::make_shared<LoggingGpio>(&log);
  HalfDuplexServoBus bus(ser, gpio, kRx, kTx, kTiming, [](int) {});
  std::string err;
  ASSERT_TRUE(bus.init(&err));
  // 1회차: 체크섬 오류, 2회차: 정상
  auto bad = sp::encode(2, sp::kVinRead, {0xE8, 0x1C});
  bad.back() ^= 0xFF;
  ser->responses.push_back(bad);
  ser->responses.push_back(sp::encode(2, sp::kVinRead, {0xE8, 0x1C}));
  auto mv = bus.read_voltage_mv(2, &err);
  ASSERT_TRUE(mv.has_value()) << err;
  EXPECT_EQ(*mv, 7400);
  EXPECT_EQ(ser->written.size(), 2u);
  // 응답 없음 → read_attempts 회 시도 후 실패, 방향은 보내기 모드로 돌아와 있어야 함
  err.clear();
  EXPECT_FALSE(bus.read_temperature_c(3, &err).has_value());
  EXPECT_NE(err.find("서보 3"), std::string::npos);
  EXPECT_EQ(gpio->output_level(kTx), 1);
  EXPECT_EQ(gpio->output_level(kRx), 0);
  // 다른 서보의 응답(ID 불일치)은 거부
  ser->responses.push_back(sp::encode(5, sp::kPosRead, {0xF4, 0x01}));
  ser->responses.push_back(sp::encode(5, sp::kPosRead, {0xF4, 0x01}));
  EXPECT_FALSE(bus.read_position(4, &err).has_value());
}

// ───────────── 가짜 장치 ─────────────

TEST(MockServoBus, LinearMotionOverDuration) {
  double t = 100.0;
  MockServoBus bus([&t] { return t; });
  std::string err;
  EXPECT_FALSE(bus.torque(3));
  ASSERT_TRUE(bus.move(3, 1000, 1000, &err));  // 500 → 1000, 1 s
  EXPECT_TRUE(bus.torque(3));
  EXPECT_NEAR(bus.position(3), 500.0, 1e-9);
  t = 100.5;
  EXPECT_NEAR(bus.position(3), 750.0, 1e-9);
  EXPECT_EQ(bus.read_position(3, &err).value(), 750);
  // 이동 중 새 명령: 현재 위치에서 다시 시작
  ASSERT_TRUE(bus.move(3, 250, 500, &err));
  t = 100.75;
  EXPECT_NEAR(bus.position(3), 500.0, 1e-9);
  t = 102.0;
  EXPECT_NEAR(bus.position(3), 250.0, 1e-9);
  EXPECT_EQ(bus.goal(3), 250);
  // MOVE_STOP: 그 자리에서 멈춤
  ASSERT_TRUE(bus.move(3, 750, 1000, &err));
  t = 102.2;
  ASSERT_TRUE(bus.stop(3, &err));
  t = 105.0;
  EXPECT_NEAR(bus.position(3), 350.0, 1e-9);
  EXPECT_EQ(bus.writes(), 4u);
  EXPECT_EQ(bus.unload_commands(), 0u);
  EXPECT_EQ(bus.read_voltage_mv(1, &err).value(), 7400);
  bus.set_fail(true);
  EXPECT_FALSE(bus.move(3, 500, 20, &err));
  EXPECT_FALSE(bus.move(7, 500, 20, &err));
}

TEST(MockHardware, DefaultsAndFailureInjection) {
  auto hw = make_mock_hardware();
  ASSERT_TRUE(hw.mock);
  std::string err;
  EXPECT_TRUE(hw.mock_motors->initialized());
  EXPECT_EQ(hw.expansion->read_battery_mv(&err).value(), 7400);
  ASSERT_TRUE(hw.expansion->set_dc_motor(1, -100, &err));
  EXPECT_EQ(hw.mock_expansion->dc_motor_register_value(1), -100 * kTest.expansion.dc_motor_sign);
  ASSERT_TRUE(hw.motors->set_speeds({1, 2, 3, 4}, &err));
  EXPECT_EQ(hw.mock_motors->speeds()[3], 4);
  hw.mock_motors->set_fail(true);
  EXPECT_FALSE(hw.motors->set_speeds({0, 0, 0, 0}, &err));
  // 가짜 GPIO: 버튼(풀업) 기본 1, 누르면 0
  ASSERT_TRUE(hw.gpio->request_input(kTest.key1_bcm, true, &err));
  EXPECT_EQ(hw.gpio->read(kTest.key1_bcm, &err).value(), 1);
  hw.mock_gpio->set_input_level(kTest.key1_bcm, 0);
  EXPECT_EQ(hw.gpio->read(kTest.key1_bcm, &err).value(), 0);
  ASSERT_TRUE(hw.gpio->request_output(kTest.buzzer_bcm, 0, &err));
  ASSERT_TRUE(hw.gpio->write(kTest.buzzer_bcm, 1, &err));
  EXPECT_EQ(hw.mock_gpio->output_level(kTest.buzzer_bcm), 1);
  EXPECT_FALSE(hw.gpio->write(5, 1, &err));  // 요청하지 않은 핀
}

TEST(RealHardware, OpenFailsGracefullyWithoutDevices) {
  // 개발 컨테이너에는 /dev/i2c-1, /dev/ttyS0 이 없거나 쓸 수 없다. 예외 없이 이유만 모아야 한다.
  HardwareConfig cfg;
  cfg.i2c_device = "/nonexistent/i2c-99";
  cfg.servo_port = "/nonexistent/ttyS99";
  cfg.gpio_chip = "/nonexistent/gpiochip99";
  std::string errors;
  auto hw = open_real_hardware(cfg, &errors);
  EXPECT_FALSE(hw.mock);
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(hw.motors, nullptr);
  EXPECT_EQ(hw.servos, nullptr);
}
