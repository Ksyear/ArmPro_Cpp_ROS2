// 저수준 전송 인터페이스: I2C 버스, 직렬 포트, GPIO.
// 실제 구현은 Linux(i2c-dev ioctl, termios, libgpiod v1.6). 시험에서는 가짜 전송으로 바꿔 끼운다.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace armpi_hardware {

// ───────────── I2C ─────────────
class I2cBus {
 public:
  virtual ~I2cBus() = default;
  // 쓰기 트랜잭션 1개(START, 주소+W, data…, STOP)
  virtual bool write(uint8_t addr, const std::vector<uint8_t>& data, std::string* err) = 0;
  // 읽기 트랜잭션 1개(START, 주소+R, n 바이트, STOP)
  virtual bool read(uint8_t addr, size_t n, std::vector<uint8_t>* out, std::string* err) = 0;
};

// /dev/i2c-N 을 열어 I2C_RDWR ioctl 로 전송한다.
// 여러 스레드에서 불러도 되도록 내부 뮤텍스를 쓴다. 실패 시 nullptr, err 에 이유.
std::shared_ptr<I2cBus> open_linux_i2c_bus(const std::string& device, std::string* err);

// ───────────── 직렬 포트 ─────────────
class SerialPort {
 public:
  virtual ~SerialPort() = default;
  virtual bool write_all(const std::vector<uint8_t>& data, std::string* err) = 0;
  // 출력 버퍼가 실제로 다 나갈 때까지 기다림(tcdrain)
  virtual bool drain(std::string* err) = 0;
  // 받은 입력 버리기(tcflush TCIFLUSH)
  virtual void flush_input() = 0;
  // n 바이트를 timeout_ms 안에 읽어 out 뒤에 붙인다. 모자라면 false.
  virtual bool read_exact(size_t n, int timeout_ms, std::vector<uint8_t>* out, std::string* err) = 0;
};

// termios raw 8N1(흐름 제어 없음). baud 는 115200 등 표준값만.
std::shared_ptr<SerialPort> open_linux_serial_port(const std::string& path, int baud, std::string* err);

// ───────────── GPIO (BCM 번호 = gpiochip0 의 line offset) ─────────────
class Gpio {
 public:
  virtual ~Gpio() = default;
  virtual bool request_output(int bcm, int initial, std::string* err) = 0;
  // pull_up 이 커널에서 지원되지 않으면 bias 없이 요청하고 err 에 경고를 남긴 뒤 true
  virtual bool request_input(int bcm, bool pull_up, std::string* err) = 0;
  virtual bool write(int bcm, int value, std::string* err) = 0;
  virtual std::optional<int> read(int bcm, std::string* err) = 0;
};

// libgpiod v1.6 C++ (gpiod.hpp). 빌드 시 libgpiod 가 없으면 항상 nullptr + 이유.
std::shared_ptr<Gpio> open_gpiod_gpio(const std::string& chip, const std::string& consumer, std::string* err);
bool gpiod_available();

}  // namespace armpi_hardware
