#include <cerrno>
#include <cstdio>
#include <cstring>
#include <mutex>

#include "armpi_hardware/transport.hpp"

#if defined(__linux__)
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace armpi_hardware {

#if defined(__linux__)
namespace {
class LinuxI2cBus : public I2cBus {
 public:
  LinuxI2cBus(int fd, std::string device) : fd_(fd), device_(std::move(device)) {}
  ~LinuxI2cBus() override {
    if (fd_ >= 0) ::close(fd_);
  }

  bool write(uint8_t addr, const std::vector<uint8_t>& data, std::string* err) override {
    std::vector<uint8_t> buf(data);  // ioctl 이 const 가 아닌 포인터를 요구
    i2c_msg msg{};
    msg.addr = addr;
    msg.flags = 0;
    msg.len = static_cast<__u16>(buf.size());
    msg.buf = buf.data();
    return transfer(&msg, "쓰기", addr, err);
  }

  bool read(uint8_t addr, size_t n, std::vector<uint8_t>* out, std::string* err) override {
    std::vector<uint8_t> buf(n, 0);
    i2c_msg msg{};
    msg.addr = addr;
    msg.flags = I2C_M_RD;
    msg.len = static_cast<__u16>(n);
    msg.buf = buf.data();
    if (!transfer(&msg, "읽기", addr, err)) return false;
    if (out) *out = std::move(buf);
    return true;
  }

 private:
  bool transfer(i2c_msg* msg, const char* what, uint8_t addr, std::string* err) {
    std::lock_guard<std::mutex> lk(mu_);
    i2c_rdwr_ioctl_data data{};
    data.msgs = msg;
    data.nmsgs = 1;
    if (::ioctl(fd_, I2C_RDWR, &data) < 0) {
      if (err) {
        char hex[8];
        std::snprintf(hex, sizeof(hex), "0x%02X", addr);
        *err = device_ + " " + hex + " " + what + " 실패: " + std::strerror(errno);
      }
      return false;
    }
    return true;
  }

  int fd_;
  std::string device_;
  std::mutex mu_;
};
}  // namespace

std::shared_ptr<I2cBus> open_linux_i2c_bus(const std::string& device, std::string* err) {
  const int fd = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    if (err) *err = device + " 열기 실패: " + std::strerror(errno);
    return nullptr;
  }
  return std::make_shared<LinuxI2cBus>(fd, device);
}
#else
std::shared_ptr<I2cBus> open_linux_i2c_bus(const std::string& device, std::string* err) {
  if (err) *err = device + ": Linux 가 아니어서 I2C 를 쓸 수 없음";
  return nullptr;
}
#endif

}  // namespace armpi_hardware
