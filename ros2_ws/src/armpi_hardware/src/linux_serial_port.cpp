#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>

#include "armpi_hardware/transport.hpp"

#if defined(__linux__)
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace armpi_hardware {

#if defined(__linux__)
namespace {
speed_t baud_constant(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 1000000: return B1000000;
    default: return 0;
  }
}

class LinuxSerialPort : public SerialPort {
 public:
  LinuxSerialPort(int fd, std::string path) : fd_(fd), path_(std::move(path)) {}
  ~LinuxSerialPort() override {
    if (fd_ >= 0) {
      ::tcflush(fd_, TCIOFLUSH);
      ::close(fd_);
    }
  }

  bool write_all(const std::vector<uint8_t>& data, std::string* err) override {
    size_t off = 0;
    while (off < data.size()) {
      const ssize_t n = ::write(fd_, data.data() + off, data.size() - off);
      if (n < 0) {
        if (errno == EINTR) continue;
        if (errno == EAGAIN) {  // 비차단이 아니므로 드묾. 잠깐 기다렸다가 다시
          pollfd p{fd_, POLLOUT, 0};
          ::poll(&p, 1, 10);
          continue;
        }
        if (err) *err = path_ + " 쓰기 실패: " + std::strerror(errno);
        return false;
      }
      off += static_cast<size_t>(n);
    }
    return true;
  }

  bool drain(std::string* err) override {
    while (::tcdrain(fd_) != 0) {
      if (errno == EINTR) continue;
      if (err) *err = path_ + " tcdrain 실패: " + std::strerror(errno);
      return false;
    }
    return true;
  }

  void flush_input() override { ::tcflush(fd_, TCIFLUSH); }

  bool read_exact(size_t n, int timeout_ms, std::vector<uint8_t>* out, std::string* err) override {
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
    size_t got = 0;
    uint8_t buf[64];
    while (got < n) {
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - clock::now()).count();
      if (left < 0) break;
      pollfd p{fd_, POLLIN, 0};
      const int r = ::poll(&p, 1, static_cast<int>(left));
      if (r < 0) {
        if (errno == EINTR) continue;
        if (err) *err = path_ + " poll 실패: " + std::strerror(errno);
        return false;
      }
      if (r == 0) break;
      const size_t want = std::min(n - got, sizeof(buf));
      const ssize_t k = ::read(fd_, buf, want);
      if (k < 0) {
        if (errno == EINTR || errno == EAGAIN) continue;
        if (err) *err = path_ + " 읽기 실패: " + std::strerror(errno);
        return false;
      }
      if (k == 0) continue;
      if (out) out->insert(out->end(), buf, buf + k);
      got += static_cast<size_t>(k);
    }
    if (got < n) {
      if (err) *err = path_ + " 응답 시간 초과(" + std::to_string(got) + "/" + std::to_string(n) + " B)";
      return false;
    }
    return true;
  }

 private:
  int fd_;
  std::string path_;
};
}  // namespace

std::shared_ptr<SerialPort> open_linux_serial_port(const std::string& path, int baud, std::string* err) {
  const speed_t sp = baud_constant(baud);
  if (sp == 0) {
    if (err) *err = "지원하지 않는 baud: " + std::to_string(baud);
    return nullptr;
  }
  const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd < 0) {
    if (err) *err = path + " 열기 실패: " + std::strerror(errno);
    return nullptr;
  }
  termios tio{};
  if (::tcgetattr(fd, &tio) != 0) {
    if (err) *err = path + " tcgetattr 실패: " + std::strerror(errno);
    ::close(fd);
    return nullptr;
  }
  ::cfmakeraw(&tio);
  tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | PARENB | CRTSCTS | CSIZE);
  tio.c_cflag |= CS8 | CLOCAL | CREAD;
  tio.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  ::cfsetispeed(&tio, sp);
  ::cfsetospeed(&tio, sp);
  if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
    if (err) *err = path + " tcsetattr 실패: " + std::strerror(errno);
    ::close(fd);
    return nullptr;
  }
  ::tcflush(fd, TCIOFLUSH);
  return std::make_shared<LinuxSerialPort>(fd, path);
}
#else
std::shared_ptr<SerialPort> open_linux_serial_port(const std::string& path, int, std::string* err) {
  if (err) *err = path + ": Linux 가 아니어서 직렬 포트를 쓸 수 없음";
  return nullptr;
}
#endif

}  // namespace armpi_hardware
