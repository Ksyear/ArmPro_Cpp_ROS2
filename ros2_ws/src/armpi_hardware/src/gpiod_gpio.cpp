// libgpiod v1.6 C++ 바인딩(gpiod.hpp, Ubuntu 22.04 libgpiod-dev 1.6.3)으로 GPIO 사용.
// CMake 가 libgpiodcxx 를 찾으면 ARMPI_HAVE_GPIOD 를 정의한다.
#include <map>
#include <mutex>
#include <system_error>

#include "armpi_hardware/transport.hpp"

#ifdef ARMPI_HAVE_GPIOD
#include <gpiod.hpp>
#endif

namespace armpi_hardware {

#ifdef ARMPI_HAVE_GPIOD
namespace {
class GpiodGpio : public Gpio {
 public:
  GpiodGpio(const std::string& chip, std::string consumer)
      : chip_(chip, gpiod::chip::OPEN_LOOKUP), consumer_(std::move(consumer)) {}

  ~GpiodGpio() override {
    std::lock_guard<std::mutex> lk(mu_);
    for (auto& kv : lines_) {
      try {
        kv.second.release();
      } catch (...) {
      }
    }
  }

  bool request_output(int bcm, int initial, std::string* err) override {
    std::lock_guard<std::mutex> lk(mu_);
    try {
      gpiod::line line = chip_.get_line(static_cast<unsigned>(bcm));
      gpiod::line_request req;
      req.consumer = consumer_;
      req.request_type = gpiod::line_request::DIRECTION_OUTPUT;
      line.request(req, initial ? 1 : 0);
      lines_[bcm] = line;
      return true;
    } catch (const std::exception& e) {
      if (err) *err = "GPIO" + std::to_string(bcm) + " 출력 요청 실패: " + e.what();
      return false;
    }
  }

  bool request_input(int bcm, bool pull_up, std::string* err) override {
    std::lock_guard<std::mutex> lk(mu_);
    gpiod::line line;
    try {
      line = chip_.get_line(static_cast<unsigned>(bcm));
    } catch (const std::exception& e) {
      if (err) *err = "GPIO" + std::to_string(bcm) + " 찾기 실패: " + e.what();
      return false;
    }
    gpiod::line_request req;
    req.consumer = consumer_;
    req.request_type = gpiod::line_request::DIRECTION_INPUT;
    if (pull_up) {
      req.flags = gpiod::line_request::FLAG_BIAS_PULL_UP;
      try {
        line.request(req);
        lines_[bcm] = line;
        return true;
      } catch (const std::exception& e) {
        // 커널이 bias 를 지원하지 않으면 bias 없이 다시 요청(외부 풀업이 있으면 동작)
        if (err) *err = "GPIO" + std::to_string(bcm) + " 풀업 요청 실패(" + e.what() + "), bias 없이 다시 요청";
      }
    }
    try {
      req.flags = 0;
      line.request(req);
      lines_[bcm] = line;
      return true;
    } catch (const std::exception& e) {
      if (err) *err = "GPIO" + std::to_string(bcm) + " 입력 요청 실패: " + e.what();
      return false;
    }
  }

  bool write(int bcm, int value, std::string* err) override {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = lines_.find(bcm);
    if (it == lines_.end()) {
      if (err) *err = "GPIO" + std::to_string(bcm) + " 요청되지 않음";
      return false;
    }
    try {
      it->second.set_value(value ? 1 : 0);
      return true;
    } catch (const std::exception& e) {
      if (err) *err = "GPIO" + std::to_string(bcm) + " 쓰기 실패: " + e.what();
      return false;
    }
  }

  std::optional<int> read(int bcm, std::string* err) override {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = lines_.find(bcm);
    if (it == lines_.end()) {
      if (err) *err = "GPIO" + std::to_string(bcm) + " 요청되지 않음";
      return std::nullopt;
    }
    try {
      return it->second.get_value();
    } catch (const std::exception& e) {
      if (err) *err = "GPIO" + std::to_string(bcm) + " 읽기 실패: " + e.what();
      return std::nullopt;
    }
  }

 private:
  gpiod::chip chip_;
  std::string consumer_;
  std::map<int, gpiod::line> lines_;
  std::mutex mu_;
};
}  // namespace

bool gpiod_available() { return true; }

std::shared_ptr<Gpio> open_gpiod_gpio(const std::string& chip, const std::string& consumer, std::string* err) {
  try {
    return std::make_shared<GpiodGpio>(chip, consumer);
  } catch (const std::exception& e) {
    if (err) *err = "GPIO 칩 " + chip + " 열기 실패: " + e.what();
    return nullptr;
  }
}
#else
bool gpiod_available() { return false; }

std::shared_ptr<Gpio> open_gpiod_gpio(const std::string& chip, const std::string&, std::string* err) {
  if (err) *err = "libgpiod 없이 빌드됨: GPIO 칩 " + chip + " 을 쓸 수 없음 (libgpiod-dev 설치 후 다시 빌드)";
  return nullptr;
}
#endif

}  // namespace armpi_hardware
