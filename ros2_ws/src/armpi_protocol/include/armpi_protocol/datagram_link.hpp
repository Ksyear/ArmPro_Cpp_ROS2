// 클라이언트의 데이터그램 송수신 경로. 실제 UDP(UdpLink)와 시험용 가짜를 바꿔 끼운다(Strategy).
#pragma once

#include <memory>
#include <optional>
#include <string>

namespace armpi_protocol {

class DatagramLink {
 public:
  virtual ~DatagramLink() = default;
  // 데이터그램 1개 송신. 실패하면 false 와 error.
  virtual bool send(const std::string& data, std::string* error) = 0;
  // 받을 것이 없으면 nullopt(비차단). 수신 오류(ICMP port unreachable 등)면 error 를 채우고 nullopt.
  virtual std::optional<std::string> receive(std::string* error) = 0;
  // "ip:port" (상태 표시용)
  virtual std::string peer() const = 0;
};

// connect 된 비차단 UDP 소켓: 운영체제가 로봇 주소·포트에서 온 데이터그램만 넘겨준다(사양 1절).
class UdpLink : public DatagramLink {
 public:
  // host: IPv4 주소 또는 이름. 실패하면 nullptr 와 error.
  static std::unique_ptr<UdpLink> open(const std::string& host, int port, std::string* error);
  ~UdpLink() override;
  UdpLink(const UdpLink&) = delete;
  UdpLink& operator=(const UdpLink&) = delete;

  bool send(const std::string& data, std::string* error) override;
  std::optional<std::string> receive(std::string* error) override;
  std::string peer() const override { return peer_; }

 private:
  UdpLink(int fd, std::string peer) : fd_(fd), peer_(std::move(peer)) {}
  int fd_;
  std::string peer_;
};

}  // namespace armpi_protocol
