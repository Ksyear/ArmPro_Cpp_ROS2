// 로봇 ROS 상태·서비스 경로. 노드에서는 rclcpp 구현(RosRobotLink), 시험에서는 가짜를 끼운다(Adapter).
// 파이썬 조종기의 RosBridge(rosbridge WebSocket) 자리를 대신한다.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "armpi_common/json.hpp"

namespace armpi_web_gateway {

constexpr double kStatusFreshS = 2.0;  // /armpi/status 가 이보다 오래되면 '오래됨'

class RobotLink {
 public:
  virtual ~RobotLink() = default;
  // /armpi/status 가 들어오고 있는가(화면의 "연결" 표시)
  virtual bool connected() const = 0;
  // 최근 kStatusFreshS 안에 받은 /armpi/status. 없거나 오래됐으면 nullopt.
  virtual std::optional<armpi_common::json::Value> fresh_status() const = 0;
  // 화면용 상태 JSON 객체: enabled, connected, status, status_age_s, battery, battery_age_s, last_result
  virtual std::string snapshot_json() const = 0;
  // std_srvs/Trigger 서비스 비동기 호출. 결과는 last_result 로 보인다. 부를 수 없으면 false 와 error.
  virtual bool call_trigger(const std::string& service, std::string* error) = 0;
  // /armpi/arm/joint_debug(sensor_msgs/JointState) 발행
  virtual bool publish_joint_debug(const std::vector<std::string>& names, const std::vector<double>& positions,
                                   std::string* error) = 0;
};

}  // namespace armpi_web_gateway
