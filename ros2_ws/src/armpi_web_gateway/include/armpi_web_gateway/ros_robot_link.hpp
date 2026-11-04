// RobotLink 의 rclcpp 구현: /armpi/status·/armpi/battery 구독, Trigger 서비스 비동기 호출, joint_debug 발행.
// 파이썬 조종기는 rosbridge(9090) WebSocket 으로 같은 일을 했다. 로봇 안에서는 rclcpp 로 직접 한다.
#pragma once

#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>

#include "armpi_web_gateway/robot_link.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace armpi_web_gateway {

class RosRobotLink : public RobotLink {
 public:
  RosRobotLink(rclcpp::Node& node, std::function<void(const std::string&)> log);

  bool connected() const override;
  std::optional<armpi_common::json::Value> fresh_status() const override;
  std::string snapshot_json() const override;
  bool call_trigger(const std::string& service, std::string* error) override;
  bool publish_joint_debug(const std::vector<std::string>& names, const std::vector<double>& positions,
                           std::string* error) override;

 private:
  static double mono();
  rclcpp::Node& node_;
  std::function<void(const std::string&)> log_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;

  mutable std::mutex mu_;
  std::optional<std::string> status_text_;  // 해석에 성공한 마지막 /armpi/status
  double status_time_{-1.0};
  struct Battery {
    double voltage;
    double percentage;
  };
  std::optional<Battery> battery_;
  double battery_time_{-1.0};
  std::string last_result_json_{"null"};
};

}  // namespace armpi_web_gateway
