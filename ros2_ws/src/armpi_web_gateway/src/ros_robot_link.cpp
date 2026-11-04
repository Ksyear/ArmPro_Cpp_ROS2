#include "armpi_web_gateway/ros_robot_link.hpp"

#include <chrono>
#include <cmath>

namespace armpi_web_gateway {

namespace json = armpi_common::json;

namespace {
constexpr double kConnectedWithinS = 3.0;  // /armpi/status(2 Hz)가 이 시간 안에 들어왔으면 "연결"

double round_to(double v, int digits) {
  const double p = std::pow(10.0, digits);
  return std::round(v * p) / p;
}

double wall_time() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
}  // namespace

double RosRobotLink::mono() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

RosRobotLink::RosRobotLink(rclcpp::Node& node, std::function<void(const std::string&)> log)
    : node_(node), log_(std::move(log)) {
  const auto reliable10 = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  status_sub_ = node_.create_subscription<std_msgs::msg::String>(
      "/armpi/status", reliable10, [this](const std_msgs::msg::String::SharedPtr m) {
        json::Value v;
        std::string err;
        if (!json::parse(m->data, v, err, json::lenient_topic_options()) || !v.is_object()) return;
        std::lock_guard<std::mutex> lk(mu_);
        status_text_ = m->data;
        status_time_ = mono();
      });
  battery_sub_ = node_.create_subscription<sensor_msgs::msg::BatteryState>(
      "/armpi/battery", reliable10, [this](const sensor_msgs::msg::BatteryState::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mu_);
        battery_ = Battery{static_cast<double>(m->voltage), static_cast<double>(m->percentage)};
        battery_time_ = mono();
      });
  joint_pub_ = node_.create_publisher<sensor_msgs::msg::JointState>("/armpi/arm/joint_debug", reliable10);
  for (const char* s : {"/armpi/emergency_stop", "/armpi/reset_estop", "/armpi/arm/home"}) {
    clients_[s] = node_.create_client<std_srvs::srv::Trigger>(s);
  }
}

bool RosRobotLink::connected() const {
  std::lock_guard<std::mutex> lk(mu_);
  return status_text_ && mono() - status_time_ <= kConnectedWithinS;
}

std::optional<json::Value> RosRobotLink::fresh_status() const {
  std::string text;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!status_text_ || mono() - status_time_ > kStatusFreshS) return std::nullopt;
    text = *status_text_;
  }
  json::Value v;
  std::string err;
  if (!json::parse(text, v, err, json::lenient_topic_options())) return std::nullopt;
  return v;
}

std::string RosRobotLink::snapshot_json() const {
  std::lock_guard<std::mutex> lk(mu_);
  const double t = mono();
  const bool conn = status_text_ && t - status_time_ <= kConnectedWithinS;
  std::string j = "{\"enabled\":true,\"connected\":" + json::boolean(conn);
  j += ",\"status\":" + (status_text_ ? *status_text_ : std::string("null"));  // 해석이 확인된 JSON 객체
  j += ",\"status_age_s\":" + (status_text_ ? json::number(round_to(t - status_time_, 2)) : std::string("null"));
  j += ",\"battery\":" + (battery_ ? "{\"voltage\":" + json::number(battery_->voltage) +
                                         ",\"percentage\":" + json::number(battery_->percentage) + "}"
                                   : std::string("null"));
  j += ",\"battery_age_s\":" + (battery_ ? json::number(round_to(t - battery_time_, 2)) : std::string("null"));
  j += ",\"last_result\":" + last_result_json_;
  return j + "}";
}

bool RosRobotLink::call_trigger(const std::string& service, std::string* error) {
  const auto it = clients_.find(service);
  if (it == clients_.end()) {
    if (error) *error = "모르는 서비스: " + service;
    return false;
  }
  if (!it->second->service_is_ready()) {
    if (error) *error = "로봇 서비스 준비 안 됨: " + service;
    return false;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  it->second->async_send_request(req, [this, service](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
    const auto res = f.get();
    const bool ok = res->success;
    {
      std::lock_guard<std::mutex> lk(mu_);
      last_result_json_ = "{\"service\":" + json::quote(service) + ",\"ok\":" + json::boolean(ok) +
                          ",\"message\":" + json::quote(res->message) + ",\"time\":" + json::number(wall_time()) + "}";
    }
    if (log_) log_("서비스 응답 " + service + ": " + (ok ? "성공 " : "실패 ") + res->message);
  });
  return true;
}

bool RosRobotLink::publish_joint_debug(const std::vector<std::string>& names, const std::vector<double>& positions,
                                       std::string*) {
  sensor_msgs::msg::JointState m;
  m.header.stamp = node_.now();
  m.name = names;
  m.position = positions;
  joint_pub_->publish(m);
  return true;
}

}  // namespace armpi_web_gateway
