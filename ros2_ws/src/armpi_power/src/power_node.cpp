// armpi_power_node: KEY2 종료 요청을 받아 시스템을 끈다(armpi-shutdown.service, root 로 실행).
// 예전 서비스는 `ros2 topic echo --once /armpi/shutdown_requested && systemctl poweroff`(파이썬 CLI, 약 110 MB)였다.
//
// 매개변수
//   trigger        "topic"(기본: 지금과 같은 동작 — 같은 ROS 망의 누구나 이 토픽으로 끌 수 있음, docs/11)
//                  "file" (docs/12 E2 준비: request_file 이 생기면 끈다. 제어기 쪽 변경이 아직 없어 기본으로 쓰지 않음)
//   topic          "/armpi/shutdown_requested"
//   request_file   "/run/armpi/shutdown_requested",  file_poll_s 0.5
//   command        ["systemctl", "poweroff"]
//   dry_run        false (true 면 끄지 않고 dry_run_marker 에 표시만: 시험·시연)
#include <memory>
#include <string>
#include <vector>

#include "armpi_power/power_policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

namespace ap = armpi_power;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("armpi_power");
  auto logger = node->get_logger();
  const std::string trigger = node->declare_parameter<std::string>("trigger", "topic");
  const std::string topic = node->declare_parameter<std::string>("topic", "/armpi/shutdown_requested");
  const std::string request_file = node->declare_parameter<std::string>("request_file", "/run/armpi/shutdown_requested");
  const double poll_s = node->declare_parameter<double>("file_poll_s", 0.5);
  const auto command =
      node->declare_parameter<std::vector<std::string>>("command", std::vector<std::string>{"systemctl", "poweroff"});
  const bool dry_run = node->declare_parameter<bool>("dry_run", false);
  const std::string marker = node->declare_parameter<std::string>("dry_run_marker", "/run/armpi/poweroff_dry_run");
  if (trigger != "topic" && trigger != "file") {
    RCLCPP_FATAL(logger, "trigger 는 topic 또는 file");
    return 2;
  }

  std::unique_ptr<ap::PowerAction> action;
  if (dry_run) {
    action = std::make_unique<ap::DryRunPowerAction>(marker);
  } else {
    action = std::make_unique<ap::CommandPowerAction>(command);
  }
  ap::ShutdownLatch latch(std::move(action), [logger](const std::string& m) { RCLCPP_WARN(logger, "%s", m.c_str()); });

  auto after = [&](bool ok) {
    if (ok) rclcpp::shutdown();  // 한 번 실행하면 끝(systemd Restart=on-failure 는 정상 종료 뒤 다시 켜지 않음)
  };
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub;
  rclcpp::TimerBase::SharedPtr timer;
  if (trigger == "topic") {
    sub = node->create_subscription<std_msgs::msg::Empty>(
        topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [&](const std_msgs::msg::Empty::SharedPtr) { after(latch.request("토픽 " + topic)); });
  } else {
    timer = node->create_wall_timer(std::chrono::duration<double>(poll_s), [&] {
      if (ap::file_exists(request_file)) after(latch.request("파일 " + request_file));
    });
  }
  RCLCPP_INFO(logger, "종료 대기: %s %s → %s", trigger == "topic" ? "토픽" : "파일",
              (trigger == "topic" ? topic : request_file).c_str(), latch.action().describe().c_str());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
