// ROS 2 Protocol v3 수신 노드 (docs/02_ROS2_INTERFACES.md).
//
// Mac·VR 조종기 ──UDP 5001 JSON v3──▶ 이 노드 ──▶
//   /armpi/cmd_vel        geometry_msgs/TwistStamped (reliable, depth 1, 20 Hz + 변경 즉시)
//   /armpi/arm/target     armpi_interfaces/ArmTarget (reliable, depth 1, 수락마다 + active 동안 20 Hz)
//   /armpi/estop_request  std_msgs/String JSON {"source":"udp","latch":bool}
//   /armpi_vr_bridge/state std_msgs/String JSON (transient_local, 2 Hz + 변화)
// ◀── /armpi/control_state std_msgs/String JSON (transient_local) — armpi_control 의 게이트
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "armpi_interfaces/msg/arm_target.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "armpi_vr_bridge/udp_server.hpp"

namespace armpi_vr_bridge {

class VrBridgeNode : public rclcpp::Node {
 public:
  explicit VrBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~VrBridgeNode() override;

  // 종료 처리: 서버·검색 정지 후 정지 명령(cmd_vel 0, ArmTarget active:false)을 3회 발행한다.
  void shutdown_outputs();

 private:
  void load_parameters();
  void on_control_state(const std_msgs::msg::String::SharedPtr msg);
  void on_check_timer();
  void on_publish_timer();
  void publish_outputs(bool arm_force);
  void publish_cmd_vel_locked(const BaseTarget& t);
  void publish_arm_locked(const ArmOutput& a);
  void publish_estop_request(bool latch);
  void publish_state(bool force);
  std::string state_json(const SessionSnapshot& s, const RxSummary& stats) const;
  void log_stats();
  void ros_log(LogLevel level, const std::string& message);

  // ── 매개변수 ──
  std::string bind_address_;
  int port_{5001};
  double poll_interval_s_{0.02};
  double rx_stall_timeout_s_{0.5};
  double publish_rate_hz_{20.0};
  double state_rate_hz_{2.0};
  double stats_log_period_s_{10.0};
  SessionConfig session_cfg_;
  std::vector<std::string> allowed_sources_;
  bool discovery_enabled_{true};
  int discovery_port_{5002};
  std::string discovery_name_;
  int discovery_max_rate_{20};
  std::string frame_id_;
  std::string arm_frame_id_;
  std::string cmd_vel_topic_, arm_target_topic_, estop_request_topic_, control_state_topic_;

  // ── ROS ──
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<armpi_interfaces::msg::ArmTarget>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_state_sub_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::unique_ptr<UdpServer> server_;
  std::unique_ptr<DiscoveryServer> discovery_;

  // 출력 발행은 이 mutex 아래에서 "세션 최신 상태 읽기 + 발행"을 함께 해 순서가 뒤바뀌지 않게 한다.
  std::mutex out_mutex_;
  std::optional<BaseTarget> last_base_;
  std::optional<ArmOutput> last_arm_;
  double last_arm_pub_{-1e9};

  std::mutex state_mutex_;
  std::string last_state_key_;
  double last_state_pub_{-1e9};
  double last_stats_log_{0.0};
  double last_cs_warn_{-1e9};

  std::atomic<bool> shut_down_{false};
};

}  // namespace armpi_vr_bridge
