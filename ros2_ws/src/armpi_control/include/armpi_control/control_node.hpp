// armpi_control_node: 비상정지 latch, 2차 watchdog, 차체 cmd_vel → 바퀴, 팔 목표 보간 → 서보, 상태 발행.
// 인터페이스: docs/02_ROS2_INTERFACES.md. 실행기: 단일 스레드(모든 콜백이 순서대로 실행되어 경쟁 없음).
#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "armpi_control/arm_controller.hpp"
#include "armpi_control/boot_check.hpp"
#include "armpi_control/buzzer_pattern.hpp"
#include "armpi_control/chassis_controller.hpp"
#include "armpi_control/safety.hpp"
#include "armpi_hardware/hardware.hpp"
#include "armpi_interfaces/msg/arm_target.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace armpi_control {

class ControlNode : public rclcpp::Node {
 public:
  explicit ControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ControlNode() override;
  // 종료 시: 바퀴 0, 부저 끔. 서보 토크는 유지(해제 금지).
  void shutdown_outputs();

 private:
  // ── 설정 ──
  void load_parameters();
  void load_hardware_profile();

  // ── 콜백 ──
  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_arm_target(const armpi_interfaces::msg::ArmTarget::SharedPtr msg);
  void on_estop_request(const std_msgs::msg::String::SharedPtr msg);
  void on_joint_debug(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_bridge_state(const std_msgs::msg::String::SharedPtr msg);
  void on_mock_cmd(const std_msgs::msg::String::SharedPtr msg);

  void srv_emergency_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void srv_reset_estop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void srv_home(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // ── 타이머 ──
  void chassis_tick();
  void arm_tick();
  void misc_tick();
  void status_tick();
  void joint_states_tick();
  void battery_tick();
  void hw_state_tick();

  // ── 동작 ──
  void latch_estop(const std::string& source, const std::string& why);
  void after_estop_cleared();
  void set_fault(const std::string& reason);
  void write_wheels(const std::array<int8_t, 4>& pulses, double now, bool force);
  void beep();                       // 비상정지 알림(우선순위 높음)
  void startup_beep();               // 시작 알림: 정상 1번, 경고 2번, fault 3번
  void update_buzzer(double now);    // 패턴 재생기 → GPIO(바뀔 때만 씀)
  void run_boot_check();
  void drain_arm_events();
  bool inhibit() const { return estop_.active() || fault(); }
  bool fault() const { return fault_ || arm_->fault(); }
  std::string fault_reason() const { return fault_ ? fault_reason_ : arm_->fault_reason(); }
  bool udp_allowed(double now) const;
  bool bridge_fresh(double now) const;
  bool owner_present(double now) const;
  void publish_control_state(bool force);
  void publish_status();
  std::string control_state_json(double now) const;
  std::string status_json(double now) const;
  static double now_s();

  // ── 설정값 ──
  bool mock_{false};
  armpi_hardware::HardwareConfig hw_cfg_;
  std::string profile_missing_;  // 실제 하드웨어 모드에서 비어 있는 프로필 항목
  int fault_threshold_{5};
  bool fan_enabled_{true};
  int fan_speed_{0};
  int fan_motor_index_{-1};
  bool buzzer_enabled_{true};
  int buzzer_bcm_{-1};
  double estop_beep_s_{0.1};
  bool startup_beep_enabled_{true};
  double startup_beep_on_s_{0.1};
  double startup_beep_off_s_{0.15};
  bool buttons_enabled_{true};
  int key1_bcm_{-1};
  int key2_bcm_{-1};
  double button_debounce_s_{0.05};
  double key2_hold_s_{2.0};
  ChassisConfig chassis_cfg_;
  double chassis_rate_hz_{50.0};
  double wheel_refresh_s_{0.5};
  ArmConfig arm_cfg_;
  double arm_rate_hz_{50.0};
  bool home_on_start_{true};
  double status_rate_hz_{2.0};
  double joint_states_rate_hz_{20.0};
  double battery_rate_hz_{1.0};
  double hw_state_rate_hz_{10.0};
  double battery_empty_v_{6.6};
  double battery_full_v_{8.4};
  double battery_low_v_{6.8};
  std::string bridge_state_topic_{"/armpi_vr_bridge/state"};
  double bridge_state_timeout_s_{2.0};
  bool boot_check_enabled_{true};
  std::string boot_manifest_{"/opt/armpi/manifests/stage20.env"};

  // ── 장치·제어기 ──
  armpi_hardware::Hardware hw_;
  std::unique_ptr<ChassisController> chassis_;
  std::unique_ptr<ArmController> arm_;
  EstopLatch estop_;
  std::unique_ptr<ButtonDetector> key1_;
  std::unique_ptr<ButtonDetector> key2_;

  // ── 상태 ──
  bool fault_{false};
  std::string fault_reason_;
  std::string init_warnings_;
  bool buzzer_ok_{false};
  bool buttons_ok_{false};
  bool fan_ok_{false};
  bool buzzer_on_{false};
  BuzzerSequencer buzzer_seq_;
  std::array<int8_t, 4> last_pulses_{0, 0, 0, 0};
  bool last_wheel_write_ok_{false};
  double last_wheel_write_{-1e18};
  int wheel_consecutive_failures_{0};
  uint64_t wheel_writes_{0};
  uint64_t wheel_failures_{0};
  bool wheel_scaled_{false};
  double last_chassis_tick_{-1.0};
  double last_arm_tick_{-1.0};
  bool chassis_wd_expired_{true};
  uint64_t cmd_dropped_stale_{0};
  uint64_t cmd_dropped_invalid_{0};
  uint64_t cmd_ignored_inhibit_{0};
  uint64_t cmd_received_{0};
  uint64_t arm_targets_received_{0};
  std::string last_arm_target_result_{"none"};
  std::optional<int> battery_mv_;
  uint64_t battery_failures_{0};
  // armpi_vr_bridge 상태
  double bridge_rx_time_{-1e18};
  std::optional<std::string> owner_;
  std::optional<std::string> owner_addr_;
  std::string bridge_base_state_{"unknown"};
  std::string bridge_arm_state_{"unknown"};
  std::string last_cs_key_;
  double last_cs_pub_{-1e18};
  uint64_t shutdown_requests_{0};
  // 부팅 점검(시작할 때 한 번): 이미지 커널·부팅 설정과 다르면 경고
  std::string kernel_release_;
  std::vector<std::string> boot_warnings_;

  // ── ROS ──
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<armpi_interfaces::msg::ArmTarget>::SharedPtr arm_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr estop_req_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_debug_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bridge_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mock_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr shutdown_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hw_state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;
  rclcpp::TimerBase::SharedPtr chassis_timer_, arm_timer_, misc_timer_, status_timer_, joint_timer_, battery_timer_,
      hw_state_timer_;
  bool outputs_shut_down_{false};
};

}  // namespace armpi_control
