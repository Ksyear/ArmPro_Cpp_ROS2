// armpi_control_node: 주기 작업(차체·팔·버튼·부저·상태·관절·배터리·장치 상태) (ControlNode 의 일부. 클래스 선언은 control_node.hpp)
#include "armpi_control/control_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

#include "armpi_common/json.hpp"
#include "control_node_detail.hpp"

namespace armpi_control {

namespace json = armpi_common::json;
namespace ah = armpi_hardware;


// ───────────────────────── 타이머 ─────────────────────────

void ControlNode::chassis_tick() {
  const double now = now_s();
  const double dt = last_chassis_tick_ < 0 ? 1.0 / chassis_rate_hz_ : now - last_chassis_tick_;
  last_chassis_tick_ = now;
  const bool expired = chassis_->watchdog_expired(now);
  if (expired && !chassis_wd_expired_ && chassis_->watchdog_ever_fed() && !chassis_->target().is_zero()) {
    RCLCPP_WARN(get_logger(), "cmd_vel 이 %.2f s 동안 없음 → 차체 정지(2차 watchdog)", chassis_cfg_.cmd_timeout_s);
  }
  chassis_wd_expired_ = expired;
  const Twist2D out = chassis_->tick(now, dt, inhibit());
  bool scaled = false;
  const auto pulses = chassis_->wheel_pulses(out, &scaled);
  if (scaled && !wheel_scaled_) {
    RCLCPP_WARN(get_logger(), "바퀴 펄스가 상한 %d 를 넘어 같은 비율로 줄임", chassis_cfg_.max_wheel_pulse);
  }
  wheel_scaled_ = scaled;
  write_wheels(pulses, now, false);
}

void ControlNode::arm_tick() {
  const double now = now_s();
  const double dt = last_arm_tick_ < 0 ? 1.0 / arm_rate_hz_ : now - last_arm_tick_;
  last_arm_tick_ = now;
  const bool was_fault = arm_->fault();
  arm_->tick(now, dt, inhibit());
  arm_->poll_feedback(now);
  drain_arm_events();
  if (!was_fault && arm_->fault()) {
    // 팔 출력 실패도 하드웨어 fault: 바퀴 정지 시도
    chassis_->force_stop();
    write_wheels({0, 0, 0, 0}, now, true);
    publish_control_state(true);
  }
}

void ControlNode::misc_tick() {
  const double now = now_s();
  // 부저 패턴(시작 알림·비상정지)
  update_buzzer(now);
  // 버튼(누르면 0)
  if (buttons_ok_ && hw_.gpio) {
    std::string e;
    const auto k1 = hw_.gpio->read(key1_bcm_, &e);
    if (k1) {
      const int ev = key1_->update(*k1 == 0, now);
      if (ev & ButtonDetector::kPressed) latch_estop("button", "KEY1");
    }
    const auto k2 = hw_.gpio->read(key2_bcm_, &e);
    if (k2) {
      const int ev = key2_->update(*k2 == 0, now);
      if (ev & ButtonDetector::kHeld) {
        ++shutdown_requests_;
        RCLCPP_WARN(get_logger(), "KEY2 %.1f s 누름 → /armpi/shutdown_requested 발행(종료는 시스템 스크립트 담당)",
                    key2_hold_s_);
        shutdown_pub_->publish(std_msgs::msg::Empty());
      }
    }
  }
  publish_control_state(false);  // 바뀌었을 때만
}

void ControlNode::status_tick() {
  publish_control_state(true);
  publish_status();
}

void ControlNode::joint_states_tick() {
  if (!arm_->initialized()) return;
  sensor_msgs::msg::JointState js;
  js.header.stamp = now();
  const auto pos = arm_->reported_joint_positions();
  for (int i = 0; i < ah::kNumJoints; ++i) {
    js.name.emplace_back(ah::kJointNames[i]);
    js.position.push_back(pos[i]);
  }
  joint_pub_->publish(js);
}

void ControlNode::battery_tick() {
  if (!hw_.expansion) return;
  std::string e;
  const auto mv = hw_.expansion->read_battery_mv(&e);
  if (!mv) {
    ++battery_failures_;
    battery_mv_.reset();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "배터리 읽기 실패: %s", e.c_str());
    return;
  }
  battery_mv_ = *mv;
  sensor_msgs::msg::BatteryState b;
  b.header.stamp = now();
  b.voltage = static_cast<float>(*mv / 1000.0);
  b.temperature = std::numeric_limits<float>::quiet_NaN();
  b.current = std::numeric_limits<float>::quiet_NaN();
  b.charge = std::numeric_limits<float>::quiet_NaN();
  b.capacity = std::numeric_limits<float>::quiet_NaN();
  b.design_capacity = std::numeric_limits<float>::quiet_NaN();
  // 대략값: 빈 전압~가득 전압 선형(실측 방전 곡선 아님)
  const double span = battery_full_v_ - battery_empty_v_;
  b.percentage = span > 0 ? static_cast<float>(std::max(0.0, std::min(1.0, (*mv / 1000.0 - battery_empty_v_) / span)))
                          : std::numeric_limits<float>::quiet_NaN();
  b.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  b.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  b.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  b.present = true;
  b.location = "expansion_board";
  battery_pub_->publish(b);
}

void ControlNode::hw_state_tick() {
  // 장치 출력 상태(진단·시험용). 가짜 하드웨어면 가짜 서보의 현재 위치도 넣는다.
  const auto cmd = arm_->commanded_servo();
  std::string j = "{\"mock\":" + json::boolean(mock_);
  j += ",\"wheels\":[" + std::to_string(last_pulses_[0]) + "," + std::to_string(last_pulses_[1]) + "," +
       std::to_string(last_pulses_[2]) + "," + std::to_string(last_pulses_[3]) + "]";
  j += ",\"wheel_writes\":" + std::to_string(wheel_writes_) + ",\"wheel_failures\":" + std::to_string(wheel_failures_);
  j += ",\"servo_cmd\":{";
  for (int id = 1; id <= 6; ++id) j += (id > 1 ? "," : "") + json::quote(std::to_string(id)) + ":" + std::to_string(cmd[id]);
  j += "},\"servo_writes\":" + std::to_string(arm_->servo_writes()) +
       ",\"servo_failures\":" + std::to_string(arm_->servo_write_failures());
  j += ",\"buzzer\":" + json::boolean(buzzer_on_) + ",\"fan_ok\":" + json::boolean(fan_ok_);
  if (mock_) {
    j += ",\"mock_servo_pos\":{";
    for (int id = 1; id <= 6; ++id) {
      j += (id > 1 ? "," : "") + json::quote(std::to_string(id)) + ":" +
           json::fixed(hw_.mock_servos->position(static_cast<uint8_t>(id)), 1);
    }
    j += "},\"mock_torque\":{";
    for (int id = 1; id <= 6; ++id) {
      j += (id > 1 ? "," : "") + json::quote(std::to_string(id)) + ":" +
           json::boolean(hw_.mock_servos->torque(static_cast<uint8_t>(id)));
    }
    j += "},\"mock_unload_commands\":" + std::to_string(hw_.mock_servos->unload_commands());
    j += ",\"mock_fan_register\":" + std::to_string(hw_.mock_expansion->dc_motor_register_value(fan_motor_index_));
    j += ",\"mock_buzzer_gpio\":" + std::to_string(hw_.mock_gpio->output_level(buzzer_bcm_));
  }
  j += "}";
  std_msgs::msg::String m;
  m.data = std::move(j);
  hw_state_pub_->publish(m);
}

}  // namespace armpi_control
