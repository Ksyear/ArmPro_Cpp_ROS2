// armpi_control_node: /armpi/control_state·/armpi/status JSON 작성과 발행 (ControlNode 의 일부. 클래스 선언은 control_node.hpp)
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
using detail::pose_json;
using detail::str_list;

// ───────────────────────── 상태 발행 ─────────────────────────

std::string ControlNode::control_state_json(double now) const {
  return "{\"estop\":" + json::boolean(estop_.active()) + ",\"estop_sources\":" + str_list(estop_.sources()) +
         ",\"fault\":" + json::boolean(fault()) + ",\"fault_reason\":" + json::quote(fault() ? fault_reason() : "") +
         ",\"udp_allowed\":" + json::boolean(udp_allowed(now)) + "}";
}

void ControlNode::publish_control_state(bool force) {
  if (!control_state_pub_) return;
  const double now = now_s();
  std::string j = control_state_json(now);
  if (!force && j == last_cs_key_) return;
  std_msgs::msg::String m;
  m.data = j;
  control_state_pub_->publish(m);
  if (j != last_cs_key_) publish_status();  // 상태가 바뀌면 /armpi/status 도 즉시
  last_cs_key_ = std::move(j);
  last_cs_pub_ = now;
}

std::string ControlNode::status_json(double now) const {
  const bool fresh = bridge_fresh(now);
  const auto& out = chassis_->output();
  std::string j = "{\"source\":\"armpi_control\"";
  j += ",\"hardware\":" + json::quote(mock_ ? "mock" : "real");
  j += ",\"owner\":" + (fresh && owner_ ? json::quote(*owner_) : std::string("null"));
  j += ",\"owner_addr\":" + (fresh && owner_addr_ ? json::quote(*owner_addr_) : std::string("null"));
  j += ",\"base_state\":" + json::quote(fresh ? bridge_base_state_ : "unknown");
  j += ",\"arm_state\":" + json::quote(fresh ? bridge_arm_state_ : "unknown");
  j += ",\"vr_bridge_state_age_s\":" + (bridge_rx_time_ > -1e17 ? json::fixed(now - bridge_rx_time_, 2) : "null");
  j += ",\"emergency_stop\":" + json::boolean(estop_.active());
  j += ",\"estop_sources\":" + str_list(estop_.sources());
  j += ",\"fault\":" + json::boolean(fault());
  j += ",\"fault_reason\":" + json::quote(fault() ? fault_reason() : "");
  j += ",\"udp_allowed\":" + json::boolean(udp_allowed(now));
  j += ",\"battery_v\":" + (battery_mv_ ? json::fixed(*battery_mv_ / 1000.0, 3) : "null");
  j += ",\"battery_low\":" + json::boolean(battery_mv_ && *battery_mv_ / 1000.0 < battery_low_v_);
  j += ",\"position_source\":" + json::quote(arm_->position_source());
  j += ",\"arm_initialized\":" + json::boolean(arm_->initialized());
  j += ",\"arm_mode\":" + json::quote(arm_->busy(now) ? arm_->timed_kind() : to_string(arm_->mode()));
  if (arm_->initialized()) {
    j += ",\"arm_target\":" + pose_json(arm_->goal());
    j += ",\"arm_current\":" + pose_json(arm_->reported_pose());
  } else {
    j += ",\"arm_target\":null,\"arm_current\":null";
  }
  j += ",\"arm_moving\":" + json::boolean(arm_->moving(now));
  j += ",\"arm_ik_blocked\":" + json::boolean(arm_->ik_blocked());
  j += ",\"arm_last_target_result\":" + json::quote(last_arm_target_result_);
  j += ",\"base_cmd\":{\"vx\":" + json::fixed(out.vx, 4) + ",\"vy\":" + json::fixed(out.vy, 4) +
       ",\"wz\":" + json::fixed(out.wz, 4) + "}";
  j += ",\"wheel_pulses\":[" + std::to_string(last_pulses_[0]) + "," + std::to_string(last_pulses_[1]) + "," +
       std::to_string(last_pulses_[2]) + "," + std::to_string(last_pulses_[3]) + "]";
  j += ",\"cmd_vel\":{\"received\":" + std::to_string(cmd_received_) +
       ",\"dropped_stale\":" + std::to_string(cmd_dropped_stale_) +
       ",\"dropped_invalid\":" + std::to_string(cmd_dropped_invalid_) +
       ",\"ignored_estop_fault\":" + std::to_string(cmd_ignored_inhibit_) +
       ",\"watchdog_expired\":" + json::boolean(chassis_->watchdog_expired(now)) + "}";
  j += ",\"buttons_ok\":" + json::boolean(buttons_ok_) + ",\"buzzer_ok\":" + json::boolean(buzzer_ok_) +
       ",\"fan_ok\":" + json::boolean(fan_ok_);
  j += ",\"warnings\":" + json::quote(init_warnings_);
  j += ",\"kernel\":" + json::quote(kernel_release_);
  j += ",\"boot_warnings\":" + str_list(boot_warnings_);
  j += "}";
  return j;
}

void ControlNode::publish_status() {
  if (!status_pub_) return;
  std_msgs::msg::String m;
  m.data = status_json(now_s());
  status_pub_->publish(m);
}

}  // namespace armpi_control
