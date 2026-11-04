// armpi_control_node: 구독 콜백(cmd_vel·팔 목표·비상정지 요청·관절 디버그·수신기 상태·mock)과 서비스 (ControlNode 의 일부. 클래스 선언은 control_node.hpp)
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
using detail::join;

// ───────────────────────── 구독 ─────────────────────────

void ControlNode::on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  ++cmd_received_;
  const auto& tw = msg->twist;
  if (!std::isfinite(tw.linear.x) || !std::isfinite(tw.linear.y) || !std::isfinite(tw.angular.z)) {
    ++cmd_dropped_invalid_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cmd_vel 유한값 아님 → 버림");
    return;
  }
  double age = std::numeric_limits<double>::infinity();
  try {
    age = (now() - rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type())).seconds();
  } catch (const std::exception&) {
  }
  if (!cmd_stamp_fresh(age, chassis_cfg_.max_cmd_age_s)) {
    ++cmd_dropped_stale_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cmd_vel stamp 나이 %.3f s (한도 %.2f s) → 버림", age,
                         chassis_cfg_.max_cmd_age_s);
    return;
  }
  if (inhibit()) {
    ++cmd_ignored_inhibit_;
    return;
  }
  const double now = now_s();
  if (chassis_->on_command(Twist2D{tw.linear.x, tw.linear.y, tw.angular.z}, now)) {
    write_wheels({0, 0, 0, 0}, now, true);  // 정지는 다음 틱을 기다리지 않고 즉시
  }
}

void ControlNode::on_arm_target(const armpi_interfaces::msg::ArmTarget::SharedPtr msg) {
  ++arm_targets_received_;
  const double now = now_s();
  const auto r = arm_->on_target(msg->active, ArmPose{msg->x, msg->y, msg->z, msg->pitch, msg->gripper}, now, inhibit());
  last_arm_target_result_ = to_string(r);
  switch (r) {
    case TargetResult::kAccepted:
    case TargetResult::kHold:
    case TargetResult::kIgnoredInhibit:
      break;
    default:
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "ArmTarget 무시(%s): x=%.3f y=%.3f z=%.3f pitch=%.3f g=%.2f", to_string(r), msg->x, msg->y,
                           msg->z, msg->pitch, msg->gripper);
  }
  drain_arm_events();
}

void ControlNode::on_estop_request(const std_msgs::msg::String::SharedPtr msg) {
  json::Value v;
  std::string err;
  if (!json::parse(msg->data, v, err, json::lenient_topic_options(4096, 4)) || !v.is_object()) {
    RCLCPP_WARN(get_logger(), "estop_request JSON 오류(%s): %s", err.c_str(), msg->data.c_str());
    return;
  }
  const auto* src = v.find("source");
  const auto* latch = v.find("latch");
  if (!src || !src->is_string() || !valid_estop_source(src->string) || !latch || !latch->is_bool()) {
    RCLCPP_WARN(get_logger(), "estop_request 형식 오류(source 문자열 [a-z0-9_], latch 불리언 필요): %s",
                msg->data.c_str());
    return;
  }
  if (latch->boolean) {
    latch_estop(src->string, "/armpi/estop_request");
    return;
  }
  if (!estop_source_releasable_by_request(src->string)) {
    RCLCPP_WARN(get_logger(), "출처 '%s' 비상정지는 /armpi/reset_estop 으로만 해제", src->string.c_str());
    return;
  }
  const bool was = estop_.release(src->string);
  if (was) {
    const std::string rest = estop_.active() ? " (남은 출처: " + join(estop_.sources()) + ")" : " → 모두 해제";
    RCLCPP_WARN(get_logger(), "비상정지 출처 '%s' 해제%s", src->string.c_str(), rest.c_str());
    if (!estop_.active()) after_estop_cleared();
  }
  publish_control_state(true);
}

void ControlNode::on_joint_debug(const sensor_msgs::msg::JointState::SharedPtr msg) {
  const double now = now_s();
  std::string why;
  if (owner_present(now)) {
    why = "UDP 조종 세션 있음(owner " + *owner_ + ")";
  } else if (estop_.active()) {
    why = "비상정지 중";
  } else if (fault()) {
    why = "fault: " + fault_reason();
  } else if (arm_->busy(now) && arm_->timed_kind() == "homing") {
    why = "기본 자세로 이동 중";
  } else if (arm_->channel_active(now)) {
    why = "팔 목표(ArmTarget) 활성 중";
  }
  if (!why.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "joint_debug 무시: %s", why.c_str());
    return;
  }
  std::string m;
  if (arm_->joint_debug(msg->name, msg->position, now, &m)) {
    RCLCPP_INFO(get_logger(), "joint_debug 적용: %s", m.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "joint_debug 거부: %s", m.c_str());
  }
  drain_arm_events();
  publish_control_state(false);
}

void ControlNode::on_bridge_state(const std_msgs::msg::String::SharedPtr msg) {
  json::Value v;
  std::string err;
  if (!json::parse(msg->data, v, err, json::lenient_topic_options()) || !v.is_object()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s JSON 오류: %s", bridge_state_topic_.c_str(),
                         err.c_str());
    return;
  }
  bridge_rx_time_ = now_s();
  const auto* o = v.find("owner");
  owner_ = (o && o->is_string()) ? std::optional<std::string>(o->string) : std::nullopt;
  const auto* a = v.find("owner_addr");
  owner_addr_ = (a && a->is_string()) ? std::optional<std::string>(a->string) : std::nullopt;
  const auto* b = v.find("base_state");
  bridge_base_state_ = (b && b->is_string()) ? b->string : "unknown";
  const auto* s = v.find("arm_state");
  bridge_arm_state_ = (s && s->is_string()) ? s->string : "unknown";
}

void ControlNode::on_mock_cmd(const std_msgs::msg::String::SharedPtr msg) {
  // 가짜 하드웨어 조작(시험용): "gpio <bcm> <0|1>", "fail <i2c|servo|expansion|all> <0|1>", "battery <mV>"
  std::istringstream is(msg->data);
  std::string cmd;
  is >> cmd;
  if (cmd == "gpio") {
    int bcm = -1, level = 1;
    if (is >> bcm >> level) {
      hw_.mock_gpio->set_input_level(bcm, level);
      RCLCPP_INFO(get_logger(), "[mock] GPIO%d 입력 = %d", bcm, level);
      return;
    }
  } else if (cmd == "fail") {
    std::string what;
    int on = 0;
    if (is >> what >> on) {
      if (what == "i2c" || what == "all") hw_.mock_motors->set_fail(on != 0);
      if (what == "expansion" || what == "all") hw_.mock_expansion->set_fail(on != 0);
      if (what == "servo" || what == "all") hw_.mock_servos->set_fail(on != 0);
      RCLCPP_WARN(get_logger(), "[mock] 실패 주입 %s = %d", what.c_str(), on);
      return;
    }
  } else if (cmd == "battery") {
    int mv = 0;
    if (is >> mv) {
      hw_.mock_expansion->set_battery_mv(mv);
      return;
    }
  }
  RCLCPP_WARN(get_logger(), "[mock] 알 수 없는 명령: %s", msg->data.c_str());
}

// ───────────────────────── 서비스 ─────────────────────────

void ControlNode::srv_emergency_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  latch_estop("service", "/armpi/emergency_stop");
  res->success = true;
  res->message = "비상정지 latch(service). 출처: " + join(estop_.sources());
}

void ControlNode::srv_reset_estop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  const auto cleared = estop_.reset_all();
  if (!cleared.empty()) {
    RCLCPP_WARN(get_logger(), "비상정지 모두 해제 (%s)", join(cleared).c_str());
    after_estop_cleared();
  }
  publish_control_state(true);
  res->success = true;
  res->message = cleared.empty() ? "이미 해제 상태" : "비상정지 해제: " + join(cleared);
  if (fault()) res->message += " (fault 는 남아 있음: " + fault_reason() + ")";
}

void ControlNode::srv_home(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  const double now = now_s();
  res->success = false;
  if (estop_.active()) {
    res->message = "비상정지 중에는 home 거부";
  } else if (fault()) {
    res->message = "fault 상태: " + fault_reason();
  } else if (arm_->channel_active(now) || (bridge_fresh(now) && bridge_arm_state_ == "active")) {
    res->message = "팔 조종 중(팔 채널 active)에는 home 거부";
  } else {
    std::string m;
    res->success = arm_->home(now, &m);
    res->message = m;
  }
  RCLCPP_INFO(get_logger(), "/armpi/arm/home → %s: %s", res->success ? "성공" : "거부", res->message.c_str());
  drain_arm_events();
  publish_control_state(true);
}

}  // namespace armpi_control
