#include "armpi_vr_bridge/bridge_node.hpp"

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>

#include "armpi_common/json.hpp"

namespace armpi_vr_bridge {

namespace json = armpi_common::json;

namespace {

std::string opt_str(const std::optional<std::string>& v) { return v ? json::quote(*v) : std::string("null"); }
std::string opt_num(const std::optional<double>& v) { return v ? json::number(*v) : std::string("null"); }
const char* jbool(bool b) { return b ? "true" : "false"; }

rclcpp::QoS reliable_depth(std::size_t depth) { return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable(); }
rclcpp::QoS latched_state() { return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(); }

}  // namespace

VrBridgeNode::VrBridgeNode(const rclcpp::NodeOptions& options) : rclcpp::Node("armpi_vr_bridge", options) {
  load_parameters();

  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, reliable_depth(1));
  arm_pub_ = create_publisher<armpi_interfaces::msg::ArmTarget>(arm_target_topic_, reliable_depth(1));
  estop_pub_ = create_publisher<std_msgs::msg::String>(estop_request_topic_, reliable_depth(10));
  state_pub_ = create_publisher<std_msgs::msg::String>("~/state", latched_state());

  ServerConfig scfg;
  scfg.bind_address = bind_address_;
  scfg.port = static_cast<uint16_t>(port_);
  scfg.poll_interval_s = poll_interval_s_;
  scfg.stall_timeout_s = rx_stall_timeout_s_;
  SourceFilter filter(allowed_sources_);  // 잘못된 CIDR 은 여기서 예외 → 노드 시작 실패

  ServerCallbacks cb;
  cb.on_packet = [this](const HandleResult& r) {
    publish_outputs(r.arm_accepted);
    publish_state(false);
  };
  cb.on_estop = [this](bool latch) { publish_estop_request(latch); };
  cb.on_stop = [this](const std::string&) {
    publish_outputs(false);
    publish_state(false);
  };
  cb.log = [this](LogLevel level, const std::string& m) { ros_log(level, m); };
  server_ = std::make_unique<UdpServer>(session_cfg_, scfg, filter, cb);

  // 제어기 상태를 받기 전에는 활성 명령을 not_owner 로 거부한다(세션 기본값: 게이트 차단).
  control_state_sub_ = create_subscription<std_msgs::msg::String>(
      control_state_topic_, latched_state(),
      [this](const std_msgs::msg::String::SharedPtr msg) { on_control_state(msg); });

  server_->start();  // bind 실패 시 예외 → 노드 시작 실패(중복 실행이 드러난다)

  if (discovery_enabled_) {
    try {
      std::string name = discovery_name_;
      if (name.empty()) {
        char host[256] = {0};
        if (::gethostname(host, sizeof(host) - 1) == 0) name = host;
        if (name.size() > 32) name.resize(32);
        if (name.empty()) name = "armpi";
      }
      DiscoveryResponder responder(name, device_id(), server_->bound_port(), discovery_max_rate_);
      DiscoveryServerConfig dcfg;
      dcfg.bind_address = bind_address_;
      dcfg.port = static_cast<uint16_t>(discovery_port_);
      discovery_ = std::make_unique<DiscoveryServer>(std::move(responder), dcfg, filter,
                                                     [this](LogLevel l, const std::string& m) { ros_log(l, m); });
      discovery_->start();
      RCLCPP_INFO(get_logger(), "검색 응답: UDP %u (이름 %s, id %s)", discovery_->bound_port(),
                  discovery_->name().c_str(), discovery_->identity().c_str());
    } catch (const std::exception& e) {
      // 검색 실패는 제어에 영향 없음
      RCLCPP_WARN(get_logger(), "검색 응답기 시작 실패: %s", e.what());
      discovery_.reset();
    }
  }

  check_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { on_check_timer(); });
  publish_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz_),
                                     [this]() { on_publish_timer(); });
  last_stats_log_ = monotonic_seconds();

  RCLCPP_INFO(get_logger(),
              "Protocol v3 수신 %s:%u → %s, %s (|vx|≤%.2f m/s, |vy|≤%.2f m/s, |wz|≤%.2f rad/s, "
              "watchdog 차체 %.2fs/팔 %.2fs, lease %.1fs, control_state 최대 나이 %.1fs)",
              bind_address_.c_str(), server_->bound_port(), cmd_vel_topic_.c_str(), arm_target_topic_.c_str(),
              session_cfg_.max_vx_m_s, session_cfg_.max_vy_m_s, session_cfg_.max_wz_rad_s,
              session_cfg_.base_watchdog_s, session_cfg_.arm_watchdog_s, session_cfg_.lease_s,
              session_cfg_.control_state_max_age_s);
  publish_outputs(false);
  publish_state(true);
}

VrBridgeNode::~VrBridgeNode() {
  try {
    shutdown_outputs();
  } catch (...) {
  }
}

void VrBridgeNode::load_parameters() {
  bind_address_ = declare_parameter<std::string>("bind_address", "0.0.0.0");
  port_ = static_cast<int>(declare_parameter<int64_t>("port", 5001));
  poll_interval_s_ = declare_parameter<double>("poll_interval_sec", 0.02);
  rx_stall_timeout_s_ = declare_parameter<double>("rx_stall_timeout_sec", 0.5);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
  state_rate_hz_ = declare_parameter<double>("state_rate_hz", 2.0);
  stats_log_period_s_ = declare_parameter<double>("stats_log_period_sec", 10.0);

  session_cfg_.base_watchdog_s = declare_parameter<double>("base_watchdog_sec", 0.4);
  session_cfg_.arm_watchdog_s = declare_parameter<double>("arm_watchdog_sec", 0.4);
  session_cfg_.lease_s = declare_parameter<double>("lease_sec", 1.0);
  session_cfg_.control_state_max_age_s = declare_parameter<double>("control_state_max_age_sec", 2.0);
  session_cfg_.accept_estop_from_any_client = declare_parameter<bool>("accept_estop_from_any_client", true);
  const int64_t retired = declare_parameter<int64_t>("retired_limit", 1024);
  if (retired < 1 || retired > 1000000) throw std::invalid_argument("retired_limit 은 1~1000000 이어야 합니다");
  session_cfg_.retired_limit = static_cast<std::size_t>(retired);

  session_cfg_.max_vx_m_s = declare_parameter<double>("chassis.max_vx_m_s", 0.25);  // 사용자 결정 2026-11-04, 바퀴 방향 실기 시험 전
  session_cfg_.max_vy_m_s = declare_parameter<double>("chassis.max_vy_m_s", 0.25);
  session_cfg_.max_wz_rad_s = declare_parameter<double>("chassis.max_wz_rad_s", 1.2);

  auto& w = session_cfg_.workspace;
  w.x_min = declare_parameter<double>("arm.workspace.x_min", 0.0);
  w.x_max = declare_parameter<double>("arm.workspace.x_max", 0.30);
  w.y_min = declare_parameter<double>("arm.workspace.y_min", -0.25);
  w.y_max = declare_parameter<double>("arm.workspace.y_max", 0.25);
  w.z_min = declare_parameter<double>("arm.workspace.z_min", -0.05);
  w.z_max = declare_parameter<double>("arm.workspace.z_max", 0.35);

  auto& k = session_cfg_.kinematics;
  const std::string kp = "arm.kinematics.";
  k.base_height_m = declare_parameter<double>(kp + "base_height_m", 0.10);
  k.upper_arm_m = declare_parameter<double>(kp + "upper_arm_m", 0.10);
  k.forearm_m = declare_parameter<double>(kp + "forearm_m", 0.095);
  k.hand_m = declare_parameter<double>(kp + "hand_m", 0.12);
  // 관절 기준·범위는 하드웨어 프로필 값이다. 비어 있으면(값 없음·null) 제한 없는 기본값으로 사전 확인만 하고 경고한다
  // (그때 실제 하드웨어의 제어 노드는 장치를 열지 않고 fault 로 시작한다). 형식이 틀리면 시작하지 않는다.
  std::vector<std::string> blank_keys;
  auto prof = [&](const std::string& name) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.dynamic_typing = true;
    const auto v = declare_parameter(name, rclcpp::ParameterValue{}, d);
    const bool is_blank = v.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
                          (v.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
                           (v.get<std::string>().empty() || v.get<std::string>() == "null" || v.get<std::string>() == "~"));
    if (is_blank) blank_keys.push_back(name);
    return std::make_pair(is_blank, v);
  };
  auto prof_d = [&](const std::string& name, double def) {
    const auto [is_blank, v] = prof(name);
    if (is_blank) return def;
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return v.template get<double>();
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) return static_cast<double>(v.template get<int64_t>());
    throw std::invalid_argument(name + " 는 실수여야 합니다");
  };
  auto prof_dv = [&](const std::string& name, const std::vector<double>& def) {
    const auto [is_blank, v] = prof(name);
    if (is_blank) return def;
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) return v.template get<std::vector<double>>();
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      const auto iv = v.template get<std::vector<int64_t>>();
      return std::vector<double>(iv.begin(), iv.end());
    }
    throw std::invalid_argument(name + " 는 실수 목록이어야 합니다");
  };
  k.j1_sign = prof_d(kp + "j1_sign", 1.0);
  k.j1_offset = prof_d(kp + "j1_offset", 0.0);
  k.j2_sign = prof_d(kp + "j2_sign", 1.0);
  k.j2_offset = prof_d(kp + "j2_offset", 0.0);
  k.j3_sign = prof_d(kp + "j3_sign", 1.0);
  k.j3_offset = prof_d(kp + "j3_offset", 0.0);
  k.j4_sign = prof_d(kp + "j4_sign", 1.0);
  k.j4_offset = prof_d(kp + "j4_offset", 0.0);
  // armpi_kinematics 기본값과 같다(제한 없음). 손목 높이 제한 등은 armpi.yaml 에서 준다.
  k.min_wrist_height_m = declare_parameter<double>(kp + "min_wrist_height_m", -1.0e9);
  k.min_elbow_bend_rad = declare_parameter<double>(kp + "min_elbow_bend_rad", 0.0);
  const auto jmin = prof_dv(kp + "joint_min", std::vector<double>{-3.2, -3.2, -3.2, -3.2});
  const auto jmax = prof_dv(kp + "joint_max", std::vector<double>{3.2, 3.2, 3.2, 3.2});
  if (!blank_keys.empty()) {
    std::string list;
    for (const auto& n : blank_keys) list += (list.empty() ? "" : ", ") + n;
    RCLCPP_WARN(get_logger(), "하드웨어 프로필 값 없음 → 제한 없는 기본값으로 사전 확인: %s", list.c_str());
  }
  if (jmin.size() != 4 || jmax.size() != 4) {
    throw std::invalid_argument("arm.kinematics.joint_min/joint_max 는 실수 4개여야 합니다");
  }
  for (int i = 0; i < 4; ++i) {
    k.joint_min[i] = jmin[static_cast<std::size_t>(i)];
    k.joint_max[i] = jmax[static_cast<std::size_t>(i)];
  }

  allowed_sources_ = declare_parameter<std::vector<std::string>>("allowed_sources", std::vector<std::string>{});

  discovery_enabled_ = declare_parameter<bool>("discovery.enabled", true);
  discovery_port_ = static_cast<int>(declare_parameter<int64_t>("discovery.port", 5002));
  discovery_name_ = declare_parameter<std::string>("discovery.name", "");
  discovery_max_rate_ = static_cast<int>(declare_parameter<int64_t>("discovery.max_replies_per_second", 20));

  frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
  arm_frame_id_ = declare_parameter<std::string>("arm_frame_id", "armpi_arm_base");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/armpi/cmd_vel");
  arm_target_topic_ = declare_parameter<std::string>("arm_target_topic", "/armpi/arm/target");
  estop_request_topic_ = declare_parameter<std::string>("estop_request_topic", "/armpi/estop_request");
  control_state_topic_ = declare_parameter<std::string>("control_state_topic", "/armpi/control_state");

  if (port_ < 1 || port_ > 65535) throw std::invalid_argument("port 는 1~65535 여야 합니다");
  if (discovery_port_ < 1 || discovery_port_ > 65535) throw std::invalid_argument("discovery.port 는 1~65535 여야 합니다");
  if (!(publish_rate_hz_ > 0.0) || publish_rate_hz_ > 200.0) throw std::invalid_argument("publish_rate_hz 는 0~200 이어야 합니다");
  if (!(state_rate_hz_ > 0.0)) throw std::invalid_argument("state_rate_hz 는 0보다 커야 합니다");
  if (!(stats_log_period_s_ > 0.0)) throw std::invalid_argument("stats_log_period_sec 는 0보다 커야 합니다");
  session_cfg_.validate();  // 시간값·속도 상한·작업 공간·기구학 검사
}

// ── 구독 ──

void VrBridgeNode::on_control_state(const std_msgs::msg::String::SharedPtr msg) {
  json::Value v;
  std::string err;
  json::ParseOptions opt;
  opt.max_bytes = 65536;
  opt.max_depth = 8;
  bool ok = json::parse(msg->data, v, err, opt) && v.type == json::Type::kObject;
  const json::Value* estop = ok ? v.find("estop") : nullptr;
  const json::Value* fault = ok ? v.find("fault") : nullptr;
  const json::Value* udp = ok ? v.find("udp_allowed") : nullptr;
  ok = ok && estop && estop->type == json::Type::kBool && fault && fault->type == json::Type::kBool && udp &&
       udp->type == json::Type::kBool;
  if (!ok) {
    // 형식이 틀린 상태는 반영하지 않는다(갱신 안 됨 → 2 s 뒤 게이트 차단).
    const double now = monotonic_seconds();
    if (now - last_cs_warn_ >= 1.0) {
      last_cs_warn_ = now;
      RCLCPP_WARN(get_logger(), "control_state 형식 오류(estop/fault/udp_allowed 불리언 필요): %s",
                  err.empty() ? "필드 누락·자료형 오류" : err.c_str());
    }
    return;
  }
  ControlState cs;
  cs.estop = estop->boolean;
  // UDP 출처("udp") 비상정지는 이 수신기가 직접 관리하는 latch 를 제어기가 따라 건 것이다.
  // 제어기 상태에 "udp" 만 남아 있으면 세션 비상정지는 수신기 자신의 latch 로만 판단한다
  // (clear_emergency 직후 제어기 상태가 갱신되기 전에 나가는 ACK 가 estop:true 로 남는 경쟁 방지).
  // 제어기는 자체 latch 로 출력을 계속 막으므로 안전에는 영향이 없다. estop_sources 가 없으면 불리언을 그대로 쓴다.
  if (const auto* src = v.find("estop_sources"); src && src->type == json::Type::kArray) {
    bool other = src->array.empty();  // 출처 목록이 비어 있으면 불리언 값을 그대로 믿는다
    for (const auto& e : src->array) {
      if (e.type != json::Type::kString || e.string != "udp") { other = true; break; }
    }
    cs.estop = estop->boolean && other;
  }
  cs.fault = fault->boolean;
  cs.udp_allowed = udp->boolean;
  if (const auto* r = v.find("fault_reason"); r && r->type == json::Type::kString) cs.fault_reason = r->string;
  server_->set_control_state(cs);
  publish_state(false);
}

// ── 타이머 ──

void VrBridgeNode::on_check_timer() {
  if (shut_down_.load()) return;
  server_->check_timeouts();  // 만료가 있으면 on_stop 콜백이 출력을 갱신한다
  publish_state(false);
  const double now = monotonic_seconds();
  if (now - last_stats_log_ >= stats_log_period_s_) {
    last_stats_log_ = now;
    log_stats();
  }
}

void VrBridgeNode::on_publish_timer() {
  if (shut_down_.load()) return;
  std::lock_guard<std::mutex> lk(out_mutex_);
  const auto outs =
      server_->with_session([](Session& s) { return std::make_pair(s.base_output(), s.arm_output()); });
  publish_cmd_vel_locked(outs.first);  // 20 Hz (정지 중에도 0 을 계속 보낸다)
  const double now = monotonic_seconds();
  // 팔은 active 인 동안만 20 Hz 재발행. 수락 직후 발행한 지 얼마 안 됐으면 건너뛴다(중복 줄임).
  if (outs.second.active && now - last_arm_pub_ >= 0.5 / publish_rate_hz_) publish_arm_locked(outs.second);
}

// ── 발행 ──

void VrBridgeNode::publish_outputs(bool arm_force) {
  if (shut_down_.load()) return;
  std::lock_guard<std::mutex> lk(out_mutex_);
  const auto outs =
      server_->with_session([](Session& s) { return std::make_pair(s.base_output(), s.arm_output()); });
  if (!last_base_ || *last_base_ != outs.first) publish_cmd_vel_locked(outs.first);  // 값 변경 시 즉시
  if (arm_force || !last_arm_ || *last_arm_ != outs.second) publish_arm_locked(outs.second);
}

void VrBridgeNode::publish_cmd_vel_locked(const BaseTarget& t) {
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  msg.twist.linear.x = t.vx;
  msg.twist.linear.y = t.vy;
  msg.twist.angular.z = t.wz;
  cmd_pub_->publish(msg);
  last_base_ = t;
}

void VrBridgeNode::publish_arm_locked(const ArmOutput& a) {
  armpi_interfaces::msg::ArmTarget msg;
  msg.header.stamp = now();
  msg.header.frame_id = arm_frame_id_;
  msg.active = a.active;
  msg.x = a.x;
  msg.y = a.y;
  msg.z = a.z;
  msg.pitch = a.pitch;
  msg.gripper = a.gripper;
  arm_pub_->publish(msg);
  last_arm_ = a;
  last_arm_pub_ = monotonic_seconds();
}

void VrBridgeNode::publish_estop_request(bool latch) {
  std_msgs::msg::String msg;
  msg.data = std::string("{\"source\":\"udp\",\"latch\":") + jbool(latch) + "}";
  estop_pub_->publish(msg);
  RCLCPP_WARN(get_logger(), "비상정지 %s 요청 발행 (%s)", latch ? "latch" : "해제", estop_request_topic_.c_str());
}

void VrBridgeNode::publish_state(bool force) {
  std::lock_guard<std::mutex> lk(state_mutex_);
  const double now = monotonic_seconds();
  const SessionSnapshot s = server_->with_session([&](Session& ss) { return ss.snapshot(server_->now()); });
  std::string key = s.owner.value_or("-") + "|" + s.owner_addr.value_or("-") + "|" + to_string(s.base_state) + "|" +
                    to_string(s.arm_state) + "|" + jbool(s.emergency_stop) + "|" + s.fault.value_or("-") + "|" +
                    jbool(s.udp_permitted);
  if (!force && key == last_state_key_ && now - last_state_pub_ < 1.0 / state_rate_hz_) return;
  std_msgs::msg::String msg;
  msg.data = state_json(s, server_->stats());
  state_pub_->publish(msg);
  last_state_key_ = std::move(key);
  last_state_pub_ = now;
}

std::string VrBridgeNode::state_json(const SessionSnapshot& s, const RxSummary& st) const {
  std::string reasons = "{";
  bool first = true;
  for (const auto& kv : st.reasons) {
    if (!first) reasons += ",";
    first = false;
    reasons += json::quote(kv.first) + ":" + std::to_string(kv.second);
  }
  reasons += "}";
  std::string j = "{";
  j += "\"owner\":" + opt_str(s.owner);
  j += ",\"owner_addr\":" + opt_str(s.owner_addr);
  j += ",\"base_state\":" + json::quote(to_string(s.base_state));
  j += ",\"arm_state\":" + json::quote(to_string(s.arm_state));
  j += std::string(",\"emergency_stop\":") + jbool(s.emergency_stop);
  j += ",\"packets\":{\"received\":" + std::to_string(st.packets) + ",\"accepted\":" + std::to_string(st.accepted) +
       ",\"filtered\":" + std::to_string(st.filtered) + ",\"bytes\":" + std::to_string(st.bytes) +
       ",\"reasons\":" + reasons + ",\"rate_hz\":" + opt_num(st.rate_hz) +
       ",\"interval_p95_s\":" + opt_num(st.interval_p95_s) + ",\"interval_max_s\":" + opt_num(st.interval_max_s) +
       ",\"sequence_gaps\":" + std::to_string(st.sequence_gaps) +
       ",\"estimated_lost\":" + std::to_string(st.estimated_lost) +
       ",\"reordered_or_duplicate\":" + std::to_string(st.reordered_or_duplicate) +
       ",\"last_rx_age_s\":" + opt_num(st.last_rx_age_s) + "}";
  j += std::string(",\"udp_estop_latched\":") + jbool(s.udp_estop_latched);
  j += std::string(",\"controller_estop\":") + jbool(s.controller_estop);
  j += ",\"fault\":" + opt_str(s.fault);
  j += std::string(",\"udp_permitted\":") + jbool(s.udp_permitted);
  j += ",\"blocked_reason\":" + opt_str(s.blocked_reason);
  j += ",\"control_state_age_s\":" + opt_num(s.control_state_age_s);
  j += ",\"last_sequence\":" + std::to_string(s.last_sequence);
  j += ",\"last_reason\":" + json::quote(to_string(s.last_reason));
  j += ",\"base_target\":{\"vx\":" + json::number(s.base_target.vx) + ",\"vy\":" + json::number(s.base_target.vy) +
       ",\"wz\":" + json::number(s.base_target.wz) + "}";
  const auto& a = s.arm_output;
  j += std::string(",\"arm_target\":{\"active\":") + jbool(a.active) + ",\"has_goal\":" + jbool(a.has_goal) +
       ",\"x\":" + json::number(a.x) + ",\"y\":" + json::number(a.y) + ",\"z\":" + json::number(a.z) +
       ",\"pitch\":" + json::number(a.pitch) + ",\"gripper\":" + json::number(a.gripper) + "}";
  j += ",\"port\":" + std::to_string(server_->bound_port());
  if (discovery_) {
    j += ",\"discovery\":{\"port\":" + std::to_string(discovery_->bound_port()) +
         ",\"replies\":" + std::to_string(discovery_->replies_sent()) +
         ",\"filtered\":" + std::to_string(discovery_->filtered()) + "}";
  } else {
    j += ",\"discovery\":null";
  }
  j += "}";
  return j;
}

void VrBridgeNode::log_stats() {
  const RxSummary st = server_->stats();
  if (st.packets == 0 && st.filtered == 0) return;
  auto f = [](const std::optional<double>& v) {
    if (!v) return std::string("-");
    char b[32];
    std::snprintf(b, sizeof(b), "%.3f", *v);
    return std::string(b);
  };
  std::string reasons;
  for (const auto& kv : st.reasons) reasons += kv.first + "=" + std::to_string(kv.second) + " ";
  RCLCPP_INFO(get_logger(),
              "v3 통계: 수신 %lu, 수락 %lu, 주기 %s Hz, p95 간격 %s s, 최대 간격 %s s, 유실 추정 %lu, "
              "역순/중복 %lu, 출처 차단 %lu, 사유 %s",
              static_cast<unsigned long>(st.packets), static_cast<unsigned long>(st.accepted), f(st.rate_hz).c_str(),
              f(st.interval_p95_s).c_str(), f(st.interval_max_s).c_str(),
              static_cast<unsigned long>(st.estimated_lost), static_cast<unsigned long>(st.reordered_or_duplicate),
              static_cast<unsigned long>(st.filtered), reasons.c_str());
}

void VrBridgeNode::ros_log(LogLevel level, const std::string& message) {
  switch (level) {
    case LogLevel::kInfo: RCLCPP_INFO(get_logger(), "%s", message.c_str()); break;
    case LogLevel::kWarn: RCLCPP_WARN(get_logger(), "%s", message.c_str()); break;
    case LogLevel::kError: RCLCPP_ERROR(get_logger(), "%s", message.c_str()); break;
  }
}

// ── 종료 ──

void VrBridgeNode::shutdown_outputs() {
  if (shut_down_.exchange(true)) return;
  if (check_timer_) check_timer_->cancel();
  if (publish_timer_) publish_timer_->cancel();
  if (discovery_) discovery_->shutdown();
  if (!server_) return;
  server_->shutdown();  // 수신 스레드 정지 + 세션 정지(새 중립 요구)
  std::lock_guard<std::mutex> lk(out_mutex_);
  ArmOutput hold = server_->with_session([](Session& s) { return s.arm_output(); });
  hold.active = false;
  for (int i = 0; i < 3; ++i) {
    if (i) std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 깊이 1 QoS 에서 앞 메시지가 덮이지 않게
    publish_cmd_vel_locked(BaseTarget{});
    publish_arm_locked(hold);
  }
  RCLCPP_INFO(get_logger(), "종료: 정지 명령(cmd_vel 0, ArmTarget active:false) 3회 발행");
}

}  // namespace armpi_vr_bridge
