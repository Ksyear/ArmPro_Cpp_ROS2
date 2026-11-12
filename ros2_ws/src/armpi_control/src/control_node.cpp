// armpi_control_node: 매개변수 읽기, 장치·구독·타이머 준비, 종료 처리.
// 나머지 멤버 함수는 control_node_{safety,inputs,timers,status}.cpp 에 있다(같은 클래스, 책임별 파일).
#include "armpi_control/control_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

#include "armpi_common/json.hpp"
#include "armpi_kinematics/ros_params.hpp"
#include "control_node_detail.hpp"

namespace armpi_control {

namespace json = armpi_common::json;

namespace ah = armpi_hardware;
using std::placeholders::_1;
using std::placeholders::_2;

namespace {
rclcpp::QoS reliable(size_t depth) { return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable(); }
rclcpp::QoS latched() { return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(); }

std::chrono::nanoseconds period_of(double hz) {
  hz = std::max(0.1, hz);
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));
}
}  // namespace

double ControlNode::now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ───────────────────────── 설정 ─────────────────────────

void ControlNode::load_parameters() {
  mock_ = declare_parameter<bool>("hardware.mock", false);
  hw_cfg_.i2c_device = declare_parameter<std::string>("hardware.i2c_device", "/dev/i2c-1");
  hw_cfg_.motor_board_address = static_cast<uint8_t>(declare_parameter<int64_t>("hardware.motor_board_address", 0x34));
  hw_cfg_.motor_type = static_cast<uint8_t>(declare_parameter<int64_t>("hardware.motor_type", 3));
  hw_cfg_.servo_port = declare_parameter<std::string>("hardware.servo_port", "/dev/ttyS0");
  hw_cfg_.servo_baud = static_cast<int>(declare_parameter<int64_t>("hardware.servo_baud", 115200));
  hw_cfg_.gpio_chip = declare_parameter<std::string>("hardware.gpio_chip", "gpiochip0");
  fault_threshold_ = static_cast<int>(declare_parameter<int64_t>("hardware.fault_threshold", 5));

  hw_cfg_.servo_timing.drain_before_turnaround = declare_parameter<bool>("servo.drain_before_turnaround", true);
  arm_cfg_.read_position = declare_parameter<bool>("servo.read_position", false);
  arm_cfg_.read_rate_hz = declare_parameter<double>("servo.read_rate_hz", 5.0);

  fan_enabled_ = declare_parameter<bool>("fan.enabled", true);
  buzzer_enabled_ = declare_parameter<bool>("buzzer.enabled", true);
  estop_beep_s_ = declare_parameter<double>("buzzer.estop_beep_sec", 0.1);
  startup_beep_enabled_ = declare_parameter<bool>("buzzer.startup_beep", true);
  startup_beep_on_s_ = declare_parameter<double>("buzzer.startup_beep_on_sec", 0.1);
  startup_beep_off_s_ = declare_parameter<double>("buzzer.startup_beep_off_sec", 0.15);
  buttons_enabled_ = declare_parameter<bool>("buttons.enabled", true);
  button_debounce_s_ = declare_parameter<double>("buttons.debounce_sec", 0.05);
  key2_hold_s_ = declare_parameter<double>("buttons.key2_hold_sec", 2.0);

  // 차체 (chassis.max_* 는 armpi_vr_bridge 와 같은 이름·값)
  chassis_cfg_.max_vx_m_s = declare_parameter<double>("chassis.max_vx_m_s", 0.25);  // 사용자 결정 2026-11-04, 바퀴 방향 실기 시험 전
  chassis_cfg_.max_vy_m_s = declare_parameter<double>("chassis.max_vy_m_s", 0.25);
  chassis_cfg_.max_wz_rad_s = declare_parameter<double>("chassis.max_wz_rad_s", 1.2);
  chassis_cfg_.max_linear_accel_m_s2 = declare_parameter<double>("chassis.max_linear_accel_m_s2", 0.6);
  chassis_cfg_.max_angular_accel_rad_s2 = declare_parameter<double>("chassis.max_angular_accel_rad_s2", 4.0);
  chassis_cfg_.cmd_timeout_s = declare_parameter<double>("chassis.cmd_timeout_sec", 0.6);
  chassis_cfg_.max_cmd_age_s = declare_parameter<double>("chassis.max_cmd_age_sec", 0.5);
  chassis_cfg_.angular_sign = declare_parameter<double>("chassis.angular_sign", 1.0);
  chassis_cfg_.max_wheel_pulse = static_cast<int>(declare_parameter<int64_t>("chassis.max_wheel_pulse", 100));
  chassis_rate_hz_ = declare_parameter<double>("chassis.rate_hz", 50.0);
  wheel_refresh_s_ = declare_parameter<double>("chassis.wheel_refresh_sec", 0.5);

  // 팔
  arm_rate_hz_ = declare_parameter<double>("arm.rate_hz", 50.0);
  home_on_start_ = declare_parameter<bool>("arm.home_on_start", true);
  arm_cfg_.target_timeout_s = declare_parameter<double>("arm.target_timeout_sec", 0.6);
  arm_cfg_.limits.max_linear_speed_m_s = declare_parameter<double>("arm.max_linear_speed_m_s", 0.15);  // 사용자 결정 2026-11-04, 바퀴 방향 실기 시험 전
  arm_cfg_.limits.max_pitch_rate_rad_s = declare_parameter<double>("arm.max_pitch_rate_rad_s", 1.0);
  arm_cfg_.limits.max_gripper_rate_per_s = declare_parameter<double>("arm.max_gripper_rate_per_s", 1.0);
  arm_cfg_.max_joint_speed_rad_s = declare_parameter<double>("arm.max_joint_speed_rad_s", 2.0);
  arm_cfg_.command_duration_ms = static_cast<int>(declare_parameter<int64_t>("arm.command_duration_ms", 40));
  arm_cfg_.home_duration_ms = static_cast<int>(declare_parameter<int64_t>("arm.home_duration_ms", 1500));
  arm_cfg_.joint5_servo_value = static_cast<int>(declare_parameter<int64_t>("arm.joint5_servo_value", 500));
  arm_cfg_.joint_debug_speed_rad_s = declare_parameter<double>("arm.joint_debug.speed_rad_s", 1.0);
  arm_cfg_.joint_debug_min_duration_ms =
      static_cast<int>(declare_parameter<int64_t>("arm.joint_debug.min_duration_ms", 200));
  arm_cfg_.joint_debug_max_duration_ms =
      static_cast<int>(declare_parameter<int64_t>("arm.joint_debug.max_duration_ms", 3000));
  arm_cfg_.fault_threshold = fault_threshold_;

  auto& w = arm_cfg_.workspace;  // armpi_vr_bridge 와 같은 이름·값
  w.x_min = declare_parameter<double>("arm.workspace.x_min", 0.05);
  w.x_max = declare_parameter<double>("arm.workspace.x_max", 0.32);
  w.y_min = declare_parameter<double>("arm.workspace.y_min", -0.25);
  w.y_max = declare_parameter<double>("arm.workspace.y_max", 0.25);
  w.z_min = declare_parameter<double>("arm.workspace.z_min", -0.08);
  w.z_max = declare_parameter<double>("arm.workspace.z_max", 0.32);

  auto& k = arm_cfg_.kinematics;  // 기본값
  const std::string kp = "arm.kinematics.";
  k.base_height_m = declare_parameter<double>(kp + "base_height_m", 0.064605);
  k.upper_arm_m = declare_parameter<double>(kp + "upper_arm_m", 0.10048);
  k.forearm_m = declare_parameter<double>(kp + "forearm_m", 0.094714);
  k.hand_m = declare_parameter<double>(kp + "hand_m", 0.16331);
  k.min_wrist_height_m = declare_parameter<double>(kp + "min_wrist_height_m", 0.0);
  k.min_elbow_bend_rad = declare_parameter<double>(kp + "min_elbow_bend_rad", 0.05);

  load_hardware_profile();
  // 보간 경로가 막혔을 때의 처리(armpi_vr_bridge·armpi_web_gateway 와 같은 arm.target_policy·arm.nearest.*)
  {
    const auto tp = armpi_kinematics::declare_target_params(*this);
    arm_cfg_.target_policy = tp.policy;
    arm_cfg_.reach = tp.reach;
  }

  status_rate_hz_ = declare_parameter<double>("status.rate_hz", 2.0);
  joint_states_rate_hz_ = declare_parameter<double>("joint_states.rate_hz", 20.0);
  battery_rate_hz_ = declare_parameter<double>("battery.rate_hz", 1.0);
  battery_empty_v_ = declare_parameter<double>("battery.empty_v", 6.6);
  battery_full_v_ = declare_parameter<double>("battery.full_v", 8.4);
  battery_low_v_ = declare_parameter<double>("battery.low_v", 6.8);
  hw_state_rate_hz_ = declare_parameter<double>("hw_state.rate_hz", 10.0);
  bridge_state_topic_ = declare_parameter<std::string>("vr_bridge_state_topic", "/armpi_vr_bridge/state");
  bridge_state_timeout_s_ = declare_parameter<double>("vr_bridge_state_timeout_sec", 2.0);
  boot_check_enabled_ = declare_parameter<bool>("boot_check.enabled", true);
  boot_manifest_ = declare_parameter<std::string>("boot_check.manifest", "/opt/armpi/manifests/stage20.env");
}

// 하드웨어 프로필(로봇마다 다른 장치 값, 코드 기본값 없음). armpi_hardware/hardware_profile.hpp 참고.
void ControlNode::load_hardware_profile() {
  namespace ah_ = armpi_hardware;
  ah_::HardwareProfile p;
  const double nan = ah_::kUnsetDouble;
  using PT = rclcpp::ParameterType;
  // 형식을 정하지 않고 받는다: 값 없음·null·~·빈 문자열은 "비어 있음"(본보기 그대로 넣어도 노드가 죽지 않게),
  // 형식이 틀리면 bad_type 에 모아 fault 이유로 보고한다. 실수 자리의 정수는 받아 준다.
  std::vector<std::string> bad_type;
  auto raw = [this](const std::string& name) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.dynamic_typing = true;
    return declare_parameter(name, rclcpp::ParameterValue{}, d);
  };
  auto blank = [](const rclcpp::ParameterValue& v) {
    if (v.get_type() == PT::PARAMETER_NOT_SET) return true;
    if (v.get_type() != PT::PARAMETER_STRING) return false;
    const auto& t = v.get<std::string>();
    return t.empty() || t == "null" || t == "~";
  };
  auto dval = [&](const std::string& name) -> double {
    const auto v = raw(name);
    if (blank(v)) return nan;
    if (v.get_type() == PT::PARAMETER_DOUBLE) return v.get<double>();
    if (v.get_type() == PT::PARAMETER_INTEGER) return static_cast<double>(v.get<int64_t>());
    bad_type.push_back(name + "(실수)");
    return nan;
  };
  auto ival = [&](const std::string& name, int unset = ah_::kUnsetInt) -> int {
    const auto v = raw(name);
    if (blank(v)) return unset;
    if (v.get_type() == PT::PARAMETER_INTEGER) return static_cast<int>(v.get<int64_t>());
    bad_type.push_back(name + "(정수)");
    return unset;
  };
  auto ivec = [&](const std::string& name) -> std::vector<int64_t> {
    const auto v = raw(name);
    if (blank(v)) return {};
    if (v.get_type() == PT::PARAMETER_INTEGER_ARRAY) return v.get<std::vector<int64_t>>();
    bad_type.push_back(name + "(정수 목록)");
    return {};
  };
  auto dvec = [&](const std::string& name) -> std::vector<double> {
    const auto v = raw(name);
    if (blank(v)) return {};
    if (v.get_type() == PT::PARAMETER_DOUBLE_ARRAY) return v.get<std::vector<double>>();
    if (v.get_type() == PT::PARAMETER_INTEGER_ARRAY) {
      const auto iv = v.get<std::vector<int64_t>>();
      return std::vector<double>(iv.begin(), iv.end());
    }
    bad_type.push_back(name + "(실수 목록)");
    return {};
  };

  p.mecanum.a_mm = dval("chassis.a_mm");
  p.mecanum.b_mm = dval("chassis.b_mm");
  p.mecanum.wheel_diameter_mm = dval("chassis.wheel_diameter_mm");
  p.mecanum.pulse_per_cycle = dval("chassis.pulse_per_cycle");

  const auto ids = ivec("arm.servo_ids");
  const auto dirs = ivec("arm.joint_directions");
  if (ids.size() == ah_::kNumJoints && dirs.size() == ah_::kNumJoints) {
    for (int i = 0; i < ah_::kNumJoints; ++i) {
      p.servo_map.servo_ids[i] = static_cast<int>(ids[i]);
      p.servo_map.directions[i] = static_cast<int>(dirs[i]);
    }
  }
  p.gripper.open_value = ival("arm.gripper.open_value");
  p.gripper.close_value = ival("arm.gripper.close_value");
  p.gripper_servo_min = ival("arm.gripper.servo_min");
  p.gripper_servo_max = ival("arm.gripper.servo_max");
  const auto home = ivec("arm.home_servo_values");
  if (home.size() == ah_::kNumJoints) {
    for (int i = 0; i < ah_::kNumJoints; ++i) p.home_servo_values[i] = static_cast<int>(home[i]);
  }

  const std::string kp = "arm.kinematics.";
  for (int i = 0; i < 4; ++i) {
    const std::string n = std::to_string(i + 1);
    p.joint_sign[i] = dval(kp + "j" + n + "_sign");
    p.joint_offset[i] = dval(kp + "j" + n + "_offset");
  }
  const auto jmin = dvec(kp + "joint_min");
  const auto jmax = dvec(kp + "joint_max");
  if (jmin.size() == 4 && jmax.size() == 4) {
    for (int i = 0; i < 4; ++i) {
      p.joint_min[i] = jmin[i];
      p.joint_max[i] = jmax[i];
    }
  }

  p.expansion.address = ival("hardware.expansion_board_address");
  p.expansion.battery_register = ival("hardware.battery_register");
  p.expansion.dc_motor_register = ival("hardware.dc_motor_register");
  p.expansion.dc_motor_sign = ival("hardware.dc_motor_sign", 0);
  p.fan_motor_index = ival("fan.motor_index");
  p.fan_speed = ival("fan.speed", ah_::kUnsetSpeed);
  p.servo_rx_con_bcm = ival("hardware.servo_rx_con_bcm");
  p.servo_tx_con_bcm = ival("hardware.servo_tx_con_bcm");
  p.buzzer_bcm = ival("buzzer.bcm");
  p.key1_bcm = ival("buttons.key1_bcm");
  p.key2_bcm = ival("buttons.key2_bcm");
  p.servo_timing.drain_before_turnaround = hw_cfg_.servo_timing.drain_before_turnaround;
  p.servo_timing.pre_write_delay_us = ival("servo.pre_write_delay_us");
  p.servo_timing.post_write_delay_us = ival("servo.post_write_delay_us");
  p.servo_timing.read_turnaround_delay_us = ival("servo.read_turnaround_delay_us");
  p.servo_timing.read_timeout_ms = ival("servo.read_timeout_ms");
  p.servo_timing.read_attempts = ival("servo.read_attempts");

  if (!bad_type.empty()) {
    std::string list;
    for (const auto& b : bad_type) list += (list.empty() ? "" : ", ") + b;
    RCLCPP_ERROR(get_logger(), "하드웨어 프로필 값의 형식이 틀림(비어 있는 것으로 봄): %s", list.c_str());
  }
  if (mock_) {
    const auto filled = ah_::fill_unset(&p, ah_::test_hardware_profile());
    if (!filled.empty()) {
      std::string list;
      for (const auto& f : filled) list += (list.empty() ? "" : ", ") + f;
      RCLCPP_WARN(get_logger(), "가짜 하드웨어: 비어 있는 프로필 값을 시험용 값으로 채움(실제 로봇 값 아님): %s",
                  list.c_str());
    }
  } else {
    auto missing = ah_::missing_profile_items(p, ah_::ProfileUse{fan_enabled_, buzzer_enabled_, buttons_enabled_});
    for (const auto& b : bad_type) missing.push_back(b + " 형식이 틀림");
    for (const auto& m : missing) profile_missing_ += (profile_missing_.empty() ? "" : "; ") + m;
  }

  // 적용
  chassis_cfg_.mecanum = p.mecanum;
  arm_cfg_.servo_map = p.servo_map;
  arm_cfg_.gripper = p.gripper;
  arm_cfg_.gripper_servo_min = p.gripper_servo_min;
  arm_cfg_.gripper_servo_max = p.gripper_servo_max;
  arm_cfg_.home_servo_values = p.home_servo_values;
  auto& k = arm_cfg_.kinematics;
  k.j1_sign = p.joint_sign[0];
  k.j2_sign = p.joint_sign[1];
  k.j3_sign = p.joint_sign[2];
  k.j4_sign = p.joint_sign[3];
  k.j1_offset = p.joint_offset[0];
  k.j2_offset = p.joint_offset[1];
  k.j3_offset = p.joint_offset[2];
  k.j4_offset = p.joint_offset[3];
  for (int i = 0; i < 4; ++i) {
    k.joint_min[i] = p.joint_min[i];
    k.joint_max[i] = p.joint_max[i];
  }
  hw_cfg_.expansion = p.expansion;
  hw_cfg_.servo_rx_con_bcm = p.servo_rx_con_bcm;
  hw_cfg_.servo_tx_con_bcm = p.servo_tx_con_bcm;
  hw_cfg_.servo_timing = p.servo_timing;
  fan_motor_index_ = p.fan_motor_index;
  fan_speed_ = p.fan_speed;
  buzzer_bcm_ = p.buzzer_bcm;
  key1_bcm_ = p.key1_bcm;
  key2_bcm_ = p.key2_bcm;
}

// ───────────────────────── 생성·종료 ─────────────────────────

ControlNode::ControlNode(const rclcpp::NodeOptions& options) : rclcpp::Node("armpi_control", options) {
  load_parameters();
  const double now = now_s();

  // 장치
  if (mock_) {
    hw_ = ah::make_mock_hardware(&ControlNode::now_s, 7400, hw_cfg_.expansion);
    RCLCPP_WARN(get_logger(), "가짜 하드웨어(hardware.mock:=true)로 실행: 실제 바퀴·서보·GPIO 를 쓰지 않음");
  } else if (!profile_missing_.empty()) {
    // 하드웨어 프로필이 비어 있으면 장치를 열지 않는다(바퀴·서보에 아무것도 보내지 않음)
    hw_ = ah::Hardware{};
    set_fault("하드웨어 프로필 값 없음(armpi_bringup hardware_profile.yaml 확인): " + profile_missing_);
  } else {
    std::string errors;
    hw_ = ah::open_real_hardware(hw_cfg_, &errors);
    if (!errors.empty()) set_fault("하드웨어 초기화 실패: " + errors);
  }

  chassis_ = std::make_unique<ChassisController>(chassis_cfg_);
  arm_ = std::make_unique<ArmController>(arm_cfg_, hw_.servos);
  key1_ = std::make_unique<ButtonDetector>(button_debounce_s_, 1e9);  // KEY1 은 길게 누름 없음
  key2_ = std::make_unique<ButtonDetector>(button_debounce_s_, key2_hold_s_);

  std::string why;
  if (!arm_->validate(&why)) set_fault("팔 설정 오류: " + why);

  // GPIO: 부저(출력 0), 버튼(입력 풀업)
  if (hw_.gpio) {
    std::string e;
    if (buzzer_enabled_) {
      buzzer_ok_ = hw_.gpio->request_output(buzzer_bcm_, 0, &e);
      if (!buzzer_ok_) init_warnings_ += "부저 GPIO" + std::to_string(buzzer_bcm_) + ": " + e + "; ";
    }
    if (buttons_enabled_) {
      e.clear();
      std::string e2;
      const bool k1 = hw_.gpio->request_input(key1_bcm_, true, &e);
      const bool k2 = hw_.gpio->request_input(key2_bcm_, true, &e2);
      buttons_ok_ = k1 && k2;
      if (!e.empty()) init_warnings_ += "KEY1: " + e + "; ";
      if (!e2.empty()) init_warnings_ += "KEY2: " + e2 + "; ";
    }
  } else if (buzzer_enabled_ || buttons_enabled_) {
    init_warnings_ += "GPIO 없음: 부저·버튼 사용 불가; ";
  }
  if (!init_warnings_.empty()) RCLCPP_WARN(get_logger(), "초기화 경고: %s", init_warnings_.c_str());

  // 바퀴 정지, 팬 켜기(모터 1번 = 팬)
  write_wheels({0, 0, 0, 0}, now, true);
  if (fan_enabled_ && hw_.expansion) {
    std::string e;
    fan_ok_ = hw_.expansion->set_dc_motor(fan_motor_index_, fan_speed_, &e);
    if (fan_ok_) {
      RCLCPP_INFO(get_logger(), "냉각 팬 켬 (DC 모터 %d, 속도 %d)", fan_motor_index_, fan_speed_);
    } else {
      RCLCPP_WARN(get_logger(), "냉각 팬 켜기 실패: %s", e.c_str());
    }
  }

  // ROS 인터페이스
  control_state_pub_ = create_publisher<std_msgs::msg::String>("/armpi/control_state", latched());
  status_pub_ = create_publisher<std_msgs::msg::String>("/armpi/status", reliable(1));
  battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/armpi/battery", reliable(10));
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", reliable(10));
  shutdown_pub_ = create_publisher<std_msgs::msg::Empty>("/armpi/shutdown_requested", reliable(1));
  hw_state_pub_ = create_publisher<std_msgs::msg::String>("~/hw_state", reliable(1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/armpi/cmd_vel", reliable(1), std::bind(&ControlNode::on_cmd_vel, this, _1));
  arm_target_sub_ = create_subscription<armpi_interfaces::msg::ArmTarget>(
      "/armpi/arm/target", reliable(1), std::bind(&ControlNode::on_arm_target, this, _1));
  estop_req_sub_ = create_subscription<std_msgs::msg::String>(
      "/armpi/estop_request", reliable(10), std::bind(&ControlNode::on_estop_request, this, _1));
  joint_debug_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/armpi/arm/joint_debug", reliable(10), std::bind(&ControlNode::on_joint_debug, this, _1));
  bridge_state_sub_ = create_subscription<std_msgs::msg::String>(
      bridge_state_topic_, latched(), std::bind(&ControlNode::on_bridge_state, this, _1));
  if (mock_) {
    mock_cmd_sub_ = create_subscription<std_msgs::msg::String>("~/mock_cmd", reliable(10),
                                                               std::bind(&ControlNode::on_mock_cmd, this, _1));
  }

  estop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/armpi/emergency_stop", std::bind(&ControlNode::srv_emergency_stop, this, _1, _2));
  reset_srv_ = create_service<std_srvs::srv::Trigger>("/armpi/reset_estop",
                                                      std::bind(&ControlNode::srv_reset_estop, this, _1, _2));
  home_srv_ =
      create_service<std_srvs::srv::Trigger>("/armpi/arm/home", std::bind(&ControlNode::srv_home, this, _1, _2));

  // 시작 자세
  if (home_on_start_ && !fault()) {
    std::string msg;
    if (arm_->home(now, &msg)) {
      RCLCPP_INFO(get_logger(), "시작 자세: %s", msg.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "시작 자세 실패: %s", msg.c_str());
    }
  } else if (!home_on_start_) {
    RCLCPP_WARN(get_logger(), "arm.home_on_start=false: /armpi/arm/home 을 부르기 전에는 팔 목표를 무시함(현재 위치 모름)");
  }
  drain_arm_events();

  chassis_timer_ = create_wall_timer(period_of(chassis_rate_hz_), std::bind(&ControlNode::chassis_tick, this));
  arm_timer_ = create_wall_timer(period_of(arm_rate_hz_), std::bind(&ControlNode::arm_tick, this));
  misc_timer_ = create_wall_timer(period_of(50.0), std::bind(&ControlNode::misc_tick, this));
  status_timer_ = create_wall_timer(period_of(status_rate_hz_), std::bind(&ControlNode::status_tick, this));
  joint_timer_ = create_wall_timer(period_of(joint_states_rate_hz_), std::bind(&ControlNode::joint_states_tick, this));
  battery_timer_ = create_wall_timer(period_of(battery_rate_hz_), std::bind(&ControlNode::battery_tick, this));
  hw_state_timer_ = create_wall_timer(period_of(hw_state_rate_hz_), std::bind(&ControlNode::hw_state_tick, this));

  RCLCPP_INFO(get_logger(),
              "armpi_control 시작 (%s): 차체 |vx|≤%.2f |vy|≤%.2f m/s |wz|≤%.2f rad/s, 가속 %.2f m/s²·%.1f rad/s², "
              "watchdog %.2fs, stamp 최대 %.2fs, angular_sign %+.0f | 팔 %.2f m/s, %.1f rad/s, 그리퍼 %.1f/s, "
              "관절 %.1f rad/s, %d ms, 위치 피드백 %s, 경로 막힘 처리 %s",
              mock_ ? "가짜 하드웨어" : "실제 하드웨어", chassis_cfg_.max_vx_m_s, chassis_cfg_.max_vy_m_s,
              chassis_cfg_.max_wz_rad_s, chassis_cfg_.max_linear_accel_m_s2, chassis_cfg_.max_angular_accel_rad_s2,
              chassis_cfg_.cmd_timeout_s, chassis_cfg_.max_cmd_age_s, chassis_cfg_.angular_sign,
              arm_cfg_.limits.max_linear_speed_m_s, arm_cfg_.limits.max_pitch_rate_rad_s,
              arm_cfg_.limits.max_gripper_rate_per_s, arm_cfg_.max_joint_speed_rad_s, arm_cfg_.command_duration_ms,
              arm_cfg_.read_position ? "POS_READ" : "명령값", armpi_kinematics::to_string(arm_cfg_.target_policy));
  run_boot_check();
  startup_beep();
  publish_control_state(true);
}

void ControlNode::run_boot_check() {
  if (!boot_check_enabled_) return;
  const BootFacts f = read_boot_facts(boot_manifest_);
  kernel_release_ = f.kernel_release;
  boot_warnings_ = boot_warnings(f);
  for (const auto& w : boot_warnings_) RCLCPP_ERROR(get_logger(), "부팅 점검: %s", w.c_str());
  if (f.manifest_found && boot_warnings_.empty()) {
    RCLCPP_INFO(get_logger(), "부팅 점검: 이미지와 같은 커널 %s·부팅 설정", f.kernel_release.c_str());
  }
}

ControlNode::~ControlNode() { shutdown_outputs(); }

void ControlNode::shutdown_outputs() {
  if (outputs_shut_down_) return;
  outputs_shut_down_ = true;
  try {
    if (hw_.motors) {
      std::string e;
      for (int i = 0; i < 3; ++i) {
        if (hw_.motors->set_speeds({0, 0, 0, 0}, &e)) break;
      }
    }
    if (hw_.gpio && buzzer_ok_) {
      std::string e;
      hw_.gpio->write(buzzer_bcm_, 0, &e);
    }
  } catch (...) {
  }
  // 서보 토크는 유지한다(LOAD_OR_UNLOAD 0 을 보내지 않음): 팔이 떨어지지 않게.
}

}  // namespace armpi_control
