// ROS 2 노드 매개변수에서 팔 작업 공간·기구학·목표 보정 설정을 읽는다(header-only).
// rclcpp 를 쓰는 패키지(armpi_vr_bridge, armpi_web_gateway)에서만 include 한다. armpi_kinematics 라이브러리 자체는
// ROS 에 의존하지 않는다.
//
// 매개변수 이름은 armpi_bringup/config/armpi.yaml 의 /** 구역과 하드웨어 프로필(hardware_profile.yaml)을 따른다.
//   arm.workspace.{x,y,z}_{min,max}
//   arm.kinematics.{base_height_m, upper_arm_m, forearm_m, hand_m, min_wrist_height_m, min_elbow_bend_rad}
//   arm.kinematics.{j1..j4}_{sign,offset}, joint_min, joint_max        (하드웨어 프로필 값, 비어 있으면 제한 없는 기본값)
//   arm.target_policy                                                  ("reject" | "nearest")
//   arm.nearest.{max_pitch_change_rad, pitch_step_rad, pitch_min_rad, pitch_max_rad, allow_position_change,
//                bisect_iterations}
#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "armpi_kinematics/arm_kinematics.hpp"
#include "armpi_kinematics/reachability.hpp"

namespace armpi_kinematics {

// 노드마다 코드 기본값이 다를 수 있다(수신기는 단독 시험용 값, 로봇에서는 armpi.yaml 이 덮어쓴다).
struct ArmParamDefaults {
  Workspace workspace{0.0, 0.30, -0.25, 0.25, -0.05, 0.35};
  double base_height_m{0.10};
  double upper_arm_m{0.10};
  double forearm_m{0.095};
  double hand_m{0.12};
  double min_wrist_height_m{-1.0e9};  // armpi_kinematics 기본값(제한 없음)
  double min_elbow_bend_rad{0.0};
  TargetPolicy target_policy{TargetPolicy::kReject};  // 매개변수가 없으면 규격 v3 원래 동작
};

struct ArmParams {
  Workspace workspace{};
  Params kinematics{};
  TargetPolicy target_policy{TargetPolicy::kReject};
  ReachOptions reach{};
  std::vector<std::string> blank_profile_keys;  // 하드웨어 프로필 값이 비어 제한 없는 기본값을 쓴 항목
};

struct TargetParams {
  TargetPolicy policy{TargetPolicy::kReject};
  ReachOptions reach{};
};

// arm.target_policy 와 arm.nearest.* 만 읽는다(작업 공간·기구학을 따로 읽는 armpi_control 용). 형식이 틀리면 예외.
inline TargetParams declare_target_params(rclcpp::Node& node, TargetPolicy default_policy = TargetPolicy::kReject) {
  TargetParams out;
  const std::string policy = node.declare_parameter<std::string>("arm.target_policy", to_string(default_policy));
  if (!parse_target_policy(policy, &out.policy)) {
    throw std::invalid_argument("arm.target_policy 는 \"reject\" 또는 \"nearest\": " + policy);
  }
  auto& r = out.reach;
  const ReachOptions rd;
  const std::string np = "arm.nearest.";
  r.max_pitch_change_rad = node.declare_parameter<double>(np + "max_pitch_change_rad", rd.max_pitch_change_rad);
  r.pitch_step_rad = node.declare_parameter<double>(np + "pitch_step_rad", rd.pitch_step_rad);
  r.pitch_min_rad = node.declare_parameter<double>(np + "pitch_min_rad", rd.pitch_min_rad);
  r.pitch_max_rad = node.declare_parameter<double>(np + "pitch_max_rad", rd.pitch_max_rad);
  r.allow_position_change = node.declare_parameter<bool>(np + "allow_position_change", rd.allow_position_change);
  r.bisect_iterations = static_cast<int>(node.declare_parameter<int64_t>(np + "bisect_iterations", rd.bisect_iterations));
  const std::string err = r.validate();
  if (!err.empty()) throw std::invalid_argument(err);
  return out;
}

// 형식이 틀리면 std::invalid_argument(한국어 설명). 값이 비어 있는 프로필 항목은 blank_profile_keys 에 남긴다.
inline ArmParams declare_arm_params(rclcpp::Node& node, const ArmParamDefaults& d = ArmParamDefaults{}) {
  ArmParams out;
  auto& w = out.workspace;
  w.x_min = node.declare_parameter<double>("arm.workspace.x_min", d.workspace.x_min);
  w.x_max = node.declare_parameter<double>("arm.workspace.x_max", d.workspace.x_max);
  w.y_min = node.declare_parameter<double>("arm.workspace.y_min", d.workspace.y_min);
  w.y_max = node.declare_parameter<double>("arm.workspace.y_max", d.workspace.y_max);
  w.z_min = node.declare_parameter<double>("arm.workspace.z_min", d.workspace.z_min);
  w.z_max = node.declare_parameter<double>("arm.workspace.z_max", d.workspace.z_max);

  auto& k = out.kinematics;
  const std::string kp = "arm.kinematics.";
  k.base_height_m = node.declare_parameter<double>(kp + "base_height_m", d.base_height_m);
  k.upper_arm_m = node.declare_parameter<double>(kp + "upper_arm_m", d.upper_arm_m);
  k.forearm_m = node.declare_parameter<double>(kp + "forearm_m", d.forearm_m);
  k.hand_m = node.declare_parameter<double>(kp + "hand_m", d.hand_m);

  // 관절 기준·범위는 하드웨어 프로필 값이다. 비어 있으면(값 없음·null) 제한 없는 기본값을 쓰고 이름을 남긴다.
  auto prof = [&](const std::string& name) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.dynamic_typing = true;
    const auto v = node.declare_parameter(name, rclcpp::ParameterValue{}, desc);
    const bool is_blank = v.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
                          (v.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
                           (v.get<std::string>().empty() || v.get<std::string>() == "null" ||
                            v.get<std::string>() == "~"));
    if (is_blank) out.blank_profile_keys.push_back(name);
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
  k.min_wrist_height_m = node.declare_parameter<double>(kp + "min_wrist_height_m", d.min_wrist_height_m);
  k.min_elbow_bend_rad = node.declare_parameter<double>(kp + "min_elbow_bend_rad", d.min_elbow_bend_rad);
  const auto jmin = prof_dv(kp + "joint_min", std::vector<double>{-3.2, -3.2, -3.2, -3.2});
  const auto jmax = prof_dv(kp + "joint_max", std::vector<double>{3.2, 3.2, 3.2, 3.2});
  if (jmin.size() != 4 || jmax.size() != 4) {
    throw std::invalid_argument("arm.kinematics.joint_min/joint_max 는 실수 4개여야 합니다");
  }
  for (int i = 0; i < 4; ++i) {
    k.joint_min[i] = jmin[static_cast<std::size_t>(i)];
    k.joint_max[i] = jmax[static_cast<std::size_t>(i)];
  }

  const auto target = declare_target_params(node, d.target_policy);
  out.target_policy = target.policy;
  out.reach = target.reach;
  return out;
}

}  // namespace armpi_kinematics
