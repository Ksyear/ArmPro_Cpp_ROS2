// armpi_control_node 소스 파일들이 같이 쓰는 작은 도우미(패키지 밖으로 설치하지 않음).
#pragma once

#include <string>
#include <vector>

#include "armpi_common/json.hpp"
#include "armpi_control/arm_controller.hpp"

namespace armpi_control::detail {

// 팔 자세 → JSON 객체(소수 4자리, /armpi/status 형식)
inline std::string pose_json(const ArmPose& p) {
  namespace json = armpi_common::json;
  return "{\"x\":" + json::fixed(p.x, 4) + ",\"y\":" + json::fixed(p.y, 4) + ",\"z\":" + json::fixed(p.z, 4) +
         ",\"pitch\":" + json::fixed(p.pitch, 4) + ",\"gripper\":" + json::fixed(p.gripper, 4) + "}";
}

// 문자열 목록 → JSON 배열
inline std::string str_list(const std::vector<std::string>& v) {
  std::string o = "[";
  for (size_t i = 0; i < v.size(); ++i) o += (i ? "," : "") + armpi_common::json::quote(v[i]);
  return o + "]";
}

// 로그용 쉼표 목록(비면 "-")
inline std::string join(const std::vector<std::string>& v) {
  std::string o;
  for (size_t i = 0; i < v.size(); ++i) o += (i ? "," : "") + v[i];
  return o.empty() ? "-" : o;
}

}  // namespace armpi_control::detail
