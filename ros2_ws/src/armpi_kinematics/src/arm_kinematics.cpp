#include "armpi_kinematics/arm_kinematics.hpp"

#include <cmath>

namespace armpi_kinematics {
namespace {
bool finite(double v) { return std::isfinite(v); }
}  // namespace

ArmKinematics::ArmKinematics(const Params& p) : p_(p) {}

bool ArmKinematics::valid() const {
  if (!(p_.upper_arm_m > 0 && p_.forearm_m > 0 && p_.hand_m >= 0)) return false;
  for (int i = 0; i < 4; ++i) {
    if (!(p_.joint_min[i] < p_.joint_max[i])) return false;
  }
  return p_.j1_sign != 0 && p_.j2_sign != 0 && p_.j3_sign != 0 && p_.j4_sign != 0;
}

IkResult ArmKinematics::inverse(const Pose& t) const {
  IkResult out;
  if (!valid() || !finite(t.x) || !finite(t.y) || !finite(t.z) || !finite(t.pitch)) {
    out.status = IkStatus::kInvalid;
    return out;
  }
  const double r = std::hypot(t.x, t.y);
  const double yaw = (r < 1e-9) ? 0.0 : std::atan2(t.y, t.x);
  // 팔 평면 안의 손목 중심
  const double rw = r - p_.hand_m * std::cos(t.pitch);
  const double zw = (t.z - p_.base_height_m) - p_.hand_m * std::sin(t.pitch);
  if (zw + p_.base_height_m < p_.min_wrist_height_m - 1e-12) {
    out.status = IkStatus::kOutOfReach;
    return out;
  }
  const double L1 = p_.upper_arm_m, L2 = p_.forearm_m;
  const double d2 = rw * rw + zw * zw;
  double c3 = (d2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  if (c3 > 1.0 + 1e-9 || c3 < -1.0 - 1e-9) {
    out.status = IkStatus::kOutOfReach;
    return out;
  }
  if (c3 > 1.0) c3 = 1.0;
  if (c3 < -1.0) c3 = -1.0;
  const double q3 = -std::acos(c3);  // elbow-up: 팔꿈치가 위로, 아래팔이 아래로 꺾임
  if (-q3 < p_.min_elbow_bend_rad) {
    out.status = IkStatus::kOutOfReach;
    return out;
  }
  const double q2 = std::atan2(zw, rw) - std::atan2(L2 * std::sin(q3), L1 + L2 * std::cos(q3));
  const double q4 = t.pitch - q2 - q3;

  JointAngles j;
  j.j1 = p_.j1_sign * yaw + p_.j1_offset;
  j.j2 = p_.j2_sign * q2 + p_.j2_offset;
  j.j3 = p_.j3_sign * q3 + p_.j3_offset;
  j.j4 = p_.j4_sign * q4 + p_.j4_offset;
  const double v[4] = {j.j1, j.j2, j.j3, j.j4};
  for (int i = 0; i < 4; ++i) {
    if (!finite(v[i]) || v[i] < p_.joint_min[i] - 1e-9 || v[i] > p_.joint_max[i] + 1e-9) {
      out.status = IkStatus::kJointLimit;
      out.joints = j;
      return out;
    }
  }
  out.status = IkStatus::kOk;
  out.joints = j;
  return out;
}

Pose ArmKinematics::forward(const JointAngles& q) const {
  const double yaw = (q.j1 - p_.j1_offset) / p_.j1_sign;
  const double q2 = (q.j2 - p_.j2_offset) / p_.j2_sign;
  const double q3 = (q.j3 - p_.j3_offset) / p_.j3_sign;
  const double q4 = (q.j4 - p_.j4_offset) / p_.j4_sign;
  const double a2 = q2, a3 = q2 + q3, a4 = q2 + q3 + q4;
  const double r = p_.upper_arm_m * std::cos(a2) + p_.forearm_m * std::cos(a3) + p_.hand_m * std::cos(a4);
  const double z = p_.base_height_m + p_.upper_arm_m * std::sin(a2) + p_.forearm_m * std::sin(a3) +
                   p_.hand_m * std::sin(a4);
  Pose p;
  p.x = r * std::cos(yaw);
  p.y = r * std::sin(yaw);
  p.z = z;
  p.pitch = a4;
  return p;
}

bool inside(const Workspace& w, const Pose& p) {
  return p.x >= w.x_min && p.x <= w.x_max && p.y >= w.y_min && p.y <= w.y_max && p.z >= w.z_min &&
         p.z <= w.z_max;
}

const char* to_string(IkStatus s) {
  switch (s) {
    case IkStatus::kOk: return "ok";
    case IkStatus::kOutOfReach: return "out_of_reach";
    case IkStatus::kJointLimit: return "joint_limit";
    case IkStatus::kInvalid: return "invalid";
  }
  return "unknown";
}

}  // namespace armpi_kinematics
