// ArmPi Pro 팔 기구학 (IK/FK). 좌표계: armpi_arm_base, REP-103 (x 앞, y 왼쪽, z 위), 단위 m·rad.
// 관절: joint1 베이스 회전, joint2 어깨, joint3 팔꿈치, joint4 손목 pitch. (joint5 손목 roll·그리퍼는 IK 대상 아님)
// 링크 길이는 armpi_bringup config/armpi.yaml(Hiwonder 공개 ArmPi FPV 값, ArmPi Pro 실측 확인 필요),
// 관절 기준(sign/offset)과 관절 범위는 하드웨어 프로필(hardware_profile.yaml)에서 넣는다.
#pragma once

namespace armpi_kinematics {

struct Params {
  double base_height_m{0.0};  // armpi_arm_base 원점 -> 어깨 관절축 높이
  double upper_arm_m{0.0};    // 어깨 -> 팔꿈치
  double forearm_m{0.0};      // 팔꿈치 -> 손목
  double hand_m{0.0};         // 손목 -> 손끝(그리퍼 끝점)
  // 기하 각도 -> 관절 각도: joint = sign * geometric + offset (rad)
  double j1_sign{1.0}, j1_offset{0.0};
  double j2_sign{1.0}, j2_offset{0.0};
  double j3_sign{1.0}, j3_offset{0.0};
  double j4_sign{1.0}, j4_offset{0.0};
  // 손목 중심(손목 관절축)의 최소 높이(armpi_arm_base z 기준, m). 그보다 낮으면 거부한다.
  // armpi.yaml 은 0 을 쓴다. 이 구조체 기본값은 제한 없음.
  double min_wrist_height_m{-1e9};
  // 팔꿈치 최소 굽힘(rad). 팔이 일자로 펴진 특이 자세 근처를 거부한다. 기본값 0(제한 없음).
  double min_elbow_bend_rad{0.0};
  double joint_min[4]{-3.2, -3.2, -3.2, -3.2};
  double joint_max[4]{3.2, 3.2, 3.2, 3.2};
};

struct Pose {
  double x{0.0}, y{0.0}, z{0.0}, pitch{0.0};
};

struct JointAngles {
  double j1{0.0}, j2{0.0}, j3{0.0}, j4{0.0};
};

enum class IkStatus { kOk, kOutOfReach, kJointLimit, kInvalid };

struct IkResult {
  IkStatus status{IkStatus::kInvalid};
  JointAngles joints{};
};

class ArmKinematics {
 public:
  explicit ArmKinematics(const Params& p);
  // 팔꿈치 위(elbow-up) 해. 해가 없거나 관절 한계를 넘으면 status != kOk.
  IkResult inverse(const Pose& target) const;
  Pose forward(const JointAngles& q) const;
  const Params& params() const { return p_; }
  // 링크 길이가 모두 양수이고 한계가 min<max 인지
  bool valid() const;

 private:
  Params p_;
};

struct Workspace {
  double x_min{0.0}, x_max{0.0}, y_min{0.0}, y_max{0.0}, z_min{0.0}, z_max{0.0};
};

bool inside(const Workspace& w, const Pose& p);

const char* to_string(IkStatus s);

}  // namespace armpi_kinematics
