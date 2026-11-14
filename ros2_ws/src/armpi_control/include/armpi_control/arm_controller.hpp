// 팔 제어(ROS 비의존): ArmTarget → 손끝 보간(속도 제한) → IK(armpi_kinematics) → 관절 속도 제한
// → 서보값 → MOVE_TIME_WRITE. 기본 자세(home), 관절 직접 지정(joint_debug), 위치 피드백(선택).
//
// 안전 규칙 (docs/02_ROS2_INTERFACES.md 제어기 2차 방어, docs/01_PROTOCOL_V3_SPEC.md 상태 규칙):
//   - ArmTarget 이 timeout(0.6 s) 동안 없음 / active:false / 비상정지 → 현재 명령 자세 유지. 새 명령을 보내지 않는다.
//   - 토크 해제(LOAD_OR_UNLOAD 0)는 절대 보내지 않는다(팔이 떨어짐).
//   - IK 실패 → 직전 유효 자세 유지 + 경고. (arm.target_policy: nearest 이면 보간 경로에서만 가까운 자세로 보정)
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "armpi_hardware/conversions.hpp"
#include "armpi_hardware/devices.hpp"
#include "armpi_kinematics/arm_kinematics.hpp"
#include "armpi_kinematics/reachability.hpp"

namespace armpi_control {

struct ArmPose {
  double x{0.0}, y{0.0}, z{0.0}, pitch{0.0};  // armpi_arm_base, m·rad
  double gripper{0.0};                        // 0 열림 .. 1 닫힘
};
bool same_pose(const ArmPose& a, const ArmPose& b);

struct ArmRateLimits {
  // 라이브러리 기본값(보수값, 단위 시험 기준). 로봇에서는 노드 매개변수(armpi.yaml)가 덮어쓴다
  double max_linear_speed_m_s{0.10};
  double max_pitch_rate_rad_s{1.0};
  double max_gripper_rate_per_s{1.0};
};

// 손끝 위치(직선)·pitch·그리퍼를 각각 속도 제한으로 한 걸음 보간. 남은 거리가 한 걸음보다 짧으면 목표 그대로.
ArmPose interpolate_step(const ArmPose& cur, const ArmPose& goal, const ArmRateLimits& lim, double dt);

using Joints5 = std::array<double, 5>;  // joint1..joint5 (rad)
// 관절별로 max_step 이하로 이동. max_step ≤ 0 이면 제한 없음.
Joints5 step_joints(const Joints5& cur, const Joints5& target, double max_step);

struct ArmConfig {
  armpi_kinematics::Params kinematics{};
  armpi_kinematics::Workspace workspace{};
  ArmRateLimits limits{};
  double max_joint_speed_rad_s{2.0};  // IK 뒤 관절 속도 제한(특이점 근처 급회전 방지). 0 이하 = 끔
  // 보간 경로 중간에서 IK 가 막혔을 때. kReject = 직전 유효 자세 유지(원래 동작, 라이브러리 기본값),
  // kNearest = 같은 보정 규칙(armpi_kinematics/reachability.hpp)으로 가까운 자세를 찾아 계속 간다. armpi.yaml 의 arm.target_policy.
  armpi_kinematics::TargetPolicy target_policy{armpi_kinematics::TargetPolicy::kReject};
  armpi_kinematics::ReachOptions reach{};
  double target_timeout_s{0.6};
  int command_duration_ms{40};
  // 아래 네 묶음은 하드웨어 프로필 값(코드 기본값 없음, armpi_hardware/hardware_profile.hpp)
  armpi_hardware::JointServoMap servo_map{};  // 관절 ↔ 서보 ID·방향
  std::array<int, 6> home_servo_values{-1, -1, -1, -1, -1, -1};  // 서보 ID1..6, 켤 때 보낼 값
  armpi_hardware::GripperParams gripper{};
  int gripper_servo_min{-1};    // 그리퍼 서보 허용 범위
  int gripper_servo_max{-1};
  int home_duration_ms{1500};
  int joint5_servo_value{500};  // 손목 회전 고정값(서보 중앙)
  double joint_debug_speed_rad_s{1.0};
  int joint_debug_min_duration_ms{200};
  int joint_debug_max_duration_ms{3000};
  int fault_threshold{5};       // 연속 쓰기 실패 횟수
  bool read_position{false};    // POS_READ 실측(실기 미검증 → 기본 끔)
  double read_rate_hz{5.0};
};

enum class ArmMode { kUninitialized, kTimedMove, kHold, kTracking };
const char* to_string(ArmMode m);

enum class TargetResult {
  kAccepted,
  kHold,                  // active:false → 현재 위치 유지
  kRejectedInvalid,       // 유한값 아님
  kRejectedWorkspace,     // 작업 공간 상자 밖
  kRejectedIk,            // IK 해 없음
  kIgnoredInhibit,        // 비상정지·fault
  kIgnoredUninitialized,  // 아직 기본 자세로 보내지 않음(위치 모름)
  kIgnoredBusy,           // home·joint_debug 시간 이동 중
};
const char* to_string(TargetResult r);

struct ArmEvent {
  enum Level { kInfo, kWarn, kError } level;
  std::string text;
};

class ArmController {
 public:
  ArmController(const ArmConfig& cfg, std::shared_ptr<armpi_hardware::ServoBus> bus);

  // 기구학 설정·기본 자세 FK/IK 일관성 검사
  bool validate(std::string* why) const;

  // 기본 자세(서보 ID1..6 = home_servo_values, home_duration_ms)로 보낸다. 보간 상태를 그 자세의 FK 로 초기화.
  bool home(double now, std::string* msg);
  TargetResult on_target(bool active, const ArmPose& target, double now, bool inhibit);
  // 관절 직접 지정(이름 joint1..joint5, r_joint; rad). 시간은 가장 큰 변화 / joint_debug_speed.
  bool joint_debug(const std::vector<std::string>& names, const std::vector<double>& positions, double now,
                   std::string* msg);
  // 현재 명령 자세에서 멈춤(새 명령 안 보냄, 토크 유지)
  void hold();
  // 50 Hz
  void tick(double now, double dt, bool inhibit);
  // read_position 일 때 한 번에 서보 1개씩 POS_READ
  void poll_feedback(double now);

  ArmMode mode() const { return mode_; }
  bool initialized() const { return initialized_; }
  bool busy(double now) const { return now < timed_until_; }
  const std::string& timed_kind() const { return timed_kind_; }
  bool channel_active(double now) const;
  bool moving(double now) const;
  ArmPose goal() const { return goal_; }
  ArmPose reported_pose() const;
  // 인덱스 = 서보 ID(1..6), 0 은 안 씀. 모르면 −1
  std::array<int, 7> commanded_servo() const { return cmd_servo_; }
  std::array<int, 7> reported_servo() const;
  std::string position_source() const;
  // joint1..joint5, r_joint (rad)
  std::array<double, 6> reported_joint_positions() const;
  bool ik_blocked() const { return ik_blocked_; }
  bool fault() const { return fault_; }
  const std::string& fault_reason() const { return fault_reason_; }
  uint64_t servo_writes() const { return writes_; }
  uint64_t servo_write_failures() const { return write_failures_; }
  uint64_t feedback_failures() const { return read_failures_; }
  std::vector<ArmEvent> take_events();
  const ArmConfig& config() const { return cfg_; }
  armpi_kinematics::Pose forward(const Joints5& j) const;

 private:
  // 바뀐 서보만 MOVE_TIME_WRITE. 실패가 있으면 false.
  bool send_servos(const std::array<int, 7>& want, int duration_ms, std::string* err);
  std::array<int, 7> servo_values_for(const Joints5& j, double gripper) const;
  void note_result(bool ok, const std::string& err);
  void event(ArmEvent::Level l, std::string t) { events_.push_back({l, std::move(t)}); }
  double joint5_rad() const;

  ArmConfig cfg_;
  std::shared_ptr<armpi_hardware::ServoBus> bus_;
  armpi_kinematics::ArmKinematics kin_;
  armpi_kinematics::TargetProjector projector_;
  bool map_ok_{false};

  ArmMode mode_{ArmMode::kUninitialized};
  bool initialized_{false};
  double timed_until_{-1.0};
  std::string timed_kind_;
  ArmPose current_{};  // 보간 상태(손끝 목표 궤적)
  ArmPose goal_{};
  Joints5 desired_joints_{};
  Joints5 cmd_joints_{};
  std::array<int, 7> cmd_servo_{-1, -1, -1, -1, -1, -1, -1};
  std::array<int, 7> last_sent_{-1, -1, -1, -1, -1, -1, -1};
  double last_feed_{-1e18};
  bool last_active_{false};
  bool ik_blocked_{false};
  bool path_adjusting_{false};  // 보간 경로를 보정하는 중(로그를 바뀔 때만 남긴다)

  int consecutive_failures_{0};
  bool fault_{false};
  std::string fault_reason_;
  uint64_t writes_{0};
  uint64_t write_failures_{0};

  std::array<int, 7> measured_{-1, -1, -1, -1, -1, -1, -1};
  std::array<double, 7> measured_time_{-1e18, -1e18, -1e18, -1e18, -1e18, -1e18, -1e18};  // 마지막 성공
  std::array<double, 7> attempt_time_{-1e18, -1e18, -1e18, -1e18, -1e18, -1e18, -1e18};   // 마지막 시도
  double last_feedback_now_{-1e18};
  uint64_t read_failures_{0};

  std::vector<ArmEvent> events_;
};

}  // namespace armpi_control
