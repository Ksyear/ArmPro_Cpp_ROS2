// 팔 목표 보정(TargetProjector): 닿을 수 없는 손끝 목표를 "닿을 수 있는 가장 가까운 자세"로 바꾼다.
//
// 안전 범위는 바꾸지 않는다. 결과는 항상 작업 공간 상자 안이고 IK 해가 있다
// (관절 범위·손목 높이·팔꿈치 굽힘 제한을 모두 통과). 바뀌는 것은 "거부하고 멈춤" → "가까운 곳까지 따라감" 뿐이다.
//
// 순서
//   1) 위치를 상자 안으로 자른다(kAdjustBox).
//   2) 요청 pitch 로 IK 가 안 되면 요청에 가까운 pitch 부터 찾는다(kAdjustPitch).
//      찾는 범위: 요청 ± max_pitch_change_rad 이면서 [pitch_min_rad, pitch_max_rad] 안.
//      같은 거리면 기준 자세(anchor) pitch 에 가까운 쪽을 먼저 본다(자세가 튀지 않게).
//   3) 그래도 없으면 anchor → 목표 직선 위에서 닿을 수 있는 가장 먼 점(이분 탐색, kAdjustPosition).
//      anchor 가 없거나 anchor 자체가 닿을 수 없으면 실패(ok=false).
//
// 쓰는 곳: armpi_vr_bridge(받은 arm_command), armpi_web_gateway(버튼으로 움직이는 목표), armpi_control(보간 경로).
// 같은 매개변수(armpi.yaml 의 arm.target_policy·arm.nearest.*)를 쓰므로 어느 조종기든 보정 규칙이 같다.
#pragma once

#include <string>

#include "armpi_kinematics/arm_kinematics.hpp"

namespace armpi_kinematics {

// kReject: 규격 v3 의 원래 동작(작업 공간 밖 → workspace_limit, IK 해 없음 → ik_unreachable).
// kNearest: 위 1)–3) 으로 보정해서 받는다.
enum class TargetPolicy { kReject, kNearest };
bool parse_target_policy(const std::string& text, TargetPolicy* out);
const char* to_string(TargetPolicy p);

struct ReachOptions {
  double max_pitch_change_rad{1.5707963267948966};  // 요청 pitch 에서 최대 이만큼 바꾼다(90°). 0 = pitch 를 바꾸지 않음
  double pitch_step_rad{0.017453292519943295};      // 찾는 간격 1°
  // 바꾼 pitch 가 들어가야 하는 범위. 요청 pitch 자체는 이 범위 밖이어도 그대로 먼저 시도한다.
  double pitch_min_rad{-1.5707963267948966};  // −90°(손끝이 바로 아래)
  double pitch_max_rad{0.5235987755982988};   // +30°
  bool allow_position_change{true};           // 3) 단계 사용 여부
  int bisect_iterations{14};                  // 3) 단계 반복 수(0.5 m 구간이면 약 0.03 mm 정밀도)

  // 잘못된 값이면 이유, 맞으면 빈 문자열
  std::string validate() const;
};

enum ReachAdjust : unsigned {
  kAdjustNone = 0u,
  kAdjustBox = 1u,       // 상자 밖 위치를 상자 면으로
  kAdjustPitch = 2u,     // pitch 를 바꿈
  kAdjustPosition = 4u,  // anchor 쪽으로 위치를 당김
};
// "box+pitch" 처럼 '+' 로 잇는다. 0 이면 "none"
std::string adjust_to_string(unsigned flags);

struct ReachResult {
  bool ok{false};
  Pose pose{};
  unsigned adjust{kAdjustNone};
  IkStatus requested_status{IkStatus::kInvalid};  // 상자로 자른 위치·요청 pitch 그대로의 IK 결과
};

Pose clamp_to(const Workspace& w, const Pose& p);

class TargetProjector {
 public:
  TargetProjector(const Params& kinematics, const Workspace& workspace, const ReachOptions& options);

  // anchor: 닿을 수 있다고 알려진 자세(마지막으로 받은 목표나 현재 자세). 없으면 nullptr.
  ReachResult project(const Pose& target, const Pose* anchor) const;

  // 상자 안이고 IK 해가 있는지
  bool reachable(const Pose& p) const;

  const ArmKinematics& kinematics() const { return kin_; }
  const Workspace& workspace() const { return ws_; }
  const ReachOptions& options() const { return opt_; }

 private:
  // 위치 (x, y, z) 에서 want 에 가장 가까운, IK 해가 있는 pitch.
  // prefer: 같은 거리일 때 먼저 볼 쪽. extra: 마지막으로 시도할 pitch(보통 anchor pitch). 둘 다 nullptr 가능.
  bool find_pitch(double x, double y, double z, double want, const double* prefer, const double* extra,
                  double* out) const;
  bool ik_ok(double x, double y, double z, double pitch) const;

  ArmKinematics kin_;
  Workspace ws_;
  ReachOptions opt_;
};

}  // namespace armpi_kinematics
