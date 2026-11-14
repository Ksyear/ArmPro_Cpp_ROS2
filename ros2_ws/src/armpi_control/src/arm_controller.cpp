#include "armpi_control/arm_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace armpi_control {

namespace ah = armpi_hardware;
namespace ak = armpi_kinematics;

namespace {
std::string fmt_pose(const ArmPose& p) {
  char b[160];
  std::snprintf(b, sizeof(b), "x=%.3f y=%.3f z=%.3f pitch=%.3f g=%.2f", p.x, p.y, p.z, p.pitch, p.gripper);
  return b;
}
bool finite_pose(const ArmPose& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(p.pitch) &&
         std::isfinite(p.gripper);
}
}  // namespace

bool same_pose(const ArmPose& a, const ArmPose& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z && a.pitch == b.pitch && a.gripper == b.gripper;
}

ArmPose interpolate_step(const ArmPose& cur, const ArmPose& goal, const ArmRateLimits& lim, double dt) {
  dt = std::max(0.0, std::min(dt, 0.2));
  ArmPose out = cur;
  // 위치: 직선, 거리 제한
  const double dx = goal.x - cur.x, dy = goal.y - cur.y, dz = goal.z - cur.z;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double max_d = std::max(0.0, lim.max_linear_speed_m_s) * dt;
  if (dist <= max_d) {
    out.x = goal.x;
    out.y = goal.y;
    out.z = goal.z;
  } else if (dist > 0.0) {
    const double k = max_d / dist;
    out.x = cur.x + dx * k;
    out.y = cur.y + dy * k;
    out.z = cur.z + dz * k;
  }
  auto step1 = [](double c, double g, double m) {
    const double d = g - c;
    if (std::fabs(d) <= m) return g;
    return c + std::copysign(m, d);
  };
  out.pitch = step1(cur.pitch, goal.pitch, std::max(0.0, lim.max_pitch_rate_rad_s) * dt);
  out.gripper = step1(cur.gripper, goal.gripper, std::max(0.0, lim.max_gripper_rate_per_s) * dt);
  return out;
}

Joints5 step_joints(const Joints5& cur, const Joints5& target, double max_step) {
  if (!(max_step > 0.0)) return target;
  Joints5 out = cur;
  for (size_t i = 0; i < out.size(); ++i) {
    const double d = target[i] - cur[i];
    out[i] = std::fabs(d) <= max_step ? target[i] : cur[i] + std::copysign(max_step, d);
  }
  return out;
}

const char* to_string(ArmMode m) {
  switch (m) {
    case ArmMode::kUninitialized: return "uninitialized";
    case ArmMode::kTimedMove: return "timed_move";
    case ArmMode::kHold: return "hold";
    case ArmMode::kTracking: return "tracking";
  }
  return "unknown";
}

const char* to_string(TargetResult r) {
  switch (r) {
    case TargetResult::kAccepted: return "accepted";
    case TargetResult::kHold: return "hold";
    case TargetResult::kRejectedInvalid: return "invalid";
    case TargetResult::kRejectedWorkspace: return "workspace_limit";
    case TargetResult::kRejectedIk: return "ik_unreachable";
    case TargetResult::kIgnoredInhibit: return "inhibited";
    case TargetResult::kIgnoredUninitialized: return "uninitialized";
    case TargetResult::kIgnoredBusy: return "busy";
  }
  return "unknown";
}

ArmController::ArmController(const ArmConfig& cfg, std::shared_ptr<ah::ServoBus> bus)
    : cfg_(cfg),
      bus_(std::move(bus)),
      kin_(cfg.kinematics),
      projector_(cfg.kinematics, cfg.workspace, cfg.reach),
      map_ok_(cfg.servo_map.valid()) {}

double ArmController::joint5_rad() const { return ah::servo_to_joint(cfg_.servo_map, 4, cfg_.joint5_servo_value); }

ak::Pose ArmController::forward(const Joints5& j) const {
  return kin_.forward(ak::JointAngles{j[0], j[1], j[2], j[3]});
}

bool ArmController::validate(std::string* why) const {
  if (!cfg_.servo_map.valid(why)) return false;
  if (!cfg_.gripper.valid(why)) return false;
  if (!kin_.valid()) {
    if (why) *why = "기구학 설정 오류(링크 길이·관절 한계·sign)";
    return false;
  }
  for (int v : cfg_.home_servo_values) {
    if (v < ah::kServoMin || v > ah::kServoMax) {
      if (why) *why = "home_servo_values 범위 밖(0..1000)";
      return false;
    }
  }
  Joints5 j{};
  for (int i = 0; i < 5; ++i) j[i] = ah::servo_to_joint(cfg_.servo_map, i, cfg_.home_servo_values[cfg_.servo_map.id(i) - 1]);
  const ak::Pose p = forward(j);
  const auto r = kin_.inverse(p);
  if (r.status != ak::IkStatus::kOk) {
    if (why) *why = std::string("기본 자세의 FK 를 IK 가 풀지 못함: ") + ak::to_string(r.status);
    return false;
  }
  if (!ak::inside(cfg_.workspace, p)) {
    char b[200];
    std::snprintf(b, sizeof(b), "기본 자세 손끝(x=%.3f y=%.3f z=%.3f)이 작업 공간 밖", p.x, p.y, p.z);
    if (why) *why = b;
    return false;
  }
  if (cfg_.gripper_servo_min < ah::kServoMin || cfg_.gripper_servo_max > ah::kServoMax ||
      cfg_.gripper_servo_min > cfg_.gripper_servo_max) {
    if (why) *why = "gripper_servo_min/max 가 비어 있거나 범위 밖";
    return false;
  }
  return true;
}

std::array<int, 7> ArmController::servo_values_for(const Joints5& j, double gripper) const {
  if (!map_ok_) return std::array<int, 7>{-1, -1, -1, -1, -1, -1, -1};  // 서보 배정(하드웨어 프로필)이 없으면 아무것도 보내지 않는다
  std::array<int, 7> want{-1, -1, -1, -1, -1, -1, -1};
  for (int i = 0; i < 5; ++i) want[cfg_.servo_map.id(i)] = ah::joint_to_servo(cfg_.servo_map, i, j[i]);
  want[cfg_.servo_map.gripper_id()] =
      ah::clamp_servo(ah::gripper_to_servo(gripper, cfg_.gripper), cfg_.gripper_servo_min, cfg_.gripper_servo_max);
  return want;
}

void ArmController::note_result(bool ok, const std::string& err) {
  if (ok) {
    consecutive_failures_ = 0;
    return;
  }
  ++consecutive_failures_;
  if (!fault_ && consecutive_failures_ >= std::max(1, cfg_.fault_threshold)) {
    fault_ = true;
    fault_reason_ = "서보 버스 쓰기 연속 실패 " + std::to_string(consecutive_failures_) + "회: " + err;
    event(ArmEvent::kError, "fault: " + fault_reason_);
  }
}

bool ArmController::send_servos(const std::array<int, 7>& want, int duration_ms, std::string* err) {
  bool all_ok = true;
  for (int id = 1; id <= 6; ++id) {
    if (want[id] < 0) continue;
    cmd_servo_[id] = want[id];
    if (want[id] == last_sent_[id]) continue;
    std::string e;
    if (bus_ && bus_->move(static_cast<uint8_t>(id), want[id], duration_ms, &e)) {
      last_sent_[id] = want[id];
      ++writes_;
    } else {
      last_sent_[id] = -1;  // 다음 틱에 다시 보냄
      ++write_failures_;
      all_ok = false;
      if (err && err->empty()) *err = bus_ ? e : "서보 버스 없음";
    }
  }
  return all_ok;
}

bool ArmController::home(double now, std::string* msg) {
  if (!map_ok_) { if (msg) *msg = "서보 배정(하드웨어 프로필) 없음"; return false; }  // 서보 배정(하드웨어 프로필)이 없으면 아무것도 보내지 않는다
  if (fault_) {
    if (msg) *msg = "fault 상태: " + fault_reason_;
    return false;
  }
  std::array<int, 7> want{-1, -1, -1, -1, -1, -1, -1};
  for (int id = 1; id <= 6; ++id) want[id] = ah::clamp_servo(cfg_.home_servo_values[id - 1]);
  // home 은 항상 6개 모두 보낸다(현재 위치를 모르는 시작 시점 포함)
  last_sent_.fill(-1);
  std::string err;
  const bool ok = send_servos(want, std::max(0, cfg_.home_duration_ms), &err);
  note_result(ok, err);
  for (int i = 0; i < 5; ++i) cmd_joints_[i] = ah::servo_to_joint(cfg_.servo_map, i, want[cfg_.servo_map.id(i)]);
  desired_joints_ = cmd_joints_;
  const ak::Pose p = forward(cmd_joints_);
  current_ = ArmPose{p.x, p.y, p.z, p.pitch, ah::servo_to_gripper(want[cfg_.servo_map.gripper_id()], cfg_.gripper)};
  goal_ = current_;
  timed_until_ = now + std::max(0, cfg_.home_duration_ms) / 1000.0;
  timed_kind_ = "homing";
  mode_ = ArmMode::kTimedMove;
  initialized_ = true;
  ik_blocked_ = false;
  if (msg) *msg = ok ? "기본 자세로 이동 (" + fmt_pose(current_) + ", " + std::to_string(cfg_.home_duration_ms) + " ms)"
                     : "기본 자세 명령 일부 실패: " + err;
  event(ok ? ArmEvent::kInfo : ArmEvent::kWarn, ok ? "home: " + fmt_pose(current_) : "home 실패: " + err);
  return ok;
}

TargetResult ArmController::on_target(bool active, const ArmPose& target_in, double now, bool inhibit) {
  if (!map_ok_) return TargetResult::kIgnoredInhibit;  // 서보 배정(하드웨어 프로필)이 없으면 아무것도 보내지 않는다
  if (!active) {
    last_feed_ = now;
    last_active_ = false;
    if (mode_ == ArmMode::kTracking) hold();
    return TargetResult::kHold;
  }
  ArmPose t = target_in;
  if (!finite_pose(t)) return TargetResult::kRejectedInvalid;
  t.gripper = std::max(0.0, std::min(1.0, t.gripper));
  const ak::Pose kp{t.x, t.y, t.z, t.pitch};
  if (!ak::inside(cfg_.workspace, kp)) return TargetResult::kRejectedWorkspace;
  if (kin_.inverse(kp).status != ak::IkStatus::kOk) return TargetResult::kRejectedIk;
  if (fault_ || inhibit) return TargetResult::kIgnoredInhibit;
  if (!initialized_) return TargetResult::kIgnoredUninitialized;
  if (busy(now)) return TargetResult::kIgnoredBusy;
  last_feed_ = now;
  last_active_ = true;
  goal_ = t;
  if (mode_ != ArmMode::kTracking) mode_ = ArmMode::kTracking;
  return TargetResult::kAccepted;
}

void ArmController::hold() {
  if (!initialized_) return;
  if (mode_ == ArmMode::kTracking) mode_ = ArmMode::kHold;
  desired_joints_ = cmd_joints_;
  const ak::Pose p = forward(cmd_joints_);
  current_ = ArmPose{p.x, p.y, p.z, p.pitch, current_.gripper};
  goal_ = current_;
}

bool ArmController::channel_active(double now) const {
  return last_active_ && now - last_feed_ <= cfg_.target_timeout_s;
}

bool ArmController::moving(double now) const {
  if (busy(now)) return true;
  if (mode_ != ArmMode::kTracking) return false;
  return !same_pose(current_, goal_) || cmd_joints_ != desired_joints_;
}

void ArmController::tick(double now, double dt, bool inhibit) {
  if (!map_ok_) return;  // 서보 배정(하드웨어 프로필)이 없으면 아무것도 보내지 않는다
  if (fault_ || !initialized_) return;
  if (mode_ == ArmMode::kTimedMove && !busy(now)) {
    mode_ = ArmMode::kHold;
    event(ArmEvent::kInfo, timed_kind_ + " 완료");
  }
  if (inhibit) {
    if (mode_ == ArmMode::kTracking) hold();
    return;
  }
  if (mode_ != ArmMode::kTracking) return;
  if (now - last_feed_ > cfg_.target_timeout_s) {
    hold();
    last_active_ = false;
    char b[96];
    std::snprintf(b, sizeof(b), "ArmTarget 이 %.2f s 동안 없음 → 현재 위치 유지", cfg_.target_timeout_s);
    event(ArmEvent::kWarn, b);
    return;
  }
  const ArmPose cand = interpolate_step(current_, goal_, cfg_.limits, dt);
  if (!same_pose(cand, current_)) {
    const auto r = kin_.inverse(ak::Pose{cand.x, cand.y, cand.z, cand.pitch});
    // 경로가 막혔을 때 nearest 정책이면 같은 보정 규칙으로 가까운 자세를 찾는다(안전 범위는 그대로).
    // 기준은 지금 보간 자세(항상 닿음). 보정한 자세가 목표까지 남은 거리를 줄일 때만 받는다
    // (같은 자리에서 pitch 만 오가며 떨지 않게). 줄지 않으면 막힌 것으로 보고 지금 자세를 유지한다.
    bool adjusted = false;
    ak::ReachResult pr;
    if (r.status != ak::IkStatus::kOk && cfg_.target_policy == ak::TargetPolicy::kNearest) {
      const ak::Pose anchor{current_.x, current_.y, current_.z, current_.pitch};
      pr = projector_.project(ak::Pose{cand.x, cand.y, cand.z, cand.pitch}, &anchor);
      if (pr.ok) {
        // 남은 거리 = 위치 거리 + w × pitch 차이. w 는 두 속도 상한의 비(1 rad 의 pitch 를 m 로 환산)
        const double w = cfg_.limits.max_pitch_rate_rad_s > 0.0
                             ? cfg_.limits.max_linear_speed_m_s / cfg_.limits.max_pitch_rate_rad_s
                             : 0.1;
        auto remaining = [&](double x, double y, double z, double pitch) {
          return std::sqrt((x - goal_.x) * (x - goal_.x) + (y - goal_.y) * (y - goal_.y) +
                           (z - goal_.z) * (z - goal_.z)) +
                 w * std::fabs(pitch - goal_.pitch);
        };
        adjusted = remaining(pr.pose.x, pr.pose.y, pr.pose.z, pr.pose.pitch) <
                   remaining(current_.x, current_.y, current_.z, current_.pitch) - 1e-6;
      }
    }
    if (r.status == ak::IkStatus::kOk || adjusted) {
      if (adjusted) {
        const auto r2 = kin_.inverse(pr.pose);  // project() 결과는 항상 IK 해가 있다
        current_ = ArmPose{pr.pose.x, pr.pose.y, pr.pose.z, pr.pose.pitch, cand.gripper};
        desired_joints_ = {r2.joints.j1, r2.joints.j2, r2.joints.j3, r2.joints.j4, joint5_rad()};
        if (!path_adjusting_) {
          path_adjusting_ = true;
          event(ArmEvent::kInfo, "보간 경로 보정(" + ak::adjust_to_string(pr.adjust) + "): " + fmt_pose(current_));
        }
      } else {
        current_ = cand;
        desired_joints_ = {r.joints.j1, r.joints.j2, r.joints.j3, r.joints.j4, joint5_rad()};
        path_adjusting_ = false;
      }
      if (ik_blocked_) {
        ik_blocked_ = false;
        event(ArmEvent::kInfo, "IK 다시 풀림: " + fmt_pose(current_));
      }
    } else {
      current_.gripper = cand.gripper;  // 그리퍼는 IK 와 무관하게 진행
      if (!ik_blocked_) {
        ik_blocked_ = true;
        event(ArmEvent::kWarn, std::string("보간 중 IK 실패(") + ak::to_string(r.status) + ", " + fmt_pose(cand) +
                                   ") → 직전 유효 자세 유지");
      }
    }
  } else if (desired_joints_[4] != joint5_rad()) {
    desired_joints_[4] = joint5_rad();
  }
  cmd_joints_ = step_joints(cmd_joints_, desired_joints_, cfg_.max_joint_speed_rad_s * std::max(0.0, dt));
  std::string err;
  const bool ok = send_servos(servo_values_for(cmd_joints_, current_.gripper), cfg_.command_duration_ms, &err);
  note_result(ok, err);
}

bool ArmController::joint_debug(const std::vector<std::string>& names, const std::vector<double>& positions,
                                double now, std::string* msg) {
  if (!map_ok_) { if (msg) *msg = "서보 배정(하드웨어 프로필) 없음"; return false; }  // 서보 배정(하드웨어 프로필)이 없으면 아무것도 보내지 않는다
  auto fail = [msg](const std::string& m) {
    if (msg) *msg = m;
    return false;
  };
  if (fault_) return fail("fault 상태: " + fault_reason_);
  if (!initialized_) return fail("팔 초기화 전(먼저 /armpi/arm/home)");
  if (names.size() != positions.size()) return fail("name 과 position 개수가 다름");
  Joints5 j = cmd_joints_;
  const int gid = cfg_.servo_map.gripper_id();
  int grip = cmd_servo_[gid] >= 0 ? cmd_servo_[gid] : cfg_.home_servo_values[gid - 1];
  bool any = false;
  const double j5_lim = 120.0 * M_PI / 180.0;
  for (size_t k = 0; k < names.size(); ++k) {
    const double p = positions[k];
    if (!std::isfinite(p)) return fail("유한값이 아닌 관절값: " + names[k]);
    int idx = -1;
    for (int i = 0; i < ah::kNumJoints; ++i) {
      if (names[k] == ah::kJointNames[i]) idx = i;
    }
    if (idx < 0) return fail("알 수 없는 관절 이름: " + names[k]);
    if (idx < 4) {
      j[idx] = std::max(cfg_.kinematics.joint_min[idx], std::min(cfg_.kinematics.joint_max[idx], p));
    } else if (idx == 4) {
      j[4] = std::max(-j5_lim, std::min(j5_lim, p));
    } else {
      grip = ah::clamp_servo(ah::joint_to_servo(cfg_.servo_map, ah::kGripperIndex, p), cfg_.gripper_servo_min, cfg_.gripper_servo_max);
    }
    any = true;
  }
  if (!any) return fail("관절 값이 없음");
  std::array<int, 7> want{-1, -1, -1, -1, -1, -1, -1};
  for (int i = 0; i < 5; ++i) want[cfg_.servo_map.id(i)] = ah::joint_to_servo(cfg_.servo_map, i, j[i]);
  want[1] = grip;
  // 시간 = 가장 큰 관절 변화 / 속도 (그리퍼는 서보 각도 변화로 계산)
  double max_delta = 0.0;
  for (int i = 0; i < 5; ++i) max_delta = std::max(max_delta, std::fabs(j[i] - cmd_joints_[i]));
  if (cmd_servo_[1] >= 0) max_delta = std::max(max_delta, std::fabs(grip - cmd_servo_[1]) * ah::kServoDegPerUnit * M_PI / 180.0);
  const double speed = cfg_.joint_debug_speed_rad_s > 0.0 ? cfg_.joint_debug_speed_rad_s : 1.0;
  int dur = static_cast<int>(std::lround(max_delta / speed * 1000.0));
  dur = std::max(cfg_.joint_debug_min_duration_ms, std::min(cfg_.joint_debug_max_duration_ms, dur));
  std::string err;
  const bool ok = send_servos(want, dur, &err);
  note_result(ok, err);
  cmd_joints_ = j;
  desired_joints_ = j;
  const ak::Pose p = forward(j);
  current_ = ArmPose{p.x, p.y, p.z, p.pitch, ah::servo_to_gripper(grip, cfg_.gripper)};
  goal_ = current_;
  timed_until_ = now + dur / 1000.0;
  timed_kind_ = "joint_debug";
  mode_ = ArmMode::kTimedMove;
  ik_blocked_ = false;
  if (!ok) return fail("서보 쓰기 실패: " + err);
  if (msg) *msg = "관절 직접 지정(" + std::to_string(dur) + " ms): " + fmt_pose(current_);
  return true;
}

void ArmController::poll_feedback(double now) {
  if (!map_ok_) return;  // 서보 배정(하드웨어 프로필)이 없으면 아무것도 보내지 않는다
  if (!cfg_.read_position || !bus_ || cfg_.read_rate_hz <= 0.0) return;
  const double period = 1.0 / cfg_.read_rate_hz;
  int pick = -1;
  double oldest = 1e300;
  for (int id = 1; id <= 6; ++id) {
    if (now - attempt_time_[id] >= period && attempt_time_[id] < oldest) {
      oldest = attempt_time_[id];
      pick = id;
    }
  }
  last_feedback_now_ = now;
  if (pick < 0) return;
  attempt_time_[pick] = now;  // 한 틱에 서보 1개, 서보마다 period 간격
  std::string err;
  const auto v = bus_->read_position(static_cast<uint8_t>(pick), &err);
  if (v) {
    measured_[pick] = *v;
    measured_time_[pick] = now;
  } else {
    ++read_failures_;
  }
}

std::string ArmController::position_source() const {
  if (!cfg_.read_position || cfg_.read_rate_hz <= 0.0) return "commanded";
  const double max_age = std::max(1.0, 3.0 / cfg_.read_rate_hz);
  for (int id = 1; id <= 6; ++id) {
    if (measured_[id] < 0 || last_feedback_now_ - measured_time_[id] > max_age) return "commanded";
  }
  return "measured";
}

std::array<int, 7> ArmController::reported_servo() const {
  return position_source() == "measured" ? measured_ : cmd_servo_;
}

std::array<double, 6> ArmController::reported_joint_positions() const {
  if (!map_ok_) return std::array<double, 6>{};  // 서보 배정(하드웨어 프로필)이 없으면 아무것도 보내지 않는다
  const auto s = reported_servo();
  std::array<double, 6> out{};
  for (int i = 0; i < ah::kNumJoints; ++i) {
    const int v = s[cfg_.servo_map.id(i)];
    out[i] = v >= 0 ? ah::servo_to_joint(cfg_.servo_map, i, v) : 0.0;
  }
  return out;
}

ArmPose ArmController::reported_pose() const {
  if (!map_ok_) return ArmPose{};  // 서보 배정(하드웨어 프로필)이 없으면 아무것도 보내지 않는다
  const auto s = reported_servo();
  Joints5 j{};
  for (int i = 0; i < 5; ++i) {
    const int v = s[cfg_.servo_map.id(i)];
    j[i] = v >= 0 ? ah::servo_to_joint(cfg_.servo_map, i, v) : 0.0;
  }
  const ak::Pose p = forward(j);
  return ArmPose{p.x, p.y, p.z, p.pitch, s[cfg_.servo_map.gripper_id()] >= 0 ? ah::servo_to_gripper(s[cfg_.servo_map.gripper_id()], cfg_.gripper) : 0.0};
}

std::vector<ArmEvent> ArmController::take_events() {
  std::vector<ArmEvent> out;
  out.swap(events_);
  return out;
}

}  // namespace armpi_control
