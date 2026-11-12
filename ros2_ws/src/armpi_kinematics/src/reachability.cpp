#include "armpi_kinematics/reachability.hpp"

#include <algorithm>
#include <cmath>

namespace armpi_kinematics {
namespace {
constexpr double kEps = 1e-9;
bool finite_pose(const Pose& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(p.pitch);
}
}  // namespace

bool parse_target_policy(const std::string& text, TargetPolicy* out) {
  if (text == "reject") {
    *out = TargetPolicy::kReject;
    return true;
  }
  if (text == "nearest") {
    *out = TargetPolicy::kNearest;
    return true;
  }
  return false;
}

const char* to_string(TargetPolicy p) { return p == TargetPolicy::kNearest ? "nearest" : "reject"; }

std::string ReachOptions::validate() const {
  if (!(std::isfinite(max_pitch_change_rad) && max_pitch_change_rad >= 0.0 && max_pitch_change_rad <= M_PI)) {
    return "arm.nearest.max_pitch_change_rad 는 0 이상 π 이하";
  }
  if (!(std::isfinite(pitch_step_rad) && pitch_step_rad >= 1e-4 && pitch_step_rad <= 0.5)) {
    return "arm.nearest.pitch_step_rad 는 0.0001 이상 0.5 이하";
  }
  if (!(std::isfinite(pitch_min_rad) && std::isfinite(pitch_max_rad) && pitch_min_rad < pitch_max_rad &&
        pitch_min_rad >= -M_PI && pitch_max_rad <= M_PI)) {
    return "arm.nearest.pitch_min_rad < pitch_max_rad, 둘 다 −π..π";
  }
  if (bisect_iterations < 1 || bisect_iterations > 40) return "arm.nearest.bisect_iterations 는 1..40";
  return "";
}

std::string adjust_to_string(unsigned flags) {
  if (flags == kAdjustNone) return "none";
  std::string s;
  auto add = [&s](const char* n) { s += (s.empty() ? "" : "+") + std::string(n); };
  if (flags & kAdjustBox) add("box");
  if (flags & kAdjustPitch) add("pitch");
  if (flags & kAdjustPosition) add("position");
  return s;
}

Pose clamp_to(const Workspace& w, const Pose& p) {
  Pose q = p;
  q.x = std::min(std::max(p.x, w.x_min), w.x_max);
  q.y = std::min(std::max(p.y, w.y_min), w.y_max);
  q.z = std::min(std::max(p.z, w.z_min), w.z_max);
  return q;
}

TargetProjector::TargetProjector(const Params& kinematics, const Workspace& workspace, const ReachOptions& options)
    : kin_(kinematics), ws_(workspace), opt_(options) {}

bool TargetProjector::ik_ok(double x, double y, double z, double pitch) const {
  return kin_.inverse(Pose{x, y, z, pitch}).status == IkStatus::kOk;
}

bool TargetProjector::reachable(const Pose& p) const {
  return finite_pose(p) && inside(ws_, p) && kin_.inverse(p).status == IkStatus::kOk;
}

bool TargetProjector::find_pitch(double x, double y, double z, double want, const double* prefer,
                                 const double* extra, double* out) const {
  if (ik_ok(x, y, z, want)) {
    *out = want;
    return true;
  }
  const double cap = opt_.max_pitch_change_rad;
  if (cap > 0.0) {
    const int n = static_cast<int>(std::floor(cap / opt_.pitch_step_rad + kEps));
    auto allowed = [this](double c) { return c >= opt_.pitch_min_rad - kEps && c <= opt_.pitch_max_rad + kEps; };
    for (int i = 1; i <= n + 1; ++i) {
      // 마지막 한 번은 간격이 맞지 않아도 cap 끝값 자체를 본다
      const double d = (i <= n) ? i * opt_.pitch_step_rad : cap;
      if (i == n + 1 && std::fabs(d - n * opt_.pitch_step_rad) < kEps) break;
      double first = want - d, second = want + d;
      if (prefer && std::fabs(second - *prefer) < std::fabs(first - *prefer)) std::swap(first, second);
      for (const double c : {first, second}) {
        if (allowed(c) && ik_ok(x, y, z, c)) {
          *out = c;
          return true;
        }
      }
    }
  }
  if (extra && std::isfinite(*extra) && ik_ok(x, y, z, *extra)) {
    *out = *extra;
    return true;
  }
  return false;
}

ReachResult TargetProjector::project(const Pose& target, const Pose* anchor) const {
  ReachResult r;
  if (!finite_pose(target) || !kin_.valid()) {
    r.requested_status = IkStatus::kInvalid;
    return r;
  }
  const Pose p = clamp_to(ws_, target);
  if (p.x != target.x || p.y != target.y || p.z != target.z) r.adjust |= kAdjustBox;
  r.requested_status = kin_.inverse(p).status;
  if (r.requested_status == IkStatus::kOk) {
    r.ok = true;
    r.pose = p;
    return r;
  }

  const double* prefer = anchor ? &anchor->pitch : nullptr;
  double pc = 0.0;
  if (find_pitch(p.x, p.y, p.z, p.pitch, prefer, nullptr, &pc)) {
    r.ok = true;
    r.pose = Pose{p.x, p.y, p.z, pc};
    r.adjust |= kAdjustPitch;
    return r;
  }

  if (!opt_.allow_position_change || !anchor || !reachable(*anchor)) return r;  // ok=false

  // anchor(닿음) → p(안 닿음) 직선에서 닿는 가장 먼 점. 각 점에서 pitch 는 요청에 가장 가까운 값(마지막에 anchor pitch).
  Pose best = *anchor;
  double lo = 0.0, hi = 1.0;
  for (int it = 0; it < opt_.bisect_iterations; ++it) {
    const double s = 0.5 * (lo + hi);
    const double x = anchor->x + s * (p.x - anchor->x);
    const double y = anchor->y + s * (p.y - anchor->y);
    const double z = anchor->z + s * (p.z - anchor->z);
    if (find_pitch(x, y, z, p.pitch, prefer, &anchor->pitch, &pc)) {
      lo = s;
      best = Pose{x, y, z, pc};
    } else {
      hi = s;
    }
  }
  r.ok = true;
  r.pose = best;
  r.adjust |= kAdjustPosition;
  if (best.pitch != p.pitch) r.adjust |= kAdjustPitch;
  return r;
}

}  // namespace armpi_kinematics
