// TargetProjector(닿을 수 없는 목표 → 닿을 수 있는 가장 가까운 자세) 시험.
// 기구학 값: 링크 길이는 armpi.yaml(공개 ArmPi FPV 값), 관절 기준·범위는 시험용 프로필(test_hardware_profile.yaml) 값.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "armpi_kinematics/reachability.hpp"

namespace ak = armpi_kinematics;

namespace {

ak::Params test_params() {
  ak::Params p;
  p.base_height_m = 0.064605;
  p.upper_arm_m = 0.10048;
  p.forearm_m = 0.094714;
  p.hand_m = 0.16331;
  p.j2_offset = -1.4;
  p.j3_sign = -1.0;
  p.j4_offset = 0.1;
  p.min_wrist_height_m = 0.0;
  p.min_elbow_bend_rad = 0.05;
  const double jmin[4] = {-2.0943951023931953, -1.4, -2.0943951023931953, -1.9};
  for (int i = 0; i < 4; ++i) {
    p.joint_min[i] = jmin[i];
    p.joint_max[i] = -jmin[i];
  }
  return p;
}

ak::Workspace test_box() { return ak::Workspace{0.05, 0.32, -0.25, 0.25, -0.08, 0.32}; }

ak::TargetProjector make(const ak::ReachOptions& o = ak::ReachOptions{}) {
  return ak::TargetProjector(test_params(), test_box(), o);
}

bool ik_ok(const ak::TargetProjector& tp, double x, double y, double z, double pitch) {
  return tp.kinematics().inverse(ak::Pose{x, y, z, pitch}).status == ak::IkStatus::kOk;
}

// 결정적 난수(LCG)
struct Lcg {
  uint64_t s{12345};
  double next(double lo, double hi) {
    s = s * 6364136223846793005ULL + 1442695040888963407ULL;
    return lo + (hi - lo) * static_cast<double>(s >> 11) / static_cast<double>(1ULL << 53);
  }
};

// 손끝 아래(−90°) 자세가 안 되지만 다른 pitch 는 되는 점
bool find_pitch_case(const ak::TargetProjector& tp, ak::Pose* out) {
  for (double x = 0.06; x <= 0.32; x += 0.01) {
    for (double z = -0.06; z <= 0.30; z += 0.01) {
      if (ik_ok(tp, x, 0.0, z, -M_PI / 2)) continue;
      for (double pd = -89.0; pd <= 30.0; pd += 1.0) {
        if (ik_ok(tp, x, 0.0, z, pd * M_PI / 180.0)) {
          *out = ak::Pose{x, 0.0, z, -M_PI / 2};
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace

TEST(Reachability, ParsePolicyAndNames) {
  ak::TargetPolicy p{};
  EXPECT_TRUE(ak::parse_target_policy("reject", &p));
  EXPECT_EQ(p, ak::TargetPolicy::kReject);
  EXPECT_TRUE(ak::parse_target_policy("nearest", &p));
  EXPECT_EQ(p, ak::TargetPolicy::kNearest);
  EXPECT_FALSE(ak::parse_target_policy("Nearest", &p));
  EXPECT_FALSE(ak::parse_target_policy("", &p));
  EXPECT_STREQ(ak::to_string(ak::TargetPolicy::kNearest), "nearest");
  EXPECT_EQ(ak::adjust_to_string(ak::kAdjustNone), "none");
  EXPECT_EQ(ak::adjust_to_string(ak::kAdjustBox | ak::kAdjustPitch), "box+pitch");
  EXPECT_EQ(ak::adjust_to_string(ak::kAdjustPosition), "position");
}

TEST(Reachability, OptionsValidate) {
  EXPECT_EQ(ak::ReachOptions{}.validate(), "");
  ak::ReachOptions o;
  o.max_pitch_change_rad = -0.1;
  EXPECT_NE(o.validate(), "");
  o = ak::ReachOptions{};
  o.pitch_step_rad = 0.0;
  EXPECT_NE(o.validate(), "");
  o = ak::ReachOptions{};
  o.pitch_min_rad = 0.6;
  EXPECT_NE(o.validate(), "");
  o = ak::ReachOptions{};
  o.bisect_iterations = 0;
  EXPECT_NE(o.validate(), "");
}

TEST(Reachability, ReachableTargetUnchanged) {
  const auto tp = make();
  const ak::Pose home{0.1047, 0.0, 0.0753, -1.0263};
  ASSERT_TRUE(tp.reachable(home));
  const auto r = tp.project(home, nullptr);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.adjust, ak::kAdjustNone);
  EXPECT_EQ(r.requested_status, ak::IkStatus::kOk);
  EXPECT_DOUBLE_EQ(r.pose.x, home.x);
  EXPECT_DOUBLE_EQ(r.pose.y, home.y);
  EXPECT_DOUBLE_EQ(r.pose.z, home.z);
  EXPECT_DOUBLE_EQ(r.pose.pitch, home.pitch);
}

TEST(Reachability, OutsideBoxIsClampedToFace) {
  const auto tp = make();
  // 기본 자세에서 옆으로만 상자 밖(y = 0.40). 자른 뒤 y = y_max.
  const ak::Pose anchor{0.1047, 0.0, 0.0753, -1.0263};
  const auto r = tp.project(ak::Pose{0.1047, 0.40, 0.0753, -1.0263}, &anchor);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.adjust & ak::kAdjustBox);
  EXPECT_TRUE(ak::inside(test_box(), r.pose));
  EXPECT_TRUE(tp.reachable(r.pose));
  const auto c = ak::clamp_to(test_box(), ak::Pose{0.5, -0.5, 0.5, 0.2});
  EXPECT_DOUBLE_EQ(c.x, 0.32);
  EXPECT_DOUBLE_EQ(c.y, -0.25);
  EXPECT_DOUBLE_EQ(c.z, 0.32);
  EXPECT_DOUBLE_EQ(c.pitch, 0.2);
}

TEST(Reachability, PitchIsClosestReachable) {
  const auto tp = make();
  ak::Pose t;
  ASSERT_TRUE(find_pitch_case(tp, &t));
  const auto r = tp.project(t, nullptr);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.adjust, ak::kAdjustPitch);
  EXPECT_NE(r.requested_status, ak::IkStatus::kOk);
  EXPECT_DOUBLE_EQ(r.pose.x, t.x);
  EXPECT_DOUBLE_EQ(r.pose.z, t.z);
  EXPECT_TRUE(tp.reachable(r.pose));
  const double d = std::fabs(r.pose.pitch - t.pitch);
  EXPECT_LE(d, tp.options().max_pitch_change_rad + 1e-9);
  EXPECT_GE(r.pose.pitch, tp.options().pitch_min_rad - 1e-9);
  EXPECT_LE(r.pose.pitch, tp.options().pitch_max_rad + 1e-9);
  // 더 가까운 후보 pitch(간격 단위)는 모두 안 된다
  const double step = tp.options().pitch_step_rad;
  for (double e = step; e < d - 1e-9; e += step) {
    for (const double c : {t.pitch - e, t.pitch + e}) {
      if (c < tp.options().pitch_min_rad - 1e-9 || c > tp.options().pitch_max_rad + 1e-9) continue;
      EXPECT_FALSE(ik_ok(tp, t.x, t.y, t.z, c)) << "더 가까운 pitch " << c << " 가 됨";
    }
  }
}

TEST(Reachability, NoPitchChangeAndNoAnchorFails) {
  ak::ReachOptions o;
  o.max_pitch_change_rad = 0.0;
  const auto tp = make(o);
  ak::Pose t;
  ASSERT_TRUE(find_pitch_case(tp, &t));
  const auto r = tp.project(t, nullptr);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.requested_status, ak::IkStatus::kOk);
}

TEST(Reachability, PositionFallbackStopsOnSegment) {
  const auto tp = make();
  const ak::Pose anchor{0.1047, 0.0, 0.0753, -1.0263};
  ASSERT_TRUE(tp.reachable(anchor));
  // 상자 안이지만 어떤 pitch 로도 닿지 않는 먼 모서리(r ≈ 0.40 m)
  const ak::Pose far{0.32, 0.25, 0.30, 0.0};
  const auto r = tp.project(far, &anchor);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.adjust & ak::kAdjustPosition);
  EXPECT_TRUE(tp.reachable(r.pose));
  // anchor → far 직선 위
  const double s = (r.pose.x - anchor.x) / (far.x - anchor.x);
  EXPECT_GT(s, 0.05);
  EXPECT_LT(s, 1.0);
  EXPECT_NEAR(r.pose.y, anchor.y + s * (far.y - anchor.y), 1e-9);
  EXPECT_NEAR(r.pose.z, anchor.z + s * (far.z - anchor.z), 1e-9);
  // 같은 직선에서 조금 더 가면(5 mm) 허용 pitch 범위 안 어떤 값으로도 닿지 않는다
  const double len = std::sqrt(std::pow(far.x - anchor.x, 2) + std::pow(far.y - anchor.y, 2) +
                               std::pow(far.z - anchor.z, 2));
  const double s2 = s + 0.005 / len;
  const double x2 = anchor.x + s2 * (far.x - anchor.x), y2 = anchor.y + s2 * (far.y - anchor.y),
               z2 = anchor.z + s2 * (far.z - anchor.z);
  bool any = false;
  for (double pd = -90.0; pd <= 30.0; pd += 0.5) any = any || ik_ok(tp, x2, y2, z2, pd * M_PI / 180.0);
  EXPECT_FALSE(any);
}

TEST(Reachability, PositionFallbackDisabledFails) {
  ak::ReachOptions o;
  o.allow_position_change = false;
  const auto tp = make(o);
  const ak::Pose anchor{0.1047, 0.0, 0.0753, -1.0263};
  EXPECT_FALSE(tp.project(ak::Pose{0.32, 0.25, 0.30, 0.0}, &anchor).ok);
}

TEST(Reachability, UnreachableAnchorFails) {
  const auto tp = make();
  const ak::Pose bad_anchor{0.32, 0.25, 0.30, 0.0};
  ASSERT_FALSE(tp.reachable(bad_anchor));
  EXPECT_FALSE(tp.project(ak::Pose{0.32, 0.25, 0.31, 0.0}, &bad_anchor).ok);
}

TEST(Reachability, RandomTargetsAlwaysReachableWithAnchor) {
  const auto tp = make();
  const ak::Pose home{0.1047, 0.0, 0.0753, -1.0263};
  Lcg rng;
  int pitch_only = 0, position = 0;
  ak::Pose anchor = home;
  for (int i = 0; i < 4000; ++i) {
    // 상자보다 넓은 범위, pitch 는 −π..π
    const ak::Pose t{rng.next(-0.2, 0.5), rng.next(-0.5, 0.5), rng.next(-0.3, 0.5), rng.next(-M_PI, M_PI)};
    const auto r = tp.project(t, &anchor);
    ASSERT_TRUE(r.ok) << i;
    ASSERT_TRUE(tp.reachable(r.pose)) << i;
    if (r.adjust == ak::kAdjustPitch) ++pitch_only;
    if (r.adjust & ak::kAdjustPosition) ++position;
    // 바꾼 pitch 는 허용 범위 안(요청 pitch 그대로이거나 anchor pitch 인 경우는 예외)
    if ((r.adjust & ak::kAdjustPitch) && r.pose.pitch != anchor.pitch) {
      EXPECT_GE(r.pose.pitch, tp.options().pitch_min_rad - 1e-9);
      EXPECT_LE(r.pose.pitch, tp.options().pitch_max_rad + 1e-9);
    }
    anchor = r.pose;  // 다음 목표의 기준(조종기가 연속으로 보내는 상황)
  }
  EXPECT_GT(pitch_only, 0);
  EXPECT_GT(position, 0);
}

TEST(Reachability, TieBreakPrefersAnchorPitchSide) {
  const auto tp = make();
  // 요청 pitch 는 안 되고, 같은 거리의 양쪽(want−d, want+d)이 둘 다 되는 점을 찾는다
  const double step = tp.options().pitch_step_rad;
  for (double x = 0.08; x <= 0.30; x += 0.01) {
    for (double z = -0.04; z <= 0.28; z += 0.01) {
      for (double wd = -80.0; wd <= 20.0; wd += 5.0) {
        const double want = wd * M_PI / 180.0;
        if (ik_ok(tp, x, 0.0, z, want)) continue;
        for (int i = 1; i <= 20; ++i) {
          const double lo = want - i * step, hi = want + i * step;
          const bool okl = ik_ok(tp, x, 0.0, z, lo), okh = ik_ok(tp, x, 0.0, z, hi);
          if (!okl && !okh) continue;
          if (!(okl && okh) || lo < tp.options().pitch_min_rad || hi > tp.options().pitch_max_rad) break;
          const ak::Pose a_low{x, 0.0, z, lo - 0.3}, a_high{x, 0.0, z, hi + 0.3};
          const auto rl = tp.project(ak::Pose{x, 0.0, z, want}, &a_low);
          const auto rh = tp.project(ak::Pose{x, 0.0, z, want}, &a_high);
          ASSERT_TRUE(rl.ok && rh.ok);
          EXPECT_DOUBLE_EQ(rl.pose.pitch, lo);
          EXPECT_DOUBLE_EQ(rh.pose.pitch, hi);
          return;
        }
      }
    }
  }
  GTEST_SKIP() << "양쪽이 같은 거리에서 함께 풀리는 점이 이 격자에 없음";
}

TEST(Reachability, RequestedPitchOutsideBoundsKeptWhenReachable) {
  const auto tp = make();
  // 요청 pitch 가 허용 범위(−90..+30°) 밖이어도 그대로 풀리면 바꾸지 않는다
  for (double x = 0.10; x <= 0.30; x += 0.01) {
    for (double z = 0.0; z <= 0.30; z += 0.01) {
      const double want = 45.0 * M_PI / 180.0;
      if (!ik_ok(tp, x, 0.0, z, want)) continue;
      const auto r = tp.project(ak::Pose{x, 0.0, z, want}, nullptr);
      ASSERT_TRUE(r.ok);
      EXPECT_EQ(r.adjust, ak::kAdjustNone);
      EXPECT_DOUBLE_EQ(r.pose.pitch, want);
      return;
    }
  }
  GTEST_SKIP() << "+45° 로 풀리는 점이 이 격자에 없음";
}

TEST(Reachability, InvalidInputFails) {
  const auto tp = make();
  EXPECT_FALSE(tp.project(ak::Pose{NAN, 0.0, 0.1, 0.0}, nullptr).ok);
  EXPECT_FALSE(tp.project(ak::Pose{0.1, 0.0, 0.1, INFINITY}, nullptr).ok);
}
