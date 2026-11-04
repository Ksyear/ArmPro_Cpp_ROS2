// 부팅 점검 시험: 이미지 매니페스트와 커널·모듈·cmdline 이 다를 때 경고(SD 카드가 꽂혀 섞여 켜진 경우 등)
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <sys/utsname.h>

#include "armpi_control/boot_check.hpp"

using namespace armpi_control;

namespace {
BootFacts image_ok() {
  BootFacts f;
  f.kernel_release = "5.15.0-1110-raspi";
  f.modules_dir_exists = true;
  f.cmdline = "console=tty1 root=PARTUUID=3ef5baf4-02 rootfstype=ext4 rootwait cfg80211.ieee80211_regdom=KR";
  f.manifest_found = true;
  f.manifest = {{"kernel", "5.15.0-1110-raspi"}, {"partuuid_root", "3ef5baf4-02"}};
  return f;
}
}  // namespace

TEST(BootCheck, MatchingImageHasNoWarnings) { EXPECT_TRUE(boot_warnings(image_ok()).empty()); }

TEST(BootCheck, MixedBootFromSdCardIsReported) {
  BootFacts f = image_ok();
  f.kernel_release = "5.4.0-1068-raspi";
  f.modules_dir_exists = false;
  f.cmdline = "console=tty1 root=LABEL=writable rootfstype=ext4 elevator=deadline rootwait fixrtc";
  const auto w = boot_warnings(f);
  ASSERT_EQ(w.size(), 3u);
  EXPECT_NE(w[0].find("5.4.0-1068-raspi"), std::string::npos);
  EXPECT_NE(w[1].find("/lib/modules/5.4.0-1068-raspi 없음"), std::string::npos);
  EXPECT_NE(w[2].find("root=LABEL=writable"), std::string::npos);
}

TEST(BootCheck, NoManifestMeansNotAnImage) {
  BootFacts f;
  f.kernel_release = "6.8.0-linuxkit";
  EXPECT_TRUE(boot_warnings(f).empty());
}

TEST(BootCheck, ParseEnvAndReadFromFiles) {
  const auto m = parse_env("# 주석\nkernel=5.15.0-1110-raspi\n\n partuuid_root = 3ef5baf4-02 \nbad\n=x\n");
  EXPECT_EQ(m.at("kernel"), "5.15.0-1110-raspi");
  EXPECT_EQ(m.at("partuuid_root"), "3ef5baf4-02");
  EXPECT_EQ(m.size(), 2u);

  char dir[] = "/tmp/armpi_bootcheck_XXXXXX";
  ASSERT_NE(::mkdtemp(dir), nullptr);
  const std::string d = dir;
  utsname u{};
  ASSERT_EQ(::uname(&u), 0);
  std::ofstream(d + "/stage20.env") << "kernel=" << u.release << "\npartuuid_root=abcd-02\n";
  std::ofstream(d + "/cmdline") << "root=PARTUUID=abcd-02 rootwait\n";
  ::mkdir((d + "/modules").c_str(), 0700);
  ::mkdir((d + "/modules/" + u.release).c_str(), 0700);
  const BootFacts f = read_boot_facts(d + "/stage20.env", d + "/modules", d + "/cmdline");
  EXPECT_TRUE(f.manifest_found);
  EXPECT_TRUE(f.modules_dir_exists);
  EXPECT_TRUE(boot_warnings(f).empty());
  const BootFacts g = read_boot_facts(d + "/none.env", d + "/modules", d + "/cmdline");
  EXPECT_FALSE(g.manifest_found);
}
