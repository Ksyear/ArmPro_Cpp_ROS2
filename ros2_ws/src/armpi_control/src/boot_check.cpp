#include "armpi_control/boot_check.hpp"

#include <sys/stat.h>
#include <sys/utsname.h>

#include <fstream>
#include <sstream>

namespace armpi_control {

namespace {

bool read_text(const std::string& path, std::string* out) {
  std::ifstream f(path);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  *out = ss.str();
  return true;
}

std::string trim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  return s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
}

// cmdline 에서 root= 값
std::string root_arg(const std::string& cmdline) {
  std::istringstream in(cmdline);
  std::string tok;
  while (in >> tok) {
    if (tok.rfind("root=", 0) == 0) return tok.substr(5);
  }
  return "";
}

}  // namespace

std::map<std::string, std::string> parse_env(const std::string& text) {
  std::map<std::string, std::string> out;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#') continue;
    const auto eq = line.find('=');
    if (eq == std::string::npos || eq == 0) continue;
    out[trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
  }
  return out;
}

std::vector<std::string> boot_warnings(const BootFacts& f) {
  std::vector<std::string> w;
  if (!f.manifest_found) return w;  // 이미지가 아님(개발 컨테이너 등): 비교할 기준이 없다
  const auto k = f.manifest.find("kernel");
  if (k != f.manifest.end() && !k->second.empty() && k->second != f.kernel_release) {
    w.push_back("실행 중 커널 " + f.kernel_release + " 이 이미지 커널 " + k->second +
                " 과 다름 — 다른 저장장치(꽂힌 SD 카드 등)의 부팅 파일로 켜졌을 수 있음");
  }
  if (!f.modules_dir_exists) {
    w.push_back("/lib/modules/" + f.kernel_release + " 없음 — 이 커널의 드라이버(무선·카메라 등)가 올라오지 않음");
  }
  const auto p = f.manifest.find("partuuid_root");
  if (p != f.manifest.end() && !p->second.empty()) {
    const std::string want = "PARTUUID=" + p->second;
    const std::string got = root_arg(f.cmdline);
    if (got != want) {
      w.push_back("부팅 설정(cmdline)의 root=" + (got.empty() ? std::string("(없음)") : got) + " 가 이 이미지의 " + want +
                  " 가 아님 — 다른 저장장치의 부팅 파티션으로 켜졌을 수 있음");
    }
  }
  return w;
}

BootFacts read_boot_facts(const std::string& manifest_path, const std::string& modules_root,
                          const std::string& cmdline_path) {
  BootFacts f;
  utsname u{};
  if (::uname(&u) == 0) f.kernel_release = u.release;
  struct stat st {};
  f.modules_dir_exists = !f.kernel_release.empty() &&
                         ::stat((modules_root + "/" + f.kernel_release).c_str(), &st) == 0 && S_ISDIR(st.st_mode);
  std::string text;
  if (read_text(cmdline_path, &text)) f.cmdline = trim(text);
  if (read_text(manifest_path, &text)) {
    f.manifest_found = true;
    f.manifest = parse_env(text);
  }
  return f;
}

}  // namespace armpi_control
