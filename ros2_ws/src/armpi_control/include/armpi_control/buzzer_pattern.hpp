// 부저 패턴 재생기(ROS 비의존, 시간은 호출자가 넘긴다).
//
// 패턴 = "on_s 동안 울림 → off_s 동안 쉼"을 count 번. 노드는 주기 작업(misc_tick, 50 Hz)마다 level(now)를 읽어
// 바뀌었을 때만 GPIO 에 쓴다. 우선순위가 높은 패턴(비상정지)이 울리는 중에는 낮은 패턴(시작 알림)이 끼어들지 못하고,
// 높은 패턴은 낮은 패턴을 바로 덮어쓴다.
#pragma once

#include <string>

namespace armpi_control {

struct BeepPattern {
  int count{1};
  double on_s{0.1};
  double off_s{0.15};
};

// 시작 알림 횟수: 정상 1, 경고(부팅 점검·초기화 경고) 2, fault 3
int startup_beep_count(bool fault, bool has_warnings);
// 사람이 읽는 설명(로그용)
std::string startup_beep_reason(bool fault, bool has_warnings);

class BuzzerSequencer {
 public:
  enum Priority { kStartup = 0, kEstop = 1 };

  // 재생을 요청한다. 더 높은 우선순위가 재생 중이면 무시하고 false.
  bool play(const BeepPattern& p, double now, int priority);
  // now 에 부저가 켜져 있어야 하는가
  bool level(double now) const;
  // 아직 재생 중인가(마지막 울림이 끝나기 전)
  bool active(double now) const;
  void stop() { active_ = false; }

 private:
  bool active_{false};
  BeepPattern pattern_;
  double start_{0.0};
  int priority_{kStartup};
};

}  // namespace armpi_control
