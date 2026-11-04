// 수신 주기·거부 사유·순서 역전 요약 통계. 스레드 안전하지 않다.
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>

#include "armpi_vr_bridge/protocol.hpp"

namespace armpi_vr_bridge {

struct RxSummary {
  uint64_t packets{0};
  uint64_t accepted{0};
  uint64_t bytes{0};
  uint64_t filtered{0};  // allowed_sources 로 버린 수
  std::map<std::string, uint64_t> reasons;
  std::optional<double> rate_hz;
  std::optional<double> interval_mean_s;
  std::optional<double> interval_p50_s;
  std::optional<double> interval_p95_s;
  std::optional<double> interval_max_s;
  uint64_t sequence_gaps{0};
  uint64_t estimated_lost{0};
  uint64_t reordered_or_duplicate{0};
  std::optional<double> last_rx_age_s;
};

class RxStats {
 public:
  explicit RxStats(std::size_t window = 400) : window_(window) {}
  void observe(double now, std::size_t size, const Ack& ack);
  void count_filtered() { ++filtered_; }
  RxSummary summary(double now) const;

 private:
  std::size_t window_;
  uint64_t packets_{0}, accepted_{0}, bytes_{0}, filtered_{0};
  std::map<std::string, uint64_t> reasons_;
  std::optional<double> last_rx_;
  std::deque<double> intervals_;
  std::unordered_map<std::string, int64_t> last_seq_;
  std::deque<std::string> seq_order_;  // last_seq_ 크기 제한(64)용 삽입 순서
  uint64_t gaps_{0}, missing_{0}, reordered_{0};
};

}  // namespace armpi_vr_bridge
