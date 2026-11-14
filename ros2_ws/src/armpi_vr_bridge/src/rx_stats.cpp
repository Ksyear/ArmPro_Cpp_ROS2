#include "armpi_vr_bridge/rx_stats.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace armpi_vr_bridge {

void RxStats::observe(double now, std::size_t size, const Ack& ack, unsigned arm_adjust) {
  ++packets_;
  bytes_ += size;
  if (last_rx_) {
    intervals_.push_back(now - *last_rx_);
    while (intervals_.size() > window_) intervals_.pop_front();
  }
  last_rx_ = now;
  ++reasons_[to_string(ack.reason)];
  if (ack.accepted) ++accepted_;
  if (ack.accepted && arm_adjust != 0) ++arm_adjust_[armpi_kinematics::adjust_to_string(arm_adjust)];
  if (ack.client_id && ack.sequence >= 0) {
    const std::string& cid = *ack.client_id;
    auto it = last_seq_.find(cid);
    if (it != last_seq_.end()) {
      const int64_t last = it->second;
      if (ack.sequence > last + 1) {
        ++gaps_;
        missing_ += static_cast<uint64_t>(ack.sequence - last - 1);
      } else if (ack.sequence <= last) {
        ++reordered_;
      }
      if (ack.sequence > last) it->second = ack.sequence;
    } else {
      last_seq_[cid] = ack.sequence;
      seq_order_.push_back(cid);
      if (seq_order_.size() > 64) {
        last_seq_.erase(seq_order_.front());
        seq_order_.pop_front();
      }
    }
  }
}

RxSummary RxStats::summary(double now) const {
  RxSummary s;
  s.packets = packets_;
  s.accepted = accepted_;
  s.bytes = bytes_;
  s.filtered = filtered_;
  s.reasons = reasons_;
  s.arm_adjust = arm_adjust_;
  s.sequence_gaps = gaps_;
  s.estimated_lost = missing_;
  s.reordered_or_duplicate = reordered_;
  if (last_rx_) s.last_rx_age_s = now - *last_rx_;
  if (!intervals_.empty()) {
    std::vector<double> iv(intervals_.begin(), intervals_.end());
    std::sort(iv.begin(), iv.end());
    const double mean = std::accumulate(iv.begin(), iv.end(), 0.0) / static_cast<double>(iv.size());
    auto pct = [&](double p) {
      const auto idx = static_cast<std::size_t>(std::lround(p * static_cast<double>(iv.size() - 1)));
      return iv[std::min(iv.size() - 1, idx)];
    };
    s.interval_mean_s = mean;
    if (mean > 0) s.rate_hz = 1.0 / mean;
    s.interval_p50_s = pct(0.5);
    s.interval_p95_s = pct(0.95);
    s.interval_max_s = iv.back();
  }
  return s;
}

}  // namespace armpi_vr_bridge
