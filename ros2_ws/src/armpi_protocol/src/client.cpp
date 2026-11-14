#include "armpi_protocol/client.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <vector>

#include "armpi_common/json.hpp"

namespace armpi_protocol {

namespace json = armpi_common::json;

namespace {

constexpr size_t kMaxSent = 512;
constexpr size_t kMaxRtts = 200;

double clip(double v, double lo, double hi) {
  if (!std::isfinite(v)) return (lo <= 0.0 && 0.0 <= hi) ? 0.0 : lo;
  return std::max(lo, std::min(hi, v));
}

double round_to(double v, int digits) {
  const double p = std::pow(10.0, digits);
  return std::round(v * p) / p;
}

std::string target_json(const ArmTarget& t) {
  return "{\"x\":" + json::number(t.x) + ",\"y\":" + json::number(t.y) + ",\"z\":" + json::number(t.z) +
         ",\"pitch\":" + json::number(t.pitch) + ",\"gripper\":" + json::number(t.gripper) + "}";
}

std::string channel_json(const ChannelStatus& c) {
  return "{\"state\":" + json::quote(c.server_state) + ",\"reason\":" + json::quote(c.server_reason) +
         ",\"armed\":" + json::boolean(c.armed) + ",\"neutral_confirmed\":" + json::boolean(c.neutral_confirmed) +
         ",\"needs_release\":" + json::boolean(c.needs_release) + ",\"intent\":" + json::boolean(c.intent) +
         ",\"blocks\":" + std::to_string(c.blocks) + ",\"auto_rearms\":" + std::to_string(c.auto_rearms) +
         ",\"last_block_reason\":" + json::quote(c.last_block_reason) + "}";
}

bool is_event(PacketType t) {
  return t == PacketType::kEmergencyStop || t == PacketType::kClearEmergency || t == PacketType::kDisconnect;
}

}  // namespace

std::string random_uuid_v4() {
  std::random_device rd;
  unsigned char b[16];
  for (int i = 0; i < 16; i += 4) {
    const uint32_t r = rd();
    for (int k = 0; k < 4; ++k) b[i + k] = static_cast<unsigned char>((r >> (8 * k)) & 0xFF);
  }
  b[6] = static_cast<unsigned char>((b[6] & 0x0F) | 0x40);  // version 4
  b[8] = static_cast<unsigned char>((b[8] & 0x3F) | 0x80);  // variant 10
  char out[37];
  std::snprintf(out, sizeof(out), "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x", b[0], b[1],
                b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
  return out;
}

V3Client::V3Client(std::unique_ptr<DatagramLink> link, ClientOptions options)
    : link_(std::move(link)), opt_(std::move(options)) {
  if (!link_) throw std::invalid_argument("DatagramLink 필요");
  if (opt_.ack_max_age_s < 0) opt_.ack_max_age_s = opt_.ack_timeout_s;
  if (!opt_.new_uuid) opt_.new_uuid = random_uuid_v4;
  std::lock_guard<std::recursive_mutex> lk(mu_);
  new_session_locked(false);
}

double V3Client::now() const {
  if (opt_.clock) return opt_.clock();
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void V3Client::log(const std::string& msg) const {
  if (opt_.log) opt_.log(msg);
}

// ── 세션 ──

void V3Client::new_session_locked(bool keep_emergency) {
  client_id_ = opt_.new_uuid();
  sequence_ = 0;
  last_ack_sequence_ = -1;
  last_ack_time_.reset();
  connected_ = false;
  faulted_ = false;
  retired_ = false;
  if (!keep_emergency) emergency_ = false;
  pending_estop_ = false;
  last_estop_send_ = -1e18;
  estop_acked_seq_ = -1;
  sent_.clear();
  base_ = ChannelStatus{};
  arm_ = ChannelStatus{};
  arm_target_ = opt_.home;
  arm_last_accepted_.reset();
  arm_last_reject_.reset();
  ++sessions_started_;
  log("새 세션 " + client_id_);
}

void V3Client::new_session() {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  new_session_locked(true);
}

// ── 팔 목표 ──

ArmTarget V3Client::set_arm_target(std::optional<double> x, std::optional<double> y, std::optional<double> z,
                                   std::optional<double> pitch, std::optional<double> gripper, bool accepted) {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  ArmTarget t = arm_target_;
  if (x && std::isfinite(*x)) t.x = *x;
  if (y && std::isfinite(*y)) t.y = *y;
  if (z && std::isfinite(*z)) t.z = *z;
  if (pitch) t.pitch = clip(*pitch, -M_PI, M_PI);
  if (gripper) t.gripper = clip(*gripper, 0.0, 1.0);
  if (opt_.arm_target_filter) {
    const ArmTarget f = opt_.arm_target_filter(t, arm_target_);
    if (std::isfinite(f.x) && std::isfinite(f.y) && std::isfinite(f.z) && std::isfinite(f.pitch) &&
        std::isfinite(f.gripper)) {
      t = ArmTarget{f.x, f.y, f.z, clip(f.pitch, -M_PI, M_PI), clip(f.gripper, 0.0, 1.0)};
    }
  }
  arm_target_ = t;
  if (accepted) arm_last_accepted_ = t;
  return t;
}

ArmTarget V3Client::set_arm_target(const ArmTarget& t, bool accepted) {
  return set_arm_target(t.x, t.y, t.z, t.pitch, t.gripper, accepted);
}

ArmTarget V3Client::move_arm_target(double dx, double dy, double dz, double dpitch, double dgripper) {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  const ArmTarget t = arm_target_;
  return set_arm_target(t.x + dx, t.y + dy, t.z + dz, t.pitch + dpitch, t.gripper + dgripper);
}

// ── 송신 ──

int64_t V3Client::next_seq() {
  if (sequence_ >= kMaxSequence) throw std::overflow_error("sequence 소진: 새 세션이 필요합니다");
  return ++sequence_;
}

void V3Client::send_locked(const std::string& data, int64_t seq, PacketType kind, bool arm_channel, bool neutral,
                           std::optional<ArmTarget> target) {
  sent_[seq] = Sent{now(), kind, arm_channel, neutral, target};
  while (sent_.size() > kMaxSent) sent_.erase(sent_.begin());
  std::string err;
  if (link_->send(data, &err)) {
    ++packets_sent_;
  } else {
    ++send_errors_;
    last_error_ = "송신 실패: " + err;
  }
}

bool V3Client::can_drive_locked() const { return connected_ && !emergency_ && !faulted_ && !retired_; }

void V3Client::send_command_locked(double forward, double strafe, double turn) {
  const bool active = base_.armed && base_.intent && can_drive_locked();
  const int64_t seq = next_seq();
  const std::string data = encode_command(client_id_, seq, active, active ? clip(forward, -1, 1) : 0.0,
                                          active ? clip(strafe, -1, 1) : 0.0, active ? clip(turn, -1, 1) : 0.0);
  send_locked(data, seq, PacketType::kCommand, false, !active, std::nullopt);
}

void V3Client::send_arm_locked() {
  const bool active = arm_.armed && arm_.intent && can_drive_locked();
  // 중립 패킷에도 유효한 숫자가 필요하다. 마지막 수락값(없으면 현재 목표)을 담는다(로봇은 무시).
  const ArmTarget& src = active ? arm_target_ : (arm_last_accepted_ ? *arm_last_accepted_ : arm_target_);
  ArmTarget t = src;
  t.pitch = clip(src.pitch, -M_PI, M_PI);
  t.gripper = clip(src.gripper, 0.0, 1.0);
  const int64_t seq = next_seq();
  send_locked(encode_arm_command(client_id_, seq, active, t), seq, PacketType::kArmCommand, true, !active, t);
}

void V3Client::send_event_locked(PacketType kind) {
  const int64_t seq = next_seq();
  send_locked(encode_event(kind, client_id_, seq), seq, kind, false, true, std::nullopt);
  if (kind == PacketType::kEmergencyStop) last_estop_send_ = now();
}

// ── 차단·무장 ──

void V3Client::block_locked(ChannelStatus& ch, bool hard, const std::string& why) {
  if (ch.armed || ch.neutral_confirmed) ++ch.blocks;
  ch.armed = false;
  ch.neutral_confirmed = false;
  ch.neutral_after = sequence_;
  ch.last_block_reason = why;
  if (hard && ch.intent) ch.needs_release = true;
}

void V3Client::block_all_locked(bool hard, const std::string& why) {
  block_locked(base_, hard, why);
  block_locked(arm_, hard, why);
}

void V3Client::update_intent_locked(ChannelStatus& ch, bool intent) {
  if (!intent) {
    // 놓기: 다시 무장하려면 놓은 뒤 보낸 중립이 확인돼야 한다.
    if (ch.intent) block_locked(ch, false, "의도 해제");
    ch.needs_release = false;
    ch.armed = false;
  } else if (!ch.armed && ch.neutral_confirmed && !ch.needs_release && can_drive_locked()) {
    ch.armed = true;
    if (ch.blocks) ++ch.auto_rearms;
  }
  ch.intent = intent;
}

// ── 수신 ──

void V3Client::poll() {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  poll_locked();
}

void V3Client::poll_locked() {
  while (true) {
    std::string err;
    const auto data = link_->receive(&err);
    if (!data) {
      if (!err.empty()) last_error_ = "수신 오류: " + err;  // ICMP port unreachable 등
      break;
    }
    const auto ack = parse_ack(*data);
    if (!ack) {
      ++bad_acks_;
      continue;
    }
    on_ack_locked(*ack, now());
  }
}

void V3Client::on_ack_locked(const AckMessage& ack, double t_now) {
  if (!ack.client_id) {
    ++null_acks_;  // 식별 불가 패킷의 ACK(우리 패킷은 늘 식별 가능해야 함)
    return;
  }
  std::string cid = *ack.client_id;
  std::transform(cid.begin(), cid.end(), cid.begin(), [](unsigned char c) { return std::tolower(c); });
  if (cid != client_id_) return;
  const int64_t seq = ack.sequence;
  const auto it = sent_.find(seq);
  if (it == sent_.end() || seq <= last_ack_sequence_) return;
  const Sent info = it->second;
  const double age = t_now - info.time;
  if (age < 0 || age > opt_.ack_max_age_s) {
    ++late_acks_;
    return;
  }
  ++acks_received_;
  last_ack_sequence_ = seq;
  last_ack_time_ = t_now;
  connected_ = true;
  last_rtt_ = age;
  rtts_.push_back(age);
  while (rtts_.size() > kMaxRtts) rtts_.pop_front();
  ChannelStatus& ch = info.arm_channel ? arm_ : base_;
  ch.server_state = to_string(ack.state);
  ch.server_reason = to_string(ack.reason);
  ch.last_ack_time = t_now;

  if (ack.reason == Reason::kFault || ack.state == ChannelState::kFault) {
    if (!faulted_) log("로봇 fault 보고");
    faulted_ = true;
    block_all_locked(true, "fault");
  }

  const bool robot_estop = ack.emergency_stop || ack.state == ChannelState::kEmergencyStop;
  if (robot_estop) {
    if (info.kind == PacketType::kEmergencyStop || pending_estop_) {
      if (pending_estop_) log("비상정지 ACK 수신(seq " + std::to_string(seq) + ")");
      pending_estop_ = false;
      estop_acked_seq_ = std::max(estop_acked_seq_, seq);
    }
    if (!emergency_) log("로봇 비상정지 상태");
    emergency_ = true;
    block_all_locked(true, "비상정지");
  } else if (emergency_ && !pending_estop_ && seq > estop_acked_seq_) {
    // 로봇이 latch 가 풀렸다고 보고(우리 clear 또는 다른 경로의 해제). 무장은 의도를 새로 내야 한다.
    emergency_ = false;
    log("비상정지 해제 확인");
    block_all_locked(true, "비상정지 해제");
  }

  if (is_event(info.kind)) {
    if (info.kind == PacketType::kDisconnect && ack.accepted) retired_ = true;
    if (info.kind == PacketType::kClearEmergency && ack.accepted) block_all_locked(true, "clear_emergency");
    return;
  }

  if (!ack.accepted) {
    if (info.arm_channel && (ack.reason == Reason::kWorkspaceLimit || ack.reason == Reason::kIkUnreachable)) {
      arm_last_reject_ = Reject{ack.reason, info.target, t_now, seq};
      arm_target_ = arm_last_accepted_ ? *arm_last_accepted_ : opt_.home;
      return;
    }
    if (ack.reason == Reason::kNeutralRequired &&
        (ack.state == ChannelState::kTimeout || ack.state == ChannelState::kAwaitingNeutral)) {
      block_locked(ch, false, std::string("로봇 ") + to_string(ack.state));
    } else if (ack.reason == Reason::kNotOwner) {
      block_all_locked(true, "not_owner");
    } else {
      block_locked(ch, true, to_string(ack.reason));
    }
    return;
  }

  if (ack.state == ChannelState::kTimeout || ack.state == ChannelState::kAwaitingNeutral) {
    block_locked(ch, false, std::string("로봇 ") + to_string(ack.state));
  } else if (info.neutral && seq > ch.neutral_after && ack.state == ChannelState::kIdle) {
    ch.neutral_confirmed = true;
  }
  if (info.arm_channel && !info.neutral && info.target) arm_last_accepted_ = *info.target;
}

void V3Client::check_link_locked(double t_now) {
  if (connected_ && last_ack_time_ && t_now - *last_ack_time_ > opt_.ack_timeout_s) {
    connected_ = false;
    ++link_drops_;
    char why[64];
    std::snprintf(why, sizeof(why), "ACK %.0f ms 없음", opt_.ack_timeout_s * 1e3);
    block_all_locked(false, why);
  }
  for (ChannelStatus* ch : {&base_, &arm_}) {
    if (ch->armed && ch->last_ack_time && t_now - *ch->last_ack_time > opt_.ack_timeout_s) {
      block_locked(*ch, false, "채널 ACK 없음");
    }
  }
}

// ── 주기 처리 ──

void V3Client::tick(bool base_intent, double forward, double strafe, double turn, bool arm_intent) {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  poll_locked();
  const double t_now = now();
  check_link_locked(t_now);
  if (pending_estop_ && t_now - last_estop_send_ >= kEstopRetryPeriodS - 0.005) {
    send_event_locked(PacketType::kEmergencyStop);
  }
  if (retired_) return;
  update_intent_locked(base_, base_intent);
  update_intent_locked(arm_, arm_intent && opt_.send_arm);
  send_command_locked(forward, strafe, turn);
  if (opt_.send_arm) send_arm_locked();
}

// ── 이벤트 ──

void V3Client::emergency_stop() {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  emergency_ = true;
  pending_estop_ = true;
  block_all_locked(true, "비상정지 요청");
  send_event_locked(PacketType::kEmergencyStop);
  log("비상정지 송신");
}

bool V3Client::clear_emergency() {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  if (retired_) return false;
  send_event_locked(PacketType::kClearEmergency);
  log("비상정지 해제 요청 송신");
  return true;
}

void V3Client::disconnect() {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  if (retired_) return;
  update_intent_locked(base_, false);
  update_intent_locked(arm_, false);
  send_command_locked(0.0, 0.0, 0.0);
  if (opt_.send_arm) send_arm_locked();
  send_event_locked(PacketType::kDisconnect);
  retired_ = true;
  log("disconnect 송신");
}

// ── 조회 ──

bool V3Client::retired() const {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  return retired_;
}

bool V3Client::pending_estop() const {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  return pending_estop_;
}

ChannelStatus V3Client::base() const {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  return base_;
}

ChannelStatus V3Client::arm() const {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  return arm_;
}

ArmTarget V3Client::arm_target() const {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  return arm_target_;
}

std::string V3Client::client_id() const {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  return client_id_;
}

void V3Client::set_last_error(const std::string& text) {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  last_error_ = text;
}

std::string V3Client::snapshot_json() const {
  std::lock_guard<std::recursive_mutex> lk(mu_);
  std::vector<double> r(rtts_.begin(), rtts_.end());
  std::sort(r.begin(), r.end());
  const double t_now = now();
  auto ms = [](std::optional<double> v) { return v ? json::number(round_to(*v * 1e3, 2)) : std::string("null"); };
  std::string rej = "null";
  if (arm_last_reject_) {
    rej = "{\"reason\":" + json::quote(to_string(arm_last_reject_->reason)) +
          ",\"target\":" + (arm_last_reject_->target ? target_json(*arm_last_reject_->target) : "null") +
          ",\"sequence\":" + std::to_string(arm_last_reject_->sequence) +
          ",\"age_s\":" + json::number(round_to(t_now - arm_last_reject_->time, 3)) + "}";
  }
  std::string j = "{\"robot\":" + json::quote(link_->peer());
  j += ",\"client_id\":" + json::quote(client_id_);
  j += ",\"sequence\":" + std::to_string(sequence_);
  j += ",\"connected\":" + json::boolean(connected_);
  j += ",\"emergency\":" + json::boolean(emergency_);
  j += ",\"estop_pending\":" + json::boolean(pending_estop_);
  j += ",\"faulted\":" + json::boolean(faulted_);
  j += ",\"retired\":" + json::boolean(retired_);
  j += ",\"base\":" + channel_json(base_);
  j += ",\"arm\":" + channel_json(arm_);
  j += ",\"arm_target\":" + target_json(arm_target_);
  j += ",\"arm_last_accepted\":" + (arm_last_accepted_ ? target_json(*arm_last_accepted_) : std::string("null"));
  j += ",\"arm_last_reject\":" + rej;
  j += ",\"ack_ms_last\":" + ms(last_rtt_);
  j += ",\"ack_ms_p50\":" + ms(r.empty() ? std::nullopt : std::optional<double>(r[r.size() / 2]));
  j += ",\"ack_ms_max\":" + ms(r.empty() ? std::nullopt : std::optional<double>(r.back()));
  j += ",\"packets_sent\":" + std::to_string(packets_sent_);
  j += ",\"acks\":" + std::to_string(acks_received_);
  j += ",\"bad_acks\":" + std::to_string(bad_acks_);
  j += ",\"late_acks\":" + std::to_string(late_acks_);
  j += ",\"null_acks\":" + std::to_string(null_acks_);
  j += ",\"send_errors\":" + std::to_string(send_errors_);
  j += ",\"link_drops\":" + std::to_string(link_drops_);
  j += ",\"last_error\":" + json::quote(last_error_);
  j += ",\"ack_timeout_s\":" + json::number(opt_.ack_timeout_s);
  j += "}";
  return j;
}

}  // namespace armpi_protocol
