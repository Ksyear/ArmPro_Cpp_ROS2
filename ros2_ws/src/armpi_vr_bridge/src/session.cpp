#include "armpi_vr_bridge/session.hpp"

#include <cmath>
#include <cstdio>
#include <iterator>
#include <stdexcept>

namespace armpi_vr_bridge {

namespace {

std::string fmt_seconds(const char* prefix, double s, const char* suffix) {
  char buf[160];
  std::snprintf(buf, sizeof(buf), "%s%.3fs%s", prefix, s, suffix);
  return buf;
}

bool positive_finite(double v) { return std::isfinite(v) && v > 0.0; }

}  // namespace

void SessionConfig::validate() const {
  auto need = [](bool ok, const char* what) {
    if (!ok) throw std::invalid_argument(what);
  };
  need(positive_finite(base_watchdog_s), "base watchdog 은 0보다 커야 합니다");
  need(positive_finite(arm_watchdog_s), "arm watchdog 은 0보다 커야 합니다");
  need(positive_finite(lease_s), "lease 는 0보다 커야 합니다");
  need(lease_s > base_watchdog_s && lease_s > arm_watchdog_s, "lease 는 두 채널 watchdog 보다 커야 합니다");
  need(positive_finite(control_state_max_age_s), "control_state 최대 나이는 0보다 커야 합니다");
  need(positive_finite(max_vx_m_s) && positive_finite(max_vy_m_s) && positive_finite(max_wz_rad_s),
       "차체 속도 상한은 0보다 큰 유한값이어야 합니다");
  need(retired_limit >= 1, "retired_limit 은 1 이상이어야 합니다");
  const auto& w = workspace;
  need(std::isfinite(w.x_min) && std::isfinite(w.x_max) && std::isfinite(w.y_min) && std::isfinite(w.y_max) &&
           std::isfinite(w.z_min) && std::isfinite(w.z_max),
       "arm.workspace 값은 유한해야 합니다");
  need(w.x_min < w.x_max && w.y_min < w.y_max && w.z_min < w.z_max, "arm.workspace 는 min < max 여야 합니다");
  need(armpi_kinematics::ArmKinematics(kinematics).valid(),
       "arm.kinematics 가 올바르지 않습니다(링크 길이>0, sign≠0, joint_min<joint_max)");
}

Session::Session(const SessionConfig& config) : cfg_(config), kin_(config.kinematics) { cfg_.validate(); }

// ── 조회 ──

bool Session::emergency_stop() const { return udp_estop_ || (cs_received_ && cs_.estop); }

std::optional<std::string> Session::fault() const {
  if (internal_fault_) return internal_fault_;
  if (cs_received_ && cs_.fault) {
    return std::string("제어기 fault") + (cs_.fault_reason.empty() ? "" : ": " + cs_.fault_reason);
  }
  return std::nullopt;
}

ChannelState Session::channel_state(const ChannelCore& ch) const {
  if (fault()) return ChannelState::kFault;
  if (emergency_stop()) return ChannelState::kEmergencyStop;
  if (ch.timed_out) return ChannelState::kTimeout;
  if (!ch.neutral_confirmed) return ChannelState::kAwaitingNeutral;
  return ch.active ? ChannelState::kActive : ChannelState::kIdle;
}

ChannelState Session::state(Channel channel) const {
  return channel == Channel::kArm ? channel_state(arm_) : channel_state(base_);
}

ArmOutput Session::arm_output() const {
  ArmOutput o = arm_.goal;
  o.active = arm_.active;
  return o;
}

SessionSnapshot Session::snapshot(double now) const {
  SessionSnapshot s;
  s.owner = owner_;
  if (owner_addr_) s.owner_addr = owner_addr_->to_string();
  s.base_state = base_state();
  s.arm_state = arm_state();
  s.emergency_stop = emergency_stop();
  s.udp_estop_latched = udp_estop_;
  s.controller_estop = cs_received_ && cs_.estop;
  s.fault = fault();
  s.udp_permitted = gate_permitted_;
  s.blocked_reason = blocked_reason_;
  s.control_state_received = cs_received_;
  if (cs_received_) s.control_state_age_s = now - cs_rx_time_;
  s.last_sequence = last_sequence_;
  s.last_reason = last_reason_;
  s.base_target = base_.target;
  s.arm_output = arm_output();
  s.retired_count = retired_order_.size();
  return s;
}

// ── 외부 입력 ──

std::vector<std::string> Session::set_control_state(const ControlState& state, double now) {
  const bool em_before = emergency_stop();
  const bool fault_before = fault().has_value();
  cs_ = state;
  cs_received_ = true;
  cs_rx_time_ = now;
  std::vector<std::string> ev;
  const bool em_after = emergency_stop();
  if (em_before != em_after) {
    reset_all();
    ev.push_back(em_after ? "비상정지 상태(제어기 보고); 차체 정지·팔 위치 유지"
                          : "비상정지 해제(제어기 보고); 두 채널 새 중립 대기");
  }
  const auto f = fault();
  if (fault_before != f.has_value()) {
    reset_all();
    ev.push_back(f ? "fault 상태: " + *f : std::string("제어기 fault 해제; 두 채널 새 중립 대기"));
  }
  evaluate_gate(now, ev);
  return ev;
}

void Session::force_stop() { reset_all(); }

void Session::enter_fault(const std::string& reason) {
  internal_fault_ = reason;
  reset_all();
}

// ── 시간 기반 정지 ──

std::vector<std::string> Session::check_timeouts(double now) {
  std::vector<std::string> ev;
  if (base_.last_rx && now - *base_.last_rx >= cfg_.base_watchdog_s) {
    reset_base();
    base_.timed_out = true;
    ev.push_back(fmt_seconds("차체 watchdog ", cfg_.base_watchdog_s, " 만료; 차체 정지"));
  }
  if (arm_.last_rx && now - *arm_.last_rx >= cfg_.arm_watchdog_s) {
    reset_arm();
    arm_.timed_out = true;
    ev.push_back(fmt_seconds("팔 watchdog ", cfg_.arm_watchdog_s, " 만료; 팔 현재 위치 유지"));
  }
  if (owner_ && now - last_owner_activity_ >= cfg_.lease_s) {
    previous_owner_ = std::make_pair(*owner_, last_sequence_);
    release_owner();
    ev.push_back(fmt_seconds("조종자 lease ", cfg_.lease_s, " 만료; 소유권 해제"));
  }
  evaluate_gate(now, ev);
  return ev;
}

void Session::evaluate_gate(double now, std::vector<std::string>& events) {
  bool permitted = false;
  std::optional<std::string> why;
  if (!cs_received_) {
    why = "control_state 미수신";
  } else if (now - cs_rx_time_ > cfg_.control_state_max_age_s) {
    why = fmt_seconds("control_state ", cfg_.control_state_max_age_s, " 넘게 미수신");
  } else if (!cs_.udp_allowed) {
    why = "제어기가 UDP 제어를 허용하지 않음(udp_allowed=false)";
  } else {
    permitted = true;
  }
  if (gate_permitted_ && !permitted) {
    bool stopped = false;
    if (!base_.target.is_zero()) {
      reset_base();
      stopped = true;
    }
    if (arm_.active) {
      reset_arm();
      stopped = true;
    }
    events.push_back("UDP 활성 명령 차단: " + *why + (stopped ? "; 정지·새 중립 대기" : ""));
  } else if (!gate_permitted_ && permitted) {
    events.push_back("UDP 활성 명령 허용(control_state 정상)");
  }
  gate_permitted_ = permitted;
  blocked_reason_ = why;
}

// ── 패킷 처리 ──

HandleResult Session::handle_datagram(std::string_view data, const Endpoint& from, double now) {
  auto expired = check_timeouts(now);
  ParseResult pr = parse_packet(data);
  HandleResult r = pr.ok() ? handle_current(*pr.packet, from, now)
                           : reject(pr.error.client_id, pr.error.sequence, pr.error.reason, pr.error.channel,
                                    pr.error.message);
  r.events.insert(r.events.begin(), expired.begin(), expired.end());
  return r;
}

HandleResult Session::handle_packet(const Packet& packet, const Endpoint& from, double now) {
  // 만료 검사는 sequence·소유권·명령 갱신보다 반드시 먼저 한다.
  auto expired = check_timeouts(now);
  HandleResult r = handle_current(packet, from, now);
  r.events.insert(r.events.begin(), expired.begin(), expired.end());
  return r;
}

HandleResult Session::handle_current(const Packet& p, const Endpoint& from, double now) {
  const std::string& sid = p.session_id;
  const bool is_owner = owner_ && *owner_ == sid && owner_addr_ && *owner_addr_ == from;

  // 안전 정지 우선: 소유자가 아닌 유효 클라이언트의 비상정지도 latch 한다.
  // 소유권·sequence 흐름은 바꾸지 않는다(다른 조종자의 번호 흐름 보호).
  if (p.type == PacketType::kEmergencyStop && cfg_.accept_estop_from_any_client && !is_owner &&
      (owner_ || is_retired(sid))) {
    const bool was = udp_estop_;
    udp_estop_ = true;
    reset_all();
    HandleResult r = accept(p, was ? std::nullopt : std::optional<std::string>("비상정지 latch (비소유자 " + sid + ")"));
    if (!was) r.estop_change = true;
    return r;
  }

  if (is_retired(sid)) {
    return reject(p.client_id, p.sequence, Reason::kNotOwner, p.channel(), "종료되었거나 교체된 세션");
  }

  std::optional<std::string> claimed;
  if (!owner_) {
    if (p.type == PacketType::kDisconnect) {
      retire(sid);
      if (previous_owner_ && previous_owner_->first == sid) previous_owner_.reset();
      return accept(p);
    }
    int64_t baseline = -1;
    if (previous_owner_ && previous_owner_->first == sid) baseline = previous_owner_->second;
    if (p.sequence <= baseline) {
      return reject(p.client_id, p.sequence, Reason::kOutOfOrder, p.channel(), "sequence 역전 또는 중복");
    }
    if (previous_owner_ && previous_owner_->first != sid) retire(previous_owner_->first);
    previous_owner_.reset();
    owner_ = sid;
    owner_addr_ = from;
    last_sequence_ = baseline;
    reset_all();
    claimed = "조종자 획득: " + sid + " " + from.to_string();
  } else if (!is_owner) {
    return reject(p.client_id, p.sequence, Reason::kNotOwner, p.channel(), "다른 조종자가 제어 중");
  }

  if (p.sequence <= last_sequence_) {
    return reject(p.client_id, p.sequence, Reason::kOutOfOrder, p.channel(), "sequence 역전 또는 중복");
  }
  last_sequence_ = p.sequence;
  last_owner_activity_ = now;  // lease 는 어느 채널이든(이벤트 포함) 소유자의 새 번호 패킷으로 갱신

  HandleResult r = apply(p, now);
  if (claimed) r.events.insert(r.events.begin(), *claimed);
  return r;
}

HandleResult Session::apply(const Packet& p, double now) {
  switch (p.type) {
    case PacketType::kEmergencyStop: {
      const bool was = udp_estop_;
      udp_estop_ = true;
      reset_all();
      HandleResult r = accept(p, was ? std::nullopt : std::optional<std::string>("비상정지 latch"));
      if (!was) r.estop_change = true;
      return r;
    }
    case PacketType::kClearEmergency: {
      const bool was = emergency_stop();
      udp_estop_ = false;
      reset_all();
      HandleResult r =
          accept(p, was ? std::optional<std::string>("비상정지 해제 요청; 두 채널 새 중립 대기") : std::nullopt);
      if (was) r.estop_change = false;
      return r;
    }
    case PacketType::kDisconnect: {
      retire(*owner_);
      release_owner();
      return accept(p, "조종자 disconnect; 소유권 해제");
    }
    case PacketType::kCommand:
      return apply_base(p, now);
    case PacketType::kArmCommand:
      return apply_arm(p, now);
  }
  return reject(p.client_id, p.sequence, Reason::kInvalidPacket, p.channel(), "알 수 없는 type");
}

HandleResult Session::apply_base(const Packet& p, double now) {
  if (p.is_neutral()) {
    std::optional<std::string> ev;
    if (!fault() && !emergency_stop()) {
      if (!base_.neutral_confirmed) ev = "차체 중립 확인";
      base_.neutral_confirmed = true;
      base_.timed_out = false;
      base_.active = p.control_active;
      base_.last_rx = now;
    }
    base_.target = BaseTarget{};
    return accept(p, ev);
  }
  if (auto f = fault()) return reject(p.client_id, p.sequence, Reason::kFault, Channel::kBase, *f);
  if (emergency_stop()) return reject(p.client_id, p.sequence, Reason::kEmergencyStop, Channel::kBase, "비상정지 중");
  if (!base_.neutral_confirmed) {
    return reject(p.client_id, p.sequence, Reason::kNeutralRequired, Channel::kBase, "차체 중립 확인 전");
  }
  if (!gate_permitted_) {
    return reject(p.client_id, p.sequence, Reason::kNotOwner, Channel::kBase, blocked_reason_.value_or("차단"));
  }
  base_.active = true;
  base_.target = BaseTarget{p.forward * cfg_.max_vx_m_s, p.strafe * cfg_.max_vy_m_s, p.turn * cfg_.max_wz_rad_s};
  base_.last_rx = now;
  return accept(p);
}

HandleResult Session::apply_arm(const Packet& p, double now) {
  if (!p.control_active) {
    std::optional<std::string> ev;
    if (!fault() && !emergency_stop()) {
      if (!arm_.neutral_confirmed) ev = "팔 중립 확인";
      arm_.neutral_confirmed = true;
      arm_.timed_out = false;
      arm_.active = false;
      arm_.last_rx = now;
    }
    HandleResult r = accept(p, ev);
    r.arm_accepted = true;
    return r;
  }
  if (auto f = fault()) return reject(p.client_id, p.sequence, Reason::kFault, Channel::kArm, *f);
  if (emergency_stop()) return reject(p.client_id, p.sequence, Reason::kEmergencyStop, Channel::kArm, "비상정지 중");
  if (!arm_.neutral_confirmed) {
    return reject(p.client_id, p.sequence, Reason::kNeutralRequired, Channel::kArm, "팔 중립 확인 전");
  }
  if (!gate_permitted_) {
    return reject(p.client_id, p.sequence, Reason::kNotOwner, Channel::kArm, blocked_reason_.value_or("차단"));
  }
  const armpi_kinematics::Pose pose{p.x, p.y, p.z, p.pitch};
  // 거부는 목표를 바꾸지 않고 watchdog 도 갱신하지 않는다.
  if (!armpi_kinematics::inside(cfg_.workspace, pose)) {
    return reject(p.client_id, p.sequence, Reason::kWorkspaceLimit, Channel::kArm, "팔 목표가 작업 공간 밖");
  }
  const auto ik = kin_.inverse(pose);
  if (ik.status != armpi_kinematics::IkStatus::kOk) {
    return reject(p.client_id, p.sequence, Reason::kIkUnreachable, Channel::kArm,
                  std::string("IK 해 없음: ") + armpi_kinematics::to_string(ik.status));
  }
  arm_.active = true;
  arm_.goal.has_goal = true;
  arm_.goal.x = p.x;
  arm_.goal.y = p.y;
  arm_.goal.z = p.z;
  arm_.goal.pitch = p.pitch;
  arm_.goal.gripper = p.gripper;
  arm_.last_rx = now;
  HandleResult r = accept(p);
  r.arm_accepted = true;
  return r;
}

// ── 내부 ──

void Session::reset_base() {
  base_.reset();
  base_.target = BaseTarget{};
}

void Session::reset_arm() { arm_.reset(); }  // 마지막 목표 값은 정보용으로 남긴다(active=false)

void Session::reset_all() {
  reset_base();
  reset_arm();
}

void Session::release_owner() {
  owner_.reset();
  owner_addr_.reset();
  last_sequence_ = -1;
  last_owner_activity_ = 0.0;
  reset_all();
}

void Session::retire(const std::string& session_id) {
  auto it = retired_index_.find(session_id);
  if (it != retired_index_.end()) {
    retired_order_.splice(retired_order_.end(), retired_order_, it->second);
    return;
  }
  retired_order_.push_back(session_id);
  retired_index_[session_id] = std::prev(retired_order_.end());
  while (retired_order_.size() > cfg_.retired_limit) {
    retired_index_.erase(retired_order_.front());
    retired_order_.pop_front();
  }
}

bool Session::is_retired(const std::string& session_id) const {
  return retired_index_.find(session_id) != retired_index_.end();
}

Ack Session::make_ack(const std::optional<std::string>& client_id, int64_t sequence, bool accepted, Reason reason,
                      Channel channel) {
  last_reason_ = reason;
  Ack a;
  a.client_id = client_id;
  a.sequence = sequence;
  a.accepted = accepted;
  a.reason = reason;
  a.emergency_stop = emergency_stop();
  a.state = state(channel);
  return a;
}

HandleResult Session::accept(const Packet& p, std::optional<std::string> event) {
  HandleResult r;
  r.channel = p.channel();
  r.ack = make_ack(p.client_id, p.sequence, true, Reason::kOk, r.channel);
  if (event) r.events.push_back(std::move(*event));
  return r;
}

HandleResult Session::reject(const std::optional<std::string>& client_id, int64_t sequence, Reason reason,
                             Channel channel, std::string detail) {
  HandleResult r;
  r.channel = channel;
  r.ack = make_ack(client_id, sequence, false, reason, channel);
  r.detail = std::move(detail);
  return r;
}

}  // namespace armpi_vr_bridge
