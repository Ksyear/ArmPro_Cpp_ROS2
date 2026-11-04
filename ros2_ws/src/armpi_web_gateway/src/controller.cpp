#include "armpi_web_gateway/controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <stdexcept>

namespace armpi_web_gateway {

const std::vector<std::string> kJointNames = {"joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"};

namespace {

double round_to(double v, int digits) {
  const double p = std::pow(10.0, digits);
  return std::round(v * p) / p;
}

std::string fmt(const char* f, double a, double b = 0, double c = 0, double d = 0, double e = 0) {
  char buf[160];
  std::snprintf(buf, sizeof(buf), f, a, b, c, d, e);
  return buf;
}

std::string wall_clock_hms_ms() {
  const auto tp = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(tp);
  std::tm tm{};
  ::localtime_r(&t, &tm);
  const int ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count() % 1000);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d", tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
  return buf;
}

std::string short_id(const std::string& id) { return id.substr(0, 4); }

}  // namespace

// ───────────────────────── CommandResult ─────────────────────────

std::string CommandResult::to_json() const {
  std::string j = "{\"ok\":" + json::boolean(ok);
  if (control) j += ",\"control\":" + json::boolean(*control);
  if (released) j += ",\"released\":" + json::boolean(*released);
  if (!error.empty()) j += ",\"error\":" + json::quote(error);
  return j + "}";
}

int CommandResult::http_code() const {
  if (http_code_override) return http_code_override;
  if (ok) return 200;
  return (control && !*control) ? 409 : 400;
}

std::string Controller::Owner::label() const { return (name.empty() ? "API" : name) + "(" + (addr.empty() ? "?" : addr) + ")"; }

// ───────────────────────── 생성·기록 ─────────────────────────

Controller::Controller(std::shared_ptr<armpi_protocol::V3Client> client, RobotLink* robot, ControllerOptions options)
    : client_(std::move(client)), robot_(robot), opt_(std::move(options)) {
  if (!client_) throw std::invalid_argument("V3Client 필요");
  session_live_ = !opt_.on_demand;  // always 모드는 시작부터 세션을 잡는다
  register_commands();
}

Controller::~Controller() { quit(); }

double Controller::now() const {
  if (opt_.clock) return opt_.clock();
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void Controller::log(const std::string& msg) {
  const std::string line = "[" + wall_clock_hms_ms() + "] " + msg;
  {
    std::lock_guard<std::mutex> lk(events_mu_);
    events_.push_back(line);
    while (events_.size() > 60) events_.pop_front();
  }
  if (opt_.log) opt_.log(msg);
}

// ───────────────────────── 조종권 ─────────────────────────

void Controller::neutral_locked() {
  inp_ = InputState{};
  last_input_ = 0.0;
}

void Controller::expire_owner_locked(double t) {
  if (!owner_) return;
  std::string why;
  if (t - owner_->last > kOwnerTimeoutS) {
    why = fmt("화면 응답 없음 %.1f s", kOwnerTimeoutS);
  } else if (opt_.on_demand && opt_.idle_release_s > 0 && t - owner_->last_intent > opt_.idle_release_s) {
    why = fmt("%.0f s 동안 조작 없음", opt_.idle_release_s);
    last_owner_id_.reset();  // 조작 없이 다시 잡지 못하게(그래야 다른 조종기가 로봇을 쓸 수 있음)
  } else {
    return;
  }
  const std::string label = owner_->label();
  owner_.reset();
  neutral_locked();
  ++control_changes_;
  log("조종권 풀림: " + label + " — " + why);
}

void Controller::seen_locked(const Screen& s, double t) {
  const std::string name = s.name.empty() ? "API" : s.name;
  auto it = std::find_if(screens_.begin(), screens_.end(), [&](const Seen& x) { return x.id == s.id; });
  if (it != screens_.end()) {
    it->name = name;
    it->addr = s.addr;
    it->time = t;
  } else {
    screens_.push_back(Seen{s.id, name, s.addr, t});
  }
  if (screens_.size() > 32) {
    screens_.erase(std::remove_if(screens_.begin(), screens_.end(), [&](const Seen& x) { return t - x.time > 10.0; }),
                   screens_.end());
  }
}

bool Controller::claim_locked(const Screen& s, bool intent, bool focused, bool force) {
  const double t = now();
  seen_locked(s, t);
  expire_owner_locked(t);
  if (owner_ && owner_->id == s.id) {
    owner_->last = t;
    if (!s.addr.empty()) owner_->addr = s.addr;
    if (!s.name.empty()) owner_->name = s.name;
    if (intent) owner_->last_intent = t;
    return true;
  }
  if (owner_ && !force) return false;
  if (!owner_ && !force && !intent && !(focused && last_owner_id_ && s.id == *last_owner_id_)) return false;
  if (!last_owner_id_ || s.id != *last_owner_id_ || owner_) {
    // 다른 화면으로 넘어감: 이전 화면의 입력·팔 조종 토글을 버린다.
    neutral_locked();
    arm_enabled_ = false;
    ++control_changes_;
    log("조종권: " + (owner_ ? owner_->label() + " → " : std::string()) + (s.name.empty() ? "API" : s.name) + "(" +
        (s.addr.empty() ? "?" : s.addr) + ")");
  }
  owner_ = Owner{s.id, s.name, s.addr, t, t, t, std::nullopt};
  last_owner_id_ = s.id;
  return true;
}

bool Controller::release_control(const std::string& sid) {
  std::string label;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!owner_ || owner_->id != sid) return false;
    label = owner_->label();
    owner_.reset();
    last_owner_id_.reset();
    neutral_locked();
    arm_enabled_ = false;
    live_until_ = 0.0;
    ++control_changes_;
  }
  log("조종권 놓음: " + label);
  return true;
}

std::string Controller::control_snapshot_json(const std::string& sid) {
  const double t = now();
  std::lock_guard<std::mutex> lk(mu_);
  expire_owner_locked(t);
  std::string owner = "null";
  if (owner_) {
    owner = "{\"name\":" + json::quote(owner_->name.empty() ? "API" : owner_->name) +
            ",\"addr\":" + json::quote(owner_->addr) + ",\"short\":" + json::quote(short_id(owner_->id)) +
            ",\"held_s\":" + json::number(round_to(t - owner_->since, 1)) +
            ",\"idle_s\":" + json::number(round_to(t - owner_->last_intent, 1)) + "}";
  }
  std::string screens = "[";
  bool first = true;
  for (const auto& s : screens_) {
    if (t - s.time > 3.0) continue;
    screens += std::string(first ? "" : ",") + "{\"name\":" + json::quote(s.name) + ",\"addr\":" + json::quote(s.addr) +
               ",\"short\":" + json::quote(short_id(s.id)) + ",\"age_s\":" + json::number(round_to(t - s.time, 1)) +
               ",\"you\":" + json::boolean(s.id == sid) + "}";
    first = false;
  }
  screens += "]";
  return "{\"owner\":" + owner + ",\"you\":" + json::boolean(owner_ && owner_->id == sid) +
         ",\"timeout_s\":" + json::number(kOwnerTimeoutS) + ",\"screens\":" + screens +
         ",\"idle_release_s\":" + json::number(opt_.on_demand ? opt_.idle_release_s : 0.0) +
         ",\"changes\":" + std::to_string(control_changes_) + "}";
}

// ───────────────────────── 입력 ─────────────────────────

std::string Controller::set_input(const json::Value& req, const Screen& screen) {
  const InputState in = parse_input(req);
  const auto n = input_counter(req);
  std::lock_guard<std::mutex> lk(mu_);
  if (!claim_locked(screen, in.focused && (in.held || in.any_arm()), in.focused)) {
    return "{\"ok\":true,\"control\":false}";
  }
  if (n) {
    if (owner_->last_n && *n <= *owner_->last_n) return "{\"ok\":true,\"control\":true,\"stale\":true}";  // 늦게 온 옛 입력
    owner_->last_n = *n;
  }
  inp_ = in;
  last_input_ = now();
  return "{\"ok\":true,\"control\":true}";
}

// ───────────────────────── 세션·루프 ─────────────────────────

std::pair<bool, bool> Controller::session_step_locked(double t) {
  expire_owner_locked(t);
  const bool want = !opt_.on_demand || owner_.has_value() || t < live_until_ || client_->pending_estop();
  const bool start = want && !session_live_;
  const bool stop = session_live_ && !want;
  if (start || stop) session_live_ = want;
  if (stop) arm_enabled_ = false;
  return {start, stop};
}

std::string Controller::session_reason_locked(double t) const {
  if (!session_live_) return "none";
  if (!opt_.on_demand) return "always";
  if (owner_) return "user";
  if (t < live_until_ || client_->pending_estop()) return "estop_only";  // 비상정지만 보내려고 연 세션
  return "closing";  // 조종권을 놓은 직후, 다음 루프에서 닫힘
}

void Controller::start_session(const std::string& why) {
  if (client_->retired()) client_->new_session();
  log("조종 세션 시작(" + why + ")");
}

void Controller::step() {
  const double t = now();
  bool start = false, stop = false, live = false, fresh = false, arm_on = false;
  InputState in;
  {
    std::lock_guard<std::mutex> lk(mu_);
    std::tie(start, stop) = session_step_locked(t);
    live = session_live_;
    in = inp_;
    fresh = t - last_input_ < kInputTimeoutS;
    arm_on = arm_enabled_;
  }
  if (start) start_session("화면이 조종권을 잡음");
  if (stop) {
    client_->disconnect();
    log("조종하는 화면 없음 → 세션 종료(disconnect). 다른 조종기(VR 등)가 로봇을 쓸 수 있음");
  }
  const bool valid = live && fresh && in.focused;
  const bool base_intent = valid && in.held;
  const bool arm_intent = valid && arm_on;
  armpi_protocol::ChannelStatus arm_ch;
  bool retired = false;
  {
    std::lock_guard<std::recursive_mutex> ck(client_->mutex());  // 팔 목표 이동과 tick 을 한 덩어리로
    if (live) {
      if (arm_intent && client_->arm().armed && in.any_arm()) {
        const auto& r = opt_.rates;
        client_->move_arm_target(in.arm[0] * r.arm_m_s * kLoopPeriodS, in.arm[1] * r.arm_m_s * kLoopPeriodS,
                                 in.arm[2] * r.arm_m_s * kLoopPeriodS, in.arm[3] * r.pitch_rad_s * kLoopPeriodS,
                                 in.arm[4] * r.gripper_per_s * kLoopPeriodS);
      }
      try {
        client_->tick(base_intent, in.f, in.s, in.t, arm_intent);
      } catch (const std::exception& e) {
        client_->set_last_error(e.what());
      }
    }
    arm_ch = client_->arm();
    retired = client_->retired();
  }
  if (arm_on && (arm_ch.needs_release || retired)) {
    // 비상정지·not_owner·fault·연결 끊기 같은 강한 차단 뒤에는 사용자가 다시 켜야 한다.
    {
      std::lock_guard<std::mutex> lk(mu_);
      arm_enabled_ = false;
    }
    log("팔 조종 꺼짐(" + (arm_ch.last_block_reason.empty() ? std::string("연결 끊김") : arm_ch.last_block_reason) +
        ") — 다시 켜야 함");
  }
}

void Controller::start() {
  if (running_.exchange(true)) return;
  thread_ = std::thread([this] {
    using clk = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<clk::duration>(std::chrono::duration<double>(kLoopPeriodS));
    auto next = clk::now();
    while (running_) {
      step();
      next += period;
      if (next < clk::now()) {
        ++loop_overruns_;
        next = clk::now() + period;
      }
      while (running_ && clk::now() < next) {
        client_->poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  });
}

void Controller::quit() {
  const bool was_running = running_.exchange(false);
  if (thread_.joinable()) thread_.join();
  bool live;
  {
    std::lock_guard<std::mutex> lk(mu_);
    live = session_live_ || !opt_.on_demand;
  }
  if ((was_running || live) && live && !client_->retired()) {
    client_->disconnect();  // 중립 2개 + disconnect
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    client_->poll();
  }
}

bool Controller::session_live() const {
  std::lock_guard<std::mutex> lk(mu_);
  return session_live_;
}

std::string Controller::session_reason() const {
  std::lock_guard<std::mutex> lk(mu_);
  return session_reason_locked(now());
}

bool Controller::arm_enabled() const {
  std::lock_guard<std::mutex> lk(mu_);
  return arm_enabled_;
}

std::optional<std::string> Controller::owner_id() const {
  std::lock_guard<std::mutex> lk(mu_);
  return owner_ ? std::optional<std::string>(owner_->id) : std::nullopt;
}

// ───────────────────────── 명령 ─────────────────────────

void Controller::arm_enable(bool on) {
  bool was;
  {
    std::lock_guard<std::mutex> lk(mu_);
    was = arm_enabled_;
  }
  if (on && !was) {
    const auto st = robot_ ? robot_->fresh_status() : std::nullopt;
    const json::Value* cur = st ? st->find("arm_current") : nullptr;
    auto num = [&](const char* k) { const auto* v = cur ? cur->find(k) : nullptr; return v && v->is_number(); };
    if (cur && cur->is_object() && num("x") && num("y") && num("z") && num("pitch")) {
      const double g = num("gripper") ? cur->find("gripper")->number : client_->arm_target().gripper;
      client_->set_arm_target(cur->find("x")->number, cur->find("y")->number, cur->find("z")->number,
                              cur->find("pitch")->number, g, true);
      log(fmt("팔 조종 켬: 목표를 로봇 현재값으로 맞춤 x=%.3f y=%.3f z=%.3f", cur->find("x")->number,
              cur->find("y")->number, cur->find("z")->number));
    } else {
      log("팔 조종 켬: 목표 유지(로봇 현재값 모름 — 로봇 상태 없음)");
    }
  } else if (!on && was) {
    log("팔 조종 끔");
  }
  std::lock_guard<std::mutex> lk(mu_);
  arm_enabled_ = on;
}

bool Controller::joint_debug_allowed() const {
  if (!robot_ || !robot_->connected()) return false;
  const auto st = robot_->fresh_status();
  bool no_session;
  {
    std::lock_guard<std::mutex> lk(mu_);
    no_session = client_->retired() || (opt_.on_demand && !session_live_);
  }
  if (!no_session || !st) return false;
  const json::Value* owner = st->find("owner");
  return owner == nullptr || owner->is_null();
}

void Controller::register_commands() {
  auto ok = [] { return CommandResult{}; };
  auto fail = [](const std::string& e) {
    CommandResult r;
    r.ok = false;
    r.error = e;
    return r;
  };
  auto service_error = [](const std::string& e) {
    CommandResult r;
    r.ok = false;
    r.error = e;
    r.http_code_override = 503;
    return r;
  };

  // 조종권 없이도 받는 명령(안전·조종권 관리)
  commands_["take_control"] = {false, [this](const json::Value&, const Screen& s) {
                                 std::lock_guard<std::mutex> lk(mu_);
                                 claim_locked(s, true, true, true);
                                 CommandResult r;
                                 r.control = true;
                                 return r;
                               }};
  commands_["release_control"] = {false, [this](const json::Value&, const Screen& s) {
                                    CommandResult r;
                                    r.released = release_control(s.id);
                                    return r;
                                  }};
  commands_["estop"] = {false, [this, ok](const json::Value&, const Screen&) {
                          bool start = false;
                          {
                            std::lock_guard<std::mutex> lk(mu_);
                            if (opt_.on_demand) {
                              // 세션이 닫혀 있어도 비상정지는 보낸다: 새 UUID 로 세션을 열고(비소유자의 비상정지도
                              // latch 됨) ACK 를 받을 때까지 재전송한 뒤 조종권이 없으면 다시 닫는다.
                              live_until_ = std::max(live_until_, now() + kEstopLingerS);
                              start = !session_live_;
                              session_live_ = true;
                            }
                          }
                          if (start) start_session("비상정지 송신");
                          client_->emergency_stop();
                          std::lock_guard<std::mutex> lk(mu_);
                          arm_enabled_ = false;
                          return ok();
                        }};
  auto robot_service = [this, fail, service_error](const std::string& service, bool home_target) {
    return [this, service, home_target, fail, service_error](const json::Value&, const Screen&) {
      if (!robot_) return fail("로봇 상태 경로 없음");
      std::string err;
      if (!robot_->call_trigger(service, &err)) return service_error(err);
      log("서비스 호출 " + service);
      if (home_target) client_->set_arm_target(opt_.home);
      return CommandResult{};
    };
  };
  commands_["robot_estop"] = {false, robot_service("/armpi/emergency_stop", false)};

  // 조종권을 가진 화면만
  commands_["clear"] = {true, [this, ok](const json::Value&, const Screen&) {
                          client_->clear_emergency();
                          return ok();
                        }};
  commands_["disconnect"] = {true, [this, ok](const json::Value&, const Screen& s) {
                               {
                                 std::lock_guard<std::mutex> lk(mu_);
                                 arm_enabled_ = false;
                               }
                               if (opt_.on_demand) {
                                 release_control(s.id);  // 세션은 루프가 닫는다(disconnect)
                               } else {
                                 client_->disconnect();
                               }
                               return ok();
                             }};
  commands_["connect"] = {true, [this, ok](const json::Value&, const Screen&) {
                            if (opt_.on_demand) {
                              log("연결 요청: 조종권을 잡았으므로 세션을 연다");
                            } else {
                              client_->new_session();
                              log("새 세션으로 연결 시도");
                            }
                            return ok();
                          }};
  commands_["arm_enable"] = {true, [this, ok](const json::Value& req, const Screen&) {
                               arm_enable(pyconv::truthy(req.find("on")));
                               return ok();
                             }};
  commands_["arm_target"] = {true, [this, ok, fail](const json::Value& req, const Screen&) {
                               std::optional<double> vals[5];
                               const char* keys[5] = {"x", "y", "z", "pitch", "gripper"};
                               for (int i = 0; i < 5; ++i) {
                                 const json::Value* v = req.find(keys[i]);
                                 if (v == nullptr || v->is_null()) continue;
                                 const auto d = pyconv::to_float(v);
                                 if (!d) return fail(std::string(keys[i]) + " 숫자 아님");
                                 if (!std::isfinite(*d)) return fail(std::string(keys[i]) + " 유한값 아님");
                                 vals[i] = *d;
                               }
                               const auto t = client_->set_arm_target(vals[0], vals[1], vals[2], vals[3], vals[4]);
                               log(fmt("팔 목표 입력: x=%.3f y=%.3f z=%.3f pitch=%.3f gripper=%.2f", t.x, t.y, t.z,
                                       t.pitch, t.gripper));
                               return ok();
                             }};
  commands_["gripper"] = {true, [this, ok, fail](const json::Value& req, const Screen&) {
                            const auto d = pyconv::to_float(req.find("value"));
                            if (!d) return fail("value 숫자 아님");
                            client_->set_arm_target(std::nullopt, std::nullopt, std::nullopt, std::nullopt, *d);
                            return ok();
                          }};
  commands_["arm_sync"] = {true, [this, ok, fail](const json::Value&, const Screen&) {
                             const auto st = robot_ ? robot_->fresh_status() : std::nullopt;
                             const json::Value* cur = st ? st->find("arm_current") : nullptr;
                             if (cur == nullptr || !cur->is_object()) return fail("로봇 현재값 없음(로봇 상태 필요)");
                             auto get = [&](const char* k) -> std::optional<double> {
                               const json::Value* v = cur->find(k);
                               return (v && v->is_number()) ? std::optional<double>(v->number) : std::nullopt;
                             };
                             client_->set_arm_target(get("x"), get("y"), get("z"), get("pitch"), get("gripper"), true);
                             return ok();
                           }};
  commands_["arm_home_target"] = {true, [this, ok](const json::Value&, const Screen&) {
                                    client_->set_arm_target(opt_.home);
                                    return ok();
                                  }};
  commands_["robot_reset"] = {true, robot_service("/armpi/reset_estop", false)};
  commands_["arm_home"] = {true, robot_service("/armpi/arm/home", true)};
  commands_["joint_debug"] = {true, [this, ok, fail, service_error](const json::Value& req, const Screen&) {
                                if (!joint_debug_allowed()) {
                                  return fail("조종 세션이 있으면 관절 디버그 불가(먼저 '연결 끊기')");
                                }
                                const json::Value* pos = req.find("positions");
                                if (pos == nullptr || !pos->is_array() || pos->array.size() != kJointNames.size()) {
                                  return fail("positions 6개 필요");
                                }
                                std::vector<double> p;
                                for (const auto& v : pos->array) {
                                  const auto d = pyconv::to_float(&v);
                                  if (!d) return fail("positions 숫자 아님");
                                  p.push_back(std::max(-kJointLimitRad, std::min(kJointLimitRad, *d)));
                                }
                                for (double v : p) {
                                  if (!std::isfinite(v)) return fail("positions 유한값 아님");
                                }
                                std::string err;
                                if (!robot_->publish_joint_debug(kJointNames, p, &err)) return service_error(err);
                                return ok();
                              }};
}

CommandResult Controller::command(const json::Value& req, const Screen& screen) {
  const json::Value* c = req.find("cmd");
  const std::string name = (c && c->is_string()) ? c->string : "";
  const auto it = commands_.find(name);
  if (it == commands_.end()) {
    CommandResult r;
    r.ok = false;
    r.error = "알 수 없는 cmd";
    return r;
  }
  if (it->second.owner_only) {
    bool mine;
    std::string label = "?";
    {
      std::lock_guard<std::mutex> lk(mu_);
      mine = claim_locked(screen, true, true);
      if (owner_) label = owner_->label();
    }
    if (!mine) {
      CommandResult r;
      r.ok = false;
      r.control = false;
      r.error = "다른 기기가 조종 중입니다: " + label;
      return r;
    }
  }
  return it->second.run(req, screen);
}

// ───────────────────────── 상태 ─────────────────────────

std::string Controller::state_json(const std::string& screen_id) {
  InputState in;
  bool fresh, arm_on, live;
  std::string reason;
  const std::string control = control_snapshot_json(screen_id);  // 조종권 만료를 먼저 반영
  {
    std::lock_guard<std::mutex> lk(mu_);
    const double t = now();
    in = inp_;
    fresh = t - last_input_ < kInputTimeoutS;
    arm_on = arm_enabled_;
    live = session_live_;
    reason = session_reason_locked(t);
  }
  std::string events = "[";
  {
    std::lock_guard<std::mutex> lk(events_mu_);
    const size_t from = events_.size() > 10 ? events_.size() - 10 : 0;
    for (size_t i = from; i < events_.size(); ++i) events += (i > from ? "," : "") + json::quote(events_[i]);
  }
  events += "]";
  const std::string rb = robot_ ? robot_->snapshot_json() : "{\"enabled\":false,\"connected\":false}";
  return "{\"udp\":" + client_->snapshot_json() + ",\"arm_enabled\":" + json::boolean(arm_on) +
         ",\"input\":{\"fresh\":" + json::boolean(fresh) + ",\"focused\":" + json::boolean(in.focused) +
         ",\"held\":" + json::boolean(in.held) + "},\"rosbridge\":" + rb +
         ",\"joint_debug_allowed\":" + json::boolean(joint_debug_allowed()) +
         ",\"loop_overruns\":" + std::to_string(loop_overruns_.load()) +
         ",\"control\":" + control + ",\"session\":{\"mode\":" +
         json::quote(opt_.on_demand ? "on-demand" : "always") + ",\"live\":" + json::boolean(live) +
         ",\"reason\":" + json::quote(reason) + "},\"events\":" + events + "}";
}

}  // namespace armpi_web_gateway
