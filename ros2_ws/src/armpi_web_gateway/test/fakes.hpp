// 시험용 가짜: 보낸 패킷마다 바로 ACK 를 돌려주는 로봇(AutoAckLink), 로봇 상태·서비스(FakeRobotLink), 시계.
#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "armpi_protocol/client.hpp"
#include "armpi_web_gateway/controller.hpp"
#include "armpi_web_gateway/robot_link.hpp"

namespace gwtest {

using namespace armpi_protocol;
namespace json = armpi_common::json;

struct Wire {
  std::vector<std::string> sent;
  std::deque<std::string> inbox;
  bool auto_ack{true};
  bool robot_estop{false};
  Packet last(size_t back = 0) const { return *parse_packet(sent.at(sent.size() - 1 - back)).packet; }
  int count(PacketType t) const {
    int n = 0;
    for (const auto& s : sent) n += parse_packet(s).packet->type == t;
    return n;
  }
};

struct AutoAckLink : DatagramLink {
  Wire* w;
  explicit AutoAckLink(Wire* wire) : w(wire) {}
  bool send(const std::string& data, std::string*) override {
    w->sent.push_back(data);
    if (!w->auto_ack) return true;
    const auto r = parse_packet(data);
    Ack a;
    a.client_id = r.packet->client_id;
    a.sequence = r.packet->sequence;
    a.accepted = true;
    a.reason = Reason::kOk;
    if (r.packet->type == PacketType::kEmergencyStop) w->robot_estop = true;
    if (r.packet->type == PacketType::kClearEmergency) w->robot_estop = false;
    a.emergency_stop = w->robot_estop;
    a.state = w->robot_estop ? ChannelState::kEmergencyStop
                             : (r.packet->control_active ? ChannelState::kActive : ChannelState::kIdle);
    w->inbox.push_back(encode_ack(a));
    return true;
  }
  std::optional<std::string> receive(std::string*) override {
    if (w->inbox.empty()) return std::nullopt;
    std::string d = w->inbox.front();
    w->inbox.pop_front();
    return d;
  }
  std::string peer() const override { return "127.0.0.1:5001"; }
};

struct FakeRobotLink : armpi_web_gateway::RobotLink {
  bool conn{true};
  std::string status{"{\"owner\":null,\"arm_current\":{\"x\":0.15,\"y\":0.01,\"z\":0.05,\"pitch\":-0.5,\"gripper\":0.2}}"};
  bool services_ready{true};
  std::vector<std::string> calls;
  std::vector<std::vector<double>> joint_debug;
  bool connected() const override { return conn; }
  std::optional<json::Value> fresh_status() const override {
    if (!conn) return std::nullopt;
    json::Value v;
    std::string e;
    json::parse(status, v, e, json::lenient_topic_options());
    return v;
  }
  std::string snapshot_json() const override {
    return "{\"enabled\":true,\"connected\":" + json::boolean(conn) + ",\"status\":" + status + "}";
  }
  bool call_trigger(const std::string& s, std::string* err) override {
    if (!services_ready) {
      *err = "로봇 서비스 준비 안 됨: " + s;
      return false;
    }
    calls.push_back(s);
    return true;
  }
  bool publish_joint_debug(const std::vector<std::string>&, const std::vector<double>& p, std::string*) override {
    joint_debug.push_back(p);
    return true;
  }
};

struct Rig {
  double t{1000.0};
  Wire wire;
  FakeRobotLink robot;
  std::shared_ptr<V3Client> client;
  std::unique_ptr<armpi_web_gateway::Controller> ctl;
  explicit Rig(bool on_demand = true, double idle_release = 60.0) {
    ClientOptions co;
    co.clock = [this] { return t; };
    client = std::make_shared<V3Client>(std::make_unique<AutoAckLink>(&wire), co);
    armpi_web_gateway::ControllerOptions o;
    o.on_demand = on_demand;
    o.idle_release_s = idle_release;
    o.clock = [this] { return t; };
    ctl = std::make_unique<armpi_web_gateway::Controller>(client, &robot, o);
  }
  // 루프를 n 번(각 50 ms) 돌린다
  void run(int n) {
    for (int i = 0; i < n; ++i) {
      t += 0.05;
      ctl->step();
      client->poll();
    }
  }
  static json::Value parse(const std::string& s) {
    json::Value v;
    std::string e;
    json::parse(s, v, e, json::lenient_topic_options());
    return v;
  }
  static json::Value input(bool held, bool focused, double f = 0, int arm_x = 0, std::optional<int64_t> n = {}) {
    std::string s = "{\"f\":" + json::number(f) + ",\"held\":" + json::boolean(held) +
                    ",\"focused\":" + json::boolean(focused) + ",\"arm\":{\"x\":" + std::to_string(arm_x) + "}";
    if (n) s += ",\"n\":" + std::to_string(*n);
    return parse(s + "}");
  }
  static json::Value cmd(const std::string& c, const std::string& extra = "") {
    return parse("{\"cmd\":\"" + c + "\"" + extra + "}");
  }
};

inline armpi_web_gateway::Screen screen(const std::string& id, const std::string& addr = "192.168.0.10") {
  return armpi_web_gateway::Screen{id, "폰 " + id, addr};
}

}  // namespace gwtest
