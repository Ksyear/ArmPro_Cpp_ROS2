// 웹 조종기 핵심: 화면 입력(/api/input)을 받아 50 ms 마다 Protocol v3 패킷을 보내는 루프와 명령(/api/cmd).
// 파이썬 조종기 Controller(clients/mac_controller/armpi_mac_controller.py)를 C++ 로 옮겼다. 규칙은 같다.
//
// 조종권(화면 토큰): 요청의 screen 값으로 화면을 구분한다(없으면 "api").
//  - 조종권이 비어 있을 때 실제 조작(누름·팔 방향·명령)을 보낸 화면이 잡는다.
//  - 그 화면이 kOwnerTimeoutS 동안 아무 요청도 없으면 풀린다. 다른 화면은 take_control 로만 가져온다
//    (이때 입력은 중립, 팔 조종 토글은 끔).
//  - 비상정지(estop·robot_estop)는 조종권과 관계없이 받는다.
// 세션: always = 계속 잡음, on-demand = 조종권을 가진 화면이 있을 때만 연다(로봇 안 상주 기본).
//  - 열린 이유(/api/state session.reason): none(닫힘) · always · user(조종권을 가진 화면) ·
//    estop_only(조종권 없이 비상정지를 보내려고 잠깐 연 세션, kEstopLingerS 뒤 닫힘) · closing(닫히는 중).
//    화면은 estop_only·closing 을 "연결"로 보이지 않는다(비상정지 표시는 그대로).
// 입력이 kInputTimeoutS 끊기면 두 채널 모두 중립.
//
// 스레드: 공개 메서드는 HTTP 처리 스레드에서 불러도 된다. 잠금 순서는 Controller → V3Client.
#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "armpi_common/json.hpp"
#include "armpi_protocol/client.hpp"
#include "armpi_web_gateway/input.hpp"
#include "armpi_web_gateway/robot_link.hpp"

namespace armpi_web_gateway {

constexpr double kInputTimeoutS = 0.3;   // 브라우저 입력이 이 시간 끊기면 중립
constexpr double kLoopPeriodS = 0.05;    // 채널마다 20 Hz
constexpr double kOwnerTimeoutS = 1.5;   // 조종권을 가진 화면이 이 시간 동안 요청이 없으면 풀림
constexpr double kEstopLingerS = 2.0;    // on-demand 에서 비상정지를 보내려고 연 세션을 유지하는 최소 시간
constexpr double kJointLimitRad = 2.0944;  // ±120°
extern const std::vector<std::string> kJointNames;

struct Rates {
  double arm_m_s{0.05};
  double pitch_rad_s{0.6};
  double gripper_per_s{0.8};
};

struct ControllerOptions {
  armpi_protocol::ArmTarget home{armpi_protocol::kDefaultHome};
  Rates rates;
  bool on_demand{true};
  double idle_release_s{60.0};
  std::function<double()> clock;                // 단조 시계(초). 비우면 steady_clock
  std::function<void(const std::string&)> log;  // 기록 줄을 밖으로(ROS 로그 등)
};

// 요청한 화면
struct Screen {
  std::string id{"api"};
  std::string name;
  std::string addr;
};

// /api/cmd 결과
struct CommandResult {
  bool ok{true};
  std::optional<bool> control;   // false 면 HTTP 409
  std::optional<bool> released;
  std::string error;
  int http_code_override{0};     // 0 이 아니면 그 코드(로봇 서비스 불가 503 등)
  std::string to_json() const;
  int http_code() const;
};

class Controller {
 public:
  Controller(std::shared_ptr<armpi_protocol::V3Client> client, RobotLink* robot, ControllerOptions options);
  ~Controller();
  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;

  // ── HTTP 쪽 ──
  std::string set_input(const json::Value& req, const Screen& screen);  // JSON 응답 본문
  CommandResult command(const json::Value& req, const Screen& screen);
  std::string state_json(const std::string& screen_id);

  // ── 루프 ──
  void start();  // 50 ms 루프 스레드(사이사이 2 ms 마다 ACK 비우기)
  void step();   // 루프 1회(시험에서는 직접 부른다)
  void quit();   // 루프 정지, 세션이 있으면 중립·disconnect

  // ── 조회(시험·상태) ──
  bool session_live() const;
  std::string session_reason() const;  // none|always|user|estop_only|closing
  bool arm_enabled() const;
  std::optional<std::string> owner_id() const;
  uint64_t loop_overruns() const { return loop_overruns_; }

 private:
  struct Owner {
    std::string id, name, addr;
    double since, last, last_intent;
    std::optional<int64_t> last_n;
    std::string label() const;
  };
  struct Seen {
    std::string id, name, addr;
    double time;
  };
  using Handler = std::function<CommandResult(const json::Value&, const Screen&)>;
  struct CommandSpec {
    bool owner_only;  // 조종권을 가진 화면만(없으면 잡고) 실행
    Handler run;
  };

  double now() const;
  void log(const std::string& msg);
  void register_commands();

  // 조종권(mu_ 잡은 상태)
  void neutral_locked();
  void expire_owner_locked(double t);
  void seen_locked(const Screen& s, double t);
  bool claim_locked(const Screen& s, bool intent, bool focused, bool force = false);
  bool release_control(const std::string& sid);
  std::string control_snapshot_json(const std::string& sid);

  // 세션
  std::pair<bool, bool> session_step_locked(double t);
  std::string session_reason_locked(double t) const;
  void start_session(const std::string& why);

  void arm_enable(bool on);
  bool joint_debug_allowed() const;

  std::shared_ptr<armpi_protocol::V3Client> client_;
  RobotLink* robot_;
  ControllerOptions opt_;
  std::map<std::string, CommandSpec> commands_;

  mutable std::mutex mu_;
  InputState inp_;
  double last_input_{0.0};
  bool arm_enabled_{false};
  bool session_live_{false};
  double live_until_{0.0};
  std::optional<Owner> owner_;
  std::optional<std::string> last_owner_id_;
  uint64_t control_changes_{0};
  std::vector<Seen> screens_;  // 최근 요청한 화면(요청 순서 유지)

  std::mutex events_mu_;
  std::deque<std::string> events_;  // 최근 60줄

  std::atomic<bool> running_{false};
  std::atomic<uint64_t> loop_overruns_{0};
  std::thread thread_;
};

}  // namespace armpi_web_gateway
