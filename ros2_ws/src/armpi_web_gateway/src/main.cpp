// armpi_web_gateway_node: 로봇 안 웹 조종기(휴대폰·PC 브라우저 화면).
// 파이썬 armpi_mac_controller.py --robot 127.0.0.1 --bind 0.0.0.0 --http-port 18765 --no-browser 를 대체한다.
//
// 조립(Composition root): UdpLink → V3Client → Controller(조종권·세션·명령) ← RosRobotLink(상태·서비스)
//                         WebApi(HTTP 경로) → HttpServer(소켓)
// 스레드: ROS 실행기(주 스레드), 50 ms 조종 루프, HTTP 받기 + 연결별 처리.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "armpi_protocol/client.hpp"
#include "armpi_web_gateway/controller.hpp"
#include "armpi_web_gateway/http_server.hpp"
#include "armpi_web_gateway/ros_robot_link.hpp"
#include "armpi_web_gateway/web_api.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gw = armpi_web_gateway;
namespace json = armpi_common::json;

namespace {

std::string local_ipv4() {
  // 화면 안내용 주 IPv4(UDP connect 는 패킷을 보내지 않는다). 모르면 빈 문자열.
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return "";
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(9);
  ::inet_pton(AF_INET, "192.0.2.1", &a.sin_addr);  // TEST-NET-1, 경로 선택만 함
  std::string out;
  if (::connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0) {
    sockaddr_in me{};
    socklen_t len = sizeof(me);
    if (::getsockname(fd, reinterpret_cast<sockaddr*>(&me), &len) == 0) {
      char ip[INET_ADDRSTRLEN] = {0};
      ::inet_ntop(AF_INET, &me.sin_addr, ip, sizeof(ip));
      out = ip;
    }
  }
  ::close(fd);
  return out.rfind("0.", 0) == 0 ? "" : out;
}

std::string str_array(const std::vector<std::string>& v) {
  std::string o = "[";
  for (size_t i = 0; i < v.size(); ++i) o += (i ? "," : "") + json::quote(v[i]);
  return o + "]";
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("armpi_web_gateway");
  auto logger = node->get_logger();

  // ── 매개변수(기본값은 파이썬 조종기와 같다) ──
  const std::string robot = node->declare_parameter<std::string>("robot", "127.0.0.1");
  const int port = static_cast<int>(node->declare_parameter<int64_t>("port", 5001));
  const std::string bind = node->declare_parameter<std::string>("http_bind", "0.0.0.0");
  const int http_port = static_cast<int>(node->declare_parameter<int64_t>("http_port", 18765));
  const auto allow_hosts = node->declare_parameter<std::vector<std::string>>("allow_hosts", std::vector<std::string>{});
  const std::string session = node->declare_parameter<std::string>("session", "auto");  // auto|always|on-demand
  const double idle_release = node->declare_parameter<double>("idle_release_s", 60.0);
  const double ack_timeout = node->declare_parameter<double>("ack_timeout_s", armpi_protocol::kClientAckTimeoutS);
  const auto home_v = node->declare_parameter<std::vector<double>>(
      "home", {armpi_protocol::kDefaultHome.x, armpi_protocol::kDefaultHome.y, armpi_protocol::kDefaultHome.z,
               armpi_protocol::kDefaultHome.pitch, armpi_protocol::kDefaultHome.gripper});
  gw::Rates rates;
  rates.arm_m_s = node->declare_parameter<double>("arm_speed", 0.15);  // 사용자 결정 2026-11-04, 바퀴 방향 실기 시험 전
  rates.pitch_rad_s = node->declare_parameter<double>("pitch_speed", 0.6);
  rates.gripper_per_s = node->declare_parameter<double>("gripper_speed", 0.8);
  // 휴대폰 화면 속도 버튼 기본값(화면이 저장한 값이 없을 때). 로봇 쪽 속도 상한은 armpi_control·수신기 설정이 정한다.
  const double ui_default_speed = node->declare_parameter<double>("ui_default_speed", 0.6);
  const int camera_port = static_cast<int>(node->declare_parameter<int64_t>("camera_port", 8080));
  std::string video_url = node->declare_parameter<std::string>("camera_url", "");
  // web_video_server 스트림 주소의 qos_profile. 기본 "sensor_data"(best-effort).
  // 원시 영상(약 0.9 MB/장)을 기본(RELIABLE)으로 받으면 로봇 안 ROS 통신이 막힐 수 있다(2026-11-04 실기, docs/11).
  // 빈 값이면 qos_profile 을 붙이지 않는다(web_video_server 기본 = RELIABLE, 권장하지 않음).
  const std::string camera_qos = node->declare_parameter<std::string>("camera_qos_profile", "sensor_data");
  std::string www_dir = node->declare_parameter<std::string>("www_dir", "");

  if (home_v.size() != 5) {
    RCLCPP_FATAL(logger, "home 은 숫자 5개(x,y,z,pitch,gripper)여야 합니다");
    return 2;
  }
  if (!(ui_default_speed > 0.0 && ui_default_speed <= 1.0)) {
    RCLCPP_FATAL(logger, "ui_default_speed 는 0 초과 1 이하");
    return 2;
  }
  for (const double v : {rates.arm_m_s, rates.pitch_rad_s, rates.gripper_per_s}) {
    if (!(v > 0.0) || !std::isfinite(v)) {
      RCLCPP_FATAL(logger, "arm_speed·pitch_speed·gripper_speed 는 0 보다 큰 유한값");
      return 2;
    }
  }
  if (session != "auto" && session != "always" && session != "on-demand") {
    RCLCPP_FATAL(logger, "session 은 auto|always|on-demand");
    return 2;
  }
  const armpi_protocol::ArmTarget home{home_v[0], home_v[1], home_v[2], home_v[3], home_v[4]};
  const bool public_bind = bind != "127.0.0.1" && bind != "localhost" && bind.rfind("127.", 0) != 0;
  const bool on_demand = session == "on-demand" || (session == "auto" && public_bind);
  if (www_dir.empty()) www_dir = ament_index_cpp::get_package_share_directory("armpi_web_gateway") + "/www";
  if (video_url.empty()) {
    video_url = "http://" + robot + ":" + std::to_string(camera_port) +
                "/stream?topic=/usb_cam/image_raw&type=ros_compressed";
    if (!camera_qos.empty()) video_url += "&qos_profile=" + camera_qos;
  }

  // ── 조립 ──
  std::string err;
  auto link = armpi_protocol::UdpLink::open(robot, port, &err);
  if (!link) {
    RCLCPP_FATAL(logger, "UDP %s:%d 열기 실패: %s", robot.c_str(), port, err.c_str());
    return 1;
  }
  auto log = [logger](const std::string& m) { RCLCPP_INFO(logger, "%s", m.c_str()); };
  armpi_protocol::ClientOptions copt;
  copt.ack_timeout_s = ack_timeout;
  copt.home = home;
  copt.log = log;
  auto client = std::make_shared<armpi_protocol::V3Client>(std::move(link), copt);

  gw::RosRobotLink robot_link(*node, log);
  gw::ControllerOptions opt;
  opt.home = home;
  opt.rates = rates;
  opt.on_demand = on_demand;
  opt.idle_release_s = idle_release;
  opt.log = log;
  gw::Controller ctl(client, &robot_link, opt);

  char hn[256] = {0};
  ::gethostname(hn, sizeof(hn) - 1);
  gw::WebApiOptions wopt;
  wopt.http_port = http_port;
  wopt.public_bind = public_bind;
  wopt.hostname = hn;
  wopt.extra_hosts = allow_hosts;
  wopt.www_dir = www_dir;
  const std::string session_mode = on_demand ? "on-demand" : "always";
  wopt.config_json =
      "{\"robot\":" + json::quote(robot) + ",\"port\":" + std::to_string(port) + ",\"video_url\":" +
      json::quote(video_url) + ",\"home\":{\"x\":" + json::number(home.x) + ",\"y\":" + json::number(home.y) +
      ",\"z\":" + json::number(home.z) + ",\"pitch\":" + json::number(home.pitch) +
      ",\"gripper\":" + json::number(home.gripper) + "},\"rosbridge\":null,\"backend\":\"armpi_web_gateway(C++)\"" +
      ",\"ack_timeout_s\":" + json::number(ack_timeout) + ",\"input_timeout_s\":" + json::number(gw::kInputTimeoutS) +
      ",\"rates\":{\"arm_m_s\":" + json::number(rates.arm_m_s) + ",\"pitch_rad_s\":" + json::number(rates.pitch_rad_s) +
      ",\"gripper_per_s\":" + json::number(rates.gripper_per_s) + "},\"joint_names\":" + str_array(gw::kJointNames) +
      ",\"joint_limit_rad\":" + json::number(gw::kJointLimitRad) + ",\"session_mode\":" + json::quote(session_mode) +
      ",\"control_timeout_s\":" + json::number(gw::kOwnerTimeoutS) +
      ",\"idle_release_s\":" + json::number(on_demand ? idle_release : 0.0) +
      ",\"ui\":{\"default_speed\":" + json::number(ui_default_speed) + "}}";
  gw::WebApi api(ctl, wopt);
  gw::HttpServer server(bind, http_port, [&api](const gw::HttpRequest& r) { return api.handle(r); });
  if (!server.start(&err)) {
    RCLCPP_FATAL(logger, "HTTP 열기 실패: %s", err.c_str());
    ctl.quit();
    rclcpp::shutdown();
    return 1;
  }
  ctl.start();

  RCLCPP_INFO(logger, "ArmPi 웹 조종기(C++): 로봇 %s UDP %d, 세션 %s, 화면 파일 %s", robot.c_str(), port,
              session_mode.c_str(), www_dir.c_str());
  RCLCPP_INFO(logger, "HTTP 수신 주소: %s:%d", bind.c_str(), server.port());
  if (public_bind) {
    const std::string ip = local_ipv4();
    if (!ip.empty()) {
      RCLCPP_INFO(logger, "휴대폰 화면: http://%s:%d/mobile   (같은 망의 누구나 열 수 있음 — 인증 없음)", ip.c_str(),
                  server.port());
    }
  }

  rclcpp::spin(node);  // SIGINT·SIGTERM 에서 끝난다

  server.stop();
  ctl.quit();  // 중립·disconnect
  RCLCPP_INFO(logger, "웹 조종기 종료(중립·disconnect 송신)");
  rclcpp::shutdown();
  return 0;
}
