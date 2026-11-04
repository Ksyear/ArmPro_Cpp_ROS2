// armpi_vr_bridge 노드 실행 파일.
// rclcpp 기본 신호 처리기를 끄고 직접 SIGINT/SIGTERM 을 받는다. rclcpp 가 먼저 context 를 닫으면
// 종료 정지 명령(3회)을 발행할 수 없기 때문이다. 순서: 신호 → spin 종료 → 정지 3회 발행 → rclcpp 종료.
#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <thread>

#include "armpi_vr_bridge/bridge_node.hpp"

namespace {
std::atomic<bool> g_stop{false};
extern "C" void on_signal(int) { g_stop.store(true); }
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  struct sigaction sa {};
  sa.sa_handler = on_signal;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  std::shared_ptr<armpi_vr_bridge::VrBridgeNode> node;
  try {
    node = std::make_shared<armpi_vr_bridge::VrBridgeNode>();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("armpi_vr_bridge"), "시작 실패: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (!g_stop.load() && rclcpp::ok()) {
    executor.spin_once(std::chrono::milliseconds(50));
  }
  executor.remove_node(node);
  try {
    node->shutdown_outputs();
  } catch (const std::exception& e) {
    // context 가 이미 닫힌 경우 등. 제어기 2차 watchdog(0.6 s)이 정지를 맡는다.
    RCLCPP_ERROR(rclcpp::get_logger("armpi_vr_bridge"), "종료 정지 명령 발행 실패: %s", e.what());
  }
  // reliable QoS 전송 여유(구독자가 마지막 정지 명령을 받을 시간)
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  node.reset();
  rclcpp::shutdown();
  return 0;
}
