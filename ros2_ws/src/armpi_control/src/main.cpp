#include <memory>

#include "armpi_control/control_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  std::shared_ptr<armpi_control::ControlNode> node;
  try {
    node = std::make_shared<armpi_control::ControlNode>();
    // 단일 스레드 실행기: 차체·팔·상태 콜백이 순서대로 실행된다(공유 상태 경쟁 없음).
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("armpi_control"), "예외로 종료: %s", e.what());
    rc = 1;
  }
  if (node) {
    node->shutdown_outputs();  // 바퀴 0, 부저 끔(서보 토크는 유지)
    node.reset();
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return rc;
}
