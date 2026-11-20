#include <atomic>
#include <csignal>
#include <thread>

#include "rover_state/mission_node.hpp"
namespace {
std::atomic_bool interrupted{false};
void signal_handler(int) { interrupted = true; }
}  // namespace
int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  try {
    auto node = std::make_shared<rover_state::MissionNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !interrupted) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    node->stop();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (rclcpp::ok() && !node->stopped() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const bool stopped = node->stopped();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return stopped ? 0 : 1;
  } catch (const std::exception &e) {
    fprintf(stderr, "mission_state: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
