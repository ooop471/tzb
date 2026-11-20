// Copyright 2026 Rover Project Contributors
// SPDX-License-Identifier: Apache-2.0
#include "rover_explore/exploration_nav2_planner.hpp"
#include <csignal>
#include <thread>

namespace
{
volatile std::sig_atomic_t stopping = 0;
void interrupt(int) { stopping = 1; }
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, interrupt);
  std::signal(SIGTERM, interrupt);
  int result = 0;
  try
  {
    auto node = std::make_shared<rover_explore::ExplorationNav2Planner>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stopping)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    node->stop();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    while (rclcpp::ok() && !node->stopped() && std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!node->stopped())
    {
      RCLCPP_ERROR(node->get_logger(), "Shutdown timeout awaiting planner terminal cancellation");
      result = 1;
    }
    executor.remove_node(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("exploration_nav2_planner"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
