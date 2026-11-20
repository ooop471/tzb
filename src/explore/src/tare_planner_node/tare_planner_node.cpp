#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <exception>
#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sensor_coverage_planner_3d_ns::SensorCoveragePlanner3D>();
    if (!node->initialize()) {
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
    rclcpp::spin(node);
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("rover_explore"), "Invalid exploration setup: %s", exception.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return 0;
}
