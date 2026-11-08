// Explicit main for the rm_serial_driver `node` executable.
//
// Replaces the rclcpp_components generated main so that SIGINT/SIGTERM do not
// kill the arm serial stream while tzb_catch is still homing the arm after the
// same signal. See rm_serial_driver/shutdown_hold_gate.hpp for the contract.
#include <rclcpp/rclcpp.hpp>

#include <memory>

#include <rm_serial_driver/node.hpp>
#include <rm_serial_driver/shutdown_hold_gate.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  // The node constructor blocks until the serial device appears; the handlers
  // must already be installed so a signal during that wait still shuts the
  // context down (rclcpp::ok() is polled by that loop).
  ShutdownHoldGate::installSignalHandlers();
  rclcpp::executors::SingleThreadedExecutor exec;
  {
    auto node = std::make_shared<rm::serial::DriverNode>(rclcpp::NodeOptions());
    // A signal during the device wait already shut the context down; creating
    // the gate subscription or spinning would throw on the invalid context.
    if (rclcpp::ok()) {
      ShutdownHoldGate gate(*node);
      exec.add_node(node);
      exec.spin();
      exec.remove_node(node);
    }
  }
  rclcpp::shutdown();
  return 0;
}
