// Shutdown lease gate shared by the processes tzb_catch depends on while it
// homes the arm after SIGINT/SIGTERM (planning_node, rm_serial_driver node).
//
// Identical copies live in
//   lite_motion_planner/include/lite_motion_planner/shutdown_hold_gate.hpp
//   rm_serial_driver/include/rm_serial_driver/shutdown_hold_gate.hpp
// so that neither package has to depend on the other. Keep them byte-identical.
//
// Contract:
//   * The process must be started with rclcpp::SignalHandlerOptions::None and
//     call ShutdownHoldGate::installSignalHandlers() right after rclcpp::init(),
//     before any node constructor that may block.
//   * tzb_catch publishes std_msgs/Bool on /tzb_catch/shutdown_hold at 10 Hz:
//     true ONLY while its shutdown homing / place-and-release motion is in
//     progress, false otherwise (idle, finished, nothing to do).
//   * On the first SIGINT/SIGTERM/SIGHUP: wait up to kLeaseGraceSec for a
//     fresh `true` lease (covers a group Ctrl+C delivered to everyone at once:
//     tzb_catch needs a few hundred ms to stop its worker and start homing).
//     None -> rclcpp::shutdown() at once. Otherwise keep spinning until the
//     lease turns false, goes stale (no message for kHoldFreshSec) or
//     tzb.shutdown_hold_timeout_sec elapses; then rclcpp::shutdown().
//   * Repeated SIGINT/SIGTERM/SIGHUP while holding are logged and ignored.
//   * SIGQUIT always shuts down immediately.
#ifndef SHUTDOWN_HOLD_GATE_HPP_
#define SHUTDOWN_HOLD_GATE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

class ShutdownHoldGate {
public:
  static constexpr double kHoldFreshSec = 0.5;
  static constexpr double kLeaseGraceSec = 1.5;
  static constexpr const char* kHoldTopic = "/tzb_catch/shutdown_hold";
  static constexpr const char* kTimeoutParam = "tzb.shutdown_hold_timeout_sec";
  static constexpr double kDefaultTimeoutSec = 45.0;

  // Installs async-signal-safe handlers (self-pipe) for SIGINT/SIGTERM/SIGQUIT
  // and starts the watcher thread. Until a gate is attached every signal shuts
  // the rclcpp context down immediately (this also breaks blocking constructor
  // loops that poll rclcpp::ok()).
  static void installSignalHandlers() {
    static std::once_flag once;
    std::call_once(once, []() {
      if (::pipe(pipeFds()) != 0) {
        throw std::runtime_error("ShutdownHoldGate: pipe() failed");
      }
      const int flags = ::fcntl(pipeFds()[1], F_GETFL, 0);
      ::fcntl(pipeFds()[1], F_SETFL, flags | O_NONBLOCK);
      struct sigaction action {};
      action.sa_handler = &ShutdownHoldGate::onSignal;
      sigemptyset(&action.sa_mask);
      action.sa_flags = SA_RESTART;
      sigaction(SIGINT, &action, nullptr);
      sigaction(SIGTERM, &action, nullptr);
      sigaction(SIGHUP, &action, nullptr);  // SSH drop behaves like SIGTERM
      sigaction(SIGQUIT, &action, nullptr);
      // ros2 launch exits at once on SIGTERM and closes our stdout pipe; a
      // log line must not kill the process with SIGPIPE while it holds.
      signal(SIGPIPE, SIG_IGN);
      std::thread(&ShutdownHoldGate::watcherLoop).detach();
    });
  }

  explicit ShutdownHoldGate(rclcpp::Node& node)
  : node_(node) {
    if (!node_.has_parameter(kTimeoutParam)) {
      node_.declare_parameter<double>(kTimeoutParam, kDefaultTimeoutSec);
    }
    timeout_sec_ = node_.get_parameter(kTimeoutParam).as_double();
    hold_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
      kHoldTopic, rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        const auto now_ns = nowNs();
        if (msg->data) {
          last_true_ns_.store(now_ns);
        } else {
          last_false_ns_.store(now_ns);
        }
      });
    std::lock_guard<std::mutex> lock(attachMutex());
    attached() = this;
    RCLCPP_INFO(
      node_.get_logger(),
      "[SHUTDOWN][GATE] listening on %s (grace=%.1fs for a lease, fresh<=%.2fs, max_hold=%.1fs); SIGQUIT bypasses the hold",
      kHoldTopic, kLeaseGraceSec, kHoldFreshSec, timeout_sec_);
  }

  ~ShutdownHoldGate() {
    std::lock_guard<std::mutex> lock(attachMutex());
    attached() = nullptr;
  }

  ShutdownHoldGate(const ShutdownHoldGate&) = delete;
  ShutdownHoldGate& operator=(const ShutdownHoldGate&) = delete;

private:
  static int* pipeFds() {
    static int fds[2] = {-1, -1};
    return fds;
  }
  static std::mutex& attachMutex() {
    static std::mutex m;
    return m;
  }
  static ShutdownHoldGate*& attached() {
    static ShutdownHoldGate* gate = nullptr;
    return gate;
  }
  static int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static void onSignal(int signum) {
    const unsigned char byte = static_cast<unsigned char>(signum);
    const int saved_errno = errno;
    (void)!::write(pipeFds()[1], &byte, 1);
    errno = saved_errno;
  }

  static void watcherLoop() {
    bool shutting_down = false;
    for (;;) {
      unsigned char byte = 0;
      const ssize_t n = ::read(pipeFds()[0], &byte, 1);
      if (n <= 0) {
        if (n < 0 && errno == EINTR) {
          continue;
        }
        return;
      }
      const int signum = static_cast<int>(byte);
      std::lock_guard<std::mutex> lock(attachMutex());
      ShutdownHoldGate* gate = attached();
      if (shutting_down) {
        if (gate) {
          RCLCPP_WARN(gate->node_.get_logger(),
            "[SHUTDOWN][SIGNAL][IGNORED] signal=%d already shutting down", signum);
        }
        continue;
      }
      if (!gate) {
        // No node attached yet (or already detached): nothing can be held.
        shutting_down = true;
        rclcpp::shutdown();
        continue;
      }
      if (signum == SIGQUIT) {
        RCLCPP_ERROR(gate->node_.get_logger(),
          "[SHUTDOWN][RELEASE] reason=sigquit_emergency signal=%d", signum);
        shutting_down = true;
        rclcpp::shutdown();
        continue;
      }
      if (gate->holding_.load()) {
        RCLCPP_WARN(gate->node_.get_logger(),
          "[SHUTDOWN][SIGNAL][IGNORED] signal=%d repeated while waiting for tzb_catch homing", signum);
        continue;
      }
      shutting_down = true;
      gate->holdThenShutdown(signum);
    }
  }

  bool holdFresh(int64_t now_ns) const {
    const int64_t last_true = last_true_ns_.load();
    const int64_t last_false = last_false_ns_.load();
    if (last_true == 0 || last_false > last_true) {
      return false;
    }
    return static_cast<double>(now_ns - last_true) * 1e-9 <= kHoldFreshSec;
  }

  // Runs on the watcher thread while the executor keeps spinning on the main
  // thread, so the hold subscription stays alive during the wait. Repeated
  // signals are drained by the watcher loop only after this returns, which is
  // what makes them "ignored" (the pipe buffers them).
  void holdThenShutdown(int signum) {
    const int64_t start_ns = nowNs();
    // Grace: tzb_catch raises the lease only once its shutdown motion starts,
    // a few hundred ms after a simultaneous group signal.
    holding_.store(true);
    while (!holdFresh(nowNs())) {
      if (drainRepeatedSignals()) {
        holding_.store(false);
        RCLCPP_ERROR(node_.get_logger(), "[SHUTDOWN][RELEASE] reason=sigquit_emergency signal=%d", signum);
        rclcpp::shutdown();
        return;
      }
      if (!rclcpp::ok() || static_cast<double>(nowNs() - start_ns) * 1e-9 >= kLeaseGraceSec) {
        holding_.store(false);
        RCLCPP_INFO(node_.get_logger(),
          "[SHUTDOWN][RELEASE] reason=no_lease_within_grace signal=%d waited=%.2fs", signum,
          static_cast<double>(nowNs() - start_ns) * 1e-9);
        rclcpp::shutdown();
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    RCLCPP_WARN(node_.get_logger(),
      "[SHUTDOWN][HOLD] waiting for tzb_catch homing (signal=%d, max %.1fs)", signum, timeout_sec_);
    std::string reason;
    for (;;) {
      // Drain repeated signals without blocking so they are logged promptly;
      // SIGQUIT is the only one that cuts the hold short.
      if (drainRepeatedSignals()) {
        reason = "sigquit_emergency";
        break;
      }
      const int64_t now_ns = nowNs();
      const double elapsed = static_cast<double>(now_ns - start_ns) * 1e-9;
      if (!rclcpp::ok()) {
        reason = "context_shutdown";
        break;
      }
      if (last_false_ns_.load() > last_true_ns_.load()) {
        reason = "hold_released";
        break;
      }
      if (!holdFresh(now_ns)) {
        reason = "hold_stale";
        break;
      }
      if (elapsed >= timeout_sec_) {
        reason = "hold_timeout";
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const double elapsed = static_cast<double>(nowNs() - start_ns) * 1e-9;
    if (reason == "hold_timeout" || reason == "sigquit_emergency") {
      RCLCPP_ERROR(node_.get_logger(),
        "[SHUTDOWN][RELEASE] reason=%s waited=%.2fs", reason.c_str(), elapsed);
    } else {
      RCLCPP_INFO(node_.get_logger(),
        "[SHUTDOWN][RELEASE] reason=%s waited=%.2fs", reason.c_str(), elapsed);
    }
    holding_.store(false);
    rclcpp::shutdown();
  }

  // Returns true when a SIGQUIT was pending.
  bool drainRepeatedSignals() {
    bool quit = false;
    struct pollfd pfd {};
    pfd.fd = pipeFds()[0];
    pfd.events = POLLIN;
    while (::poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
      unsigned char byte = 0;
      if (::read(pipeFds()[0], &byte, 1) != 1) {
        break;
      }
      if (static_cast<int>(byte) == SIGQUIT) {
        quit = true;
        continue;
      }
      RCLCPP_WARN(node_.get_logger(),
        "[SHUTDOWN][SIGNAL][IGNORED] signal=%d repeated while waiting for tzb_catch homing",
        static_cast<int>(byte));
    }
    return quit;
  }

  rclcpp::Node& node_;
  double timeout_sec_{kDefaultTimeoutSec};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_sub_;
  std::atomic<int64_t> last_true_ns_{0};
  std::atomic<int64_t> last_false_ns_{0};
  std::atomic_bool holding_{false};
};

#endif  // SHUTDOWN_HOLD_GATE_HPP_
