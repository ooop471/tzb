#include "rover_state/heading_guard.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MotionGate : public rclcpp::Node {
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;

 public:
  MotionGate() : Node("mission_motion_gate") {
    require_arm_ = declare_parameter("require_arm_ready", true);
    max_linear_ = declare_parameter("max_linear_speed", .2);
    if(!std::isfinite(max_linear_) || max_linear_<=0)
      throw std::invalid_argument("Gate max_linear_speed must be finite and positive");
    lease_ = declare_parameter("lease_timeout", 2.0);
    input_timeout_ = declare_parameter("command_timeout", 0.6);
    linear_acceleration_ = declare_parameter("linear_acceleration", 0.4);
    angular_acceleration_ = declare_parameter("angular_acceleration", 1.2);
    if (!std::isfinite(lease_) || !std::isfinite(input_timeout_) || lease_ <= 0 ||
        input_timeout_ <= 0 || !std::isfinite(linear_acceleration_) || linear_acceleration_ <= 0 ||
        !std::isfinite(angular_acceleration_) || angular_acceleration_ <= 0)
      throw std::invalid_argument("Gate timeouts must be positive");
    output_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SensorDataQoS());
    status_ = create_publisher<std_msgs::msg::String>("/mission/gate_state", 10);
    input_ = create_subscription<geometry_msgs::msg::Twist>(
        "/mission/nav_cmd_vel", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          for (double x :
               {m->linear.x, m->linear.y, m->linear.z, m->angular.x, m->angular.y, m->angular.z})
            if (!std::isfinite(x)) {
              command_time_.reset();
              return;
            }
          command_ = *m;
          command_time_ = Clock::now();
        });
    recovery_input_=create_subscription<geometry_msgs::msg::Twist>(
      "/mission/recovery_cmd_vel",rclcpp::SensorDataQoS(),[this](geometry_msgs::msg::Twist::ConstSharedPtr m){
        if(!std::isfinite(m->linear.x)||!std::isfinite(m->linear.y)||!std::isfinite(m->angular.z)){recovery_time_.reset();return;}
        recovery_command_=*m;recovery_time_=Clock::now();
      });
    rotation_input_=create_subscription<geometry_msgs::msg::Twist>(
      "/mission/alignment_cmd_vel",rclcpp::SensorDataQoS(),[this](geometry_msgs::msg::Twist::ConstSharedPtr m){
        if(!std::isfinite(m->angular.z)||!std::isfinite(m->linear.x)){rotation_time_.reset();return;}
        rotation_command_=*m;rotation_time_=Clock::now();
      });
    permission_ = create_subscription<std_msgs::msg::String>(
        "/mission/motion_permission", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
          try {
            auto j = nlohmann::json::parse(m->data);
            const double age = now().seconds() - j.at("stamp").get<double>();
            const auto session = j.at("session").get<std::string>();
            const bool allow = j.at("allow").get<bool>();
            const double heading=j.value("heading_error",0.),bound=j.value("heading_limit",0.);
            if(!std::isfinite(heading)||!std::isfinite(bound)||bound<0||bound>1.57)return;
            const double linear = j.at("max_linear").get<double>(),
                         angular = j.at("max_angular").get<double>();
            if (session.empty() || !std::isfinite(age) || age < -0.15 || age > lease_ ||
                !std::isfinite(linear) || !std::isfinite(angular) || linear < 0 || (linear == 0 && !j.value("recovery_rotation",false)) || linear > max_linear_ ||
                angular <= 0 || angular > 0.8)
              return;
            if (session != session_ && allow) return;
            const bool recovery=j.value("recovery_rotation",false);
            const bool backing=j.value("recovery_backup",false);
            if(backing && (recovery || linear>.10))return;
            const bool rotation=j.value("rotation_only",false);
            if(recovery && (linear!=0 || rotation))return;
            const bool nudge=rotation && j.value("nudge",false);
            if (!allow || session != session_ || rotation!=rotation_only_ || nudge!=nudge_ || recovery!=recovery_rotation_ || backing!=recovery_backup_) {
              command_time_.reset();rotation_time_.reset();recovery_time_.reset();
            }
            session_ = session;
            allowed_ = allow;
            rotation_only_ = rotation;
            recovery_rotation_=recovery;recovery_backup_=backing;
            nudge_ = nudge;
            linear_ = linear;
            angular_ = angular;
            heading_error_=heading;heading_limit_=bound;
            permission_time_ = Clock::now();
            permission_stamp_ = j.at("stamp").get<double>();
          } catch (const std::exception &) {
          }
        });
    arm_ = create_subscription<std_msgs::msg::String>(
        "/arm_task/state", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
          try {
            auto j = nlohmann::json::parse(m->data);
            const double age = now().seconds() - j.at("stamp").get<double>();
            if (!std::isfinite(age) || age < -0.15 || age > lease_) return;
            arm_session_ = j.at("session").get<std::string>();
            arm_ready_ =
                j.at("safe_to_drive").get<bool>() && j.at("phase").get<std::string>() == "READY";
            arm_time_ = Clock::now();
            arm_stamp_ = j.at("stamp").get<double>();
          } catch (const std::exception &) {
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { publish(); });
  }

 private:
  bool fresh(std::optional<Time> t, double limit) const {
    return t && std::chrono::duration<double>(Clock::now() - *t).count() <= limit;
  }
  void publish() {
    const double ros_now = now().seconds();
    const bool allow =
        allowed_ && fresh(permission_time_, lease_) && ros_now - permission_stamp_ <= lease_ &&
        ros_now - permission_stamp_ >= -0.15 && (!require_arm_ || (arm_ready_ && fresh(arm_time_, lease_) &&
        ros_now - arm_stamp_ <= lease_ && ros_now - arm_stamp_ >= -0.15 && arm_session_ == session_));
    geometry_msgs::msg::Twist out;
    bool active_input=false;
    // Recovery has its own input: controller/smoother zeroes cannot overwrite it.
    if(allow && (recovery_rotation_||recovery_backup_) && fresh(recovery_time_,input_timeout_)) {
      active_input=true;
      if(recovery_backup_)out.linear.x=std::clamp(recovery_command_.linear.x,-linear_,0.);
      else out.angular.z=std::clamp(recovery_command_.angular.z,-angular_,angular_);
    } else if(allow && !recovery_rotation_ && !recovery_backup_ && rotation_only_ && fresh(rotation_time_,std::min(.2,input_timeout_))) {
      active_input=true;
      if(nudge_) out.linear.x=std::clamp(rotation_command_.linear.x,-linear_,linear_);  // grasp nudge: straight only
      else out.angular.z=std::clamp(rotation_command_.angular.z,-angular_,angular_);
    } else if(allow && !recovery_rotation_ && !recovery_backup_ && !rotation_only_ && fresh(command_time_,input_timeout_)) {
      active_input=true;
      const double speed=std::hypot(command_.linear.x,command_.linear.y);
      const double scale=speed>linear_ ? linear_/speed : 1.0;
      out.linear.x=command_.linear.x*scale;out.linear.y=command_.linear.y*scale;
      out.angular.z=std::clamp(command_.angular.z,-angular_,angular_);
      if(recovery_backup_){out.linear.x=std::clamp(command_.linear.x,-linear_,0.);out.linear.y=0.;out.angular.z=0.;}
    }
    if(active_input && heading_limit_>0 && !recovery_backup_) {
      if(std::abs(heading_error_)>heading_limit_)out.linear.x=out.linear.y=0.;
      out.angular.z=rover_state::corridor_angular_command(out.angular.z,heading_error_,heading_limit_,angular_);
    }
    // Task handoffs and communication gaps decelerate instead of cutting speed.
    const auto current = Clock::now();
    const double dt =
        std::clamp(std::chrono::duration<double>(current - last_output_time_).count(), 0.001, 0.1);
    const double dx = out.linear.x - last_output_.linear.x,
                 dy = out.linear.y - last_output_.linear.y;
    const double delta = std::hypot(dx, dy), step = linear_acceleration_ * dt;
    const double scale = delta > step ? step / delta : 1.0;
    out.linear.x = last_output_.linear.x + dx * scale;
    out.linear.y = last_output_.linear.y + dy * scale;
    out.angular.z = last_output_.angular.z + std::clamp(out.angular.z - last_output_.angular.z,
                                                        -angular_acceleration_ * dt,
                                                        angular_acceleration_ * dt);
    out.angular.z=std::clamp(out.angular.z,-angular_,angular_);
    if(active_input && heading_limit_>0 && !recovery_backup_)
      out.angular.z=rover_state::corridor_angular_command(out.angular.z,heading_error_,heading_limit_,angular_);
    last_output_ = out;
    last_output_time_ = current;
    output_->publish(out);
    std_msgs::msg::String state;
    state.data =
        nlohmann::json(
            {{"session", session_},
             {"stamp", now().seconds()},
             {"allow", allow},
             {"zero",
              std::abs(out.linear.x) + std::abs(out.linear.y) + std::abs(out.angular.z) < 1e-9}})
            .dump();
    status_->publish(state);
  }
  bool require_arm_{true},rotation_only_{false},nudge_{false},recovery_rotation_{false},recovery_backup_{false};
  double max_linear_{.2};
  double lease_, input_timeout_, linear_{0.15}, angular_{0.8};
  double linear_acceleration_, angular_acceleration_;
  double permission_stamp_{0}, arm_stamp_{0};
  double heading_error_{0},heading_limit_{0};
  bool allowed_{false}, arm_ready_{false};
  std::string session_, arm_session_;
  geometry_msgs::msg::Twist command_,rotation_command_;
  geometry_msgs::msg::Twist last_output_;
  Time last_output_time_{Clock::now()};
  std::optional<Time> rotation_time_, command_time_, permission_time_, arm_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr recovery_input_;
  geometry_msgs::msg::Twist recovery_command_;
  std::optional<Time> recovery_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_, rotation_input_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr permission_, arm_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotionGate>());
  } catch (const std::exception &e) {
    fprintf(stderr, "mission_motion_gate: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
