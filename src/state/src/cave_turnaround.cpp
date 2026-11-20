#include "rover_state/mission_node.hpp"
#include <algorithm>
#include <cmath>

namespace rover_state {
void MissionNode::clear_dead_end() {
  dead_end_since_.reset();dead_end_received_.reset();
  dead_end_first_stamp_=dead_end_last_stamp_=0;turnaround_hold_=false;
}

void MissionNode::dead_end_observation(const std::string &message) {
  if(state_!=State::EXPLORING || !turnaround_enabled_ || turnaround_done_ || exit_latched_)return;
  try {
    const auto j=nlohmann::json::parse(message);
    const double source=j.at("observation_stamp").get<double>(),sent=j.at("stamp").get<double>();
    const double now_stamp=stamp();
    if(j.at("frame").get<std::string>()!=world_ || !std::isfinite(source)||!std::isfinite(sent)||
       source<phase_stamp_ || source>now_stamp+.15 || now_stamp-source>grid_timeout_ ||
       sent<source || sent>now_stamp+.15 || now_stamp-sent>grid_timeout_ ||
       !j.at("closed").get<bool>()) {clear_dead_end();return;}
    const auto &p=j.at("position");
    Pose wall{{p.at(0).get<double>(),p.at(1).get<double>(),0},j.at("yaw").get<double>()};
    const double width=j.at("width").get<double>();
    if(!finite(wall.p)||!std::isfinite(wall.yaw)||!std::isfinite(width)||width<=0) {
      clear_dead_end();return;
    }
    if(source<=dead_end_last_stamp_)return; // cached maps cannot extend confirmation
    if(dead_end_since_ && (!age_ok(dead_end_received_,grid_timeout_) ||
       source-dead_end_last_stamp_>grid_timeout_ || distance(wall.p,dead_end_anchor_.p)>.20 ||
       std::abs(angle(wall.yaw-dead_end_anchor_.yaw))>.15 || std::abs(width-dead_end_width_)>.15))
      clear_dead_end();
    if(!dead_end_since_) {
      dead_end_since_=Clock::now();dead_end_first_stamp_=source;
      dead_end_anchor_=wall;dead_end_width_=width;
      RCLCPP_INFO(get_logger(),"Observed closed corridor at (%.2f, %.2f); holding for %.1f s confirmation",
          wall.p.x,wall.p.y,turnaround_confirmation_);
    }
    dead_end_received_=Clock::now();dead_end_last_stamp_=source;turnaround_hold_=true;
  } catch(const std::exception &) {clear_dead_end();}
}

bool MissionNode::check_dead_end() {
  if(!turnaround_enabled_||turnaround_done_||exit_latched_||!arm_ready()||
     follower_state_.value("exit_forward",false) || !age_ok(dead_end_received_,grid_timeout_) ||
     stamp()-dead_end_last_stamp_>grid_timeout_ || stamp()<dead_end_last_stamp_-.15) {
    clear_dead_end();return false;
  }
  if(!dead_end_since_ || age(*dead_end_since_)<turnaround_confirmation_ ||
     dead_end_last_stamp_-dead_end_first_stamp_<turnaround_confirmation_)return false;
  command_exploration(false);cancel();
  turnaround_active_=true;turnaround_started_=turnaround_rotated_=false;
  turnaround_progress_=0;turnaround_reoriented_.reset();turnaround_reorient_pending_=false;
  hints_.clear();centers_.clear();tracks_.clear();pending_targets_.clear();
  transition(State::TURNAROUND_HANDOFF,"Corridor end wall confirmed continuously; stopping navigation before turnaround");
  return true;
}

bool MissionNode::turnaround_pose_clear(Pose p) const {
  if(!finite(p.p)||!std::isfinite(p.yaw))return false;
  const auto &f=turnaround_footprint_;
  return global_grid_.centre_free(p.p,true)&&local_grid_.centre_free(p.p,true)&&
      global_grid_.rectangle_free(p,f[0],f[1],f[2],f[3],true)&&
      local_grid_.rectangle_free(p,f[0],f[1],f[2],f[3],true);
}

bool MissionNode::turnaround_sweep_clear(double turn) const {
  const int steps=std::max(1,static_cast<int>(std::ceil(std::abs(turn)/.02)));
  for(int i=0;i<=steps;++i) {
    Pose p=robot_;p.yaw+=turn*i/steps;
    if(!turnaround_pose_clear(p))return false;
  }
  return true;
}

void MissionNode::turnaround_tick() {
  geometry_msgs::msg::Twist command;
  if(!turnaround_active_){fault("Turnaround state lost its active operation");return;}
  if(!arm_ready()||!age_ok(odom_received_,sensor_timeout_)) {
    rotation_pub_->publish(command);fault("Fresh odometry and safe arm required for turnaround");return;
  }
  if(state_==State::TURNAROUND_HANDOFF) {
    rotation_pub_->publish(command);
    if(age(entered_)>cancel_timeout_) {fault("Turnaround handoff did not reach a stopped, idle chassis");return;}
    if(!stopped()||!follower_idle()||!gate_locked()||!stationary())return;
    if(!collision_ready())return;
    if(!turnaround_started_) {
      const bool left=turnaround_sweep_clear(M_PI),right=turnaround_sweep_clear(-M_PI);
      if(!left&&!right){fault("Closed corridor has insufficient footprint clearance for an in-place turnaround");return;}
      turnaround_direction_=left?1.:-1.;turnaround_progress_=0;
      turnaround_origin_=robot_;turnaround_last_yaw_=robot_.yaw;turnaround_started_=true;
    } else if(!turnaround_sweep_clear(turnaround_direction_*M_PI-turnaround_progress_)) {
      fault("Remaining turnaround sweep blocked after interruption");return;
    }
    transition(State::TURNING_AROUND,"Navigation idle; rotating in place by 180 degrees");return;
  }
  if(age(entered_)>turnaround_timeout_) {
    rotation_pub_->publish(command);fault("Turnaround or corridor reorientation timed out");return;
  }
  if(!stopped()||!follower_idle()||!collision_ready()||
     distance(robot_.p,turnaround_origin_.p)>.04||!turnaround_pose_clear(robot_)) {
    rotation_pub_->publish(command);fault("Turnaround interrupted by navigation, map validity, collision or position drift");return;
  }
  const double remaining=turnaround_direction_*M_PI-turnaround_progress_;
  if(!turnaround_rotated_) {
    if(std::abs(remaining)<=rotation_tolerance_)turnaround_rotated_=true;
    else {
      if(!turnaround_sweep_clear(remaining)) {
        rotation_pub_->publish(command);fault("Updated obstacle map blocks remaining turnaround sweep");return;
      }
      command.angular.z=std::copysign(std::clamp(1.5*std::abs(remaining),rotation_min_speed_,turnaround_speed_),remaining);
    }
    rotation_pub_->publish(command);
    return;
  }
  rotation_pub_->publish(command);
  if(!gate_locked()||!stationary())return;
  if(std::abs(remaining)>rotation_tolerance_+.02) {
    fault("Turnaround heading changed while settling");return;
  }
  if(turnaround_reoriented_) {
    if(!*turnaround_reoriented_){fault("Exploration planner rejected corridor reorientation");return;}
    turnaround_done_=true;turnaround_active_=false;
    corridor_line_.clear();corridor_received_.reset();
    hints_.clear();centers_.clear();tracks_.clear();pending_targets_.clear();
    command_exploration(true);
    transition(State::RESUMING,"Turnaround complete; corridor direction reset, waiting for a fresh exploration path");return;
  }
  if(!turnaround_reorient_pending_ && reorient_client_->service_is_ready()) {
    turnaround_reorient_pending_=true;const auto request=++turnaround_request_;
    try {
      reorient_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
          [this,request](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            if(request!=turnaround_request_)return;
            turnaround_reorient_pending_=false;
            try {turnaround_reoriented_=future.get()->success;}
            catch(const std::exception &){turnaround_reoriented_=false;}
          });
    } catch(const std::exception &) {turnaround_reorient_pending_=false;turnaround_reoriented_=false;}
  }
}
} // namespace rover_state
