#include "rover_state/mission_node.hpp"
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rover_state {
namespace {
double yaw_of(const geometry_msgs::msg::Quaternion &q) {
  const double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if(!std::isfinite(n)||std::abs(n-1)>.05)throw std::invalid_argument("Invalid planar orientation");
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
geometry_msgs::msg::PoseStamped stamped(Pose pose,const std::string &frame,rclcpp::Time time) {
  geometry_msgs::msg::PoseStamped out;out.header.frame_id=frame;out.header.stamp=time;
  out.pose.position.x=pose.p.x;out.pose.position.y=pose.p.y;out.pose.position.z=pose.p.z;
  out.pose.orientation.z=std::sin(pose.yaw/2);out.pose.orientation.w=std::cos(pose.yaw/2);return out;
}
}
bool MissionNode::centered_state() const {
  return state_==State::TARGET_TRACKING||state_==State::CENTER_HANDOFF||
    state_==State::CENTER_SETTLING||state_==State::CENTER_OBSERVING||state_==State::CENTER_REFOLDING||state_==State::ALIGN_TARGET||state_==State::CENTER_RETURN_WAIT||state_==State::RESTORE_CORRIDOR;
}
bool MissionNode::rotation_state() const {return state_==State::ALIGN_TARGET||state_==State::RESTORE_CORRIDOR;}
void MissionNode::corridor(const nav_msgs::msg::Path &message) {
  const double t=rclcpp::Time(message.header.stamp).seconds();
  if(message.header.frame_id!=world_ || stamp()-t>centerline_timeout_ || stamp()-t<-.15 || message.poses.size()>1000)return;
  std::vector<Pose> points;
  try {
    for(const auto &p:message.poses) {
      Point point{p.pose.position.x,p.pose.position.y,p.pose.position.z};
      if(!finite(point))return;
      points.push_back({point,yaw_of(p.pose.orientation)});
    }
  } catch(const std::exception&){return;}
  corridor_line_=std::move(points);corridor_received_=Clock::now();corridor_stamp_=t;
}
bool MissionNode::collision_ready() {
  if(!global_map_||!local_map_)return false;
  const auto fresh=[this](const auto &m){const double age=stamp()-rclcpp::Time(m.header.stamp).seconds();return age>=-.15&&age<=grid_timeout_;};
  if(!fresh(*global_map_)||!fresh(*local_map_))return false;
  const auto origin=[this](const auto &header,const geometry_msgs::msg::Pose &p) {
    auto in=stamped({{p.position.x,p.position.y,p.position.z},yaw_of(p.orientation)},header.frame_id,now()),out=in;
    if(header.frame_id!=world_) {
      const auto tf=tf_->lookupTransform(world_,header.frame_id,tf2::TimePointZero);
      const double t=rclcpp::Time(tf.header.stamp).seconds();
      if(t!=0 && (stamp()-t>grid_timeout_||stamp()-t<-.15))throw std::runtime_error("Stale collision-map transform");
      tf2::doTransform(in,out,tf);
    }
    return Pose{{out.pose.position.x,out.pose.position.y,out.pose.position.z},yaw_of(out.pose.orientation)};
  };
  try {
    const auto &g=*global_map_;const auto &l=*local_map_;
    if(g.header.frame_id.empty()||l.header.frame_id.empty()||!std::isfinite(g.info.resolution)||g.info.resolution<=0||
       !std::isfinite(l.metadata.resolution)||l.metadata.resolution<=0||g.info.width==0||g.info.height==0||
       l.metadata.size_x==0||l.metadata.size_y==0||g.data.size()!=size_t(g.info.width)*g.info.height||
       l.data.size()!=size_t(l.metadata.size_x)*l.metadata.size_y)return false;
    if(cached_global_map_!=global_map_) {
      global_grid_.resolution=g.info.resolution;global_grid_.width=g.info.width;global_grid_.height=g.info.height;
      global_grid_.cells.resize(g.data.size());
      for(size_t i=0;i<g.data.size();++i)global_grid_.cells[i]=g.data[i]<0 ? 3 : (g.data[i]>=100 ? 2 : (g.data[i]>=99 ? 1 : 0));
      cached_global_map_=global_map_;
    }
    if(cached_local_map_!=local_map_) {
      local_grid_.resolution=l.metadata.resolution;local_grid_.width=l.metadata.size_x;local_grid_.height=l.metadata.size_y;
      local_grid_.cells.resize(l.data.size());
      for(size_t i=0;i<l.data.size();++i)local_grid_.cells[i]=l.data[i]>=255 ? 3 : (l.data[i]>=254 ? 2 : (l.data[i]>=253 ? 1 : 0));
      cached_local_map_=local_map_;
    }
    global_grid_.origin=origin(g.header,g.info.origin);local_grid_.origin=origin(l.header,l.metadata.origin);
    return finite(global_grid_.origin.p)&&finite(local_grid_.origin.p);
  }catch(const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Centered grasp waits for collision maps: %s",e.what());return false;
  }
}
bool MissionNode::pose_clear(Pose p) const {
  return finite(p.p)&&std::isfinite(p.yaw)&&global_grid_.centre_free(p.p,true)&&local_grid_.centre_free(p.p,false)&&
    global_grid_.rectangle_free(p,front_,body_rear_,body_half_width_,body_margin_,true)&&
    local_grid_.rectangle_free(p,front_,body_rear_,body_half_width_,body_margin_,false);
}
void MissionNode::collect_target(Point p,double t,bool precise) {
  tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[this](const Track &v){return age(v.seen)>target_timeout_;}),tracks_.end());
  size_t index=tracks_.size();double best=association_;
  for(size_t i=0;i<tracks_.size();++i) {
    const auto &samples=tracks_[i].hints.empty()?tracks_[i].centres:tracks_[i].hints;
    if(!samples.empty() && distance(p,samples.back().pose.p)<best){best=distance(p,samples.back().pose.p);index=i;}
  }
  if(index==tracks_.size()) {
    if(tracks_.size()>=32)tracks_.erase(tracks_.begin());
    tracks_.push_back({{}, {}, {}, Clock::now()});index=tracks_.size()-1;
  }
  auto &track=tracks_[index];auto &samples=precise ? track.centres : track.hints;
  if(!samples.empty()&&t<=samples.back().stamp)return;
  if(!samples.empty() && t-samples.back().stamp>max_sample_gap_)samples.clear();
  samples.push_back({t,{p,0}});
  while(samples.size()>3 && t-samples[1].stamp>=confirmation_)samples.pop_front();
  track.seen=Clock::now();if(precise)track.centre_received=Clock::now();
}
void MissionNode::exploration_route(const nav_msgs::msg::Path &message) {
  if(message.poses.size()<2||message.poses.size()>20000||message.header.frame_id.empty())return;
  try {
    auto path=message;geometry_msgs::msg::TransformStamped transform;
    if(message.header.frame_id!=world_)transform=tf_->lookupTransform(world_,message.header.frame_id,tf2::TimePointZero);
    std::vector<Pose> points;
    for(auto pose:path.poses) {
      if(!pose.header.frame_id.empty()&&pose.header.frame_id!=message.header.frame_id)return;
      if(message.header.frame_id!=world_)tf2::doTransform(pose,pose,transform);
      Point p{pose.pose.position.x,pose.pose.position.y,pose.pose.position.z};
      if(!finite(p))return;
      if(points.empty()||distance(points.back().p,p)>1e-5)points.push_back({p,0});
    }
    if(points.size()<2)return;
    for(size_t i=0;i+1<points.size();++i)points[i].yaw=std::atan2(points[i+1].p.y-points[i].p.y,points[i+1].p.x-points[i].p.x);
    points.back().yaw=points[points.size()-2].yaw;exploration_route_=std::move(points);
  }catch(const std::exception &e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Accepted exploration path TF unavailable: %s",e.what());}
}
bool MissionNode::select_centered_target() {
  const bool due=age(last_grasp_diagnostic_)>=2.;
  auto report=[&](const std::string &reason) {
    grasp_diagnostics_["reason"]=reason;
    if(due){last_grasp_diagnostic_=Clock::now();RCLCPP_INFO(get_logger(),"[GRASP_SELECT] %s",grasp_diagnostics_.dump().c_str());}
  };
  grasp_diagnostics_={{"route_points",exploration_route_.size()},{"tracks",tracks_.size()},
    {"route_ready",follower_state_.value("route_ready",false)},
    {"few_samples",0},{"short_window",0},{"stale",0},{"unstable",0},{"ignored",0},
    {"behind_vehicle",0}};
  if(!follower_running()) {report("waiting_for_exploration");return false;}
  double nearest=1e9;std::optional<size_t> chosen;Point point;
  auto reject=[&](const char *key){grasp_diagnostics_[key]=grasp_diagnostics_[key].get<int>()+1;};
  for(size_t i=0;i<tracks_.size();++i) {
    const auto &samples=tracks_[i].hints;
    grasp_diagnostics_["last_samples"]=samples.size();
    if(samples.size()<3){reject("few_samples");continue;}
    grasp_diagnostics_["last_window"]=samples.back().stamp-samples.front().stamp;
    grasp_diagnostics_["last_age"]=stamp()-samples.back().stamp;
    if(samples.back().stamp-samples.front().stamp<confirmation_){reject("short_window");continue;}
    if(stamp()-samples.back().stamp>sensor_timeout_){reject("stale");continue;}
    Point mean{};for(const auto &v:samples){mean.x+=v.pose.p.x;mean.y+=v.pose.p.y;mean.z+=v.pose.p.z;}
    mean.x/=samples.size();mean.y/=samples.size();mean.z/=samples.size();
    double spread=0;for(const auto &v:samples)spread=std::max(spread,std::max(distance(v.pose.p,mean),std::abs(v.pose.p.z-mean.z)));
    const auto relative=alignment(robot_,mean,front_);
    grasp_diagnostics_["last_target"]={mean.x,mean.y,mean.z};
    grasp_diagnostics_["last_front_xy"]={relative.gap,relative.lateral};
    grasp_diagnostics_["last_spread"]=spread;
    if(spread>target_spread_){reject("unstable");continue;}
    if(ignored(mean)){reject("ignored");continue;}
    if(relative.gap<=0){reject("behind_vehicle");continue;}
    if(distance(mean,robot_.p)<nearest){nearest=distance(mean,robot_.p);chosen=i;point=mean;}
  }
  if(!chosen){report("no_lockable_target");return false;}
  report("locked");
  locked_=point;hints_=tracks_[*chosen].hints;centers_=tracks_[*chosen].centres;
  grasp_identity_anchor_=point;
  grasp_alignment_recoveries_=0;
  grasp_precise_repositions_=0;grasp_precise_position_=false;
  grasp_precise_approach_=false;
  center_received_=tracks_[*chosen].centre_received;locked_seen_=tracks_[*chosen].seen;
  pending_targets_.clear();
  RCLCPP_INFO(get_logger(),"[GRASP_LOCK] fixed world target=(%.3f,%.3f,%.3f); no further camera position updates required",
      locked_.x,locked_.y,locked_.z);
  preview_complete_=false;preview_seen_=false;grasp_after_refold_=true;refold_attempts_=0;
  centered_active_=true;attempts_=0;locked_corridor_=corridor_line_.size()>=2?corridor_line_:exploration_route_;
  grasp_adjust_attempts_=0;grasp_adjust_used_=0;corridor_restore_attempts_=0;
  preview_searches_=0;preview_target_.reset();
  transition(State::TARGET_TRACKING,"Stable target position frozen; continuing the accepted exploration path");
  return true;
}
void MissionNode::track_grasp_target() {
  if(corridor_line_.size()>=2)locked_corridor_=corridor_line_;
  else if(exploration_route_.size()>=2)locked_corridor_=exploration_route_;
  const auto release=[this](const std::string &why) {
    handled_.push_back({locked_,Clock::now()+std::chrono::seconds(30)});
    centered_active_=false;tracks_.clear();hints_.clear();centers_.clear();
    transition(State::EXPLORING,why);
  };
  if(exit_latched_||follower_state_.value("exit_forward",false)||follower_state_.value("exit_complete",false)) {
    release("Exit phase takes priority over target tracking");return;
  }
  if(age(entered_)>target_tracking_timeout_) {
    release("Fixed-target approach timed out; continuing exploration");return;
  }
  // Exploration retains its existing path and recovery. No future parking-pose search.
  const auto relative=alignment(robot_,locked_,front_);
  grasp_diagnostics_["forward_distance"]=relative.gap;
  grasp_diagnostics_["lateral_distance"]=relative.lateral;
  grasp_diagnostics_["stop_forward_distance"]=grasp_stop_forward_distance_;
  goal_=robot_;
  const double forward=std::max(0.,relative.gap-grasp_stop_forward_distance_);
  goal_.p.x+=forward*std::cos(robot_.yaw);goal_.p.y+=forward*std::sin(robot_.yaw);
  goal_pub_->publish(stamped(goal_,world_,now()));
  // Stop at the configured measured distance, without predictive braking.
  grasp_diagnostics_["stop_allowance"]=0.;
  if(relative.gap<=grasp_stop_forward_distance_) {
    corridor_yaw_=robot_.yaw;
    command_exploration(false);
    RCLCPP_INFO(get_logger(),"[GRASP_STOP] forward=%.3f m lateral=%.3f m threshold=%.3f m; no braking-distance compensation",
      relative.gap,relative.lateral,grasp_stop_forward_distance_);
    transition(State::CENTER_HANDOFF,"Configured front-edge target distance reached; stopping exploration");
  }
}
void MissionNode::rotate_centered(double yaw,bool restore) {
  if(!stopped()||!follower_idle())return;
  if(restore && corridor_restore_attempts_++>=2) {resume_centered("Returning heading alignment to exploration after bounded retries");return;}
  if(!restore && attempts_++>=max_attempts_) {return_to_corridor("Target alignment retries exhausted; continuing exploration",true);return;}
  if(!collision_ready()||!sweep_clear(robot_,yaw,[this](Pose p){return pose_clear(p);})) {
    if(restore)resume_centered("Heading restoration blocked; returning control to exploration navigation");
    else return_to_corridor("Target alignment sweep blocked",true);
    return;
  }
  rotation_origin_=robot_;goal_=robot_;goal_.yaw=yaw;
  // All navigation requests are terminal before this bounded closed-loop turn.
  // Use the gate's dedicated input so Nav2 smoother zeroes cannot overwrite it.
  nav_success_.reset();
  goal_pub_->publish(stamped(goal_,world_,now()));
  transition(restore ? State::RESTORE_CORRIDOR : State::ALIGN_TARGET,
      restore ? "Restoring local corridor tangent" : "Small bounded yaw adjustment inside the grasp workspace");
}
void MissionNode::return_to_corridor(const std::string &reason,bool mark_target) {
  command_exploration(false);cancel();
  if(mark_target)handled_.push_back({locked_,Clock::now()+std::chrono::seconds(30)});
  hints_.clear();centers_.clear();tracks_.clear();
  transition(State::CENTER_RETURN_WAIT,reason);
}
void MissionNode::begin_centered_grasp() {
  // Observe first at the actual stopped pose, before deciding reachability.
  if(!grasp_enabled_){transition(State::READY_TO_GRASP,"Stopped beside the fixed target; arm execution disabled");return;}
  // Standalone launch owns observation, precise registration and arm motion.
  // No mission perception target, camera TF or pose-identity gate is supplied.
  begin_arm("grasp");
  transition(State::GRASPING,"Position adjustment complete; running standalone bring_launch grasp sequence");
}

void MissionNode::resume_centered(const std::string &reason) {
  centered_active_=false;locked_corridor_.clear();tracks_.clear();hints_.clear();centers_.clear();
  command_exploration(true);transition(State::RESUMING,reason);
}
bool MissionNode::retry_positioned_preview() {
  // Position correction is computed from the original target and current pose,
  // independently of which other blocks the camera can currently see.
  if(preview_searches_>=2)return false;
  ++preview_searches_;preview_complete_=false;
  if(try_nudge()) {
    RCLCPP_INFO(get_logger(),"[GRASP_REACQUIRE] re-observe original target after feedback distance correction (%d/2)",preview_searches_);
    return true;
  }
  RCLCPP_INFO(get_logger(),"[GRASP_REACQUIRE] no distance move selected; retain original target and re-observe without blind reverse (%d/2)",preview_searches_);
  transition(State::CENTER_HANDOFF,"Rechecking original target visibility from the current position");return true;
}

void MissionNode::tick_centered() {
  if(state_==State::TARGET_TRACKING){track_grasp_target();return;}
  if(state_==State::CENTER_HANDOFF) {
    if(stopped()&&follower_idle()&&gate_locked()&&stationary()) {
      const auto reference=corridor_projection(locked_corridor_,robot_.p);
      corridor_yaw_=reference ? reference->yaw : robot_.yaw;
      transition(State::CENTER_SETTLING,"Stopped on exploration path; checking the fixed target against the grasp sector");
    }
    else if(age(entered_)>cancel_timeout_) {
      cancel();
      if(stopped() && follower_idle() && arm_ready())
        resume_centered("Standstill confirmation delayed; handing control back to exploration");
      else if(age(entered_)>cancel_timeout_+preparation_timeout_)
        fault("Navigation cancellation or arm readiness could not complete");
    }
    return;
  }
  if(state_==State::CENTER_OBSERVING) {
    if(!arm_result_complete_) {
      if(age(entered_)>observe_move_timeout_)return_to_corridor("Observe motion timed out; recovering before continuing",true);
      return;
    }
    if(!observe_done_) {
      observe_done_=Clock::now();pending_targets_.clear();
    }
    if(arm_result_success_ && !preview_seen_ && age(*observe_done_)<preview_wait_)return;
    RCLCPP_INFO(get_logger(),"[GRASP_PREVIEW] visible=%d observe_success=%d; refolding before precise grasp",
      preview_seen_,arm_result_success_);
    begin_arm("prepare");refold_attempts_=1;
    transition(State::CENTER_REFOLDING,"Preview complete; refolding before the precise observation and grasp");return;
  }
  if(state_==State::CENTER_REFOLDING) {
    if(arm_result_complete_ && arm_ready()) {
      if(grasp_after_refold_) {
        if(!preview_seen_ && retry_positioned_preview())return;
        if(preview_seen_ && preview_target_) {
          locked_=*preview_target_;
          RCLCPP_INFO(get_logger(),"[GRASP_PREVIEW_LOCK] original block re-observed at (%.3f,%.3f,%.3f); one stopped-pose refinement",locked_.x,locked_.y,locked_.z);
        }
        preview_complete_=true;
        transition(State::CENTER_SETTLING,"Observation and refold complete; checking bounded heading adjustment");
      }
      else return_to_corridor("Arm recovered; continuing exploration",false);
      return;
    }
    if(arm_result_complete_ && !arm_result_success_ && refold_attempts_<2) {
      ++refold_attempts_;begin_arm("prepare");return;
    }
    if(age(entered_)>preparation_timeout_)fault("Arm refolding could not be confirmed after retries");
    return;
  }
  if(state_==State::CENTER_RETURN_WAIT) {
    if(age(entered_)>75.) {
      if(stopped()&&follower_idle()&&arm_ready())resume_centered("Grasp recovery complete; exploration takes over alignment");
      else fault("Grasp cleanup could not confirm navigation/arm readiness");
      return;
    }
    if(!stopped()||!follower_idle())return;
    if(!arm_ready()) {
      const auto phase=arm_state_.value("phase","");
      if(arm_result_complete_ && (phase=="READY"||phase=="ERROR") && age_ok(arm_received_,runtime_grace_)) {
        grasp_after_refold_=false;refold_attempts_=1;begin_arm("prepare");
        transition(State::CENTER_REFOLDING,"Recovering the arm before returning to exploration");return;
      }
      if(age(entered_)>75.)fault("Arm return could not be confirmed after bounded cleanup");
      return;
    }
    if(!gate_locked()||!stationary())return;
    const auto reference=corridor_projection(locked_corridor_,robot_.p);
    if(!reference||reference->distance>centered_parameters_.maximum_offset) {
      resume_centered("Resuming exploration from the actual vehicle pose");return;
    }
    if(!collision_ready()||!pose_clear(robot_)) {
      resume_centered("Exploration navigation will replan from the current pose");return;
    }
    if(std::abs(angle(robot_.yaw-reference->yaw))>rotation_tolerance_) {rotate_centered(reference->yaw,true);return;}
    resume_centered("Arm returned and corridor heading restored; continuing centreline exploration");return;
  }
  if(rotation_state() && (!collision_ready()||!pose_clear(robot_))) {
    if(state_==State::RESTORE_CORRIDOR)resume_centered("Returning heading control to exploration after a map update");
    else return_to_corridor("Collision-map validity lost during centred approach/alignment",true);
    return;
  }
  if(rotation_state()) {
    if(distance(robot_.p,rotation_origin_.p)>.04 ||
       (state_==State::ALIGN_TARGET && std::abs(angle(robot_.yaw-corridor_yaw_))>centered_parameters_.maximum_turn+.015)) {
      if(state_==State::RESTORE_CORRIDOR)resume_centered("Position changed during heading restoration; resuming exploration");
      else {
        geometry_msgs::msg::Twist stop;rotation_pub_->publish(stop);
        RCLCPP_WARN(get_logger(),"[GRASP_REALIGN] drift=%.3f m heading_error=%.3f rad recovery=%d/2",
          distance(robot_.p,rotation_origin_.p),angle(robot_.yaw-corridor_yaw_),grasp_alignment_recoveries_+1);
        if(grasp_alignment_recoveries_++<2)
          transition(State::CENTER_HANDOFF,"Alignment changed position; stopping and retrying from measured pose");
        else return_to_corridor("Target alignment recovery budget exhausted",true);
      }
      return;
    }
    const double error=angle(goal_.yaw-robot_.yaw);
    geometry_msgs::msg::Twist command;
    if(age(entered_)>rotation_timeout_) {
      if(state_==State::RESTORE_CORRIDOR)resume_centered("Heading adjustment timed out; exploration controller will align the vehicle");
      else if(grasp_alignment_recoveries_++<2)
        transition(State::CENTER_HANDOFF,"Alignment timed out; stopping and re-evaluating target");
      else return_to_corridor("Target alignment recovery budget exhausted",true);
    } else if(std::abs(error)<=rotation_tolerance_) {
      if(state_==State::RESTORE_CORRIDOR)transition(State::CENTER_RETURN_WAIT,"Corridor heading reached; checking standstill");
      else transition(State::CENTER_SETTLING,"Target heading reached; checking the fixed target coordinates");
    } else command.angular.z=std::copysign(
        std::clamp(1.5*std::abs(error),rotation_min_speed_,rotation_speed_),error);
    rotation_pub_->publish(command);
  } else if(state_==State::CENTER_SETTLING) {
    if(age(entered_)>centered_localize_timeout_){return_to_corridor("Stopped-pose target feedback or readiness confirmation timed out",true);return;}
    if(!stopped()||!follower_idle()||!gate_locked()||!stationary()||!arm_ready())return;
    // Require observations acquired after this stop, not cached driving samples.
    auto refine=[this](const auto &samples,const auto &received)->std::optional<Point> {
      if(!age_ok(received,sensor_timeout_) || samples.size()<3 ||
         samples.front().stamp < stamp()-age(entered_) ||
         samples.back().stamp-samples.front().stamp<confirmation_)return {};
      Point mean{};
      for(const auto &s:samples){mean.x+=s.pose.p.x;mean.y+=s.pose.p.y;mean.z+=s.pose.p.z;}
      mean.x/=samples.size();mean.y/=samples.size();mean.z/=samples.size();
      for(const auto &s:samples)
        if(distance(mean,s.pose.p)>target_spread_ || std::abs(mean.z-s.pose.p.z)>target_spread_)return {};
      return mean;
    };
    auto measurement=refine(centers_,center_received_);
    const bool precise=bool(measurement);
    if(!measurement)measurement=refine(hints_,hint_received_);
    // A weak folded-camera view must not prevent the standalone arm sequence
    // from moving to its observation pose and acquiring its own precise target.
    // The retained world lock is used only for bounded chassis positioning.
    if(!measurement && !grasp_precise_position_ && age(entered_)<2.)return;
    if(measurement && !grasp_precise_position_)locked_=*measurement;
    const char *source=grasp_precise_position_?"arm_precise":measurement?(precise?"precise":"depth"):"retained_lock";
    grasp_diagnostics_["stopped_feedback"]={{"source",source},
      {"world",{locked_.x,locked_.y,locked_.z}},{"anchor_error",distance(locked_,grasp_identity_anchor_)}};
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
      "[GRASP_FEEDBACK] source=%s world=(%.3f,%.3f,%.3f) forward=%.3f lateral=%.3f",
      source,locked_.x,locked_.y,locked_.z,
      alignment(robot_,locked_,front_).gap,alignment(robot_,locked_,front_).lateral);
    const auto reference=corridor_projection(locked_corridor_,robot_.p);
    corridor_yaw_=reference ? reference->yaw : robot_.yaw;
    if(!collision_ready() || !pose_clear(robot_))return;
    if(grasp_precise_approach_) {
      const auto correction=grasp_distance_correction(robot_,locked_,centered_parameters_,.015);
      if(correction && *correction>std::max(.015,nudge_tolerance_) && try_nudge())return;
      if(!correction || std::abs(*correction)>std::max(.015,nudge_tolerance_)) {
        return_to_corridor("Closer grasp pose unavailable within collision/movement limits",true);return;
      }
      grasp_precise_approach_=false;
    }
    // A depth-only estimate does not force movement inside the grasp sector;
    // precise approach requests above deliberately take priority over this.
    if(aligned()){begin_centered_grasp();return;}
    // Being inside the sector is not enough: first improve a side-biased target
    // toward the camera/gripper centre, then correct the longitudinal distance.
    const auto relative=alignment(robot_,locked_,front_);
    const auto adjustment=joint_grasp_adjustment(robot_,corridor_yaw_,locked_,centered_parameters_,
      grasp_adjust_max_-grasp_adjust_used_,[this](Pose p){return pose_clear(p);});
    if(adjustment) {
      Pose candidate=robot_;candidate.yaw=adjustment->yaw;
      const double lateral=std::abs(alignment(candidate,locked_,front_).lateral);
      grasp_diagnostics_["joint_adjustment"]={{"yaw",adjustment->yaw},{"translation",adjustment->translation},
        {"candidate_lateral",lateral},{"budget_remaining",grasp_adjust_max_-grasp_adjust_used_}};
      if(std::abs(relative.lateral)>grasp_lateral_tolerance_ &&
         lateral+.01<std::abs(relative.lateral) &&
         std::abs(angle(adjustment->yaw-robot_.yaw))>rotation_tolerance_ && attempts_<max_attempts_) {
        RCLCPP_INFO(get_logger(),"[GRASP_CENTER] lateral=%.3f -> %.3f m yaw_change=%.1f deg",
          relative.lateral,lateral,angle(adjustment->yaw-robot_.yaw)*180/M_PI);
        rotate_centered(adjustment->yaw,false);return;
      }
    }
    // An unreachable centre must not discard an otherwise valid grasp. After
    // bounded centering attempts, try from the best safe pose inside the sector.
    if(try_nudge())return;
    if(aligned()){begin_centered_grasp();return;}
    if(!adjustment){return_to_corridor("No collision-free bounded grasp position and heading",true);return;}
    if(std::abs(angle(adjustment->yaw-robot_.yaw))>rotation_tolerance_ && attempts_<max_attempts_) {
      rotate_centered(adjustment->yaw,false);return;
    }
    if(!collision_ready()){return_to_corridor("Heading adjustment unavailable after observation; continuing exploration",true);return;}
    const auto yaw=minimum_grasp_yaw(robot_,corridor_yaw_,locked_,centered_parameters_,[this](Pose p){return pose_clear(p);});
    if(!yaw){return_to_corridor("Target outside sector after stopping; continuing exploration",true);return;}
    if(std::abs(angle(*yaw-robot_.yaw))>rotation_tolerance_) {rotate_centered(*yaw,false);return;}
    begin_centered_grasp();
  }
}
} // namespace rover_state
