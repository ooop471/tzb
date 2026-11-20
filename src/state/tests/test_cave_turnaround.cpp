#include "rover_state/mission_node.hpp"
#include <cassert>
#include <iostream>
#include <thread>

namespace rover_state {
struct TurnaroundTest {
  static void run() {
    auto node=std::make_shared<MissionNode>();auto &n=*node;n.timer_->cancel();
    using S=MissionNode::State;
    n.cave_only_=true;n.robot_={{0,0,0},3.05};n.state_=S::EXPLORING;n.phase_stamp_=n.stamp()-1;
    n.follower_state_=nlohmann::json::object();
    assert(n.turnaround_confirmation_==10.);
    const auto observation=[&](double source,bool closed=true,double x=1.8){
      return nlohmann::json({{"stamp",n.stamp()},{"observation_stamp",source},{"frame",n.world_},
        {"closed",closed},{"position",{x,0.}},{"yaw",0.},{"width",1.2}}).dump();
    };
    n.dead_end_observation(observation(n.stamp()));assert(n.turnaround_hold_&&n.dead_end_since_);
    assert(!n.check_dead_end());
    n.dead_end_since_=MissionNode::Clock::now()-std::chrono::seconds(11);
    n.dead_end_observation(observation(n.dead_end_last_stamp_));
    assert(!n.check_dead_end()); // duplicate source cannot accumulate ten seconds
    n.dead_end_first_stamp_=n.dead_end_last_stamp_-9.9;assert(!n.check_dead_end());
    n.dead_end_first_stamp_=n.dead_end_last_stamp_-10.1;
    n.budget_.start(MissionNode::Clock::now()-std::chrono::seconds(601));
    assert(n.check_dead_end());assert(n.state_==S::TURNAROUND_HANDOFF&&n.turnaround_active_&&!n.explore_enabled_);

    n.turnaround_active_=false;n.transition(S::EXPLORING,"fixture");
    n.phase_stamp_=n.stamp()-1;n.dead_end_observation(observation(n.stamp()));
    assert(n.turnaround_hold_);
    n.transition(S::CENTER_OBSERVING,"grasp observation");
    assert(!n.dead_end_since_&&!n.turnaround_hold_);
    n.dead_end_observation(observation(n.stamp()));assert(!n.dead_end_since_);
    n.transition(S::EXPLORING,"resume after grasp");
    n.dead_end_observation(observation(n.phase_stamp_-.01));assert(!n.dead_end_since_);
    n.dead_end_observation(observation(n.stamp()));assert(n.dead_end_since_);
    n.dead_end_observation(observation(n.stamp(),false));assert(!n.dead_end_since_);
    n.dead_end_observation(observation(n.stamp()));
    n.dead_end_observation(observation(n.stamp(),true,2.5));
    assert(n.dead_end_anchor_.p.x==2.5&&n.dead_end_first_stamp_==n.dead_end_last_stamp_);
    n.dead_end_observation("{}");assert(!n.turnaround_hold_);
    n.phase_stamp_=n.stamp()-1;n.dead_end_observation(observation(n.stamp()));
    n.dead_end_received_=MissionNode::Clock::now()-std::chrono::seconds(3);
    assert(!n.check_dead_end()&&!n.turnaround_hold_);

    auto global=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    auto local=std::make_shared<nav2_msgs::msg::Costmap>();
    global->header.frame_id=local->header.frame_id=n.world_;
    global->header.stamp=local->header.stamp=n.now();
    global->info.resolution=local->metadata.resolution=.05;
    global->info.width=global->info.height=local->metadata.size_x=local->metadata.size_y=160;
    global->info.origin.position.x=global->info.origin.position.y=-4;
    local->metadata.origin=global->info.origin;
    global->info.origin.orientation.w=local->metadata.origin.orientation.w=1;
    global->data.assign(160*160,0);local->data.assign(160*160,0);
    n.global_map_=global;n.local_map_=local;assert(n.collision_ready());
    n.robot_.yaw=0;
    assert(n.turnaround_pose_clear(n.robot_)&&n.turnaround_sweep_clear(M_PI));
    // A narrow corridor permits driving but the rear corners cannot sweep through it.
    for(int x=0;x<160;++x)for(int y:{69,90}) {
      global->data[y*160+x]=100;local->data[y*160+x]=254;
    }
    n.cached_global_map_.reset();n.cached_local_map_.reset();assert(n.collision_ready());
    assert(n.turnaround_pose_clear(n.robot_)&&!n.turnaround_sweep_clear(M_PI));
    global->data.assign(160*160,0);local->data.assign(160*160,0);
    n.cached_global_map_.reset();n.cached_local_map_.reset();assert(n.collision_ready());
    const auto ready=[&] {
      n.odom_received_=n.follower_received_=n.gate_received_=MissionNode::Clock::now();
      n.follower_state_={{"session",n.session_},{"generation",n.generation_},{"idle",true},{"enabled",false}};
      n.gate_state_={{"session",n.session_},{"allow",false},{"zero",true}};
      const double t=n.stamp();n.odom_history_.clear();
      for(int i=4;i>=0;--i)n.odom_history_.push_back({t-i*.1,n.robot_});
    };
    n.robot_.yaw=3.05;n.turnaround_active_=true;n.turnaround_started_=n.turnaround_rotated_=false;
    n.transition(S::TURNAROUND_HANDOFF,"fixture");ready();n.turnaround_tick();
    assert(n.state_==S::TURNING_AROUND&&n.turnaround_started_);
    nav_msgs::msg::Odometry odom;odom.header.frame_id=n.world_;odom.child_frame_id=n.base_;
    const double start=n.stamp();
    for(int i=1;i<=200;++i) {
      const double yaw=3.05+M_PI*i/200;
      odom.header.stamp=rclcpp::Time(static_cast<int64_t>((start+i*.0001)*1e9));
      odom.pose.pose.orientation.z=std::sin(yaw/2);odom.pose.pose.orientation.w=std::cos(yaw/2);
      n.odometry(odom);
    }
    assert(std::abs(n.turnaround_progress_-M_PI)<1e-8); // crosses +/-pi without switching direction
    ready();n.turnaround_tick();assert(n.turnaround_rotated_);
    n.transition(S::PAUSED,"pause after rotation");
    assert(n.turnaround_active_&&n.turnaround_rotated_);
    n.transition(S::TURNAROUND_HANDOFF,"resume fixture");ready();n.turnaround_tick();
    assert(n.state_==S::TURNING_AROUND);
    auto fixture=std::make_shared<rclcpp::Node>("turnaround_reorientation_fixture");
    int requests=0;
    auto service=fixture->create_service<std_srvs::srv::Trigger>("/explore/reorient_corridor",
      [&](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response) {
        assert(n.turnaround_rotated_&&!n.explore_enabled_&&n.state_==S::TURNING_AROUND);
        ++requests;response->success=true;
      });
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);executor.add_node(fixture);
    const auto until=[&](auto predicate) {
      const auto deadline=MissionNode::Clock::now()+std::chrono::seconds(2);
      while(!predicate()&&MissionNode::Clock::now()<deadline) {
        executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      assert(predicate());
    };
    until([&]{return n.reorient_client_->service_is_ready();});
    ready();n.turnaround_tick();assert(n.turnaround_reorient_pending_&&!n.explore_enabled_);
    until([&]{return n.turnaround_reoriented_.has_value();});
    assert(requests==1);ready();n.turnaround_tick();
    assert(n.state_==S::RESUMING&&n.turnaround_done_&&!n.turnaround_active_&&n.explore_enabled_);
    n.transition(S::EXPLORING,"return path");
    n.dead_end_observation(observation(n.stamp()));assert(!n.turnaround_hold_); // once per cave mission
    n.turnaround_active_=true;n.turnaround_started_=true;n.turnaround_rotated_=false;
    n.turnaround_origin_=n.robot_;n.robot_.p.x+=.1;
    n.transition(S::TURNING_AROUND,"drift fixture");ready();n.turnaround_tick();assert(n.state_==S::FAULT);
    std::cout<<"PASS: 10 s source confirmation, grasp reset, stale observations, footprint sweep, yaw wrap, pause, reorientation, single turn and drift stop\n";
  }
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);rover_state::TurnaroundTest::run();rclcpp::shutdown();}
