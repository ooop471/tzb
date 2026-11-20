#include "rover_state/mission_node.hpp"
#include <cassert>
#include <iostream>
namespace rover_state {
struct UnloadAlignmentTest {
  static void run() {
    auto node=std::make_shared<MissionNode>();auto &n=*node;n.timer_->cancel();
    using S=MissionNode::State;
    n.cave_only_=true;n.unload_enabled_=true;n.unload_alignment_enabled_=true;
    n.unload_zone_pose_={{0.,0.,0.},0.};n.unload_zone_valid_=true;
    n.exit_latched_=true;
    auto refresh=[&](double base_x,double yaw=0.) {
      n.robot_={{base_x,0.,0.},yaw};n.odom_received_=n.gate_received_=n.follower_received_=MissionNode::Clock::now();
      n.gate_state_={{"session",n.session_},{"allow",false},{"zero",true}};
      n.follower_state_={{"session",n.session_},{"generation",n.generation_},{"idle",true}};
      n.odom_history_.clear();const double t=n.stamp();
      for(double d:{.4,.2,0.})n.odom_history_.push_back({t-d,n.robot_});
      geometry_msgs::msg::TransformStamped tf;tf.header.frame_id=n.world_;tf.child_frame_id=n.base_;
      tf.transform.translation.x=base_x;tf.transform.rotation.z=std::sin(yaw/2);tf.transform.rotation.w=std::cos(yaw/2);
      n.tf_->setTransform(tf,"test",true);
      tf.header.frame_id=n.base_;tf.child_frame_id="base_link";
      tf.transform.translation.x=0.;tf.transform.rotation.z=0.;tf.transform.rotation.w=1.;
      n.tf_->setTransform(tf,"test",true);
      tf.header.frame_id=n.planning_frame_;tf.child_frame_id=n.world_;
      n.tf_->setTransform(tf,"test",true);
      n.update_unload_reference();
    };
    refresh(0.);assert(!n.unload_position_ok());
    assert(std::abs(n.unload_reference_world_.x+.4)<1e-9);
    n.check_unload_alignment("test");n.unload_alignment_tick();
    assert(n.state_==S::EXIT_HANDOFF&&n.unload_alignment_active_); // no premature unloading
    refresh(.4);assert(n.unload_position_ok());
    // Full TF must project a pitched vehicle's 40 cm rear offset correctly.
    geometry_msgs::msg::TransformStamped tilted;
    tilted.header.frame_id=n.world_;tilted.child_frame_id=n.base_;
    tilted.transform.translation.x=.4;tilted.transform.rotation.y=std::sin(.25);tilted.transform.rotation.w=std::cos(.25);
    n.tf_->setTransform(tilted,"test",true);n.update_unload_reference();
    assert(std::abs(n.unload_reference_world_.x-(.4-.4*std::cos(.5)))<1e-9);
    refresh(.4);
    n.odom_history_.clear();assert(!n.stationary());
    n.check_unload_alignment("test");n.unload_alignment_tick();
    assert(n.state_==S::UNLOAD_WAIT_STOP); // position suffices without odometry dwell
    n.unload_received_=MissionNode::Clock::now();
    n.unload_feedback_={{"session",n.session_},{"id",n.unload_id_},{"direction",0},
      {"active",true},{"tx_ok",true},{"lease_ok",true},{"interlock",true},{"phase_ok",true}};
    n.unload_tick();assert(n.state_==S::UNLOAD_OPENING);
    n.unload_feedback_["direction"]=-1;
    n.unload_tick();assert(n.state_==S::UNLOAD_OPENING); // stationary=false no longer aborts
    // Zero-command interlock remains required while position stays valid.
    n.gate_state_["zero"]=false;n.unload_tick();assert(n.state_==S::FAULT);
    // Position lost during unloading immediately blocks it and faults.
    n.state_=S::UNLOAD_OPENING;refresh(.8);n.unload_tick();
    assert(n.state_==S::FAULT&&!n.unload_complete_);
    // Freshness and TF validity are independently required.
    refresh(.4);n.odom_received_=MissionNode::Clock::now()-std::chrono::seconds(2);
    assert(!n.unload_position_ok());
    refresh(.4);n.unload_reference_valid_=false;assert(!n.unload_position_ok());
    // Heading pi is accepted when the actual bin point is in the square.
    refresh(-.4,M_PI);assert(n.unload_position_ok());
    n.unload_reference_config_.x=-.4;
    n.unload_alignment_attempts_=n.unload_alignment_max_attempts_;
    refresh(0.);n.state_=S::UNLOAD_ALIGN_CHECK;n.unload_alignment_tick();assert(n.state_==S::FAULT);
    // Deadline is bounded even when using ordinary coordinate navigation.
    refresh(.4);n.state_=S::UNLOAD_ALIGN_CHECK;
    n.unload_alignment_started_=MissionNode::Clock::now()-std::chrono::seconds(61);
    n.tick();assert(n.state_==S::FAULT);
    std::cout<<"PASS: TF bin reference, position-only alignment/zero-command interlock, no heading constraint, stale input, drift interlock, retries/deadline\n";
  }
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);rover_state::UnloadAlignmentTest::run();rclcpp::shutdown();}
