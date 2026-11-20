#include "rover_state/mission_node.hpp"
#include <cassert>
#include <iostream>
namespace rover_state {
struct DeadlineUnloadTest {
  static void run() {
    auto node=std::make_shared<MissionNode>();auto &n=*node;n.timer_->cancel();
    using S=MissionNode::State;using Clock=MissionNode::Clock;
    n.cave_only_=true;n.unload_enabled_=n.unload_alignment_enabled_=n.unload_deadline_enabled_=true;
    n.start_captured_=true;n.start_pose_={{0.,0.,0.},0.};
    n.unload_deadline_seconds_=60.;
    n.destination_={0.,0.,0.};n.destination_frame_=n.world_;n.final_relative_=true;
    auto remaining_at=[&](int seconds){n.budget_.start(Clock::now()-std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(n.budget_.total_seconds-seconds)));};
    auto refresh=[&](double x,double y=0.) {
      n.robot_={{x,y,0.},0.};n.odom_received_=n.gate_received_=n.follower_received_=Clock::now();
      n.gate_state_={{"session",n.session_},{"allow",false},{"zero",true}};
      n.follower_state_={{"session",n.session_},{"generation",n.generation_},{"idle",true}};
      n.odom_history_.clear();const double t=n.stamp();
      for(double d:{.4,.2,0.})n.odom_history_.push_back({t-d,n.robot_});
      geometry_msgs::msg::TransformStamped tf;tf.header.frame_id=n.world_;tf.child_frame_id=n.base_;
      tf.transform.translation.x=x;tf.transform.translation.y=y;tf.transform.rotation.w=1.;
      n.tf_->setTransform(tf,"test",true);
      tf.header.frame_id=n.base_;tf.child_frame_id="base_link";tf.transform.translation.x=tf.transform.translation.y=0.;
      n.tf_->setTransform(tf,"test",true);
      tf.header.frame_id=n.planning_frame_;tf.child_frame_id=n.world_;
      n.tf_->setTransform(tf,"test",true);n.update_unload_reference();
    };
    refresh(1.0);remaining_at(60);
    // Being near the destination in the last minute is never enough before exit.
    for(const auto state:{S::EXPLORING,S::FINAL_NAVIGATING,S::FAULT}) {
      n.state_=state;
      assert(!n.try_deadline_unload()&&!n.unload_deadline_active_);
      assert(n.state_==state);
    }
    n.exit_latched_=true;n.state_=S::FINAL_NAVIGATING;remaining_at(61);
    assert(!n.try_deadline_unload()&&!n.unload_deadline_active_); // 61 s left
    remaining_at(60);refresh(1.4001);assert(!n.try_deadline_unload()); // bin >1 m
    refresh(1.2,.8);assert(!n.try_deadline_unload()); // circle, not +/-1 m square
    refresh(1.4);assert(n.try_deadline_unload()); // exact 1 m accepted; chassis is >1 m away
    assert(n.unload_deadline_active_&&n.state_==S::UNLOAD_ALIGN_CHECK);
    assert(!n.unload_position_ok()); // relaxed position applies only to the deadline branch
    n.exit_latched_=false;assert(!n.unload_deadline_position_ok());
    n.exit_latched_=true;assert(n.unload_deadline_position_ok());
    n.follow_pending_=true;n.unload_alignment_tick();assert(n.state_==S::UNLOAD_ALIGN_CHECK);
    n.follow_pending_=false;n.gate_state_["zero"]=false;
    n.unload_alignment_tick();assert(n.state_==S::UNLOAD_ALIGN_CHECK); // cannot unload while rolling
    refresh(1.0);n.odom_history_.clear();assert(!n.stationary());
    n.unload_alignment_tick();assert(n.state_==S::UNLOAD_WAIT_STOP);
    // Real unload handshake remains required, even for the emergency branch.
    n.unload_received_=Clock::now();
    n.unload_feedback_={{"session",n.session_},{"id",n.unload_id_},{"direction",0},
      {"active",true},{"tx_ok",true},{"lease_ok",true},{"interlock",true},{"phase_ok",true}};
    n.unload_tick();assert(n.state_==S::UNLOAD_OPENING);
    refresh(1.5);n.unload_tick();assert(n.state_==S::FAULT); // leaving 1 m blocks unloading
    refresh(1.0);n.unload_interrupted_=false;n.tick();assert(n.state_==S::FAULT); // fallback is attempted once, not a fault-retry loop
    n.unload_deadline_active_=false;n.unload_interrupted_=false;
    refresh(1.0);remaining_at(10);n.state_=S::PAUSED;n.tick();assert(n.state_==S::PAUSED&&!n.unload_deadline_active_);
    n.state_=S::COMPLETED;assert(!n.try_deadline_unload());
    n.state_=S::FINAL_NAVIGATING;n.unload_enabled_=false;assert(!n.try_deadline_unload()); // nav-only
    n.unload_enabled_=true;n.unload_reference_valid_=false;assert(!n.try_deadline_unload());
    refresh(1.0);n.odom_received_=Clock::now()-std::chrono::seconds(3);assert(!n.try_deadline_unload());
    refresh(1.0);n.cave_only_=false;n.arm_received_=Clock::now();
    n.arm_state_={{"session",n.session_},{"phase","ERROR"},{"safe_to_drive",false}};
    assert(!n.try_deadline_unload()); // safe arm is never bypassed
    n.cave_only_=true;n.state_=S::FAULT;n.unload_alignment_started_=Clock::now()-std::chrono::seconds(80);
    n.tick();assert(n.unload_deadline_active_&&n.state_==S::UNLOAD_WAIT_STOP); // position/nav fault can be rescued
    std::cout<<"PASS: confirmed-exit 60 s/1 m fallback, pre-exit exclusions, circle boundary, stop/arm/freshness/ack interlocks and post-exit fault rescue\n";
  }
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);rover_state::DeadlineUnloadTest::run();rclcpp::shutdown();}
