#include "rover_state/mission_node.hpp"
#include <cassert>
#include <iostream>
namespace rover_state {
struct MissionBudgetStateTest {
  static void run() {
    auto node=std::make_shared<MissionNode>();auto &n=*node;
    using S=MissionNode::State;
    auto expired=MissionNode::Clock::now()-std::chrono::seconds(361);
    n.budget_.start(expired);n.state_=S::GRASPING;n.centered_active_=true;
    n.arm_sent_=n.arm_goal_submitted_=true;n.arm_result_complete_=false;
    n.tick(); // Must cancel even when odometry/perception is absent.
    assert(n.budget_.closed()&&n.state_==S::CENTER_RETURN_WAIT);
    assert(n.cancel_requested_&&!n.explore_enabled_&&n.arm_cancel_requested_);
    n.state_=S::CENTER_SETTLING;n.tick();
    assert(n.state_==S::CENTER_RETURN_WAIT); // cannot start another grasp
    n.state_=S::PAUSED;n.tick();assert(n.state_==S::PAUSED);
    assert(n.budget_.elapsed(MissionNode::Clock::now())>=361.);
    n.budget_.start(MissionNode::Clock::now());
    n.budget_.completed(1);n.budget_.completed(2);n.budget_.completed(3);
    n.state_=S::TARGET_TRACKING;n.tick();
    assert(n.budget_.closed()&&n.state_==S::CENTER_RETURN_WAIT);
    n.budget_.start(expired);n.centered_active_=false;n.state_=S::GRASPING;n.tick();
    assert(n.state_==S::RECOVERING); // alternative approach pipeline also stops
    // Approach must not hand off early at the old ~33 cm braking threshold.
    n.timer_->cancel();n.budget_.start(MissionNode::Clock::now());
    n.exit_latched_=false;n.follower_state_={{"exit_forward",false},{"exit_complete",false}};
    n.robot_={{0.,0.,0.},0.};n.front_=.30;
    n.centered_parameters_.preferred_radius=.25;
    n.corridor_line_.clear();n.exploration_route_.clear();
    for(double gap:{.33,.26,.2501,.2499,.20}) {
      n.state_=S::TARGET_TRACKING;n.entered_=MissionNode::Clock::now();
      n.centered_active_=true;n.explore_enabled_=true;n.locked_={n.front_+gap,0.,0.};
      n.track_grasp_target();
      assert(n.state_==(gap>.25?S::TARGET_TRACKING:S::CENTER_HANDOFF));
      assert(n.explore_enabled_==(gap>.25));
      assert(n.grasp_diagnostics_["stop_allowance"]==0.);
    }
    std::cout<<"PASS: live mission cutoff during grasp/tracking/settling, missing inputs, pause and 3 results\n";
  }
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);rover_state::MissionBudgetStateTest::run();rclcpp::shutdown();}
