#include "rover_explore/exploration_path_follower.hpp"
#include <cassert>
#include <iostream>
#include <nlohmann/json.hpp>
namespace rover_explore {
struct ExplorationFollowerTest {
  static void run() {
    auto node=std::make_shared<ExplorationPathFollower>();auto &n=*node;
    n.timer_->cancel();
    // The follower must not own any velocity publisher, even when paths fail.
    assert(n.count_publishers("/mission/recovery_cmd_vel")==0);
    assert(n.count_publishers("/cmd_vel_nav")==0);
    assert(n.count_publishers("/cmd_vel")==0);
    // Repeated failed controllers only schedule fresh-path retries.
    for(int attempt=1;attempt<=5;++attempt) {
      n.terminal_=rclcpp_action::ResultCode::ABORTED;
      n.terminal_rejected_=false;n.drain();
      assert(n.failures_==attempt&&n.retry_after_&&n.requests_.empty());
      assert(!n.complete_&&n.stopped());
    }
    // Missing/stale inputs keep execution stopped, without reporting completion.
    n.enabled_=true;n.tick();
    assert(!n.enabled_&&n.stopped()&&!n.complete_);
    // Task disable still clears paths and failure history and drains navigation.
    n.managed_=true;n.management_session_="test";n.management_generation_=1;
    n.requested_=true;n.enabled_=true;n.route_.push_back(geometry_msgs::msg::Point());
    n.management_command(nlohmann::json({{"session","test"},{"generation",2},
      {"enabled",false},{"stamp",n.now().seconds()}}).dump());
    assert(!n.requested_&&!n.enabled_&&n.route_.empty()&&n.failures_==0&&n.stopped());
    n.management_status();
    assert(n.count_publishers("/mission/recovery_cmd_vel")==0);
    assert(n.count_publishers("/cmd_vel_nav")==0);
    std::cout<<"PASS: failed tracking retries, input waiting and management disable issue no recovery velocity commands\n";
  }
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);rover_explore::ExplorationFollowerTest::run();rclcpp::shutdown();}
