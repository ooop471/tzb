#include "rover_state/mission_node.hpp"
#include <cassert>
#include <iostream>
#include <thread>
namespace rover_state {
struct OutdoorNavigationTest {
  static void run() {
    auto mission=std::make_shared<MissionNode>();auto &n=*mission;n.timer_->cancel();
    auto server_node=std::make_shared<rclcpp::Node>("outdoor_navigation_fixture");
    using Follow=nav2_msgs::action::FollowPath;
    using ServerGoal=rclcpp_action::ServerGoalHandle<Follow>;
    std::vector<Follow::Goal> received;
    auto server=rclcpp_action::create_server<Follow>(server_node,"/follow_path",
      [&](const auto&,std::shared_ptr<const Follow::Goal> goal){received.push_back(*goal);return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](std::shared_ptr<ServerGoal>){return rclcpp_action::CancelResponse::ACCEPT;},
      [](std::shared_ptr<ServerGoal> goal){goal->succeed(std::make_shared<Follow::Result>());});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(mission);executor.add_node(server_node);
    auto until=[&](auto ready){
      auto end=std::chrono::steady_clock::now()+std::chrono::seconds(5);
      while(!ready()&&std::chrono::steady_clock::now()<end){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
      assert(ready());
    };
    until([&]{return n.follower_->action_server_is_ready();});
    nav_msgs::msg::Path path;path.header.frame_id="map";
    geometry_msgs::msg::PoseStamped p;p.header=path.header;p.pose.orientation.w=1.;
    path.poses.push_back(p);p.pose.position.x=2.;p.pose.position.y=3.;path.poses.push_back(p);
    n.start_captured_=true;n.entry_enabled_=true;n.entry_arrived_=false;
    n.approach_by_hint_=true; // a previous grasp must not alter coordinate navigation
    n.follow(path);until([&]{return received.size()==1&&!n.follow_pending_;});
    assert(received[0].controller_id=="OutdoorPath");
    assert(received[0].goal_checker_id=="entry_goal_checker");
    n.exit_latched_=true;n.approach_by_hint_=true;
    n.follow(path);until([&]{return received.size()==2&&!n.follow_pending_;});
    assert(received[1].controller_id==received[0].controller_id);
    assert(received[1].goal_checker_id=="exit_goal_checker");
    assert(received[1].path.poses.back().pose.position.x==2.);
    assert(received[1].path.poses.back().pose.position.y==3.);
    n.unload_alignment_enabled_=n.unload_alignment_active_=true;
    n.follow(path);until([&]{return received.size()==3&&!n.follow_pending_;});
    assert(received[2].controller_id=="UnloadAlignPath");
    assert(received[2].goal_checker_id=="exit_goal_checker");
    n.start_pose_={{1.,2.,0.},0.};
    const auto absolute=n.navigation_target({7.,8.,.3},n.world_,false);
    const auto relative=n.navigation_target({7.,8.,.3},n.world_,true);
    assert(absolute.pose.position.x==7.&&absolute.pose.position.y==8.);
    assert(relative.pose.position.x==8.&&relative.pose.position.y==10.);
    std::cout<<"PASS: entry/destination share OutdoorPath, ignore grasp checker, preserve path and configured coordinates\n";
  }
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);rover_state::OutdoorNavigationTest::run();rclcpp::shutdown();}
