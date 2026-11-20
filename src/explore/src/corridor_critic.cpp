#include "rover_explore/corridor_geometry.hpp"
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav2_mppi_controller/critic_function.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mppi::critics {
class CorridorCritic : public CriticFunction {
public:
  void initialize() override {
    auto get=parameters_handler_->getParamGetter(name_);
    get(margin_,"side_margin",.03);
    get(weight_,"center_weight",30.);
    get(angle_weight_,"heading_weight",6.);
    get(timeout_,"geometry_timeout",2.5);
    get(max_heading_,"max_heading_degrees",10.);
    if(!std::isfinite(max_heading_)||max_heading_<=0||max_heading_>45)throw std::invalid_argument("Invalid corridor heading bound");
    max_heading_*=3.141592653589793/180.;
    if(!std::isfinite(margin_)||!std::isfinite(weight_)||!std::isfinite(angle_weight_)||
        !std::isfinite(timeout_)||margin_<0||weight_<0||angle_weight_<0||timeout_<=0)
      throw std::invalid_argument("Invalid corridor critic parameters");
    get(clearance_weight_,"clearance_weight",30.);
    if(!std::isfinite(clearance_weight_)||clearance_weight_<0)throw std::invalid_argument("Invalid corridor clearance weight");
    auto node=parent_.lock();
    phase_sub_=node->create_subscription<std_msgs::msg::String>("/explore/corridor_phase",
      rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);entry_=msg->data!="FOLLOW";
      });
    sub_=node->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "/explore/corridor_boundary",rclcpp::QoS(1).transient_local(),
      [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);boundary_=*msg;
      });
    enabled_sub_=node->create_subscription<std_msgs::msg::Bool>(
      "/explore/enabled",rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);exploring_=msg->data;
      });
  }
  void score(CriticData& data) override {
    geometry_msgs::msg::PolygonStamped boundary;bool entry;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(!enabled_||!exploring_)return;
      boundary=boundary_;entry=entry_;
    }
    auto node=parent_.lock();
    const double age=(node->now()-rclcpp::Time(boundary.header.stamp)).seconds();
    if(age<-.15||age>timeout_||boundary.polygon.points.size()<4||
        boundary.polygon.points.size()%2) { return; }
    std::vector<rover_explore::CorridorSection> line;
    try {
      const auto tf=costmap_ros_->getTfBuffer()->lookupTransform(
        costmap_ros_->getGlobalFrameID(),boundary.header.frame_id,tf2::TimePointZero);
      const auto n=boundary.polygon.points.size()/2;
      for(size_t i=0;i<n;++i) {
        geometry_msgs::msg::PointStamped left,right,l,r;
        const auto&a=boundary.polygon.points[i];const auto&b=boundary.polygon.points[2*n-1-i];
        left.point.x=a.x;left.point.y=a.y;left.point.z=a.z;
        right.point.x=b.x;right.point.y=b.y;right.point.z=b.z;
        tf2::doTransform(left,l,tf);tf2::doTransform(right,r,tf);
        line.push_back({.5*(l.point.x+r.point.x),.5*(l.point.y+r.point.y),
            std::hypot(l.point.x-r.point.x,l.point.y-r.point.y)});
        if(!std::isfinite(line.back().x)||!std::isfinite(line.back().y)||
            !std::isfinite(line.back().width)||line.back().width<=0)
          throw std::runtime_error("Invalid corridor section");
      }
    } catch(const std::exception& e) {
      RCLCPP_WARN_THROTTLE(logger_,*node->get_clock(),2000,"Corridor unavailable: %s",e.what());
      return;
    }
    const rover_explore::CorridorGeometry geometry(line);
    const auto footprint=costmap_ros_->getRobotFootprint();
    const auto& t=data.trajectories;
    for(size_t i=0;i<t.x.shape()[0];++i) {
      double cost=0;
      const auto initial=geometry.project(t.x(i,0),t.y(i,0));
      const double initial_angle=std::abs(std::atan2(std::sin(t.yaws(i,0)-initial.yaw),std::cos(t.yaws(i,0)-initial.yaw)));
      for(size_t j=0;j<t.x.shape()[1];++j) {
        const double x=t.x(i,j),y=t.y(i,j),yaw=t.yaws(i,j);
        if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)) {cost+=1e6;continue;}
        const auto p=geometry.project(x,y);
        const double angle=std::atan2(std::sin(yaw-p.yaw),std::cos(yaw-p.yaw));
        // No virtual walls behind the first measured section. At the entrance
        // softly join the corridor; the physical costmap remains authoritative.
        const double ramp=std::clamp(p.arc/.6,0.,1.);
        const double blend=entry?.25*ramp:1.;
        // Always prefer returning inside the heading band, including when the
        // vehicle starts outside it. Reject sampled turns that worsen excess yaw.
        const double excess=std::max(0.,std::abs(angle)-max_heading_);
        cost+=1000*excess*excess;
        if(std::abs(angle)>std::max(max_heading_,initial_angle)+.01)cost+=1e6;
        const double deficit=std::max(0.,margin_-geometry.clearance(footprint,x,y,yaw));
        cost+=angle_weight_*angle*angle+blend*(weight_*p.lateral*p.lateral+
          clearance_weight_*deficit*deficit);
      }
      data.costs(i)+=cost/std::max<size_t>(1,t.x.shape()[1]);
    }
  }
private:
  double margin_,weight_,angle_weight_,timeout_,clearance_weight_,max_heading_;
  std::mutex mutex_;bool exploring_=false,entry_=true;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub_;
  geometry_msgs::msg::PolygonStamped boundary_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
};
}
PLUGINLIB_EXPORT_CLASS(mppi::critics::CorridorCritic,mppi::critics::CriticFunction)
