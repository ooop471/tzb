#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

namespace rover_state {
// Switch both Nav2 costmaps before allowing the next motion phase to start.
class InflationProfile {
public:
  using Radii=std::array<double,2>; // global, local
  InflationProfile(rclcpp::Node* node,double timeout,double settle,
      std::array<std::string,2> names={"/global_costmap/global_costmap","/local_costmap/local_costmap"})
      : node_(node),timeout_(timeout),settle_(settle) {
    for(size_t i=0;i<2;++i)clients_[i]=std::make_shared<rclcpp::AsyncParametersClient>(node,names[i]);
  }
  bool ensure(Radii radii,double global_map_stamp,double local_map_stamp) {
    if(!wanted_||*wanted_!=radii) {
      wanted_=radii;started_=Clock::now();phase_=Phase::SERVICES;error_.clear();
    }
    if(!error_.empty())return false;
    if(phase_==Phase::READY)return true;
    if(seconds(started_)>timeout_) {error_="Timed out switching global/local inflation radii";return false;}
    try {
      if(phase_==Phase::SERVICES) {
        if(!clients_[0]->service_is_ready()||!clients_[1]->service_is_ready())return false;
        for(size_t i=0;i<2;++i)set_[i]=clients_[i]->set_parameters_atomically({
            rclcpp::Parameter("inflation_layer.inflation_radius",radii[i])});
        phase_=Phase::SET;return false;
      }
      if(phase_==Phase::SET) {
        if(!complete(set_[0])||!complete(set_[1]))return false;
        for(size_t i=0;i<2;++i) {
          const auto result=set_[i].get();
          if(!result.successful) {error_=(i==0?"Global":"Local")+std::string(" costmap rejected inflation radius: ")+result.reason;return false;}
        }
        for(size_t i=0;i<2;++i)get_[i]=clients_[i]->get_parameters({"inflation_layer.inflation_radius"});
        phase_=Phase::VERIFY;return false;
      }
      if(phase_==Phase::VERIFY) {
        if(!complete(get_[0])||!complete(get_[1]))return false;
        for(size_t i=0;i<2;++i) {
          const auto values=get_[i].get();
          if(values.size()!=1||values[0].get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE||
             std::abs(values[0].as_double()-radii[i])>1e-6) {
            error_="Costmap inflation parameter readback differs from requested radius";return false;
          }
        }
        confirmed_=Clock::now();confirmed_stamp_=node_->now().seconds();
        phase_=Phase::MAPS;return false;
      }
      if(phase_==Phase::MAPS&&seconds(confirmed_)>=settle_&&
         global_map_stamp>confirmed_stamp_&&local_map_stamp>confirmed_stamp_) {
        phase_=Phase::READY;return true;
      }
    } catch(const std::exception&e) {error_=e.what();}
    return false;
  }
  const std::string& error() const{return error_;}
  bool ready() const{return wanted_&&phase_==Phase::READY;}
  const std::optional<Radii>& requested()const{return wanted_;}
private:
  using Clock=std::chrono::steady_clock;
  enum class Phase {SERVICES,SET,VERIFY,MAPS,READY};
  template<class T>static bool complete(const std::shared_future<T>& f) {
    return f.valid()&&f.wait_for(std::chrono::seconds(0))==std::future_status::ready;
  }
  static double seconds(Clock::time_point t){return std::chrono::duration<double>(Clock::now()-t).count();}
  rclcpp::Node* node_;
  double timeout_,settle_,confirmed_stamp_=0;
  std::array<rclcpp::AsyncParametersClient::SharedPtr,2> clients_;
  std::array<std::shared_future<rcl_interfaces::msg::SetParametersResult>,2> set_;
  std::array<std::shared_future<std::vector<rclcpp::Parameter>>,2> get_;
  std::optional<Radii> wanted_;
  Phase phase_=Phase::SERVICES;
  Clock::time_point started_{},confirmed_{};
  std::string error_;
};
} // namespace rover_state
