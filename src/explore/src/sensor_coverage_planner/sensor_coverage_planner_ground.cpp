#include <sstream>
/**
 * @file sensor_coverage_planner_ground.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"
#include "rover_explore/centerline_path.hpp"
#include "graph/graph.h"
#include <memory>
#include <limits>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

namespace sensor_coverage_planner_3d_ns {
// PlannerParameters::PlannerParameters()
// {
// }

// bool PlannerParameters::ReadParameters(rclcpp::Node::SharedPtr node_)
void SensorCoveragePlanner3D::ReadParameters() {
  input_timeout_ = declare_parameter<double>("input_timeout", 2.0);
  sync_tolerance_ = declare_parameter<double>("sync_tolerance", 0.15);
  declare_parameter<bool>("corridor.enabled", false);
  declare_parameter<bool>("corridor.dead_end.enabled",true);
  declare_parameter<double>("corridor.dead_end.detection_distance",2.0);
  ground_observations_.sensor_height=declare_parameter<double>("corridor.ground.sensor_height",.20);
  ground_observations_.height_tolerance=declare_parameter<double>("corridor.ground.height_tolerance",.10);
  ground_observations_.radius=declare_parameter<double>("corridor.ground.support_radius",.14);
  ground_observations_.max_age=declare_parameter<double>("corridor.ground.max_age",4.0);
  ground_observations_.fit_radius=declare_parameter<double>("corridor.ground.fit_radius",3.0);
  ground_observations_.fit_tolerance=declare_parameter<double>("corridor.ground.fit_tolerance",.035);
  ground_observations_.max_slope_degrees=declare_parameter<double>("corridor.ground.max_slope_degrees",20.0);
  ground_observations_.minimum_inliers=declare_parameter<int>("corridor.ground.minimum_inliers",12);
  ground_observations_.validate();
  declare_parameter<double>("corridor.transition_width",2.4);
  declare_parameter<double>("corridor.wall_reference_height",.45);
  declare_parameter<double>("corridor.nominal_width",1.1);
  declare_parameter<double>("corridor.minimum_width",.70);
  declare_parameter<double>("corridor.maximum_width",1.55);
  declare_parameter<double>("corridor.section_step",.20);
  declare_parameter<double>("corridor.section_half_length",.22);
  declare_parameter<double>("corridor.horizon",3.0);
  declare_parameter<double>("corridor.entry_lookahead",.8);
  declare_parameter<double>("corridor.lookahead",1.2);
  declare_parameter<double>("corridor.target_min_distance",.2);
  declare_parameter<double>("corridor.target_max_distance",2.0);
  declare_parameter<double>("corridor.center_band",.14);
  declare_parameter<double>("corridor.body_half_width",.32);
  declare_parameter<double>("corridor.side_margin",.03);
  declare_parameter<double>("corridor.handoff_distance",.35);
  declare_parameter<double>("corridor.hold_time",1.5);
  declare_parameter<double>("corridor.center_smoothing_time",.8);
  declare_parameter<double>("corridor.center_max_update",.04);
  declare_parameter<double>("corridor.center_jump_threshold",.12);
  declare_parameter<int>("corridor.center_confirmation_cycles",3);
  declare_parameter<double>("corridor.max_heading_degrees",10.);
  declare_parameter<double>("corridor.progress_timeout",8.0);
  declare_parameter<int>("corridor.confirmation_cycles",2);
  declare_parameter<bool>("corridor.exit_enabled",true);
  declare_parameter<double>("corridor.body_front",.22);
  declare_parameter<double>("corridor.body_rear",.60);
  declare_parameter<double>("corridor.exit_margin",.05);
  declare_parameter<double>("corridor.exit_extension",3.0);
  declare_parameter<double>("corridor.exit_forward_distance",2.0);
  declare_parameter<double>("corridor.exit_lookahead",.8);
  declare_parameter<std::string>("corridor.ceiling_exit.raw_topic","/livox/lidar");
  ceiling_exit_.parameters.radius=declare_parameter<double>("corridor.ceiling_exit.radius",2.0);
  ceiling_exit_.parameters.forward_min=declare_parameter<double>("corridor.ceiling_exit.forward_min",-0.5);
  ceiling_exit_.parameters.forward_max=declare_parameter<double>("corridor.ceiling_exit.forward_max",2.0);
  ceiling_exit_.parameters.half_width=declare_parameter<double>("corridor.ceiling_exit.half_width",1.0);
  ceiling_exit_.parameters.cell_size=declare_parameter<double>("corridor.ceiling_exit.cell_size",0.25);
  ceiling_exit_.parameters.minimum_relative_height=declare_parameter<double>("corridor.ceiling_exit.minimum_relative_height",0.3);
  ceiling_exit_.parameters.height_threshold=declare_parameter<double>("corridor.ceiling_exit.height_threshold",2.0);
  ceiling_exit_.parameters.minimum_high_fraction=declare_parameter<double>("corridor.ceiling_exit.minimum_high_fraction",ceiling_exit_.parameters.minimum_high_fraction);
  ceiling_exit_.parameters.max_height=declare_parameter<double>("corridor.ceiling_exit.max_height",12.0);
  ceiling_exit_.parameters.minimum_elevation_degrees=declare_parameter<double>("corridor.ceiling_exit.minimum_elevation_degrees",30.0);
  ceiling_exit_.parameters.confirm_time=declare_parameter<double>("corridor.ceiling_exit.confirm_time",0.8);
  ceiling_exit_.parameters.maximum_gap=declare_parameter<double>("corridor.ceiling_exit.maximum_gap",0.5);
  ceiling_exit_.parameters.minimum_cloud_points=declare_parameter<int>("corridor.ceiling_exit.minimum_cloud_points",100);
  ceiling_exit_.parameters.minimum_upper_points=declare_parameter<int>("corridor.ceiling_exit.minimum_upper_points",20);
  ceiling_exit_.parameters.minimum_upper_cells=declare_parameter<int>("corridor.ceiling_exit.minimum_upper_cells",3);
  ceiling_exit_.parameters.minimum_cell_points=declare_parameter<int>("corridor.ceiling_exit.minimum_cell_points",2);
  ceiling_exit_.parameters.validate();
  navigation_grid_enabled_ = declare_parameter<bool>("navigation_grid.enabled", false);
  navigation_grid_timeout_ = declare_parameter<double>("navigation_grid.timeout", 2.0);
  declare_parameter<std::string>("navigation_grid.global_topic", "/global_costmap/costmap");
  declare_parameter<std::string>("navigation_grid.local_topic", "/local_costmap/costmap_raw");
  if (!std::isfinite(navigation_grid_timeout_) || navigation_grid_timeout_ <= 0)
    throw std::invalid_argument("navigation_grid.timeout must be finite and positive");
  const double confirmation_time = declare_parameter<double>("coverage_confirmation_time", 10.0);
  const int confirmation_cycles = declare_parameter<int>("coverage_confirmation_cycles", 10);
  coverage_confirmation_ = rover_explore::CoverageConfirmation(
      confirmation_time, confirmation_cycles, input_timeout_);
  declare_parameter<double>("kViewpointVerticalHalfFovDeg", 12.0);
  declare_parameter<bool>("goal_selection.enabled", true);
  declare_parameter<int>("goal_selection.empty_cycles", 3);
  declare_parameter<double>("goal_selection.main_cone_degrees", 35.0);
  declare_parameter<double>("goal_selection.max_turn_degrees", 85.0);
  declare_parameter<double>("goal_selection.min_distance", 0.4);
  declare_parameter<double>("goal_selection.max_distance", 4.0);
  declare_parameter<double>("goal_selection.target_distance", 2.0);
  declare_parameter<double>("goal_selection.handoff_distance", 0.6);
  declare_parameter<double>("goal_selection.progress_distance", 0.15);
  declare_parameter<double>("goal_selection.progress_timeout", 8.0);
  declare_parameter<double>("goal_selection.retry_cooldown", 12.0);
  declare_parameter<double>("goal_selection.retry_radius", 0.6);
  declare_parameter<bool>("exploration_region.enabled", true);
  declare_parameter<double>("exploration_region.forward_distance", 4.0);
  declare_parameter<double>("exploration_region.half_width", 4.0);
  declare_parameter<double>("exploration_region.entry_margin", 0.05);
  declare_parameter<bool>("self_filter.enabled", true);
  declare_parameter<double>("self_filter.radius", 0.6);
  declare_parameter<std::vector<double>>("self_filter.minimum", {0.20, -0.12, 0.05});
  declare_parameter<std::vector<double>>("self_filter.maximum", {0.36, 0.08, 0.60});
  this->declare_parameter<std::string>("sub_start_exploration_topic_",
                                       "/exploration_start");
  this->declare_parameter<std::string>("sub_state_estimation_topic_",
                                       "/state_estimation_at_scan");
  this->declare_parameter<std::string>("sub_registered_scan_topic_",
                                       "/registered_scan");
  this->declare_parameter<std::string>("sub_terrain_map_topic_",
                                       "/terrain_map");
  this->declare_parameter<std::string>("sub_terrain_map_ext_topic_",
                                       "/terrain_map_ext");
  this->declare_parameter<std::string>("sub_coverage_boundary_topic_",
                                       "/coverage_boundary");
  this->declare_parameter<std::string>("sub_viewpoint_boundary_topic_",
                                       "/navigation_boundary");
  this->declare_parameter<std::string>("sub_nogo_boundary_topic_",
                                       "/nogo_boundary");
  this->declare_parameter<std::string>("sub_joystick_topic_", "/joy");
  this->declare_parameter<std::string>("sub_reset_waypoint_topic_",
                                       "/reset_waypoint");
  this->declare_parameter<std::string>("pub_exploration_finish_topic_",
                                       "exploration_finish");
  this->declare_parameter<std::string>("pub_runtime_breakdown_topic_",
                                       "runtime_breakdown");
  this->declare_parameter<std::string>("pub_runtime_topic_", "/runtime");
  this->declare_parameter<std::string>("pub_waypoint_topic_", "/way_point");
  this->declare_parameter<std::string>("pub_momentum_activation_count_topic_",
                                       "momentum_activation_count");

  // Bool
  this->declare_parameter<bool>("kUseFrontier", true);
  this->declare_parameter<bool>("kAutoStart", false);
  this->declare_parameter<bool>("kRushHome", false);
  this->declare_parameter<bool>("kUseTerrainHeight", true);
  this->declare_parameter<bool>("kCheckTerrainCollision", true);
  this->declare_parameter<bool>("kExtendWayPoint", true);
  this->declare_parameter<bool>("kUseLineOfSightLookAheadPoint", true);
  this->declare_parameter<bool>("kNoExplorationReturnHome", true);
  this->declare_parameter<bool>("kUseMomentum", false);

  // Double
  this->declare_parameter<double>("kKeyposeCloudDwzFilterLeafSize", 0.2);
  this->declare_parameter<double>("kRushHomeDist", 10.0);
  this->declare_parameter<double>("kAtHomeDistThreshold", 0.5);
  this->declare_parameter<double>("kTerrainCollisionThreshold", 0.10);
  this->declare_parameter<double>("kLookAheadDistance", 5.0);
  this->declare_parameter<double>("kExtendWayPointDistanceBig", 8.0);
  this->declare_parameter<double>("kExtendWayPointDistanceSmall", 3.0);

  // Int
  this->declare_parameter<int>("kDirectionChangeCounterThr", 4);
  this->declare_parameter<int>("kDirectionNoChangeCounterThr", 5);
  this->declare_parameter<int>("kResetWaypointJoystickAxesID", 0);

  // grid_world
  this->declare_parameter<int>("kGridWorldXNum", 121);
  this->declare_parameter<int>("kGridWorldYNum", 121);
  this->declare_parameter<int>("kGridWorldZNum", 12);
  this->declare_parameter<double>("kGridWorldCellHeight", 8.0);
  this->declare_parameter<int>("kGridWorldNearbyGridNum", 5);
  this->declare_parameter<int>("kMinAddPointNumSmall", 60);
  this->declare_parameter<int>("kMinAddPointNumBig", 100);
  this->declare_parameter<int>("kMinAddFrontierPointNum", 30);
  this->declare_parameter<int>("kCellExploringToCoveredThr", 1);
  this->declare_parameter<int>("kCellCoveredToExploringThr", 10);
  this->declare_parameter<int>("kCellExploringToAlmostCoveredThr", 10);
  this->declare_parameter<int>("kCellAlmostCoveredToExploringThr", 20);
  this->declare_parameter<int>("kCellUnknownToExploringThr", 1);

  // keypose_graph
  this->declare_parameter<double>("keypose_graph/kAddNodeMinDist", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddNonKeyposeNodeMinDist",
                                  0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeConnectDistThr", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeToLastKeyposeDistThr",
                                  0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeVerticalThreshold",
                                  0.5);
  this->declare_parameter<double>(
      "keypose_graph/kAddEdgeCollisionCheckResolution", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeCollisionCheckRadius",
                                  0.5);
  this->declare_parameter<int>(
      "keypose_graph/kAddEdgeCollisionCheckPointNumThr", 1);

  // local_coverage_planner
  this->declare_parameter<int>("kGreedyViewPointSampleRange", 5);
  this->declare_parameter<int>("kLocalPathOptimizationItrMax", 10);

  // planning_env
  this->declare_parameter<double>("kSurfaceCloudDwzLeafSize", 0.2);
  this->declare_parameter<double>("kCollisionCloudDwzLeafSize", 0.2);
  this->declare_parameter<int>("kKeyposeCloudStackNum", 5);
  this->declare_parameter<int>("kPointCloudRowNum", 20);
  this->declare_parameter<int>("kPointCloudColNum", 20);
  this->declare_parameter<int>("kPointCloudLevelNum", 10);
  this->declare_parameter<int>("kMaxCellPointNum", 100000);
  this->declare_parameter<double>("kPointCloudCellSize", 24.0);
  this->declare_parameter<double>("kPointCloudCellHeight", 3.0);
  this->declare_parameter<int>("kPointCloudManagerNeighborCellNum", 5);
  this->declare_parameter<double>("kCoverCloudZSqueezeRatio", 2.0);
  this->declare_parameter<double>("kFrontierClusterTolerance", 1.0);
  this->declare_parameter<int>("kFrontierClusterMinSize", 30);
  this->declare_parameter<bool>("kUseCoverageBoundaryOnFrontier", false);
  this->declare_parameter<bool>("kUseCoverageBoundaryOnObjectSurface", false);

  // rolling_occupancy_grid
  this->declare_parameter<double>("rolling_occupancy_grid/resolution_x", 0.3);
  this->declare_parameter<double>("rolling_occupancy_grid/resolution_y", 0.3);
  this->declare_parameter<double>("rolling_occupancy_grid/resolution_z", 0.3);

  // viewpoint_manager
  this->declare_parameter<int>("viewpoint_manager/number_x", 80);
  this->declare_parameter<int>("viewpoint_manager/number_y", 80);
  this->declare_parameter<int>("viewpoint_manager/number_z", 40);
  this->declare_parameter<double>("viewpoint_manager/resolution_x", 0.5);
  this->declare_parameter<double>("viewpoint_manager/resolution_y", 0.5);
  this->declare_parameter<double>("viewpoint_manager/resolution_z", 0.5);
  this->declare_parameter<double>("kConnectivityHeightDiffThr", 0.25);
  this->declare_parameter<double>("kViewPointCollisionMargin", 0.5);
  this->declare_parameter<double>("kViewPointCollisionMarginZPlus", 0.5);
  this->declare_parameter<double>("kViewPointCollisionMarginZMinus", 0.5);
  this->declare_parameter<double>("kCollisionGridZScale", 2.0);
  this->declare_parameter<double>("kCollisionGridResolutionX", 0.5);
  this->declare_parameter<double>("kCollisionGridResolutionY", 0.5);
  this->declare_parameter<double>("kCollisionGridResolutionZ", 0.5);
  this->declare_parameter<bool>("kLineOfSightStopAtNearestObstacle", true);
  this->declare_parameter<bool>("kCheckDynamicObstacleCollision", true);
  this->declare_parameter<int>("kCollisionFrameCountMax", 3);
  this->declare_parameter<double>("kViewPointHeightFromTerrain", 0.75);
  this->declare_parameter<double>("kViewPointHeightFromTerrainChangeThreshold",
                                  0.6);
  this->declare_parameter<int>("kCollisionPointThr", 3);
  this->declare_parameter<double>("kCoverageOcclusionThr", 1.0);
  this->declare_parameter<double>("kCoverageDilationRadius", 1.0);
  this->declare_parameter<double>("kCoveragePointCloudResolution", 1.0);
  this->declare_parameter<double>("kSensorRange", 10.0);
  this->declare_parameter<double>("kNeighborRange", 3.0);

  // tare_visualizer
  this->declare_parameter<bool>("kExploringSubspaceMarkerColorGradientAlpha",
                                true);
  this->declare_parameter<double>("kExploringSubspaceMarkerColorMaxAlpha", 1.0);
  this->declare_parameter<double>("kExploringSubspaceMarkerColorR", 0.0);
  this->declare_parameter<double>("kExploringSubspaceMarkerColorG", 1.0);
  this->declare_parameter<double>("kExploringSubspaceMarkerColorB", 0.0);
  this->declare_parameter<double>("kExploringSubspaceMarkerColorA", 1.0);
  this->declare_parameter<double>("kLocalPlanningHorizonMarkerColorR", 0.0);
  this->declare_parameter<double>("kLocalPlanningHorizonMarkerColorG", 1.0);
  this->declare_parameter<double>("kLocalPlanningHorizonMarkerColorB", 0.0);
  this->declare_parameter<double>("kLocalPlanningHorizonMarkerColorA", 1.0);
  this->declare_parameter<double>("kLocalPlanningHorizonMarkerWidth", 0.3);
  this->declare_parameter<double>("kLocalPlanningHorizonHeight", 3.0);

  bool got_parameter = true;
  got_parameter &= this->get_parameter("sub_start_exploration_topic_",
                                       sub_start_exploration_topic_);
  if (!got_parameter) {
    std::cout << "Failed to get parameter sub_start_exploration_topic_"
              << std::endl;
  }
  this->get_parameter("sub_state_estimation_topic_",
                      sub_state_estimation_topic_);
  this->get_parameter("sub_registered_scan_topic_", sub_registered_scan_topic_);
  this->get_parameter("sub_terrain_map_topic_", sub_terrain_map_topic_);
  this->get_parameter("sub_terrain_map_ext_topic_", sub_terrain_map_ext_topic_);
  this->get_parameter("sub_coverage_boundary_topic_",
                      sub_coverage_boundary_topic_);
  this->get_parameter("sub_viewpoint_boundary_topic_",
                      sub_viewpoint_boundary_topic_);
  this->get_parameter("sub_nogo_boundary_topic_", sub_nogo_boundary_topic_);
  this->get_parameter("sub_joystick_topic_", sub_joystick_topic_);
  this->get_parameter("sub_reset_waypoint_topic_", sub_reset_waypoint_topic_);
  this->get_parameter("pub_exploration_finish_topic_",
                      pub_exploration_finish_topic_);
  this->get_parameter("pub_runtime_breakdown_topic_",
                      pub_runtime_breakdown_topic_);
  this->get_parameter("pub_runtime_topic_", pub_runtime_topic_);
  this->get_parameter("pub_waypoint_topic_", pub_waypoint_topic_);
  this->get_parameter("pub_momentum_activation_count_topic_",
                      pub_momentum_activation_count_topic_);

  this->get_parameter("kAutoStart", kAutoStart);

  std::cout << "parameter kAutoStart: " << kAutoStart << std::endl;

  this->get_parameter("kRushHome", kRushHome);
  this->get_parameter("kUseTerrainHeight", kUseTerrainHeight);
  this->get_parameter("kCheckTerrainCollision", kCheckTerrainCollision);
  this->get_parameter("kExtendWayPoint", kExtendWayPoint);
  this->get_parameter("kUseLineOfSightLookAheadPoint",
                      kUseLineOfSightLookAheadPoint);
  this->get_parameter("kNoExplorationReturnHome", kNoExplorationReturnHome);
  this->get_parameter("kUseMomentum", kUseMomentum);

  this->get_parameter("kKeyposeCloudDwzFilterLeafSize",
                      kKeyposeCloudDwzFilterLeafSize);
  this->get_parameter("kRushHomeDist", kRushHomeDist);
  this->get_parameter("kAtHomeDistThreshold", kAtHomeDistThreshold);
  this->get_parameter("kTerrainCollisionThreshold", kTerrainCollisionThreshold);
  this->get_parameter("kLookAheadDistance", kLookAheadDistance);
  this->get_parameter("kExtendWayPointDistanceBig", kExtendWayPointDistanceBig);
  this->get_parameter("kExtendWayPointDistanceSmall",
                      kExtendWayPointDistanceSmall);

  this->get_parameter("kDirectionChangeCounterThr", kDirectionChangeCounterThr);
  this->get_parameter("kDirectionNoChangeCounterThr",
                      kDirectionNoChangeCounterThr);
  this->get_parameter("kResetWaypointJoystickAxesID",
                      kResetWaypointJoystickAxesID);
}

// PlannerData::PlannerData()
// {
// }

// void PlannerData::Initialize(rclcpp::Node::SharedPtr node_)
void SensorCoveragePlanner3D::InitializeData() {
  keypose_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
          shared_from_this(), "keypose_cloud", kWorldFrameID);
  registered_scan_stack_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>>(
          shared_from_this(), "registered_scan_stack", kWorldFrameID);
  registered_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "registered_cloud", kWorldFrameID);
  large_terrain_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "terrain_cloud_large", kWorldFrameID);
  terrain_collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "terrain_collision_cloud", kWorldFrameID);
  terrain_ext_collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "terrain_ext_collision_cloud", kWorldFrameID);
  viewpoint_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "viewpoint_vis_cloud", kWorldFrameID);
  grid_world_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "grid_world_vis_cloud", kWorldFrameID);
  exploration_path_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "bspline_path_cloud", kWorldFrameID);

  selected_viewpoint_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "selected_viewpoint_vis_cloud", kWorldFrameID);
  exploring_cell_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "exploring_cell_vis_cloud", kWorldFrameID);
  collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "collision_cloud", kWorldFrameID);
  lookahead_point_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "lookahead_point_cloud", kWorldFrameID);
  keypose_graph_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "keypose_graph_cloud", kWorldFrameID);
  viewpoint_in_collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "viewpoint_in_collision_cloud_", kWorldFrameID);
  point_cloud_manager_neighbor_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "pointcloud_manager_cloud", kWorldFrameID);
  reordered_global_subspace_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "reordered_global_subspace_cloud", kWorldFrameID);

  viewpoint_manager_ = std::make_shared<viewpoint_manager_ns::ViewPointManager>(
      shared_from_this());
  corridor_.parameters=rover_explore::read_corridor(*this);
  if (corridor_.parameters.enabled)
  {
    if (!navigation_grid_enabled_)
      throw std::invalid_argument("corridor mode requires navigation_grid.enabled");
    viewpoint_manager_->SetExplorationConstraint(
        [this](const Eigen::Vector3d &p) { return corridor_.contains(p) && ground_observations_.supports(p,now().seconds()); });
  }
  keypose_graph_ =
      std::make_shared<keypose_graph_ns::KeyposeGraph>(shared_from_this());
  planning_env_ =
      std::make_shared<planning_env_ns::PlanningEnv>(shared_from_this());
  const std::weak_ptr<planning_env_ns::PlanningEnv> environment = planning_env_;
  viewpoint_manager_->SetObservedFreeSpaceChecker([this,environment](const Eigen::Vector3d &position) {
    // If odometry lies at floor level, a floor return occupies the viewpoint's
    // 3-D voxel. Measured floor is itself positive observation evidence; Nav2
    // still independently rejects obstacles and checks every connecting edge.
    if (corridor_.parameters.enabled && ground_observations_.supports(position,now().seconds())) return true;
    const auto env = environment.lock();
    return env && env->IsObservedFreeSpace(position);
  });
  if (navigation_grid_enabled_ || (corridor_.parameters.enabled && corridor_.parameters.exit_enabled))
  {
    navigation_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    navigation_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
        *navigation_tf_buffer_, this, false);
  }
  if (navigation_grid_enabled_) {
    navigation_global_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        get_parameter("navigation_grid.global_topic").as_string(), rclcpp::QoS(1).reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) { navigation_global_map_=message; });
    navigation_local_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
        get_parameter("navigation_grid.local_topic").as_string(), rclcpp::QoS(1).reliable(),
        [this](nav2_msgs::msg::Costmap::ConstSharedPtr message) { navigation_local_map_=message; });
    viewpoint_manager_->SetNavigationCollisionChecker(
        [this](const Eigen::Vector3d &p) { return navigation_collision_map_.traversable(p); },
        [this](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
          return navigation_collision_map_.segment_traversable(a,b);
        });
    RCLCPP_INFO(get_logger(), "Exploration collision source: Nav2 global + local maps, with observed-free goals");
  }
  grid_world_ = std::make_shared<grid_world_ns::GridWorld>(shared_from_this());
  grid_world_->SetUseKeyposeGraph(true);
  local_coverage_planner_ =
      std::make_shared<local_coverage_planner_ns::LocalCoveragePlanner>(
          shared_from_this());
  local_coverage_planner_->SetViewPointManager(viewpoint_manager_);

  visualizer_ =
      std::make_shared<tare_visualizer_ns::TAREVisualizer>(shared_from_this());

  initial_position_.x() = 0.0;
  initial_position_.y() = 0.0;
  initial_position_.z() = 0.0;

  cur_keypose_node_ind_ = 0;

  keypose_graph_node_marker_ = std::make_shared<misc_utils_ns::Marker>(
      shared_from_this(), "keypose_graph_node_marker", kWorldFrameID);
  keypose_graph_node_marker_->SetType(visualization_msgs::msg::Marker::POINTS);
  keypose_graph_node_marker_->SetScale(0.4, 0.4, 0.1);
  keypose_graph_node_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);
  keypose_graph_edge_marker_ = std::make_shared<misc_utils_ns::Marker>(
      shared_from_this(), "keypose_graph_edge_marker", kWorldFrameID);
  keypose_graph_edge_marker_->SetType(
      visualization_msgs::msg::Marker::LINE_LIST);
  keypose_graph_edge_marker_->SetScale(0.05, 0.0, 0.0);
  keypose_graph_edge_marker_->SetColorRGBA(1.0, 1.0, 0.0, 0.9);

  nogo_boundary_marker_ = std::make_shared<misc_utils_ns::Marker>(
      shared_from_this(), "nogo_boundary_marker", kWorldFrameID);
  nogo_boundary_marker_->SetType(visualization_msgs::msg::Marker::LINE_LIST);
  nogo_boundary_marker_->SetScale(0.05, 0.0, 0.0);
  nogo_boundary_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  grid_world_marker_ = std::make_shared<misc_utils_ns::Marker>(
      shared_from_this(), "grid_world_marker", kWorldFrameID);
  grid_world_marker_->SetType(visualization_msgs::msg::Marker::CUBE_LIST);
  grid_world_marker_->SetScale(1.0, 1.0, 1.0);
  grid_world_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  robot_yaw_ = 0.0;
  goal_selection_.parameters=rover_explore::read_goal_selection(*this);
  ResetGoalPolicies();
  lookahead_point_ = Eigen::Vector3d::Zero();
  lookahead_point_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  moving_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  moving_forward_ = true;

  Eigen::Vector3d viewpoint_resolution = viewpoint_manager_->GetResolution();
  double add_non_keypose_node_min_dist =
      std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;
  keypose_graph_->SetAddNonKeyposeNodeMinDist() = add_non_keypose_node_min_dist;

  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;

  last_robot_position_ = robot_position_;
}

SensorCoveragePlanner3D::SensorCoveragePlanner3D()
    : Node("rover_explore"), keypose_cloud_update_(false),
      initialized_(false), lookahead_point_update_(false), relocation_(false),
      start_exploration_(false), exploration_finished_(false),
      near_home_(false), at_home_(false), stopped_(false),
      test_point_update_(false), viewpoint_ind_update_(false), step_(false),
      use_momentum_(false), lookahead_point_in_line_of_sight_(true),
      reset_waypoint_(false), registered_cloud_count_(0), keypose_count_(0),
      direction_change_count_(0), direction_no_change_count_(0),
      momentum_activation_count_(0), reset_waypoint_joystick_axis_value_(-1.0) {
  std::cout << "finished constructor" << std::endl;
}

bool SensorCoveragePlanner3D::initialize() {
  ReadParameters();
  rover_explore::validate_configuration(*this);
  self_filter_ = rover_explore::read_self_filter(*this);
  // if (!ReadParameters(shared_from_this()))
  // {
  //   RCLCPP_ERROR(this->get_logger(), "Read parameters failed");
  //   return false;
  // }

  // Initialize(shared_from_this());
  InitializeData();

  RCLCPP_INFO(get_logger(),
      "Rover exploration: %.1f m sensor range, %.1f m lookahead, %.2f m surface/coverage voxel; fixed-return filter %s",
      get_parameter("kSensorRange").as_double(), kLookAheadDistance,
      get_parameter("kCoveragePointCloudResolution").as_double(),
      self_filter_.enabled ? "enabled" : "disabled");

  keypose_graph_->SetAllowVerticalEdge(false);

  lidar_model_ns::LiDARModel::setCloudDWZResol(
      planning_env_->GetPlannerCloudResolution());

  execution_timer_ = this->create_wall_timer(
      1000ms, std::bind(&SensorCoveragePlanner3D::execute, this));

  exploration_start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      sub_start_exploration_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&SensorCoveragePlanner3D::ExplorationStartCallback, this,
                std::placeholders::_1));
  raw_lidar_sub_=create_subscription<livox_ros_driver2::msg::CustomMsg>(
      get_parameter("corridor.ceiling_exit.raw_topic").as_string(),rclcpp::SensorDataQoS(),
      std::bind(&SensorCoveragePlanner3D::RawLidarCallback,this,std::placeholders::_1));
  registered_scan_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          sub_registered_scan_topic_, rclcpp::SensorDataQoS(),
          std::bind(&SensorCoveragePlanner3D::RegisteredScanCallback, this,
                    std::placeholders::_1));
  terrain_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      sub_terrain_map_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SensorCoveragePlanner3D::TerrainMapCallback, this,
                std::placeholders::_1));
  terrain_map_ext_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          sub_terrain_map_ext_topic_, rclcpp::SensorDataQoS(),
          std::bind(&SensorCoveragePlanner3D::TerrainMapExtCallback, this,
                    std::placeholders::_1));
  state_estimation_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      sub_state_estimation_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SensorCoveragePlanner3D::StateEstimationCallback, this,
                std::placeholders::_1));
  coverage_boundary_sub_ =
      this->create_subscription<geometry_msgs::msg::PolygonStamped>(
          sub_coverage_boundary_topic_, 5,
          std::bind(&SensorCoveragePlanner3D::CoverageBoundaryCallback, this,
                    std::placeholders::_1));
  viewpoint_boundary_sub_ =
      this->create_subscription<geometry_msgs::msg::PolygonStamped>(
          sub_viewpoint_boundary_topic_, 5,
          std::bind(&SensorCoveragePlanner3D::ViewPointBoundaryCallback, this,
                    std::placeholders::_1));
  nogo_boundary_sub_ =
      this->create_subscription<geometry_msgs::msg::PolygonStamped>(
          sub_nogo_boundary_topic_, 5,
          std::bind(&SensorCoveragePlanner3D::NogoBoundaryCallback, this,
                    std::placeholders::_1));
  joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      sub_joystick_topic_, 5,
      std::bind(&SensorCoveragePlanner3D::JoystickCallback, this,
                std::placeholders::_1));
  reset_waypoint_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      sub_reset_waypoint_topic_, 1,
      std::bind(&SensorCoveragePlanner3D::ResetWaypointCallback, this,
                std::placeholders::_1));

  global_path_full_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>("global_path_full", 1);
  global_path_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>("global_path", 1);
  old_global_path_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>("old_global_path", 1);
  to_nearest_global_subspace_path_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>(
          "to_nearest_global_subspace_path", 1);
  local_tsp_path_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>("local_path", 1);
  exploration_path_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>("exploration_path", 1);
  waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      pub_waypoint_topic_, 2);
  exploration_finish_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      pub_exploration_finish_topic_, 2);
  runtime_breakdown_pub_ =
      this->create_publisher<std_msgs::msg::Int32MultiArray>(
          pub_runtime_breakdown_topic_, 2);
  runtime_pub_ =
      this->create_publisher<std_msgs::msg::Float32>(pub_runtime_topic_, 2);
  momentum_activation_count_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      pub_momentum_activation_count_topic_, 2);
  // Debug
  pointcloud_manager_neighbor_cells_origin_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          "pointcloud_manager_neighbor_cells_origin", 1);

  status_pub_ = create_publisher<std_msgs::msg::String>("/explore/status", rclcpp::QoS(1).transient_local());
  control_path_pub_ = create_publisher<nav_msgs::msg::Path>("/explore/control_path", 2);
  cloud_exit_debug_pub_ = create_publisher<std_msgs::msg::String>("/explore/exit_detection", rclcpp::QoS(1).transient_local());
  corridor_phase_pub_ = create_publisher<std_msgs::msg::String>("/explore/corridor_phase", rclcpp::QoS(1).transient_local());
  corridor_path_pub_ = create_publisher<nav_msgs::msg::Path>("/explore/reference_path", rclcpp::QoS(1).transient_local());
  corridor_centerline_pub_ = create_publisher<nav_msgs::msg::Path>("/explore/corridor_centerline", rclcpp::QoS(1).transient_local());
  corridor_boundary_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/explore/corridor_boundary", rclcpp::QoS(1).transient_local());
  corridor_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/explore/corridor_selection", rclcpp::QoS(1).transient_local());
  dead_end_pub_=create_publisher<std_msgs::msg::String>("/explore/dead_end",1);
  reorient_service_=create_service<std_srvs::srv::Trigger>("/explore/reorient_corridor",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        const double age=(now()-last_odom_time_).seconds();
        if(start_exploration_ || !corridor_.parameters.enabled || !odometry_received_ ||
           age< -sync_tolerance_ || age>input_timeout_ || corridor_.exiting() || corridor_.exited()) {
          response->message="Reorientation requires disabled cave exploration and fresh odometry before exit";
          return;
        }
        corridor_.reorient(Eigen::Vector3d(robot_position_.x,robot_position_.y,robot_position_.z),robot_yaw_);
        goal_selection_.reset();lookahead_point_update_=false;
        nav_msgs::msg::Path empty;empty.header.frame_id=kWorldFrameID;empty.header.stamp=now();
        corridor_centerline_pub_->publish(empty);corridor_path_pub_->publish(empty);control_path_pub_->publish(empty);
        response->success=true;response->message="Corridor direction set from completed turnaround odometry";
      });
  reset_service_ = create_service<std_srvs::srv::Trigger>("/explore/reset",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (start_exploration_) {
          response->success = false;
          response->message = "Disable exploration before reset";
          return;
        }
        InitializeData();
        ceiling_exit_.reset();
        keypose_graph_->SetAllowVerticalEdge(false);
        initialized_ = false;
        home_initialized_ = false;
        cloud_received_ = terrain_received_ = odometry_received_ = false;
        keypose_cloud_update_ = exploration_finished_ = stopped_ = false;
        coverage_confirmation_.reset();
        lookahead_point_update_ = relocation_ = near_home_ = at_home_ = false;
        use_momentum_ = reset_waypoint_ = false;
        lookahead_point_in_line_of_sight_ = true;
        registered_cloud_count_ = keypose_count_ = 0;
        direction_change_count_ = direction_no_change_count_ = momentum_activation_count_ = 0;
        visited_positions_.clear();
        exploration_path_.nodes_.clear();
        odometry_buffer_.clear();
        PublishStatus("DISABLED");
        response->success = true;
        response->message = "Exploration reset; home will be captured when enabled";
      });
  PublishStatus("DISABLED");
  return true;
}

void SensorCoveragePlanner3D::ExplorationStartCallback(
    const std_msgs::msg::Bool::ConstSharedPtr start_msg) {
  if (start_msg->data && !start_exploration_ && !home_initialized_ && odometry_received_) {
    initial_position_ = Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z);
    home_initialized_ = true;
  }
  const bool pausing=start_exploration_&&!start_msg->data;
  start_exploration_ = start_msg->data;
  if(pausing){goal_selection_.reset();corridor_.release_target();lookahead_point_update_=false;}
  if (!start_exploration_) {
    PublishStatus("DISABLED");
  }
}

void SensorCoveragePlanner3D::StateEstimationCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr state_estimation_msg) {
  if (state_estimation_msg->header.frame_id != kWorldFrameID) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Odometry must use odom_init frame");
    return;
  }
  if (!std::isfinite(state_estimation_msg->pose.pose.position.x) ||
      !std::isfinite(state_estimation_msg->pose.pose.position.y) ||
      !std::isfinite(state_estimation_msg->pose.pose.position.z)) return;
  const auto &orientation = state_estimation_msg->pose.pose.orientation;
  const Eigen::Quaterniond quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
  if (state_estimation_msg->child_frame_id != "base_frame" ||
      !quaternion.coeffs().allFinite() || std::abs(quaternion.norm() - 1.0) > 0.05) return;
  const auto odom_stamp = rclcpp::Time(state_estimation_msg->header.stamp);
  if (odometry_received_ && odom_stamp <= last_odom_time_) return;
  robot_position_ = state_estimation_msg->pose.pose.position;
  last_odom_time_ = rclcpp::Time(state_estimation_msg->header.stamp);
  odometry_received_ = true;
  odometry_buffer_.push_back(*state_estimation_msg);
  while (odometry_buffer_.size() > 400) odometry_buffer_.pop_front();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geo_quat =
      state_estimation_msg->pose.pose.orientation;
  tf2::Matrix3x3(
      tf2::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w))
      .getRPY(roll, pitch, yaw);

  robot_yaw_ = yaw;

  if (state_estimation_msg->twist.twist.linear.x > 0.4) {
    moving_forward_ = true;
  } else if (state_estimation_msg->twist.twist.linear.x < -0.4) {
    moving_forward_ = false;
  }
  // initialized_ = true;
}

void SensorCoveragePlanner3D::RawLidarCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
  if(!corridor_.parameters.enabled||!corridor_.parameters.exit_enabled||(!kAutoStart&&!start_exploration_))return;
  if(odometry_buffer_.empty()||!navigation_tf_buffer_)return;
  const rclcpp::Time timestamp(message->header.stamp);
  const double age=(now()-timestamp).seconds();
  if(age>input_timeout_||age<-sync_tolerance_)return;
  const auto odometry=std::min_element(odometry_buffer_.begin(),odometry_buffer_.end(),
      [&timestamp](const auto&a,const auto&b){return std::abs((rclcpp::Time(a.header.stamp)-timestamp).seconds())<
          std::abs((rclcpp::Time(b.header.stamp)-timestamp).seconds());});
  if(std::abs((rclcpp::Time(odometry->header.stamp)-timestamp).seconds())>sync_tolerance_)return;
  const auto &p=odometry->pose.pose.position;const auto &q=odometry->pose.pose.orientation;
  const Eigen::Vector3d position(p.x,p.y,p.z);
  const Eigen::Matrix3d rotation=Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
  Eigen::Matrix3d sensor_rotation=Eigen::Matrix3d::Identity();Eigen::Vector3d sensor_translation=Eigen::Vector3d::Zero();
  const std::string base=odometry->child_frame_id.empty()?"base_frame":odometry->child_frame_id;
  try {
    if(message->header.frame_id!=base){
      const auto transform=navigation_tf_buffer_->lookupTransform(base,message->header.frame_id,tf2::TimePointZero);
      const auto &r=transform.transform.rotation;const auto&t=transform.transform.translation;
      sensor_rotation=Eigen::Quaterniond(r.w,r.x,r.y,r.z).normalized().toRotationMatrix();
      sensor_translation=Eigen::Vector3d(t.x,t.y,t.z);
    }
  }catch(const std::exception&e){
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Ceiling exit waits for raw lidar transform: %s",e.what());return;
  }
  std::vector<Eigen::Vector3d> world;world.reserve(message->points.size());
  for(const auto &point:message->points)
    world.push_back(position+rotation*(sensor_rotation*Eigen::Vector3d(point.x,point.y,point.z)+sensor_translation));
  const double yaw=std::atan2(rotation(1,0),rotation(0,0));
  const bool previously_detected=bool(ceiling_exit_.detection());
  ceiling_exit_.update(world,position,yaw,timestamp.seconds(),
      [this,&timestamp](const Eigen::Vector3d&point){return ground_observations_.height_at(point,timestamp.seconds());},
      position+rotation*sensor_translation);
  const auto &v=ceiling_exit_.statistics;const auto &detected=ceiling_exit_.detection();
  if(detected&&!previously_detected){
    PublishStatus("EXIT_FORWARD");
    RCLCPP_INFO(get_logger(),"Ceiling exit confirmed: high fraction=%.3f >= %.3f, high/upper=%d/%d above %.2fm lidar-relative; continue %.2f m",
        v.high_fraction,ceiling_exit_.parameters.minimum_high_fraction,v.high_points,v.upper_points,ceiling_exit_.parameters.height_threshold,corridor_.parameters.exit_forward_distance);
  }
  const double progress=detected?(position-detected->position).head<2>().dot(
      Eigen::Vector2d(std::cos(detected->yaw),std::sin(detected->yaw))):0.;
  std_msgs::msg::String debug;std::ostringstream report;report.precision(16);
  report<<"{\"method\":\"raw_upper_returns_relative_lidar\",\"stamp\":"<<timestamp.seconds()
        <<",\"height_threshold\":"<<ceiling_exit_.parameters.height_threshold
        <<",\"high_fraction\":"<<v.high_fraction<<",\"minimum_high_fraction\":"<<ceiling_exit_.parameters.minimum_high_fraction
        <<",\"total\":"<<v.total<<",\"upper_points\":"<<v.upper_points<<",\"upper_cells\":"<<v.upper_cells
        <<",\"low_points\":"<<v.low_points<<",\"high_points\":"<<v.high_points
        <<",\"minimum_height\":"<<v.minimum_height<<",\"maximum_height\":"<<v.maximum_height
        <<",\"enough_returns\":"<<(v.enough_returns?"true":"false")
        <<",\"candidate\":"<<(v.candidate?"true":"false")<<",\"confirm_seconds\":"<<v.confirm_seconds
        <<",\"detected\":"<<(detected?"true":"false")<<",\"forward_progress\":"<<progress
        <<",\"forward_remaining\":"<<std::max(0.,corridor_.parameters.exit_forward_distance-progress)<<"}";
  debug.data=report.str();cloud_exit_debug_pub_->publish(debug);
  RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
      "Ceiling exit (lidar-relative vertical): upper=%d cells=%d low/high=%d/%d height=%.2f..%.2fm threshold=%.2fm fraction=%.3f required=%.3f enough=%d candidate=%d hold=%.2f detected=%d forward=%.2fm",
      v.upper_points,v.upper_cells,v.low_points,v.high_points,v.minimum_height,v.maximum_height,
      ceiling_exit_.parameters.height_threshold,v.high_fraction,ceiling_exit_.parameters.minimum_high_fraction,v.enough_returns,v.candidate,v.confirm_seconds,bool(detected),progress);
}

void SensorCoveragePlanner3D::RegisteredScanCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg) {
  if (registered_scan_msg->header.frame_id != kWorldFrameID || odometry_buffer_.empty()) return;
  const rclcpp::Time stamp(registered_scan_msg->header.stamp);
  if (cloud_received_ && stamp <= last_cloud_time_) return;
  const double age = (now() - stamp).seconds();
  if (age < -sync_tolerance_ || age > input_timeout_) return;
  const auto closest = std::min_element(odometry_buffer_.begin(), odometry_buffer_.end(),
      [&stamp](const auto &a, const auto &b) {
        return std::abs((rclcpp::Time(a.header.stamp) - stamp).seconds()) <
               std::abs((rclcpp::Time(b.header.stamp) - stamp).seconds());
      });
  if (std::abs((rclcpp::Time(closest->header.stamp) - stamp).seconds()) > sync_tolerance_) return;
  if (registered_scan_msg->width * registered_scan_msg->height == 0) {
    if(!ceiling_exit_.detection())ceiling_exit_.invalidate();
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_tmp(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*registered_scan_msg, *registered_scan_tmp);
  auto &points = registered_scan_tmp->points;
  const auto &pose = closest->pose.pose;
  const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
  const auto &q = pose.orientation;
  const Eigen::Matrix3d rotation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  points.erase(std::remove_if(points.begin(), points.end(), [this, &position, &rotation](const pcl::PointXYZ &p) {
    return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      self_filter_.contains_world(Eigen::Vector3d(p.x, p.y, p.z), position, rotation);
  }), points.end());
  registered_scan_tmp->width = points.size();
  registered_scan_tmp->height = 1;
  registered_scan_tmp->is_dense = true;
  if (registered_scan_tmp->points.empty()) {
    if(!ceiling_exit_.detection())ceiling_exit_.invalidate();
    return;
  }
  if(corridor_.parameters.enabled) {
    std::vector<Eigen::Vector3d> ground_samples;ground_samples.reserve(registered_scan_tmp->size());
    for(const auto&p:*registered_scan_tmp)ground_samples.emplace_back(p.x,p.y,p.z);
    ground_observations_.update(ground_samples,position,stamp.seconds());
    const auto floor=ground_observations_.height_at(position,stamp.seconds());
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
        "Observed ground: fitted=%d floor_z=%.3f odometry_z=%.3f cells=%zu inliers=%zu",
        bool(floor),floor.value_or(std::numeric_limits<double>::quiet_NaN()),position.z(),
        ground_observations_.size(),ground_observations_.inliers());
  }
  cloud_received_ = true;
  last_cloud_time_ = stamp;
  if (!initialized_ || (!kAutoStart && !start_exploration_)) return;
  // Use the pose at the scan, rather than the latest high-frequency prediction.
  robot_position_ = pose.position;
  *(registered_scan_stack_->cloud_) += *(registered_scan_tmp);
  pointcloud_downsizer_.Downsize(
      registered_scan_tmp, kKeyposeCloudDwzFilterLeafSize,
      kKeyposeCloudDwzFilterLeafSize, kKeyposeCloudDwzFilterLeafSize);
  registered_cloud_->cloud_->clear();
  pcl::copyPointCloud(*registered_scan_tmp, *(registered_cloud_->cloud_));

  planning_env_->UpdateRobotPosition(robot_position_);
  planning_env_->UpdateRegisteredCloud<pcl::PointXYZI>(
      registered_cloud_->cloud_);

  registered_cloud_count_ = (registered_cloud_count_ + 1) % 5;
  if (registered_cloud_count_ == 0) {
    // initialized_ = true;
    keypose_.pose.pose.position = robot_position_;
    keypose_.pose.covariance[0] = keypose_count_++;
    cur_keypose_node_ind_ =
        keypose_graph_->AddKeyposeNode(keypose_, *(planning_env_));

    pointcloud_downsizer_.Downsize(
        registered_scan_stack_->cloud_, kKeyposeCloudDwzFilterLeafSize,
        kKeyposeCloudDwzFilterLeafSize, kKeyposeCloudDwzFilterLeafSize);

    keypose_cloud_->cloud_->clear();
    pcl::copyPointCloud(*(registered_scan_stack_->cloud_),
                        *(keypose_cloud_->cloud_));
    // keypose_cloud_->Publish();
    registered_scan_stack_->cloud_->clear();
    keypose_cloud_update_ = true;
  }
}

void SensorCoveragePlanner3D::TerrainMapCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_msg) {
  if (terrain_map_msg->header.frame_id != kWorldFrameID) return;
  terrain_received_ = true;
  last_terrain_time_ = rclcpp::Time(terrain_map_msg->header.stamp);
  if (kCheckTerrainCollision) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_map_tmp(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_msg, *terrain_map_tmp);
    terrain_collision_cloud_->cloud_->clear();
    for (auto &point : terrain_map_tmp->points) {
      if (point.intensity > kTerrainCollisionThreshold) {
        terrain_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}

void SensorCoveragePlanner3D::TerrainMapExtCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_ext_msg) {
  if (terrain_map_ext_msg->header.frame_id != kWorldFrameID) return;
  if (kUseTerrainHeight) {
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg,
                                    *(large_terrain_cloud_->cloud_));
  }
  if (kCheckTerrainCollision) {
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg,
                                    *(large_terrain_cloud_->cloud_));
    terrain_ext_collision_cloud_->cloud_->clear();
    for (auto &point : large_terrain_cloud_->cloud_->points) {
      if (point.intensity > kTerrainCollisionThreshold) {
        terrain_ext_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}

void SensorCoveragePlanner3D::CoverageBoundaryCallback(
    const geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_msg) {
  planning_env_->UpdateCoverageBoundary((*polygon_msg).polygon);
}

void SensorCoveragePlanner3D::ViewPointBoundaryCallback(
    const geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_msg) {
  viewpoint_manager_->UpdateViewPointBoundary((*polygon_msg).polygon);
}

void SensorCoveragePlanner3D::NogoBoundaryCallback(
    const geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_msg) {
  if (polygon_msg->polygon.points.empty()) {
    return;
  }
  double polygon_id = polygon_msg->polygon.points[0].z;
  int polygon_point_size = polygon_msg->polygon.points.size();
  std::vector<geometry_msgs::msg::Polygon> nogo_boundary;
  geometry_msgs::msg::Polygon polygon;
  for (int i = 0; i < polygon_point_size; i++) {
    if (polygon_msg->polygon.points[i].z == polygon_id) {
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    } else {
      nogo_boundary.push_back(polygon);
      polygon.points.clear();
      polygon_id = polygon_msg->polygon.points[i].z;
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    }
  }
  nogo_boundary.push_back(polygon);
  viewpoint_manager_->UpdateNogoBoundary(nogo_boundary);

  geometry_msgs::msg::Point point;
  for (int i = 0; i < nogo_boundary.size(); i++) {
    for (int j = 0; j < nogo_boundary[i].points.size() - 1; j++) {
      point.x = nogo_boundary[i].points[j].x;
      point.y = nogo_boundary[i].points[j].y;
      point.z = nogo_boundary[i].points[j].z;
      nogo_boundary_marker_->marker_.points.push_back(point);
      point.x = nogo_boundary[i].points[j + 1].x;
      point.y = nogo_boundary[i].points[j + 1].y;
      point.z = nogo_boundary[i].points[j + 1].z;
      nogo_boundary_marker_->marker_.points.push_back(point);
    }
    point.x = nogo_boundary[i].points.back().x;
    point.y = nogo_boundary[i].points.back().y;
    point.z = nogo_boundary[i].points.back().z;
    nogo_boundary_marker_->marker_.points.push_back(point);
    point.x = nogo_boundary[i].points.front().x;
    point.y = nogo_boundary[i].points.front().y;
    point.z = nogo_boundary[i].points.front().z;
    nogo_boundary_marker_->marker_.points.push_back(point);
  }
  nogo_boundary_marker_->Publish();
}

void SensorCoveragePlanner3D::JoystickCallback(
    const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg) {
  if (kResetWaypointJoystickAxesID >= 0 &&
      kResetWaypointJoystickAxesID < joy_msg->axes.size()) {
    if (reset_waypoint_joystick_axis_value_ > -0.1 &&
        joy_msg->axes[kResetWaypointJoystickAxesID] < -0.1) {
      reset_waypoint_ = true;

      // Set waypoint to the current robot position to stop the robot in place
      geometry_msgs::msg::PointStamped waypoint;
      waypoint.header.frame_id = "odom_init";
      waypoint.header.stamp = this->now();
      waypoint.point.x = robot_position_.x;
      waypoint.point.y = robot_position_.y;
      waypoint.point.z = robot_position_.z;
      waypoint_pub_->publish(waypoint);
      std::cout << "reset waypoint" << std::endl;
    }
    reset_waypoint_joystick_axis_value_ =
        joy_msg->axes[kResetWaypointJoystickAxesID];
  }
}

void SensorCoveragePlanner3D::ResetWaypointCallback(
    const std_msgs::msg::Empty::ConstSharedPtr empty_msg) {
  reset_waypoint_ = true;

  // Set waypoint to the current robot position to stop the robot in place
  geometry_msgs::msg::PointStamped waypoint;
  waypoint.header.frame_id = "odom_init";
  waypoint.header.stamp = this->now();
  waypoint.point.x = robot_position_.x;
  waypoint.point.y = robot_position_.y;
  waypoint.point.z = robot_position_.z;
  waypoint_pub_->publish(waypoint);
  std::cout << "reset waypoint" << std::endl;
}

void SensorCoveragePlanner3D::SendInitialWaypoint() {
  // send waypoint ahead
  double lx = 12.0;
  double ly = 0.0;
  double dx = cos(robot_yaw_) * lx - sin(robot_yaw_) * ly;
  double dy = sin(robot_yaw_) * lx + cos(robot_yaw_) * ly;

  geometry_msgs::msg::PointStamped waypoint;
  waypoint.header.frame_id = "odom_init";
  waypoint.header.stamp = this->now();
  waypoint.point.x = robot_position_.x + dx;
  waypoint.point.y = robot_position_.y + dy;
  waypoint.point.z = robot_position_.z;
  waypoint_pub_->publish(waypoint);
}

void SensorCoveragePlanner3D::UpdateKeyposeGraph() {
  misc_utils_ns::Timer update_keypose_graph_timer("update keypose graph");
  update_keypose_graph_timer.Start();

  keypose_graph_->GetMarker(keypose_graph_node_marker_->marker_,
                            keypose_graph_edge_marker_->marker_);
  // keypose_graph_node_marker_->Publish();
  keypose_graph_edge_marker_->Publish();
  keypose_graph_vis_cloud_->cloud_->clear();
  keypose_graph_->CheckLocalCollision(robot_position_, viewpoint_manager_);
  keypose_graph_->CheckConnectivity(robot_position_);
  keypose_graph_->GetVisualizationCloud(keypose_graph_vis_cloud_->cloud_);
  keypose_graph_vis_cloud_->Publish();

  update_keypose_graph_timer.Stop(false);
}

int SensorCoveragePlanner3D::UpdateViewPoints() {
  misc_utils_ns::Timer collision_cloud_timer("update collision cloud");
  collision_cloud_timer.Start();
  collision_cloud_->cloud_ = planning_env_->GetCollisionCloud();
  collision_cloud_timer.Stop(false);

  misc_utils_ns::Timer viewpoint_manager_update_timer(
      "update viewpoint manager");
  viewpoint_manager_update_timer.Start();
  if (kUseTerrainHeight) {
    viewpoint_manager_->SetViewPointHeightWithTerrain(
        large_terrain_cloud_->cloud_);
  }
  if (kCheckTerrainCollision) {
    *(collision_cloud_->cloud_) += *(terrain_collision_cloud_->cloud_);
    *(collision_cloud_->cloud_) += *(terrain_ext_collision_cloud_->cloud_);
  }
  viewpoint_manager_->CheckViewPointCollision(collision_cloud_->cloud_);
  viewpoint_manager_->CheckViewPointLineOfSight();
  viewpoint_manager_->CheckViewPointConnectivity();
  int viewpoint_candidate_count = viewpoint_manager_->GetViewPointCandidate();

  UpdateVisitedPositions();
  viewpoint_manager_->UpdateViewPointVisited(visited_positions_);
  viewpoint_manager_->UpdateViewPointVisited(grid_world_);

  // For visualization
  collision_cloud_->Publish();
  // collision_grid_cloud_->Publish();
  viewpoint_manager_->GetCollisionViewPointVisCloud(
      viewpoint_in_collision_cloud_->cloud_);
  viewpoint_in_collision_cloud_->Publish();

  viewpoint_manager_update_timer.Stop(false);
  return viewpoint_candidate_count;
}

void SensorCoveragePlanner3D::UpdateViewPointCoverage() {
  // Update viewpoint coverage
  misc_utils_ns::Timer update_coverage_timer("update viewpoint coverage");
  update_coverage_timer.Start();
  if (get_parameter("kCheckDynamicObstacleCollision").as_bool()) {
    viewpoint_manager_->RebuildCandidateViewPointCoverage<pcl::PointXYZI>(
        planning_env_->GetCoverageOcclusionCloud());
  } else {
    viewpoint_manager_->UpdateViewPointCoverage<PlannerCloudPointType>(
        planning_env_->GetDiffCloud());
    viewpoint_manager_->UpdateRolledOverViewPointCoverage<PlannerCloudPointType>(
        planning_env_->GetStackedCloud());
  }
  // Update robot coverage
  robot_viewpoint_.ResetCoverage();
  geometry_msgs::msg::Pose robot_pose;
  robot_pose.position = robot_position_;
  robot_viewpoint_.setPose(robot_pose);
  UpdateRobotViewPointCoverage();
  update_coverage_timer.Stop(false);
}

void SensorCoveragePlanner3D::UpdateRobotViewPointCoverage() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud =
      planning_env_->GetCollisionCloud();
  for (const auto &point : cloud->points) {
    if (viewpoint_manager_->InFOVAndRange(
            Eigen::Vector3d(point.x, point.y, point.z),
            Eigen::Vector3d(robot_position_.x, robot_position_.y,
                            robot_position_.z))) {
      robot_viewpoint_.UpdateCoverage<pcl::PointXYZI>(point);
    }
  }
}

void SensorCoveragePlanner3D::UpdateCoveredAreas(
    int &uncovered_point_num, int &uncovered_frontier_point_num) {
  // Update covered area
  misc_utils_ns::Timer update_coverage_area_timer("update covered area");
  update_coverage_area_timer.Start();
  planning_env_->UpdateCoveredArea(robot_viewpoint_, viewpoint_manager_);

  update_coverage_area_timer.Stop(false);
  misc_utils_ns::Timer get_uncovered_area_timer("get uncovered area");
  get_uncovered_area_timer.Start();
  planning_env_->GetUncoveredArea(viewpoint_manager_, uncovered_point_num,
                                  uncovered_frontier_point_num);

  get_uncovered_area_timer.Stop(false);
  planning_env_->PublishUncoveredCloud();
  planning_env_->PublishUncoveredFrontierCloud();
}

void SensorCoveragePlanner3D::UpdateVisitedPositions() {
  Eigen::Vector3d robot_current_position(robot_position_.x, robot_position_.y,
                                         robot_position_.z);
  // Sample densely enough that the smaller visited radius leaves no gaps.
  const double spacing = std::min(0.5, get_parameter("kNeighborRange").as_double() / 2.0);
  bool existing = false;
  for (int i = 0; i < visited_positions_.size(); i++) {
    if ((robot_current_position - visited_positions_[i]).norm() < spacing) {
      existing = true;
      break;
    }
  }
  if (!existing) {
    visited_positions_.push_back(robot_current_position);
  }
}

void SensorCoveragePlanner3D::UpdateGlobalRepresentation() {
  local_coverage_planner_->SetRobotPosition(
      Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z));
  bool viewpoint_rollover = viewpoint_manager_->UpdateRobotPosition(
      Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z));
  viewpoint_manager_->SetRobotYaw(robot_yaw_);
  if (!grid_world_->Initialized() || viewpoint_rollover) {
    grid_world_->UpdateNeighborCells(robot_position_);
  }

  planning_env_->UpdateRobotPosition(robot_position_);
  planning_env_->GetVisualizationPointCloud(
      point_cloud_manager_neighbor_cloud_->cloud_);
  point_cloud_manager_neighbor_cloud_->Publish();

  // DEBUG
  Eigen::Vector3d pointcloud_manager_neighbor_cells_origin =
      planning_env_->GetPointCloudManagerNeighborCellsOrigin();
  geometry_msgs::msg::PointStamped
      pointcloud_manager_neighbor_cells_origin_point;
  pointcloud_manager_neighbor_cells_origin_point.header.frame_id = "odom_init";
  pointcloud_manager_neighbor_cells_origin_point.header.stamp = this->now();
  pointcloud_manager_neighbor_cells_origin_point.point.x =
      pointcloud_manager_neighbor_cells_origin.x();
  pointcloud_manager_neighbor_cells_origin_point.point.y =
      pointcloud_manager_neighbor_cells_origin.y();
  pointcloud_manager_neighbor_cells_origin_point.point.z =
      pointcloud_manager_neighbor_cells_origin.z();
  pointcloud_manager_neighbor_cells_origin_pub_->publish(
      pointcloud_manager_neighbor_cells_origin_point);

  if (exploration_finished_ && kNoExplorationReturnHome) {
    planning_env_->SetUseFrontier(false);
  }
  planning_env_->UpdateKeyposeCloud<PlannerCloudPointType>(
      keypose_cloud_->cloud_);

  int closest_node_ind = keypose_graph_->GetClosestNodeInd(robot_position_);
  geometry_msgs::msg::Point closest_node_position =
      keypose_graph_->GetClosestNodePosition(robot_position_);
  grid_world_->SetCurKeyposeGraphNodeInd(closest_node_ind);
  grid_world_->SetCurKeyposeGraphNodePosition(closest_node_position);

  grid_world_->UpdateRobotPosition(robot_position_);
  if (!grid_world_->HomeSet()) {
    grid_world_->SetHomePosition(initial_position_);
  }
}

void SensorCoveragePlanner3D::GlobalPlanning(
    std::vector<int> &global_cell_tsp_order,
    exploration_path_ns::ExplorationPath &global_path) {
  misc_utils_ns::Timer global_tsp_timer("Global planning");
  global_tsp_timer.Start();

  grid_world_->UpdateCellStatus(viewpoint_manager_);
  grid_world_->UpdateCellKeyposeGraphNodes(keypose_graph_);
  grid_world_->AddPathsInBetweenCells(viewpoint_manager_, keypose_graph_);

  viewpoint_manager_->UpdateCandidateViewPointCellStatus(grid_world_);

  global_path = grid_world_->SolveGlobalTSP(
      viewpoint_manager_, global_cell_tsp_order, keypose_graph_);

  global_tsp_timer.Stop(false);
  global_planning_runtime_ = global_tsp_timer.GetDuration("ms");
}

void SensorCoveragePlanner3D::PublishGlobalPlanningVisualization(
    const exploration_path_ns::ExplorationPath &global_path,
    const exploration_path_ns::ExplorationPath &local_path) {
  nav_msgs::msg::Path global_path_full = global_path.GetPath();
  global_path_full.header.frame_id = "odom_init";
  global_path_full.header.stamp = this->now();
  global_path_full_publisher_->publish(global_path_full);
  // Get the part that connects with the local path

  int start_index = 0;
  for (int i = 0; i < global_path.nodes_.size(); i++) {
    if (global_path.nodes_[i].type_ ==
            exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
        global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
        !viewpoint_manager_->InLocalPlanningHorizon(
            global_path.nodes_[i].position_)) {
      break;
    }
    start_index = i;
  }

  int end_index = global_path.nodes_.size() - 1;
  for (int i = global_path.nodes_.size() - 1; i >= 0; i--) {
    if (global_path.nodes_[i].type_ ==
            exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
        global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
        !viewpoint_manager_->InLocalPlanningHorizon(
            global_path.nodes_[i].position_)) {
      break;
    }
    end_index = i;
  }

  nav_msgs::msg::Path global_path_trim;
  if (local_path.nodes_.size() >= 2) {
    geometry_msgs::msg::PoseStamped first_pose;
    first_pose.pose.position.x = local_path.nodes_.front().position_.x();
    first_pose.pose.position.y = local_path.nodes_.front().position_.y();
    first_pose.pose.position.z = local_path.nodes_.front().position_.z();
    global_path_trim.poses.push_back(first_pose);
  }

  for (int i = start_index; i <= end_index; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = global_path.nodes_[i].position_.x();
    pose.pose.position.y = global_path.nodes_[i].position_.y();
    pose.pose.position.z = global_path.nodes_[i].position_.z();
    global_path_trim.poses.push_back(pose);
  }
  if (local_path.nodes_.size() >= 2) {
    geometry_msgs::msg::PoseStamped last_pose;
    last_pose.pose.position.x = local_path.nodes_.back().position_.x();
    last_pose.pose.position.y = local_path.nodes_.back().position_.y();
    last_pose.pose.position.z = local_path.nodes_.back().position_.z();
    global_path_trim.poses.push_back(last_pose);
  }
  global_path_trim.header.frame_id = "odom_init";
  global_path_trim.header.stamp = this->now();
  global_path_publisher_->publish(global_path_trim);

  grid_world_->GetVisualizationCloud(grid_world_vis_cloud_->cloud_);
  grid_world_vis_cloud_->Publish();
  grid_world_->GetMarker(grid_world_marker_->marker_);
  grid_world_marker_->Publish();
  nav_msgs::msg::Path full_path = exploration_path_.GetPath();
  full_path.header.frame_id = "odom_init";
  full_path.header.stamp = this->now();
  // exploration_path_publisher_->publish(full_path);
  exploration_path_.GetVisualizationCloud(exploration_path_cloud_->cloud_);
  exploration_path_cloud_->Publish();
  // planning_env_->PublishStackedCloud();
}

void SensorCoveragePlanner3D::LocalPlanning(
    int uncovered_point_num, int uncovered_frontier_point_num,
    const exploration_path_ns::ExplorationPath &global_path,
    exploration_path_ns::ExplorationPath &local_path) {
  misc_utils_ns::Timer local_tsp_timer("Local planning");
  local_tsp_timer.Start();
  if (lookahead_point_update_) {
    local_coverage_planner_->SetLookAheadPoint(lookahead_point_);
  }
  local_path = local_coverage_planner_->SolveLocalCoverageProblem(
      global_path, uncovered_point_num, uncovered_frontier_point_num);
  local_tsp_timer.Stop(false);
}

void SensorCoveragePlanner3D::PublishLocalPlanningVisualization(
    const exploration_path_ns::ExplorationPath &local_path) {
  viewpoint_manager_->GetVisualizationCloud(viewpoint_vis_cloud_->cloud_);
  viewpoint_vis_cloud_->Publish();
  lookahead_point_cloud_->Publish();
  nav_msgs::msg::Path local_tsp_path = local_path.GetPath();
  local_tsp_path.header.frame_id = "odom_init";
  local_tsp_path.header.stamp = this->now();
  local_tsp_path_publisher_->publish(local_tsp_path);
  local_coverage_planner_->GetSelectedViewPointVisCloud(
      selected_viewpoint_vis_cloud_->cloud_);
  selected_viewpoint_vis_cloud_->Publish();

  // Visualize local planning horizon box
}

exploration_path_ns::ExplorationPath
SensorCoveragePlanner3D::ConcatenateGlobalLocalPath(
    const exploration_path_ns::ExplorationPath &global_path,
    const exploration_path_ns::ExplorationPath &local_path) {
  exploration_path_ns::ExplorationPath full_path;
  if (exploration_finished_ && near_home_ && kRushHome) {
    exploration_path_ns::Node node;
    node.position_.x() = robot_position_.x;
    node.position_.y() = robot_position_.y;
    node.position_.z() = robot_position_.z;
    node.type_ = exploration_path_ns::NodeType::ROBOT;
    full_path.nodes_.push_back(node);
    node.position_ = initial_position_;
    node.type_ = exploration_path_ns::NodeType::HOME;
    full_path.nodes_.push_back(node);
    return full_path;
  }

  // A short observed route is useful in a small cave. The original 3/5 m
  // cutoffs can discard every route in the rover's reduced planning horizon.
  if (local_path.nodes_.empty()) {
    return full_path;
  } else {
    full_path = local_path;
    if (local_path.nodes_.front().type_ ==
            exploration_path_ns::NodeType::LOCAL_PATH_END &&
        local_path.nodes_.back().type_ ==
            exploration_path_ns::NodeType::LOCAL_PATH_START) {
      full_path.Reverse();
    } else if (local_path.nodes_.front().type_ ==
                   exploration_path_ns::NodeType::LOCAL_PATH_START &&
               local_path.nodes_.back() == local_path.nodes_.front()) {
      full_path.nodes_.back().type_ =
          exploration_path_ns::NodeType::LOCAL_PATH_END;
    } else if (local_path.nodes_.front().type_ ==
                   exploration_path_ns::NodeType::LOCAL_PATH_END &&
               local_path.nodes_.back() == local_path.nodes_.front()) {
      full_path.nodes_.front().type_ =
          exploration_path_ns::NodeType::LOCAL_PATH_START;
    }
  }

  return full_path;
}

void SensorCoveragePlanner3D::ResetGoalPolicies() {
  goal_selection_.reset();
  corridor_.reset();
  ground_observations_.reset();
  if(corridor_centerline_pub_) {
    nav_msgs::msg::Path path;path.header.frame_id=kWorldFrameID;path.header.stamp=now();
    corridor_centerline_pub_->publish(path);
    corridor_path_pub_->publish(path);
    geometry_msgs::msg::PolygonStamped empty;empty.header=path.header;
    corridor_boundary_pub_->publish(empty);
  }
}

bool SensorCoveragePlanner3D::SelectStableGoal(
    const exploration_path_ns::ExplorationPath &global_path) {
  const Eigen::Vector3d robot(robot_position_.x, robot_position_.y, robot_position_.z);
  const auto valid=[&](const Eigen::Vector3d &p) {
    if(corridor_.parameters.enabled)return navigation_collision_map_.traversable(p);
    const int index=viewpoint_manager_->GetViewPointInd(p);
    return viewpoint_manager_->InRange(index) && viewpoint_manager_->IsViewPointCandidate(index);
  };
  std::optional<rover_explore::SelectedGoal> choice;
  size_t local_count=0,global_count=0;
  if (corridor_.parameters.enabled)
  {
    std::vector<Eigen::Vector3d> obstacles,observed;
    obstacles.reserve(terrain_collision_cloud_->cloud_->size());
    for (const auto &p:*terrain_collision_cloud_->cloud_) obstacles.emplace_back(p.x,p.y,p.z);
    // Fresh observed ground supplies the navigation targets, including the configured 1 m exit leg.
    for(double dx=-3.;dx<=3.;dx+=.15)for(double dy=-3.;dy<=3.;dy+=.15) {
      if(dx*dx+dy*dy>9.)continue;
      const Eigen::Vector3d p=robot+Eigen::Vector3d(dx,dy,0);
      if(ground_observations_.supports(p,now().seconds())&&valid(p))observed.push_back(p);
    }
    local_count=observed.size();
    if(const auto &exit=ceiling_exit_.detection();exit && corridor_.parameters.exit_enabled)
      corridor_.begin_cloud_exit(exit->position,exit->yaw);
    const auto target=corridor_.update(robot,robot_yaw_,now().seconds(),obstacles,observed,valid,
        [this](const Eigen::Vector3d&p){return ground_observations_.supports(p,now().seconds());},
        [this](const Eigen::Vector3d&p){return ground_observations_.height_at(p,now().seconds());},
        [this](const Eigen::Vector3d&p){return navigation_collision_map_.physical_cell(p);});
    const auto &end=corridor_.end_observation();
    // Real source times are included: republishing a cached fit/map never adds
    // confirmation time, and observations acquired during grasping are rejected.
    double observation_stamp=std::min(last_cloud_time_.seconds(),last_terrain_time_.seconds());
    if(navigation_global_map_&&navigation_local_map_)
      observation_stamp=std::min({observation_stamp,
          rclcpp::Time(navigation_global_map_->header.stamp).seconds(),
          rclcpp::Time(navigation_local_map_->header.stamp).seconds()});
    else observation_stamp=0;
    std_msgs::msg::String end_message;
    end_message.data=nlohmann::json({{"stamp",now().seconds()},{"observation_stamp",observation_stamp},
        {"frame",kWorldFrameID},{"closed",end.closed},{"position",{end.position.x(),end.position.y()}},
        {"yaw",end.yaw},{"width",end.width},{"reason",end.reason}}).dump();
    dead_end_pub_->publish(end_message);
    viewpoint_manager_->RefreshExplorationCandidates();
    nav_msgs::msg::Path reference;reference.header.frame_id=kWorldFrameID;reference.header.stamp=now();
    const auto &sections=corridor_.sections();
    for(size_t i=0;i<sections.size();++i) {
      geometry_msgs::msg::PoseStamped pose;pose.header=reference.header;
      pose.pose.position.x=sections[i].center.x();pose.pose.position.y=sections[i].center.y();pose.pose.position.z=sections[i].center.z();
      const auto d=sections[i==sections.size()-1 ? i : i+1].center-sections[i==sections.size()-1 && i>0 ? i-1 : i].center;
      const double yaw=std::atan2(d.y(),d.x());pose.pose.orientation.z=std::sin(yaw/2);pose.pose.orientation.w=std::cos(yaw/2);
      reference.poses.push_back(pose);
    }
    corridor_centerline_pub_->publish(reference);
    // Polygon order: left side forward, right side backward; widths are measured
    // corridor widths, distinct from the narrower goal-selection band.
    geometry_msgs::msg::PolygonStamped tube;tube.header=reference.header;
    std::vector<geometry_msgs::msg::Point32> left_side,right_side;
    for(size_t i=0;i<sections.size();++i) {
      const auto&a=sections[i==0?0:i-1].center;
      const auto&b=sections[i+1<sections.size()?i+1:i].center;
      const double yaw=std::atan2(b.y()-a.y(),b.x()-a.x());
      for(int side:{1,-1}) {
        geometry_msgs::msg::Point32 p;
        p.x=sections[i].center.x()-side*.5*sections[i].width*std::sin(yaw);
        p.y=sections[i].center.y()+side*.5*sections[i].width*std::cos(yaw);
        p.z=sections[i].center.z();
        (side==1?left_side:right_side).push_back(p);
      }
    }
    tube.polygon.points=left_side;
    tube.polygon.points.insert(tube.polygon.points.end(),right_side.rbegin(),right_side.rend());
    corridor_boundary_pub_->publish(tube);
    // The center path is a preference/reference only. Nav2 plans the executable
    // path and MPPI checks the actual footprint against real obstacles.
    nav_msgs::msg::Path center_path;center_path.header=reference.header;
    std::string path_reason="WAITING_FOR_OBSERVED_CENTER";
    if(target) {
      try {
        std::vector<rover_explore::CorridorSection> line;
        for(const auto&s:sections)line.push_back({s.center.x(),s.center.y(),s.width});
        const auto route=rover_explore::centerline_path(line,robot.x(),robot.y(),robot_yaw_,target->x(),target->y());
        for(const auto&p:route) {
          geometry_msgs::msg::PoseStamped pose;pose.header=reference.header;
          pose.pose.position.x=p.x;pose.pose.position.y=p.y;pose.pose.position.z=robot.z();
          pose.pose.orientation.z=std::sin(p.yaw/2);pose.pose.orientation.w=std::cos(p.yaw/2);
          center_path.poses.push_back(pose);
        }
      }catch(const std::exception&) {
        // A noisy or sharply changing fit can still supply a valid short goal;
        // an invalid reference spline must not suppress Nav2 planning.
      }
      path_reason="CENTER_GUIDANCE_READY";
    }
    corridor_path_pub_->publish(center_path);
    std_msgs::msg::String phase;
    phase.data=corridor_.exiting()?"EXIT":(corridor_.entered()?"FOLLOW":"ENTRY");
    corridor_phase_pub_->publish(phase);
    // Visualize center samples, endpoint, measured walls and the executable path.
    visualization_msgs::msg::MarkerArray debug;
    visualization_msgs::msg::Marker clear;clear.action=visualization_msgs::msg::Marker::DELETEALL;
    debug.markers.push_back(clear);
    const auto marker=[&](int id,int type,float r,float g,float b) {
      visualization_msgs::msg::Marker m;m.header=reference.header;m.ns="corridor_selection";
      m.id=id;m.type=type;m.action=m.ADD;m.pose.orientation.w=1.;
      m.color.r=r;m.color.g=g;m.color.b=b;m.color.a=1.;
      m.lifetime=rclcpp::Duration::from_seconds(2.5);return m;
    };
    const auto points=[](const auto&input) {
      std::vector<geometry_msgs::msg::Point> output;
      for(const auto&p:input){geometry_msgs::msg::Point q;q.x=p.x();q.y=p.y();q.z=p.z();output.push_back(q);}
      return output;
    };
    auto observed_marker=marker(0,visualization_msgs::msg::Marker::POINTS,.5,.5,.5);
    observed_marker.scale.x=observed_marker.scale.y=.035;observed_marker.points=points(observed);
    debug.markers.push_back(observed_marker);
    auto eligible=marker(1,visualization_msgs::msg::Marker::POINTS,0.,1.,0.);
    eligible.scale.x=eligible.scale.y=.075;eligible.points=points(corridor_.eligible_candidates());
    debug.markers.push_back(eligible);
    if(target){auto selected=marker(3,visualization_msgs::msg::Marker::SPHERE,1.,0.,1.);
      selected.scale.x=selected.scale.y=selected.scale.z=.15;
      selected.pose.position.x=target->x();selected.pose.position.y=target->y();selected.pose.position.z=target->z();
      if(!center_path.poses.empty())selected.pose.position=center_path.poses.back().pose.position;
      debug.markers.push_back(selected);}
    auto text=marker(4,visualization_msgs::msg::Marker::TEXT_VIEW_FACING,1.,1.,1.);
    text.scale.z=.12;text.pose.position.x=robot.x();text.pose.position.y=robot.y();text.pose.position.z=robot.z()+1.;
    text.text=path_reason+" | "+std::string(corridor_.exiting()?"EXIT_GUIDANCE":(corridor_.entered()?"FOLLOW":"ENTRY"))+
      " | fit="+corridor_.fit_source()+" | observed="+std::to_string(observed.size())+" eligible="+std::to_string(corridor_.eligible_candidates().size())+
      " | range="+std::to_string(corridor_.parameters.target_min_distance)+".."+
      std::to_string(corridor_.parameters.target_max_distance)+"m | endpoint="+(target?"READY":"WAITING");
    debug.markers.push_back(text);corridor_debug_pub_->publish(debug);

    if(target) {
      lookahead_point_=*target;lookahead_point_update_=true;
      lookahead_point_in_line_of_sight_=true;relocation_=false;
      return true; // Nav2 plans to the observed goal with flexible center preference.
    }
    return false;
  }
  else
  {
    const double minimum_surface=get_parameter("kMinAddPointNumSmall").as_int();
    const double minimum_frontier=get_parameter("kMinAddFrontierPointNum").as_int();
    std::vector<rover_explore::GoalOption> local, global;
    for (int index:viewpoint_manager_->GetExplorationViewPointCandidateIndices()) {
      if (viewpoint_manager_->ViewPointVisited(index) ||
          !viewpoint_manager_->ViewPointInExploringCell(index)) continue;
      const auto p=viewpoint_manager_->GetViewPointPosition(index);
      const double surface=viewpoint_manager_->GetViewPointCoveredPointNum(index)/minimum_surface;
      const double frontier=viewpoint_manager_->UseFrontier() ?
          viewpoint_manager_->GetViewPointCoveredFrontierPointNum(index)/minimum_frontier : 0;
      const double gain=std::max(surface,frontier);
      if (gain>=1) local.push_back({{p.x,p.y,p.z},gain,0});
    }
    // Use a short prefix of an existing global task route, never a local TSP
    // connection anchor or a HOME node masquerading as new exploration work.
    if (!grid_world_->IsReturningHome() && global_path.nodes_.size()>1) {
      for (bool reverse:{false,true}) {
        const auto &nodes=global_path.nodes_;
        const auto node_at=[&](std::size_t i)->const exploration_path_ns::Node& {
          return nodes[reverse ? nodes.size()-1-i : i];
        };
        if ((node_at(0).position_-robot).head<2>().norm()>.8) continue;
        double arc=0;
        Eigen::Vector3d previous=robot;
        for (std::size_t i=0;i<nodes.size();++i) {
          const auto &node=node_at(i);
          if (node.type_==exploration_path_ns::NodeType::HOME) break;
          const double segment=(node.position_-previous).head<2>().norm();
          if (segment>1e-6 && arc+segment>=kLookAheadDistance) {
            const double ratio=(kLookAheadDistance-arc)/segment;
            global.push_back({previous+ratio*(node.position_-previous),0,kLookAheadDistance});
            break;
          }
          arc+=segment;previous=node.position_;
          if (node.type_==exploration_path_ns::NodeType::GLOBAL_VIEWPOINT &&
              (node.position_-robot).head<2>().norm()>=goal_selection_.parameters.min_distance) {
            global.push_back({node.position_,0,arc});break;
          }
        }
      }
    }
    local_count=local.size();global_count=global.size();
    choice=goal_selection_.update(robot,robot_yaw_,now().seconds(),local,global,valid);
  }
  if (!choice) return false;
  const auto route=viewpoint_manager_->GetViewPointShortestPath(robot,choice->position);
  if (route.poses.size()<2) {
    ResetGoalPolicies();
    RCLCPP_WARN(get_logger(),"Selected goal has no connected viewpoint route; reevaluating");
    return false;
  }
  exploration_path_.nodes_.clear();
  exploration_path_ns::Node node(robot);
  node.type_=exploration_path_ns::NodeType::ROBOT;
  exploration_path_.nodes_.push_back(node);
  for (const auto &pose:route.poses) {
    const auto &p=pose.pose.position;
    node.position_={p.x,p.y,p.z};
    if ((node.position_-exploration_path_.nodes_.back().position_).norm()<1e-5) continue;
    node.type_=exploration_path_ns::NodeType::LOCAL_VIA_POINT;
    exploration_path_.nodes_.push_back(node);
  }
  lookahead_point_=choice->position;
  node.position_=lookahead_point_;
  node.type_=choice->mode==rover_explore::GoalMode::RELOCATE ?
      exploration_path_ns::NodeType::GLOBAL_VIEWPOINT : exploration_path_ns::NodeType::LOCAL_VIEWPOINT;
  if ((node.position_-exploration_path_.nodes_.back().position_).norm()>1e-5)
    exploration_path_.nodes_.push_back(node);
  else exploration_path_.nodes_.back().type_=node.type_;
  lookahead_point_update_=true;
  lookahead_point_in_line_of_sight_=viewpoint_manager_->InCurrentFrameLineOfSight(lookahead_point_);
  relocation_=choice->mode==rover_explore::GoalMode::RELOCATE;
  const int selected_index=viewpoint_manager_->GetViewPointInd(lookahead_point_);
  if (viewpoint_manager_->InRange(selected_index)) viewpoint_manager_->SetViewPointSelected(selected_index,true);
  RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
      "Goal selection: mode=%s target=(%.2f, %.2f) distance=%.2f local_work=%zu global_options=%zu",
      rover_explore::goal_mode_name(choice->mode),lookahead_point_.x(),lookahead_point_.y(),
      (lookahead_point_-robot).head<2>().norm(),local_count,global_count);
  return true;
}

bool SensorCoveragePlanner3D::GetLookAheadPoint(
    const exploration_path_ns::ExplorationPath &local_path,
    const exploration_path_ns::ExplorationPath &global_path,
    Eigen::Vector3d &lookahead_point) {
  Eigen::Vector3d robot_position(robot_position_.x, robot_position_.y,
                                 robot_position_.z);

  // Determine which direction to follow on the global path
  double dist_from_start = 0.0;
  for (int i = 1; i < global_path.nodes_.size(); i++) {
    dist_from_start +=
        (global_path.nodes_[i - 1].position_ - global_path.nodes_[i].position_)
            .norm();
    if (global_path.nodes_[i].type_ ==
        exploration_path_ns::NodeType::GLOBAL_VIEWPOINT) {
      break;
    }
  }

  double dist_from_end = 0.0;
  for (int i = global_path.nodes_.size() - 2; i > 0; i--) {
    dist_from_end +=
        (global_path.nodes_[i + 1].position_ - global_path.nodes_[i].position_)
            .norm();
    if (global_path.nodes_[i].type_ ==
        exploration_path_ns::NodeType::GLOBAL_VIEWPOINT) {
      break;
    }
  }

  bool local_path_too_short = true;
  for (int i = 0; i < local_path.nodes_.size(); i++) {
    double dist_to_robot =
        (robot_position - local_path.nodes_[i].position_).norm();
    if (dist_to_robot > kLookAheadDistance / 5) {
      local_path_too_short = false;
      break;
    }
  }
  if (local_path.GetNodeNum() < 1 || local_path_too_short) {
    if (dist_from_start < dist_from_end) {
      double dist_from_robot = 0.0;
      for (int i = 1; i < global_path.nodes_.size(); i++) {
        dist_from_robot += (global_path.nodes_[i - 1].position_ -
                            global_path.nodes_[i].position_)
                               .norm();
        if (dist_from_robot > kLookAheadDistance / 2) {
          lookahead_point = global_path.nodes_[i].position_;
          break;
        }
      }
    } else {
      double dist_from_robot = 0.0;
      for (int i = global_path.nodes_.size() - 2; i > 0; i--) {
        dist_from_robot += (global_path.nodes_[i + 1].position_ -
                            global_path.nodes_[i].position_)
                               .norm();
        if (dist_from_robot > kLookAheadDistance / 2) {
          lookahead_point = global_path.nodes_[i].position_;
          break;
        }
      }
    }
    return false;
  }

  bool has_lookahead = false;
  bool dir = true;
  int robot_i = 0;
  int lookahead_i = 0;
  for (int i = 0; i < local_path.nodes_.size(); i++) {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::ROBOT) {
      robot_i = i;
    }
    if (local_path.nodes_[i].type_ ==
        exploration_path_ns::NodeType::LOOKAHEAD_POINT) {
      has_lookahead = true;
      lookahead_i = i;
    }
  }

  if (reset_waypoint_) {
    has_lookahead = false;
  }

  int forward_viewpoint_count = 0;
  int backward_viewpoint_count = 0;

  bool local_loop = false;
  if (local_path.nodes_.front() == local_path.nodes_.back() &&
      local_path.nodes_.front().type_ == exploration_path_ns::NodeType::ROBOT) {
    local_loop = true;
  }

  if (local_loop) {
    robot_i = 0;
  }
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++) {
    if (local_path.nodes_[i].type_ ==
        exploration_path_ns::NodeType::LOCAL_VIEWPOINT) {
      forward_viewpoint_count++;
    }
  }
  if (local_loop) {
    robot_i = local_path.nodes_.size() - 1;
  }
  for (int i = robot_i - 1; i >= 0; i--) {
    if (local_path.nodes_[i].type_ ==
        exploration_path_ns::NodeType::LOCAL_VIEWPOINT) {
      backward_viewpoint_count++;
    }
  }

  Eigen::Vector3d forward_lookahead_point = robot_position;
  Eigen::Vector3d backward_lookahead_point = robot_position;

  bool has_forward = false;
  bool has_backward = false;

  if (local_loop) {
    robot_i = 0;
  }
  bool forward_lookahead_point_in_los = true;
  bool backward_lookahead_point_in_los = true;
  double length_from_robot = 0.0;
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++) {
    length_from_robot +=
        (local_path.nodes_[i].position_ - local_path.nodes_[i - 1].position_)
            .norm();
    double dist_to_robot =
        (local_path.nodes_[i].position_ - robot_position).norm();
    bool in_line_of_sight = true;
    if (i < local_path.GetNodeNum() - 1) {
      in_line_of_sight = viewpoint_manager_->InCurrentFrameLineOfSight(
          local_path.nodes_[i + 1].position_);
    }
    if ((length_from_robot > kLookAheadDistance ||
         (kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ ==
             exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ ==
             exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ ==
             exploration_path_ns::NodeType::LOCAL_PATH_END ||
         i == local_path.GetNodeNum() - 1))

    {
      if (kUseLineOfSightLookAheadPoint && !in_line_of_sight) {
        forward_lookahead_point_in_los = false;
      }
      forward_lookahead_point = local_path.nodes_[i].position_;
      has_forward = true;
      break;
    }
  }
  if (local_loop) {
    robot_i = local_path.nodes_.size() - 1;
  }
  length_from_robot = 0.0;
  for (int i = robot_i - 1; i >= 0; i--) {
    length_from_robot +=
        (local_path.nodes_[i].position_ - local_path.nodes_[i + 1].position_)
            .norm();
    double dist_to_robot =
        (local_path.nodes_[i].position_ - robot_position).norm();
    bool in_line_of_sight = true;
    if (i > 0) {
      in_line_of_sight = viewpoint_manager_->InCurrentFrameLineOfSight(
          local_path.nodes_[i - 1].position_);
    }
    if ((length_from_robot > kLookAheadDistance ||
         (kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ ==
             exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ ==
             exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ ==
             exploration_path_ns::NodeType::LOCAL_PATH_END ||
         i == 0))

    {
      if (kUseLineOfSightLookAheadPoint && !in_line_of_sight) {
        backward_lookahead_point_in_los = false;
      }
      backward_lookahead_point = local_path.nodes_[i].position_;
      has_backward = true;
      break;
    }
  }

  if (forward_viewpoint_count > 0 && !has_forward) {
    std::cout << "forward viewpoint count > 0 but does not have forward "
                 "lookahead point"
              << std::endl;
    exit(1);
  }
  if (backward_viewpoint_count > 0 && !has_backward) {
    std::cout << "backward viewpoint count > 0 but does not have backward "
                 "lookahead point"
              << std::endl;
    exit(1);
  }

  double dx = lookahead_point_direction_.x();
  double dy = lookahead_point_direction_.y();

  if (reset_waypoint_) {
    reset_waypoint_ = false;
    double lx = 1.0;
    double ly = 0.0;

    dx = cos(robot_yaw_) * lx - sin(robot_yaw_) * ly;
    dy = sin(robot_yaw_) * lx + cos(robot_yaw_) * ly;
  }

  double forward_angle_score = -2;
  double backward_angle_score = -2;
  double lookahead_angle_score = -2;

  double dist_robot_to_lookahead = 0.0;
  if (has_forward) {
    Eigen::Vector3d forward_diff = forward_lookahead_point - robot_position;
    forward_diff.z() = 0.0;
    forward_diff = forward_diff.normalized();
    forward_angle_score = dx * forward_diff.x() + dy * forward_diff.y();
  }
  if (has_backward) {
    Eigen::Vector3d backward_diff = backward_lookahead_point - robot_position;
    backward_diff.z() = 0.0;
    backward_diff = backward_diff.normalized();
    backward_angle_score = dx * backward_diff.x() + dy * backward_diff.y();
  }
  if (has_lookahead) {
    Eigen::Vector3d prev_lookahead_point =
        local_path.nodes_[lookahead_i].position_;
    dist_robot_to_lookahead = (robot_position - prev_lookahead_point).norm();
    Eigen::Vector3d diff = prev_lookahead_point - robot_position;
    diff.z() = 0.0;
    diff = diff.normalized();
    lookahead_angle_score = dx * diff.x() + dy * diff.y();
  }

  lookahead_point_cloud_->cloud_->clear();

  if (forward_viewpoint_count == 0 && backward_viewpoint_count == 0) {
    relocation_ = true;
  } else {
    relocation_ = false;
  }
  if (relocation_) {
    if (use_momentum_ && kUseMomentum) {
      if (forward_angle_score > backward_angle_score) {
        lookahead_point = forward_lookahead_point;
      } else {
        lookahead_point = backward_lookahead_point;
      }
    } else {
      // follow the shorter distance one
      if (dist_from_start < dist_from_end &&
          local_path.nodes_.front().type_ !=
              exploration_path_ns::NodeType::ROBOT) {
        lookahead_point = backward_lookahead_point;
      } else if (dist_from_end < dist_from_start &&
                 local_path.nodes_.back().type_ !=
                     exploration_path_ns::NodeType::ROBOT) {
        lookahead_point = forward_lookahead_point;
      } else {
        lookahead_point = forward_angle_score > backward_angle_score
                              ? forward_lookahead_point
                              : backward_lookahead_point;
      }
    }
  } else if (has_lookahead && lookahead_angle_score > 0 &&
             dist_robot_to_lookahead > kLookAheadDistance / 2 &&
             viewpoint_manager_->InLocalPlanningHorizon(
                 local_path.nodes_[lookahead_i].position_))

  {
    lookahead_point = local_path.nodes_[lookahead_i].position_;
  } else {
    if (forward_angle_score > backward_angle_score) {
      if (forward_viewpoint_count > 0) {
        lookahead_point = forward_lookahead_point;
      } else {
        lookahead_point = backward_lookahead_point;
      }
    } else {
      if (backward_viewpoint_count > 0) {
        lookahead_point = backward_lookahead_point;
      } else {
        lookahead_point = forward_lookahead_point;
      }
    }
  }

  if ((lookahead_point == forward_lookahead_point &&
       !forward_lookahead_point_in_los) ||
      (lookahead_point == backward_lookahead_point &&
       !backward_lookahead_point_in_los)) {
    lookahead_point_in_line_of_sight_ = false;
  } else {
    lookahead_point_in_line_of_sight_ = true;
  }

  lookahead_point_direction_ = lookahead_point - robot_position;
  lookahead_point_direction_.z() = 0.0;
  if (lookahead_point_direction_.norm() > 1e-6) lookahead_point_direction_.normalize();

  pcl::PointXYZI point;
  point.x = lookahead_point.x();
  point.y = lookahead_point.y();
  point.z = lookahead_point.z();
  point.intensity = 1.0;
  lookahead_point_cloud_->cloud_->points.push_back(point);

  if (has_lookahead) {
    point.x = local_path.nodes_[lookahead_i].position_.x();
    point.y = local_path.nodes_[lookahead_i].position_.y();
    point.z = local_path.nodes_[lookahead_i].position_.z();
    point.intensity = 0;
    lookahead_point_cloud_->cloud_->points.push_back(point);
  }
  return true;
}

void SensorCoveragePlanner3D::PublishWaypoint() {
  geometry_msgs::msg::PointStamped waypoint;
  if (exploration_finished_ && near_home_ && kRushHome) {
    waypoint.point.x = initial_position_.x();
    waypoint.point.y = initial_position_.y();
    waypoint.point.z = initial_position_.z();
  } else {
    double dx = lookahead_point_.x() - robot_position_.x;
    double dy = lookahead_point_.y() - robot_position_.y;
    double r = sqrt(dx * dx + dy * dy);
    double extend_dist = lookahead_point_in_line_of_sight_
                             ? kExtendWayPointDistanceBig
                             : kExtendWayPointDistanceSmall;
    if (r > 1e-6 && r < extend_dist && kExtendWayPoint) {
      dx = dx / r * extend_dist;
      dy = dy / r * extend_dist;
    }
    waypoint.point.x = dx + robot_position_.x;
    waypoint.point.y = dy + robot_position_.y;
    waypoint.point.z = lookahead_point_.z();
  }
  misc_utils_ns::Publish(shared_from_this(), waypoint_pub_, waypoint,
                         kWorldFrameID);
}

void SensorCoveragePlanner3D::PublishStatus(const std::string &status) {
  if (!status_pub_) return;
  std_msgs::msg::String message;
  message.data = status;
  status_pub_->publish(message);
}

void SensorCoveragePlanner3D::PublishControlPath(
    const exploration_path_ns::ExplorationPath &global_path) {
  nav_msgs::msg::Path path;
  path.header.frame_id = kWorldFrameID;
  path.header.stamp = now();
  const Eigen::Vector3d robot(robot_position_.x, robot_position_.y, robot_position_.z);
  const exploration_path_ns::ExplorationPath *source = nullptr;
  int first = -1, last = -1;
  double best_length = std::numeric_limits<double>::infinity();
  const exploration_path_ns::ExplorationPath *candidates[] = {&exploration_path_, &global_path};
  for (const auto *candidate : candidates) {
    for (int i = 0; i < static_cast<int>(candidate->nodes_.size()); ++i) {
      if ((candidate->nodes_[i].position_ - robot).norm() > 0.7) continue;
      for (int j = 0; j < static_cast<int>(candidate->nodes_.size()); ++j) {
        if (i == j || (candidate->nodes_[j].position_ - lookahead_point_).norm() > 0.3) continue;
        double length = 0.0;
        for (int k = std::min(i,j); k < std::max(i,j); ++k)
          length += (candidate->nodes_[k+1].position_ - candidate->nodes_[k].position_).norm();
        if (length > 0.2 && length < best_length) {
          source = candidate; first = i; last = j; best_length = length;
        }
      }
    }
  }
  if (!source) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No control path: local_nodes=%zu global_nodes=%zu target_distance=%.2f",
        exploration_path_.nodes_.size(), global_path.nodes_.size(),
        (lookahead_point_ - robot).norm());
    PublishStatus("NO_CONTROL_PATH");
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.header;
  pose.pose.position = robot_position_;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);
  const int step = last > first ? 1 : -1;
  for (int i = first; ; i += step) {
    const auto &p = source->nodes_[i].position_;
    pose.pose.position.x = p.x(); pose.pose.position.y = p.y(); pose.pose.position.z = p.z();
    if (p.allFinite()) path.poses.push_back(pose);
    if (i == last) break;
  }
  control_path_pub_->publish(path);
}

void SensorCoveragePlanner3D::PublishRuntime() {
  local_viewpoint_sampling_runtime_ =
      local_coverage_planner_->GetViewPointSamplingRuntime() / 1000;
  local_path_finding_runtime_ = (local_coverage_planner_->GetFindPathRuntime() +
                                 local_coverage_planner_->GetTSPRuntime()) /
                                1000;

  std_msgs::msg::Int32MultiArray runtime_breakdown_msg;
  runtime_breakdown_msg.data.clear();
  runtime_breakdown_msg.data.push_back(update_representation_runtime_);
  runtime_breakdown_msg.data.push_back(local_viewpoint_sampling_runtime_);
  runtime_breakdown_msg.data.push_back(local_path_finding_runtime_);
  runtime_breakdown_msg.data.push_back(global_planning_runtime_);
  runtime_breakdown_msg.data.push_back(trajectory_optimization_runtime_);
  runtime_breakdown_msg.data.push_back(overall_runtime_);
  runtime_breakdown_pub_->publish(runtime_breakdown_msg);

  float runtime = 0;
  if (!exploration_finished_ && kNoExplorationReturnHome) {
    for (int i = 0; i < runtime_breakdown_msg.data.size() - 1; i++) {
      runtime += runtime_breakdown_msg.data[i];
    }
  }

  std_msgs::msg::Float32 runtime_msg;
  runtime_msg.data = runtime / 1000.0;
  runtime_pub_->publish(runtime_msg);
}

double SensorCoveragePlanner3D::GetRobotToHomeDistance() {
  Eigen::Vector3d robot_position(robot_position_.x, robot_position_.y,
                                 robot_position_.z);
  return (robot_position - initial_position_).norm();
}

void SensorCoveragePlanner3D::PublishExplorationState() {
  std_msgs::msg::Bool exploration_finished_msg;
  exploration_finished_msg.data = exploration_finished_;
  exploration_finish_pub_->publish(exploration_finished_msg);
}

void SensorCoveragePlanner3D::PrintExplorationStatus(std::string status,
                                                     bool clear_last_line) {
  if (clear_last_line) {
    printf(cursup);
    printf(cursclean);
    printf(cursup);
    printf(cursclean);
  }
  std::cout << std::endl << "\033[1;32m" << status << "\033[0m" << std::endl;
}

void SensorCoveragePlanner3D::CountDirectionChange() {
  Eigen::Vector3d current_moving_direction_ =
      Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z) -
      Eigen::Vector3d(last_robot_position_.x, last_robot_position_.y,
                      last_robot_position_.z);

  if (current_moving_direction_.norm() > 0.5) {
    if (moving_direction_.dot(current_moving_direction_) < 0) {
      direction_change_count_++;
      direction_no_change_count_ = 0;
      if (direction_change_count_ > kDirectionChangeCounterThr) {
        if (!use_momentum_) {
          momentum_activation_count_++;
        }
        use_momentum_ = true;
      }
    } else {
      direction_no_change_count_++;
      if (direction_no_change_count_ > kDirectionNoChangeCounterThr) {
        direction_change_count_ = 0;
        use_momentum_ = false;
      }
    }
    moving_direction_ = current_moving_direction_;
  }
  last_robot_position_ = robot_position_;

  std_msgs::msg::Int32 momentum_activation_count_msg;
  momentum_activation_count_msg.data = momentum_activation_count_;
  momentum_activation_count_pub_->publish(momentum_activation_count_msg);
}

bool SensorCoveragePlanner3D::PrepareNavigationCollisionMap() {
  navigation_collision_map_.invalidate();
  if (!navigation_global_map_ || !navigation_local_map_) return false;
  const auto transform = [this](const std::string &frame) {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    if (frame == kWorldFrameID) return result;
    const auto stamped = navigation_tf_buffer_->lookupTransform(frame,kWorldFrameID,tf2::TimePointZero);
    const rclcpp::Time stamp(stamped.header.stamp, get_clock()->get_clock_type());
    const double age=(now()-stamp).seconds();
    // TF time zero is used for static transforms.
    if (stamp.nanoseconds()!=0 && (age>navigation_grid_timeout_ || age < -sync_tolerance_))
      throw std::runtime_error("navigation map TF is stale");
    const auto &q=stamped.transform.rotation;
    Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    if (!rotation.coeffs().allFinite() || std::abs(rotation.norm()-1)>1e-3)
      throw std::runtime_error("navigation map TF rotation is invalid");
    result.linear()=rotation.normalized().toRotationMatrix();
    const auto &p=stamped.transform.translation;
    result.translation()=Eigen::Vector3d(p.x,p.y,p.z);
    return result;
  };
  try {
    const auto global_transform=transform(navigation_global_map_->header.frame_id);
    const auto local_transform=transform(navigation_local_map_->header.frame_id);
    return navigation_collision_map_.prepare(navigation_global_map_,navigation_local_map_,
        global_transform,local_transform,now().seconds(),navigation_grid_timeout_,sync_tolerance_);
  } catch (const std::exception &error) {
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Navigation collision map unavailable: %s",error.what());
    return false;
  }
}

void SensorCoveragePlanner3D::execute() {
  if (!kAutoStart && !start_exploration_) {
    coverage_confirmation_.reset();
    PublishStatus("DISABLED");
    return;
  }
  const auto t = now();
  if (!odometry_received_ || !cloud_received_ || !terrain_received_ ||
      (t - last_odom_time_).seconds() > input_timeout_ ||
      (t - last_cloud_time_).seconds() > input_timeout_ ||
      (t - last_terrain_time_).seconds() > input_timeout_ ||
      (t - last_odom_time_).seconds() < -sync_tolerance_ ||
      (t - last_cloud_time_).seconds() < -sync_tolerance_ ||
      (t - last_terrain_time_).seconds() < -sync_tolerance_) {
    coverage_confirmation_.reset();
    goal_selection_.reset();corridor_.release_target();
    PublishStatus("WAITING_FOR_INPUT");
    return;
  }
  if (navigation_grid_enabled_ && !PrepareNavigationCollisionMap()) {
    coverage_confirmation_.reset();
    goal_selection_.reset();corridor_.release_target();
    PublishStatus("WAITING_FOR_NAVIGATION_MAP");
    return;
  }
  Timer overall_processing_timer("overall processing");
  update_representation_runtime_ = 0;
  local_viewpoint_sampling_runtime_ = 0;
  local_path_finding_runtime_ = 0;
  global_planning_runtime_ = 0;
  trajectory_optimization_runtime_ = 0;
  overall_runtime_ = 0;

  if (!initialized_) {
    // Wait for observed geometry; never command the original blind 12 m move.
    if (!home_initialized_) {
      initial_position_ = Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z);
      home_initialized_ = true;
    }
    start_time_ = this->now().seconds();
    if(start_time_ == 0.0){
      RCLCPP_ERROR(this->get_logger(), "Start time is zero, time source (use_time_time) not set correctly. Exiting...");
      exit(1);
    }
    global_direction_switch_time_ = this->now().seconds();
    initialized_ = true;
    return;
  }

  overall_processing_timer.Start();
  if (keypose_cloud_update_) {
    keypose_cloud_update_ = false;
    const bool previously_finished = exploration_finished_;
    // Always reevaluate new geometry, including after confirmed completion.
    exploration_finished_ = false;

    CountDirectionChange();

    misc_utils_ns::Timer update_representation_timer("update representation");
    update_representation_timer.Start();

    // Update grid world
    UpdateGlobalRepresentation();

    int viewpoint_candidate_count = UpdateViewPoints();
    if(corridor_.parameters.enabled) {
      exploration_path_ns::ExplorationPath unused;
      const bool ready=SelectStableGoal(unused);
      exploration_finished_=false;coverage_confirmation_.reset();
      PublishExplorationState();
      PublishStatus(corridor_.exited()?"EXITED":(corridor_.exiting()?"EXIT_FORWARD":(ready?"RUNNING":"WAITING_FOR_CENTER_PATH")));
      if(ready)PublishWaypoint();
      overall_processing_timer.Stop(false);
      overall_runtime_=overall_processing_timer.GetDuration("ms");PublishRuntime();
      return;
    }
    if (viewpoint_candidate_count == 0) {
      coverage_confirmation_.reset();
      PublishExplorationState();
      PublishStatus("NO_CANDIDATE_VIEWPOINT");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No candidate viewpoints");
      return;
    }

    UpdateKeyposeGraph();

    int uncovered_point_num = 0;
    int uncovered_frontier_point_num = 0;
    UpdateViewPointCoverage();
    UpdateCoveredAreas(uncovered_point_num, uncovered_frontier_point_num);

    update_representation_timer.Stop(false);
    update_representation_runtime_ +=
        update_representation_timer.GetDuration("ms");

    // Global TSP
    std::vector<int> global_cell_tsp_order;
    exploration_path_ns::ExplorationPath global_path;
    GlobalPlanning(global_cell_tsp_order, global_path);

    // Local TSP
    exploration_path_ns::ExplorationPath local_path;
    LocalPlanning(uncovered_point_num, uncovered_frontier_point_num,
                  global_path, local_path);

    near_home_ = GetRobotToHomeDistance() < kRushHomeDist;
    at_home_ = GetRobotToHomeDistance() < kAtHomeDistThreshold;

    const bool original_no_tasks = grid_world_->IsReturningHome() &&
        local_coverage_planner_->IsLocalCoverageComplete();
    bool stable_goal = false;
    if (corridor_.parameters.enabled) {
      stable_goal = SelectStableGoal(global_path);
    } else if (goal_selection_.parameters.enabled) {
      if (original_no_tasks && uncovered_point_num == 0 && uncovered_frontier_point_num == 0 &&
          planning_env_->GetFilteredFrontierCount() == 0) ResetGoalPolicies();
      else stable_goal = SelectStableGoal(global_path);
    }
    const bool no_tasks = original_no_tasks && !stable_goal;
    // "No selectable goal" can also mean occlusion, disconnected candidates or
    // insufficient gain. Remaining surfaces/frontiers are not proof of completion.
    const bool no_work = no_tasks && uncovered_point_num == 0 &&
        uncovered_frontier_point_num == 0 && planning_env_->GetFilteredFrontierCount() == 0;
    exploration_finished_ = coverage_confirmation_.update(no_work, now().seconds());
    if (previously_finished && !exploration_finished_)
      RCLCPP_INFO(get_logger(), "New observation invalidated coverage completion; replanning");
    if (exploration_finished_ && !previously_finished)
      RCLCPP_INFO(get_logger(), "Coverage complete after stable fresh observations; continuing to monitor");

    PublishExplorationState();
    // Publish current debug geometry even when no route is available.
    visualizer_->GetGlobalSubspaceMarker(grid_world_, global_cell_tsp_order);
    Eigen::Vector3d viewpoint_origin = viewpoint_manager_->GetOrigin();
    visualizer_->GetLocalPlanningHorizonMarker(
        viewpoint_origin.x(), viewpoint_origin.y(), robot_position_.z);
    visualizer_->PublishMarkers();
    PublishLocalPlanningVisualization(local_path);
    PublishGlobalPlanningVisualization(global_path, local_path);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "Exploration: navigation_candidates=%d forward_candidates=%zu surface_gain=%d frontier_gain=%d filtered_frontiers=%zu no_tasks=%d complete=%d",
        viewpoint_candidate_count, viewpoint_manager_->GetExplorationViewPointCandidateIndices().size(),
        uncovered_point_num, uncovered_frontier_point_num,
        planning_env_->GetFilteredFrontierCount(), no_tasks, exploration_finished_);
    if (corridor_.parameters.enabled && corridor_.exited()) {
      PublishStatus("EXITED");
      return;
    }
    if (exploration_finished_ || (no_tasks && kNoExplorationReturnHome)) {
      PublishStatus(exploration_finished_ ? "COVERAGE_COMPLETE" :
                    (no_work ? "COVERAGE_PENDING" : "WAITING_FOR_TARGET"));
      return;
    }
    if ((corridor_.parameters.enabled || goal_selection_.parameters.enabled) && !stable_goal) {
      PublishStatus("WAITING_FOR_TARGET");
      return;
    }
    PublishStatus("RUNNING");
    if (!corridor_.parameters.enabled && !goal_selection_.parameters.enabled) {
      exploration_path_ = ConcatenateGlobalLocalPath(global_path, local_path);
      lookahead_point_ = Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z);
      lookahead_point_update_ = GetLookAheadPoint(exploration_path_, global_path, lookahead_point_);
    }
    PublishWaypoint();
    PublishControlPath(global_path);

    overall_processing_timer.Stop(false);
    overall_runtime_ = overall_processing_timer.GetDuration("ms");

    PublishRuntime();
  }
}
} // namespace sensor_coverage_planner_3d_ns
