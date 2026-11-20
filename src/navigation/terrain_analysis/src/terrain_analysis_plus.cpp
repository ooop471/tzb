#include <math.h>
#include <algorithm>
#include <chrono>
#include <unordered_map>
#include <stdexcept>
#include "terrain_realtime.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

// 体素键结构体 - 用于哈希表索引
struct VoxelKey {
    int x;
    int y;
    
    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y;
    }
};

// 体素键哈希函数
struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
        return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1);
    }
};

class TerrainAnalysisNode : public rclcpp::Node {
public:
  TerrainAnalysisNode() : Node("terrain_analysis") {
    // 声明并获取参数
    this->declare_parameter<double>("scanVoxelSize", scanVoxelSize);
    this->declare_parameter<double>("decayTime", decayTime);
    this->declare_parameter<double>("noDecayDis", noDecayDis);
    this->declare_parameter<double>("clearingDis", clearingDis);
    this->declare_parameter<bool>("useSorting", useSorting);
    this->declare_parameter<double>("quantileZ", quantileZ);
    this->declare_parameter<bool>("considerDrop", considerDrop);
    this->declare_parameter<bool>("limitGroundLift", limitGroundLift);
    this->declare_parameter<double>("maxGroundLift", maxGroundLift);
    this->declare_parameter<bool>("clearDyObs", clearDyObs);
    this->declare_parameter<double>("minDyObsDis", minDyObsDis);
    this->declare_parameter<double>("minDyObsAngle", minDyObsAngle);
    this->declare_parameter<double>("minDyObsRelZ", minDyObsRelZ);
    this->declare_parameter<double>("absDyObsRelZThre", absDyObsRelZThre);
    this->declare_parameter<double>("minDyObsVFOV", minDyObsVFOV);
    this->declare_parameter<double>("maxDyObsVFOV", maxDyObsVFOV);
    this->declare_parameter<int>("minDyObsPointNum", minDyObsPointNum);
    this->declare_parameter<bool>("noDataObstacle", noDataObstacle);
    this->declare_parameter<int>("noDataBlockSkipNum", noDataBlockSkipNum);
    this->declare_parameter<int>("minBlockPointNum", minBlockPointNum);
    this->declare_parameter<double>("vehicleHeight", vehicleHeight);
    this->declare_parameter<double>("minObstacleHeight", minObstacleHeight);
    this->declare_parameter<int>("voxelPointUpdateThre", voxelPointUpdateThre);
    this->declare_parameter<double>("voxelTimeUpdateThre", voxelTimeUpdateThre);
    this->declare_parameter<double>("minRelZ", minRelZ);
    this->declare_parameter<double>("maxRelZ", maxRelZ);
    this->declare_parameter<double>("disRatioZ", disRatioZ);
    // 新增参数 - 清理距离较远的体素的阈值
    this->declare_parameter<double>("maxVoxelStorageDist", maxVoxelStorageDist);
    this->declare_parameter<bool>("selfFilterEnabled", selfFilterEnabled);
    this->declare_parameter<double>("selfFilterRadius", selfFilterRadius);
    this->declare_parameter<double>("selfFilterMinX", selfFilterMinX);
    this->declare_parameter<double>("selfFilterMaxX", selfFilterMaxX);
    this->declare_parameter<double>("selfFilterMinY", selfFilterMinY);
    this->declare_parameter<double>("selfFilterMaxY", selfFilterMaxY);
    this->declare_parameter<double>("selfFilterMinZ", selfFilterMinZ);
    this->declare_parameter<double>("selfFilterMaxZ", selfFilterMaxZ);

    this->get_parameter("scanVoxelSize", scanVoxelSize);
    this->get_parameter("decayTime", decayTime);
    this->get_parameter("noDecayDis", noDecayDis);
    this->get_parameter("clearingDis", clearingDis);
    this->get_parameter("useSorting", useSorting);
    this->get_parameter("quantileZ", quantileZ);
    this->get_parameter("considerDrop", considerDrop);
    this->get_parameter("limitGroundLift", limitGroundLift);
    this->get_parameter("maxGroundLift", maxGroundLift);
    this->get_parameter("clearDyObs", clearDyObs);
    this->get_parameter("minDyObsDis", minDyObsDis);
    this->get_parameter("minDyObsAngle", minDyObsAngle);
    this->get_parameter("minDyObsRelZ", minDyObsRelZ);
    this->get_parameter("absDyObsRelZThre", absDyObsRelZThre);
    this->get_parameter("minDyObsVFOV", minDyObsVFOV);
    this->get_parameter("maxDyObsVFOV", maxDyObsVFOV);
    this->get_parameter("minDyObsPointNum", minDyObsPointNum);
    this->get_parameter("noDataObstacle", noDataObstacle);
    this->get_parameter("noDataBlockSkipNum", noDataBlockSkipNum);
    this->get_parameter("minBlockPointNum", minBlockPointNum);
    this->get_parameter("vehicleHeight", vehicleHeight);
    this->get_parameter("minObstacleHeight", minObstacleHeight);
    this->get_parameter("voxelPointUpdateThre", voxelPointUpdateThre);
    this->get_parameter("voxelTimeUpdateThre", voxelTimeUpdateThre);
    this->get_parameter("minRelZ", minRelZ);
    this->get_parameter("maxRelZ", maxRelZ);
    this->get_parameter("disRatioZ", disRatioZ);
    this->get_parameter("maxVoxelStorageDist", maxVoxelStorageDist);
    this->get_parameter("selfFilterEnabled", selfFilterEnabled);
    this->get_parameter("selfFilterRadius", selfFilterRadius);
    this->get_parameter("selfFilterMinX", selfFilterMinX);
    this->get_parameter("selfFilterMaxX", selfFilterMaxX);
    this->get_parameter("selfFilterMinY", selfFilterMinY);
    this->get_parameter("selfFilterMaxY", selfFilterMaxY);
    this->get_parameter("selfFilterMinZ", selfFilterMinZ);
    this->get_parameter("selfFilterMaxZ", selfFilterMaxZ);

    if (!std::isfinite(minObstacleHeight) || minObstacleHeight < 0.0 ||
        !std::isfinite(vehicleHeight) || minObstacleHeight >= vehicleHeight) {
      throw std::invalid_argument("minObstacleHeight must be finite, nonnegative and below vehicleHeight");
    }
    if(!std::isfinite(decayTime)||decayTime<=0)throw std::invalid_argument("decayTime must be finite and positive");
    RCLCPP_INFO(get_logger(), "Terrain obstacle height threshold: %.3f m above local ground", minObstacleHeight);

    // Process the newest available observation after each callback. A deep
    // queue turns computational load into old geometry, even while data flows.
    // Keep reliable compatibility with the existing Fast-LIO publisher.
    // 初始化订阅者
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::QoS(1), std::bind(&TerrainAnalysisNode::odometryHandler, this, std::placeholders::_1));

    laser_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_registered", rclcpp::QoS(1), std::bind(&TerrainAnalysisNode::laserCloudHandler, this, std::placeholders::_1));

    clearing_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "map_clearing", 5, std::bind(&TerrainAnalysisNode::clearingHandler, this, std::placeholders::_1));

    // 初始化发布者
    terrain_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map", 2);
    
    // 添加定时清理旧体素的定时器
    cleanup_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&TerrainAnalysisNode::cleanupOldVoxels, this));

    // 初始化点云对象
    laserCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudCrop.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudDwz.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudElev.reset(new pcl::PointCloud<pcl::PointXYZI>());
    obstacleCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    accumulatedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    // 设置下采样滤波器
    downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

    RCLCPP_INFO(this->get_logger(), "TerrainAnalysisNode 初始化完成 - 动态体素版本");
  }

private:
  // 参数
  double scanVoxelSize = 0.05;
  double decayTime = 2.0;
  double noDecayDis = 0.0; // Legacy no-data initialization only; never bypasses point expiry.
  double clearingDis = 8.0;
  bool clearingCloud = false;
  bool useSorting = true;
  double quantileZ = 0.25;
  bool considerDrop = false;
  bool limitGroundLift = false;
  double maxGroundLift = 0.15;
  bool clearDyObs = false;
  double minDyObsDis = 0.3;
  double minDyObsAngle = 0;
  double minDyObsRelZ = -0.5;
  double absDyObsRelZThre = 0.2;
  double minDyObsVFOV = -16.0;
  double maxDyObsVFOV = 16.0;
  int minDyObsPointNum = 1;
  bool noDataObstacle = false;
  int noDataBlockSkipNum = 0;
  int minBlockPointNum = 10;
  double vehicleHeight = 1.5;
  double minObstacleHeight = 0.10;
  int voxelPointUpdateThre = 100;
  double voxelTimeUpdateThre = 2.0;
  double minRelZ = -1.5;
  double maxRelZ = 0.2;
  double disRatioZ = 0.2;
  double maxVoxelStorageDist = 10.0; // 最大存储距离，超过此距离的体素会被清理

  // Narrow box covering the measured fixed robot return in base_frame.
  bool selfFilterEnabled = true;
  double selfFilterRadius = 0.40;
  double selfFilterMinX = 0.20;
  double selfFilterMaxX = 0.36;
  double selfFilterMinY = -0.12;
  double selfFilterMaxY = 0.08;
  double selfFilterMinZ = 0.05;
  double selfFilterMaxZ = 0.60;

  // 用于累积点云的变量
  // The terrain voxel map already retains observations for decayTime. Do not
  // accumulate derived obstacle snapshots: they lose source-observation ages.
  terrain_realtime::ObstacleUnion<pcl::PointXYZI> currentUnion{0.05};
  using SteadyClock = std::chrono::steady_clock;
  double voxelMs=0., groundMs=0., classifyMs=0., mergeMs=0., serializeMs=0., publishMs=0.;
  static double milliseconds(SteadyClock::time_point begin) {
    return std::chrono::duration<double,std::milli>(SteadyClock::now()-begin).count();
  }
  double lastPublishTime = 0.0;

  // 体素参数
  float terrainVoxelSize = 0.5;
  float planarVoxelSize = 0.1;

  // 点云对象
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev;
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulatedCloud;
  
  // 使用哈希映射替代固定大小的数组
  std::unordered_map<VoxelKey, pcl::PointCloud<pcl::PointXYZI>::Ptr, VoxelKeyHash> terrainVoxelCloudMap;
  std::unordered_map<VoxelKey, int, VoxelKeyHash> terrainVoxelUpdateNumMap;
  std::unordered_map<VoxelKey, float, VoxelKeyHash> terrainVoxelUpdateTimeMap;
  std::unordered_map<VoxelKey, float, VoxelKeyHash> planarVoxelElevMap;
  std::unordered_map<VoxelKey, int, VoxelKeyHash> planarVoxelEdgeMap;
  std::unordered_map<VoxelKey, int, VoxelKeyHash> planarVoxelDyObsMap;
  std::unordered_map<VoxelKey, std::vector<float>, VoxelKeyHash> planarPointElevMap;

  // 时间和状态变量
  double laserCloudTime = 0;
  bool newlaserCloud = false;
  double systemInitTime = 0;
  bool systemInited = false;
  int noDataInited = 0;

  // 位姿相关变量
  float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
  float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
  float vehicleXRec = 0, vehicleYRec = 0;
  float sinVehicleRoll = 0, cosVehicleRoll = 0;
  float sinVehiclePitch = 0, cosVehiclePitch = 0;
  float sinVehicleYaw = 0, cosVehicleYaw = 0;

  // 滤波器
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

  // 订阅者和发布者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laser_cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr clearing_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_cloud_pub_;
  rclcpp::TimerBase::SharedPtr cleanup_timer_;

  bool isInsideSelfFilterBox(const pcl::PointXYZI& point) const {
    if (!selfFilterEnabled) {
      return false;
    }

    const float dx = point.x - vehicleX;
    const float dy = point.y - vehicleY;
    const float localX = cosVehicleYaw * dx + sinVehicleYaw * dy;
    const float localY = -sinVehicleYaw * dx + cosVehicleYaw * dy;
    const float localZ = point.z - vehicleZ;

    // Brute-force safety region: anything this close to the vehicle center is
    // inside the robot envelope and must never become a terrain obstacle.
    if (localX * localX + localY * localY <=
        selfFilterRadius * selfFilterRadius) {
      return true;
    }

    return localX >= selfFilterMinX && localX <= selfFilterMaxX &&
           localY >= selfFilterMinY && localY <= selfFilterMaxY &&
           localZ >= selfFilterMinZ && localZ <= selfFilterMaxZ;
  }

  // 回调函数：处理里程计数据
  void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
        .getRPY(roll, pitch, yaw);

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;

    sinVehicleRoll = sin(vehicleRoll);
    cosVehicleRoll = cos(vehicleRoll);
    sinVehiclePitch = sin(vehiclePitch);
    cosVehiclePitch = cos(vehiclePitch);
    sinVehicleYaw = sin(vehicleYaw);
    cosVehicleYaw = cos(vehicleYaw);

    if (noDataInited == 0) {
      vehicleXRec = vehicleX;
      vehicleYRec = vehicleY;
      noDataInited = 1;
    }
    if (noDataInited == 1) {
      float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                      (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
      if (dis >= noDecayDis)
        noDataInited = 2;
    }
  }

  // 回调函数：处理激光点云数据
  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2) {
    const auto start_time = SteadyClock::now();
    const double sourceAge = now().seconds()-rclcpp::Time(laserCloud2->header.stamp).seconds();
    
    laserCloudTime = rclcpp::Time(laserCloud2->header.stamp).seconds();
    if (!systemInited) {
      systemInitTime = laserCloudTime;
      systemInited = true;
    }

    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      if (isInsideSelfFilterBox(point)) {
        continue;
      }

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                      (pointY - vehicleY) * (pointY - vehicleY));
      if (pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
          pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
          dis < maxVoxelStorageDist) {  // 将距离限制修改为更大的值
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        point.intensity = laserCloudTime - systemInitTime;
        laserCloudCrop->push_back(point);
      }
    }

    // 处理点云数据
    const double cropMs=milliseconds(start_time);
    processPointCloud();
    const double outputAge=now().seconds()-laserCloudTime;
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
      "[TERRAIN_TIMING] source_age=%.3fs output_age=%.3fs total=%.1fms crop=%.1f voxel=%.1f ground=%.1f classify=%.1f merge=%.1f serialize=%.1f publish=%.1f input=%zu terrain=%zu obstacle=%zu output=%zu",
      sourceAge,outputAge,milliseconds(start_time),cropMs,voxelMs,groundMs,classifyMs,mergeMs,serializeMs,publishMs,
      laserCloud->size(),terrainCloud->size(),obstacleCloud->size(),accumulatedCloud->size());
    if(outputAge>1.5)RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
      "[TERRAIN_LATENCY] output_age=%.3fs approaching/exceeding follower limit 2.0s; source stamp preserved",outputAge);
  }

  // 回调函数：处理地图清除数据
  void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis) {
    noDataInited = 0;
    clearingDis = dis->data;
    clearingCloud = true;
  }
  
  // 获取体素键
  VoxelKey getTerrainVoxelKey(float x, float y) {
    // Persistent bins must stay fixed in world coordinates as the rover moves.
    int indX = static_cast<int>(std::floor(x / terrainVoxelSize));
    int indY = static_cast<int>(std::floor(y / terrainVoxelSize));
    return {indX, indY};
  }
  
  VoxelKey getPlanarVoxelKey(float x, float y) {
    int indX = static_cast<int>((x - vehicleX) / planarVoxelSize);
    int indY = static_cast<int>((y - vehicleY) / planarVoxelSize);
    return {indX, indY};
  }
  
  // 清理旧体素数据
  void cleanupOldVoxels() {
    int removedCount = 0;
    
    // 清理terrain体素
    for (auto it = terrainVoxelCloudMap.begin(); it != terrainVoxelCloudMap.end();) {
      VoxelKey key = it->first;
      float voxelCenterX = (key.x + .5f) * terrainVoxelSize;
      float voxelCenterY = (key.y + .5f) * terrainVoxelSize;
      
      float distance = sqrt(pow(voxelCenterX - vehicleX, 2) + pow(voxelCenterY - vehicleY, 2));
      
      if (distance > maxVoxelStorageDist) {
        it = terrainVoxelCloudMap.erase(it);
        terrainVoxelUpdateNumMap.erase(key);
        terrainVoxelUpdateTimeMap.erase(key);
        removedCount++;
      } else {
        ++it;
      }
    }
    
    // 清理planar体素
    for (auto it = planarVoxelElevMap.begin(); it != planarVoxelElevMap.end();) {
      VoxelKey key = it->first;
      float voxelCenterX = (key.x * planarVoxelSize) + vehicleX;
      float voxelCenterY = (key.y * planarVoxelSize) + vehicleY;
      
      float distance = sqrt(pow(voxelCenterX - vehicleX, 2) + pow(voxelCenterY - vehicleY, 2));
      
      if (distance > maxVoxelStorageDist) {
        it = planarVoxelElevMap.erase(it);
        planarVoxelEdgeMap.erase(key);
        planarVoxelDyObsMap.erase(key);
        planarPointElevMap.erase(key);
        removedCount++;
      } else {
        ++it;
      }
    }
    
    if (removedCount > 0) {
      RCLCPP_INFO(this->get_logger(), "已清理 %d 个远离机器人的体素", removedCount);
    }
  }

  // 处理点云数据的主函数
  void processPointCloud() {
    auto phase = SteadyClock::now();
    // 创建或获取体素点云
    auto getTerrainVoxelCloud = [this](const VoxelKey& key) -> pcl::PointCloud<pcl::PointXYZI>::Ptr {
      auto it = terrainVoxelCloudMap.find(key);
      if (it == terrainVoxelCloudMap.end()) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        terrainVoxelCloudMap[key] = cloud;
        terrainVoxelUpdateNumMap[key] = 0;
        terrainVoxelUpdateTimeMap[key] = 0;
        return cloud;
      }
      return it->second;
    };

    // 将点云分配到体素中
    pcl::PointXYZI point;
    int laserCloudCropSize = laserCloudCrop->points.size();
    for (int i = 0; i < laserCloudCropSize; i++) {
      point = laserCloudCrop->points[i];
      
      VoxelKey key = getTerrainVoxelKey(point.x, point.y);
      auto terrainVoxelCloudPtr = getTerrainVoxelCloud(key);
      terrainVoxelCloudPtr->push_back(point);
      terrainVoxelUpdateNumMap[key]++;
    }

    // 检查并更新每个体素
    for (auto& voxelPair : terrainVoxelCloudMap) {
      VoxelKey key = voxelPair.first;
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = voxelPair.second;
      
      // Expire individual source observations on EVERY scan, even when this
      // voxel has not reached its downsampling/rebuild interval. Remove before
      // VoxelGrid averages timestamp intensity with fresh observations.
      auto& points=terrainVoxelCloudPtr->points;
      points.erase(std::remove_if(points.begin(),points.end(),[this](const pcl::PointXYZI& p) {
        const double age=laserCloudTime-systemInitTime-p.intensity;
        const double distance=std::hypot(p.x-vehicleX,p.y-vehicleY);
        return !std::isfinite(age)||age<-.001||age>=decayTime||distance>=maxVoxelStorageDist||
               (clearingCloud&&distance<clearingDis);
      }),points.end());
      terrainVoxelCloudPtr->width=points.size();terrainVoxelCloudPtr->height=1;

      // 如果体素需要更新
      if (terrainVoxelUpdateNumMap[key] >= voxelPointUpdateThre ||
          laserCloudTime - systemInitTime - terrainVoxelUpdateTimeMap[key] >= voxelTimeUpdateThre ||
          clearingCloud) {
        
        // Read from a separate downsampled cloud before rebuilding this voxel.
        // Clearing the input first silently erased all fresh ground/obstacles.
        downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
        downSizeFilter.filter(*laserCloudDwz);
        terrainVoxelCloudPtr->clear();
        int laserCloudDwzSize = laserCloudDwz->points.size();
        for (int i = 0; i < laserCloudDwzSize; i++) {
          point = laserCloudDwz->points[i];
          float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                          (point.y - vehicleY) * (point.y - vehicleY));
          if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
              point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
              laserCloudTime - systemInitTime - point.intensity < decayTime &&
              !(dis < clearingDis && clearingCloud)) {
            terrainVoxelCloudPtr->push_back(point);
          }
        }

        terrainVoxelUpdateNumMap[key] = 0;
        terrainVoxelUpdateTimeMap[key] = laserCloudTime - systemInitTime;
      }
    }

    // 合并所有体素点云
    terrainCloud->clear();
    for (const auto& voxelPair : terrainVoxelCloudMap) {
      *terrainCloud += *(voxelPair.second);
    }

    voxelMs=milliseconds(phase); phase=SteadyClock::now();
    // 清除planar体素数据用于重新计算
    planarVoxelElevMap.clear();
    planarVoxelEdgeMap.clear();
    planarVoxelDyObsMap.clear();
    planarPointElevMap.clear();

    // 估计地面高度并计算每个点的高度差
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      VoxelKey planarKey = getPlanarVoxelKey(point.x, point.y);

      if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
        // 计算周围9个体素的影响
        for (int dX = -1; dX <= 1; dX++) {
          for (int dY = -1; dY <= 1; dY++) {
            VoxelKey neighborKey = {planarKey.x + dX, planarKey.y + dY};
            planarPointElevMap[neighborKey].push_back(point.z);
          }
        }
      }

      if (clearDyObs) {
        float pointX1 = point.x - vehicleX;
        float pointY1 = point.y - vehicleY;
        float pointZ1 = point.z - vehicleZ;

        float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
        float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / M_PI;
        if (angle1 > minDyObsAngle) {
          planarVoxelDyObsMap[planarKey] = 0;
        }
      }
    }

    // 使用排序或最小值方法计算地面高度
    if (useSorting) {
      for (auto& elevPair : planarPointElevMap) {
        std::vector<float>& elevations = elevPair.second;
        int planarPointElevSize = elevations.size();
        
        if (planarPointElevSize > 0) {
          planarVoxelElevMap[elevPair.first] = terrain_realtime::ground_quantile(
              elevations, quantileZ, limitGroundLift, maxGroundLift);
        }
      }
    } else {
      for (auto& elevPair : planarPointElevMap) {
        std::vector<float>& elevations = elevPair.second;
        int planarPointElevSize = elevations.size();
        
        if (planarPointElevSize > 0) {
          float minZ = 1000.0;
          int minID = -1;
          for (int j = 0; j < planarPointElevSize; j++) {
            if (elevations[j] < minZ) {
              minZ = elevations[j];
              minID = j;
            }
          }

          if (minID != -1) {
            planarVoxelElevMap[elevPair.first] = elevations[minID];
          }
        }
      }
    }

    groundMs=milliseconds(phase); phase=SteadyClock::now();
    terrainCloudElev->clear();
    obstacleCloud->clear();

    // 检测障碍物
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];
      if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
        VoxelKey planarKey = getPlanarVoxelKey(point.x, point.y);
        
        // 检查体素中是否有高度信息
        auto elevIt = planarVoxelElevMap.find(planarKey);
        if (elevIt != planarVoxelElevMap.end()) {
          auto dyObsIt = planarVoxelDyObsMap.find(planarKey);
          bool dyObsCheck = dyObsIt == planarVoxelDyObsMap.end() || 
                            dyObsIt->second < minDyObsPointNum ||
                            !clearDyObs;
                            
          if (dyObsCheck) {
            float disZ = point.z - elevIt->second;
            
            auto pointElevIt = planarPointElevMap.find(planarKey);
            int planarPointElevSize = (pointElevIt != planarPointElevMap.end()) ? 
                                      pointElevIt->second.size() : 0;
                
            if (disZ > minObstacleHeight && disZ < vehicleHeight && 
                planarPointElevSize >= minBlockPointNum) {
              point.intensity = disZ;
              obstacleCloud->push_back(point);
            }
          }
        }
      }
    }

    // 处理无数据区域
    if (noDataObstacle && noDataInited == 2) {
      // 标记边缘体素
      for (auto& elevPair : planarPointElevMap) {
        VoxelKey key = elevPair.first;
        int planarPointElevSize = elevPair.second.size();
        
        if (planarPointElevSize < minBlockPointNum) {
          planarVoxelEdgeMap[key] = 1;
        }
      }

      // 扩展边缘体素
      for (int noDataBlockSkipCount = 0;
           noDataBlockSkipCount < noDataBlockSkipNum;
           noDataBlockSkipCount++) {
        
        std::unordered_map<VoxelKey, int, VoxelKeyHash> tempEdgeMap = planarVoxelEdgeMap;
        
        for (auto& edgePair : planarVoxelEdgeMap) {
          if (edgePair.second >= 1) {
            VoxelKey key = edgePair.first;
            bool edgeVoxel = false;
            
            for (int dX = -1; dX <= 1; dX++) {
              for (int dY = -1; dY <= 1; dY++) {
                VoxelKey neighborKey = {key.x + dX, key.y + dY};
                
                auto neighborIt = planarVoxelEdgeMap.find(neighborKey);
                if (neighborIt != planarVoxelEdgeMap.end() && 
                    neighborIt->second < edgePair.second) {
                  edgeVoxel = true;
                }
              }
            }

            if (!edgeVoxel)
              tempEdgeMap[key]++;
          }
        }
        
        planarVoxelEdgeMap = tempEdgeMap;
      }

      // 为无数据区域添加障碍物点
      for (auto& edgePair : planarVoxelEdgeMap) {
        if (edgePair.second > noDataBlockSkipNum) {
          VoxelKey key = edgePair.first;
          
          point.x = planarVoxelSize * key.x + vehicleX;
          point.y = planarVoxelSize * key.y + vehicleY;
          point.z = vehicleZ;
          point.intensity = vehicleHeight;

          // 创建四个角点以表示无数据区域
          point.x -= planarVoxelSize / 4.0;
          point.y -= planarVoxelSize / 4.0;
          terrainCloudElev->push_back(point);

          point.x += planarVoxelSize / 2.0;
          terrainCloudElev->push_back(point);

          point.y += planarVoxelSize / 2.0;
          terrainCloudElev->push_back(point);

          point.x -= planarVoxelSize / 2.0;
          terrainCloudElev->push_back(point);
        }
      }
    }

    classifyMs=milliseconds(phase); phase=SteadyClock::now();
    // Build only from the current age-filtered terrain map. Replaying a
    // processed cloud with a new header would keep expired obstacles alive.
    std::size_t selfPointsRemovedAtOutput = 0;
    currentUnion.clear();
    currentUnion.add(*obstacleCloud,[this,&selfPointsRemovedAtOutput](const pcl::PointXYZI& p) {
      if(isInsideSelfFilterBox(p)){++selfPointsRemovedAtOutput;return false;}
      return true;
    });
    accumulatedCloud->assign(currentUnion.points().begin(),currentUnion.points().end());

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "自车点云发布前过滤：删除 %zu 个点，剩余 %zu 个点",
        selfPointsRemovedAtOutput, accumulatedCloud->size());
      
    mergeMs=milliseconds(phase); phase=SteadyClock::now();
    // 发布合并后的点云
    sensor_msgs::msg::PointCloud2 obstacleCloud2;
    pcl::toROSMsg(*accumulatedCloud, obstacleCloud2);
    obstacleCloud2.header.stamp = 
        rclcpp::Time(static_cast<uint64_t>(laserCloudTime * 1e9));
    obstacleCloud2.header.frame_id = "odom_init";
    serializeMs=milliseconds(phase); phase=SteadyClock::now();
    terrain_cloud_pub_->publish(obstacleCloud2);
    publishMs=milliseconds(phase);
    
    lastPublishTime = laserCloudTime;
    
    clearingCloud = false;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerrainAnalysisNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
