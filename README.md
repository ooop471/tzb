# tzb

## 洞内探索配套抓取入口（2026-11-01）

新增 `ros2 launch lite_motion_planner exploration_grasp_arm.launch.py`：运行相机、
视觉、规划器和受状态机管理的抓取节点，不启动串口、不自动移动机械臂。
配合 nav_explore_ws 的 `rover_state/exploration_grasp.launch.py` 使用，详见
`../nav_explore_ws/src/state/README.md`。旧的抓取独立入口默认行为保留。
新增导航中心输出与原抓取位姿分离；新模式通过真实关节反馈计算相机位置，
通过 session/task_id 握手和本次归位姿态的实际反馈允许恢复底盘。
已同步机载的 `tzb.pre_ini_joint_positions`：抓取放置后返回
`[0.0, 1.0, -2.61, 0.0, 1.3, 0.0]` rad。根据首轮实测反馈，任务准备、成功返回和失败恢复
均使用这个 pre_ini 姿态继续探索。到达抓取位置后仍按原顺序经过 ini → observe，
再进行视觉定位与抓取。
导航视觉按 RGB 拍摄时间从 0.75 s 深度历史中选取对应点云，置信度与原检测发布门槛
一致（0.1）；同一次检测只产生一次导航观测，避免沿用旧框生成连续确认。
`/perception/navigation_status` 报告点云配对、ROI 点数、配准和目标中心发布结果。


## 本地编译（Ubuntu 22.04 / ROS 2 Humble）

工作空间：`/home/drr/tzb_ws`。在新终端执行：

```bash
cd /home/drr/tzb_ws
bash scripts/build_workspace.bash
source setup.bash
```

构建脚本默认逐包编译，每个包最多 2 个编译进程，使用 Release 配置。
可通过 `TZB_BUILD_JOBS=4 bash scripts/build_workspace.bash` 调整进程数。
仅修改一个包时，例如：

```bash
bash scripts/build_workspace.bash --packages-select lite_motion_planner
```

缺失的构建依赖从本机配置的 apt 仓库下载，并解压到 `.deps/root`，不需要
sudo，也不会写入系统 ROS 安装。首次构建自动准备依赖；需要重新准备时执行
`bash scripts/prepare_local_deps.bash`。依赖包括 Pinocchio、Coal、Ruckig、
OMPL、MoveIt 消息、geometric_shapes、serial_driver 和 Orbbec 的构建依赖。
上游 Pinocchio/Coal 的 Python 导出目标使用绝对路径，准备脚本将其改为
相对于本地依赖前缀的路径。OMPL 1.5 的 CMake 变量接口由规划器构建文件兼容。

运行或使用 Python 接口前，建议加载工作空间根目录的 `setup.bash`，它同时加载
ROS、本地依赖和本工作空间。目录移动后需重新生成 build/install，勿复用旧缓存。
不要预先加载导航工作空间的环境，两个工作空间包含不同用途的串口驱动。

编译与完整抓取运行的依赖不同：本机尚未安装 `open3d`、`ultralytics`、
`openvino`、`torch`；可选的旧版 `nav2arm` 还依赖外部 `tzb_state_test` 包。
总启动文件、规划器 YAML、视觉模型配置仍有 `/home/tzb/tzb_ws` 机载绝对路径，
启动本地抓取前需单独适配。当前阶段只验证本地构建，不自动启动相机、串口或抓取任务。
下载的 Orbbec 源码不含 `orbbec_camera/config` 预设目录，安装时明确给出警告并
跳过该目录；依赖其中 YAML 的相机启动方式需要从机载原目录补齐配置后再使用。

2026-11-01 本机验证结果：15 个包全部编译安装成功。`planning_node`、
`tzb_catch`、相机驱动和串口驱动的动态库依赖均能解析，自定义消息导入正常，
80 个 Python 文件语法检查通过。使用仅在 build 目录中替换机载路径的临时参数，
在 ROS 域 219、仅本机通信模式下启动 `planning_node`，成功加载 20 个碰撞几何体、
189 对碰撞关系，并正常退出；未连接硬件。该检查验证模型与依赖能加载，不代表
抓取效果或完整视觉链路已验证。日志为 `log/local_planner_smoke.log`。
本次使用 Pinocchio 4.0.0、Coal 3.0.3、Ruckig 0.9.2、OMPL 1.5.2。

# ROS 2 话题接口

本文档列出当前工程主要节点的默认 ROS 2 话题。话题名均可通过节点参数或 YAML 配置修改。

## 数据流

```text
Orbbec 相机
  ├─ RGB 图像 ─> yolo_node ─> 2D 检测框
  └─ 点云 ──────────────────> pose_estimator ─> 目标 6DoF 位姿
                                                   │
                                                   v
                                              tzb_catch
                                                   │
                                                   v
                                               串口驱动
```

## `yolo_node`

| 方向 | 话题 | 消息类型 | 含义 |
| --- | --- | --- | --- |
| 订阅 | `/camera/color/image_raw` | `sensor_msgs/Image` | 相机 RGB 原始图像 |
| 发布 | `/yolo/detections` | `yolo_node/DetectionArray` | YOLO 检测结果 |
| 发布 | `/yolo/annotated_image` | `sensor_msgs/Image` | 绘制检测框、类别和置信度的图像 |

`DetectionArray.detections[]` 中单个目标的字段：

| 字段 | 含义 |
| --- | --- |
| `x`, `y` | 检测框左上角像素坐标 |
| `width`, `height` | 检测框宽高，单位为像素 |
| `confidence` | 检测置信度，范围为 `0.0~1.0` |
| `class_id` | 类别编号 |
| `class_name` | 类别名称 |

检测结果只有在置信度达到发布门限时才发布；标注图像仅在存在订阅者时生成。配置见 [yolo_detector.py](src/yolo_node/yolo_node/yolo_detector.py#L132)。

## `pose_estimator`

| 方向 | 话题 | 消息类型 | 含义 |
| --- | --- | --- | --- |
| 订阅 | `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | 与彩色图对齐的有序点云 |
| 订阅 | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| 订阅 | `/yolo/detections` | `yolo_node/DetectionArray` | YOLO 二维检测框 |
| 发布 | `/pose_estimator/object_pose` | `geometry_msgs/PoseStamped` | 目标最终 6DoF 位姿 |
| 发布 | `/pose_estimation_node/roi_cloud` | `sensor_msgs/PointCloud2` | 检测框内降采样后的目标点云 |
| 发布 | `/pose_estimation_node/coarse_pose` | `geometry_msgs/PoseStamped` | RANSAC 粗配准位姿 |
| 发布 | `/pose_estimation_node/coarse_model_marker` | `visualization_msgs/Marker` | 粗配准模型，RViz 中为蓝色 |
| 发布 | `/pose_estimation_node/model_marker` | `visualization_msgs/Marker` | ICP 最终配准模型，RViz 中为绿色 |
| 发布 | `/tf` | `tf2_msgs/TFMessage` | 父坐标系到 `object_pose` 的动态变换 |

`object_pose` 字段含义：

- `header.frame_id`：位姿参考坐标系，通常来自相机点云。
- `position`：目标中心位置，单位为米。
- `orientation`：目标姿态四元数；当前 `pose_yaw: 0.0` 会覆盖输出 yaw。

配置见 [params.yaml](src/pose_estimator/config/params.yaml#L1)。

## `tzb_catch`

| 方向 | 话题 | 消息类型 | 含义 |
| --- | --- | --- | --- |
| 订阅 | `/pose_estimator/object_pose` | `geometry_msgs/PoseStamped` | 视觉目标位姿 |
| 订阅 | `/serial/arm_joint_state` | `sensor_msgs/JointState` | 机械臂实际关节反馈 |
| 订阅 | `/navigation_goal_status` | `std_msgs/Int8` | `0`：等待或重新使能；`1`：触发抓取 |
| 订阅 | `/tzb_catch/start` | `std_msgs/Int8` | 值为 `1` 时启动抓取任务 |
| 发布 | `/tzb_catch/arm_joint_command` | `sensor_msgs/JointState` | 六个机械臂关节目标角，单位为 rad |
| 发布 | `/tzb_catch/gripper_command` | `std_msgs/Int8` | `1`：闭合；`-1`：张开 |
| 发布 | `/tzb_catch/result` | `std_msgs/Int8`  自动查询 TF。因此配置矩阵的方向必须正确。

**先核对正在使用的矩阵。**运行 ros2 param get /tzb_catch tzb.pose_estimator_refer| `1`：任务成功；`-1`：任务失败 |
| 发布 | `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 规划后的关节轨迹 |
| 发布 | `/joint_states` | `sensor_msgs/JointState` | RViz 模型关节状态 |
| 发布 | `/tzb_catch/e3_marker` | `visualization_msgs/Marker` | 目标物体模型 |
| 发布 | `/tzb_catch/reference_frames` | `visualization_msgs/MarkerArray` | 参考坐标系和 TCP 轨迹可视化 |

默认配置见 [tzb_planner.yaml](src/lite_motion_planner/config/tzb_planner.yaml#L41)。

## `rm_serial_driver`

| 方向 | 话题 | 消息类型 | 含义 |
| --- | --- | --- | --- |
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度命令 |
| 订阅 | `/tzb_catch/arm_joint_command` | `sensor_msgs/JointState` | 机械臂关节命令 |
| 订阅 | `/tzb_catch/gripper_command` | `std_msgs/Int8` | 夹爪开合命令 |
| 发布 | `/serial/arm_joint_state` | `sensor_msgs/JointState` | 下位机返回的实际关节角 |

## 可选通信节点

- `arm2nav`：通过 `/arm2nav`、`/arm2nav/command`、`/arm2nav/from_arm` 交换目标停车、导航建议位置、抓取触发和返航状态。
- `nav2arm`：旧版接口，通过 `tzb_state_test`、`/tzb_catch/start` 和 `/tzb_catch/result` 协调导航与机械臂。
- 默认总启动文件中两者均未启用，分别需要设置 `use_arm2nav:=true` 或 `use_nav2arm:=true`。




2026.10.10
新增了视觉给导航二维地图发送终点和yaw值（相对于此时的自身位置，请注意坐标系转换）的能力


运行指令：
```text
source install/setup.bash 
ros2 launch lite_motion_planner bring_launch.launch.py 
```
