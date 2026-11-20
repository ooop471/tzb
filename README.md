# nav_explore_ws

基于 ROS 2 Humble 的车辆定位、洞穴探索和抓取任务工作区。
完整流程：入口导航 → 探索 → 发现目标并接近抓取 → 恢复探索 → 检测出洞并前行 1 m → 终点导航 → 卸载。

## 目录

| 目录 | 功能 |
| --- | --- |
| `src/localization/fast_lio` | Fast-LIO 定位与点云建图 |
| `src/navigation` | Nav2 启动配置、地形分析 |
| `src/explore` | 探索目标选择、Nav2 路径规划与跟随协调 |
| `src/state` | 任务状态机、机械臂协作和速度门控 |
| `src/bringup` | 完整算法入口、导航参数和 RViz 可视化 |
| `src/preprocess`、`src/drivers` | IMU／点云预处理、Livox 雷达驱动 |
| `scripts` | 一键启动、录包脚本 |
| `.deps` | 本地 Livox SDK2 和 OR-Tools 依赖 |

## 编译

需要 ROS 2 Humble、Nav2，以及相邻的 `tzb_ws`（视觉、机械臂及动作接口）。
完整运行还需要 `serial_t`（底盘串口）。默认目录如下：

```text
~/nav_explore_ws
~/tzb_ws
~/serial_t
```

```bash
cd ~/nav_explore_ws
source /opt/ros/humble/setup.bash
source ../tzb_ws/install/setup.bash
source .deps/livox-sdk2/setup.bash
MAKEFLAGS=-j2 colcon build --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

跨电脑迁移时，不复制 `build/`、`install/`、`log/`；在目标电脑重新编译。
`.deps` 中的库需匹配目标电脑架构。

## 一键启动完整任务

```bash
cd ~/nav_explore_ws
bash scripts/start_mission.bash --check
bash scripts/start_mission.bash
```

脚本依次启动相机、雷达并检查实际数据，再启动底盘和机械臂串口，收到机械臂关节反馈后启动完整算法。
每个传感器最多尝试 3 次，每次等待数据最多 40 秒；失败后重启，仍失败则停止流程。

任务在输入和设备就绪后自动开始，无需手动调用启动服务。
入口导航和终点导航最高前进速度为 0.30 m/s；洞内探索及出洞后的 1 m 延伸段仍为 0.15 m/s，抓取接近和倒退恢复速度保持原设置。洞外速度参数为 `src/state/config/mission.yaml` 中的 `navigation_speed.outdoor_max_linear`。
入口导航和终点导航将全局、局部膨胀半径设为 0.37 m；洞内探索及出洞后的 1 m 延伸段使用原配置（当前 0.10 m）。阶段交接时先完成膨胀参数切换和地图更新，再开始规划或探索。
出洞检测直接使用 `/livox/lidar` 原始点云：以拟合地面为高度基准，先确认看到不高于 1.8 m 的低顶棚；随后超过 2 m 的高处点云投影覆盖至少 1 m²、占顶面有效网格至少 65%，并持续 0.8 秒时确认出洞。确认前还会将高处回波沿雷达观测射线反投影到低洞顶高度：若集中在小开口内，且至少三侧（包括前方）仍有同高度低顶棚，则按天窗处理。随后沿确认方向前行实际 1 m，再导航返回任务起点；不再使用左右侧壁点数下降判断。
参数位于 `src/explore/config/rover.yaml` 的 `corridor.ceiling_exit.*` 和 `corridor.exit_forward_distance`，诊断话题为 `/explore/exit_detection`。
Ctrl+C 退出时先停止算法，保留串口供机械臂回位，再停止串口和传感器。
脚本追踪本次启动的进程及子进程（含数据检查进程）；正常退出超时后依次发送 SIGTERM、SIGKILL，最后检查残留。
终端断开也会触发清理；退出完成会显示 `Cleanup complete: no processes from this run remain.`。
日志保存于 `log/mission/时间/`。

脚本通过 `mission_arm.launch.py` 调用机械臂新入口 `lite_motion_planner/bring_launch.launch.py`，由状态机控制准备、抓取和卸载。
新入口已移除相机启动；任务模式复用外部机械臂串口，保留关节反馈。首次适配会备份修改前的 launch 文件。
相机只由传感器启动阶段启动。工作区位置不同时，可通过 `ARM_WS`、`CHASSIS_WS` 指定。

## 分开调试

先在每个终端加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/tzb_ws/install/setup.bash
source ~/nav_explore_ws/.deps/livox-sdk2/setup.bash
source ~/nav_explore_ws/install/setup.bash
```

| 命令 | 启动内容 |
| --- | --- |
| `ros2 launch fast_lio mapping.launch.py` | 雷达、IMU 预处理、Fast-LIO |
| `ros2 launch rover_explore explore.launch.py` | 雷达、定位、地形分析、探索及 Nav2 跟随 |
| `ros2 launch bringup bringup.launch.py` | 完整任务算法；相机、雷达和串口由外部启动 |
| `ros2 launch bringup state_visual.launch.py` | RViz 和任务文字状态显示 |

RViz 展示定位点云、地图、通道选点、导航路径、任务目标和机械臂等状态。
通道显示中：灰点为出口检测用的地面观测，绿点为中线上满足条件的路径端点样本，端点范围为前方 0.2～2.0 m，黄线为拟合中线，紫球为当前引导目标，白框为测得的通道边界。粗绿线是中线参考，青线是 Nav2 规划路径（含入口、探索和返程），加粗红线是探索控制器已经接收的跟随路径。
`/explore/corridor_selection` 保留阶段、候选数量和目标就绪状态供录包分析；RViz 使用 `/visualization/corridor_geometry`，仅显示几何标记。
机载端与可视化电脑均使用 ROS 默认域，不设置自定义 `ROS_DOMAIN_ID`，也不启用 localhost-only 限制。
局域网需允许 DDS 发现和数据通信。

## 录包

在另一个终端运行：

```bash
bash ~/nav_explore_ws/scripts/record_mission.bash
# 同时记录相机图像和深度点云：
bash ~/nav_explore_ws/scripts/record_mission.bash --vision
```

默认记录定位、雷达／IMU、地图、探索、导航路径、速度、状态机、视觉检测结果、机械臂及卸载数据。
保存到 `~/bags/mission_时间/`，每约 2 GB 分卷；相邻 `_context/` 目录保存参数快照、QoS 和命令。
使用 `--output /path/to/new_bag` 指定新目录，Ctrl+C 正常结束录制。
两个脚本均支持 `--dry-run` 查看命令而不启动任务或录包。

## 常用参数

| 文件 | 内容 |
| --- | --- |
| `src/state/config/mission.yaml` | 入口／终点、抓取范围、车体尺寸、任务超时和卸载设置 |
| `src/explore/config/rover.yaml` | 探索范围、候选目标、覆盖与通道策略 |
| `src/explore/config/exploration_planner.yaml` | 探索目标到 Nav2 路径的规划设置 |
| `src/explore/config/exploration_follower.yaml` | 路径跟随、更新与控制交接 |
| `src/bringup/config/nav2_params.yaml` | Nav2 控制器、规划器和代价地图 |
| `src/localization/fast_lio/config/mid360.yaml` | Fast-LIO 输入、滤波和外参 |
| `src/navigation/terrain_analysis/config/terrain_analysis_plus.yaml` | 地形分析与障碍物过滤 |

修改源码或参数后重新编译，再重新启动。

通道探索采用软中线引导：中线由连续的洞内左右内壁配对后取中点，洞外开阔地面不参与计算；一侧内壁缺测时，沿用已确认的洞宽，根据另一侧内壁推算中心。地面观测仅检查目标是否有地面支撑，不能把几何中线移向一侧。目标位于前方 0.2～2.0 m 的已观测通道中线，Nav2 自由规划接入路径，MPPI 根据偏心、朝向和拟合侧向余量优先居中，允许为接入或绕障偏离参考线。入口阶段减弱中线代价；实际地图上的碰撞仍由 Nav2 检查。膨胀半径为 0.10 m。中线横向估计经过空间平滑，航向参考采用较长区间的切向；洞内常规跟随角速度限制为 0.35 rad/s。

探索连续 8 秒没有向前进展，或跟随反复失败时，自动取消旧跟随，优先微退，确认实际后移 0.12 m 后再做小角度调整和重新规划。未退开时保持倒退优先，不回到同一受阻位置继续向前顶。单次倒退最多 0.30 m、速度 0.10 m/s，按定位反馈限制在上次向前进展位置后方 0.30 m；单次转向最多 10°，累计偏转限制约 ±12°。重试次数耗尽不会结束任务或永久等待同一路径变化。

Nav2 恢复动作因碰撞中止后，可进行有距离/角度/时间限制的接触调整：允许少量地图轮廓重叠，或逐步脱离已有重叠，拒绝继续加深明显重叠和扩大未知区域。该调整与其他恢复指令均经过速度平滑器和任务运动门控；暂停、租约失效、输入过期、退出或关闭时取消恢复。

`/explore/reference_path` 为软参考，`/explore/control_path` 为执行路径，`/explore/corridor_phase` 为接入/洞内/出口阶段；`/exploration/state` 包含恢复状态、次数与当前动作。RViz 和录包均已接入。离线录包回放可检查规划和控制输出，无法替代真实碰撞、车轮打滑及恢复运动验证。

当前正常行驶速度上限为 0.15 m/s。

可视化不显示浮动状态文字；路径只在显示副本中抬高，原始规划与控制话题保持原样。

紫色线显示 MPPI 优化后的局部预测轨迹，控制停止超过 1 秒后清除。绿色历史障碍栅格由地形障碍投影到 XY 平面后累计，同一位置不同高度合并为一个 5 cm 栅格，最多保留 20 万个栅格；只用于显示，不参与控制，新任务或定位重置时清空。可调用 `/visualization/clear_obstacle_history` 手动清空。

历史障碍二维地图话题为 `/visualization/obstacle_history_map`（OccupancyGrid），RViz 用绿色平面格子显示同一份占据数据。

抓取流程：探索阶段只运行 YOLO 和远距离定位；状态机锁定方块，沿探索路径接近，在车头前 30 cm 处触发停车，按定位反馈前后微调、必要时小幅调整朝向。到位后直接启动 `ros2 launch lite_motion_planner bring_launch.launch.py` 完成一次独立抓取，使用该入口原有的模型、手眼标定、配准和机械臂配置；状态机不再预观察、生成精配准参数副本或用粗定位筛选精配准结果。相机与串口复用全局脚本已启动的进程。抓取结果通过 `/tzb_catch/result` 返回，等待单独抓取进程退出且串口反馈确认机械臂收拢后，恢复探索识别和导航。每次抓取的日志保存在 `log/standalone_grasp/`。启动导航前仅检查机械臂已收拢，不自动驱动机械臂。

Orbbec 彩色帧队列仅保留最新一帧，解码和点云处理在队列锁外执行；相机使用 SDK 主机接收时间戳，图像/深度/点云采用传感器 QoS，内参仍保留可靠传输，避免旧画面积压。该修改不把过期目标改写成当前时刻。

卡住恢复：先倒退，再朝中线前方约 0.5 m 的方向小幅调整，重新规划并继续跟随；仍无进展时可继续使用剩余退让距离。单轮总退让范围保持 0.30 m，倒退速度 0.10 m/s，单次转向最多 10°。恢复指令经独立话题 `/mission/recovery_cmd_vel` 进入速度闸门，不受普通导航零速度覆盖；实际前进达到进展阈值后才清除恢复记录。

洞内探索和抓取位姿调整的角速度上限为 0.35 rad/s，相对稳定中线的偏转上限为 ±10°。普通抓取暂停保留已拟合中线与洞内阶段；新中线的明显横移需连续确认，单次横向更新最多 4 cm，避免随短时观测摆动。


探索中抓取失败会结束本次尝试、收臂并继续探索，失败目标暂时跳过30秒；单独抓取默认执行上限90秒，朝向调整和中线恢复均限制重试次数。详细分支见 [探索与抓取流程](docs/exploration_grasp_flow.md)。
