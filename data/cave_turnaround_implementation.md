# 封闭洞穴原地掉头

本次变更使用走廊地图中的双侧墙壁和端墙识别封闭端，连续确认时间为 **10 秒**。

## 触发条件

- 复用现有走廊前进轴线与实测双侧墙宽，不使用车头朝向判断封堵。
- 在中线投影前方 2 m 内查找连接双侧墙壁的端墙，同时要求低墙障碍点和新鲜全局、局部地图的实体障碍支持。
- 检查端墙前的两侧墙壁是否连续，并独立检查侧向通道。墙体缺失、遮挡、未知地图、单纯规划失败和绘图多边形的端边都不能确认封闭。
- 要求已观测到可通行的接近地面；证据不足时输出 UNKNOWN。
- 封闭候选出现后，任务状态仍为 EXPLORING，但运动门控先停车。随后要求同一端墙持续获得新数据，单调时钟时间和源观测时间跨度都达到 10 秒。重复发布旧数据不能累计确认。
- 离开探索状态即清零。抓取结束恢复探索后，只接受该次恢复之后采集的观测，重新确认完整 10 秒。
- 抓取时间或次数耗尽后，封闭检测仍有效；已进入出口前行阶段时不触发。

## 执行流程

EXPLORING → TURNAROUND_HANDOFF → TURNING_AROUND → RESUMING → EXPLORING。

先关闭探索、等待导航动作结束及底盘停稳，再检查左右两个方向的完整 180° 车身扫掠。使用现有对齐速度通道和运动门控，线速度为零，依据里程计累计角度闭环旋转；累计角度能够跨越 ±π。

车身轮廓由任务启动文件从 Nav2 的矩形 footprint 和 padding 派生，不复用抓取的小角度调整范围。地图更新导致扫掠碰撞、输入失效、位置漂移超过 4 cm 或超时，均停车进入 FAULT。当前轮廓绕 base_frame 的扫掠直径约 1.33 m，另需余量；空间不足不会强行旋转。

旋转后先停稳，经 `/explore/reorient_corridor` 确认局部方向重置成功，才重新开启探索。该接口保留已有地图、观测地面和探索记录。普通抓取暂停不重置走廊方向。暂停/取消保留已转角度，显式恢复后先重新检查剩余扫掠；异步接口的过期回复不会恢复运动。

每次任务只完成一次自动掉头；任务重置或新任务开始后清除标志。

## 参数及观测

`src/state/config/mission.yaml`：

```yaml
turnaround.enabled: true
turnaround.confirmation_seconds: 10.0
turnaround.angular_speed: 0.35
turnaround.timeout: 30.0
turnaround.footprint: [0.21, 0.59, 0.31, 0.01]
```

footprint 数组依次为前伸、后伸、半宽、余量，单位为米；完整任务启动时由实际 Nav2 配置覆盖。角速度单位为 rad/s，超时覆盖旋转及方向重置等待。

`src/explore/config/rover.yaml`：

```yaml
corridor.dead_end.enabled: true
corridor.dead_end.detection_distance: 2.0
```

`/explore/dead_end` 输出 `closed`、端墙位置/方向/宽度、原因、源观测时间。`/mission/state` 新增 `turnaround_hold`、`turnaround_active`、`turnaround_complete`、`dead_end_confirmation_elapsed` 和 `turnaround_progress_radians` 等字段。

## 验证范围

两个 ROS 包编译通过。任务包 8 项 CTest、探索包 3 项 CTest 全部通过。

新增测试覆盖实体封闭、90° 侧向延续、缺墙、未知地图、观测地面缺失、端墙通行缺口、检测距离、中线拟合与封闭检测的连接、车头方向变化、掉头后重新选路，以及 10 秒门限、重复源数据、抓取清零、过期观测、端墙候选变化、车身扫掠、±π 转角、暂停恢复、异步方向重置和漂移停车。时间门限测试通过构造时钟状态验证边界，不依赖真实等待 10 秒。

上述为自动化验证，尚未进行实车洞穴验证。本次没有启动或重启实车任务。

修改前源码备份：`data/change_backups/cave_turnaround_20261120_112821.tar.gz`。
