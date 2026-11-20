# 2026-11-19 下午任务自启动故障诊断

时间为 Asia/Shanghai（北京时间）。检查与修复未启动任何真实驱动或机器人任务。

## 1. 15:46–17:11：参数解析失败（已修复）

10 个运行目录：154601、154745、155040、155357、160053、160409、160717、161046、170323、171053。
这些目录的 `algorithms.log` 均记录：

```
RCLInvalidROSArgsError
Sequence should be of same type. Value type 'integer' do not belong at line_num 9
```

实际读取文件为 `install/rover_state/share/rover_state/config/mission.yaml`。
`entry_navigation.goal: [1.0,0,0]` 混用 double 与 integer，ROS 2 的 C 参数解析器不接受。
源码与安装副本的修改时间均为 15:42；这与后续运行开始报错一致，但不能据此确定是谁修改。
`mission_state` 首先以 -6 退出，launch 随后结束其他节点，监督脚本报告 algorithms exited (1)。
这是任务配置问题；安装脚本只安装/启用 systemd 单元，本身没有在开机时执行。

修复：源码和安装副本均改为 `[1.0, 0.0, 0.0]`，保持原有目标值。
在 `start_mission.bash` 启动驱动之前调用新增 `check_mission_params.py`，使用
`rclpy.utilities.remove_ros_args` 调用原生 ROS 参数解析器，验证安装后的 mission、rover、mid360 和 Nav2 参数。
此检查不创建节点、不初始化 ROS context、不访问串口、不发送运动命令。
之前的 preflight 只确认环境和包存在，不能发现这类参数错误。

## 2. 15:10、15:19：入口导航无法取得进展（仍需实机定位）

两个任务已成功自动启动，经过机械臂准备、Nav2 激活、入口规划后，FollowPath 每约 6 秒
报告 `Failed to make progress`；3 次重试后进入 `FAULT: Entry navigation retry limit reached`。
这不是自启动失败。

`/home/tzb/serial_log/serial_20261119_151927.txt` 显示 15:19:44–15:20:03 发送过非零指令：
linear_x 最高 0.30 m/s、angular_z 最高约 0.26017 rad/s，而该时段日志中的定位高度和地面观测变化很小。
串口 TX 成功只能证明上位机写出，不能证明下位机收到或实际执行。
当前日志不能区分底盘使能/遥控模式、低速转向死区、物理受阻、定位反馈异常或控制参数问题。

后续应在受控实机验证中同时记录 `/mission/nav_cmd_vel`、`/cmd_vel`、`/mission/gate_state`、
`/mission/motion_permission`、`/odometry`、底盘反馈与实际车体运动，比较命令和反馈。
先确认底盘能执行指令与定位正常，再评估 progress checker 的 0.08 m、6 s 阈值。
本次未放宽阈值，也未更改运动限速或安全门。

## 3. 停止时 arm_serial 超时（已修复信号传递路径）

监督脚本只给启动进程 PID 发 SIGINT；`ros2 run` 的包装进程捕获 SIGINT 后继续等待，
假设子进程也收到了终端进程组信号，并不转发单 PID 信号。因此实际串口驱动一直运行，
直到 10 秒后 supervisor 向剩余进程发 SIGTERM。最新 arm_serial.log 的 shutdown signal=15 与此一致。

改为在相同 arm 环境内直接 exec 同一个已安装的 `rm_serial_driver/node`，保留全部参数，
让 supervisor 的 SIGINT 直接到达驱动。增加可执行文件预检。
离线替身进程测试已验证 SIGINT 被接收、正常退出、没有进程残留；真实机械臂停机尚未实机复测。

## 4. 退出阶段的其他错误

`KeyboardInterrupt`、`InvalidHandle / cannot use Destroyable`、context invalid 以及
fastlio_mapping 的 -11 出现在 mission_state 首次崩溃、launch 开始发 SIGINT 之后。
它们属于此次退出时发生的次生错误；不能用它们替代参数错误作为启动失败根因。
FAST-LIO 退出段错误仍是独立清理缺陷，如修正配置后正常停机仍复现，应单独用 core/backtrace 定位。

## 验证与部署状态

- 使用原生 ROS 解析器复现旧配置错误，并验证修复后源码与安装配置均可解析。
- `bash scripts/start_mission.bash --check` 通过，包括四个安装参数文件。
- `python3 -m unittest discover -s scripts/tests -v`：9 项离线测试通过。
- Bash 语法检查和 systemd 单元 verify 通过。
- 不需要重编译 C++：已同步实际安装配置，systemd 直接调用工作区脚本。
- 未启动/重启任务；自启动仍为 enabled，下次开机使用修复后的配置与脚本。
- systemd 当前 failed 为此前运行的历史结果，不能表示修复后已通过实机运行。

原文件备份：`data/change_backups/mission_autostart_fix_20261119_180310/`。
