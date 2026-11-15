# tzb_gripper_test

只给机械臂夹爪发 `std_msgs/Int8` 报文的小工具，不依赖机械臂/视觉/规划栈。

## 协议

| 值 | 含义 |
|---:|---|
| -1 | 张开 |
| 1 | 闭合档位 1（blue_block） |
| 2 | 闭合档位 2（big_rock） |
| 3 | 闭合档位 3（small_rock） |

话题：`/tzb_catch/gripper_command`（QoS：reliable + transient_local，和 `rm_serial_driver` 的订阅一致）。
其它值会被忽略并打 WARNING。

## 启动

```bash
source /opt/ros/humble/setup.bash
source ~/tzb_ws/install/setup.bash

# 张开
ros2 launch tzb_gripper_test gripper_test.launch.py command:=-1
# 闭合档位 1/2/3
ros2 launch tzb_gripper_test gripper_test.launch.py command:=1
ros2 launch tzb_gripper_test gripper_test.launch.py command:=2
ros2 launch tzb_gripper_test gripper_test.launch.py command:=3
```

或者不用 launch：

```bash
ros2 run tzb_gripper_test gripper_test --ros-args -p command:=2
```

运行中切换档位（不用重启）：

```bash
ros2 topic pub --once /gripper_test/command std_msgs/msg/Int8 "{data: 3}"
```

确认发出去的值：

```bash
ros2 topic echo /tzb_catch/gripper_command --qos-durability transient_local
```

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `command` | -1 | 初始报文，只接受 -1/1/2/3 |
| `topic` | `/tzb_catch/gripper_command` | 目标话题 |
| `command_topic` | `/gripper_test/command` | 运行时改值的输入话题 |
| `publish_hz` | 10.0 | 重复发送频率（驱动会锁存最后一条） |

## 前提

串口驱动要在跑，否则报文不会到电控：

```bash
ros2 run rm_serial_driver node --ros-args \
  --params-file ~/tzb_ws/install/rm_serial_driver/share/rm_serial_driver/config/serial_params.yaml \
  -r __node:=arm_serial_driver -r /cmd_vel:=/arm_task/unused_chassis_command
```

节点只发夹爪命令，不发任何关节轨迹；但夹爪会真的动，注意爪里/周围别夹到手或线。
