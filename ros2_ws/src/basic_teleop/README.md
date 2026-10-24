# basic_teleop

终端键盘全向底盘遥操作，发布 `geometry_msgs/msg/TwistStamped`。

| 按键 | 机体速度目标 |
| --- | --- |
| W / S | 前进 / 后退，linear.x = ±0.05 m/s |
| A / D | 左 / 右横移，linear.y = ±0.05 m/s |
| Q / E | 逆 / 顺时针旋转，angular.z = ±0.15 rad/s |
| 空格 | 所有速度归零 |
| Ctrl+C | 退出 |

接受大小写字母；每次按键仅选择一个运动轴，其他轴归零，不支持多键同时组合。未识别的按键同样归零。终端仅关闭规范输入（ICANON），保留原有回显设置，无需回车，退出时恢复终端设置。

500 ms 未收到新的按键字符时归零。终端没有可靠的键释放事件，长按依赖操作系统按键重复，因此该超时不是精确的“松手即停”；系统的初始按键重复延迟也可能造成短暂停顿。发布端退出或失联后的停止由控制器自己的命令超时负责。

节点默认 namespace 为 `/hei`。话题为相对名称 `base_controller/cmd_vel`，默认解析为 `/hei/base_controller/cmd_vel`。接当前 HEI mock：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
# ROS_DOMAIN_ID 与 bringup 一致；需要先通过独立 manager 接口激活控制器
ros2 run basic_teleop basic_teleop_node
```

可以使用 `ros2 run basic_teleop basic_teleop_node --ros-args -r __ns:=/robot1` 覆盖默认 namespace。

控制值是 ROS 机体速度目标，底盘几何、方向与硬件适配仍以实际验证为准。

速度可以在启动时配置，接受有限非负数；0 可禁用对应平移或旋转轴。参数启动后只读，修改需重启节点：

```bash
ros2 run basic_teleop basic_teleop_node --ros-args \
  -p linear_speed:=0.02 -p angular_speed:=0.1
```

默认平移 0.05 m/s、旋转 0.15 rad/s；该节点也可直接连接 `damiao_base.launch.py read_only:=false` 的真实底盘，默认话题一致。
