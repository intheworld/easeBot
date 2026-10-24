# 达妙底盘

底盘使用独立 `/dev/hei_chassis` USB-CAN 串口，由现有 `hei_hardware/DamiaoSystem` 管理四个 DM4310，导出四个 velocity 命令接口。上层复用 `omni_wheel_drive_controller`，接收 `geometry_msgs/msg/TwistStamped`；不使用 Action 控制连续底盘速度，不包含升降电机。

`hei_bringup/config/base.yaml` 沿用 hei-rebot-lift 的映射和协议配置：

| 电机 ID | 反馈 ID | 轮子 | ROS 关节 |
| --- | --- | --- | --- |
| 1 | 17 | 右前 | front_right_wheel_joint |
| 2 | 18 | 右后 | rear_right_wheel_joint |
| 3 | 19 | 左后 | rear_left_wheel_joint |
| 4 | 20 | 左前 | front_left_wheel_joint |

串口 921600，VEL 模式 3，协议量程 p/v/t 为 12.5/30/10，方向初始均 +1、零偏 0、传动比 1，轮速命令范围 ±9 rad/s。配置范围属于项目命令约束，不用于检查轮子绝对转角。原项目的 9/2 是归一化车体输入的经验增益，不能当成轮半径或直接套到 ROS 的 m/s、rad/s；本入口保留现有底盘控制器的几何参数（轮半径 0.05 m、机器人半径 0.3162 m、轮角偏置 π/4），尚未实机标定。不声明里程计精度，默认不发布 odom TF。原项目每轮 8 rad/s² 的变化率限制尚未接入，此版本只验证速度控制链路和超时停止。

## 启动

```bash
# 默认可写：自动配置 VEL、激活控制器，使能四轮并发送零速度。
ros2 launch hei_bringup damiao_base.launch.py

# 只读：四轮反馈，不切换模式、不使能、不加载运动控制器。
ros2 launch hei_bringup damiao_base.launch.py read_only:=true

# 关闭自动配置模式，只接受已处于 VEL 的电机。
ros2 launch hei_bringup damiao_base.launch.py configure_control_mode:=false
```

`activate_base:=false` 可将可写底盘控制器留在 inactive。`config_file` 可指定硬件配置。先退出占用同一串口的进程；底盘可与双臂 launch 同时运行，使用独立管理器 `/hei/base_controller_manager`。状态话题为 `/hei/base_joint_states`，description 和 TF 分别使用 `base_` 前缀，避免与双臂冲突。不是全机器人 TF 合并入口。

```bash
ros2 control list_controllers -c /hei/base_controller_manager
ros2 topic echo /hei/base_joint_states
```

## 速度命令

先架空检查轮序与转动方向。以下是持续以 10 Hz 发布的低速向前目标；首次实机运动仍需验证方向和几何关系：

```bash
ros2 topic pub -r 10 /hei/base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.01, y: 0.0}, angular: {z: 0.0}}}'
```

`linear.x/y` 为 m/s，`angular.z` 为 rad/s。必须携带当前时间戳，`header: auto` 由 ROS CLI 逐次填写。停止发布后，控制器在 `cmd_vel_timeout: 0.3` 秒后输出四轮零速度；需要停止控制器并失能时：

```bash
ros2 control set_controller_state base_controller inactive -c /hei/base_controller_manager
```

超出 ±9 rad/s 的轮速命令会被硬件适配器拒绝并触发失能，不自动按比例缩放；首次测试应使用低速目标。通信超时、模式变化和电机故障沿用共享驱动的失能处理。

2026-10-24 实机只读检查：四个 ID 均响应，模式 3、失能、无故障；没有使能或运动测试。虚拟串口测试覆盖只读、自动激活、保留 inactive、RAM VEL 设置、Twist 四轮输出、超时归零及退出失能。
