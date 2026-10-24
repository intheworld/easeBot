# 升降控制

`hei_hardware/LiftSystem` 复用 `DamiaoSystem` 的电机通信、VEL 模式、故障和使能确认，在其上处理升降机构的多圈位置、上下限位、软件回零和高度闭环。`hei_hardware/LiftTrajectoryController` 在标准 `joint_trajectory_controller` 上增加高度输入校验，复用其轨迹执行、取消和容差处理；没有另起进程争用电机串口。

参考配置来自 hei-rebot-lift 的 `_LiftRuntime`：DM4310，电机 ID 1 / 反馈 ID 17，`/dev/hei_lift`、921600 baud、VEL=3。限位 IO 为 `/dev/hei_lift_io`、115200 baud、8N1，被动接收带 Modbus CRC16 的 `01 04 02 00 XX CRClo CRChi` 帧；bit1 为上限位、bit2 为下限位，置 1 表示触发。不发送 IO 查询帧。

## 启动与状态

加载 Jazzy 和 workspace 环境后，默认可写、自动配置模式、回零并激活：

```bash
ros2 launch hei_bringup damiao_lift.launch.py
ros2 topic echo /hei/lift_joint_states
ros2 topic echo /hei/lift_dynamic_joint_states
```

显式使用 `read_only:=true` 时，只读不会配置电机模式、使能、回零或加载高度控制器。未回零时 `lift_joint/position` 只是从当前进程首次读数开始的相对位移，不能当作绝对高度；`lift_status/homed` 为 0。

只检查反馈、不回零：

```bash
ros2 launch hei_bringup damiao_lift.launch.py read_only:=true
```

回零向上运动，遇上限位立即发零速度，确认电机停止后建立软件高度零点；若启动时上限位已触发，直接建立软件零点。没有固件设零指令，不保存 EEPROM。回零成功后才自动激活 `lift_controller`。`activate_lift:=false` 可关闭回零后的自动激活；`read_only:=false home:=false` 只配置 inactive 控制器，未回零时硬件拒绝高度接口占用。

默认 `configure_control_mode:=true`，在失能反馈下设置并回读 RAM VEL 模式；可显式 false 只检查模式。进程重启或硬件重新激活后需重新回零；不复用上次进程的多圈偏移。

管理器为 `/hei/lift_controller_manager`，Action 为 `/hei/lift_controller/follow_joint_trajectory`。描述、状态与 TF 使用 `lift_` 前缀，可与独立底盘和双臂入口并行运行，不提供整机 TF 合并。

`lift_status` 状态包含：`upper`、`lower`、`io_online`、`homed`、`phase`、`motor_position`（原始电机 rad）、`motor_torque`（电机 N·m）。phase：0 未回零，1 回零中，2 已回零，3 故障。

## 高度命令

高度单位为米：上限位为 0，向下为负，参考目标范围为 [-0.8, 0]。确认控制器 active 后，下面命令下降 10 mm，轨迹用时 3 秒：

```bash
ros2 control list_controllers -c /hei/lift_controller_manager
ros2 action send_goal /hei/lift_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [lift_joint], points: [{positions: [-0.01], time_from_start: {sec: 3}}]}}' \
  --feedback
```

停用高度控制器会失能升降电机：

```bash
ros2 control set_controller_state lift_controller inactive -c /hei/lift_controller_manager
```

目标是相对于上限位零点的绝对高度，不是位移增量。例如 `-0.05` 表示上限位下方 50 mm，`0.05` 超出合法范围。Action 在接受之前校验整条轨迹，任一点越界或包含非有限值时拒绝整个目标，正在执行的合法目标继续运行。`joint_trajectory` 话题使用同样的校验，非法消息会丢弃并记录原因。范围读取自 URDF 的 position command interface，与硬件使用同一份配置。

合法端点配合速度/加速度也可能导致样条插值越界；此时只中止当前轨迹，返回 `INVALID_GOAL` 和插值越界说明，并保持当前位置。控制器和状态广播器保持 active，可继续发送合法目标。硬件层对遗漏的非法高度指令立即输出零速度并告警，不把输入错误升级为硬件故障。

## 参数与故障处理

硬件参数在 `hei_bringup/config/lift.yaml`，可用 `config_file` 指定其他配置。导程 10 mm/rev、向上电机符号 -1、反馈回绕周期 25 rad；展开多圈后换算高度。回零速度 10 rad/s，最大速度 18 rad/s，变化率 30 rad/s²，位置比例增益 0.45 rad/s/mm，停止容差 1 mm，均来自参考配置。

保留参考项目的回零超时 30 秒。按上述导程和回零速度，30 秒约覆盖 0.477 m，不能保证从 0.8 m 全行程下端回零；需要覆盖全行程时，应根据实机行程调整 YAML 的 `home_timeout_sec`。轨迹时长也须匹配最大高度速度约 0.0286 m/s。

限位 IO 必须有新鲜且 CRC 正确的数据才允许使能。上限位只阻止向上、下限位只阻止向下，允许反向离开；限位阻挡在变化率限制之后执行，避免减速残余速度继续顶限位。IO 失联/坏帧持续 0.5 秒、双限位同时触发、回零超时、电机反馈超时或模式/固件故障会停止并失能升降电机。故障后不会自动恢复运动。回零和目标高度使用同一硬件控制循环，动作目标不会在未回零时排队等待执行。

2026-10-24 真机只读检查：电机模式 3、失能、无故障，IO 持续输出有效帧，上限位触发、下限位未触发。没有执行真实回零或升降运动；真实导程、方向和高度精度尚未通过运动标定。虚拟串口覆盖回零、高度 Action、正反向回绕、限位方向阻挡、IO 超时/坏 CRC/双限位及回零超时。
