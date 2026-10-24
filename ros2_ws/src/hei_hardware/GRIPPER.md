# hei_hardware 夹爪控制器

`hei_hardware/GripperController` 是与 `DamiaoSystem` 同包的独立 Torque_Pos ros2_control 插件，仅提供标准 `control_msgs/action/GripperCommand`。所有串口 I/O 属于同侧的 DamiaoSystem；控制器占用 motor7 的 position、velocity、current_ratio、enabled 接口，与六轴机械臂控制器共享串口、独立占用资源。

到位后保持限流位置命令。取消或超时只请求失能 motor7，收到新的失能反馈后才结束 Action；硬件总线故障会停止同侧所有已占用电机。

## 启动

加载 Jazzy 与工程环境（机器人使用 `~/hei_jazzy_deps/install/setup.bash`），不能混用 Humble。单侧正式入口默认可写并激活机械臂和夹爪。只读检查需显式指定参数，夹爪控制器保持 inactive，拒绝运动目标：

```bash
ros2 launch hei_bringup damiao_arm.launch.py arm:=right read_only:=true
```

只调试夹爪时，先退出占用同侧串口的旧 launch，然后启动：

```bash
ros2 launch hei_bringup damiao_arm.launch.py arm:=right read_only:=false \
  arm_read_only:=true activate_gripper:=true gripper_position_only:=true
ros2 action send_goal /hei/right_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  '{command: {position: 0.002, max_effort: 0.0}}' --feedback
```

此预设自动激活夹爪控制器，六轴只导出状态接口、只查询反馈，允许六轴保持 MIT=1，不加载机械臂控制器。motor7 在失能反馈下设置 RAM Torque_Pos=4，收到合法目标后才使能。位置使用模型的名义米制总开口；示例为 2 mm。位置调试模式 `gripper_position_only:=true` 允许未标定力时使用 `max_effort: 0`，沿用配置电流比例 0.05，不代表零电流或已知夹持力；力反馈/结果为 NaN，不能按力判断 stalled，超时仍会停止。正数 max_effort 仍要求有效力标定。

`read_only:=true` 可将此预设改为完全只读，夹爪控制器保持 inactive；`activate_gripper:=false` 只关闭自动激活。此预设不提供六轴控制接口，要同时控制整臂和夹爪，需退出后使用下面的正式入口。

确认六轴模式、零位和限位后，退出只读/调试进程，启动完整控制入口：

```bash
ros2 launch hei_bringup damiao_arm.launch.py arm:=right read_only:=false
ros2 control list_controllers -c /hei/controller_manager
```

单侧机械臂控制器默认自动 active（`activate_arm:=false` 可关闭），夹爪控制器也默认 active，并使用位置模式。六轴要求 POS_VEL=2，默认 `configure_control_mode:=true` 在新鲜失能反馈下临时设置并回读模式，不保存 EEPROM；可显式 false 只检查已有模式。motor7 在新鲜失能反馈下设置/回读 RAM Torque_Pos=4，不保存 EEPROM、不改零位。六轴激活仍检查模式、反馈和故障状态；不因当前位置超出参考目标范围拒绝激活。夹爪目标位置限位和反馈力矩保护保留；反馈位置超过目标范围不再导致硬件故障或拒绝控制器激活。

现场确认后同时激活（机械臂控制器激活会使能六轴）：

```bash
ros2 control switch_controllers -c /hei/controller_manager --strict \
  --activate right_arm_controller right_gripper_controller
```

也可只激活夹爪控制器；收到有效目标前不使能 motor7。接口：

| Action | 类型 |
| --- | --- |
| `/hei/right_arm_controller/follow_joint_trajectory` | control_msgs/action/FollowJointTrajectory |
| `/hei/right_gripper_controller/gripper_cmd` | control_msgs/action/GripperCommand |

`arm:=left` 换用左侧串口和 left 控制器名。单侧夹爪调试使用前述参数并设置 `arm:=left`。`include_gripper:=false` 用于六轴独立诊断。双侧夹爪调试入口如下（六轴仍只读）。

## 双侧同时启动

`grippers.launch.py` 是双侧夹爪入口；旧的 `gripper.launch.py` 已删除。

先退出占用任一侧串口的单侧 launch，然后运行：

```bash
ros2 launch hei_bringup grippers.launch.py
```

查看两侧控制器：

```bash
ros2 control list_controllers -c /hei/left_controller_manager
ros2 control list_controllers -c /hei/right_controller_manager
```

默认两侧控制器自动 active、收到合法目标才使能夹爪。统一 namespace 为 `/hei`；左侧使用 `/dev/hei_left_arm` 和 `/hei/left_controller_manager`，右侧使用 `/dev/hei_right_arm` 和 `/hei/right_controller_manager`。两侧管理器、硬件实例、广播器和状态发布节点均使用不同名字，不共享串口。六轴均只读；速度 10 rad/s、电流比例 5%、反馈力矩阈值 3 N·m 沿用单侧配置。

```bash
ros2 action send_goal /hei/left_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  '{command: {position: 0.003, max_effort: 0.0}}' --feedback
ros2 action send_goal /hei/right_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  '{command: {position: 0.003, max_effort: 0.0}}' --feedback
```

`read_only:=true` 使双侧完全只读，夹爪控制器 inactive。可用 `left_config_file`、`right_config_file` 指定两侧硬件 YAML，`gripper_config_file` 指定共用控制器配置（须包含两侧条目）；统一 `namespace` 默认 `hei`。启动前拒绝非法 namespace 或指向同一串口的配置，包括符号链接别名。

状态话题和 TF 在统一 namespace 下用侧名前缀隔离，例如 `/hei/left_joint_states`、`/hei/right_joint_states`、`/hei/left_robot_description`、`/hei/right_robot_description`，以及 `/hei/left_tf`、`/hei/right_tf` 和对应的 `*_tf_static`；该调试入口未汇总为整机 TF。任一侧关键进程或启动配置失败会关闭本次双侧 launch，避免只启动一半；同一进程存活期间的硬件反馈故障仍按各自 controller_manager 处理。

## 标定与目标

完整力控制模式要求 `calibrated: true` 和有效的两项力换算系数，否则拒绝配置或运动目标。位置调试模式允许上述 max_effort=0 例外。配置为 `hei_control/config/damiao_gripper.yaml`；复制为本机配置、核对位置及力的换算后，通过 `gripper_config_file:=/absolute/path/gripper.local.yaml` 加载。

```text
开口[m] = gap_at_zero_m + gap_per_motor_rad_m × 电机角度[rad]
作用力[N] = |电机反馈力矩[N·m]| × force_per_motor_torque_n_nm
电流比例 = 目标最大作用力 / (force_per_motor_torque_n_nm × motor_torque_per_current_ratio_nm)
```

位置默认记录原模型的名义映射：`gap_at_zero_m=0`、`gap_per_motor_rad_m=-0.1/4.5`。来源为 hei-rebot-lift（b7c45e1）的 `hei_robot_vr_mujoco_real.py::simulated_gripper_to_real` 和 `hei_robot_vr_mujoco_sim.py::_set_gripper`：电机 0/-4.5 rad 对应单指 0/50 mm，两指反向移动，总开口变化为 0/100 mm。这不是重新实测标定。

现有电流比例和电机力矩保护阈值不等于夹持力 N；原项目的仿真推力也不是实机力标定。当前配置的两个力换算系数仍为 0，单独将 calibrated 改为 true 不构成有效标定；调试预设会将有效力标定状态判为 false，完整力控制模式会报告配置错误。非线性传动需扩展并验证转换模型，不能强套常数。

完成标定并显式配置 calibrated=true 后，以下命令表示目标总开口 20 mm、最大作用力 1 N：

```bash
ros2 action send_goal /hei/right_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  '{command: {position: 0.02, max_effort: 1.0}}' --feedback
```

max_effort=0 使用配置的默认电流；只有力标定有效时才能计算其对应作用力。忙碌、越限、非有限值、负作用力或超过电流上限的目标拒绝；不自动截断。单次只接受一个目标，不抢占。标准位置反馈和结果使用 m，力反馈使用 N（力标定缺失时为 NaN）；停止原因记录在控制器日志，Action 结果使用标准 reached_goal/stalled 字段。

速度、超时和保护由节点配置管理，重新配置/启动后生效：

- default_velocity_rad_s、max_velocity_rad_s：默认均为 10 rad/s，与 hei-rebot-lift 的协议速度值 1000 对齐（速度放大 100 倍编码）。
- default_current_ratio、max_current_ratio：默认均为 0.05（5%），不因超时自动提高电流。
- goal_timeout_sec：0 按行程/速度估算，至少 3 秒；正数显式指定动作期限。
- max_timeout_sec：默认 120 秒，自动估算或显式期限均不能超过它。
- position_tolerance_rad：默认 0.005；三个不同样本同时满足位置及静止速度条件才成功。
- stall_timeout_sec：达到请求最大作用力且停止运动持续超过该时间，返回 stalled=true、reached_goal=false 并停止，不能仅据此推断抓取成功。

硬件限位、电流与力矩保护在对应 `*_arm.yaml` 的 gripper 段；控制器配置不能越过硬件限位。原始电机状态作为 ros2_control GPIO，含 position(rad)、velocity(rad/s)、effort(N·m)、enabled、feedback_sequence，可从 `dynamic_joint_states` 查看；不创建虚构的米制指关节位移或 TF。

## 取消和停止

```bash
ros2 service call /hei/right_gripper_controller/gripper_cmd/_action/cancel_goal \
  action_msgs/srv/CancelGoal '{}'
ros2 control switch_controllers -c /hei/controller_manager --strict \
  --deactivate right_gripper_controller
```

停止客户端终端不会自动取消目标；成功后保持的夹爪需用第二条停用并释放。两条都不撤销机械臂接口；停用后需重新激活才能再发目标。通信故障时只能尽力失能，无法确认会明确记录，不能把发出帧当作实际停稳证明。

## 无硬件验收

```bash
cd ros2_ws
python3 -m pytest -q tests/test_shared_gripper.py
```

全部使用 PTY：标准 Action 的单位转换、目标校验、忙碌拒绝、机械臂/夹爪并行、保持、取消、超时、单独停用、未标定力目标拒绝、限流位置调试、六轴 MIT 只读与夹爪独立运动、只读无写入，以及共享硬件反馈故障停用两个控制器。测试用标定仅存在于临时配置，不部署到真实硬件。


## 电机反馈力矩保护

左右真实夹爪配置按 DM-J4310-2EC V1.1：额定输出力矩 3 N·m、峰值 7 N·m。本工程将额定值作为软件反馈停机阈值，在对应 `*_arm.yaml` 的 `gripper.torque_limit_nm: 3.0` 显式配置，并通过 Xacro 传给硬件插件；这不是厂家规定的统一停机阈值，也不代表已完成夹爪机构承载验收。型号/版本不同需使用对应参数。参考 [官方 V1.1 产品页](https://www.damiaokeji.com/index.php?c=show&id=84)。

原先隐藏的 0.5 N·m 缺省值与 1 N·m 配置上限已移除；缺少阈值会拒绝可写配置。阈值必须为正且不超过配置的力矩协议量程。`t_max: 10` 仍是协议量程，不能改为额定力矩；5% 电流限制与该软件阈值分别执行，均不表示夹持力的牛顿标定。反馈超出该阈值时仍按共享硬件故障停用控制器，日志记录实际值。

双臂和双夹爪一起使用 `ros2 launch hei_bringup damiao_arms.launch.py configure_control_mode:=true`，四个运动控制器默认自动 active；夹爪仍收到目标才使能。

左右夹爪共用相同策略：目标角度仍限制在配置范围内，手动扰动造成的反馈角度越界不停止控制器、不锁存硬件故障，也不阻止再次激活。到位后的限流保持行为不变；手动掰动不会自动取消保持目标。反馈力矩超限、通信超时、电机故障和模式变化仍走原有停止流程。
