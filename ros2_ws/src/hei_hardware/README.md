# hei_hardware

`hei_hardware/DamiaoSystem` 是 C++ `ros2_control::SystemInterface` 插件，底层通信使用固定版本的 Rust motorbridge，基于 Jazzy/ros2_control 4.44.0 API，连接 HEI 原仓库使用的 USB 转 CAN 串口适配器。默认 921600、8N1；不是 SocketCAN、SLCAN，也不适配所有型号的达妙 USB 设备。

本版支持 DM4310、DM4340：读取位置/速度/力矩，提供固定的 POS_VEL 位置接口或 VEL 速度接口。ros2_control 插件已支持同侧 motor7 Torque_Pos，供 [夹爪控制器](GRIPPER.md) 与六轴并行使用；升降限位/回零和 MIT 力矩控制尚未接入。另保留受限的独立电机诊断工具。默认整机 `robot.launch.py` 仍使用 mock。

## 包内两个插件

- `hei_hardware/DamiaoSystem`：hardware_interface::SystemInterface，统一管理同侧七个电机和串口。
- `hei_hardware/GripperController`：controller_interface::ControllerInterface，处理标准夹爪 Action，通过资源接口发命令，不直接访问串口。使用说明见 [GRIPPER.md](GRIPPER.md)。

两个插件独立编译、注册到各自的 pluginlib 基类，共同维护在此包；Action 地址保持不变。

## 分层与依赖

```text
ros2_control → hei_hardware (C++ SystemInterface)
            → libhei_motorbridge_backend.so (Rust 私有 C ABI)
            → motorbridge motor_core::DmSerialBus + motor_vendor_damiao::protocol
            → USB-CAN → 电机
```

motorbridge 固定到 `d5700bcd5b7f9355dd2012488d463c3bac982259`，传递依赖由 `motorbridge_backend/Cargo.lock` 锁定。没有复制或修改上游驱动源码，也不依赖 Python SDK、JSON topic bridge 或 Web gateway。C++ 不再实现串口、帧编码与反馈解码。

本地薄适配负责有界手动接收、每帧故障/模式异常锁存、反馈时间戳与无副作用关闭；C++ 负责 ROS 生命周期、关节转换、命令限位和控制器占用。暂不使用上游 C++ Controller 的自动 shutdown，因为其析构会请求失能所有已注册电机；也不使用只保留最新值的通用 ABI 状态缓存，以免遗漏一个批次中的瞬时故障。

## 构建与无硬件测试

```bash
source /opt/ros/jazzy/setup.bash  # 机器人使用 ~/hei_jazzy_deps/install/setup.bash
source scripts/env/workspace.bash
bash scripts/setup.bash  # 统一准备构建插件和模块依赖
colcon --log-base ros2_ws/log build --base-paths ros2_ws/src \
  --build-base ros2_ws/build --install-base ros2_ws/install \
  --packages-up-to hei_hardware hei_description hei_bringup
source ros2_ws/install/setup.bash
colcon --log-base ros2_ws/log test --build-base ros2_ws/build \
  --install-base ros2_ws/install --packages-select hei_hardware \
  --event-handlers console_direct+
```

CMake 自动构建 Rust 后端并与插件一起安装到 `lib/`，使用 `$ORIGIN` 查找相邻共享库。colcon 构建使用 `cargo --offline --locked`，不隐式联网更新依赖；首次准备脚本需要网络。跨机器同步源码后，在目标机独立执行准备和编译，不复制 x86/ARM 二进制。

C++ 测试通过 Linux PTY 模拟适配器和电机，检查真实串口读写、插件加载、只读生命周期、模式拒绝、使能与失能顺序、命令校验和反馈超时，不打开真实电机设备。

2026-10-23 motorbridge 迁移验证：本机与 ARM64 机器人均通过 10 项驱动测试，故障批次用例各连续运行 50 次通过；机器人 Jazzy 9 个包构建成功，Python 全量回归 23 passed、1 skipped（47.44 秒，真实相机默认跳过）。六轴 PTY 测试通过只读启动和标准 FollowJointTrajectory 执行/停用。本机另验证共享库搬迁加载及独立 CMake 消费者构建。日志见 `ros2_ws/log/motorbridge-*.log` 与 `motorbridge-project-tests.xml`。随后进行了右臂真机只读检查，见下一节；尚未验证真实运动。

## 右臂首次真机只读检查（2026-10-23）

`/dev/hei_right_arm → ttyACM0`，未发现原驱动占用。通过真实 `controller_manager` 与 JointState broadcaster 取得六轴 20 帧反馈；18 个状态接口、0 个命令接口。本次进程已退出，未调用使能、失能或运动命令。

额外只读查询 CTRL_MODE：六个电机均为 MIT=1、失能=0，无故障。当前插件的 position 接口要求 POS_VEL=2，因此暂不能直接启用轨迹控制器。参考原 HEI 驱动在连接时会切换六轴到 POS_VEL；本次没有修改任何固件参数。

参考方向/零偏下关节位置依次约为 `[-0.00477, 0.01202, -0.00973, 0.01888, -0.03986, -0.05436] rad`。joint2 高于参考目标上限 0。参考范围仅约束新目标，不再用于拒绝六轴激活；激活时先保持实测位置，不在硬件插件中将保持目标裁剪到边界。允许从该启动位置插值返回参考范围，不允许继续向范围外扩展；首次收到范围内目标后，启动例外失效。Damiao 启动入口关闭 controller_manager 的通用命令限位器，避免它依据 URDF 的参考范围拒绝当前实测位置；命令范围仍由硬件适配器检查。这些读数不构成机械零位或方向的校准证明。

机器人记录：`ros2_ws/log/physical-right-arm-readonly.{log,json}`、`physical-right-arm-firmware-readonly.json`；本地副本在 `ros2_ws/log/robot-jazzy/`。

## 右夹爪电机级小幅测试（2026-10-23）

使用同一串口上的 motor7 / feedback 0x17 / DM4310。专用 Rust example 直接调用锁定的 motorbridge，默认只读；显式 `--move` 才执行固定测试。它不是 ROS 节点或 Action Server，也没有调用原 HEI host 的整臂 connect/enable_all。

本次真机参数：临时模式 Torque_Pos=4，电机侧相对目标 -0.02 rad，用时约 2 秒，速度上限 0.05 rad/s，电流比例 0.01（最大电流的 1%，不等同于已标定夹持力）。起点 -0.000572 rad，目标 -0.020572 rad，运动末端实测 -0.012016 rad，位移 -0.011444 rad，**未达到目标容差，程序返回失败并停止，未自动提高电流重试**。失能后读数约 -0.010490 rad。

测试结束 motor7 已失能、模式恢复 MIT=1，前六轴始终未收到使能/运动/参数写入，测试前后六轴位置读数相同且均失能。没有写零位或保存 EEPROM。日志位于机器人 `ros2_ws/log/physical-right-gripper-{readonly.json,motion.log,after.log}`，本地副本在 `ros2_ws/log/robot-jazzy/`。

复现工具（在编译机器运行，已完成统一 setup）：

```bash
repo_root="$PWD"
(cd /tmp && cargo build --offline --locked --release \
  --manifest-path "$repo_root/ros2_ws/src/hei_hardware/motorbridge_backend/Cargo.toml" \
  --target-dir "$repo_root/ros2_ws/build/hei_hardware/motorbridge-target" \
  --example gripper_probe)
# 默认只读，检查七个电机的状态和夹爪模式
ros2_ws/build/hei_hardware/motorbridge-target/release/examples/gripper_probe
# 此命令会使能右夹爪并执行上述固定小幅测试，现场确认后使用
ros2_ws/build/hei_hardware/motorbridge-target/release/examples/gripper_probe --move
```

发送帧经过白名单：前六轴仅允许查询；仅 motor7 可使用使能/失能、FORCE_POS，以及临时 CTRL_MODE 切换。必须七电机初始失能且夹爪目标在参考电机区间 [-4.5,0] rad 内才可启动。有限行程/力矩反馈保护触发后停止；SIGINT/SIGTERM 经正常清理失能夹爪并恢复模式，SIGKILL/断电不具备此保证。

夹爪现统一通过 [夹爪控制器](GRIPPER.md) 的标准 GripperCommand Action 控制；内部使用电机弧度和电流比例。现有模型开度映射为名义参考，本次电机位移不构成实测开口或夹持力标定。

## 单侧机械臂只读入口

以下命令会打开真实串口并发送状态查询，运行前确认旧版 host 已退出。驱动独占设备，失败时不改用 mock，也不把缺失反馈填成零。

同一 namespace 的 mock bringup 也应先退出，避免两个 controller_manager 和不同来源的关节状态混在一起。

串口独占行为由 motorbridge 使用的 serialport 库实现，本工程用 PTY 验证重复打开被拒绝。仍需先退出原驱动和串口调试工具。

```bash
ros2 launch hei_bringup damiao_arm.launch.py arm:=right
# 左臂
ros2 launch hei_bringup damiao_arm.launch.py arm:=left
ros2 topic echo /hei/joint_states
```

默认 `read_only:=true`：URDF 不导出命令接口，加载 inactive 的夹爪控制器以提供可发现的 Action，不加载机械臂控制器，只轮询六个机械臂关节和夹爪 motor7，不发送使能、失能、写零位、模式切换或参数写入。不会启动相机、其他电机或 mock manager。可设置 `monitor:=true` 启动只读页面；此独立入口没有 manager 诊断，监控中的整机模式保持 unknown。

设备和参考映射见 `hei_bringup/config/right_arm.yaml`、`left_arm.yaml`。使用自己的配置：

```bash
ros2 launch hei_bringup damiao_arm.launch.py arm:=right \
  config_file:=/absolute/path/right_arm.local.yaml
```

直接运行 `hei_damiao_arm.urdf.xacro` 时须显式传入 `config_file`；launch 会自动选择参考配置，description 包不反向依赖 bringup。

每个系统只对应一个独立串口/总线；两侧机械臂使用不同设备，因此可以重复使用各自的 CAN ID。配置包含对应侧六个关节和独立 gripper 段。默认七电机共享串口，不包括底盘和升降；include_gripper:=false 可用于六轴诊断。夹爪 GPIO 导出 5 个状态与 4 个命令接口；整侧共 23 个状态、10 个命令接口，只读时无命令接口。`grippers.launch.py` 在统一 `/hei` namespace 下启动双侧调试，每侧只导出夹爪的 4 个命令接口，六轴保持只读（允许 MIT 模式），夹爪控制器自动 active、收到目标才使能；未标定力时只接受 max_effort=0 的限流位置目标，力反馈 NaN。详见 [夹爪启动与命令](GRIPPER.md)。

## 显式启用控制接口

先在只读模式核对电机型号、反馈范围、关节方向、零位、传动比及实际位置。六轴命令要求 POS_VEL=2、VEL=3；默认只检查 CTRL_MODE。可显式设置 `configure_control_mode:=true`，在所有可写电机均有新鲜、失能、无故障反馈及已知模式时，临时设置并回读 RAM CTRL_MODE；不保存 EEPROM、不改零位、不使能电机。只读及 state-only 电机不写模式，运行中模式改变仍触发故障；夹爪在失能反馈下设置/回读 RAM Torque_Pos=4，不保存 EEPROM、不改零位。写模式要求电机最初为失能状态，拒绝接管已使能的电机。

```bash
ros2 launch hei_bringup damiao_arm.launch.py arm:=right read_only:=false \
  config_file:=/absolute/path/right_arm.local.yaml
```

如果启动日志出现 `CTRL_MODE mismatch`，且六轴已失能，可先用六轴配置入口：

```bash
ros2 launch hei_bringup damiao_arm.launch.py arm:=right read_only:=false \
  include_gripper:=false configure_control_mode:=true
```

该选项默认关闭，不会为已使能或故障电机清错/强制失能。模式回读正确后启动完成，轨迹控制器默认自动 active（`activate_arm:=false` 可关闭）；六轴当前位置超出参考目标范围不再阻止激活。激活控制器会使能六轴，需先核对零位/限位并支撑机械臂。

单侧可写入口默认自动激活 `right_arm_controller`，`right_gripper_controller` 默认 inactive；传 `activate_arm:=false` 可保留手动激活流程。硬件 activate 本身不发使能，机械臂控制器激活时使能六轴。手动启用命令：

```bash
ros2 control switch_controllers -c /hei/controller_manager \
  --activate right_arm_controller right_gripper_controller --strict
# 分别停用控制器只失能其占用的电机；可单独停用夹爪
ros2 control switch_controllers -c /hei/controller_manager \
  --deactivate right_gripper_controller --strict
```

控制器占用时，先在失能状态下设置当前位置/零速度目标，再发使能；收到已使能反馈后才发送后续命令。位置使用弧度，速度使用弧度/秒，力矩使用 N·m。转换为：

```text
joint_position = (motor_position - offset) * direction / gear_ratio
motor_target   = joint_target * direction * gear_ratio + offset
```

`direction` 只能为 ±1，`offset` 是电机侧弧度，`gear_ratio` 为电机转角/关节转角。力矩按理想传动比换算，不含效率补偿。`velocity_limit` 是关节侧速度上限；参考机械臂文件暂设 0.1 rad/s，不代表已经验证过该速度或停止距离。

插件拒绝 NaN、越限、过期反馈和错误模式，不静默裁剪命令。固件故障、反馈超时或传输失败会锁存错误并尽力失能已占用电机，需重新 configure 才能恢复。只读模式始终不会执行失能。

## 限制

- 已完成模拟控制、右臂真机只读和夹爪电机级低电流测试；没有验证六轴机械臂真实运动、承载制动或硬件急停，共享总线控制器只在 PTY 验证，未执行新的真实动作；用户此前反馈的独立夹爪动作在低电流下超时，不能据此认定真机控制已完成验收。
- 上游串口采用互斥锁和 10 ms I/O 超时；本版不启动后台接收线程，每次读取最多处理 256 个 CAN 帧。六轴仍需实测周期耗时，不能据此声称硬实时。传输错误后关闭连接，不再向可能残缺的字节流追加失能帧；因此断连时停止仍依赖固件及硬件措施。
- 控制循环暂停后不会用积压反馈刷新有效期。进程被 SIGKILL、主机失电或 USB 断开时，软件无法保证失能帧送达；固件看门狗、机械支撑与硬件急停须独立验证。
- 停用采用失能，会释放保持力矩，不能用于未验证的承载升降/机械臂停止策略。
- 当前反馈位置为电机协议给出的有限区间值；尚未提供连续轮轴多圈展开，不能直接作为已验证的底盘里程计。

协议和参考配置来源见 [SOURCE.md](SOURCE.md)。


2026-10-23 右臂启动复核：显式开启 `configure_control_mode` 后，六轴在失能状态下完成 RAM MIT=1 → POS_VEL=2 的配置与回读。实机启动到 `right_arm_controller: inactive` 成功，验证进程已退出；未激活六轴或发送轨迹。退出后只读确认六轴均模式 2、失能 0、无故障，未保存 EEPROM。joint2 的 +0.012016 rad 与参考上限 0 的差异仍待核对，不能将启动成功视为运动验收。


## 双臂统一启动

```bash
ros2 launch hei_bringup damiao_arms.launch.py configure_control_mode:=true
```

默认可写，双臂控制器与夹爪控制器自动 active；六轴使能并初始化当前位置保持，夹爪收到目标才使能。`activate_arm:=false` 将两侧机械臂控制器留在 inactive；`read_only:=true` 不加载机械臂控制器且禁止全部电机写操作；`include_gripper:=false` 不加载夹爪。`configure_control_mode` 默认 false，可显式 true 完成失能电机的 RAM 模式配置。

管理器分别为 `/hei/left_controller_manager` 和 `/hei/right_controller_manager`，机械臂轨迹 Action 分别为 `/hei/left_arm_controller/follow_joint_trajectory` 和 `/hei/right_arm_controller/follow_joint_trajectory`。左右状态及 TF 树按 `left_`、`right_` 前缀隔离，与双侧夹爪调试入口一致；此入口不合并全机器人 TF。两侧 YAML 可通过 `left_config_file`、`right_config_file` 指定，重复串口在启动前拒绝。`position_only:=true` 默认沿用夹爪电流比例 0.05，标准夹爪目标使用 `max_effort: 0`；使用力标定时传 `position_only:=false`。
