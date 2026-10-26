# easeBot · HEI ReBot Lift ROS 2

机器人本地 ROS 2 软件栈与局域网服务器 Agent。ROS 包参考 ROBOTIS AI Worker 的边界组织，当前基于 ROS 2 Jazzy。正式入口 `robot.launch.py` 默认连接真机，启动全部执行器并完成升降回零；本地模拟使用 `mock_robot.launch.py`。

Web 继续只做状态监控。控制通过独立 ROS 2 service、topic 和 action 完成。

## 软件包

```text
ros2_ws/src/
  hei_description/      HEI URDF、30 个原始网格、TF 与 RViz 配置
  hei_control/          ros2_control 双臂、夹爪、升降与全向底盘控制器配置
  hei_hardware/         达妙硬件系统、夹爪与升降控制器插件
  hei_robot_manager/    控制器启停、反馈检查与 diagnostics
  hei_sensors/          三相机生命周期节点、图像与持续诊断
  hei_monitor/          Rust ROS 节点、状态缓存与 Axum 只读 Web
  hei_agent_bridge/     ROS 2 ↔ MHS/MOS，默认只读，可独立启用机器人 skill set
  hei_bringup/          统一 launch
```

这是原生 ROS 2 链路：`controller_manager → hardware_interface`，`joint_state_broadcaster → joint_states → robot_state_publisher → TF`。机器人侧 ROS 源码位于 `ros2_ws/src`。

服务器侧 Agent 独立于 ROS 工作区：

```text
agent/
  pyproject.toml       标准 Python 打包与命令入口
  src/hei_agent/       GaP 风格策略图、节点注册与异步执行内核
  tests/              不依赖 ROS 的执行器测试
```

物理 Agent 的任务编排见 [hei_agent](agent/README.md)：已提供人工定义的 graph、
node、结果分支、超时、取消与 JSONL trace，以及 OpenAI 兼容 LLM 的自然语言图生成和校验修正。
默认运行内存 mock；MHS 模式可发现并调用机器人发布的状态、命名位姿与夹爪开度技能。
服务器侧用 pip 安装，无需 ROS 或 colcon；机器人侧的
`hei_agent_bridge` 保持 ROS 包结构。

```bash
python3 -m venv agent/.venv
agent/.venv/bin/python -m pip install -e ./agent
agent/.venv/bin/hei-agent
```

LLM 模式安装 `pip install -e './agent[llm]'`，配置 `agent/llm.env.example` 中的模型地址、
模型名和 API Key 后，可执行
`agent/.venv/bin/hei-agent --task '右臂先到预抓取位再到搬运位'`。
`--plan-only` 仅生成图；详细配置和生成记录见 [Agent 文档](agent/README.md#接入-llm)。

机器人侧技能入口为 `ros2 launch hei_bringup agent_skills.launch.py`，默认仅模拟技能；
服务器先 `bash scripts/setup.bash --agent-server`，再用
`agent/.venv/bin/hei-agent --backend embedded --serve --hub-id lab-agent` 常驻运行；
机器人加 `hub_id:=lab-agent` 即可自动发现和注册，无需配置服务器 IP 或启动 Avahi。
服务器终端输入 `{"list_skills":true}` 查看目录，`{"task":"任务描述"}` 提交任务。
真实 ROS 后端、示教配置与停止语义见 [MVP skill set](ros2_ws/src/hei_agent_bridge/README.md#mvp-skill-set)。

## 构建与启动

多机开发可使用 [SSH / rsync 源码同步与远端编译](docs/remote-dev.zh-CN.md)：本机编辑，推送到机器人或服务器的专用开发目录，在目标机器构建，不自动重启节点。

脚本的用途、目录和执行位置见[脚本说明](scripts/README.md)。日常环境统一使用 `source scripts/env/workspace.bash` 加载。

本机已具备 Jazzy。需要已有 Rust/C++ 工具链及 ROS Rust 消息绑定；统一入口会准备 Rust 构建插件和各模块依赖。详见 [脚本说明](scripts/README.md)。

```bash
cd /home/cw/gitdir/easeBot
source /opt/ros/jazzy/setup.bash
bash scripts/setup.bash  # 首次构建或依赖变更后；默认包含 Agent Bridge
source scripts/env/workspace.bash
# 新机器可使用 rosdep 检查并安装 package.xml 中的依赖
# rosdep install --from-paths ros2_ws/src --ignore-src -r -y
colcon --log-base ros2_ws/log build --base-paths ros2_ws/src \
  --build-base ros2_ws/build --install-base ros2_ws/install
source ros2_ws/install/setup.bash

# 本机模拟开发时限制发现范围，domain 需与控制终端保持一致
export ROS_DOMAIN_ID=71
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch hei_bringup mock_robot.launch.py
```

只读页面：[http://127.0.0.1:8081](http://127.0.0.1:8081)。默认没有 HTTP 身份认证或 TLS，绑定回环地址；跨机器访问可用 SSH 转发。`Ctrl+C` 关闭本次 launch 的节点。

可选参数：

```bash
ros2 launch hei_bringup mock_robot.launch.py rviz:=true
ros2 launch hei_bringup mock_robot.launch.py monitor:=false
ros2 launch hei_bringup mock_robot.launch.py namespace:=hei monitor_port:=8081
ros2 launch hei_bringup mock_robot.launch.py camera_backend:=opencv
```

`opencv` 只切换相机，电机仍为 mock。当前默认设备为 `/dev/hei_front_camera`、`/dev/hei_left_wrist_camera`、`/dev/hei_right_wrist_camera`，可用相机节点参数配置。相机不能同时被旧 HEI host 打开。相机内外参未验证，因此当前不发布虚构的已标定 CameraInfo。

`hei_sensors` 是三路共享状态的 LifecycleNode。默认 bringup 自动 configure/activate；手动停用后不会自动重启采集。停止、恢复和重新配置方式：

```bash
ros2 lifecycle get /hei/cameras
ros2 lifecycle set /hei/cameras deactivate
ros2 lifecycle set /hei/cameras activate
# 需要关闭设备并修改配置时：deactivate → cleanup → 修改参数 → configure → activate
```

图像仅 active 时发布，诊断在 inactive/unconfigured 时仍可用；monitor 会将停止更新的图像标记为过期。OpenCV configure 任一路设备打开失败时回滚全部设备，停留在 unconfigured。详情见 [相机生命周期说明](ros2_ws/src/hei_sensors/README.md)。

RViz 配置默认使用 `/hei/robot_description`；自定义 namespace 时需在 RViz 中选择相应 description topic。没有定位系统，默认不发布 `map → odom` 或 `odom → base_footprint`。

## 真机统一启动

加载 Jazzy 和工程安装环境后，一条命令启动双臂、双夹爪、底盘和升降：

```bash
ros2 launch hei_bringup robot.launch.py
```

默认可写、自动配置并回读 RAM 控制模式、激活全部运动控制器。机械臂以实测位置初始化保持；底盘初始零速度；夹爪沿用电流比例 0.05，收到合法目标才使能；升降自动向上回零，完成后激活高度控制器。因此启动会使能机械臂和底盘，并可能移动升降。模式配置不保存 EEPROM、不修改电机零位。

统一命名空间为 `/hei`，四个管理器为 `left_controller_manager`、`right_controller_manager`、`base_controller_manager`、`lift_controller_manager`。所有控制器就绪后输出 `Robot actuators ready`；任一关键进程退出、控制器启动失败或整体等待超过 90 秒，本次 launch 统一退出。此检查只负责启动，不自动恢复运行中的硬件故障。monitor 独立运行，仅做只读状态查询，退出不会停掉执行器。各总线的状态和 TF 保留原有前缀。

```bash
# 整机只读：不使能、不配置模式、不回零
ros2 launch hei_bringup robot.launch.py read_only:=true
# 硬件 YAML 路径可分别覆盖，其余使用 hei_bringup/config 下的配置
ros2 launch hei_bringup robot.launch.py left_config_file:=/absolute/path/left_arm.yaml
```

启动前检查四条电机总线和升降 IO 是否复用了同一设备（含符号链接）。不要同时运行占用相同总线的单独调试入口。真机入口整合执行器；Web monitor 默认随真机启动（0.0.0.0:8081，局域网访问 `http://seeed.local:8081/`），直接读取四条总线与管理器状态；三相机也默认以 OpenCV/V4L2 配置激活（可用 `cameras:=false` 禁用）；Agent Bridge 的原模拟参数保留在 `mock_robot.launch.py`，未将仅支持 mock 的 robot_manager 用于真机。

## 升降

独立升降入口默认可写，自动配置 VEL 模式、回零并激活高度控制器：

```bash
ros2 launch hei_bringup damiao_lift.launch.py
```

默认回上限位建立软件高度零点，成功后自动激活 `/hei/lift_controller/follow_joint_trajectory`。高度单位米，上端 0、向下为负；管理器 `/hei/lift_controller_manager`，可与双臂及底盘入口并行。配置、限位策略及 Action 命令见 [升降说明](ros2_ws/src/hei_hardware/LIFT.md)。

## 达妙底盘

底盘四轮复用 `DamiaoSystem` 的 VEL 接口与标准全向轮控制器，默认可写并自动激活：

```bash
ros2 launch hei_bringup damiao_base.launch.py
```

`read_only:=false` 时默认自动激活四轮；命令话题 `/hei/base_controller/cmd_vel` 使用带当前时间戳的 `TwistStamped`。管理器为 `/hei/base_controller_manager`，可与双臂入口并行运行。接线映射、启动及低速命令见 [底盘说明](ros2_ws/src/hei_hardware/BASE.md)。

## 机械臂与夹爪共享总线

同侧七个电机由一个 `hei_hardware/DamiaoSystem` 管理串口，机械臂和夹爪使用独立控制器，可同时工作。默认可写并激活机械臂和夹爪控制器：

```bash
ros2 launch hei_bringup damiao_arm.launch.py arm:=right
```

双臂统一入口（默认可写、自动激活双臂和夹爪控制器）：

```bash
ros2 launch hei_bringup damiao_arms.launch.py configure_control_mode:=true
```

默认 `activate_arm:=true`：双臂六轴自动使能并以实测位置初始化保持目标；`activate_arm:=false` 只配置机械臂控制器，`read_only:=true` 完全只读且不加载机械臂控制器。两侧管理器为 `/hei/left_controller_manager`、`/hei/right_controller_manager`；轨迹 Action 为 `/hei/left_arm_controller/follow_joint_trajectory`、`/hei/right_arm_controller/follow_joint_trajectory`。夹爪控制器自动激活，收到有效目标才使能 motor7，默认位置调试 `max_effort: 0`。可传 `include_gripper:=false` 仅启动双臂。

`configure_control_mode` 默认 true，在失能状态设置并回读 RAM POS_VEL，不保存 EEPROM。单侧入口同样默认可写、自动激活机械臂与夹爪，使用未标定力的位置模式。所有真机入口可显式使用 `read_only:=true` 检查反馈。

| 接口 | 类型 |
| --- | --- |
| `/hei/right_arm_controller/follow_joint_trajectory` | control_msgs/action/FollowJointTrajectory |
| `/hei/right_gripper_controller/gripper_cmd` | control_msgs/action/GripperCommand |

双侧夹爪同时调试运行 `ros2 launch hei_bringup grippers.launch.py`，左右 Action 分别在 `/hei/left_gripper_controller/gripper_cmd` 和 `/hei/right_gripper_controller/gripper_cmd`。统一 namespace 为 `/hei`，管理器分别命名为 `left_controller_manager` 和 `right_controller_manager`；自动激活两侧夹爪控制器，六轴保持只读。单侧调试使用 `damiao_arm.launch.py`，详见夹爪说明。默认位置调试只接受 `max_effort: 0`（沿用电流比例 0.05），未标定力时反馈为 NaN；正数牛顿目标仍需力标定。标准夹爪接口使用 m/N；内部电机限位仍为 [-4.5,0] rad。到位后保持，取消/超时只失能夹爪；同侧硬件故障会停止两个控制器。只读入口也加载 inactive 的夹爪 Action Server。真实夹爪反馈力矩阈值在硬件 YAML 中显式配置为 3 N·m，取 DM-J4310-2EC V1.1 的额定值；协议量程仍为 10 N·m。完整命令、标定与测试见 [夹爪控制器](ros2_ws/src/hei_hardware/GRIPPER.md)。

Rust motorbridge 的串口协议适配说明见 [hei_hardware](ros2_ws/src/hei_hardware/README.md)。`mock_robot.launch.py` 提供独立的整机 mock。夹爪统一使用共享控制器的标准 `gripper_cmd` Action。

## Agnes Agent 接入

可选的 `hei_agent_bridge` 默认仅提供机器人状态查询和三路 JPEG 观测；独立的 skill 入口可显式发布和执行技能，不改变 Web 只读约定。统一入口 `bash scripts/setup.bash` 默认准备机器人 SDK 依赖，启动终端使用 `source scripts/env/workspace.bash`。

```bash
# 默认自动发现 Hub；此入口仅提供观测
ros2 launch hei_bringup mock_robot.launch.py agent_bridge:=true \
  agent_hub_id:=lab-agent
# 或在已经运行的机器人栈旁单独启动
ros2 run hei_agent_bridge agent_bridge --ros-args -p hub_id:=lab-agent
```

默认不启用。设备 ID 默认 hei-01，ROS namespace 默认 /hei。它作为逻辑只读 observer 注册；MHS stop 不停止外部 ROS 客户端的运动。安装、数据含义与验证见 [bridge 说明](ros2_ws/src/hei_agent_bridge/README.md)。

## 独立 ROS 2 控制

新终端同样 source ROS 环境与 workspace，并设置相同 domain。初始仅状态广播器 active，六个运动控制器均为 inactive。

```bash
ros2 control list_controllers -c /hei/controller_manager

# 显式激活 mock 运动控制器，不等同于真实电机使能
ros2 service call /hei/robot_manager/set_enabled std_srvs/srv/SetBool '{data: true}'

# 使用标准 FollowJointTrajectory action，右臂小幅目标，持续 2 秒
ros2 run hei_robot_manager joint_demo

# 停用运动控制器；不是物理急停或真机停稳证明
ros2 service call /hei/robot_manager/stop std_srvs/srv/Trigger '{}'
```

默认整机 mock 中，双臂、两只夹爪和升降均提供 `/<namespace>/<group>_controller/follow_joint_trajectory`。双臂每侧六关节；夹爪以主指的线位移 m 表示，另一指 mimic；升降 m，上限位参考 0、向下为负。它们与旧版夹爪电机 rad、升降 mm 的 LeRobot 接口不同，需要显式转换，不能直接对接旧动作数组。

底盘使用 `omni_wheel_drive_controller`，订阅 `/hei/base_controller/cmd_vel` (`geometry_msgs/msg/TwistStamped`)，发布 `/hei/base_controller/odom`。其半径、轮径和符号是 **mock 联调参数，未经实机标定**；没有真实速度能力承诺。指令超时 0.3 秒后标准控制器归零。

ROS 原生 action 会执行其有限轨迹，即使发起端退出也不自动采用旧版 IPC 的 150 ms 动作租约。当前没有统一客户端租约和来源鉴权；同一 domain 视为受信任开发环境，原生控制器接口也可以绕过 manager 调用。上机前需要补硬件停止链路、会话仲裁和访问策略。

## 只读监控与标准话题

- `/hei/joint_states`：21 个可动关节反馈，含两只被动 mimic 指。
- `/tf`、`/tf_static`：机器人描述对应的坐标树。
- `/hei/cameras/{front,left_wrist,right_wrist}/image_raw`：标准 Image，有订阅者时发送。
- `/hei/cameras/{name}/image_raw/compressed`：JPEG CompressedImage。
- `/hei/diagnostics`：控制器、反馈新鲜度和相机状态。
- `GET /api/v1/status` 与 `GET /api/v1/cameras/{name}`：Web 的只读接口。

`hei_monitor` 是 `ament_cargo` Rust 包，使用 `rclrs` 创建原生 ROS 节点，Axum 提供只读 HTTP。节点名为 `/hei/monitor`，ROS executor 与 HTTP 在同一进程中，通过线程安全缓存共享数据，静态页面嵌入二进制。可直接检查：

```bash
ros2 node info /hei/monitor
# 也可独立启动（确保同一 namespace/domain，且没有已启动的 monitor 占用端口）
ros2 run hei_monitor monitor --ros-args -r __ns:=/hei -p port:=8081
```

Web 节点仅创建订阅者，不创建控制 publisher、service client 或 action client；写 HTTP 方法返回 405。回零和急停反馈目前为未知，页面不将 mock 初始化视为真实回零。

## 验证

在 source 了 Jazzy、`scripts/env/ros-rust.bash` 和本 workspace 的终端运行：

```bash
cargo test --locked --manifest-path ros2_ws/src/hei_monitor/Cargo.toml \
  --target-dir ros2_ws/build/hei_monitor
python3 -m pytest -q ros2_ws/tests
```

已在本机完成六包 colcon 构建、3 个 Rust 缓存单元测试、3 个相机生命周期/资源测试、2 个模型测试和 1 个 ROS 2 端到端测试；Rust monitor 另验证了端口冲突失败与 SIGTERM 正常退出。端到端覆盖：控制器默认 inactive、未启用时拒绝轨迹、显式启用后 action 成功、joint_states/TF/图像发布、底盘超时归零、停止后 inactive、Web 写方法拒绝、相机默认激活及手动停用/恢复/清理后重配。测试启动独立 ROS domain，结束时关闭节点。不是实机硬件测试，也未验证实时调度。

模型来源和 mock 限值修改见 [模型来源记录](/home/cw/gitdir/easeBot/ros2_ws/src/hei_description/SOURCE.md)，上游许可随包保留。原始几何与运动学来自 HEI 仓库；零速度/力限位被替换为用于 mock 的临时值，不能作为实机额定值。

达妙 U2CAN 已通过 `hei_hardware` 接入 motorbridge，下一阶段验证真实反馈、校准与单关节控制。管理节点分别使用关节反馈超时（默认 1 秒）和控制器查询超时（默认 5 秒），可用 ROS 参数调整。控制器查询耗时不会被误当作电机关节反馈中断。

本次不创建不可用的硬件占位插件；没有连接真机的 launch 参数。MoveIt、导航和 VR/LeRobot 接入仍为后续模块。

[ROS 2 架构与范围](/home/cw/gitdir/easeBot/docs/ros2-architecture.zh-CN.md) · [监控节点说明](/home/cw/gitdir/easeBot/ros2_ws/src/hei_monitor/README.md)

2026-10-24 真机监控只读验证：在独立 ROS domain 中临时启动四条总线的只读反馈与 monitor，HTTP 确认 19 个执行器位置持续更新、四条总线在线、夹爪失能反馈、升降 IO 在线及未回零状态；状态接口拒绝 POST（405）。验证后已退出临时进程，没有配置电机模式、使能或执行回零。记录位于机器人 `ros2_ws/log/monitor-hardware-physical-readonly.{log,json}`；夹爪开口仍使用控制器配置的名义映射，不构成实测标定。
