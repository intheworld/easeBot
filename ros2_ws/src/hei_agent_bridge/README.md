# hei_agent_bridge

原生 rclpy 节点 `/hei/agent_bridge`，同时使用官方 `agnes_mhs` Python SDK 连接 AgnesHub。第一版为**只读观测适配器**：注册设备 `hei-01`，上报 ROS 状态、提供一个只读查询工具及三路 JPEG 观测。没有运动工具、控制 publisher、业务 service client 或 action client；`hei_monitor` 保持独立只读。

## 安装与构建

Agnes SDK 当前从源码安装，不假定存在同名 PyPI 发布。仓库提供脚本，固定上游 commit（见 sdk-revision.txt）和 Python 运行依赖（requirements-mhs.lock），使用被 git 忽略的 `.deps/`，不改系统 Python：

```bash
cd /home/cw/gitdir/easeBot
source /opt/ros/jazzy/setup.bash
bash scripts/setup.bash  # 统一准备，默认包含 Agent SDK
source scripts/env/workspace.bash
colcon --log-base ros2_ws/log build --base-paths ros2_ws/src \
  --build-base ros2_ws/build --install-base ros2_ws/install \
  --packages-select hei_agent_bridge hei_bringup
source ros2_ws/install/setup.bash
```

SDK 依赖 checkout 中的共享 schema，所以保留 `.deps/agnes-harness`，不能只复制 Python 包目录。每个启动 bridge 的终端都需 source `scripts/env/agent-bridge.bash`。部署环境也可自行在 ROS 节点所用 Python 解释器中安装对应版本 SDK。

运行依赖锁同时支持 Python 3.10 和 3.11+；Python 3.10 使用 `rpds-py 0.30.0`、`websockets 15.0.1`。SDK 源码通过环境脚本加载；依赖按目标机器的 Python 版本和架构安装，不复制开发机的二进制 wheel。

## 启动

先按 Agnes 官方文档启用 `@agnes/mhs` 插件和 AgnesHub。地址是 Hub 的 WebSocket **基础地址**，不是 Web UI 地址，也不是 `/ws/mhs` 路径。

```bash
# 已运行机器人 bringup 时，另一个终端单独运行 bridge
source /opt/ros/jazzy/setup.bash
source scripts/env/agent-bridge.bash
source ros2_ws/install/setup.bash
# ROS_DOMAIN_ID、发现范围与机器人一致
ros2 run hei_agent_bridge agent_bridge \
  --ros-args -p hub_url:=ws://127.0.0.1:4180 -p device_id:=hei-01
```

也可随整套 mock 启动（先按主 README 准备 Rust monitor 环境）：

```bash
ros2 launch hei_bringup mock_robot.launch.py agent_bridge:=true \
  agent_hub_url:=ws://127.0.0.1:4180 agent_device_id:=hei-01
```

默认 `agent_bridge:=false`，没有 SDK 或 Hub 不影响原来的启动。namespace 默认 `/hei`；可用 `--ros-args -r __ns:=/robot1` 或 bringup 的 namespace 参数覆盖。每台设备使用不同 device_id；同 ID 新连接会替换旧连接。参数 hub_url、device_id、report_hz 为启动时只读参数，report_hz 默认 5、范围 [1,20]。

## 能力和语义

- MHS 设备是逻辑 ROS **observer**：kind=sensor、mobile=false、localization=none。它不声称自己拥有实体底盘运动控制或定位能力；后续加入运动工具时需重新设计设备描述与控制契约。
- `get_robot_state` 是 readOnly 工具，返回关节位置、数据年龄、manager 状态、控制器状态、底盘速度和相机可用性。Agnes 通过 `list_devices` / `read_device` 发现状态，或通过 `call_device` 调用这个查询工具。
- `enabled` 表示 manager 报告运动控制器启用，不代表真实电机使能。hardware_connected/homed 保留诊断提供的值或 unknown，不编造硬件状态。
- 关节和相机 1 秒过期，manager 诊断 2 秒过期；重复时间戳不会刷新关节或图像。valid 表示反馈格式和新鲜度，并不表示全部关节完整或机器人可运动。保留最后已知数值，因此使用时要同时检查 valid 与 age。
- MOS `telemetry` values 源最多 2 Hz；front、left_wrist、right_wrist 为 image/jpeg 源，最多各 1 Hz。发送服从 Hub 配置和 SDK wants()，不重复发送同一缓存帧；过期图像不发送。缺少标定，不声明虚构 FOV 或安装位姿。
- MOS 图像时间为桥接进程收到并验证帧的墙上时间，不将 ROS 仿真时间冒充 Unix 时间。它不是准确的相机曝光时间，也没有跨源同步承诺。
- 每路仅保留最新 JPEG（单帧上限 2 MiB），用 OpenCV 检查内容和尺寸。ROS 回调只更新线程安全缓存，asyncio 线程独占 MHS SDK，网络发送不持缓存锁。
- MHS stop 返回 stopped=[]：观测器没有正在控制的运动，不会停止其他 ROS 客户端发起的动作，也不代表物理机器人已停止。stop_scope 状态明确标为 observer_only_no_robot_motion_control。
- SDK 负责断线重连；SIGINT/SIGTERM 取消连接和上报任务，停止并等待 ROS executor。Hub 拒绝注册或同 ID 替换时进程报错退出，避免悄悄失去设备身份。

订阅保持相对名称：joint_states、diagnostics、base_controller/odom、cameras/{name}/image_raw/compressed。关节、里程计、图像使用 sensor-data QoS，诊断 reliable。桥接节点继承 ROS 的参数管理接口，但没有机器人业务写接口。

## 验证

```bash
source /opt/ros/jazzy/setup.bash
source scripts/env/ros-rust.bash
source scripts/env/agent-bridge.bash
source ros2_ws/install/setup.bash
python3 -m pytest -q ros2_ws/tests
```

测试使用真实 SDK、回环 WebSocket 测试 Hub 和完整 mock ROS bringup，检查注册 schema、状态查询、JPEG 二进制、禁止未知运动工具、只读 ROS graph、相机停用后过期、断线重注册和正常退出。它没有运行 LLM，也不代表已经连接某个用户的 Agnes 实例或验证实机。

上游：[SDK](https://github.com/AgnesAI-Labs/agnes-harness/tree/main/packages/mhs/python)、[设备接入](https://github.com/AgnesAI-Labs/agnes-harness/blob/main/docs/guide/mhs.md)。
