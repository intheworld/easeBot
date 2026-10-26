# hei_agent_bridge

原生 rclpy 节点 `/hei/agent_bridge`，使用官方 `agnes_mhs` Python SDK 连接服务器 Agent 内置 Hub 或独立 AgnesHub。

采用 Python `src` 布局：外层是 ROS 包，Python 模块放在独立的源码目录中。
包名、导入名和 `ros2 run/launch` 命令保持 `hei_agent_bridge`。

```text
hei_agent_bridge/
  package.xml / setup.py / setup.cfg   ROS 包与安装配置
  config/                             技能配置
  resource/                           ROS 包索引
  src/hei_agent_bridge/                Python 源码
    node.py                           ROS 节点入口
    discovery.py                      服务发现适配
    device.py / skill_device.py        MHS 设备与技能发布
    skills.py / ros_skills.py          技能执行与 ROS Action 适配
    cache.py                          观测缓存
```
默认保持**只读观测适配器**；显式启用 skill 模式后，发布机器人侧能力目录并执行技能。
`hei_monitor` 保持独立只读。

## MVP skill set

机器人侧 `skills.py` 统一维护定义、参数/输出校验、实现注册、执行记录和取消；
`skill_device.py` 将它们注册为 MHS 工具。服务器只发现目录，不另写一份真实技能配置。

| 技能 | 输入 | 完成条件 |
| --- | --- | --- |
| `get_robot_state` | 无 | 返回 `ready` 和 `backend`；ROS 模式检查已配置动作服务及新鲜位置/速度反馈 |
| `move_to_named_pose` | `side`、`pose` | 轨迹 action 成功、关节到位且反馈表明静止；仅发布真实配置中可用的命名点 |
| `set_gripper_gap` | `side`、`gap_m`（米） | GripperCommand 成功、`reached_goal`、名义开度到位且原始电机反馈静止 |
| `get_camera_observation` | `camera` | front/left_wrist/right_wrist 的新鲜 RGB 快照；front 附可用的对齐深度与内参 |
| `sample_depth_point` | `observation_id`、`u`、`v` | 固定快照中归一化像素位置的相机坐标 XYZ、欧氏距离和有效样本数 |

手臂使用 `/{namespace}/{side}_arm_controller/follow_joint_trajectory`，六个关节按
`a_right_joint1..6` / `b_left_joint1..6` 顺序，配置里的角度单位为 rad。
夹爪使用 `/{namespace}/{side}_gripper_controller/gripper_cmd`，`max_effort=0`，沿用
现有控制器配置的电流限制；不提供未经标定的牛顿力控制，也不声称已经抓住物体。

### RGB-D 观测

front 实机为 RealSense D435。Bridge 订阅 `cameras/front/aligned_depth_to_color/image_raw`
（32FC1 米，也支持外部 16UC1 毫米）、`cameras/front/camera_info` 及现有三路 JPEG。
时间戳、尺寸、frame_id 匹配且数据新鲜时才声明 depth_available；缺内参不允许反投影。
独立摄像头 skill 不要求执行器就绪，不会使能电机。

快照最多保留 8 份、有效 60 秒。内部工具 `read_observation_asset` 按 ID/kind 返回最大 2 MiB
的 RGB JPEG、米制 depth.npy、metadata.json，不读取任意路径。服务器校验大小和 SHA256 后，
保存到 `log/hei_agent/observations/<id>/`，返回 rgb_ref/depth_ref/metadata_ref 本地文件路径。
NPY 使用 `np.load(path, allow_pickle=False)` 读取。快照测量不是实时测距，采集时间保存在 metadata。

u/v 范围 [0,1]，中心为 .5/.5。测距使用 5×5 邻域有效深度的中值，至少三个有效样本。
光学坐标 x 向右、y 向下、z 向前；z_m 为轴向深度，distance_m 为欧氏距离。
未对齐、无深度、过期或畸变模型不支持时拒绝 XYZ；没有基座外参时不声称得到机器人坐标。
当前不包含物体识别、VLM 看图、自动重规划、点云分割或避障；规划器不会从文件引用猜测画面。

服务器常驻终端示例：

```json
{"task":"获取front的RGB-D观测，然后测量这份观测图像中心的三维点，返回图像路径和距离"}
```

mock skill 后端生成 simulated=true 的图像、1 米平面深度和测试内参。
实际相机测试需 ROS skill 配置：先 `ros2 launch hei_bringup cameras.launch.py`，再启动 bridge，
不必启动电机控制栈；整机已占用相机时不要重复启动相机入口。

### 先用机器人侧 mock 跑通

默认 `hub_url=auto`，使用进程内 zeroconf 发现 Hub；不依赖 Avahi/D-Bus。
服务器与机器人可按任意顺序启动。在加载 ROS、workspace 和 Agent SDK 环境的机器人终端运行：

```bash
ros2 launch hei_bringup agent_skills.launch.py \
  hub_id:=lab-agent device_id:=hei-01
```

此入口只启动 skill bridge，不启动整机、不使能控制器。默认配置为
[skills.mock.json](config/skills.mock.json)：所有动作和相机观测在机器人进程内模拟，包含五个技能。
其中的合成角度仅用于模拟，不是实机示教值，不能通过直接改 backend 来用于真机。
它与服务器端 `MockConnector` 不同，是真正通过网络调用的机器人侧 mock。

服务器在仓库根目录准备依赖并常驻运行：

```bash
bash scripts/setup.bash --agent-server
agent/.venv/bin/hei-agent --backend embedded --serve --hub-id lab-agent
```

在该终端输入 `{"list_skills":true}` 查看目录。加载 LLM 配置后可输入
`{"task":"检查状态，右臂到预抓取位，再到搬运位","side":"right"}`。
机器人依赖更新仍通过统一 `scripts/setup.bash` 安装至 `.deps/`，之后重建 bridge/bringup。
多 Hub 可用 hub_id 选择；未指定时仅连接唯一匹配的实例，多个候选会等待。
`discovery_interface` 为本机 IPv4 地址（逗号分隔）；默认所有非回环 IPv4 网卡。
已有健康连接不因广播变化切换，断线重新解析地址，保留技能账本/锁并停止原调用，不重放。

一次性命令仍可用 `hei-agent --backend embedded --list-skills` 或 `--task '...'`。
可加 `--plan-only` 只生成图。保存的图带 `catalog_revision`，之后通过同样的
`--backend embedded --graph /path/to/graph.json` 执行。CLI 显示 transport `backend=embedded`
以及机器人实际 `execution_backend=mock/ros`，两者含义不同。
`--serve` 在任务之间保持监听和连接，stdin EOF/Ctrl-C 退出；一次性模式随任务结束关闭。
仅提供技能链路，Nerve 帧丢弃、不提供上游网页。
完整边界见 [服务器 Agent 文档](../../../agent/README.md#使用机器人发布的-skill-set)。
已有 AgnesHub 时服务器可改用 `--backend mhs --hub-url ws://服务器地址:4180`，
机器人也显式设置 `hub_url:=ws://服务器地址:4180`（绕过发现）。

### ROS action 后端

真实配置从 [skills.ros.example.json](config/skills.ros.example.json) 复制到
已被 Git 忽略的本地配置文件。模板不包含虚构手臂位姿，因此初始目录只发布状态与夹爪技能。

```bash
cp ros2_ws/src/hei_agent_bridge/config/skills.ros.example.json deploy/skills.local.json
# 在 named_poses 中填写已确认的示教点；无需手臂技能时保留空字典
ros2 launch hei_bringup agent_skills.launch.py \
  hub_id:=lab-agent \
  skill_config_file:=$PWD/deploy/skills.local.json
```

`named_poses` 结构是 `{side: {pose_name: {positions_rad: [六个角度], duration_sec: 秒}}}`。
每个示教点需要六个有限数值，duration_sec 支持 0.1–10 秒；这只是 MVP 任务预算，
没有新增手臂角度范围或硬件限位。若配置两侧不同点位名称，仅发布两侧共有名称，
避免目录声称某个 side/pose 组合可用却找不到配置。单侧配置只发布该侧。
轨迹是在关节空间到达示教点，不包含 IK、路径避障或笛卡尔直线运动。

机器人控制栈需要独立启动并就绪；skill bridge 不激活控制器、不打开电机串口，
不配置驱动模式、不回零。ROS namespace 和 domain 必须与控制栈一致。
真实夹爪需要当前自定义 GripperCommand 控制器；旧 `mock_robot.launch.py` 的夹爪
是 JTC，不能直接充当这个 action server。ROS 后端测试使用独立的假控制器服务。

ROS 模式订阅 `joint_states`、`left/right_joint_states` 和
`left/right_dynamic_joint_states`，后者提供真实夹爪原始电机反馈。
重复时间戳不刷新数据。位置/速度超过 1 秒未更新则不接受运动；
手臂到位容差 0.03 rad，夹爪开度容差 0.002 m，静止判据为原始关节/电机速度
绝对值不超过 0.05 rad/s。它们是执行确认阈值，不是额外运动范围限制。

### 契约、执行与停止

- `get_skill_catalog` 返回 `hei.skills/v1` manifest：backend、技能版本、参数/输出、
  单位、结果、超时和配置摘要；`revision` 是规范 JSON 的 SHA256。
  配置摘要包含点位角度与持续时间，改动相同名称背后的点位也会改变版本摘要。
- MHS 技能工具由同一份定义生成，每次调用必须带 `revision` 和 `invocation_id`。
  机器人逐次核对版本，严格检查参数；MHS 自动 clamping、丢弃未知参数或默认化时
  会拒绝执行，避免实际动作偏离提交的参数。技能和配置发布后不可变，更改需重启实例。
- 同一 skill set 只允许一个运动技能，占用 `manipulation` 资源；状态读取可并发。
  不仲裁其他 ROS 客户端，也不是跨进程或全机器人控制租约。
- 执行记录区分 running/succeeded/failed/cancelled。普通 SDK 的 `accepted` 仅为接单；
  最终 `data` 包含本地技能结果与 cleanup_complete，服务器等待最终结果。
- `cancel_skill` 按 invocation_id 取消并等本地执行终止；`get_skill_execution` 可查询状态。
  未见过的取消 ID 会记为 not_started，阻止迟到请求启动。重复 ID 拒绝自动重放。
  记录只在内存保留最近最多 256 个，不提供跨进程重启的恰好一次语义。
- ROS 取消要等待 action 终态以及新鲜静止反馈，不能只等 cancel 请求 ACK。
  本地取消预算 3 秒；未确认就锁定运动，后续不能因新调用自动清除。
  重启 bridge 不会自动停止控制器，需独立核实设备状态。
- MHS stop 和 Hub↔设备断线只停止本实例拥有的技能动作，不停止其他 ROS 客户端的任务。
  本地技能有独立超时（运动 15 秒、状态 2 秒），失败/超时先做取消确认。
  服务器↔Hub 断线时 Hub 可能继续执行已接受任务，不能声称立即停机；服务器报告状态未知，
  不自动重连或重试运动。本 MVP 尚无端到端心跳租约或硬件急停集成。

部署依然需要单独核实示教点、夹爪名义开度换算和实际停止效果；当前完成的是
SDK/ROS 接口联调，没有通过本次实现操作真实机器人。

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

默认 `agent_bridge:=false`，没有 SDK 或 Hub 不影响原来的启动。namespace 默认 `/hei`；可用 `--ros-args -r __ns:=/robot1` 或 bringup 的 namespace 参数覆盖。每台设备使用不同 device_id；独立 AgnesHub 会替换同 ID 旧连接，Agent 内置 Hub 拒绝重复连接。参数 hub_url、device_id、report_hz 为启动时只读参数，report_hz 默认 5、范围 [1,20]。

## 默认只读模式的能力和语义

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

Skill 测试见 `agent/tests/test_skill_set.py`、`test_mhs_connector.py` 和
`ros2_ws/tests/test_skill_bridge.py`、`test_ros_skills.py`：覆盖单一目录、版本变化、
参数/输出类型、互斥、重复调用、取消竞态、无法确认停止、真实 SDK 与 CLI 的回环调用，
以及隔离 ROS 域中的真实 action 客户端对接假控制器。不打开真实硬件。

2026-10-25 另完成 D435 实机相机验收：只启动独立相机与 ROS skill bridge，未启动或调用电机。
640×480 RGB/对齐深度和出厂内参通过 ROS→MHS→本地资产传输验证，中心点得到
z≈2.169m、25 个有效样本、simulated=false。验收进程结束后释放相机；
该单次观测不代表精度标定、外参标定或抓取/避障验证。

上游：[SDK](https://github.com/AgnesAI-Labs/agnes-harness/tree/main/packages/mhs/python)、[设备接入](https://github.com/AgnesAI-Labs/agnes-harness/blob/main/docs/guide/mhs.md)。

## Right-arm RGB-D grasp

Optional `config/skills.ros.grasp.json` enables the right-arm OBB grasp pipeline:
`perceive_grasp_target` → `plan_right_grasp_from_object` → `execute_right_grasp`.
Perception returns an immutable robot-owned object reference and a visible-surface OBB.
Planning tries top-down candidates with IK and sampled robot/table collision checks.
Execution stops after closure; there is no automatic lift, retreat or return, and no
object-held verification. The retired one-step grasp interface and old plan format
are no longer supported. Rediscover the catalog and regenerate plans after updating.

It reuses the existing compute environment, weights and calibration without opening
serial ports in the worker. ROS controllers remain the only hardware owners.
See [右臂抓取文档](../../../docs/right-grasp.zh-CN.md) for setup and the three current examples.
