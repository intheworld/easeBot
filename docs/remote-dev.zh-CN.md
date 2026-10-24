# SSH / rsync 开发同步

本机编辑，单向推送源码，目标机器原生编译。用于专用开发目录；不要指向正在运行的部署目录，也不要在接收目录直接修改代码。服务器与机器人各有一份源码及独立的构建产物。此工具不会启动、停止或重启节点。

## 首次配置

### 1. 在本机创建 targets.local.json

`deploy/targets.local.json` 是同步工具默认读取的配置，**需要在运行工具的开发电脑上创建，不是在机器人上创建**。仓库只提交模板，新克隆的仓库不会自带本地配置。

在 easeBot 仓库根目录执行以下命令；如果配置已存在，直接编辑，不要覆盖：

```bash
cp deploy/targets.example.json deploy/targets.local.json
```

用编辑器打开 `deploy/targets.local.json`，修改成实际连接信息。例如（用户名、地址和目录均需替换）：

```json
{
  "robot": {
    "host": "robot@192.168.1.20",
    "path": "~/dev/easeBot",
    "ros_distro": "jazzy"
  },
  "server": {
    "host": "developer@192.168.1.10",
    "path": "~/dev/easeBot",
    "ros_distro": "jazzy"
  }
}
```

| 字段 | 含义 |
| --- | --- |
| `robot`、`server` | 目标名称，对应命令中的 `robot`、`server`，可以自行命名 |
| `host` | `用户名@IP`、`用户名@主机名`，或下面配置的 SSH 别名 |
| `path` | 远端专用开发目录，支持绝对路径或 `~/dev/easeBot`，不支持空格；不要指向已有 Git 仓库或运行目录 |
| `ros_distro` | 远程构建时加载的 ROS 版本，目前为 `jazzy` |
| `ros_setup` | 可选，指定远端 ROS/依赖工作空间的 `setup.bash`，支持绝对路径或 `~/`；未指定时查找 `/opt/ros/<版本>` 和 `~/ros2_<版本>/install` |

此文件已被 `.gitignore` 和 rsync 排除，不会提交或发送到远端。需要使用其他配置文件时，可以指定 `python3 scripts/remote-dev.py --config /path/to/targets.json robot check`。

机器人采用源码版 Jazzy 和独立控制器工作空间时，当前 robot 配置增加 `"ros_setup": "~/hei_jazzy_deps/install/setup.bash"`。该 setup 会加载它构建时使用的 Jazzy 基础环境。远程构建还会加载已安装的 `~/.cargo/env`，以及仓库 `.deps/build-python` 中的隔离 colcon 插件。

### 2. 配置并验证 SSH

两端需要 Bash、SSH 和 rsync；远端需要运行 SSH 服务。上面使用 `用户名@IP` 时可以直接连接，无需额外创建 SSH 别名。

如果需要别名、指定密钥、端口或跳板机，可在本机 `~/.ssh/config` 添加以下示例，并把 JSON 中的 `host` 分别改为 `hei-robot`、`hei-server`：

```sshconfig
Host hei-robot
    HostName 192.168.1.20
    User robot
    IdentityFile ~/.ssh/id_ed25519

Host hei-server
    HostName 192.168.1.10
    User developer
    IdentityFile ~/.ssh/id_ed25519
```

先使用 `ssh robot@192.168.1.20`（或配置好的 `ssh hei-robot`）验证身份和登录。工具沿用正常的 SSH 主机密钥校验，不关闭校验，也不存储密码。

### 3. 检查连接并初始化远端目录

完成本地 JSON 配置和 SSH 登录后，在仓库根目录执行：

```bash
python3 scripts/remote-dev.py robot check
python3 scripts/remote-dev.py robot init
# 需要同步到服务器时也执行：
python3 scripts/remote-dev.py server check
python3 scripts/remote-dev.py server init
```

`init` 仅接受新目录或空目录，写入 `.hei-sync-root` 标记；已初始化时可以重复执行。同步和构建必须有此标记，拒绝接收目录根部包含 `.git` 的工作区。不要手工在已有目录补标记来绕过检查。

路径以 `~/` 开头时，脚本通过 SSH 查询远端用户的 `$HOME` 后展开，不使用本机主目录。只支持当前登录用户的 `~/`，不支持 `~其他用户/`。

## 编辑和同步

```bash
# 先预览；要求已经 init，不写远端源码
python3 scripts/remote-dev.py robot sync --dry-run
python3 scripts/remote-dev.py robot sync

# 本机删除/重命名了源码时，预览并同步删除
python3 scripts/remote-dev.py robot sync --delete --dry-run
python3 scripts/remote-dev.py robot sync --delete

# 服务器使用相同流程
python3 scripts/remote-dev.py server check
python3 scripts/remote-dev.py server init
python3 scripts/remote-dev.py server sync --dry-run
python3 scripts/remote-dev.py server sync
```

同步整个当前源码树，包含未提交的新文件和修改；**不是 Git 已跟踪文件清单，也不是密钥扫描器**。除 `deploy/rsync.exclude` 中的排除项外，文件都会发送，新增私有目录需补充排除规则。

默认不删除远端多余文件；删除或重命名包后，应使用 `--delete`，否则旧代码仍可能被构建。排除项始终受保护，`--delete` 不会删除远端 `build/install/log`、`.deps` 或本机配置。移除/改名 ROS 包时，旧构建产物也仍存在，需要在停用相关进程后单独清理对应包的产物。

Git 元数据、编译目录、依赖缓存、编辑器配置和常见本机配置不发送。安全的仓库内部符号链接可同步，指向仓库外部的链接通过 `--safe-links` 跳过。目标开发目录应由可信用户独占，避免并发同步、构建及其他用户修改目录或符号链接。

## 自动同步

配置并初始化目标后，在本机单独开一个终端：

```bash
python3 scripts/remote-dev.py robot watch
# 同时同步本地文件的删除/重命名：
python3 scripts/remote-dev.py robot watch --delete
```

启动即同步一次，此后默认每 0.5 秒扫描本地文件，连续修改停止 1 秒后合并推送。Ctrl+C 停止；不自动编译、启动或重启。使用 SSH 密钥或 ssh-agent 登录，watch 使用 BatchMode，不弹密码提示。同步失败时退出并返回非零码；恢复连接后重新运行，会先补一次全量差异同步。

排除规则与手动同步一致，构建目录/缓存变化不会触发上传。watch 采用校验和比较，确保同一秒内、大小不变的文件修改也能传输；大文件较多时每次同步校验会增加开销。可以通过 `--interval 1 --debounce 2` 调整扫描与合并时间。当前监听器支持排除文件中的文件名通配规则（可带末尾 `/` 表示目录），不支持带中间路径的复杂 rsync 规则。

这是单向源码镜像，不处理两边编辑冲突，不纠正无本地变化时的远端修改。远端目录仅作接收端；同步与构建应错开，尤其不要在同一目标上并发运行多个 watcher。首次使用可先执行手动 `sync --dry-run` 检查传输范围。它仍要求使用未运行节点的专用开发目录。

## 在目标机器编译

首次需要在目标机器安装对应 ROS Jazzy、colcon、Python/Rust/C++ 工具链和消息绑定。同步不会安装系统依赖或执行 sudo。Rust monitor 的依赖详见其 README。

依赖准备也通过本机统一入口触发，在目标机器执行相同的 `scripts/setup.bash`：

```bash
python3 scripts/remote-dev.py robot sync
python3 scripts/remote-dev.py robot setup
# 不使用 Agent Bridge 时：
# python3 scripts/remote-dev.py robot setup --without-agent-bridge
```

setup 会加载目标配置的 ROS 环境，复用或准备构建插件、Rust 消息索引、motorbridge、Agent SDK。缺少完整 ROS/Rust 工具链时先报错；不隐式安装系统包、同步源码、编译或启动节点。它与 build 一样只接受带标记的专用目录。各依赖细节见[脚本说明](../scripts/README.md)。

返回本机，执行：

```bash
# 仅构建这个包及其递归依赖
python3 scripts/remote-dev.py robot build --packages hei_agent_bridge

# 构建整个工作区
python3 scripts/remote-dev.py robot build
```

构建命令优先加载 `/opt/ros/<ros_distro>/setup.bash`；不存在时尝试 `~/ros2_<ros_distro>/install/setup.bash`，以支持独立源码安装。随后加载仓库的 Rust/Agent 环境脚本，并核对实际 ROS_DISTRO。SSH 非交互环境的 PATH 必须包含 colcon、cargo、cargo-ament-build 等工具；不要依赖 `.bashrc` 中只在交互终端执行的配置。编译 stdout/stderr 直接显示，SSH/rsync/构建失败均返回非零退出码。`build` 不隐式同步，始终先执行 `sync`。

构建产物保留在目标机器的 `ros2_ws/{build,install,log}`。即使源码是 Python 包，也需要 colcon 完成安装和入口点更新。这里不使用 `--symlink-install`，但仍是开发工作区，不提供原子发布或版本回滚。同步不是事务，传输失败应重新同步成功后再构建；不要对已加载这份源码的运行进程进行同步或构建。

服务器的 Agnes Hub 不属于本仓库的 colcon 构建；`server sync` 只复制 easeBot。只有决定在服务器上运行这里的 ROS 包时，才需要 ROS 和 `server build`。

## 手动运行

登录机器人，在目标目录加载环境，按现有 README 启动；工具不会替你重启或使能机器人：

```bash
source /opt/ros/jazzy/setup.bash
source scripts/env/ros-rust.bash
source scripts/env/agent-bridge.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=71
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch hei_bringup robot.launch.py agent_bridge:=true \
  agent_hub_url:=ws://192.168.1.10:4180
```

Hub 地址替换为局域网服务器实际的 MHS 服务地址。ROS 仍限制在机器人本机，bridge 通过 WebSocket 访问服务器；SSH 同步和 ROS 节点发现是独立的事情。

## 2026-10-22 真机部署验证

实际机器人为 ARM64、Ubuntu 22.04.5、ROS 2 Humble，现有仓库的完整控制栈基于 Jazzy。**本次通过的是基础包编译和真实相机部署，不是完整机器人控制栈部署。**未打开电机串口、未发送运动命令，也未改变宿主机 ROS/Python 系统包。

- SSH/rsync 源码同步通过，目标为配置中的 `~/gitdir/easeBot`。
- Humble 原生编译通过：`hei_sensors`、`basic_teleop`、`hei_description`、`hei_control`。
- 三路真实相机 JPEG 均解码为 640×480，诊断 online；deactivate 后停止图像但保留诊断，activate 后恢复，cleanup 后重新配置可再次打开设备。
- 三相机共享 USB 2.0，总线上的默认 YUYV 并发采集出现超时。`deploy/robot/cameras.yaml` 显式使用 V4L2，腕部相机改为 MJPEG，实机验收通过。
- 用户 site-packages 中 NumPy 2 与系统 OpenCV 不兼容；使用 `PYTHONNOUSERSITE=1` 运行，不修改用户或系统安装的包。
- 尝试下载官方 `ros:jazzy-ros-base` 容器镜像，但机器人访问 Docker Hub 超时。Rust monitor、完整 ros2_control 及 Agent/Hub 联通尚未在该机器人验证。

Humble 测试使用独立目录，避免与后续 Jazzy 构建混用。在机器人专用开发目录复现：

```bash
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
colcon --log-base ros2_ws/log-humble build --base-paths ros2_ws/src \
  --build-base ros2_ws/build-humble --install-base ros2_ws/install-humble \
  --packages-select hei_sensors basic_teleop hei_description hei_control \
  --executor sequential
source ros2_ws/install-humble/setup.bash
set -o pipefail
HEI_TEST_REAL_CAMERAS=1 python3 -m pytest -q -s ros2_ws/tests/test_robot_cameras.py \
  2>&1 | tee ros2_ws/log-humble/physical-cameras.log
```

这条验收会短暂打开真实相机，结束后退出采集进程并释放设备，默认普通测试不会执行它。测试使用独立 ROS domain 和 namespace，不需要启动控制器。日志保留在机器人 `ros2_ws/log-humble/physical-cameras.log`。

`remote-dev.py robot build` 按照本地配置的 `ros_distro` 构建，不能用它代替上述隔离的 Humble 子集测试。该轮测试时还没有安装 Jazzy；后续已选择下节的独立源码构建路线。仅修改配置里的 distro 并不完成环境与依赖准备。

## Ubuntu 22.04 上独立源码安装 Jazzy

2026-10-22 已在当前 ARM64 机器人完成安装：233 个包构建成功，耗时 1 小时 5 分 34 秒。`ros2` CLI、C++→Python 与 Python→C++ 的真实 DDS 通信验收通过；确认 rclpy 来自新安装目录，rclcpp 共享库无缺失且未加载 Humble。原 `/opt/ros/humble` 保留，未修改用户 shell 的默认 ROS 环境。

Ubuntu 22.04 是 Jazzy 的 Tier 3 源码构建平台，不使用 Ubuntu 24.04 的二进制包混装。当前选择保留 `/opt/ros/humble`，在 `~/ros2_jazzy` 源码构建无 GUI 的 `ros_base` 和 `demo_nodes_cpp/demo_nodes_py`。这一步安装 ROS 基础环境，尚不包含额外的 ros2_control 控制器、Rust 工具链或 easeBot 应用部署。

下列命令在机器人执行，工作目录为本仓库。依赖还包括现有系统已具备的编译器、CMake、colcon、vcstool、rosdep、Python YAML 等；所缺系统依赖应以 rosdep 解析结果为准，不执行系统整体升级。

```bash
PYTHONNOUSERSITE=1 python3 scripts/jazzy/fetch-source.py
PYTHONNOUSERSITE=1 python3 scripts/jazzy/prepare-vendors.py
bash scripts/jazzy/build-source.bash
```

Git 传输不可用时，第一步从官方 `ros2.repos` 的仓库下载 GitHub 官方源码归档，并补充 `ros2/variants` 元包。归档和 SHA256 保存在 `~/ros2_jazzy/downloads` 与 `source-archives.lock.json`，重复执行保留已下载源码。vendor 准备脚本保持库版本，将静态声明的 Git 下载换成缓存归档；改动记录在 `vendor-downloads.patch` 和 `vendor-archives.lock.json`，原 CMake 文件也有备份。

构建脚本清除继承的 ROS 环境，禁用用户 Python site-packages，使用 Release、顺序构建包及每包两个编译任务，关闭构建测试与 tracing。构建产物位于 `~/ros2_jazzy/{build,install,log}`，终端输出保存到 `build-console.log`。失败后先处理报错，再运行构建脚本即可增量继续。

安装成功后，在**没有加载 Humble 的终端**使用和验收：

```bash
export PYTHONNOUSERSITE=1
source ~/ros2_jazzy/install/setup.bash
export ROS_DOMAIN_ID=178
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 scripts/jazzy/verify.py
```

验收检查 ROS_DISTRO、rclpy 的实际加载位置、ros2 CLI，以及 C++→Python 和 Python→C++ 的 DDS 通信；结束后清理测试进程。不要同时 source Humble 和 Jazzy，也不要复用另一个发行版的 easeBot build/install 目录。

机器人保留 `~/ros2_jazzy/verification.log`（验收输出）、`rclcpp-linkage.log`（共享库加载路径）和 `build-environment.json`（架构、工具版本、构建参数）。正常使用只需在干净终端执行 `export PYTHONNOUSERSITE=1` 后 source 上面的 setup 文件；构建与验收日志不会自动删去。

## Jazzy 工程环境与验证

2026-10-23 右夹爪电机级测试：motor7 临时切为 Torque_Pos，目标相对 -0.02 rad、约 2 秒、速度上限 0.05 rad/s、电流比例 1%。实测位移 -0.011444 rad，未到目标；已停止，未自动加电流。结束夹爪失能、恢复原 MIT 模式，前六轴保持失能且位置不变。该次直接使用 motorbridge，尚未接入 ROS 夹爪 Action。详见 [硬件测试记录](../ros2_ws/src/hei_hardware/README.md)。

2026-10-23 右臂真机只读检查：六轴 JointState 正常、18 状态接口且无命令接口。只读查询显示六轴均为 MIT 模式、失能、无故障；joint2 约 +0.01202 rad 超出当前参考上限 0。尚未使能、切换模式或发送运动，需先核对机械零位与现场条件。详见 [硬件检查记录](../ros2_ws/src/hei_hardware/README.md#右臂首次真机只读检查2026-10-23)。

2026-10-23 统一依赖入口：本地与机器人默认 `setup` 均执行成功，机器人旧的无 Git 元数据 SDK 已备份至 `.deps/agnes-harness-before-setup.*`；远程 build 随后编译 9 个包成功。脚本回归 18 项通过。日常使用 `sync → setup（首次或依赖更新后）→ build`，不再逐个调用模块安装脚本。

2026-10-23 motorbridge 迁移后：底层协议和串口由固定 commit `d5700bcd5b7f9355dd2012488d463c3bac982259` 的 Rust 库提供，Cargo.lock 固定传递依赖。机器人首次执行 `bash scripts/setup/prepare-motorbridge.bash` 后，colcon 离线构建 9 个工程包成功。10 项驱动测试通过，故障批次用例连续 50 次通过；Python 全量回归 **23 passed、1 skipped**（47.44 秒），包括六轴 PTY 上的标准轨迹执行。动态库依赖完整、未混入 Humble，插件通过 `$ORIGIN` 加载相邻 Rust 后端。本机也通过相同驱动测试和工程回归，并验证库搬迁加载与独立 CMake 消费者构建。全部使用模拟电机；默认整机 mock、单臂入口只读。机器人旧驱动源码保存在 `.deps/hardware-before-motorbridge-*`；日志为 `ros2_ws/log/motorbridge-*.log`、`motorbridge-project-tests.xml`，参考配置移至 `hei_bringup/config/`。

2026-10-23 增加达妙驱动后：机器人原生 Jazzy 编译 9 个包成功，10 项 C++ PTY 硬件测试通过，Python 全量回归 23 passed、1 skipped。新驱动只在虚拟串口上测试，未打开真实电机；单侧机械臂默认只读入口与限制见 [hei_hardware](../ros2_ws/src/hei_hardware/README.md)。本次日志为 `ros2_ws/log/damiao-*.log` 和 `damiao-project-tests.xml`。

2026-10-22 已在 ARM64/Ubuntu 22.04.5 机器人上完成原生 Jazzy 工程验证：

- 28 个控制依赖包编译成功；工程 8 个包通过 SSH 构建入口全部编译成功。
- Python 工程测试 **16 passed, 1 skipped**（37.97 秒）；默认跳过的真实相机用例已单独运行，**1 passed**（8.84 秒）。三路 JPEG 均为 640×480，生命周期停用、恢复和 cleanup 后重开通过。
- Rust monitor 单元测试 **3 passed**；完整 mock 启动、控制器状态、轨迹 Action、全向底盘命令超时、停止及反馈失效处理、只读 Web、Agent 注册/查询/图像/重连测试通过。
- 实际版本：controller_manager 4.44.0、轨迹/全向控制器 4.39.0、control_msgs 5.10.0、realtime_tools 3.13.0。CLI 插件加载正常，主要二进制动态库无缺失、未混入 Humble，测试应用进程已清理。

控制测试使用 mock 硬件，没有打开电机设备或向真实电机发送动作。Agent 使用回环测试 Hub；尚未验证真实电机控制插件和生产 AgnesHub/LLM 链路。

机器人日志在 `~/gitdir/easeBot/ros2_ws/log/`：`jazzy-project-tests.log`、`jazzy-project-tests.xml`、`jazzy-monitor-tests.log`、`jazzy-project-environment.json`；真实相机日志为 `jazzy-camera-tests/physical.log`。

机器人上的基础 ROS 位于 `~/ros2_jazzy/install`，额外控制依赖位于 `~/hei_jazzy_deps/install`，工程位于 `~/gitdir/easeBot`。第二层的 setup 会加载第一层；工程仍使用自己的 `ros2_ws/build`、`install`、`log`。精确依赖提交记录在 `deploy/robot/jazzy-project.repos`，机器人保留下载归档、SHA256 和构建日志。

控制依赖使用 ros2_control 4.44.0、ros2_controllers 4.39.0。Ubuntu 22.04/GCC 11 上显式设置 `-DCMAKE_CXX_STANDARD=17`：该版本的自动 C++20 路径会在 `lexical_casts.cpp` 出现 `std::from_chars` 编译错误。没有修改上游算法源码。依赖工作空间的构建命令为：

```bash
# 在已有依赖源码的机器人上；新机器先按 jazzy-project.repos 获取源码
source ~/ros2_jazzy/install/setup.bash
cd ~/hei_jazzy_deps
export PYTHONNOUSERSITE=1 CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
colcon build --base-paths src --merge-install --executor sequential \
  --packages-up-to controller_manager joint_state_broadcaster \
  joint_trajectory_controller omni_wheel_drive_controller xacro ros2controlcli \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 -DCMAKE_DISABLE_FIND_PACKAGE_Doxygen=TRUE
```

在机器人干净终端中加载完整环境：

```bash
cd ~/gitdir/easeBot
export PYTHONNOUSERSITE=1
source ~/hei_jazzy_deps/install/setup.bash
# 首次准备：bash scripts/setup.bash；日常只加载环境
source scripts/env/workspace.bash
source ros2_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY
```

构建插件和 Agent 依赖均保存在仓库 `.deps/`，不替换系统 Python。Python 3.10 的 Agent 依赖使用独立兼容版本；ARM64 原生 wheel 不与开发机共享。Rust 消息兼容索引也在 `.deps/`，无需修改 ROS 安装。

开发电脑更新源码后执行：

```bash
python3 scripts/remote-dev.py robot sync
python3 scripts/remote-dev.py robot build
```

上述构建依赖本地 robot 配置的 `ros_setup` 指向 `~/hei_jazzy_deps/install/setup.bash`。测试在机器人的完整环境中执行：

```bash
set -o pipefail
cd ros2_ws
python3 -m pytest -q tests 2>&1 | tee log/jazzy-project-tests.log
cargo test --locked --manifest-path src/hei_monitor/Cargo.toml \
  --target-dir build/hei_monitor 2>&1 | tee log/jazzy-monitor-tests.log
# 单独开启真实相机验收；普通测试默认跳过它
HEI_TEST_REAL_CAMERAS=1 python3 -m pytest -q -s tests/test_robot_cameras.py
```

控制栈联合测试加载 `mock_components/GenericSystem`，检查控制器、轨迹 Action、底盘命令超时、停止服务、反馈失效处理、相机生命周期及 Rust 只读 Web。Agent 测试运行真实 SDK 与回环 WebSocket 测试 Hub，不连接外部 LLM 或生产 Hub。真实相机测试只启动采集节点。


### 七电机共享总线整合（2026-10-23）

同侧机械臂与夹爪现由一个 DamiaoSystem 统一读写串口。`damiao_arm.launch.py` 默认加载六轴与 motor7，机械臂 JTC 和夹爪控制器分别占用 6 与 4 个命令接口，初始均 inactive。`gripper.launch.py` 成为同一入口的别名。可同时发轨迹与夹爪动作，到位后保持；夹爪取消/超时/单独停用不撤销机械臂接口。单侧硬件故障会停用两者。

机器人 ARM64 Jazzy 已编译，完整模拟回归 **52 passed、1 skipped**，C++ 驱动 **11 项通过**；均未发送新的真机运动目标。日志：`ros2_ws/log/gripper-shared-project-tests-robot.log`、`gripper-shared-cpp-tests-robot.log`。完整操作命令及标定来源见 [共享夹爪控制器](../ros2_ws/src/hei_hardware/GRIPPER.md)。原模型的开度换算已记录为名义参考；现有电机保护配置不包含牛顿标定，因此标准 Action 仍默认拒绝 SI 目标，夹爪已统一使用标准接口，未标定前不接受运动目标。


### 移除独立夹爪节点（2026-10-23）

已删除旧 Python 独立夹爪节点、专属 launch/配置和 15 项旧实现测试，清理本机与机器人的对应 build/install 残留并重新生成工作区环境。夹爪控制统一由共享总线控制器提供；机器人 ament 索引已确认旧包不可发现、共享控制器仍存在。清理后本机和机器人 Jazzy 回归均为 **37 passed、1 skipped**，含七电机并行控制、取消/超时隔离和硬件故障联动测试。日志为 `ros2_ws/log/remove-legacy-gripper-tests-{local,robot}.log`；本轮未运行真实电机。


### 统一为标准夹爪接口（2026-10-23）

自定义接口包及电机角度 Action 已删除，清理源码、依赖、文档与本机/机器人对应 build/install 残留。夹爪控制器仅提供标准 `control_msgs/action/GripperCommand`；速度与超时统一由节点配置管理，新增 goal_timeout_sec（0 按行程估算）。未标定时仍拒绝目标。

本机与机器人 Jazzy 完整回归均为 **38 passed、1 skipped**；测试全部使用标准夹爪 Action，覆盖并行、保持、取消、超时、目标校验、未标定拒绝与共享硬件故障。另检查 ROS 图中夹爪只有一个标准 Action，并确认 ament 索引不再包含自定义接口包。日志为 `ros2_ws/log/remove-custom-gripper-action-tests-{local,robot}.log`。未执行真机运动。


### 控制器并入硬件包（2026-10-23）

夹爪控制器已并入 hei_hardware，与 DamiaoSystem 分别作为两个独立插件注册；控制器类型为 `hei_hardware/GripperController`，串口仍只由 DamiaoSystem 管理，标准 Action 地址不变。旧的独立控制器包及其 build/install 残留已从本机和机器人清理，bringup 依赖和 pluginlib 配置已更新。包内代码为 `src/damiao_system.cpp`、`src/gripper_controller.cpp`，使用说明见 [GRIPPER.md](../ros2_ws/src/hei_hardware/GRIPPER.md)。

本机与机器人 ARM64 Jazzy 均编译通过，完整回归均 **38 passed、1 skipped**；C++ 测试均 **12 项通过**，新增直接从硬件包加载夹爪控制器的验收。日志为 `ros2_ws/log/merge-gripper-plugin-{tests,cpp}-{local,robot}.log`。本轮未执行真机运动。


### 夹爪启动预设修正（2026-10-23）

`gripper.launch.py side:=right` 现在自动激活夹爪控制器，六轴只导出状态接口、保持只读，允许现有 MIT 模式；motor7 收到合法 Action 目标才使能。正式只读入口也加载 inactive 的夹爪控制器，Action 可被发现。默认位置调试允许 max_effort=0，以配置电流比例 0.01 控制名义开口；力未标定时反馈 NaN，正数牛顿目标仍拒绝。不能仅通过 calibrated=true 绕过无效力换算系数。

本机完整回归 42 passed、1 skipped，随后新增标定标志不实的用例，两项位置调试用例通过；机器人 ARM64 Jazzy 8 包构建成功，完整回归 **43 passed、1 skipped**，两端 C++ 驱动测试均 **13 项通过**。测试使用虚拟串口，未发送真机运动目标。日志为 `ros2_ws/log/gripper-startup-{tests,cpp}-{local,robot}.log`。部署保留了旧进程映射的共享库 inode；当前旧 launch 需手动退出并重新启动才能使用新入口。具体命令见 [夹爪启动](../ros2_ws/src/hei_hardware/GRIPPER.md)。


### 夹爪反馈力矩阈值显式配置（2026-10-23）

按 DM-J4310-2EC V1.1 官方额定输出力矩，将左右硬件配置的 `gripper.torque_limit_nm` 设为 3.0，经 Xacro 传入硬件插件。移除原先隐藏的 0.5 N·m 缺省值和 1 N·m 校验上限；可写夹爪必须提供阈值，要求为正且不超过配置的协议力矩量程。协议 `t_max=10`、速度 10 rad/s、电流比例 0.05 保持。额定值是本工程选择的软件阈值，不是厂家规定的统一夹爪停机阈值；V1.1 峰值为 7 N·m，没有采用峰值作为持续输出限制。来源及单位见 [夹爪说明](../ros2_ws/src/hei_hardware/GRIPPER.md)。

本机和机器人 ARM64 Jazzy 均完成 8 包构建，完整回归均 **44 passed、1 skipped**，C++ 驱动测试均 **13 项通过**。新增虚拟串口用例验证 0.6 N·m 不触发旧阈值、3.1 N·m 触发停机，并核对日志中的实际阈值。日志为 `ros2_ws/log/gripper-rated-{build,cpp,tests}-{local,robot}.log`。未发送真机目标，当前运行进程需退出并重新启动后使用新配置。


### 双侧夹爪统一启动（2026-10-23）

> 历史记录：这里最初使用的 hei_left/hei_right namespace 已在后续统一为 hei；旧 gripper.launch.py 也已删除。当前命令以 [夹爪说明](../ros2_ws/src/hei_hardware/GRIPPER.md) 为准。

新增 `ros2 launch hei_bringup grippers.launch.py`，复用两次单侧调试入口，通过独立 scoped launch 配置隔离参数。默认左右 namespace 为 hei_left、hei_right，各自一个 controller_manager 与串口所有者；两侧六轴只读、夹爪控制器自动激活，收到合法目标才使能。状态话题和 TF 按侧隔离，不在该入口汇总整机 TF。支持 read_only、position_only、activate_gripper、左右硬件配置、共用夹爪控制器配置及左右 namespace 参数。启动前拒绝重复 namespace 和相同串口（含符号链接别名）。操作命令见 [双侧夹爪](../ros2_ws/src/hei_hardware/GRIPPER.md)。

本机和机器人 ARM64 Jazzy 均安装通过，新增 **4 项测试均通过**：双总线并发动作、单侧动作不使能另一侧、双侧完全只读、namespace/串口冲突检查；两侧六轴均未使能，退出后夹爪失能。日志 `ros2_ws/log/dual-gripper-tests-{local,robot}.log`。全部使用两条虚拟串口，未启动双侧真实硬件或发送真机目标；使用前先退出原先占用任一侧串口的 launch。


### 统一双侧 namespace 并删除旧夹爪入口（2026-10-23）

`grippers.launch.py` 默认统一 namespace 为 `/hei`，直接组合单侧基础 `damiao_arm.launch.py`。两侧管理器分别为 `/hei/left_controller_manager`、`/hei/right_controller_manager`；对应广播器、robot_state_publisher、硬件实例均用侧名区分。Action 固定为 `/hei/left_gripper_controller/gripper_cmd` 和 `/hei/right_gripper_controller/gripper_cmd`。描述、状态和 TF 话题在 `/hei` 下用 left_/right_ 前缀隔离，防止两个管理器订阅到对侧硬件描述。单侧基础入口默认名称保持兼容。统一 namespace 可用 `namespace` 参数修改。

已删除旧 `gripper.launch.py`，并清理本机、机器人源码及 install 中的对应文件和缓存。单侧夹爪诊断需要时使用基础入口的 `arm_read_only:=true activate_gripper:=true gripper_position_only:=true read_only:=false`，不再维护额外包装入口。

本机与机器人均通过 **23 项相关回归**，覆盖双侧并发、公共 namespace 内节点名称唯一、描述/状态/TF 话题隔离、只读无写入、串口冲突拒绝、单侧正式控制入口和夹爪保护。日志为 `ros2_ws/log/unify-gripper-namespace-tests-{local,robot}.log`。全部使用虚拟串口，未发送真机目标；需退出旧 launch 后重新运行 `ros2 launch hei_bringup grippers.launch.py`。


### 六轴启动模式配置选项与右臂启动验证（2026-10-23）

真实右臂可写启动失败原因为 `a_right_joint1: CTRL_MODE mismatch`；当时串口空闲，未进入限位或轨迹激活阶段。新增 `configure_control_mode:=true`，默认关闭。开启后，仅在所有可写资源均有新鲜、失能、无故障及已知模式反馈时，配置六轴 RAM CTRL_MODE 到命令接口所需 POS_VEL=2（或速度接口 VEL=3），回读正确后继续启动。不保存 EEPROM、不使能、不改零位、不清故障；只读/state-only 电机不写模式，运行中的模式异常仍锁存故障。错误日志增加 actual/expected。入口命令见 [硬件说明](../ros2_ws/src/hei_hardware/README.md)。

本机和机器人均通过 **25 项相关 ROS 回归、16 项 C++ 驱动测试**。新增用例覆盖 MIT=1 失能状态切到 POS_VEL=2、回读失败拒绝启动、拒绝已使能电机、只读时即使设置选项也不写模式。日志为 `ros2_ws/log/arm-mode-setup-{build,cpp,tests}-{local,robot}.log`。

随后在串口空闲时，以临时 namespace 和 `include_gripper:=false configure_control_mode:=true read_only:=false` 完成右臂真实启动验证，六轴 RAM 模式从 1 切到 2，`right_arm_controller` 保持 inactive，只有状态广播器 active；未激活轨迹控制器、未发送目标，验证进程已退出。退出后只读回查六轴均 mode=2、status=0、fault=0，位置与此前只读记录一致。日志 `arm-mode-setup-real-startup.log` 与 `arm-mode-setup-real-disabled.json`。没有保存 EEPROM、写零位或操作左臂/夹爪。

右臂 joint2 当时为 +0.012016 rad，高于参考目标上限 0。后续已移除因此拒绝六轴激活的检查：以实测位置初始化保持目标，新目标仍受配置范围约束。


### 2026-10-23：修复激活后通用限位器异常

仅去掉硬件插件的激活检查仍不够：controller_manager 的通用限位器会因 joint2 实测 +0.012016 rad 超过 URDF 参考上限而在首次更新时抛异常，控制器进入 unconfigured，Action 接口却仍可能接受目标。Damiao 专用启动入口现关闭该通用限位器，硬件适配器继续检查目标范围、通信和电机故障；允许从范围外启动位置保持并插值回范围内，不允许继续扩大越界。变更需要重启 launch，发送目标前应确认机械臂控制器为 active。虚拟串口集成回归使用 joint2=+0.012016 rad 验证激活、保持、轨迹结果及停止。
