# 开发与部署脚本

构建前统一执行 `bash scripts/setup.bash`；日常同步和远程编译使用 `remote-dev.py`。以下命令在仓库根目录运行，入口脚本本身不依赖当前目录。

服务器只运行 Agent 时使用 `bash scripts/setup.bash --agent-server`：创建/复用 `agent/.venv`，
安装本地共享 `packages/hei-discovery` 和 Agent 的 LLM/MHS 依赖，无需 ROS 环境。
随后 `agent/.venv/bin/hei-agent --backend embedded --serve` 常驻接收 JSON 行任务。
机器人默认 setup 同样安装共享发现包与 zeroconf 到 `.deps/agent-python`，不安装 Avahi 等系统服务。
默认 setup 还准备 D435 SDK 到 `.deps/sensors-python`；单独准备可用
`bash scripts/setup.bash --camera-sdk`，不打开相机、不安装内核模块或修改系统 Python。

```text
scripts/
  setup.bash                      统一依赖准备入口
  remote-dev.py                    SSH / rsync：check、init、sync、watch、setup、build
  env/                            用 source 加载，不安装依赖或启动节点
    workspace.bash                统一加载 Rust、隔离构建插件及以下两个环境
    ros-rust.bash                 本地 ROS typesupport 与 Rust 消息索引
    agent-bridge.bash             Agnes SDK 源码与 Python 运行依赖
  setup/                          内部依赖准备步骤，由 setup.bash 调用
    prepare-build-tools.py         复用或隔离安装 Rust colcon 插件
    build-requirements.txt          已验证的构建插件版本
    prepare-rust-ros.py            Rust 消息索引、缺少时解包 test_msgs
    prepare-agent-bridge.sh        锁定版本的 Agnes SDK 与运行依赖
    prepare-motorbridge.bash       下载锁定的 motorbridge 与 Cargo 依赖
  jazzy/                          Ubuntu 22.04 上源码安装 Jazzy 的专用工具
    fetch-source.py               下载官方 ROS 源码归档
    prepare-vendors.py            缓存 vendor 归档，记录 CMake 下载补丁
    build-source.bash             在干净环境中构建 ros_base 和 demo
    verify.py                     验证原生 C++ ↔ Python DDS 通信
    _archives.py                  下载、归档 URL 与 SHA256 公共函数
  remote/                         remote-dev.py 发送给 SSH 的 Bash 实现
    guard.bash                    检查目标标记、目录与 Git 保护
    init.bash                     初始化专用开发目录
    environment.bash              加载目标 ROS 与工作区依赖环境
    build.bash                    运行 colcon
```

## 日常开发

先按[远程开发文档](../docs/remote-dev.zh-CN.md)创建本机 `deploy/targets.local.json`，并初始化专用目标目录。此配置不会提交或同步。

```bash
python3 scripts/remote-dev.py robot sync
python3 scripts/remote-dev.py robot setup  # 首次或依赖更新后，在目标机器执行统一入口
python3 scripts/remote-dev.py robot build
python3 scripts/remote-dev.py robot build --packages hei_monitor
python3 scripts/remote-dev.py robot watch
```

`sync`/`watch` 只推送源码，`build` 只编译此前同步的源码。`remote/` 中的文件由 Python 入口组合执行，不单独调用。

在构建或启动所在的机器加载环境：

```bash
source /opt/ros/jazzy/setup.bash  # 源码安装时替换为对应 setup.bash
source scripts/env/workspace.bash
source ros2_ws/install/setup.bash  # 构建完成后
```

`workspace.bash` 不代替 ROS underlay，也不自动加载应用 install。需要时可单独 source `ros-rust.bash` 或 `agent-bridge.bash`。`env/` 文件与当前工作目录无关；直接执行它们不会改变父终端的环境。
加载后，colcon 默认通过 `CMAKE_COMMAND=/usr/bin/cmake`、`CTEST_COMMAND=/usr/bin/ctest`
使用系统原生工具，避免 PATH 中 pip/Conda 的 CMake 启动器因 Python 模块不可见而失败。
已有显式覆盖值会保留；不会卸载用户工具或修改终端启动配置。

## 首次准备依赖

每台参与编译的机器执行同一个入口：

```bash
source /opt/ros/jazzy/setup.bash  # 机器人替换为 ~/hei_jazzy_deps/install/setup.bash
bash scripts/setup.bash
source scripts/env/workspace.bash
# 然后执行 README 中的 colcon build
```

默认依次准备构建插件、ROS Rust 索引/缺少的 test_msgs、motorbridge 锁定依赖、Agent Bridge SDK。只运行核心功能时可使用 `bash scripts/setup.bash --without-agent-bridge`。可重复执行，任何步骤失败立即返回非零码，后续步骤不运行。准备脚本不会改变父终端环境，所以完成后仍要 source 环境脚本。

前置环境是完整 Jazzy underlay（含控制器依赖及生成好的 Rust 消息绑定）、Rust 工具链、C++ 编译器、CMake、Git、Python 和基础 colcon；脚本先检查这些条件，再安装工作区依赖。缺少插件时，使用 uv 或当前 Python 的 pip 安装到 `.deps/build-python`；Agent 依赖也只进入 `.deps/`。已有匹配版本的构建插件直接复用，不替换系统 Python。准备过程需要网络，Cargo 缓存位于当前用户的 Cargo 目录。旧 Agent SDK 若是不含 Git 元数据的源码目录，会先在临时目录获取固定版本，成功后把旧目录保存在 `.deps/agnes-harness-before-setup.*` 再切换；获取失败保留原目录。

统一入口不负责安装操作系统软件包、完整 ROS 或 Rust 工具链；Ubuntu 22.04 源码安装 Jazzy 仍使用下面的专用流程。正常构建不隐式安装依赖、不执行 sudo、不启动节点。`setup/prepare-*` 保留为内部步骤，只有排查单项依赖时才需要直接调用。

远程也不用 SSH 后逐个运行准备脚本：先 `remote-dev.py robot sync`，再 `remote-dev.py robot setup`，最后 `remote-dev.py robot build`。setup 使用 targets 配置中的 ROS underlay，支持 `--without-agent-bridge`；不会隐式同步或编译。

## Jazzy 源码安装

仅在需要从源码安装 Jazzy 的目标机器运行，默认工作空间为 `~/ros2_jazzy`。已有 Jazzy 的日常开发无需执行：

```bash
PYTHONNOUSERSITE=1 python3 scripts/jazzy/fetch-source.py
PYTHONNOUSERSITE=1 python3 scripts/jazzy/prepare-vendors.py
bash scripts/jazzy/build-source.bash
export PYTHONNOUSERSITE=1
source ~/ros2_jazzy/install/setup.bash
python3 scripts/jazzy/verify.py
```

下载脚本缓存归档并记录 SHA256；vendor 脚本保留原文件和差异记录。构建脚本清除继承环境，以 Release、顺序包构建、每包两个任务编译基础 ROS。工程控制依赖的版本及构建方式见[部署文档](../docs/remote-dev.zh-CN.md#jazzy-工程环境与验证)。

## 验证脚本改动

```bash
python3 -m pytest -q ros2_ws/tests/test_remote_dev.py ros2_ws/tests/test_scripts.py
```

这些用例使用临时目录、模拟 SSH 与真实 rsync，不连接机器人，不下载依赖。工程联合测试仍按各包 README 和部署文档运行。

2026-10-22 整理后验证：本地脚本测试 10 项通过；机器人原生 Jazzy 上 8 个工程包编译成功，全量测试 19 passed、1 skipped（真实相机用例默认跳过），C++ ↔ Python DDS 验收通过。机器人旧脚本副本保存在 `.deps/scripts-before-layout-*`，回归日志为 `ros2_ws/log/script-cleanup-tests.log`。

2026-10-23 统一入口验证：18 项脚本测试通过，覆盖失败即停、错误环境不安装、可选 SDK、旧 SDK 备份/下载失败保留及远端目录保护。本机与 ARM64 机器均完成默认 setup；机器人随后通过远程 build 编译 9 个包。

右臂抓取复用机器人已有计算环境：`bash scripts/setup.bash --grasp`。
此独立入口仅检查 Python 计算依赖和权重，不打开硬件、不安装到 ROS 环境。
详见 [右臂抓取说明](../docs/right-grasp.zh-CN.md)。

通用末端运动依赖检查：`bash scripts/setup.bash --arm-motion`。
复用已有 IK/MuJoCo 计算环境，无需检测器或权重；详见[末端运动说明](../docs/arm-motion.zh-CN.md)。
