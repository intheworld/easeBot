# 开发与部署脚本

从仓库根目录执行命令。日常开发使用 `remote-dev.py`；准备脚本只在首次安装依赖或修复环境时运行。

```text
scripts/
  remote-dev.py                    SSH / rsync：check、init、sync、watch、build
  env/                            用 source 加载，不安装依赖或启动节点
    workspace.bash                统一加载 Rust、隔离构建插件及以下两个环境
    ros-rust.bash                 本地 ROS typesupport 与 Rust 消息索引
    agent-bridge.bash             Agnes SDK 源码与 Python 运行依赖
  setup/                          一次性工作区依赖准备
    prepare-rust-ros.py            Rust 消息索引、缺少时解包 test_msgs
    prepare-agent-bridge.sh        锁定版本的 Agnes SDK 与运行依赖
  jazzy/                          Ubuntu 22.04 上源码安装 Jazzy 的专用工具
    fetch-source.py               下载官方 ROS 源码归档
    prepare-vendors.py            缓存 vendor 归档，记录 CMake 下载补丁
    build-source.bash             在干净环境中构建 ros_base 和 demo
    verify.py                     验证原生 C++ ↔ Python DDS 通信
    _archives.py                  下载、归档 URL 与 SHA256 公共函数
  remote/                         remote-dev.py 发送给 SSH 的 Bash 实现
    guard.bash                    检查目标标记、目录与 Git 保护
    init.bash                     初始化专用开发目录
    build.bash                    加载目标 ROS 环境并运行 colcon
```

## 日常开发

先按[远程开发文档](../docs/remote-dev.zh-CN.md)创建本机 `deploy/targets.local.json`，并初始化专用目标目录。此配置不会提交或同步。

```bash
python3 scripts/remote-dev.py robot sync
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

## 首次准备依赖

```bash
# 先 source ROS。只准备 .deps，不修改 ROS 安装或系统 Python。
python3 scripts/setup/prepare-rust-ros.py
./scripts/setup/prepare-agent-bridge.sh  # 使用 Agent Bridge 时
source scripts/env/workspace.bash
```

Rust 脚本使用现有消息 crate 创建兼容索引，优先采用 AMENT_PREFIX_PATH 中靠前的包；`test_msgs` 已存在时不下载，否则根据 apt 元数据校验 deb 后解包。Agent 脚本需要 Git 和 uv 或系统 Python pip，按锁文件为本机 Python/架构安装依赖。

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
