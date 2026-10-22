# hei_monitor

Rust 原生 ROS 2 节点，使用 rclrs 0.7、Tokio 和 Axum。`ament_cargo` 构建后安装单个 `monitor` 可执行文件；HTML/CSS/JS 编译进二进制，不需要 Python 服务或 Node.js。ROS 节点默认名为 `/hei/monitor`。

```text
src/main.rs    HTTP 监听、ROS executor 线程及信号退出
src/ros.rs     ROS 节点、host/port 参数及六个订阅
src/cache.rs   有界最新状态、图像缓存与新鲜度判断
src/http.rs    只读 HTTP 路由
web/           原有 HTML、CSS、JS
```

ROS executor 独立线程更新 `Arc<Mutex<Cache>>`，Axum 读取状态快照。系统指标在缓存锁外读取；JPEG 使用 `Bytes` 共享数据，HTTP 不在持锁期间等待网络。每路相机只保留最新一帧，单帧最多 2 MiB。关节/图像/里程计超过 1 秒陈旧，manager 诊断超过 2 秒失效；重复时间戳不会刷新关节或图像有效期，非有限反馈不计为有效状态，事件最多保留 100 条。

订阅使用相对名称，随 namespace 解析：

- `joint_states`：sensor_msgs/msg/JointState，sensor-data QoS。
- `diagnostics`：diagnostic_msgs/msg/DiagnosticArray，reliable。
- `base_controller/odom`：nav_msgs/msg/Odometry，sensor-data QoS。
- `cameras/{front,left_wrist,right_wrist}/image_raw/compressed`：sensor_msgs/msg/CompressedImage，sensor-data QoS。

HTTP 保持原接口：`GET /api/v1/status`、`GET /api/v1/cameras/{name}` 及静态页面。其他 HTTP 方法返回 405，未知路径返回 404，陈旧或缺失图像返回 503。响应禁用缓存。业务代码没有控制 publisher、service client 或 action client；rclrs 的参数服务、rosout 等仍属于 ROS 管理面。

`host`（默认 127.0.0.1）与 `port`（默认 8081）为启动时只读 ROS 参数，避免运行中修改参数而监听地址未改变。统一 bringup 的入口和参数保持兼容：

```bash
ros2 launch hei_bringup robot.launch.py
# 或单独启动，确保没有同名节点和端口冲突
ros2 run hei_monitor monitor --ros-args -r __ns:=/hei -p port:=8081
ros2 node info /hei/monitor
ros2 param get /hei/monitor port
```

构建环境除 Jazzy 外，需要 Rust、colcon-cargo、colcon-ros-cargo、cargo-ament-build，以及生成好的 ROS Rust 消息绑定。本机绑定依赖 rosidl_runtime_rs 0.6，因此固定使用 rclrs 0.7；rclrs 0.8 的 ros-env 绑定生成方式不同，不应只升级 crate 版本。rclrs 0.7 还会链接 test_msgs 的 C typesupport。系统可安装 `ros-jazzy-test-msgs`；没有 sudo 时，仓库根目录执行 `./scripts/setup/prepare-rust-ros.py`（根据 apt 元数据校验下载的 deb），然后在构建和启动的终端 `source scripts/env/ros-rust.bash`，使用被 git 忽略的 `.deps/` 本地依赖。该脚本不安装完整 ROS 或生成消息绑定。

Cargo.lock 固定解析结果；colcon 自动生成的 `.cargo/config.toml` 只保存本机 ROS 消息 crate 路径，不提交。

源码版 Jazzy 如果已有 `share/<消息包>/rust/Cargo.toml`，但没有 `rust_packages` 资源索引，`scripts/setup/prepare-rust-ros.py` 会在 `.deps/ros-rust-index/<发行版>` 创建兼容索引，指向原有消息 crate。随后 source `scripts/env/ros-rust.bash` 再执行 colcon；不修改 ROS 安装目录。该环境已在 Ubuntu 22.04/ARM64 的原生 Jazzy 上编译并通过 Rust 单元测试。

```bash
# 在仓库根目录，先 source ROS 环境；首次 colcon build 生成消息 crate 路径配置
cargo test --locked --manifest-path ros2_ws/src/hei_monitor/Cargo.toml \
  --target-dir ros2_ws/build/hei_monitor
python3 -m pytest -q ros2_ws/tests
```

SIGINT/SIGTERM 停止 HTTP、唤醒并退出 ROS executor，然后释放节点。监听端口被占用时启动立即报错。

上游参考：[rclrs 0.7 API](https://docs.rs/rclrs/0.7.0/rclrs/)、[ros2_rust](https://github.com/ros2-rust/ros2_rust)。
