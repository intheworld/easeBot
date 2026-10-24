# hei_monitor

Rust 原生 ROS 2 节点，使用 rclrs 0.7、Tokio 和 Axum。`ament_cargo` 构建后安装单个 `monitor` 可执行文件；HTML/CSS/JS 编译进二进制，不需要 Python 服务或 Node.js。ROS 节点默认名为 `/hei/monitor`。

```text
src/main.rs    HTTP 监听、ROS executor 线程及信号退出
src/ros.rs     ROS 节点、订阅与只读状态查询
src/hardware.rs  四总线缓存、升降/夹爪反馈和状态判断
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

HTTP 保持原接口：`GET /api/v1/status`、`GET /api/v1/cameras/{name}` 及静态页面。其他 HTTP 方法返回 405，未知路径返回 404，陈旧或缺失图像返回 503。响应禁用缓存。真机模式只调用 ListControllers / ListHardwareComponents 查询服务；没有控制 publisher、控制 service client 或 action client；rclrs 的参数服务、rosout 等仍属于 ROS 管理面。

`host`（默认 127.0.0.1）与 `port`（默认 8081）为启动时只读 ROS 参数，避免运行中修改参数而监听地址未改变。正式真机入口默认启动 monitor，模拟模式使用独立入口：

```bash
# 默认连接全部真机执行器，monitor 随启动；只读检查整机可加 read_only:=true
ros2 launch hei_bringup robot.launch.py
# 模拟环境
ros2 launch hei_bringup mock_robot.launch.py
# 或单独启动，确保没有同名节点和端口冲突
ros2 run hei_monitor monitor --ros-args -r __ns:=/hei -p backend:=damiao -p port:=8081
ros2 node info /hei/monitor
ros2 param get /hei/monitor port
```


## 真机状态

`backend:=damiao` 订阅 `{left,right,base,lift}_dynamic_joint_states`，每条总线独立检查时间戳、消息结构、有限数值和 1 秒新鲜度。使用 DynamicJointState 保留被普通 JointState 的 URDF 过滤器排除的夹爪原始电机。重复时间戳不能刷新有效期。过期或硬件失活的总线在 `state` 中输出 null，其他在线部件保留有效读数。

每条总线以约 2 Hz 查询各自 manager 的 `list_controllers` 和 `list_hardware_components`；单次请求等待最多 800 ms，超时或调用失败后重建客户端清理挂起请求（服务未出现时保留客户端，避免反复触发发现），3 秒未得到有效响应就标记 unknown。四条总线独立查询，一处失联不会阻塞其他总线或 HTTP。polling 随 monitor 退出停止，不发送任何运动、模式、回零或使能指令。

API 的 `robot.data.buses` 给出每条总线的连接、新鲜度、硬件生命周期和控制器状态；`lift` 给出 IO、上下限位与阶段；`homed` 仅在有效反馈下给出布尔值，否则为 null。`hardware_connected` 要求四条真实硬件资源及反馈均有效，包含有效升降 IO。`valid` 还要求控制器查询新鲜。`READY`/`controllers_ready` 表示所有运动控制器 active 且升降已回零。

控制器 active 不等于电机已使能：整机 `enabled` 保持 null；目前可确认的是夹爪导出的 enabled 反馈，显示于 `grippers.<side>.enabled`。机械臂和底盘尚未导出逐电机使能状态，页面明确标注未知。升降未回零时的高度仍是临时相对值，应结合 homed 判断；物理急停反馈未接入。

夹爪显示两指总开口，真机 launch 从实际 gripper controller 配置向 monitor 传入左右 `gap_at_zero_m`、`gap_per_motor_rad_m`。单独运行可通过 `left_gap_at_zero_m` / `left_gap_per_motor_rad_m`（右侧同理）指定；默认沿用当前模型名义换算，不作为实测标定。夹持力 N 保持未知；电机反馈力矩使用 N·m。

真机 launch 参数 `monitor` 默认 true，`monitor_host` 默认 0.0.0.0（监听全部 IPv4 网卡，局域网可访问），`monitor_port` 默认 8081。Web 端口被占用会使 monitor 启动失败，但不会关闭正在运行的执行器。局域网直接访问 `http://seeed.local:8081/` 或机器人的 IP 地址；如需仅限本机，可传 `monitor_host:=127.0.0.1`。也可使用 SSH 转发：

```bash
ssh -N -L 8081:127.0.0.1:8081 seeed@seeed.local
```

浏览器访问 `http://127.0.0.1:8081`。相机订阅兼容现有 hei_sensors；真机入口默认启动 OpenCV 相机并自动完成生命周期配置与激活；缺失或陈旧图像仍显示不可用，不伪装成 mock 画面。

构建环境除 Jazzy 外，需要 Rust、colcon-cargo、colcon-ros-cargo、cargo-ament-build，以及生成好的 ROS Rust 消息绑定。本机绑定依赖 rosidl_runtime_rs 0.6，因此固定使用 rclrs 0.7；rclrs 0.8 的 ros-env 绑定生成方式不同，不应只升级 crate 版本。rclrs 0.7 还会链接 test_msgs 的 C typesupport。系统可安装 `ros-jazzy-test-msgs`；没有 sudo 时，仓库根目录执行统一入口 `bash scripts/setup.bash`（根据 apt 元数据校验下载的 deb），然后在构建和启动的终端 `source scripts/env/ros-rust.bash`，使用被 git 忽略的 `.deps/` 本地依赖。该脚本不安装完整 ROS 或生成消息绑定。

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
