# easeBot · HEI ReBot Lift ROS 2

机器人本地 ROS 2 软件栈，参考 ROBOTIS AI Worker 的包边界组织。当前实现基于本机 ROS 2 Jazzy，默认使用 `ros2_control` 官方 mock hardware；不打开达妙电机串口，不执行真实回零。

Web 继续只做状态监控。控制通过独立 ROS 2 service、topic 和 action 完成。

## 软件包

```text
ros2_ws/src/
  hei_description/      HEI URDF、30 个原始网格、TF 与 RViz 配置
  hei_control/          ros2_control 双臂、夹爪、升降与全向底盘控制器配置
  hei_robot_manager/    控制器启停、反馈检查与 diagnostics
  hei_sensors/          三相机 Image / CompressedImage 与诊断
  hei_monitor/          Rust ROS 节点、状态缓存与 Axum 只读 Web
  hei_bringup/          统一 launch
```

这是原生 ROS 2 链路：`controller_manager → hardware_interface`，`joint_state_broadcaster → joint_states → robot_state_publisher → TF`。旧版非 ROS 原型与入口已清理，源码统一位于 `ros2_ws/src`。

## 构建与启动

本机已具备 Jazzy。Rust monitor 还需要 Rust 工具链、`colcon-cargo`、`colcon-ros-cargo`、`cargo-ament-build` 和 ROS Rust 消息绑定；详见 [监控包说明](ros2_ws/src/hei_monitor/README.md)。准备依赖后执行：

```bash
cd /home/cw/gitdir/easeBot
source /opt/ros/jazzy/setup.bash
# 系统未安装 ros-jazzy-test-msgs 时，可无 sudo 解包到工作区 .deps/
./scripts/prepare-rust-ros.sh
source scripts/ros-rust-env.bash
# 新机器可使用 rosdep 检查并安装 package.xml 中的依赖
# rosdep install --from-paths ros2_ws/src --ignore-src -r -y
colcon --log-base ros2_ws/log build --base-paths ros2_ws/src \
  --build-base ros2_ws/build --install-base ros2_ws/install
source ros2_ws/install/setup.bash

# 本机开发时限制发现范围，domain 需与控制终端保持一致
export ROS_DOMAIN_ID=71
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch hei_bringup robot.launch.py
```

只读页面：[http://127.0.0.1:8081](http://127.0.0.1:8081)。默认没有 HTTP 身份认证或 TLS，绑定回环地址；跨机器访问可用 SSH 转发。`Ctrl+C` 关闭本次 launch 的节点。

可选参数：

```bash
ros2 launch hei_bringup robot.launch.py rviz:=true
ros2 launch hei_bringup robot.launch.py monitor:=false
ros2 launch hei_bringup robot.launch.py namespace:=hei monitor_port:=8081
ros2 launch hei_bringup robot.launch.py camera_backend:=opencv
```

`opencv` 只切换相机，电机仍为 mock。当前默认设备为 `/dev/hei_front_camera`、`/dev/hei_left_wrist_camera`、`/dev/hei_right_wrist_camera`，可用相机节点参数配置。相机不能同时被旧 HEI host 打开。相机内外参未验证，因此当前不发布虚构的已标定 CameraInfo。

RViz 配置默认使用 `/hei/robot_description`；自定义 namespace 时需在 RViz 中选择相应 description topic。没有定位系统，默认不发布 `map → odom` 或 `odom → base_footprint`。

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

双臂、两只夹爪和升降均提供 `/<namespace>/<group>_controller/follow_joint_trajectory`。双臂每侧六关节；夹爪以主指的线位移 m 表示，另一指 mimic；升降 m，上限位参考 0、向下为负。它们与旧版夹爪电机 rad、升降 mm 的 LeRobot 接口不同，需要显式转换，不能直接对接旧动作数组。

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

在 source 了 Jazzy、`scripts/ros-rust-env.bash` 和本 workspace 的终端运行：

```bash
cargo test --locked --manifest-path ros2_ws/src/hei_monitor/Cargo.toml \
  --target-dir ros2_ws/build/hei_monitor
python3 -m pytest -q ros2_ws/tests
```

已在本机完成六包 colcon 构建、3 个 Rust 缓存单元测试、2 个模型测试和 1 个 ROS 2 端到端测试；Rust monitor 另验证了端口冲突失败与 SIGTERM 正常退出。端到端覆盖：控制器默认 inactive、未启用时拒绝轨迹、显式启用后 action 成功、joint_states/TF/图像发布、底盘超时归零、停止后 inactive、Web 写方法拒绝。测试启动独立 ROS domain，结束时关闭节点。不是实机硬件测试，也未验证实时调度。

模型来源和 mock 限值修改见 [模型来源记录](/home/cw/gitdir/easeBot/ros2_ws/src/hei_description/SOURCE.md)，上游许可随包保留。原始几何与运动学来自 HEI 仓库；零速度/力限位被替换为用于 mock 的临时值，不能作为实机额定值。

下一阶段实现 `hei_hardware` 的达妙 U2CAN `hardware_interface::SystemInterface`，届时再引入必要的 C++。管理节点分别使用关节反馈超时（默认 1 秒）和控制器查询超时（默认 5 秒），可用 ROS 参数调整。控制器查询耗时不会被误当作电机关节反馈中断。

本次不创建不可用的硬件占位插件；没有连接真机的 launch 参数。MoveIt、导航和 VR/LeRobot 接入仍为后续模块。

[ROS 2 架构与范围](/home/cw/gitdir/easeBot/docs/ros2-architecture.zh-CN.md) · [监控节点说明](/home/cw/gitdir/easeBot/ros2_ws/src/hei_monitor/README.md)
