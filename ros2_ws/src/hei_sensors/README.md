# hei_sensors

`Cameras` 现在是 `rclpy.lifecycle.LifecycleNode`，默认名为 `/hei/cameras`（namespace 由 launch 设置）。三路相机 front、left_wrist、right_wrist 共用一套生命周期。

front 支持 `front_backend=realsense`，通过锁定的 pyrealsense2 SDK 采集 D435
640×480/15 FPS 彩色与深度，并将深度对齐到彩色图；腕部仍使用 OpenCV/MJPG。
真机默认 `cameras.yaml` 已启用。统一 setup 安装 SDK 至 `.deps/sensors-python`，
可单独 `bash scripts/setup.bash --camera-sdk`；运行前 source `scripts/env/workspace.bash`，
支持 `PYTHONNOUSERSITE=1`，不依赖用户 site-packages。

新增话题（相对 namespace）：

| 话题 | 消息 | 语义 |
| --- | --- | --- |
| `cameras/front/aligned_depth_to_color/image_raw` | Image/32FC1 | 米制深度，已对齐 RGB；零值为无效 |
| `cameras/front/camera_info` | CameraInfo | SDK 内参，尺寸与 RGB 一致，不虚构基座外参 |

RGB、对齐深度和 CameraInfo 共用 ROS 采集时间戳及 optical frame。SDK frameset 需满足
同一时间戳域、RGB/深度偏差 ≤50ms，启动不匹配帧在五秒内有界丢弃；采集等待可中断。
这不是硬件同步精度承诺，移动场景仍需考虑曝光和时延。
出厂 inverse-Brown 等模型只有零系数时按无畸变 plumb_bob 发布；非零时保留实际模型名，
当前 XYZ 工具会拒绝不支持的模型，不套用错误公式。mock 生成 1 米深度与测试内参，
frame_id 以 mock_ 开头。RealSense configure 只识别设备，activate 才启动流，
deactivate 停流、cleanup 释放资源。同一 front 不会同时被 OpenCV 和 SDK 打开。

| 状态转换 | 资源与行为 |
| --- | --- |
| configure → inactive | 校验参数，OpenCV 打开设备、SDK 识别 D435，创建发布器与暂停的定时器 |
| activate → active | 启用图像发布器，启动 SDK 流与采集线程，恢复图像定时器 |
| deactivate → inactive | 停止图像发布和采集线程，停止 SDK 流，清除旧帧；OpenCV 设备保持打开 |
| cleanup → unconfigured | 停止线程后关闭设备，销毁图像发布器和定时器；可修改配置再 configure |
| shutdown → finalized | 停止并释放资源，生命周期不再恢复；进程不因该转换自动退出 |
| 转换错误 | 尝试清理，成功则回到 unconfigured；线程无法停止则进入 finalized，禁止复用设备 |

构造函数只声明参数、初始化管理状态并创建诊断发布器，不打开相机，也不开始图像发布。独立 `ros2 run` 启动后是 unconfigured；真机 bringup 通过一次性的生命周期服务初始化器，确认 configure 和 activate 的应答后才报告相机 active；模拟入口保留 launch 的自动转换。后续手动 deactivate 不会被自动重新激活。

```bash
# 已 source ROS 和 workspace，ROS_DOMAIN_ID 与 bringup 一致
ros2 lifecycle get /hei/cameras
ros2 lifecycle set /hei/cameras deactivate
ros2 lifecycle set /hei/cameras activate

# 关闭并重新打开设备
ros2 lifecycle set /hei/cameras deactivate
ros2 lifecycle set /hei/cameras cleanup
# 此时允许 ros2 param set 修改 backend、fps 或 <camera>_device
ros2 lifecycle set /hei/cameras configure
ros2 lifecycle set /hei/cameras activate
```

独立启动和手动初始化：

```bash
ros2 run hei_sensors cameras --ros-args -r __ns:=/hei -p backend:=mock
# 在另一个终端，同一 domain
ros2 lifecycle set /hei/cameras configure
ros2 lifecycle set /hei/cameras activate
```

真实 RGB 的话题、类型、frame_id 与 sensor-data QoS 沿用原接口；mock frame_id 新增 mock_ 标记。
图像仅 active 时生成/发布；raw RGB Image 仍只在有订阅者时发送。
deactivate 会丢弃缓存，activate 后等待新帧。monitor 无需改订阅，最后一帧在 1 秒后过期。

`diagnostics` 是普通发布器，所有生命周期状态都可报告诊断，并在 values 中增加 `lifecycle_state`。inactive、unconfigured、finalized 不标为 online。生命周期 active 表示采集已启用，不保证每个设备一直健康；运行时某一路读取失败会报告逐相机 WARN，其余相机继续采集。失败设备需通过 deactivate/cleanup/configure/activate 重新打开，不在后台自动重开。

backend/fps/设备及采集格式参数在配置完成后禁止修改，先 cleanup 再修改，避免参数与实际资源不一致。OpenCV configure 要求三路设备都打开成功；任一路失败会回滚已打开的设备并留在 unconfigured。

`capture_api` 可选 `auto`（默认）或 `v4l2`；各路 `<camera>_fourcc` 默认空字符串，沿用设备协商格式。显式指定如 `MJPG` 时，设备必须支持且实际接受该格式，否则 configure 失败并回滚。`fps` 控制节点的采集循环/发布定时器，不承诺相机硬件原生帧率。

当前机器人的三相机共享 USB 2.0 总线。实测默认三路 YUYV 并发会超时，腕部相机使用 MJPEG 后可并发采集。相机专用参数文件在仓库 `ros2_ws/src/hei_bringup/config/cameras.yaml`（旧 `deploy/robot/cameras.yaml` 是兼容链接）：

```bash
# 在仓库根目录，已 source ROS 和 workspace
ros2 launch hei_bringup cameras.launch.py
# 此入口自动 configure / activate，只启动相机，不启动执行器
```

真实相机验收需显式启用，会打开三路真实设备：

```bash
HEI_TEST_REAL_CAMERAS=1 python3 -m pytest -q -s ros2_ws/tests/test_robot_cameras.py
```

普通测试默认跳过它。不要与其他相机采集进程同时执行。部分机器的用户 Python 包会覆盖系统 OpenCV/NumPy；本次机器人使用 `PYTHONNOUSERSITE=1` 隔离，未修改全局依赖。

退出信号可能先使 ROS context 失效，因此 main 的 finally 还执行独立资源清理。OpenCV 的驱动调用并非都可中断：停止线程有等待期限，读取卡住时转换失败，绝不在仍有读线程时 release 或重新打开同一设备。此时可能需要退出进程并检查驱动；不能保证硬件 open/read 的耗时上界。

真机 `robot.launch.py` 默认包含三相机，使用 OpenCV/V4L2 与上述 USB 带宽配置。可传 `cameras:=false` 禁用，或用 `camera_config_file` 覆盖参数文件。模拟输入可显式传 `camera_backend:=mock`。

相机 launch 会从 `hei_sensors` 安装前缀向上查找工作区的
`.deps/sensors-python`，将已安装的 SDK 加入相机进程的 `PYTHONPATH`。
因此旧终端中遗漏 SDK 搜索路径也能通过 launch 启动；仍需先通过
`bash scripts/setup.bash --camera-sdk` 准备 SDK。
