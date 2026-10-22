# hei_sensors

`Cameras` 现在是 `rclpy.lifecycle.LifecycleNode`，默认名为 `/hei/cameras`（namespace 由 launch 设置）。三路相机 front、left_wrist、right_wrist 共用一套生命周期。

| 状态转换 | 资源与行为 |
| --- | --- |
| configure → inactive | 校验 backend/fps，OpenCV 打开三路设备，创建生命周期图像发布器与暂停的发布定时器 |
| activate → active | 启用图像发布器，启动 OpenCV 采集线程，恢复图像定时器 |
| deactivate → inactive | 停止图像定时器和发布器，通知并等待采集线程停止，清除旧帧；设备保持打开 |
| cleanup → unconfigured | 停止线程后关闭设备，销毁图像发布器和定时器；可修改配置再 configure |
| shutdown → finalized | 停止并释放资源，生命周期不再恢复；进程不因该转换自动退出 |
| 转换错误 | 尝试清理，成功则回到 unconfigured；线程无法停止则进入 finalized，禁止复用设备 |

构造函数只声明参数、初始化管理状态并创建诊断发布器，不打开相机，也不开始图像发布。独立 `ros2 run` 启动后是 unconfigured；默认 bringup 使用 launch 的 `LifecycleNode(autostart=True)`，自动完成一次 configure/activate。后续手动 deactivate 不会被自动重新激活。

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

图像话题、类型、frame_id 与 sensor-data QoS 保持原接口。图像仅 active 时生成/发布；raw Image 仍只在有订阅者时发送。deactivate 会丢弃缓存，activate 后等待新帧。monitor 无需改订阅，最后一帧在 1 秒后过期。

`diagnostics` 是普通发布器，所有生命周期状态都可报告诊断，并在 values 中增加 `lifecycle_state`。inactive、unconfigured、finalized 不标为 online。生命周期 active 表示采集已启用，不保证每个设备一直健康；运行时某一路读取失败会报告逐相机 WARN，其余相机继续采集。失败设备需通过 deactivate/cleanup/configure/activate 重新打开，不在后台自动重开。

backend/fps/设备及采集格式参数在配置完成后禁止修改，先 cleanup 再修改，避免参数与实际资源不一致。OpenCV configure 要求三路设备都打开成功；任一路失败会回滚已打开的设备并留在 unconfigured。

`capture_api` 可选 `auto`（默认）或 `v4l2`；各路 `<camera>_fourcc` 默认空字符串，沿用设备协商格式。显式指定如 `MJPG` 时，设备必须支持且实际接受该格式，否则 configure 失败并回滚。`fps` 控制节点的采集循环/发布定时器，不承诺相机硬件原生帧率。

当前机器人的三相机共享 USB 2.0 总线。实测默认三路 YUYV 并发会超时，腕部相机使用 MJPEG 后可并发采集。相机专用参数文件在仓库 `deploy/robot/cameras.yaml`：

```bash
# 在仓库根目录，已 source ROS 和 workspace
ros2 run hei_sensors cameras --ros-args -r __ns:=/hei \
  --params-file deploy/robot/cameras.yaml
# 在另一终端手动 configure / activate；这条命令不启动控制器
```

真实相机验收需显式启用，会打开三路真实设备：

```bash
HEI_TEST_REAL_CAMERAS=1 python3 -m pytest -q -s ros2_ws/tests/test_robot_cameras.py
```

普通测试默认跳过它。不要与其他相机采集进程同时执行。部分机器的用户 Python 包会覆盖系统 OpenCV/NumPy；本次机器人使用 `PYTHONNOUSERSITE=1` 隔离，未修改全局依赖。

退出信号可能先使 ROS context 失效，因此 main 的 finally 还执行独立资源清理。OpenCV 的驱动调用并非都可中断：停止线程有等待期限，读取卡住时转换失败，绝不在仍有读线程时 release 或重新打开同一设备。此时可能需要退出进程并检查驱动；不能保证硬件 open/read 的耗时上界。
