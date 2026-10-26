# 右臂 OBB 抓取 Skill

实现位于 `ros2_ws/src/hei_agent_bridge/src/hei_agent_bridge/grasp/`。
当前只有一条抓取链路：

```text
服务器 GaP Agent → MHS → 机器人 hei_agent_bridge
  get_camera_observation(front)
    → perceive_grasp_target(observation_id, target)
    → plan_right_grasp_from_object(object_id)
    → 检查 validated
    → execute_right_grasp(plan_id)
```

执行顺序为张爪、移动到预抓取位姿、固定朝向下降、闭爪，随后停在抓取位置。
不会自动抬升、撤回或返回；后续运动使用通用手臂 skills 单独规划。
旧的一步式抓取接口、旧示例和旧格式计划已移除，更新后必须重新发现 catalog、重新感知和规划。

## 感知与规划

前置 RealSense 提供真实、对齐的 RGB-D；YOLOE-26s 按文字提示生成分割掩码，
将有效深度反投影为米制点云，再用 eye-to-hand 标定转换到 `a_right_base_link`。
过滤极端点后，用水平投影的最小面积矩形和竖直范围计算重力对齐 OBB。
OBB 只包围可见表面，不能推断遮挡面的完整尺寸。

本实现参考 GaP 的感知、几何和执行结构，没有接入其 DINO、VLM、SAM3 链路。
当前未融合腕部相机，也没有 cuRobo 绕障或物体抓牢检测。

OBB 感知使用 1280 推理尺寸、0.25 置信度阈值；不会增加相机原始分辨率。
目标为非空物体描述，最多 160 字符，建议简短英文；每次调用设置本次目标，
没有类别白名单。示例使用 `blue cube`，可直接修改图中的 `target`。
只接受真实 front 相机、正确光学坐标、对齐深度和无畸变内参；mock 图像不能生成可执行计划。

规划先尝试短轴及对称姿态、短轴 ±15° 候选，宽度允许时也尝试长轴。
在物体上方先对齐朝向，再生成下降路径；逐个求 IK，并用机器人、桌面模型做离散碰撞检查。
HEI TCP 的 +X 是接近方向；闭合轴和指尖延伸从 URDF、碰撞 STL 读取。
不自动把目标或其他可见物体加入障碍物模型。

## 配置与标定

- `config/right_eye_to_hand.npz`：D435 光学坐标到右臂基座的变换，单位米。
  2026-10-26 更新为 20 组筛选样本，拟合平移 RMS 3.92 mm、最大 6.89 mm，
  旋转 RMS 0.45°、最大 0.90°。这些是内部一致性残差，不是独立实测精度。
  完整记录见 `right_eye_to_hand_quality.json`，来源见 `right_grasp_source.json`。
- `config/right_grasp.json`：预抓取距离 0.08 m、浅抓深度 0.01 m、
  指尖与支撑面间隙 0.003 m、开口余量 0.004 m，以及原项目的桌面配置和碰撞例外。
  桌面参数须与现场一致，不能用于未经确认的其他环境。
- 运行时使用实时升降反馈更新碰撞场景，历史 `lift_joint_m: -0.149365` 不作为位置要求。
  相机与右臂的固定安装关系须保持一致；更换安装位置后要重新标定并独立验证。
- `min_base_z_m` 默认 `null`，没有隐含的基座 Z=0 高度下限；可以显式配置下限。
- 关节速度配置为 0.1 rad/s；夹爪沿用控制器的 5% 电流上限。
  开爪电机角 -4 rad 对应总开度约 0.08889 m，闭爪总开度为 0 m。

其他环境可提供自己的 skill JSON，保留 `backend=ros`、`named_poses`、
`gripper_max_gap_m`，在 `right_grasp` 中覆盖上述 profile 字段。
例如 `{"worker_python":"/path/to/python", "model_path":"/path/to/model.pt"}`。
覆盖 `scene` 时提供完整对象。修改后重启 bridge 并重新发现 catalog。

## 准备和启动

在机器人工程根目录检查已有计算环境和权重：

```bash
bash scripts/setup.bash --grasp
```

默认复用 `~/miniconda3/envs/rebotarm/bin/python` 和
`~/gitdir/rebot_grasp/models/yoloe-26s-seg.pt`，不安装到 ROS Python。
需要 NumPy、SciPy、OpenCV、MuJoCo（MjSpec API）、Ultralytics 和匹配的 Torch。
文本编码器 `mobileclip2_b.ts` 位于原模型目录的上级；worker 也在该目录运行。

机器人和相机已启动后，停止原 skill bridge，再启动一个实例：

```bash
export PYTHONNOUSERSITE=1
source ~/hei_jazzy_deps/install/setup.bash
source scripts/env/workspace.bash
source ros2_ws/install/setup.bash
ros2 launch hei_bringup agent_skills.launch.py hub_id:=lab-agent \
  skill_config_file:=$(ros2 pkg prefix hei_agent_bridge)/share/hei_agent_bridge/config/skills.ros.grasp.json
```

此 launch 只启动 bridge，不启动执行器。完整 `robot.launch.py` 会启动真机并包含升降回零，
不要为查看 catalog 重复启动。相机和串口继续由 `hei_sensors`、`hei_hardware` 独占；
不能同时运行原项目直接控制相机或串口的程序。
计算 worker 清除 ROS Python 环境路径，不导入 ROS、不打开相机和串口。

## Agent 使用

Agent 以 `--backend embedded --hub-id lab-agent --serve` 运行时，可依次提交：

```json
{"graph":"agent/src/hei_agent/examples/right_object_obb.json"}
{"graph":"agent/src/hei_agent/examples/right_obb_grasp_plan.json"}
```

第一条只感知，第二条运行感知和 IK/碰撞规划，均不发送电机命令。
规划通过且准备实际运动后，再提交：

```json
{"graph":"agent/src/hei_agent/examples/right_obb_grasp_execute.json"}
```

执行图重新感知和规划，仅在 `validated=true` 时执行，不自动重试。
`plan_only:true` 则只让 LLM 生成图，不运行任何视觉或运动 skill。

## 生命周期、执行与诊断

- 对象最多缓存 8 个，捕获后 60 秒过期；`object_id` 是机器人持有的不可变几何引用，
  不接受任意路径或 Agent 编造的几何。底盘、升降、左臂场景或模型/标定变化后须重新感知。
- 计划最多缓存 4 个，完成后 60 秒过期，执行前标记消费；失败或取消后不能重放。
  当前计划格式为 `schema_version=3`、`strategy=top_down_direct_ik`，旧格式须重新生成。
- 执行前检查右爪控制器的开度映射、堵转保持和电流上限，检查机器人起始状态。
  执行期间监控反馈新鲜度、底盘、左臂和升降；变化时取消当前 skill 的 ROS goal。
  本地 motion owner 不代替对外部遥操客户端的全局仲裁。
- ROS Action 使用 `FollowJointTrajectory` 和 `GripperCommand`。
  轨迹发送路径点、速度和加速度，验证与发送共用 `trajectory.JointTrajectory` 五次曲线。
  同向中间点连续通过，阶段端点和换向处停下；峰值速度上限为配置的 0.6 倍，
  加速度上限为配置的 1.2 倍，每段至少 0.1 秒。没有 jerk、动力学或连续碰撞保证。
- `damiao_gripper.yaml` 两侧启用闭合方向堵转保持：无进展且低速持续 1 秒后，
  Action 返回 `stalled=true`、`reached_goal=false`，维持限定电流。
  张爪受阻、取消和超时会停止；普通 `set_gripper_gap` 仍要求到达目标开度。
  `completed=true` 只代表序列完成，`grasp_verified` 始终为 false。
- 机器人 `log/hei_grasp/<id>/` 保存请求和 worker 日志。
  感知目录含 `rgb.jpg`、米制 `depth.npy`、`perception.json`、`object_mask.png`、
  `object_cloud.npy` 和 OBB 结果；规划目录含 `target.json`、`candidates.json`、计划及碰撞报告。
  `perception.json` 区分未检出、无分割和有效深度不足；`candidates.json` 记录每个候选失败原因。
  路径在机器人上，Agent 服务器不能直接打开。NPY 读取使用 `allow_pickle=False`。

原迁移与轨迹验证记录保存在机器人 `log/grasp-migration-acceptance/` 和
`log/trajectory-optimization/`；OBB 离线记录在 `log/obb-grasp-acceptance/`。
历史记录不是现版本真机抓取成功证明，旧格式计划不再支持执行或复用。
