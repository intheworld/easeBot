# 末端位姿规划与执行 Skill

当前支持右臂，控制点是 URDF 中的 `a_right_tcp`。不依赖相机、YOLO 权重或手眼标定。
规划使用现有右臂 IK、完整机器人/桌面模型；执行仍由 `ros2_control` 的
`FollowJointTrajectory` 完成，skill 不直接打开串口。

## 四个接口

| Skill | 必需参数 | 行为 |
|---|---|---|
| `get_arm_pose` | `side`, `frame_id` | 根据新鲜关节反馈和 URDF FK 返回当前 TCP 位姿，不要求执行器运动 |
| `plan_arm_position` | `side`, `frame_id`, `x_m`, `y_m`, `z_m`, `path_mode` | 规划位置，保持规划时的 TCP 朝向，不运动 |
| `plan_arm_pose` | 上述参数加 `qx`, `qy`, `qz`, `qw` | 规划完整位置与姿态，不运动 |
| `execute_arm_plan` | `plan_id` | 执行已验证计划，保持当前夹爪状态；不自动张爪、闭爪或返回起始姿态 |

- `side` 当前只能是 `right`；左臂请求会拒绝，不会静默转换。
- 位置单位为米；四元数顺序为 **xyzw**，模长须接近 1（误差不超过 0.001）。
- `frame_id` 支持 `a_right_base_link`、`base_link`、`base_footprint`。
  后两者通过 URDF 和实时升降位置转换，属于机器人自身坐标系；不代表地图坐标。
  不接受 `world`、`map`、`odom` 或未声明坐标系，不依靠猜测或默认 TF。
- `path_mode=joint`：求目标 IK，按关节五次曲线移动；TCP 轨迹通常不是直线。
- `path_mode=linear`：按直线 TCP 路径和姿态插值逐点求 IK；不做绕障搜索，
  遇到不可达点、IK 跳支或碰撞会拒绝。
- 保持朝向仍是位姿约束 IK，有些 XYZ 虽然机械臂能到达，但保持指定朝向时可能无解。

规划结果包含 `plan_id`、`validated`、`validation_status`、`plan_ref`、完整目标位姿、
`nominal_duration_sec`。只有 `validated=true` 的计划才允许执行。
执行结果包含实测位姿、`reached_goal`、`position_error_m`、`orientation_error_rad`。
反馈 FK 与目标误差超过 5 mm 或 3° 时不会报告到位。
这只能验证基于关节编码器和 URDF 的 TCP 位置，不代表外部测量精度。

## 启用

机器人依赖检查入口：

```bash
bash scripts/setup.bash --arm-motion
```

复用 `~/miniconda3/envs/rebotarm/bin/python` 的 NumPy、SciPy、OpenCV、MuJoCo，
不检查或加载检测器和权重，也不操作硬件。计算进程与 ROS Python 环境分离。

当前已经使用 `skills.ros.grasp.json` 时，重启 bridge 后会自动同时发布这四个接口。
通用运动继承该抓取配置中的工作进程、速度和场景；`arm_motion` 配置字段可显式覆盖。

仅启用通用运动、不需要抓取检测器时：

```bash
export PYTHONNOUSERSITE=1
source ~/hei_jazzy_deps/install/setup.bash
source scripts/env/workspace.bash
source ros2_ws/install/setup.bash
ros2 launch hei_bringup agent_skills.launch.py hub_id:=lab-agent \
  skill_config_file:="$(ros2 pkg prefix hei_agent_bridge)/share/hei_agent_bridge/config/skills.ros.arm.json"
```

它只启动 skill bridge；机器人控制器由现有 bringup 提供。请替换当前 bridge，
不要同时运行两个相同 device_id 的 bridge。

独立配置示例为 `config/arm_motion.json`：`worker_python`、`joint_speed_rad_s`、`scene`。
默认速度 0.1 rad/s，与当前 ROS 配置上限一致。桌面数据和碰撞例外沿用已提供的参考场景，
不是任意环境通用模型；场景变化时更新 `scene`。
`scene.lift_joint_m` 和左右臂/夹爪姿态在规划时由当前反馈覆盖。
实际左臂、升降、夹爪和轮子反馈需新鲜，计划/执行时相应部件必须静止。
单纯读取 `get_arm_pose` 只要求右臂和升降的反馈。

## 从 Agent 测试

先在服务器 Agent 的 JSON 输入终端确认 catalog：

```json
{"list_skills":true}
```

读取当前 TCP 位姿：

```json
{"graph":"agent/src/hei_agent/examples/arm_pose.json"}
```

只规划保持当前 TCP 位置的 smoke test，不发送运动命令：

```json
{"graph":"agent/src/hei_agent/examples/arm_position_plan.json"}
```

`arm_position_execute.json` 在同一目标下规划并发送 ROS goal，验证执行链路。
**这是实际执行请求**；目标取规划前的当前位姿，不包含主动平移，但控制器仍会接管目标。
它与所有执行图一样会重新读取、重新规划，不会执行先前只规划图的那份计划。

任意目标建议先通过 LLM 生成图，再检查保存的参数。把实际位置填入任务，明确坐标系、
朝向方式与路径方式，例如在 JSON 中提交：

```text
{"task":"仅规划右臂运动：在 a_right_base_link 坐标系下，将 TCP 移到我给定的 x、y、z 米位置，保持当前朝向，path_mode=linear，不执行运动"}
```

上面是任务表述模板，使用前必须把 x/y/z 替换成实际数值。LLM 不应自行编造目标。
`plan_only:true` 只生成 GaP 图，不运行 IK/碰撞规划；“仅规划”任务则应调用
`plan_arm_position`/`plan_arm_pose` 并在返回结果后结束，不调用 `execute_arm_plan`。
完整姿态任务额外提供 xyzw 四元数；不要将 RPY 数值直接当作四元数。

也可以复制 `arm_position_plan.json`，将 plan 节点的 x/y/z `$ref` 替换为目标数值；
完整姿态则把该节点的 skill 改为 `plan_arm_pose` 并添加四元数四个参数。
CLI 会先按发现的 catalog 校验整个模板，再绑定版本，随后才运行。

## 执行、取消与实现边界

- 每个组件最多缓存 4 个计划，规划完成后 60 秒过期；重启 bridge 后失效。
- 计划执行前即标记消费；取消、故障或部分执行后不能重放，必须重新规划。
- 执行前核对起始右臂/夹爪、左臂/夹爪、升降和轮子位置；配置、URDF、网格变化使绑定失效。
  起始关节允许现有的 0.02 rad 反馈偏差，发出的首点使用最新反馈；真实跟踪和标定误差不由离散模型检查覆盖。
- 执行期间监控底盘、升降、左臂和两侧夹爪是否改变，取消只作用于该 skill 发出的 ROS goal。
- 与抓取共用 SkillSet 的运动占用和终止确认；不是对外部遥操/其他 ROS 客户端的全局仲裁。
- 共享 C2 连续五次轨迹：同向中间点连续通过，阶段起止速度/加速度为零；关节换向点速度为零。
- 全曲线峰值速度不超过配置速度的 60%，加速度不超过配置速度 × 1.2 rad/s²；碰撞检查与 ROS 下发使用同一曲线。
- 不越过各段关节端点范围；未限制 jerk，也不包含真实机械振动模型。
- `log/hei_arm/<plan_id>/request.json` 和 `result.json` 位于机器人启动目录，保存请求、计划及校验。
- 采用离散运动学碰撞检查，不含连续碰撞、动力学、外部未建模障碍或被夹持物体。
- 规划超时 90 秒、执行超时 300 秒；预计超出执行预算的路径会拒绝。

公共运行逻辑位于 `hei_agent_bridge/motion.py`；通用位姿与 worker 在 `arm/`，
抓取在 `grasp/`。两者共享计算进程清理、场景监控、原碰撞策略和同一 ROS 轨迹发送接口。

## 验证记录

2026-10-25：机器人 Jazzy 编译通过；149 项 Agent 回归、80 项末端/抓取/底盘反馈/脚本测试、
9 项模拟 ROS/MHS 集成测试通过。通用 joint 路径（右臂基座）和 linear 路径
（base_footprint）使用原抓取验证过的短段完成真实 MuJoCo 离线检查，均无碰撞；
原抓取计划的 1534 个采样回归通过。记录在机器人 `log/arm-motion-acceptance/offline.json`。

独立 arm 配置的 MHS 声明和三个示例图校验通过；使用真实 ROS 反馈读到 TCP
约 `[0.260310, -0.000827, 0.178000]` m，并完成保持当前位置的规划，
`validated=true`。父进程为 ROS Python，计算子进程为独立环境；没有加载 YOLO 或手眼文件，
没有调用执行接口或发送真实 ROS goal。记录在 `log/arm-motion-acceptance/readonly.json`。
