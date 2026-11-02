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

复用 `~/miniconda3/envs/rebotarm/bin/python` 的 NumPy、SciPy、OpenCV、python-fcl，
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

## 右臂搬运与放置

抓取、回零之外，当前还支持以下搬运接口，均使用原 ROS 轨迹控制器：

| Skill | 用途 |
|---|---|
| `plan_arm_offset` | 按指定坐标轴相对移动 TCP，保持朝向，规划直线路径 |
| `create_arm_path` | 建立机器人端路径，返回 `path_id` |
| `append_arm_waypoint` | 逐个添加完整末端位姿，位置米、四元数 xyzw |
| `plan_arm_path` | 冻结路径，检查所有段，返回普通 `plan_id` |
| `get_carried_object` | 查看逻辑携带状态及已放置物数量 |
| `plan_place_object` | 规划抬升、搬运、下降、松爪及撤回全过程 |
| `execute_place_plan` | 执行放置，松爪确认后按真实反馈重新检查撤回路径 |

`plan_arm_offset` 的 `frame_id` 除上述三个基座坐标系外，还支持 `a_right_tcp`，
表示沿规划时的工具轴移动；结果位姿仍以 `a_right_base_link` 返回。
多路点路径使用同一显式坐标系，可选 `joint` 或 `linear`；最多 32 个点，
路径缓存最多 8 条、300 秒过期，尝试规划后冻结。规划不会发送电机命令。
它不是自动绕障搜索；无法通过指定路点时会拒绝，不会偷偷换路。

### 从 Agent 输入参数

交互 JSON 支持 `inputs`，命令行对应 `--inputs '{...}'`。
缺少或类型错误的参数会在执行图前拒绝，不会猜测放置坐标。
先只规划沿基座向上 5 cm：

```json
{"graph":"agent/src/hei_agent/examples/right_arm_offset_plan.json","inputs":{"frame_id":"a_right_base_link","dx_m":0,"dy_m":0,"dz_m":0.05}}
```

确认规划后，使用相同参数提交 `right_arm_offset.json`，会重新规划再执行。
多路点示例为 `right_arm_path_plan.json` / `right_arm_path.json`，需要 `frame_id` 和
两个点的 `p1_x_m/p1_y_m/p1_z_m/p1_qx/p1_qy/p1_qz/p1_qw`、`p2_...`。
这些接口通过 `path_id` 管理路点，MHS 外部契约仍是有类型的标量，不使用任意 JSON 字符串。

### 抓取后的携带状态

`execute_right_grasp` 在闭爪前在内存记录“闭合中，状态不确定”；闭爪动作完成后，
使用最终（包括腕部矫正后的）OBB 和实测关节 FK 记录 `T_tcp_object`。
之后的通用手臂运动、相对移动、多路点和回零都会把携带物作为 TCP 上的盒形几何加入检查。
`physical_capture_verified=false` 始终表示没有物理抓牢检测；动作完成不是物体在手的证据。

默认沿用可见表面 OBB，并在盒子各面增加 3 mm 几何余量。这个余量不能推断遮挡部分。
已知完整物体尺寸时，可通过 `set_carried_object_geometry` 明确声明 TCP 坐标下的
中心 `x_m/y_m/z_m`、朝向 `qx/qy/qz/qw`、三个 `size_*_m`，并同时提供人工确认
`observed_attached=true`、`geometry_confirmed=true`。该接口也可声明升级前已在夹爪中的物体。
声明会使旧计划失效，不会运动。

携带和已放置几何只保留在 bridge 的当前进程内存中。旧的
`log/hei_payload/state.json` 不再读写；重启不会恢复历史持物状态或因此锁住新任务。
`status=empty` 表示当前没有任务持物模型，不是传感器证明夹爪物理上为空。
重启后若实际仍持物，需要重新感知/声明物体几何再规划带物搬运，不能恢复未知的旧变换。

实时夹爪反馈达到已配置的全开位置时自动清除当前持物模型。小幅角度变化不会进入永久
`unknown`；闭合/释放中断的当前任务仍可报告不确定，但明确的张爪命令可正常执行，
反馈确认全开后即可开始新任务。电机控制和轨迹/碰撞检查的限值不变。

查看当前任务模型：

```json
{"graph":"agent/src/hei_agent/examples/right_carry_state.json"}
```

读取新鲜开度和腕部 RGB，必要时对已知当前物体做标定投影和分割一致性检查：

```json
{"graph":"agent/src/hei_agent/examples/right_gripper_observation.json"}
```

`observe_gripper` 不运动、不开闭夹爪；返回派生的 `holding_state`（`open` / `holding_candidate` / `unknown`）、`jaw_state`、`gap_m`、`wrist_state`、
`object_visible`、`observation_id`、`model_present` 和 `physical_capture_verified`。
没有当前模型或标定时仍返回腕部图像引用，但不猜测物体类别、大小或抓牢情况。
视觉被遮挡、漏检、歧义或超出预期投影时为 `inconclusive`，不是空夹爪。
RGB 中看到目标只是一致性证据，`physical_capture_verified` 仍为 false。
视觉计算期间姿态/夹爪变化时结果作废。`holding.json` 是诊断日志，不用于恢复运行状态。

兼容的人工确认接口仍可使用，但正常全开反馈不再需要单独执行清状态 graph。
`confirm_gripper_empty` 只清内存任务模型，保留当前会话的已放置障碍物。

MHS 设备状态也发布 `carry_status`、`carried_object_id`、`placed_object_count`。
新进程的任务模型为空；若实际已持物，应先重新建立模型。

### 放置参数与执行

`plan_place_object` 所需参数：

- `side="right"`，`frame_id` 为支持的基座坐标系，+Z 须与重力向上方向一致。
- `x_m/y_m` 是**物体中心**的水平位置，不是 TCP 位置。
- `support_z_m` 是该坐标系中的水平支撑面高度，不是物体中心或 TCP 高度。
- `approach_height_m` 是搬运时的抬升余量，`retreat_height_m` 是松爪后撤回距离。
- `release_gap_m` 是张爪后的总开度，必须足够让带几何余量的物体通过。

工具保持当前朝向，根据 `T_tcp_object` 换算 TCP 目标。先竖直抬高，再到放置点上方，
最后竖直下降。默认物体底面在支撑面上方 6 mm 松开；这是位置式释放，不是接触力控制。
已放置盒形障碍包含释放到落稳之间的竖直包络及端点误差余量，不模拟真实掉落动力学。

示例 `right_place_object_plan.json` 只规划，`right_place_object.json` 重新规划并执行。
两者都需要上述参数作为 `inputs`，没有实机放置坐标默认值。
完整 `right_pick_place.json` 还需要 `target`，串联前置感知、抓取、搬运和放置，
只在每一步验证通过后继续；不会自动回零或重试。它使用新目录版本，需要重启 bridge 和 Agent。

执行顺序：

```text
检查携带状态和起点 → 抬升 → 水平搬运 → 下降 → 核实到位
→ 记录释放中 → 张爪并核实反馈 → 将物体转为静态障碍
→ 用实际状态重新验证撤回 → 竖直撤回
```

转移失败/取消不松爪；释放失败/取消保留 `unknown`；已释放但撤回失败时保留
空夹爪及静态障碍物，不重新闭爪或自动重放。`placement_state.json` 记录失败阶段。
执行结果的 `placement_verified=false` 表示尚无放置成功的独立视觉/接触确认。

已放置物最多记忆 16 个，后续规划继续检查；再次感知到唯一对应的已放置物时，
允许抓取手指接触它，只有闭爪完成才把该物体从静态场景转为携带几何。
轮子位置变化后旧的基座场景记录不能直接复用，需要重新核实场景。
`clear_placed_objects(scene_cleared=true)` 仅用于人工确认旧障碍物已清理，不会自动删除持物模型。

目前为右臂、用户指定放置区域的取放 MVP：没有动态障碍感知、自动绕障、负载动力学、
接触式柔顺放置、滑落检测或抓牢验证。模型中的位置式取放和实际效果仍需真机验收。

### 按区域放置：桌面上、盒子里

新增三个可组合的接口。`perceive_place_region` 接收 `observation_id`、自然语言 `target` 和关系 `on` / `inside`，生成机器人保存的 `region_id`。`plan_object_placement` 根据携物模型和该区域选择位置，生成已有 `execute_place_plan` 可以执行的计划。感知和规划不发送运动指令。

空间模型将容器边界、已观测底面、已观测障碍和未知区域分开保存。`inside` 使用 `hei.place_region/v2`：四侧上沿/侧面点拟合矩形容器的边界包络，尺寸不再由最大连续底面矩形决定。边界包含拟合残差余量；仍假设容器矩形、底面水平、侧面可用线性轮廓近似，不能视为精确重建或支持任意形状容器。`on` 和操作者声明的区域保留原接口。

`spatial_model.coverage` 是 4 mm 网格：0=未知，1=观测到支撑面，2=观测到高于支撑面的几何，3=有界局部曲面补全的支撑。完整对齐深度参与障碍判断，包括未被托盘分割掩码包含的物体；深度缺失不用于缩小容器边界。封闭空洞可由周围有效观测拟合局部二次曲面：周边至少 95% 有效、RMS ≤3.5 mm、95% 残差 ≤6 mm，累计空洞预算为掩码的 10%，平均倾斜门槛仍为法向 Z≥0.98。仅在观测凸包和高度包络（含 6 mm 残差）内求交；有效深度、开放边缘和大面积缺失不会被替换。使用更宽的 13×13 像素邻域约束曲率，并用实测环带验证。面积按实际边界裁剪单元计算，四类面积之和等于内部包络面积。查看 `region.json`、`region_diagnostics.json`、`region_coverage.png`（灰色未知、绿色实测支撑、红色占据、蓝色曲面推断支撑），并结合原始 RGB-D 检查。

放置候选必须在内部边界内，并具有覆盖携物投影和原有余量的已观测支撑范围；候选按距中心排序，最多验证 32 个。旋转物体使用实际凸投影与原有余量的 Minkowski 和，不再使用轴对齐外接矩形作最终支撑判定；边界仍保留原有余量。按网格内深度分布的 5%/95% 分位处理噪声，曲面环带只有满足拟合残差的实测点才归入支撑。释放高度和碰撞底面同步抬至支撑高度上包络，避免清除了假障碍却仍朝更低的平面运动。尺寸不匹配、观测不足与 IK/碰撞失败分别报告。碰撞模型使用同一边界、底面及网格生成的几何：未知单元作为明确标记的保守禁入体积参与检查，报告 `constraint_kind=unobserved_space`，不宣称那里真的有盒壁。网格拆成精确矩形并作为一个场景条目保存，后续携物路径也沿用这些约束；不跨越空洞合并包围盒。未改变原有碰撞余量、标定、IK 标准或运动限值。底面/四侧证据不足时仍拒绝重建，需要重新观测。


区域同时保留不同长宽比例的受支持矩形 `usable_rectangle_options`。规划依据当前物体姿态和既有余量选择能容纳物体的矩形，并用同一矩形生成盒壁碰撞模型。面积最大的窄矩形不再排除更宽的可用候选。尺寸不足的报错会同时给出估计可用尺寸和物体所需尺寸；可用区域是观测与建模结果，不等同于实际容器尺寸。

规划器保留手中物体朝向，先按物体水平投影和余量检查能否放入，再尝试区域中心及四个内部候选位置。盒底与保守盒壁参与全部阶段的碰撞检查，搬运高度包含盒壁高度；夹爪的展开和退出也必须通过检查。它不具备自动绕障、接触力控或物体实际落稳验证能力。

已经抓住物体后，在 Agent 的交互输入中先运行仅规划示例：

```json
{"graph":"agent/src/hei_agent/examples/right_place_region_plan.json","inputs":{"destination":"open cardboard box","relation":"inside"}}
```

输出 `validated`、`plan_ref` 和 `region_id`。检查结果后，可用 `right_place_region.json` 重新获取图像、规划并执行；该图会在校验未通过时结束，不松爪。完整抓取搬运示例为：

```json
{"graph":"agent/src/hei_agent/examples/right_pick_place_region.json","inputs":{"target":"blue block","destination":"open cardboard box","relation":"inside"}}
```

完整示例在抓取结束后重新拍摄盒子，避免抓取过程中区域观测过期。放到桌面上时将 `relation` 改成 `on`，并给出能唯一指认支撑面的描述。示例中的目标描述和抬升、开口参数均可修改，未绑定固定物体类别。

没有合适的视觉条件时，`create_place_region` 可使用实测数据创建同一类区域，跳过视觉感知。输入为 `frame_id`、`relation`、`geometry_confirmed:true`、`x_m/y_m/support_z_m`、`size_x_m/size_y_m`、`yaw_rad`、`wall_height_m` 和 `boundary_thickness_m`。尺寸是**可用内部尺寸**，中心 Z 是支撑面，角度绕 +Z；`on` 的壁高为零。`right_declared_place_region_plan.json` 将这些输入组合成只规划流程。确认值和尺寸必须来自操作者，不能由 Agent 编造。旧的 `plan_place_object` 指定坐标入口继续可用。

视觉区域在拍摄后 180 秒内有效（为整条放置路径的计算保留时间），人工区域有效 300 秒；计划执行前再次检查期限和底盘、升降、另一侧手臂状态。区域缓存最多 8 个，桥重启后需要重新创建。感知区域在用于规划时将底面、盒壁写入当前内存场景，后续搬运和释放后的撤离仍能检查它们；它们也会占用最多 16 个场景盒的容量。重新感知不会自动删除旧区域，避免把已知障碍静默移除；现场改变后需人工核对并用 `clear_placed_objects` 清理，再重新建立区域。当前场景是静态快照，不持续追踪被人移动的盒子。

更新后重启 Agent 和 `agent_bridge` 以刷新技能目录即可，不需要重启电机控制器。上述测试只能验证软件几何与执行状态机，首次真机运行前仍需检查现场区域模型。

### Jetson 计算加速

前置和腕部 YOLO 在 compute Python 的 CUDA 可用时使用 `cuda:0`，否则用 CPU；
不改变置信度、输入分辨率和分割阈值。感知诊断记录 `compute_device`。

`bash scripts/setup.bash --gpu-planner` 在 `.deps/gpu-planner-venv` 中构建固定版本
cuRobo v0.7.8，继承现有 CUDA PyTorch，不替换原环境。默认 profile 使用该隔离环境，
新部署需先执行此入口；CPU 部署可覆盖 worker_python 并设 ik_backend=scipy。要求 CUDA 编译器，默认
Orin SM87，可用 `TORCH_CUDA_ARCH_LIST` 覆盖。安装验证成功后会备份并更新两个源码
profile 的 `worker_python`，重新构建 hei_agent_bridge 安装配置后，
`ik_backend: auto` 会选择 cuRobo；
`scipy` 强制 CPU，`curobo` 在 GPU 后端缺失时明确报错。

GPU 批量计算 IK 候选；关节次序、关节范围和末端误差由原 URDF 正运动学复核。
完整轨迹仍使用原碰撞验证，不能将 GPU IK 成功视为碰撞验证通过。
GPU 使用固定 32 个目标的批次复用 CUDA graph；首次进程初始化仍有开销。
碰撞几何的 STL 分解按路径、修改时间和文件大小缓存，文件改变时重新计算；
不缓存碰撞通过结果。规划过程复用模型，放置规划写入 `placement_progress.json` 的逐候选耗时。

机械臂 skill 的默认速度由 0.10 调为 0.12 rad/s；共享轨迹生成器仍保留 60%
速度比例和原有平滑策略。已有运行节点不会被脚本自动重启，配置需在空闲时重新加载。

### 在线规划与碰撞后端

在线链路为 GPU cuRobo IK（不可用时按既有配置回退 SciPy）→ 轨迹生成 → FCL 几何验证。
抓取、腕部矫正、普通手臂运动及放置均默认使用 FCL；缺少 FCL 时明确报错，不静默回退到 MuJoCo，也不跳过碰撞验证。
MuJoCo 仅供离线对照，通过验证函数的 `backend="mujoco"` 显式选择，不是运行配置开关。

FCL 后端直接读取 URDF，用文件版本缓存各独立 STL 实体的凸包；批量变换 AABB，
仅对包围盒可能相交的几何对调用原生 FCL。场景、携带物体、夹爪状态、接触规则及验证结果均不跨请求缓存。
放置先筛选 GPU 给出的各个释放 IK 解：先检查完全打开的必经姿态，再检查开合过程。
释放预筛选通过后才生成完整搬运和撤退轨迹。发现碰撞后立即拒绝该候选；
通过的候选仍需检查完整轨迹及夹爪释放过程。
同一候选不同 IK 解复用几何上下文，使用前核对 URDF、场景及携带物体的一致性；接触状态每次重置。
保留已有间隙、接触容差、关节范围和未知区域规则，加入右腕固定附件的检查。

这不是完整的 cuRobo MotionGen：GPU 负责 IK 及批量 FK/AABB 候选筛选，FCL 精确查询在 CPU 上运行；当前轨迹仍是采样检查，
没有新增连续碰撞保证或自动绕障路径搜索。原始场景误差也不会因为换后端而自动消失。
安装统一使用 `bash scripts/setup.bash --gpu-planner`，已有非 GPU 计算环境使用
`bash scripts/setup.bash --arm-motion` 或 `--grasp` 安装并检查 python-fcl。

2026-10-31 Jetson 离线回放（请求 `6c3a147d4930451f8b296babaa17cf16`）：
同一批 32 个失败候选从此前约 73.3 秒降至 28.1 秒（包含进程启动、CUDA 和几何冷加载）。
结果仍是释放碰撞，不代表已完成真机放置；未执行运动或更改速度、间隙及碰撞容差。

### 常驻计算 worker

Bridge 默认通过串行常驻子进程提交计算请求。抓取 worker 复用 YOLO、CUDA/IK 和机器人几何缓存；手臂 worker 复用 IK 和几何缓存。每次仍读取当前请求、图像和现场状态，重新进行碰撞验证，不复用旧的验证结论。

每个请求目录包含独立的 `worker.log` 和 `worker-timing.json`（请求 ID、PID、计算秒数）。识别/规划拒绝不会销毁热缓存；取消、超时或协议错误会终止并回收子进程，下一请求重新启动。Bridge 正常关闭时回收 worker；默认随 Agent 常驻（`worker_idle_timeout_sec: 0`），父进程管道关闭后自动退出；设置正数可恢复指定秒数的闲置退出。修改 Bridge 调度代码需要重启 agent_skills 服务才能生效；无需重启硬件控制服务。此次变更不调整运动速度、标定或碰撞保护。

### 放置候选与碰撞查询优化

区域放置最多检查 32 个候选，交错搜索保持当前朝向、绕重力方向 ±45°、±90° 和 180° 的放置姿态。每个朝向重新计算携带物体的支撑投影，先尝试分散的位置，再补充中心附近的细采样；显式指定的释放开口不变。目标 TCP 根据旋转后的 TCP—物体变换反算，避免旋转时物体中心偏离目标。

cuRobo 一次批量求解候选释放姿态；首候选保留直接验证快路径；若失败，后续准确释放构型按每 4 个候选一组在共享现场中按需批量预检，排除不可能开爪的候选后，再生成路径。批次复用同一场景及 GPU 缓冲，找到通过完整验证的候选即停止后续搜索。共享预检暂不加入新释放物体，只用于拒绝；通过后仍检查该物体、完整开爪过程、搬运和撤离轨迹。`batch_release_screen` 记录构型数、拒绝候选数、后端、FCL 查询数和耗时。已有 GPU 解被拒绝时直接进入既有 CPU 求解，避免重复提交同一个 GPU 目标。

FCL 复用碰撞对拓扑；同一个不可变现场中，完全相同的关节和夹爪位置可复用刚计算的接触结果，任何姿态更新或显式接触查询都会使其失效，不跨现场复用。候选诊断保留 x/y 和 yaw。

此优化仍采用 FCL 精确几何查询和分段路径；尚未接入完整 MotionGen；CUDA 批处理筛选见下节，不代表 GPU 全流程避障规划已经完成。扩大搜索不能消除模型中的碰撞，也不会自动缩小用户指定的释放开口。


### CUDA 批量碰撞筛选

轨迹验证先用 CPU 检查第一个姿态，短于 8 个剩余采样点的检查直接保留 CPU；跨候选预检从第一个构型起批量处理。按 32 个采样点一批，在 CUDA 上进行完整 URDF 正运动学、几何 AABB 变换和碰撞对筛选。适用的凸体/盒子对继续在 GPU 上计算完整面法向与边叉积分离轴，排除 AABB 重叠但凸体分离的情况；大型圆滑 CAD 凸包保留 FCL 路径以限制临时显存。几何来源与 CPU FCL 相同，包含活动夹指、携带物体、已放置物体及覆盖区域排除框；没有另建简化球体模型。精确接触距离及原接触规则仍由 FCL 判断。为避免 FCL 穿透深度对浮点微差敏感，精确查询保留原 CPU 变换计算。

`HEI_COLLISION_ACCELERATOR=auto`（默认）在 worker 的 Python 环境有 CUDA PyTorch 时启用；`cpu` 可强制原 CPU 路径，`cuda` 强制要求批处理使用 CUDA。GPU 计算异常直接报错，不按无碰撞处理。验证报告的 `collision_accelerator=cuda_fk_aabb_sat+fcl` 表示 CUDA FK/AABB/凸体分离轴筛选加 FCL 距离查询。环境变量需要在启动 `agent_skills.launch.py` 前设置；已经运行的计算进程需要退出后重新加载代码。

Jetson 离线测试：128 个随机姿态（含夹指开合、携带物和障碍物）的接触对与深度和 CPU 一致；512 个连续采样点从约 1.23 秒降至约 0.64 秒。该数字是碰撞采样部分，不能当作完整规划或机器人运动的加速倍数。仍保留原采样密度、终点精度和碰撞阈值，尚不是连续碰撞检测或完整 GPU MotionGen。


### 有效深度底面与缺失深度分开建模

有效深度点通过原始底面高度带内的点拟合二次曲面，再在这些点的 XY 凸包和高度包络内解释连续底面。拟合不依赖附近是否存在深度缺失洞；后续仅使用残差范围内的点细化，并继续约束原始底面锚点和倾角，保留凸起离群点。沿用 6 mm 残差、3.5 mm 拟合噪声和 5° 倾角要求；缺失深度仍沿用原有有界插值，未知和异常点继续参与排除。诊断中新增 `spatial_model.measured_surface`。

2026-11-01 保存场景离线重放：底面分类修复后支持区域筛选从 0 个候选恢复到 8 个，但完整开爪预检查仍报告夹指与托盘前壁/残余占据框碰撞，未生成通过验证的放置轨迹。该修复不表示已验证真机放置成功。


2026-11-01 后续修复：墙体上沿保持原始测量的绝对高度，不随抬高的底面参考重复上移；占据格保存 `occupied_top_z_m`，碰撞框按实际最大观测高度生成，低于碰撞底面的点由底面包络覆盖。旧区域没有高度图时仍采用旧保守高度。未知区域继续保留排除。

候选加入物体主轴与托盘轴精确对齐，并采用从粗到细的位置覆盖。在保存请求的离线重放中，9 cm 开口的 26 个候选仍被拒绝；单独的 5 cm 开口对照通过 transfer/opening/retreat 全部验证。5 cm 只用于临时离线对照，没有自动修改用户的 9 cm 输入。新代码需重新加载 bridge，并重新感知托盘来生成新高度图。

### 放置终点修正

放置搬运终点和撤离终点若未满足原有到位标准，会先等待 0.5 秒重新读取反馈，随后最多进行两次修正。修正使用最新关节状态及当前携带物/障碍模型，重新规划到原始 TCP 目标；只有碰撞验证通过、规划期间场景和起点未改变时才发送轨迹。误差不下降、验证失败或次数耗尽仍会停止；搬运终点合格前不会开爪。控制器异常或取消不会触发自动重试。

每次观测的目标 XYZ、实际 XYZ、`actual_minus_target_xyz_m` 和关节角保存在计划目录的 `transfer_correction/feedback.json` 或 `retreat_correction/feedback.json`，坐标系为 `a_right_base_link`。第 1、2 次修正的规划输入输出保存在对应子目录。这是编码器反馈闭环，不消除视觉标定或机械变形误差。

### 操作者直接回零（不经 Agent 检测）

机器人终端执行 `ros2 run hei_bringup arm_zero --side right`；左臂用 `--side left`，双臂用 `--side both`（默认）。六个手臂关节目标全部为零，夹爪、升降及底盘不发命令。直接发送 FollowJointTrajectory，不做 IK、碰撞、场景/携带物或 TCP 到位检查。读取关节位置只用于轨迹起点，仍会报告 ROS 控制器拒绝或执行错误。

按 hei-rebot-lift VR X/A 复位的 0.015 rad 步长和电机速度上限 `[3,3,3,1.8,2.5,2.5] rad/s` 生成平滑零位轨迹。VR 循环周期未固定，因此此命令对齐步长及速度上限，不保证与 VR 相同完成时间。双臂硬件速度配置已同步此组上限；已有 Agent 轨迹的速度参数没有改变。更新配置后须重新启动 robot.launch.py 生效。使用前结束其他遥操/Agent 运动任务，避免竞争控制器。

### 启动预热与缓存驻留

`arm_motion.json` 和 `right_grasp.json` 默认启用 `startup_warmup: true`、`persistent_worker: true`，并将 `worker_idle_timeout_sec` 设为 `0`。Bridge 启动后在后台依次预热机械臂规划、抓取规划进程：初始化 cuRobo/CUDA、用合成位姿执行固定批量 IK、加载碰撞网格/凸包并预热 CUDA 碰撞筛选。预热结果只是计算诊断，不产生可执行计划，不读取反馈或调用动作。标定和现场几何仍在真实请求中独立检查。

MHS 连接和状态发布可在预热期间运行；同一 worker 的请求通过同一锁排队，预热未完成时仍需等待。终端显示 `arm compute warmup ready`、`grasp compute warmup ready` 后，该进程的首次规划不再承担模型初始化。MHS 只读工具 `get_compute_status` 返回各 worker 的预热状态、PID、是否存活和分阶段耗时；完整结果在 `log/hei_arm/_warmup_*/result.json`、`log/hei_grasp/_warmup_*/result.json`。状态查询会识别 worker 退出/被替换，不把旧 PID 的就绪状态沿用到新进程。

若预热失败，日志与状态接口会明确报告，Bridge 保持运行；真实请求仍可返回具体计算错误。停止 Agent 会取消预热、终止并回收计算进程。缓存通过驻留进程保存在内存中，不复用合成现场的碰撞结论。

Jetson 保存现场离线验证：预热约 14.85 秒，预热后首个真实场景请求约 10.64 秒，较约 25 秒冷请求减少约 14 秒；两者使用同一 PID。初始化耗时被提前到启动阶段，不代表规划计算本身降到零，也不保证提前到来的请求没有等待。

### 预热后的 IK 与候选预检

CPU 回退求出的准确释放终点会保留到开爪预检中。此前该解被丢弃，部分候选会跳过终点开爪预检，进行多段路径 IK 后才失败。现在这些候选可以提前按同一碰撞规则拒绝。近似过渡点的候选选择跳过原本未使用的 FK 误差计算；严格终点使用 NumPy 批量 FK 复核，仍保留原来的 5 mm / 3° 标准。CPU 最小二乘回退采用解析几何雅可比及 SO(3) 对数残差导数，目标函数权重、边界和收敛参数保持一致。

Jetson 保存现场离线对照：预热后首次失败搜索从约 10.64 秒降到 4.52 秒（32 个候选，19 个在开爪预检拒绝）；另一可通过现场约 4.21 秒，搬运、开爪、撤离均通过。这些数据包含请求与验证开销，不是机器人运动耗时，也不代表任意场景都可在同样时间内求解。


### 部分未知底面的放置策略（2026-11-01）

`arm_motion.json` 新增 `max_unknown_support_fraction`，默认 `0.5`：放置足迹（含原有边距）允许最多 50% 未知面积，不再要求所有网格均有深度。算法把每个相交未知格的整格面积计入未知面积，因此边缘处采用面积上界；至少一部分底面必须有支撑观测，完全未知区域仍拒绝。设为 `0` 恢复原来的完整支撑要求。配置在 `plan_object_placement` 时冻结到该次区域副本，并随请求记录。

启用部分观测策略时，未知格不再生成整列禁入盒；这是明确放宽未知空间策略，并不代表这些格已经测得空闲。实测占据格、四侧壁、底面、携物边距及其他碰撞检查不变。设置为 `0` 时恢复未知格禁入盒。候选筛选和碰撞模型使用同一份策略。已有深度和覆盖标签保持原样，不以补洞伪造测量。

### 开爪期间的保持与分阶段结果（2026-11-01）

开爪期间不再单独用“相对开爪前任一关节变化 >0.003 rad”中止，而是沿用释放到位时的标准检查同一目标：TCP 位置误差 ≤5 mm、姿态误差 ≤3°、最终关节解误差 ≤0.03 rad；场景检查保留。小幅关节回弹若仍满足已验证的释放位姿，不会单独导致失败；末端越界仍停止。开爪 action 成功且实测开度满足原有 0.02 rad 反馈容差才标记开爪确认，然后最多等待 1 s 的右臂速度稳定；不重发开爪、不新增自动运动。

`execute_place_plan` 返回 `release_started`、`release_confirmed`、`placed`、`retracted`、`failed_stage`、`detail_ref`。其中 `release_confirmed` 仅表示开爪动作和开度反馈确认，`placed` 表示释放位姿与反馈检查完成、携物几何已转为场景障碍；两者都不是视觉放置验证，`placement_verified` 仍为 false。已释放但撤回失败时，技能仍返回 failure，同时保留这些部分完成字段，提示“Object release confirmed; retreat incomplete”。未知释放、未终止动作不会被冒充为完整成功，图执行仍停在失败节点；清理未确认时部分输出也不会对外返回。

`placement_state.json` 在释放开始、开度确认、进入撤回、完成或失败时写入本次执行证据；`release_feedback.json` 保存开爪/稳定期间的关节、末端偏差、夹爪开度、场景变化和具体失败原因。这些是日志，不用于进程重启后恢复携物状态。原有标定、运动速度、放置碰撞检查和 50% 未知支撑策略不变。

当前抓取和搬运默认 `joint_speed_rad_s=0.6`，轨迹保留 60% 余量，实际速度峰值上限 0.36 rad/s。共享五次轨迹加入 jerk 上限并比较整段/局部分段计时，减少密集路点对速度与平滑性的影响。新曲线和 ROS 执行使用同一组位置、速度、加速度端点，仍需完整碰撞验证。该修改不改变快速回零命令、电机增益或硬件限值；重启空闲的 agent_skills 后重新生成计划即可生效。

### 显式允许小倾角放置

`plan_object_placement` 默认允许最多附加 10° 外倾，先搜索原有倾角的位置与水平旋转，再尝试 5°、10° 外倾。它的候选失败不代表整个盒内不存在可行放置姿态。新增 `plan_object_placement_with_tilt`，参数在原接口基础上增加必填 `max_tilt_deg`（0..10，表示相对当前姿态允许增加的倾角）。先搜索原有姿态，再搜索最多 5°、最大允许角度两档，方向为机械臂基座到容器的径向外倾。该入口仅用于明确允许倾斜的物体，不适用于需要保持直立的容器内容物。

每档重新计算物体旋转后的占地和半高，执行相同支撑覆盖、占据区、搬运、开爪及撤退碰撞检查。末端 5 mm / 3° 验证、速度、单步和碰撞标准不因倾斜而放宽。计划记录 `placement_tilt_deg`，候选进度记录每次的 `tilt_deg`。

新示例：`right_pick_place_region_tilt.json`（完整抓放）、`right_place_region_tilt.json`（已有携物模型时放置）、`right_place_region_tilt_plan.json`（仅规划）。倾角示例已固定传入 `max_tilt_deg: 10`，不需要额外输入。原接口也默认允许最多附加 10°；如需完全保持倾角，通过新接口显式传入 `max_tilt_deg: 0`。

保存场景 15108449 的完整离线重放在增加 10° 倾角后通过搬运、开爪和撤退验证，盒子、标定、支撑图和碰撞标准均未改变；5° 的 IK 成功不足以证明完整路径可行。离线通过不是实物放置成功证明。

小顶面叠放（`relation=on`）使用 2–8 mm 自适应网格，边缘内缩一格。允许方块边缘悬空，要求估计重心投影在观测到的支撑矩形内，并保留 `payload_padding_m + 0.005 m` 的重心边距。当前以均匀密度包围盒中心估计重心；这不是实测质量分布或动力学稳定性证明。`inside` 仍要求完整物体足迹放入容器。搬运、开爪、撤退的碰撞检查保持执行。
