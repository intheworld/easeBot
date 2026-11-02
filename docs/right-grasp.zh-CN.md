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
腕部 RGB 矫正为可选步骤，见文末；没有 cuRobo 绕障或物体抓牢检测。

OBB 感知使用 1280 推理尺寸、0.25 置信度阈值；不会增加相机原始分辨率。
目标为非空物体描述，最多 160 字符，建议简短英文；每次调用设置本次目标，
没有类别白名单。示例使用 `blue block`，可直接修改图中的 `target`。
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
需要 NumPy、SciPy、OpenCV、python-fcl（在线碰撞查询）、Ultralytics 和匹配的 Torch。
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

## 抓取内部的右腕 RGB 矫正

`execute_right_grasp` 已包含可选的腕部闭环，不增加对外 skill 或 GaP 节点：
张爪 → 预抓取悬停 → 新腕部图像 → 水平微调 → 再观察 → 垂直下降 → 闭爪。
已保存右腕 640×480 实测内参和手眼结果 `config/right_wrist_calibration.json`。
2026-10-28 排除第 1 组后使用 24 组样本，位置 RMS 3.93 mm、最大 7.75 mm，
最大旋转误差 1.21°；这是内部一致性，尚未完成独立物理点位验证。`right_grasp.json` 默认
`wrist_refinement.enabled=false`，保持现有抓取流程。输出 `wrist_refined=false`
明确表示本次未做腕部矫正。启用后若缺少标定或新鲜实机图像，会在任何运动前拒绝；
不会回退到未经矫正的下降。

当前机器人可使用 `config/right_wrist_calibration.json`；更换相机或安装关系后，
根据 `config/right_wrist_calibration.json.example` 重新标定并保存为 JSON：

- `width` / `height` 是用于标定的真实图像尺寸，运行图像必须一致，不自动缩放内参。
- `k` 是按行展开的 3×3 内参矩阵，`d` 为 OpenCV 畸变参数。
- `T_tcp_camera` 是 **右腕相机光学坐标 → a_right_tcp** 的 4×4 变换，平移单位米；
  `p_tcp = T_tcp_camera @ p_camera`。不要填反，不使用 CAD 安装位姿替代实测标定。
- 支持 `plumb_bob` / `rational_polynomial`；光学轴为 x 右、y 下、z 前。
- `.example` 的空值故意不能运行；前置 eye-to-hand 标定不能代替腕部 eye-in-hand 标定。

在所用 skill 配置的 `right_grasp` 中添加（路径是机器人上的实际文件）：

```json
{
  "wrist_refinement": {
    "enabled": true,
    "calibration_file": "/home/seeed/gitdir/easeBot/ros2_ws/src/hei_agent_bridge/config/right_wrist_calibration.json"
  }
}
```

更新安装、重启 bridge 并重新生成计划。标定文件内容参与 catalog/计划绑定，
运行时改动后拒绝继续执行；Agent 调用接口仍是 `execute_right_grasp(plan_id)`。

算法使用已有前置 OBB 的尺寸、朝向和顶面高度，将三维包围盒投影到腕部图像，
比较预测轮廓中心与 YOLOE 分割轮廓中心，再按目标高度平面反投影估计水平平移。
使用相机的实测内外参及拍摄时的关节 FK，目标不是图像中心。
这是近似盒形、静止桌面目标的 RGB 矫正，不是新增深度测量、多视角重建或自动标定；
OBB 形状/高度误差、遮挡、镜面和不规则目标仍可能带来偏差。

默认每步最多 5 mm，目标累计修正与手臂离原悬停位的水平偏移分别限制 20 mm；
最多移动 3 次，最后一次必须重新观察并达到 2 mm 模型估计误差才下降。
每次观测使用悬停停稳之后的新帧。目标未检出、同类目标歧义、图像裁切、
面积不一致、视线过斜、漂移、反馈异常、标定变化、IK/碰撞失败或不收敛都会中止。
每步微调的完整路径和接下来的下降路径均经过FCL 采样碰撞检查；
最终下降沿竖直方向，保持验证时的 TCP 朝向。

诊断文件位于计划目录 `wrist_00/`、`wrist_01/` 等，包括 RGB、请求、worker 结果、
`wrist_mask.png` 与 `wrist.json`（像素中心、估计误差和累计修正）。
取消/超时沿用现有进程回收及 ROS Action 取消机制，不自动闭爪、返回或重试。
`wrist_refined=true` 表示视觉估计收敛且通过轨迹检查，不证明目标已抓牢；
`grasp_verified` 仍为 false。已完成实测标定和离线/模拟验证，真机闭环验收尚未完成。


### 腕部取图质量检查

预抓取及每次微调后默认等待 `settle_sec=1.0` 秒，再检查新帧。
只计入时间戳递增且在等待结束后收到的图像；连续 `stable_frames=3` 次
帧间比较满足稳定条件后才运行分割模型（首次建立参考，因此至少需要 4 帧）。
中心区域灰度拉普拉斯方差用于诊断，默认 `min_sharpness=0`，不作硬性门槛。
只有对现场图像标定过阈值后才显式设置正数以开启清晰度筛选；
`max_frame_difference=3` 为相邻区域灰度绝对差均值上限。
这些是可配置的启发式检查，不是对焦、目标检测或相机曝光时间戳的保证。
USB 相机采集线程持续读帧、尽力请求单帧缓冲，发布频率仍由 `fps` 控制。

`frame_timeout_sec` 内未通过时中止，不下降、不闭爪；计划目录 `wrist_XX/`
保存 `capture_quality.json` 以及最后一帧 `rgb.jpg`。清晰度始终偏低时，
检查对焦、照明、曝光和机械振动，不应直接降低阈值或扩大修正范围。
修改配置后重启 bridge，重新发现 catalog 并生成计划；相机采集方式修改需要重启相机节点。

## 后续搬运

成功完成闭爪后，bridge 会维护物体相对 TCP 的盒形模型，供后续手臂轨迹做碰撞检查。
原抓取图仍在闭爪后停止。完整取放使用 `right_pick_place.json`，相对抬升、多路点和
单独放置参见 [机械臂搬运接口](arm-motion.zh-CN.md#右臂搬运与放置)。
携带记录不是抓牢验证；闭爪中断、释放中断或带物重启需按记录恢复状态。

抓取后可接区域放置流程：`right_pick_place_region.json` 使用 `target` 识别抓取物体，在闭爪后重新获取 front RGB-D，并使用 `destination` 和 `relation`（`on` / `inside`）感知放置区域。盒底和盒壁参与搬运、松爪和撤离校验。详见 [区域放置](arm-motion.zh-CN.md#按区域放置桌面上盒子里)。

### 腕部采图的反馈稳定与时间对齐

腕部矫正不再只等固定 1 秒、比较两个端点后立即退出。保留 `settle_sec` 的最短等待后，还要等待右臂关节在连续 `joint_stable_sec` 窗口内的变化范围不超过 `joint_span_rad`。ROS 后端检查窗口内所有反馈，而不是只比较采样端点；关节没有持续更新时不能通过。

默认新参数位于 `right_grasp.json` 的 `wrist_refinement` 中：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `joint_stable_sec` | 0.4 s | 运动后连续反馈稳定窗口 |
| `joint_timeout_sec` | 6 s | 每次等待关节稳定的最长时间 |
| `joint_span_rad` | 0.003 rad | 窗口关节变化、采图及计算期间允许的姿态变化 |
| `joint_sample_skew_sec` | 0.05 s | 图像时间戳到前后两组关节反馈各自的最大间隔 |
| `capture_retries` | 2 | 姿态不稳或时间对齐失败时，额外重采次数 |

ROS 后端缓存最多 1024 组、最近 5 秒的完整右臂带时间戳反馈。用图像时间戳前后两组反馈插值得到拍摄姿态，缺少包围样本、反馈过旧、间隔太大或该区间正在明显运动时，不使用最新关节值代替。USB RGB 图像时间戳是驱动取帧后的 ROS 时钟标记，属于软件时间对齐，不是硬件曝光同步。

采图时右臂仍在收敛，或视觉计算期间姿态漂移超限，会丢弃旧结果、重新等待稳定并采图，不会执行旧矫正轨迹。最多执行 `1 + capture_retries` 次尝试，整个抓取仍受技能总超时限制。真实的底盘、升降、左臂、夹爪变化、碰撞校验失败及标定变化继续中止，不通过重试掩盖。

每轮 `wrist_XX/stability.json` 记录关节状态、窗口变化范围、触发关节及变化量、时间对齐样本和重采原因。采图质量仍在 `capture_quality.json`；各次成功采图另存为 `rgb_attempt_XX.jpg`，计算输入保存为 `request_attempt_XX.json`。持续不稳定时，错误会携带具体原因和日志路径，不再只有 “Robot moved”。

修改代码后需重启 `agent_bridge` 以启用新缓存及技能配置，再刷新 Agent 的技能目录；不需要重启相机或电机控制器。

### 腕部目标漏检的局部恢复

腕部相机已经有 front OBB、手眼标定和拍摄姿态，因此整图未得到可用目标时，会在投影位置附近做一次局部重检测。搜索范围由物体 OBB 和允许的最大平移生成，并保留周围图像上下文。检测阈值仍为 **0.25**，不因漏检自动降低；面积范围、最大矫正距离和碰撞检查也不变。

分割实例中不相连的片段分别进入几何筛选，避免远离目标的小块噪声导致整个实例被拒绝。仅合并高度重叠或位置、尺度一致的嵌套重复掩码；两个分开的可行候选仍报歧义。触及裁剪边界的片段被拒绝。整图已经发现多个可行候选时，不用裁剪来隐藏歧义。

`wrist.json` 的 `attempts` 会分别记录整图和局部检测的结果、裁剪坐标、候选数量及几何筛选结果；`wrist_roi.jpg` 保存裁剪图。局部恢复后仍需经过原有的迭代收敛检查，不能把“模型重新检测到目标”当作抓取成功。

### 防止部分物体掩码造成假漂移

相邻腕部观测现在比较“掩码面积 / 当前 OBB 投影面积”，避免距离变化本身被误判。已有可靠观测后，该归一化面积相对前一轮需位于 `0.85..1.18`；突然只分出上表面或正面的掩码不能直接更新目标位置。`wrist.json` 会记录 `area_fraction`、`reference_area_fraction`、`silhouette_consistent` 和 `geometry_plausible`。

视觉结果缺失或掩码不一致通过结构化错误触发同一轮的重新采集，沿用 `capture_retries`；只有重新取得有效观测才计算并执行下一步。重采不会额外消耗机械臂矫正步数，也不会提高 `max_updates`。歧义、碰撞、标定变化及场景移动仍直接中止。检测阈值 0.25、2 mm 收敛标准及全部运动限制保持原值。

若连续重采仍只能看到部分物体，流程会停在悬停处，不会采用上一轮的旧结果继续下降。外部计算每次启动前也清除旧 `result.json`，确保重采消费的是本次新结果。

### 前置 YOLO 的场景词汇回退

`right_grasp.detector_prompt_aliases` 是可选的场景词汇配置，默认空对象，不限定 skill 的物体类别。
例如在当前灰色塑料托盘场景，可在实际使用的 `skills.ros.grasp.json` 的 `right_grasp` 中合并：

```json
{"detector_prompt_aliases": {"open box": ["gray plastic container"]}}
```

先检测任务原始描述，只有完全没有匹配检测时才按顺序尝试同义描述，每个目标最多三个。
描述必须指向同一现场物体；颜色、形状等限定应保持一致，不应把“红色盒子”回退成任意“物体”。
已有检测但缺少 mask、缺少有效深度、多目标或几何失败时，不通过换词绕过校验。
原任务 target 保留，实际选中的 `detection_prompt` 随物体/区域保存；抓取后腕部复查继续使用该描述。

前置检测保持 `conf=0.25`、`imgsz=1280`，开启原图分辨率 mask。
`perception.json` 保存 `requested_target`、`selected_prompt` 和各次 `attempts`；
`perception_mask.png` 与 `perception_overlay.jpg` 用于检查所选分割范围。
更改词汇配置后在空闲时重启 bridge，重新发现 catalog；不要复用旧计划。

保存图像 `10192eaad31747c5a28a0a838a00f1ee` 的离线对照中，`open box` 在
640/960/1280 输入尺度都未检出；`gray plastic container` 在原阈值下为 0.407，
可通过容器底面/边缘几何估计。这是单帧场景验证，不代表所有盒子或遮挡条件都能稳定识别。

前置 RGB-D 快照保留最多八帧 RGB 和八帧深度；优先选择最近到达且完整的严格同时间戳配对。
若最新 RGB 的深度丢包，可使用仍在原有一秒新鲜度要求内的完整配对，或在原有 100 ms 等待预算内等待下一组。
不会用临近时间戳冒充对齐、复用过期历史帧或改变相机内参；只读 RGB 观察仍可返回明确标记为不含深度的快照。

### 腕部累计修正限幅

累计视觉目标估计超过 `max_total_m` 时，不立即报错退出：控制目标投影到以最初
悬停点为中心、半径 `max_total_m` 的平面圆盘内，再按原有 `max_step_m` 分步移动。
悬停高度不变，碰撞校验和最大修正轮数不变。

原始视觉目标和物体几何不截断；收敛仍看实际视觉误差是否满足 `tolerance_m`，
不是看限幅后命令误差是否为零。触到边界后若仍未对准，仍不下降、不闭爪；
已有观测/修正预算耗尽后报告不收敛。图像目标关联范围也未扩大。

`wrist.json` 新增 `target_offset_saturated`、`requested_hover_offset_m`、
`limited_hover_offset_m`、`commanded_alignment_error_m`、`applied_step_m`、`hover_offset_m`。
`total_target_offset_m` 与 `alignment_error_m` 继续保存未截断的视觉估计。

### 腕部矫正固定特征跟踪（2026-11-01）

首次可靠 YOLO 分割初始化目标，后续修正不再重算分割质心，也不在全图与裁剪图的不同轮廓间切换。首帧掩膜内部提取最多 128 个图像特征，固定为本次抓取的参考；用当前 FK、腕部标定和原物体表面平面投影提供光流初值，进行双向 LK、RANSAC 一致性筛选、平面度量配准和外观核对。每轮都直接对首帧配准，避免逐帧累计漂移。配准支持至少 8 个匹配点、足够的空间分布，且检查双向误差、平面残差与图像小块相关性；匹配丢失时重采图，不改用新分割质心继续移动。

`reference_plan.wrist_tracking` 指向本次抓取目录中的 `tracking_anchor.npz`，只作为当前修正循环的图像证据，不跨抓取恢复或复用。文件记录首帧灰度图、特征及标定摘要。`wrist.json` 保存 `method`、匹配数、内点数、`registration_residual_p90_m` 和 `patch_ncc_median`。纹理不足时会要求重新观测；首帧定位仍依赖检测和现有标定，跟踪不构成重新标定或真实抓取验证。

保留原有 2 mm 收敛要求、5 mm 单步、20 mm 总修正范围、3 次修正以及所有运动碰撞检查。保存失败 `a1d7c561eb254526b1dd679a5ad0da64` 的离线图像重放中，前 3 帧估计误差为约 9.2、5.4、1.3 mm；原算法第 3 帧包含阴影，估计为 7.6 mm。第 4 帧匹配不一致仍被拒绝。该结果只验证保存图像上的配准，真实闭环运动需要另行实测。

2026-11-01 跟进 `6d2b7f12a6814907a8c44e01d7d2d86f`：暗色方块灰度纹理弱，原来的通用 8 自由度单应性拟合会将有效匹配判成运动不一致。光流改用 Lab 多通道信息，外观核对仍使用灰度小块，避免平坦颜色造成虚高相关度；几何一致性改为基于已知相机位姿和目标表面的二维度量平移共识。沿用至少 8 个内点、60% 一致比例、特征分布、2 mm 度量残差及 0.8 灰度相关性要求，不调整动作收敛或修正限值。

新诊断包含 `registration_model`、`flow_color_space`、`metric_inlier_fraction`，即使共识失败也记录实际内点数量。该失败的首次修正后观测在离线重放中得到 26/37 一致匹配（70.3%）、约 1.3 mm 的配准残差，因此可继续矫正。后两次重采仍未满足 60%，不会强行放行。旧 `a1d7...` 第四帧在新模型下可配准，但剩余末端误差仍超过 2 mm，不能标记收敛。保存图像重放不能替代真实闭环验证。

### 腕部配准的稠密恢复与全量回放（2026-11-01）

稀疏跟踪因纹理消失、匹配数量或空间分布不足时，尝试固定首帧的整块目标图像配准。锚点保存本次原始 RGB、掩膜和相机位姿；用已有内参/畸变和目标平面把模板重投影到当前视角，在原有度量关联范围内搜索。模板必须以目标为主；要求彩色相关度、无其他可信位置峰、亮度与颜色定位一致，以及正反向配准一致。相关度要求为 0.8，几何一致性使用 2 mm；目标消失、遮挡、换色、多目标歧义不会通过降低动作收敛标准放行。

整个修正循环保留同一首帧，不用新的 YOLO 质心替换目标。稀疏模型仍独立执行原有标准，失败细节保存在 `sparse_attempt`；恢复成功记录 `method=fixed_anchor_dense_registration`、`dense_score`、`reverse_error_m`、`color_luminance_difference_m` 和 `competing_peaks`。旧锚点缺少图像证据时不能使用稠密恢复；需重新启动 bridge 并重新开始任务来生成新锚点。2 mm 收敛、单步/总修正量、轮数及碰撞检查不变。

用 31 次已存历史执行建立回放集，26 个序列含有 65 组可完整回放的修正后图像/反馈；剩余记录缺少初始有效观测或后续输入。供给同一份当前标定（历史标定不能从日志独立恢复）进行图像匹配回放，稀疏模型接受 46/65，稀疏加稠密恢复接受 64/65；完整图像序列通过数从 10/26 增至 25/26。另有 108 个合成负例（消失、随机噪声、空白、换色）无误接受，并有遮挡/重复目标单测。剩下一帧的颜色与亮度定位差 6.8 mm，仍拒绝使用。以上是历史图像上的观测恢复率，不是新的真机抓取成功率，也不证明位姿估计的绝对准确度。

离线回放工具（只读日志和模型，不连接 ROS、不会运动）：

```bash
PYTHONPATH=ros2_ws/src/hei_agent_bridge/src python3 ros2_ws/tests/replay_wrist_corpus.py \
  --corpus /path/to/log/hei_grasp \
  --calibration ros2_ws/src/hei_agent_bridge/config/right_wrist_calibration.json \
  --urdf ros2_ws/src/hei_description/urdf/hei_body.urdf \
  --negative-checks --output /tmp/wrist-registration-replay.json
```

### 腕部辅助关键帧恢复（2026-11-01）

仅保留首帧外观会在相机平移后因侧面、阴影和反光变化丢失匹配。现在首帧仍是主参照；当某帧直接通过首帧配准，且修正计划通过碰撞验证后，可保存一张辅助关键帧。辅助掩膜由首帧掩膜和已验证的度量位移投影而来，目标中心继承原配准结果，不重新采用 YOLO 质心。首帧的稀疏和稠密配准都失败时，才对辅助帧执行相同的稠密验证。

辅助恢复成功记录 `keyframe_recovery=true`、`keyframe_anchor` 和 `original_attempt`。辅助恢复不能刷新关键帧，因此不会形成逐帧追踪和重复重置目标中心的链条。关键帧仅属于当前抓取，不跨任务使用；原有相关度、几何一致性、2 mm 收敛、轮数及运动限值均保留。保存图像重放可以验证恢复和拒绝行为，不能替代实际闭环成功率或外部位置精度测量。

### 腕部修正次数与收敛诊断（2026-11-01）

默认 `max_updates` 从 3 调整为 5。约 15 mm 初始偏差在单步最多 5 mm、存在反馈跟随误差及观测更新时，3 次动作未必能达到 2 mm。现在最多执行 5 次经过验证的平移，并在最后一次动作后重新观测。单步 5 mm、相对原始悬停点最大偏移 20 mm、2 mm 收敛标准和碰撞验证均保留；已有显式配置覆盖默认值。

每轮写入 `refinement_status.json`，区分 `commands_executed`、`next_correction_planned`、`remaining_error_m`、`tolerance_m` 和 `budget_exhausted`。最后观测中的规划建议不等于已执行动作。次数耗尽错误包含实际剩余误差和诊断路径；不会把有效的视觉匹配直接视为运动收敛。

### 5 mm 腕部收敛与加速平滑轨迹（2026-11-01）

按用户要求，腕部实际对齐误差在 **5 mm** 内即收敛；`tolerance_m` 允许等于 `max_step_m`，因此单步 5 mm 和总偏移 20 mm 保持不变。默认最大修正次数仍为 5。

抓取和搬运配置的 `joint_speed_rad_s` 从 0.12 调为 0.6。轨迹内部保留 60% 速度余量，因此关节速度峰值上限为 0.36 rad/s；加速度上限为 0.72 rad/s²，新增 jerk 上限为 7.2 rad/s³。没有修改电机增益或硬件速度、力矩、电流和行程参数。

过去每个 IK 路点都把加速度设为零，且间隔至少 0.1 秒；密集路点可能造成额外延时和重复加减速。新轨迹用整段五次时间曲线生成中间速度、加速度；可能产生过冲的关节保留局部单调插值。复杂转弯路径另有逐段满足 jerk 限制的计时方案，两种方案均验证速度、加速度、jerk 和不越过路点区间后选择较快者。ROS 仍发送位置、速度、加速度端点，由同一五次曲线进行控制器重建和碰撞采样；所有路点及阶段起止静止要求保留。

保存路径重放示例：预抓取 63.46→12.69 秒，接近 4.99→1.68 秒，搬运 33.38→22.05 秒，撤退 5.94→3.82 秒。这些是计算得到的指令时长，不能替代实测运动和抖动结果。新默认值需要重启空闲的 agent_skills 并重新规划；无需为这些 Python 和配置改动重启机器人底层控制。
