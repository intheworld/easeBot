# HEI Agent：GaP 风格的任务图执行内核

已实现人工定义图或 LLM 生成图 → 校验 → 执行节点 → 根据结果选择边 → 保存 trace。
执行内核只依赖 Python 3.10+ 标准库，不依赖 ROS、MHS、模型服务或 GPU；
LLM 规划是可选功能，使用独立的 SDK 依赖。
这是运行在局域网服务器上的普通 Python 项目，用 pip 安装和启动，不属于 ROS 工作区。
机器人侧的 `ros2_ws/src/hei_agent_bridge` 仍由 colcon 构建，可发布并执行 MVP skill set。

参考 [Graph-as-Policy](https://graph-robots.github.io/graph-as-policy/developers/architecture.html)
的 graph / node / connector / trace 边界。本包是本项目的最小实现，**没有安装或复用
上游 GaP 运行时，也不兼容其 workflow v3**。本地图格式为 `hei_agent.graph/v1`。
目前是串行、按结果路由的有向图，不包含上游的并行 super-step、子图、流式节点、
技能发现或仿真 rehearsal。已有本项目的 LLM 图生成和有限次校验修正，
尚不包含执行反馈驱动的重规划。后续接上游时需要显式适配。

## 结构

```text
agent/
  pyproject.toml       Python 打包、可选测试依赖和 hei-agent 命令
  COLCON_IGNORE        防止从仓库根目录运行 colcon 时误收录
  src/hei_agent/
    agent.py           Agent：持有策略与 Executor，提供 run / cancel
    graph.py           Graph、Node、Edge；JSON 加载、校验和数据引用
    node.py            节点实现、上下文、结果、注册表与 SkillConnector 协议
    catalog.py         技能参数、输出与结果目录（规划与 mock 共用）
    mhs.py             AgnesHub RPC、动态目录发现、远程技能调用与取消确认
    llm.py             OpenAI 兼容 Chat Completions 配置与异步适配器
    planner.py         文本任务 → 图生成 → 数据/类型校验 → 有限次修正
    executor.py        节点调度、结果路由、超时、取消、循环上限
    trace.py           内存事件和逐条 flush 的 JSONL 文件
    mock.py            纯内存技能；不导入 ROS，不发送设备指令
    cli.py             本地 mock / 远程 MHS 的任务入口
    examples/          人工编写的图文件（随 wheel 一起打包）
  tests/               不依赖 ROS 的执行器测试
```

`Node` 是图里的调用定义；`BaseNode` 是实现接口；`NodeRegistry` 把定义中的
`kind` 映射到实现。同一个实现可以被多个节点调用。图节点无需各自成为 ROS 节点。

```text
Agent → Executor → NodeRegistry → BaseNode.execute(context, inputs)
                                      │
                                  SkillNode
                                      │
                              SkillConnector.call
                                      │
                           本地 mock / MHS → 机器人 skill set
```

`hei_agent_bridge` 默认仍是只读观测适配器；独立的 `agent_skills.launch.py` 显式开启
机器人侧 skill set，具体见 [机器人技能说明](../ros2_ws/src/hei_agent_bridge/README.md#mvp-skill-set)。
整机 launch 默认值未变。MHS 后端会调用远端实际发布的技能，可能是真实 ROS 动作。

## 运行

服务器或开发机上，在仓库根目录创建独立环境并安装：

```bash
python3 -m venv agent/.venv
agent/.venv/bin/python -m pip install -e ./agent
agent/.venv/bin/hei-agent
# 模拟机器人未就绪；进入失败终点，退出码 1
agent/.venv/bin/hei-agent --not-ready
# 模拟预抓取动作失败，后续搬运节点不会执行
agent/.venv/bin/hei-agent --fail-at pre_grasp
# 只校验，不执行
agent/.venv/bin/hei-agent --validate
```

也支持 `agent/.venv/bin/python -m hei_agent`；普通安装用 `pip install ./agent`。
无需安装的源码运行方式为
`PYTHONPATH="agent/src${PYTHONPATH:+:$PYTHONPATH}" python3 -m hei_agent`。
示例只是模拟到达两个命名点位；没有标定位置，也不声称完成了真实抓取。
默认 trace 写入当前目录下已被 Git 忽略的 `log/hei_agent/<run_id>.jsonl`。

这一侧不需要 source ROS 环境，也不经过机器人侧的 `scripts/setup.bash`。
该脚本继续负责 ROS/Rust 和机器人 bridge 依赖。

若使用过本项目早期的 ROS 包布局，旧工作区可能残留 `ros2_ws/build/hei_agent` 和
`ros2_ws/install/hei_agent`；它们已不参与构建，可删除这两个包级产物目录，并在新终端
加载工作区。不要删除仍在使用的 `hei_agent_bridge`。

## 接入 LLM

在仓库根目录安装可选依赖，并创建本地配置：

```bash
agent/.venv/bin/python -m pip install -e './agent[llm]'
cp agent/llm.env.example agent/.env
# 编辑 agent/.env：填写 API 地址、模型 ID 和需要的 API Key
set -a
source agent/.env
set +a

# 生成、校验、保存图，再执行 mock
agent/.venv/bin/hei-agent --task '检查机器人状态，右臂先到预抓取位，再到搬运位'

# 只生成并保存，不执行
agent/.venv/bin/hei-agent --task '右臂到预抓取位' --plan-only
```

`agent/.env` 和 `agent/.env.local` 已被 Git 忽略，CLI 不会自动加载它们。
不要把 API Key 填进命令行参数或 graph JSON。

| 配置 | 用途 |
| --- | --- |
| `LLM_BASE_URL` | API 基础地址，例如 `https://api.openai.com/v1` 或 `http://服务器:8000/v1`；不包含 `/chat/completions` |
| `LLM_MODEL` | 服务端实际支持的模型 ID，必须设置；不预设模型 |
| `LLM_API_KEY` | API Key；也支持回退到 `OPENAI_API_KEY`；无鉴权的本地接口可以留空 |
| `LLM_TIMEOUT_SEC` | 单次模型请求总超时，默认 60 秒 |
| `LLM_RESPONSE_FORMAT` | `json` 默认发送 `response_format: {type: json_object}`；接口不支持时设为 `text` |
| `LLM_TRUST_ENV` | 默认 `true`，继承 HTTPX 代理与 CA 环境设置；`false` 禁用环境代理和环境 CA 配置，用于需要直连的服务 |

`--llm-base-url`、`--llm-model`、`--llm-timeout` 和 `--llm-response-format` 可覆盖环境配置，
没有 API Key 命令行选项。OpenAI 官方地址必须配置 Key；自定义地址未配置时 SDK 使用
`not-required` 占位值。地址中的凭据、查询参数和片段不被接受。

若提示 `LLM HTTP client initialization failed`，表示尚未发送模型请求，先检查
`HTTP_PROXY/HTTPS_PROXY/ALL_PROXY` 及其小写形式、`SSL_CERT_FILE/SSL_CERT_DIR`。
SOCKS 地址推荐 `socks5://` 或 `socks5h://`；Agent 会将环境中的旧式 `socks://`
在客户端内部转换为 `socks5://`，保留代理优先级与 NO_PROXY，不修改进程环境变量。
该适配使用 HTTPX 0.28 的代理映射内部接口，因此 LLM extra 将 HTTPX 限制在 0.28.x。
局域网模型需要直连时可在 `agent/.env` 设置 `LLM_TRUST_ENV=false`，重新加载环境并重启
Agent；此开关也会忽略环境 CA 配置，TLS 默认校验仍开启。默认 `true` 保留已有代理设置。
客户端初始化与响应解析分别报告错误，不记录代理 URL、密钥或服务端错误正文。

适配器按 [官方 Chat Completions 接口](https://developers.openai.com/api/reference/resources/chat/subresources/completions/methods/create)
实现，固定非流式请求；没有强制设置 temperature 或 token 参数，以减少不同服务的差异。
JSON 模式保证的范围不包含本项目图的正确性，因此无论 `json/text` 都会再次执行本地校验。
目前要求标准的 `choices[].message.content` 文本和 `finish_reason: stop`；拒绝、截断、
工具调用、空响应均不会进入执行器。依赖包含 SOCKS 支持，网络连接沿用进程的代理配置；
本地服务应按部署环境设置 `NO_PROXY`。

默认服务器内存 mock 模式中，模型看到的技能目录只有：

- `get_robot_state()`：返回 mock 就绪状态。
- `move_to_named_pose(side, pose)`：`side` 为 `left/right`，`pose` 为 `pre_grasp/transport`。

不支持的任务应生成明确失败的终点。MHS 模式会替换为机器人实际目录，当前 MVP 新增了
`set_gripper_gap`，但仍没有物体抓取验证、底盘移动或视觉定位技能，也未把相机图像送给 LLM。

生成阶段与执行阶段分离：

```text
自然语言任务 + 技能目录 + 任务输入
              ↓
      LLM → JSON 图 → 结构/能力/类型/依赖校验
              ↑                 │
              └── 校验错误修正 ─┘
                                ↓
                     保存 graph.json → Agent 执行
```

默认允许 1 次修正，即最多 2 次模型请求；`--repair-attempts 0/1/2` 可调整。
修正只发生在图执行之前，不会重试机器人动作。API 超时、HTTP 错误或拒绝响应不自动重试。
结构、技能名称、参数类型/枚举、分支结果和数据依赖校验失败时，不执行任何节点。
数据依赖检查要求生产节点出现在所有通往消费节点的路径上，并排除无输出的失败路径。
这些检查不能证明图正确理解了自然语言，也不能证明物理任务完成或运动安全。

默认生成记录位于 `log/hei_agent/plans/<plan_id>/`（可用 `--plan-dir` 覆盖）：

```text
request.json        任务、输入、技能目录、初始提示词和非密钥配置
attempt-1.json      模型输出、用量、校验结果；修正时有 attempt-2.json 等
graph.json          仅校验通过后保存，可通过 --graph 再执行
result.json         生成结果或失败/取消状态
execution.json      实际执行后记录 run_id 和执行 trace 路径
```

生成记录不写入 API Key 或请求 Authorization。原始模型输出仅作为数据保存，
不会执行其中的 Python、shell 或 import。退出码：成功生成/执行为 0，图执行失败为 1，
配置、API 或生成校验失败为 2，Ctrl+C 为 130。

Python 中也可以单独使用规划器：

```python
from hei_agent import Agent, GraphPlanner, LLMConfig, OpenAIChatModel
from hei_agent.mock import MockConnector

async def execute_task(task):
    connector = MockConnector()
    planner = GraphPlanner(OpenAIChatModel(LLMConfig.from_env()), connector.catalog)
    plan = await planner.generate(task, {'side': 'right'})
    return await Agent(plan.graph, connector).run({'side': 'right'})
```

## Python 定义与自定义节点

```python
import asyncio
from hei_agent import Agent, Edge, FunctionNode, Graph, Node, NodeRegistry, NodeResult

async def observe(context, inputs):
    # 实现应使用可取消的异步 I/O，避免在事件循环里阻塞。
    return NodeResult(outputs={'ready': True})

registry = NodeRegistry().register('observe', FunctionNode(observe))
graph = Graph(
    name='readiness', entry='observe',
    nodes=(
        Node('observe', 'observe'),
        Node('check', 'branch', {'condition': {'$ref': 'nodes.observe.ready'}}),
        Node('done', 'end', {'status': 'succeeded'}),
        Node('failed', 'end', {'status': 'failed', 'message': 'Not ready'}),
    ),
    edges=(
        Edge('observe', 'check'),
        Edge('check', 'done', on='true'),
        Edge('check', 'failed', on='false'),
    ),
)
agent = Agent(graph, registry=registry)
result = asyncio.run(agent.run(trace_dir='log/hei_agent'))
print(result.status.value)
```

JSON 版本见 [named_pose.json](src/hei_agent/examples/named_pose.json)，用 `Graph.load(path)`
加载。运行前会复制并校验整张图，检查所有节点 kind 已注册；加载文件不会执行脚本、
动态 import 或 `eval`。

## 执行约定

- **输入与输出**：仅支持有限值 JSON 数据。`NodeResult` 包含 `outcome`、`outputs`、
  `message`。`{"$ref": "inputs.side"}` 读取任务输入；
  `{"$ref": "nodes.observe.ready"}` 读取已执行节点的输出。点号只访问字典键，
  不解析表达式或数组下标。不存在的数据引用触发 `error`，不会补默认值。
- **节点**：内置 `skill`、`branch`、`end`。`skill` 输入为 `skill` 名和 `arguments`；
  `branch` 严格接受布尔值，产生 `true/false`；`end` 必须明确声明
  `succeeded/failed`。普通节点执行成功不代表整图成功，也不证明物理任务完成。
- **边**：按 `(source, outcome)` 精确匹配，每个结果最多一条边。默认结果是
  `success`。未处理的结果直接使整图失败；`timeout` 未处理时为 `timed_out`。
  异常转换为 `error`。可显式连接 `failure/error/timeout` 分支，执行器不会自动重试。
- **循环**：允许显式回边，受 `max_steps`（包含终点节点）与图总超时共同约束。
  每次访问有独立 `visit` 和 `call_id`。节点输出表示最近一次有效访问的数据，
  不是自动刷新的观测；重新访问节点时先清除该节点旧输出。`error/timeout/failure`
  不提供后续可引用输出。没有数据新鲜度检查或跨循环同步语义。
- **超时与取消**：节点超时先取消本地执行任务，并调用该实现的 `cancel`；二者
  完成后才允许走 `timeout` 分支。图总超时直接结束运行，不再走恢复分支。
  `agent.cancel()` 仅提交请求；等待 `agent.run()` 返回才能知道清理结果。
  调用方直接取消 asyncio Task 时，同样先清理，再保留 `CancelledError` 语义。
- **取消确认**：`SkillConnector.cancel(call_id)` 必须确认对应执行已终止，包含
  请求尚未被接受时的竞态；仅“取消请求已发送”不够。确认失败或清理超时会令运行
  `failed`、`cleanup_complete=False`，并锁定此执行器，阻止后续步骤和新任务。
  真机状态需要独立核实后再建立新执行器；重建对象本身不会停止设备。
- **协作式异步**：`FunctionNode` 要求 `async def`。节点不得阻塞事件循环，需在
  `finally` 内回收自己创建的资源；执行器无法强制终止同步阻塞代码、任意后台子任务
  或外部进程。忽略取消的协程会被追踪并锁定执行器。
- **并发范围**：一个 Executor 同时只接受一个任务，每次只执行一个节点。多个
  Executor、遥操作与其他 ROS 客户端间尚无统一资源仲裁，不能据此声称双臂并行安全。
- **记录**：包含图与输入快照、节点访问、解析后输入、结果、选边和最终状态。
  错误结果与反馈保留在 trace。可持久化不等于可恢复执行；不支持自动续跑或动作重放。
  trace 不包含隐式图像二进制，未来感知技能应输出图像资产引用。

## 使用机器人发布的 skill set

推荐使用 Agent 内置 Hub 与进程内 mDNS/DNS-SD：无需 Avahi、D-Bus 或系统 mDNS 服务。
服务器与机器人启动顺序任意，机器人默认等待发现服务后主动连接和注册。
监听地址默认 `0.0.0.0:4180`，接受 `--device-id` 指定的设备。

```bash
# 服务器：首次/依赖变化后准备，无需 ROS
bash scripts/setup.bash --agent-server
# 常驻终端，等待机器人并接收多次任务
agent/.venv/bin/hei-agent --backend embedded --serve --hub-id lab-agent

# 机器人：加载 ROS 和 bridge SDK 环境后运行；默认机器人 mock
ros2 launch hei_bringup agent_skills.launch.py hub_id:=lab-agent
```

机器人首次/依赖变化后执行统一 `scripts/setup.bash`，重建 bridge 和 bringup 并加载工作区。
不指定机器人 `hub_id` 时连接唯一匹配设备 ID 的 Hub；多个匹配实例时等待，不选第一个。
即使两个实例的 hub_id 相同也视为歧义。已有健康命令连接不受发现列表变化影响。

在服务器常驻终端输入一行 JSON 就提交一次请求，例如先查看目录，再执行任务：

```json
{"list_skills":true}
{"task":"检查状态，将右夹爪名义开度设置为0.025米","side":"right"}
```

也支持 `{"graph":"/绝对路径/graph.json"}`；可选 `side`、`plan_only`（仅 task）、
`validate`（仅 graph）。逐行返回 JSON，串行处理；Hub/机器人连接在请求之间保持。
EOF（Ctrl-D）或 Ctrl-C 退出。stdin 必须是终端或管道，目前不是 HTTP 服务或持久任务队列。
取消/停止未确认时关闭服务并拒绝后续任务。LLM 配置沿用 `LLM_*` 环境变量。

手动安装两个本地包也可以：
`pip install -e ./packages/hei-discovery -e './agent[llm,mhs]'`。
可用 `--listen-host`、`--listen-port` 调整监听；无需为 embedded 模式设置 `--hub-url`。
真机执行需要机器人显式使用 ROS skill 配置并启动底层控制器，参见机器人包 README。

发布类型 `_hei-mhs._tcp.local.` 是本项目约定，不是官方 MHS 服务类型。SRV/A 提供地址端口，
TXT 提供 hub_id、device_id、protocol=mhs/v1、transport=ws；没有密钥和技能内容。
启动广播前先绑定 Hub 端口，退出先等待撤销广播再关闭 Hub；非正常退出通过 15 秒 TTL 失效。
机器人断线后重新发现并轮换候选 IPv4 地址，保留本地技能账本和锁，不重放运动调用。
注册拒绝、设备替换和协议不支持依然终止 bridge，不无限重试这些错误。

MVP 仅 IPv4 同一局域网链路，网络需允许 UDP 5353 多播。默认使用所有非回环 IPv4 网卡；
多网卡时建议服务器加 `--discovery-interface 192.168.1.10`，机器人加
`discovery_interface:=192.168.1.20`，填各自本机网卡 IP（多地址用逗号分隔），不是远端地址。
具体 `--listen-host` 必须是发布网卡之一。测试回环地址需显式选 `127.0.0.1`。
跨 VLAN 或不支持多播时可用 `--no-discovery` 和机器人 `hub_url:=ws://地址:端口`；
显式地址完全绕过发现。外部 AgnesHub 若未发布本项目服务类型，也应使用显式地址。

内置 Hub 与 Graph/Executor 同属服务器 Agent 进程，`MHSConnector` 直接调用进程内
`EmbeddedHub.invoke()`。机器人侧 skill 不变，仍使用 `/ws/mhs` 注册和执行协议。
接单 ACK 不代表完成；最终结果、目录版本核对和取消确认与外部 Hub 模式相同。
命令连接断开后不自动重发调用；SDK 会触发机器人侧停止逻辑，Agent 不据此声称停止已确认。

目录可发布 `get_camera_observation(camera)` 和 `sample_depth_point(observation_id,u,v)`。
前者把 RGB、可用的对齐深度和内参传回，校验后保存至 `TRACE_DIR/observations/<id>/`，
生成 rgb.jpg、depth.npy、metadata.json；图与 trace 中只记录引用。后者对固定快照测量，
u/v 为归一化坐标，输出相机光学坐标 XYZ 与距离（米）。当前文本规划器不看图或识别物体。
`examples/camera_depth.json` 是只读图模板。CLI 发现机器人目录后，会先完整校验未绑定模板，再绑定当前 `catalog_revision`；已有 revision 的图仍严格检查版本，失配时拒绝执行。
常驻终端可提交 `{"task":"获取front的RGB-D观测，并测量图像中心的三维点和距离"}`。
真实图像需 ROS skill 后端和 RGB-D 驱动；mock 数据明确标记 simulated。

这是单设备的技能传输子集：只接受配置的设备 ID，重复连接被拒绝，不替换正在工作的连接；
该 ID 是路由限制，不是身份认证。当前适用于可信局域网，未提供 TLS/认证。
`/ws/nerve` 流仅接收并丢弃，尚未进入 Agent 的感知输入，也不提供 AgnesHub 的网页、
`/ws/hub` 北向 API、历史图像、人工遥控和多设备调度。协议实现仅校验技能链路需要的字段，
不宣称完整 MHS 一致性。需要这些上游功能时继续使用外部 Hub。

不使用 `--serve` 时仍保留一次性模式：指定 `--task`、`--graph` 或 `--list-skills`，
目录查看、规划或图执行结束后关闭内置监听；`--wait-device` 控制单次等待时间（默认 30 秒）。

如果已有独立 AgnesHub，可继续使用原来的外部模式：

服务器使用上面的统一安装入口；机器人加载 SDK/ROS 环境后启动
`ros2 launch hei_bringup agent_skills.launch.py hub_url:=ws://服务器地址:4180`。
这个独立入口默认使用机器人侧 mock，不启动整机控制栈。ROS 配置与示教点需要另外提供。

```bash
agent/.venv/bin/hei-agent --backend mhs --hub-url ws://服务器地址:4180 \
  --device-id hei-01 --list-skills

agent/.venv/bin/hei-agent --backend mhs --hub-url ws://服务器地址:4180 \
  --device-id hei-01 --task '检查状态，将右夹爪名义开度设置为0.025米'
```

Hub 地址是基础 origin，不含 `/ws/hub`。也可用 `MHS_HUB_URL`、`MHS_DEVICE_ID` 环境变量。
MHS 模式必须显式指定任务或图，不能隐式执行内置示例。`--list-skills` 不执行图。
输出 `backend=mhs` 表示传输方式，`execution_backend=mock/ros` 表示机器人实际执行方式。

`MHSConnector.discover()` 调用远端 `get_skill_catalog`，构造同一格式的 SkillCatalog；
GraphPlanner 使用这个目录，而非服务器写死的真实技能列表。图保存 `catalog_revision`，
Executor 在第一个节点之前核对，机器人在每次调用时再核对。旧图版本不匹配会拒绝，
不自动修正后续物理动作。无版本的手写图不能直接用于 MHS，需先用发现的目录编译绑定。

MHSConnector 不将接单 ACK 当作执行成功。通过 Hub 的最终结果获取本地 outcome/outputs；
取消使用独立 invocation_id，等待机器人 cleanup_complete。失败或断线状态未知时，
不会自动重连重发；图执行器在取消未确认时停止推进并锁定。MHS 的目录摘要与调用 envelope
是本项目约定，不假定上游协议自带技能版本字段。详细协议和边界见机器人 README。

## 验证

```bash
# 安装可选测试与 LLM SDK 依赖，完整验证包含回环 HTTP 服务
agent/.venv/bin/python -m pip install -e './agent[llm,test]'
agent/.venv/bin/python -m pytest agent/tests -q
# 或在 agent 目录内执行 python -m pytest（使用 agent/pyproject.toml 配置）
```

覆盖图校验、数据绑定、成功/失败分支、异常、有限循环、节点和图超时、主动取消、
asyncio 取消、重复运行保护、取消确认失败、忽略取消、JSONL、规划修正、类型与数据依赖、
真实 SDK 请求格式、API 错误、超时、截断、密钥不落盘和 CLI 全流程。
SDK 集成测试连接本地模拟 HTTP 服务，不请求外部 LLM、不消耗付费 token、不操作机器人；
未安装 `llm` extra 时这些集成测试会跳过，其余测试仍可单独运行。

右臂抓取只保留 OBB 流程，示例目标为蓝色方块（`blue cube`）。
`src/hei_agent/examples/right_object_obb.json` 只感知，`right_obb_grasp_plan.json`
运行感知和 IK/碰撞规划，`right_obb_grasp_execute.json` 在验证通过后运动并闭爪。
机器人需使用 `skills.ros.grasp.json` 配置。闭爪后停在原位，不自动抬升或返回。
长时 skill 的节点预算可使用 catalog timeout 加最多 5 秒（上限 310 秒）；
此类 catalog 的图预算上限为 600 秒，普通 catalog 仍为 300 秒。
详见 [右臂抓取说明](../docs/right-grasp.zh-CN.md)。

通用右臂末端接口为 `get_arm_pose`、`plan_arm_position`、`plan_arm_pose`、
`execute_arm_plan`。现用抓取配置重启 bridge 后会同时发布；也可选择独立 arm 配置。
`examples/arm_pose.json` 读取位姿，`examples/arm_position_plan.json` 只规划保持当前位置；
执行示例为 `examples/arm_position_execute.json`。详见[末端运动说明](../docs/arm-motion.zh-CN.md)。
