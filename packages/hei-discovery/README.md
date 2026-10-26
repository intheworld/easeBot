# HEI discovery

服务器 Agent 与机器人 bridge 共用的进程内 IPv4 mDNS/DNS-SD 约定，不含 ROS 或 MHS SDK。
依赖 python-zeroconf，不调用 Avahi、D-Bus 或操作系统名称解析器。

发布 `_hei-mhs._tcp.local.`（项目私有服务类型），TXT 包含 `hub_id`、`device_id`、
`protocol=mhs/v1`、`transport=ws`，SRV/A 包含实际监听端口与网卡地址。
这是定位 Hub 的信息，不是身份认证。发现客户端直接使用解析后的 IPv4 建立 WebSocket。

默认所有非回环 IPv4 网卡；`interfaces` 可填本机 IPv4 地址列表（逗号分隔）。
客户端按逻辑 hub_id 和 device_id 过滤，只在候选实例唯一时返回。一个服务实例的多个地址
视为同一个候选；两个实例即使 hub_id 相同也不会合并。健康连接的维护由调用方负责。
撤销等待 goodbye 广播完成，异常退出后缓存按 15 秒 TTL 过期。仅支持同链路多播。

统一安装：服务器 `bash scripts/setup.bash --agent-server`；机器人正常执行
`bash scripts/setup.bash`。手动安装 `pip install ./packages/hei-discovery` 也可。
详见 [Agent 用法](../../agent/README.md#使用机器人发布的-skill-set)。

上游：[python-zeroconf](https://github.com/python-zeroconf/python-zeroconf)，
[DNS-SD RFC 6763](https://www.rfc-editor.org/rfc/rfc6763)。
