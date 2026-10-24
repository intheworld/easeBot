# Driver dependencies and references

Production transport and protocol come from [motorbridge](https://github.com/motorbridge/motorbridge), pinned to commit [`d5700bcd5b7f9355dd2012488d463c3bac982259`](https://github.com/motorbridge/motorbridge/tree/d5700bcd5b7f9355dd2012488d463c3bac982259). The Cargo lockfile records the complete resolved Rust dependency graph. motorbridge is MIT licensed; its license is installed alongside this package's documentation.

- `motor_core::dm_serial::DmSerialBus`: serial settings, exclusive open, 30-byte USB-CAN transmit frames, 16-byte receive framing.
- `motor_vendor_damiao::protocol`: state/register decoding and feedback, mode-read, enable/disable, POS_VEL/VEL encoding.
- `motorbridge_backend/`: local, private C ABI glue. It keeps bounded manual polling and per-frame fault/mode latches. It closes the transport by dropping it, without calling an all-motor shutdown or flushing a potentially blocked serial link. It uses upstream protocol encoders. Explicit mode configuration operations write RAM CTRL_MODE only while fresh disabled feedback is available: gripper7 uses mode4; optional arm setup uses its configured POS_VEL/VEL mode. They do not save EEPROM, zero positions, clear faults or enable motors. Arm mode setup is opt-in and disabled by default.
- `src/damiao_system.cpp`: ROS 2 interfaces/lifecycle, calibration mapping, command validation and ownership.
- `test/wire_fixture.*`: independent expected command bytes for the PTY emulator only; not part of the installed driver.

The upstream generic ABI/C++ wrapper is deliberately not linked: automatic controller destruction disables all motors, and latest-state-only snapshots can hide an earlier fault in the same receive batch. There are no upstream patches in this integration.

HEI mappings and reference ranges originate from `lipengdong/hei-rebot-lift`, commit `b7c45e1fd71e65e39b98eabb2f091210b3904d22`:

- `software/lerobot-hei-rebot-lift/src/lerobot/robots/hei_rebot_lift/hei_rebot_lift.py`: six arm IDs 1..6, feedback IDs 0x11..0x16, first three DM4340, last three DM4310.
- `software/lerobot-hei-rebot-lift/src/lerobot/motors/damiao_u2can/dm_can.py`: reference adapter/protocol and ranges.

Robot-specific YAML profiles now live in `hei_bringup/config/`. Protocol ranges remain configurable and must match firmware. Sample direction/offset/ratio values are references, not measured calibration.

Plugin API: [ros2_control Jazzy hardware components](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html).

`LiftSystem` 组合现有 `DamiaoSystem`，不增加电机后端 ABI 或固件设零操作。限位 IO 的只读 7 字节帧/CRC16 解析与软件多圈高度参考来自 hei-rebot-lift 的 `_LimitSwitchReader`、`_MultiTurnPositionTracker` 和 `_LiftRuntime`；新增 IO 新鲜度检查、矛盾限位故障、变化率限制后的方向阻挡及停止确认后的软件回零。
