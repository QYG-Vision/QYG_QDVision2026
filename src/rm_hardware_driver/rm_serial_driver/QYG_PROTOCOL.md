# QYG 哨兵串口协议

本文档记录当前 `qyg_sentry` 协议。串口默认波特率为 `921600`，协议结构体使用
1 字节对齐。多字节整数和 IEEE-754 `float32` 均按小端序传输。

> 修改帧结构后，上下位机必须同时更新帧长度、字段偏移和 CRC 范围。

## 电控发送至视觉（39 字节）

帧头为 ASCII `GD`，CRC 校验前 37 字节。

| 偏移 | 长度 | 字段 | 类型 | 单位/说明 |
| ---: | ---: | --- | --- | --- |
| 0 | 2 | `header` | `uint8[2]` | 固定为 `0x47 0x44`（`GD`） |
| 2 | 1 | `current_mode` | `uint8` | QD 视觉模式，直接使用 `VisionMode` 的 0～5 编号 |
| 3 | 4 | `actual_vx` | `float32` | 底盘实际 x 速度，m/s |
| 7 | 4 | `actual_vy` | `float32` | 底盘实际 y 速度，m/s |
| 11 | 4 | `actual_wz` | `float32` | 底盘实际角速度，rad/s |
| 15 | 2 | `sentry_state` | `uint16` | 哨兵状态原始值，16 位不移位、不掩码地透传 |
| 17 | 4 | `vyaw` | `float32` | 云台 yaw，度 |
| 21 | 4 | `vpitch` | `float32` | 云台 pitch，度，抬头为正 |
| 25 | 4 | `vroll` | `float32` | 云台 roll，度 |
| 29 | 4 | `bullet_speed` | `float32` | 实时弹速，m/s |
| 33 | 4 | `mcu_timestamp` | `uint32` | 云台角采样时刻，ms |
| 37 | 2 | `crc16` | `uint16` | 前 37 字节 CRC，小端序 |

`current_mode` 与 QD `VisionMode` 直接对位：

| 值 | 模式 |
| ---: | --- |
| 0 | `AUTO_AIM_RED` |
| 1 | `AUTO_AIM_BLUE` |
| 2 | `SMALL_RUNE_RED` |
| 3 | `SMALL_RUNE_BLUE` |
| 4 | `BIG_RUNE_RED` |
| 5 | `BIG_RUNE_BLUE` |

收到大于 5 的 `current_mode` 时，上位机保持上一次合法模式，不把非法枚举传入 QD
状态机。相同非法值连续出现时只记录一次警告，恢复合法模式后重新启用告警检测。

`sentry_state` 不再参与视觉模式解析。该字段收到的 16 位数值会原样发布到
`SerialReceiveData.sentry_state`，以保证原有状态数据的比特位置不变。

`mcu_timestamp` 应与 `vyaw`、`vpitch`、`vroll` 在同一个控制周期采样。

## 视觉发送至电控（34 字节）

帧头为 ASCII `QY`，CRC 校验前 32 字节。

| 偏移 | 长度 | 字段 | 类型 | 单位/说明 |
| ---: | ---: | --- | --- | --- |
| 0 | 2 | `header` | `uint8[2]` | 固定为 `0x51 0x59`（`QY`） |
| 2 | 1 | `mode` | `uint8` | 云台控制和开火状态，见下表 |
| 3 | 4 | `yaw` | `float32` | 目标 yaw，rad，限幅到 `[-π, π]` |
| 7 | 4 | `pitch` | `float32` | 目标 pitch，rad，限幅到 `[-π, π]` |
| 11 | 4 | `linear_x` | `float32` | 底盘 x 指令，限幅到 `[-1, 1]` 后取反 |
| 15 | 4 | `linear_y` | `float32` | 底盘 y 指令，限幅到 `[-1, 1]` 后取反 |
| 19 | 4 | `angular_z` | `float32` | 底盘角速度指令，限幅到 `[-1, 1]` 后取反 |
| 23 | 4 | `distance` | `float32` | 目标距离，m；`-1` 表示无有效目标 |
| 27 | 1 | `target_id` | `uint8` | 目标装甲板类型编号，见下表 |
| 28 | 4 | `target_v_yaw` | `float32` | 目标自转角速度，rad/s |
| 32 | 2 | `crc16` | `uint16` | 前 32 字节 CRC，小端序 |

`mode` 定义：

| 值 | 含义 |
| ---: | --- |
| 0 | 禁止云台控制和开火 |
| 1 | 允许云台控制，禁止开火 |
| 2 | 允许云台控制和开火 |

`target_id` 定义：

| 值 | 目标 |
| ---: | --- |
| 0 | 无目标/未知目标 |
| 1～5 | 1～5 号装甲板 |
| 6 | 前哨站（`outpost`） |
| 7 | 哨兵（`sentry`） |
| 8 | 基地（`base`） |
| 9 | 负样本（`negative`） |

无有效目标时，上位机发送 `mode=0`、`distance=-1`、`target_id=0`、
`target_v_yaw=0`。QD 没有单独的 `IDLE` 视觉模式，云台控制使能仅由目标距离是否有效决定。

## CRC 约定

当前实现使用以下逐位算法：

- 初始值：`0xFFFF`
- 反射多项式：`0x8408`
- 不执行最终异或
- 校验范围：从帧头开始，到 `crc16` 字段之前结束
- CRC 在线路上按 `uint16` 小端序发送
- 字符串 `123456789` 的校验值为 `0x6F91`

实现位置：

- `include/rm_serial_driver/protocol/qyg_protocol.hpp`
- `src/protocol/qyg_protocol.cpp`
- `src/protocol/qyg_sentry_protocol.cpp`
