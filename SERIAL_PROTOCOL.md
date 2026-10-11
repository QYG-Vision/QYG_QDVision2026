# 视觉与电控串口通信协议

本文按代码现状整理，供视觉和电控联调使用。主分支 `main` 对应英雄、步兵等非哨兵兵种；`Qlf-Sentry` 分支的默认配置对应 QYG 哨兵。本文只描述 UART 上的字节，不把 ROS 消息字段误当作线协议字段。

核对版本：`main` 为 `de135a66f8aa820e7935f2898b614e819223fc07`，`Qlf-Sentry` 为 `4a5e3479db6e7911b25e67e22c44354f11b48c2b`。以后修改协议时，双方应同时更新此文档和对应的编解码代码。

## 先确认使用哪套帧

| 兵种/配置 | 协议名 | 视觉→电控 | 电控→视觉 | 默认 UART |
| --- | --- | ---: | ---: | --- |
| `main` 的 `hero`、`air`、`infantry` | QD 定长协议 | 16 字节 | 16 字节 | 115200，8N1，无流控 |
| `Qlf-Sentry` 默认的 `qyg_sentry` | QYG 哨兵协议 | 34 字节 | 39 字节 | 921600，8N1，无流控 |

两套协议的帧头、长度、编码、校验均不同，**不可混用**。两个分支的默认设备路径均为 `/dev/rm_usb0`，实际设备名由部署环境决定。`main` 还保留名为 `sentry` 的旧 16 字节实现；它不是本文所说的 `Qlf-Sentry` 默认协议。`main` 的英雄、空中机器人共用 `DefaultProtocol`，步兵使用 `ProtocolInfantry`；三者的串口字节布局相同。

本协议中的偏移从 0 开始。`pitch` 在电控回传及视觉下发中均以**抬头为正**。视觉后续构造 TF 时进行的符号转换不是串口协议的一部分。

### 共用的枚举

电控发送的视觉模式编号在两套协议中均为以下含义：

| 数值 | 视觉模式 |
| ---: | --- |
| 0 | `AUTO_AIM_RED`，自瞄红色目标 |
| 1 | `AUTO_AIM_BLUE`，自瞄蓝色目标 |
| 2 | `SMALL_RUNE_RED`，小符红色目标 |
| 3 | `SMALL_RUNE_BLUE`，小符蓝色目标 |
| 4 | `BIG_RUNE_RED`，大符红色目标 |
| 5 | `BIG_RUNE_BLUE`，大符蓝色目标 |

视觉下发的 `target_id`：`0` 表示无目标或未知；`1`～`5` 对应 1～5 号装甲板；`6` 前哨站；`7` 哨兵；`8` 基地；`9` 负样本。未知字符串也会编码成 `0`。**模式编号和目标编号是两种独立的字段。**

## 一、`main`：QD 16 字节定长协议

帧头固定 `FF`，帧尾固定 `0D`。所有 `int16` 和 `uint32` 字段在线路上按**大端序**排列；`int16` 为有符号二进制补码。角度、距离、弹速和自转角速度采用“实际值 × 100 后转换成整数”的定点编码，解码时除以 100。视觉当前使用 C++ `static_cast<int16_t>`，即小数部分截断；代码没有做范围饱和处理，电控不能假设越界值会自动限幅。

### 视觉→电控：16 字节

| 偏移 | 长度 | 字段 | 线类型 | 单位及含义 |
| ---: | ---: | --- | --- | --- |
| 0 | 1 | `header` | `uint8` | 固定 `0xFF` |
| 1 | 1 | `fire_advice` | `uint8` | `0` 不建议开火，`1` 建议开火 |
| 2 | 2 | `yaw` | 大端 `int16` | 目标云台 yaw，度 × 100 |
| 4 | 2 | `pitch` | 大端 `int16` | 目标云台 pitch，度 × 100，抬头为正 |
| 6 | 2 | `distance` | 大端 `int16` | 目标距离，米 × 100；负值表示无有效目标 |
| 8 | 1 | `target_id` | `uint8` | 目标编号，见共用枚举 |
| 9 | 2 | `target_v_yaw` | 大端 `int16` | 目标自转角速度，rad/s × 100 |
| 11 | 3 | `reserved` | `uint8[3]` | 当前发送代码未赋值，字节值不可靠；电控须忽略 |
| 14 | 1 | `check_byte` | `uint8` | 当前发送代码未赋值，**不是有效校验值** |
| 15 | 1 | `tail` | `uint8` | 固定 `0x0D` |

例如下发 `pitch=+10.00°` 时，偏移 4～5 为 `03 E8`。`distance=-1.00 m` 会编码为有符号 `-100`，即 `FF 9C`。当前下发逻辑直接传输求解器给出的 `fire_advice`，没有在线协议层再用 `distance` 强制关闭开火；电控应同时检查有效目标条件。

### 电控→视觉：16 字节

| 偏移 | 长度 | 字段 | 线类型 | 单位及含义 |
| ---: | ---: | --- | --- | --- |
| 0 | 1 | `header` | `uint8` | 固定 `0xFF` |
| 1 | 1 | `mode` | `uint8` | 视觉模式 0～5，见共用枚举 |
| 2 | 2 | `roll` | 大端 `int16` | 云台 roll，度 × 100 |
| 4 | 2 | `pitch` | 大端 `int16` | 云台 pitch，度 × 100，抬头为正 |
| 6 | 2 | `yaw` | 大端 `int16` | 云台 yaw，度 × 100 |
| 8 | 2 | `bullet_speed` | 大端 `int16` | 电控提供的弹速，m/s × 100 |
| 10 | 4 | `mcu_timestamp` | 大端 `uint32` | 云台角采样时刻，毫秒 |
| 14 | 1 | `check_byte` | `uint8` | 接收代码目前不检查 |
| 15 | 1 | `tail` | `uint8` | 固定 `0x0D` |

例如电控上报 `pitch=+10.00°`，偏移 4～5 应为 `03 E8`；视觉解包后保持 `+10.00°`。电控应发送已经在电控侧处理好的正弹速，视觉不再用固定弹速覆盖它。`mcu_timestamp` 应取与本帧云台角对应的采样时刻，而非发送完成时刻。

### 帧检查和发送时机

视觉接收端目前只检查**长度、帧头和帧尾**，断帧时尝试拼帧；它**不校验**偏移 14 的字节，也不验证 `mode` 是否属于 0～5。电控发来的错误但具有正确帧头/帧尾的 16 字节数据可能被当成有效帧。双方不能把 `check_byte` 当成已启用的 CRC、BCC 或异或校验。为兼容现有代码，电控接收时须忽略视觉下发的保留字节和 `check_byte`。

视觉在收到云台求解指令时发送一帧；协议本身没有序号、确认帧、重传或固定发送频率。`hero`、`air` 订阅自瞄指令，`infantry` 同时订阅自瞄和能量机关指令。

## 二、`Qlf-Sentry`：QYG 哨兵协议

帧字段为 1 字节紧凑布局。多字节整数和 IEEE-754 `float32` 在线路上均为**小端序**；无定点乘 100。电控回传角度为度，视觉下发目标角度为弧度，联调时必须区分。

### 电控→视觉：39 字节

帧头为 ASCII `GD`，即 `47 44`。CRC 覆盖偏移 0～36 的前 37 字节。

| 偏移 | 长度 | 字段 | 线类型 | 单位及含义 |
| ---: | ---: | --- | --- | --- |
| 0 | 2 | `header` | `uint8[2]` | 固定 `47 44`（`GD`） |
| 2 | 1 | `current_mode` | `uint8` | 视觉模式 0～5，见共用枚举 |
| 3 | 4 | `chassis_vx` | 小端 `float32` | 底盘实际 x 速度，m/s |
| 7 | 4 | `chassis_vy` | 小端 `float32` | 底盘实际 y 速度，m/s |
| 11 | 4 | `chassis_wz` | 小端 `float32` | 底盘实际 z 角速度，rad/s |
| 15 | 2 | `sentry_state` | 小端 `uint16` | 16 位哨兵状态原值；视觉不移位、不掩码 |
| 17 | 4 | `yaw` | 小端 `float32` | 云台 yaw，度 |
| 21 | 4 | `pitch` | 小端 `float32` | 云台 pitch，度，抬头为正 |
| 25 | 4 | `roll` | 小端 `float32` | 云台 roll，度 |
| 29 | 4 | `bullet_speed` | 小端 `float32` | 电控提供的弹速，m/s |
| 33 | 4 | `mcu_timestamp` | 小端 `uint32` | 云台角采样时刻，毫秒 |
| 37 | 2 | `crc16` | 小端 `uint16` | 前 37 字节的 CRC |

例如电控上报 `pitch=+10.0°`，偏移 21～24 应为 `00 00 20 41`。当前视觉代码直接保留角度的正负号。收到 `current_mode>5` 时，视觉保存这个原始值用于排查，但使用上一次合法模式；启动后尚无合法值时默认为模式 `0`。`sentry_state` 的具体各位含义在当前视觉协议代码中没有定义，须由电控另行给出位表；视觉只透传原值。

### 视觉→电控：34 字节

帧头为 ASCII `QY`，即 `51 59`。CRC 覆盖偏移 0～31 的前 32 字节。

| 偏移 | 长度 | 字段 | 线类型 | 单位及含义 |
| ---: | ---: | --- | --- | --- |
| 0 | 2 | `header` | `uint8[2]` | 固定 `51 59`（`QY`） |
| 2 | 1 | `mode` | `uint8` | `0` 禁止云台控制和开火；`1` 允许云台控制、禁止开火；`2` 允许云台控制和开火 |
| 3 | 4 | `yaw` | 小端 `float32` | 目标云台 yaw，rad，限幅到 `[-π, π]` |
| 7 | 4 | `pitch` | 小端 `float32` | 目标云台 pitch，rad，抬头为正，限幅到 `[-π, π]` |
| 11 | 4 | `linear_x` | 小端 `float32` | 底盘 x 指令；先限幅至 `[-1,1]`，再取反 |
| 15 | 4 | `linear_y` | 小端 `float32` | 底盘 y 指令；先限幅至 `[-1,1]`，再取反 |
| 19 | 4 | `angular_z` | 小端 `float32` | 底盘角速度指令；先限幅至 `[-1,1]`，再取反 |
| 23 | 4 | `distance` | 小端 `float32` | 目标距离，米；`-1.0` 表示无有效目标 |
| 27 | 1 | `target_id` | `uint8` | 目标编号，见共用枚举 |
| 28 | 4 | `target_v_yaw` | 小端 `float32` | 目标自转角速度，rad/s |
| 32 | 2 | `crc16` | 小端 `uint16` | 前 32 字节的 CRC |

视觉将内部的度数转换成弧度后下发 `yaw`、`pitch`，不再取反 `pitch`。`mode` 由“允许视觉控制云台”且“目标距离非负”共同决定；只有这两项成立且建议开火时才发 `2`。无目标或云台控制关闭时，视觉发 `mode=0`、`distance=-1.0`、`target_id=0`、`target_v_yaw=0`。此时帧中的 `yaw`、`pitch` 仍可能是最近一条指令值，电控必须按 `mode=0` 忽略云台控制字段。`mode=0` **不代表底盘字段为零**：导航指令仍可下发。

底盘三个指令在线路上只是经限幅、取反的浮点值；当前视觉代码没有为它们声明 m/s 或 rad/s 的物理单位，其物理含义需双方另行确认。视觉收到云台指令或底盘速度更新时立即发送整帧，未在协议层固定发送周期。若最终送入帧构造器的任一浮点值为 NaN 或无穷，当前实现发送 `mode=0` 的默认零值载荷和有效 CRC；此异常帧的 `distance` 为 `0` 而非 `-1`，电控仍应以 `mode` 为控制使能依据。距离为 NaN 或负无穷时，外层先将其判为无目标并转换为 `distance=-1`。

### CRC 和收帧规则

当前代码的 CRC 逐位算法为：初值 `0xFFFF`，每字节先与 CRC 低 8 位异或，之后对每位向右移；低位为 1 时再异或 `0x8408`；最终**不做异或**。CRC 字节在线路上低字节在前。ASCII `123456789` 的结果是 `0x6F91`。已有哨兵文档使用了 `CRC-16/DECT` 名称，但联调应以这里的**参数、覆盖范围和测试向量**为准，不能只凭名称选择 CRC 库函数。

视觉接收端校验帧头和 CRC；串流中有噪声、分片、粘包或错误 CRC 时，会重新搜索下一处 `GD`。协议没有序号、确认帧或重传字段。电控发送端须以对应长度和 CRC 输出完整帧。

## 联调核对项

1. 双方先确认协议名、波特率、8N1、无流控；抓包时检查帧头和总长度，不能用 16 字节解析 39/34 字节帧。
2. 用电控 `pitch=+10°`、正弹速样例检查：QD 回传 pitch 字节 `03 E8`；QYG 哨兵回传 pitch 字节 `00 00 20 41`。视觉看到的 pitch 应仍为 `+10°`。
3. 用模式 `0`～`5` 和各目标编号核对枚举；QYG 哨兵额外验证非法 `current_mode` 时的回退行为，以及 `sentry_state` 的原值传输。
4. 用“有目标/无目标 × 建议开火/不建议开火”检查发射控制。哨兵尤其要确认 `mode=0` 时云台不动作，底盘指令仍按独立字段处理。
5. QYG 哨兵用 `123456789 → 0x6F91` 核对 CRC 算法，再核对整帧 CRC 的覆盖范围。QD 16 字节协议目前**没有可用的校验字节**，联调不能把偏移 14 当作校验结果。

## 对应源码

- `main`：[帧结构与目标编号](https://github.com/QYG-Vision/QYG_QDVision2026/blob/de135a66f8aa820e7935f2898b614e819223fc07/src/rm_hardware_driver/rm_serial_driver/include/rm_serial_driver/protocol.hpp)、[16 字节收帧检查](https://github.com/QYG-Vision/QYG_QDVision2026/blob/de135a66f8aa820e7935f2898b614e819223fc07/src/rm_hardware_driver/rm_serial_driver/include/rm_serial_driver/fixed_packet_tool.hpp)、[英雄编解码](https://github.com/QYG-Vision/QYG_QDVision2026/blob/de135a66f8aa820e7935f2898b614e819223fc07/src/rm_hardware_driver/rm_serial_driver/src/protocol/default_protocol.cpp)、[步兵编解码](https://github.com/QYG-Vision/QYG_QDVision2026/blob/de135a66f8aa820e7935f2898b614e819223fc07/src/rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol.cpp)。
- `Qlf-Sentry`：[QYG 帧结构](https://github.com/QYG-Vision/QYG_QDVision2026/blob/4a5e3479db6e7911b25e67e22c44354f11b48c2b/src/rm_hardware_driver/rm_serial_driver/include/rm_serial_driver/protocol/qyg_protocol.hpp)、[CRC 与编解码](https://github.com/QYG-Vision/QYG_QDVision2026/blob/4a5e3479db6e7911b25e67e22c44354f11b48c2b/src/rm_hardware_driver/rm_serial_driver/src/protocol/qyg_protocol.cpp)、[串口收发](https://github.com/QYG-Vision/QYG_QDVision2026/blob/4a5e3479db6e7911b25e67e22c44354f11b48c2b/src/rm_hardware_driver/rm_serial_driver/src/protocol/qyg_sentry_protocol.cpp)、[帧和 CRC 测试](https://github.com/QYG-Vision/QYG_QDVision2026/blob/4a5e3479db6e7911b25e67e22c44354f11b48c2b/src/rm_hardware_driver/rm_serial_driver/test/test_qyg_protocol.cpp)。
