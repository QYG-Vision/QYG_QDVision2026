# infantry-jeson 串口协议（电控对接）

对应 **`infantry-jeson` 分支本次修改后的工作区**，基于提交 `dfdfef6`。当前采用 **24 字节单帧、float32 弧度协议**；视觉与电控固件须同步升级。

串口：**921600 bit/s，8N1，无流控**，设备配置 `/dev/ttyUSB0`，协议选项 `infantry`。视觉→电控固定 **24 字节**，电控→视觉固定 **24 字节**；帧头 **`0xFF`**，帧尾 **`0x0D`**。每次指令或反馈均由一帧完整表达。

以下字节编号均从 **1** 开始（代码偏移为编号减 1）。所有多字节字段采用**大端序（高字节在前）**。float32 为 **IEEE 754 单精度，直接表示物理值，不乘除 100**。

## 视觉 → 电控：24 字节

| 字节 | 字段 | 类型及含义 |
| --- | --- | --- |
| 1 | `header` | uint8，固定 `0xFF` |
| 2 | `fire_advice` | uint8，1 建议开火，0 不建议开火 |
| 3–6 | `yaw` | float32，云台目标绝对偏航角，rad，向左为正 |
| 7–10 | `pitch` | float32，云台目标绝对俯仰角，rad，向下为正 |
| 11–14 | `distance` | float32，目标距离，m；求解器通常以 −1 表示无目标 |
| 15 | `id` | uint8，目标编号 |
| 16–19 | `target_v_yaw` | float32，敌方目标自转角速度，rad/s |
| 20 | `target_state` | uint8，原始距离有限且大于 0 时为 1，否则为 0 |
| 21–22 | `reserved` | uint8[2]，保留，填 0 |
| 23 | `check_byte` | uint8，预留校验位，填 0，当前无 CRC/BCC |
| 24 | `tail` | uint8，固定 `0x0D` |

目标编号：0 未知，1～5 对应机器人编号，6 前哨站（`outpost`），7 哨兵（`sentry`），8 基地（`base`），9 `negative`；无法识别的编号字符串回退为 0。

`yaw/pitch` 是求解器输出的目标绝对角，可能包含角速度与额外预测时间产生的提前量。`target_v_yaw` 是敌方目标自转速度，不能作为本机云台角速度前馈。

当前协议没有发送云台参考角速度、角加速度、误差、`exp_yaw/exp_pitch`、命令时间戳或裁判系统数据；不能将 ROS 消息中的其他字段当作线路字段。

`target_state` 只表达距离条件，不保证其他字段合法。发送编码器对 NaN、Inf 或超出 float32 范围的浮点输入单独置 0；范围内的有限数值直接转换为 float32，不再按 int16 限幅。例如 `distance=0.001 m` 在线路上仍约为 `0.001`，状态为 1。

无目标时求解器通常输出不开火、距离 −1、角度和编号为 0。编码器保持原有各字段独立编码行为：不会仅因 `target_state=0` 自动清零其他字段、强制距离为 −1 或强制关闭开火建议。电控应结合目标状态与距离判定是否应用指令，不能把无目标的零角度当作回零命令；`fire_advice` 仅为开火建议。

## 电控 → 视觉：24 字节

| 字节 | 字段 | 类型及含义 |
| --- | --- | --- |
| 1 | `header` | uint8，固定 `0xFF` |
| 2 | `mode` | uint8，视觉模式，合法值 0～5 |
| 3–6 | `roll` | float32，实际横滚角，rad，按约定坐标系右手旋转 |
| 7–10 | `pitch` | float32，实际俯仰角，rad，向下为正 |
| 11–14 | `yaw` | float32，实际偏航角，rad，向左为正 |
| 15–18 | `bullet_speed` | float32，电控提供的实际弹速，m/s |
| 19–22 | `mcu_timestamp` | uint32，同一次姿态采样的 MCU 毫秒时间戳 |
| 23 | `check_byte` | uint8，预留校验位，电控填 0，当前视觉不校验 |
| 24 | `tail` | uint8，固定 `0x0D` |

`mode` 含义：

| 值 | 视觉模式 |
| --- | --- |
| 0 | 识别红色装甲板 |
| 1 | 识别蓝色装甲板 |
| 2 | 识别红色小能量机关 |
| 3 | 识别蓝色小能量机关 |
| 4 | 识别红色大能量机关 |
| 5 | 识别蓝色大能量机关 |

模式 0 表示识别红色装甲板。`infantry` 同时订阅自瞄与能量机关指令，两者使用相同编码。

电控应锁存一次完整采样后再编码，角度、弹速和时间戳须属于同一快照。当前反馈不包含实际 yaw/pitch 角速度。

视觉收齐 24 字节、帧头尾正确、`mode∈[0,5]` 且四个浮点值均有限时，才更新 `/serial/receive`、模式与 TF。非法反馈不会提交部分字段；后续合法帧可直接恢复。负弹速等有限值不会被协议解码器拒绝，求解器仍按原有弹速有效性规则处理。

## 编解码与时序

- 所有串口角度使用 **rad**，目标自转角速度使用 **rad/s**，距离使用 **m**，弹速使用 **m/s**。上述物理量均为 float32；模式、编号、标志位保持 uint8，时间戳保持 uint32。
- float32 大端示例：`0.5 → 3F 00 00 00`，`−0.25 → BE 80 00 00`，`1.0 → 3F 80 00 00`。MCU 不应直接发送小端 float 结构体，应复制位模式后按高字节到低字节输出。
- 线路布局按字节紧凑排列，没有结构体对齐填充。部分 float 字段起始位置未按 4 字节对齐，电控应使用字节操作或 memcpy 编解码。
- 收发使用相同参考系与正方向，默认对应 `odom → gimbal_link`。TF 直接使用反馈弧度。线路编码器不额外对 yaw 归一化，电控不能依赖它自动限制到 ±π。
- 时间戳使用 uint32 毫秒计数，不转 float32。解码允许时间戳 0；串口节点沿用原逻辑：大于 0 时进行 MCU 到 PC 时钟映射并应用 `timestamp_offset`，等于 0 时使用 PC 接收时间。重复快照应保留原采样时间。
- 按 **24 字节定长**解析；载荷内出现 `0xFF` 或 `0x0D` 不作为分隔符，短读保留半帧。当前无 CRC/BCC，有限值与模式检查不能替代载荷校验。
- 命令随话题回调发送，协议没有固定发送频率、序号、分片类型、确认、重传或分组超时。电控在收齐完整命令帧后应用指令，并按本地接收时间实现控制超时失效。
- `hero`、`air`、`sentry` 已同步采用相同的 24 字节布局；各自订阅话题及模式服务不同。哨兵底盘指令发送逻辑仍未启用。

### 完整帧样例

视觉下发 `fire_advice=1, yaw=0.5 rad, pitch=−0.25 rad, distance=1.25 m, id=3, target_v_yaw=0.125 rad/s, target_state=1`：

```text
FF 01 3F 00 00 00 BE 80 00 00 3F A0 00 00 03 3E 00 00 00 01 00 00 00 0D
```

电控反馈 `mode=1, roll=−0.125 rad, pitch=0.25 rad, yaw=1.0 rad, bullet_speed=16 m/s, mcu_timestamp=0x12345678`：

```text
FF 01 BE 00 00 00 3E 80 00 00 3F 80 00 00 41 80 00 00 12 34 56 78 00 0D
```

## 迁移与验证

本版替换先前的 16 字节多帧方案：收发均使用一帧 24 字节，已移除分片类型、组序号及组装器。也不兼容原来的 16 字节 int16 定点单帧协议，双方固件必须统一使用本表。

自瞄与能量机关求解、角度误差和角速度、检测角度阈值、手动角度补偿、虚拟串口、回放 TF、调试消息与可视化继续统一使用 rad / rad/s。相关 YAML 阈值和补偿值已换算为等价弧度，内部原有 double/Eigen 精度保留；**float32 是线路物理量的类型**。

PnP 双解筛选中原有 `10 * 180 / π` 阈值已修正为 `0.17453292519943295 rad`（原意为 10°）；这会修复该筛选条件原本几乎不生效的问题，需要在实机图像上复核。OpenCV 自身 API 的角度约定遵循库定义，业务灯条倾角由几何坐标计算为弧度。

旧录包、CSV 或外部节点若仍提供度值，必须在输入边界完成迁移，当前回放不会自动判断单位。ROS 消息字段名大体保持原样，不能据此认为旧度值数据仍然兼容。

回归测试覆盖完整 24 字节大端样例、非法反馈不修改输出及恢复、浮点异常值、模式与时间戳边界、短读与连续帧。当前宿主机没有 Docker，尚未完成 ROS 容器编译和 gtest；可在已配置环境的容器中执行：

```bash
docker exec -it rv_devel_ bash
cd /ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rm_interfaces --parallel-workers 4
source install/setup.bash
colcon build --symlink-install --packages-up-to armor_detector armor_solver rune_solver rm_serial_driver rm_auto_replay --parallel-workers 4
source install/setup.bash
colcon test --packages-select rm_serial_driver armor_detector rune_solver
colcon test-result --verbose
```

## 代码依据

- [float_protocol.hpp](src/rm_hardware_driver/rm_serial_driver/include/rm_serial_driver/float_protocol.hpp)：24 字节布局、大端 float32 编解码及反馈校验。
- [protocol.hpp](src/rm_hardware_driver/rm_serial_driver/include/rm_serial_driver/protocol.hpp)：目标编号与 ROS 数据接口。
- [infantry_protocol.cpp](src/rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol.cpp)：步兵实际收发与话题订阅。
- [fixed_packet_tool.hpp](src/rm_hardware_driver/rm_serial_driver/include/rm_serial_driver/fixed_packet_tool.hpp)：定长帧提取与短读缓存。
- [serial_driver_node.cpp](src/rm_hardware_driver/rm_serial_driver/src/serial_driver_node.cpp)：采样时间映射和姿态 TF。
- [serial_driver_params.yaml](src/rm_bringup/config/node_params/serial_driver_params.yaml)：串口参数。
- [test_protocol_encoding.cpp](src/rm_hardware_driver/rm_serial_driver/test/test_protocol_encoding.cpp)：完整帧字节与校验回归测试。
