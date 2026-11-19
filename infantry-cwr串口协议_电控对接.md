# infantry-cwr 串口协议（电控对接）

对应 **`infantry-cwr` 分支本次修改后的工作区**，基于提交 `e0b9bd6`。本版保留全部原有字段，采用 **发送 40 字节、接收 32 字节、单帧 float32 弧度协议**。

串口：**921600 bit/s，8N1，无流控**，配置设备 `/dev/ttyUSB0`，协议选项 `infantry`。帧头 **`0xFE`**，帧尾 **`0x0D`**。每次指令或反馈各发送一帧。

以下字节编号均从 **1** 开始（代码偏移为编号减 1）。所有多字节字段均为**大端序（高字节在前）**。物理量全部使用 **IEEE 754 float32 直接值，不乘除 100**；模式、编号、标志位保持 uint8，MCU 时间戳保持 uint32。

## 视觉 → 电控：40 字节

| 字节 | 字段 | 类型及含义 |
| --- | --- | --- |
| 1 | `header_pad` | uint8，固定 `0xFE` |
| 2 | `fire_advice` | uint8，1 建议开火，0 不建议开火 |
| 3–6 | `yaw` | float32，云台规划绝对偏航角，rad，向左为正 |
| 7–10 | `pitch` | float32，云台规划绝对俯仰角，rad，向下为正 |
| 11–14 | `distance` | float32，目标距离，m；无有效参考时为 −1 |
| 15 | `id` | uint8，目标编号 |
| 16–19 | `target_v_yaw` | float32，敌方目标自转角速度，rad/s |
| 20 | `target_state` | uint8，1 表示完整有效规划参考，0 表示无有效参考 |
| 21–22 | `reserved4/reserved5` | uint8[2]，保留，填 0 |
| 23–26 | `yaw_vel_rad_s` | float32，规划 yaw 参考角速度，rad/s |
| 27–30 | `pitch_vel_rad_s` | float32，规划 pitch 参考角速度，rad/s，向下为正 |
| 31–34 | `yaw_acc_rad_s2` | float32，规划 yaw 参考角加速度，rad/s² |
| 35–38 | `pitch_acc_rad_s2` | float32，规划 pitch 参考角加速度，rad/s²，向下为正 |
| 39 | `check_byte` | uint8，预留校验位，填 0，当前无 CRC/BCC |
| 40 | `tail_byte` | uint8，固定 `0x0D` |

目标编号：0 未知，1～5 对应机器人编号，6 前哨站（`outpost`），7 哨兵（`sentry`），8 基地（`base`），9 `negative`；未知编号字符串回退为 0。

两轴角度、角速度和角加速度对应**同一规划时刻、同一参考坐标系和同一正方向**，pitch 导数包含弹道及手动补偿的影响。`target_v_yaw` 是敌方目标自转速度，不能用作云台角速度前馈。ROS 中 `yaw_vel/pitch_vel` 为几何瞄准调试量，不能代替这里的四个 MPC 规划导数字段。

仅当 `reference_valid=true`、目标编号合法、八个浮点物理量均有限且未超出 float32 范围、距离在转换成 float32 后仍大于 0 时，编码器发送 `target_state=1`。不再按 int16 范围限幅，`distance=0.001 m` 可以直接编码为有效正距离。距离下溢为 0、NaN/Inf 或超范围时整份参考无效。

无有效参考时统一发送 **`target_state=0`、`fire_advice=0`、`distance=−1.0 m`，其余目标和运动字段为 0**。电控应忽略无效帧的角度并撤销对应前馈，不能将零角度当作回零命令；开火建议不代替电控自身的开火判断。

能量机关尚未接入完整 MPC 参考，其订阅入口强制清除 `reference_valid`。当前反馈模式为 2～5 时，串口层也强制发送无效参考；启动后尚无反馈时，模式缓存默认为 0。

## 电控 → 视觉：32 字节

| 字节 | 字段 | 类型及含义 |
| --- | --- | --- |
| 1 | `header` | uint8，固定 `0xFE` |
| 2 | `mode` | uint8，视觉模式，合法值 0～5 |
| 3–6 | `roll` | float32，实际横滚角，rad，按约定坐标系右手旋转 |
| 7–10 | `pitch` | float32，实际俯仰角，rad，向下为正 |
| 11–14 | `yaw` | float32，实际偏航角，rad，向左为正 |
| 15–18 | `bullet_speed` | float32，电控提供的实际弹速，m/s |
| 19–22 | `mcu_timestamp` | uint32，姿态与实际角速度的共同采样时间，MCU 毫秒计数 |
| 23–26 | `yaw_actual_vel_rad_s` | float32，实际 yaw 姿态角速度，rad/s |
| 27–30 | `pitch_actual_vel_rad_s` | float32，实际 pitch 姿态角速度，rad/s，向下为正 |
| 31 | `check_byte` | uint8，预留校验位，电控填 0，当前视觉忽略该值 |
| 32 | `tail_byte` | uint8，固定 `0x0D` |

`mode` 含义：

| 值 | 视觉模式 |
| --- | --- |
| 0 | 识别红色装甲板 |
| 1 | 识别蓝色装甲板 |
| 2 | 识别红色小能量机关 |
| 3 | 识别蓝色小能量机关 |
| 4 | 识别红色大能量机关 |
| 5 | 识别蓝色大能量机关 |

模式 0 表示识别红色装甲板。电控应锁存一次完整快照后再编码，确保姿态、姿态角速度和时间戳属于同一次采样；不能直接把未经坐标转换的机体系陀螺仪读数当作姿态角速度。

视觉验证帧头尾、模式范围及全部六个浮点值的有限性；失败时不修改输出快照或模式缓存。合法完整帧才更新 `/serial/receive` 和姿态 TF，非法帧后可直接接收下一份合法快照。有限的负弹速不由协议解码器拒绝，求解器继续执行原有弹速有效性规则。

## 编解码与时序

- float32 大端样例：`0.5 → 3F 00 00 00`，`−0.25 → BE 80 00 00`，`1.0 → 3F 80 00 00`，`−1.0 → BF 80 00 00`。不要把小端 float 结构体内存直接当作线路字节发送。
- 布局按字节紧凑排列，没有编译器对齐填充。部分 float 起始位置未按 4 字节对齐，应使用字节操作或 memcpy 编解码。
- 收发使用同一参考坐标系与正方向，默认对应 `odom → gimbal_link`；TF 直接使用反馈弧度。编码器不额外归一化角度，MPC 输出 yaw 按 ±π 附近的等价角归一化。
- MCU 时间戳保持 uint32 毫秒计数，**0 是合法 tick**，支持自然回绕。重复快照应保留原采样时间，不用发送时间给旧数据续期。节点将其映射为 ROS `header.stamp`，继续应用 `timestamp_offset`；同步时间异常超出 `max_timestamp_age` 时沿用现有 PC 接收时间回退机制。
- 命令 `header.stamp` 是规划 p/v/a 的共同参考时刻，当前为求解时刻加 `predict2send_delay`，不随命令帧发送。电控按本地完整帧接收时间实现控制超时失效。
- 发送随云台指令话题触发；协议没有固定发送频率、序号、确认、重传或分帧组装。载荷内的 `0xFE`、`0x0D` 不作为分隔符，按固定长度解析，短读保留半帧。
- 当前预留校验位未启用 CRC/BCC。模式与有限值检查不等价于载荷校验。
- `infantry`、`hero`、`air`、`sentry` 同步采用本版 40/32 字节布局，订阅话题和模式服务保持各自行为。哨兵底盘发送逻辑仍未启用。

## 完整字节样例

视觉：开火建议 1，yaw `0.5 rad`，pitch `−0.25 rad`，距离 `1.25 m`，编号 3，自转速度 `0.125 rad/s`，有效参考 1，yaw/pitch 速度 `1/−1 rad/s`，加速度 `2/−2 rad/s²`：

```text
FE 01 3F 00 00 00 BE 80 00 00 3F A0 00 00 03 3E 00 00 00 01 00 00 3F 80 00 00 BF 80 00 00 40 00 00 00 C0 00 00 00 00 0D
```

电控：模式 1，roll `−0.125 rad`，pitch `0.25 rad`，yaw `1 rad`，弹速 `16 m/s`，时间戳 `0x12345678`，yaw/pitch 实际角速度 `0.5/−0.5 rad/s`：

```text
FE 01 BE 00 00 00 3E 80 00 00 3F 80 00 00 41 80 00 00 12 34 56 78 3F 00 00 00 BF 00 00 00 00 0D
```

## 迁移与验证

本版与原 32/24 字节、int16 定点角度协议不兼容，双方须同步修改帧长、偏移、类型和单位。仍沿用 `0xFE` 帧头，不协商版本、不自动回退旧格式。

内部求解统一为 rad、rad/s、rad/s²，保留原有 double/Eigen 精度。ROS 导数字段改名为 `*_rad_s`、`*_rad_s2`，旧录包和外部节点须迁移；回放不会自动判断度或弧度。相关配置的等价换算见 [弧度迁移说明](infantry-cwr弧度迁移说明.md)。

`angle_offset` 的 pitch/yaw 手动补偿配置使用角度（°），读取时按 `rad = degree × π / 180` 转换，内部补偿和本协议的线路角度仍为 rad。

已更新编码、非法值拒收、模式限制、时间戳边界、串口短读/连续帧和 MPC 弧度回归测试。当前环境无 Docker，尚未执行 ROS 容器构建及 gtest。可在已配置环境的容器中验证：

```bash
docker exec -it rv_devel_ bash
cd /ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rm_interfaces --parallel-workers 4
source install/setup.bash
colcon build --symlink-install --packages-up-to armor_detector armor_solver rune_solver rm_serial_driver rm_auto_replay --parallel-workers 4
source install/setup.bash
colcon test --packages-select rm_serial_driver armor_detector armor_solver rune_solver
colcon test-result --verbose
```

## 代码依据

- [v2_codec.hpp](src/rm_hardware_driver/rm_serial_driver/include/rm_serial_driver/v2_codec.hpp)：40/32 字节布局、大端 float32 编解码、有效性校验。
- [protocol.hpp](src/rm_hardware_driver/rm_serial_driver/include/rm_serial_driver/protocol.hpp)：目标编号与模式限制。
- [infantry_protocol.cpp](src/rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol.cpp)：步兵收发与订阅入口。
- [fixed_packet_tool.hpp](src/rm_hardware_driver/rm_serial_driver/include/rm_serial_driver/fixed_packet_tool.hpp)：短读缓存和不同帧长收发。
- [mpc_planner.cpp](src/rm_auto_aim/armor_solver/src/mpc_planner.cpp)：同一时刻的弧度规划参考。
- [serial_driver_node.cpp](src/rm_hardware_driver/rm_serial_driver/src/serial_driver_node.cpp)：时间映射与弧度 TF。
- [serial_driver_params.yaml](src/rm_bringup/config/node_params/serial_driver_params.yaml)：设备和波特率配置。
- [test_protocol_encoding.cpp](src/rm_hardware_driver/rm_serial_driver/test/test_protocol_encoding.cpp)：独立字节样例与异常回归测试。
