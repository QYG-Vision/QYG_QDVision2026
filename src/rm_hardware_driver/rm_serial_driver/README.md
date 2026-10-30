# rm_serial_driver

FYT视觉24赛季串口通信模块

## QD26 V2 协议

UART 默认 921600 bit/s，8N1；视觉发送 32 字节，接收 24 字节，帧头 `0xFE`，帧尾 `0x0D`。
所有多字节字段显式大端编解码。新增四个规划导数、两个实际角速度使用 float32，
分别直接表示 °/s、°/s²；Pitch 向下为正。`target_v_yaw` 仍是 rad/s ×100。

当前未接入 MPC，求解器明确保持 `reference_valid=false`，仅发送无效参考帧：
`target_state=0`、开火建议 0、距离 −1 m、角度与运动字段为 0。
能量机关入口和模式 2～5 均禁止有效参考。V1 不兼容，不进行协商或回退。

反馈通过 `serial/receive` 发布实际角速度；非法模式及 NaN/Inf 整帧拒绝。
MCU 毫秒时间戳映射支持回绕，0 也是合法 tick，重复快照保持原时间。
完整协议见 [SERIAL_PROTOCOL.md](../../../SERIAL_PROTOCOL.md)。

## qd::SerialDriverNode

串口驱动节点

### 发布话题

*  `serial/receive` (`rm_interfaces/msg/SerialReceiveData`) - 下位机发送到上位机的数据
*  `tf` (`geometry_msgs/msg/TransformStamped`) - 云台的tf变换
  
### Subscribed Topics

* `armor_solver/cmd_gimbal` (`rm_interfaces/msg/GimbalCmd`) - 装甲板参考
* `rune_solver/cmd_gimbal` (`rm_interfaces/msg/GimbalCmd`) - 步兵/哨兵入口只发送无效参考

### 参数

* `target_frame` (string, default: "odom") - 下位机欧拉角的相对坐标系
* `timestamp_offset` (double, default: 0.0) - tf数据的时间戳补偿
* `port_name` (string, default: "/dev/ttyUART") - 串口设备对应的文件名
* `protocol` (string, default: "infantry") - 协议类型
* `enable_data_print` (bool, default: false) - 是否打印串口读出的原始数据

## qd::VirtualSerial

仿真串口驱动节点

### 发布话题

*  `serial/receive` (`rm_interfaces/msg/SerialReceiveData`) - 下位机发送到上位机的数据（固定数据）
*  `tf` (`geometry_msgs/msg/TransformStamped`) - 云台的tf变换（固定数据）
  
### 参数

* `pitch` (double, default: 0.0) - 固定的pitch角度 
* `yaw` (double, default: 0.0) - 固定的yaw角度 
* `vision_mode` (int, default: 0) - 视觉模式 
