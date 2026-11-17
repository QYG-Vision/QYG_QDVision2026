# rm_serial_driver

FYT视觉24赛季串口通信模块

## QD26 步兵协议

收发均为 **24 字节定长单帧**，921600 bit/s，8N1，无流控；帧头 `0xFF`、帧尾 `0x0D`。
每帧包含一次完整指令或反馈，没有分片类型、组序号与多帧组装。

角度为 float32 rad，目标自转速度为 float32 rad/s，距离为 float32 m，弹速为 float32 m/s；
均按 IEEE 754 大端直接编码，不做定点缩放。pitch 向下为正，yaw 向左为正。
时间戳保留 uint32 ms，标志位与模式保留 uint8。偏移 22（第 23 字节）的预留校验字节填 0，当前无 CRC。

完整字节表、异常组处理与迁移说明见
[infantry-jeson 串口协议](../../../docs/infantry-jeson串口协议_电控对接.md)。电控固件需同步升级。

## qd::SerialDriverNode

串口驱动节点

### 发布话题

*  `serial/receive` (`rm_interfaces/msg/SerialReceiveData`) - 下位机发送到上位机的数据
*  `tf` (`geometry_msgs/msg/TransformStamped`) - 云台的tf变换
  
### Subscribed Topics

* `armor_solver/cmd_gimbal` (`rm_interfaces/msg/GimbalCmd`) - 自瞄云台控制信息
* `rune_solver/cmd_gimbal` (`rm_interfaces/msg/GimbalCmd`) - 能量机关云台控制信息（infantry/sentry）

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

* `pitch` (double, default: 0.0) - 固定的 pitch，rad
* `yaw` (double, default: 0.0) - 固定的 yaw，rad
* `vision_mode` (int, default: 0) - 视觉模式 
