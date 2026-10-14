# rm_serial_driver

FYT视觉24赛季串口通信模块

## qd::SerialDriverNode

串口驱动节点

### 发布话题

*  `serial/receive` (`rm_interfaces/msg/SerialReceiveData`) - 下位机发送到上位机的数据
*  `serial/yaw_data_age_ms` (`std_msgs/msg/Float64`) - QD 顶板 yaw 的数据年龄，单位 ms；NaN 为旧固件，127 为缺失或超限
*  `tf` (`geometry_msgs/msg/TransformStamped`) - 云台的tf变换
  
### Subscribed Topics

* `cmd_gimbal` (`rm_interfaces/msg/GimbalCmd`) - 云台控制信息
* `cmd_chassis` (`rm_interfaces/msg/ChassisCmd`) - 底盘控制信息

### 参数

* `target_frame` (string, default: "odom") - 下位机欧拉角的相对坐标系
* `timestamp_offset` (double, default: 0.0) - tf数据的时间戳补偿
* `port_name` (string, default: "/dev/ttyUART") - 串口设备对应的文件名
* `protocol` (string, default: "infantry") - 协议类型
* `enable_data_print` (bool, default: false) - 是否打印串口读出的原始数据

## QD yaw 数据年龄扩展（hero/air）

16 字节反馈帧的第 14 字节原为预留校验位，当前收发端未启用校验。
新固件用最高位标记数据年龄扩展，低 7 位表示顶板接收 CAN yaw 到组包的毫秒年龄。
`0x80`～`0xFE` 对应 0～126 ms，`0xFF` 表示尚无有效 yaw 或年龄超限；
旧固件的 `0x00` 继续使用原有时间处理。`mcu_timestamp` 保持为顶板组包时间。

TF 时间为同步发包时间加 `timestamp_offset`，再减去有效 yaw 年龄。
roll/pitch 使用反馈历史插值到 yaw 时刻；重复 yaw 接收时刻、无效年龄或历史不足时暂不广播 TF。
`serial/receive` 保持原始角度及同步发包时间，因此其 header 时间不一定等于 TF 时间。
本扩展未测定传感器内部采样、下板处理、CAN 传输和相机曝光延迟。

## qd::VirtualSerial

仿真串口驱动节点

### 发布话题

*  `serial/receive` (`rm_interfaces/msg/SerialReceiveData`) - 下位机发送到上位机的数据（固定数据）
*  `tf` (`geometry_msgs/msg/TransformStamped`) - 云台的tf变换（固定数据）
  
### 参数

* `pitch` (double, default: 0.0) - 固定的pitch角度 
* `yaw` (double, default: 0.0) - 固定的yaw角度 
* `vision_mode` (int, default: 0) - 视觉模式 
