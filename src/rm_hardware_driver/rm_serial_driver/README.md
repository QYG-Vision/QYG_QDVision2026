# rm_serial_driver

FYT视觉24赛季串口通信模块

## qd::SerialDriverNode

串口驱动节点

### 发布话题

*  `serial/receive` (`rm_interfaces/msg/SerialReceiveData`) - 下位机发送到上位机的数据
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

## qd::VirtualSerial

仿真串口驱动节点

### 发布话题

*  `serial/receive` (`rm_interfaces/msg/SerialReceiveData`) - 下位机发送到上位机的数据（固定数据）
*  `tf` (`geometry_msgs/msg/TransformStamped`) - 云台的tf变换（固定数据）
  
### 参数

* `pitch` (double, default: 0.0) - 固定的pitch角度 
* `yaw` (double, default: 0.0) - 固定的yaw角度 
* `vision_mode` (int, default: 0) - 视觉模式 

## qd::ZeroOrderGimbalTestNode

零阶云台响应测试节点。它收到第一帧 `serial/receive` 后保持该 yaw/pitch；向
`zero_order_gimbal/target` 发布一次 `rm_interfaces/msg/GimbalCmd` 后，节点以配置频率重复
发布该消息的 yaw/pitch 到 `armor_solver/cmd_gimbal`。节点始终设置 `fire_advice=false`。

推荐流程（不启动主 `bringup_SingleProcess.launch.py`）：

```bash
ros2 launch rm_bringup zero_order_gimbal_test.launch.py
```

在另一个终端设置世界坐标系下的绝对目标角度（单位：度）：

```bash
ros2 run rm_serial_driver set_gimbal_target 5.0 0.0
```
