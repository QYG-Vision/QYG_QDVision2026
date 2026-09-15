# QYG 哨兵 Pitch 符号与 TF 坐标系说明

## 1. 结论

当前项目在 QYG 哨兵协议下采用以下约定：

- 电控回传的 `roll`、`pitch`、`yaw` 单位均为度。
- 电控 pitch 以抬头为正。
- 视觉求解器内部的 pitch 也以抬头为正。
- ROS TF 遵循右手坐标系；在前 `x`、左 `y`、上 `z` 的坐标轴定义下，正 pitch
  表现为低头，因此抬头姿态在 TF 中对应负 pitch。
- 求解器发布的 `GimbalCmd.pitch` 单位为度、抬头为正。
- QYG 串口层将 `GimbalCmd` 的 yaw、pitch 从度转换为弧度后发给电控，不改变符号。

## 2. TF、odom 和 atan2

### 2.1 TF

TF（Transform）是 ROS 中维护不同坐标系之间位置和姿态关系的系统。当前视觉链路主要涉及：

```text
odom
  └── gimbal_link
        └── camera_link
              └── camera_optical_frame
```

TF 使程序能够把相机坐标系中测得的目标位置转换到世界坐标系，也能查询云台相对于
`odom` 的当前姿态。

### 2.2 odom

`odom` 是机器人运行时使用的局部世界坐标系。项目中的目标最终保存为 `odom` 坐标系下的
三维位置 `(x, y, z)`，遵循：

```text
x：前
y：左
z：上
```

目标保存的是三维位置，而不是一个带 ROS 符号约定的“目标 pitch”。

### 2.3 atan2

求解器根据目标三维位置计算几何 pitch：

```cpp
target_pitch = atan2(z, sqrt(x * x + y * y));
```

`atan2` 根据竖直高度与水平距离计算角度，并保留正负方向。目标高于云台时 `z > 0`，
因此目标 pitch 为正，符合“抬头为正”的直觉。

例如目标比云台高 1 米：

```text
水平距离 5 m：  target_pitch = atan2(1, 5)  ≈ +11.3°
水平距离 10 m： target_pitch = atan2(1, 10) ≈  +5.7°
```

在高度不变的情况下，目标远离会使几何 pitch 逐渐接近 0。加入重力下坠补偿后，最终发射
pitch 还会受到距离、弹速和弹道模型影响。

## 3. 电控反馈到求解器的符号链路

假设云台实际抬头 10°，电控回传：

```text
frame.vpitch = +10°
```

当前正确链路为：

```text
1. QYG 串口解析
   SerialReceiveData.pitch = +10°
   单位仍为度，保持电控“抬头为正”的约定。

2. 写入 ROS TF
   tf_pitch = -10° = -0.1745 rad
   这里取一次负号，将抬头为正转换为 ROS TF 的右手旋转表示。

3. 求解器读取 TF
   current_pitch = -tf_pitch = +10°
   这里再次取负，将 ROS TF 表示恢复为求解器内部“抬头为正”的角度。
```

第三步不是直接向电控发送数据，而是为了让“当前云台角度”和“目标角度”使用相同的内部
符号约定。

## 4. 目标角度、角度误差和云台控制

假设：

```text
当前云台 pitch：+10°
目标 pitch：    +15°
```

求解器计算：

```text
pitch_diff = target_pitch - current_pitch
           = +15° - (+10°)
           = +5°
```

求解器随后：

1. 使用 yaw、pitch 误差参与目标选择和是否允许开火的判断。
2. 发布目标绝对角度 `GimbalCmd.pitch = +15°`。
3. 发布 `GimbalCmd.pitch_diff = +5°`，用于误差判断、记录和调试。
4. 不在视觉侧直接执行云台电机 PID；电控收到目标绝对角度后负责完成闭环控制。

因此视觉发给电控的是目标绝对角度，而不是把 `pitch_diff` 作为电机控制量发送。

## 5. 发给 QYG 电控的数据

装甲求解器和打符求解器发布的 `GimbalCmd.yaw`、`GimbalCmd.pitch`：

- 单位为度。
- pitch 以抬头为正。

QYG 串口层发送前执行：

```text
+15° × π / 180 = +0.2618 rad
```

最终 QYG 串口帧中的 yaw、pitch 单位是弧度。发送过程中不对 pitch 取负；如果电控的目标
角度接口同样以抬头为正，这一符号处理是正确的。

## 6. 本次代码修复

原 QYG 接收代码曾执行：

```cpp
data.pitch = -frame.vpitch;
```

公共串口节点在写入 TF 时本来就会再取一次负号，因此 QYG 解析层的额外负号会导致：

```text
电控抬头 +10° → TF +10° → 求解器内部 -10°
```

这与实际方向相反。本次修复让 QYG 回传角通过明确的映射函数进入视觉内部，其中 pitch
保持电控的抬头正方向：

```cpp
data.pitch = gimbal_angles.pitch_degrees;
```

保留的处理包括：

- 写入 ROS TF 时对 pitch 取负。
- 求解器从 TF 读取当前 pitch 后取负。
- QYG 发送时只进行度到弧度的转换，不取负。

协议帧的字段顺序、变量含义、帧大小和字节偏移均未改变。

## 7. 相关代码位置

- QYG 接收与发送：
  `src/rm_hardware_driver/rm_serial_driver/src/protocol/qyg_sentry_protocol.cpp`
- QYG 帧结构和角度映射：
  `src/rm_hardware_driver/rm_serial_driver/include/rm_serial_driver/protocol/qyg_protocol.hpp`
- ROS TF 构造：
  `src/rm_hardware_driver/rm_serial_driver/src/serial_driver_node.cpp`
- 装甲求解器当前姿态、目标角度和误差：
  `src/rm_auto_aim/armor_solver/src/armor_solver.cpp`
- 打符求解器当前姿态、目标角度和误差：
  `src/rm_rune/rune_solver/src/rune_solver.cpp`
- QYG 协议回归测试：
  `src/rm_hardware_driver/rm_serial_driver/test/test_qyg_protocol.cpp`

## 8. 建议的实车检查

缓慢抬高云台时应观察到：

```text
/serial/receive.pitch：逐渐增大（抬头为正，单位为度）
TF 中的 ROS pitch：   逐渐减小（抬头为负，内部为弧度）
求解器 current_pitch：逐渐增大（抬头为正）
```

对准一个高于云台的固定目标时，目标 pitch、当前 pitch 和误差应使用同一套“抬头为正”
约定；接近目标角度时 `pitch_diff` 应趋近于 0。
