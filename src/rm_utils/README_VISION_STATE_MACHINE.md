# Vision State Machine 使用说明

## 概述

`VisionStateMachine` 是一个静态单例状态机类，用于管理视觉系统的所有状态和相机资源。它集成了以下功能：

1. **双相机管理**：管理6mm和8mm两个相机，根据目标距离自动切换
2. **模式切换**：管理视觉模式（自瞄红/蓝、小符、大符等）
3. **跟踪状态管理**：管理跟踪器状态（LOST, DETECTING, TRACKING, TEMP_LOST）
4. **线程安全的图像获取**：提供线程安全的图像和相机内参访问接口
5. **配置管理**：支持通过配置文件或ROS2参数配置所有参数

## 主要特性

### 1. 静态单例模式
确保整个系统中只有一个状态机实例，所有识别节点共享同一个状态机。

### 2. 相机集成
- 不再需要单独的hik-camera节点
- 相机直接集成在状态机中
- 支持双相机（6mm和8mm）自动切换

### 3. 智能相机切换
- 默认使用6mm相机
- 当目标距离超过阈值时，自动切换到8mm相机以获得更好的PnP精度
- 当目标丢失或距离过近时，自动切换回6mm相机

### 4. 线程安全
- 所有状态访问都是线程安全的
- 图像和相机内参的读写使用互斥锁保护

## 使用方法

### 1. 初始化状态机

在识别节点中初始化状态机：

```cpp
#include "rm_utils/vision_state_machine.hpp"

// 在节点构造函数中
auto& state_machine = qd::utils::VisionStateMachine::getInstance();
if (!state_machine.initialize(this, "path/to/config.yaml")) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize vision state machine!");
    return;
}
```

或者使用ROS2参数（不提供配置文件）：

```cpp
auto& state_machine = qd::utils::VisionStateMachine::getInstance();
if (!state_machine.initialize(this)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize vision state machine!");
    return;
}
```

### 2. 获取图像

在识别节点的图像回调中，从状态机获取图像：

```cpp
void imageCallback() {
    sensor_msgs::msg::Image image;
    sensor_msgs::msg::CameraInfo camera_info;
    
    if (state_machine.getCurrentImage(image, camera_info)) {
        // 使用图像和相机内参进行识别
        // ...
    }
}
```

### 3. 更新状态

#### 设置视觉模式（通常在串口节点的SetMode服务回调中）

```cpp
void setModeCallback(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
                     std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
    VisionMode mode = static_cast<VisionMode>(request->mode);
    state_machine.setVisionMode(mode);
    response->success = true;
}
```

#### 更新跟踪器状态（在armor_solver中）

```cpp
// 在tracker更新后
if (tracker_->tracker_state == Tracker::LOST) {
    state_machine.setTrackerState(qd::utils::TrackerState::LOST);
} else if (tracker_->tracker_state == Tracker::DETECTING) {
    state_machine.setTrackerState(qd::utils::TrackerState::DETECTING);
} else if (tracker_->tracker_state == Tracker::TRACKING) {
    state_machine.setTrackerState(qd::utils::TrackerState::TRACKING);
} else if (tracker_->tracker_state == Tracker::TEMP_LOST) {
    state_machine.setTrackerState(qd::utils::TrackerState::TEMP_LOST);
}
```

#### 更新目标距离（用于相机切换判断）

```cpp
// 在armor_solver中，计算目标距离后
double distance = calculateTargetDistance(target);
state_machine.updateTargetDistance(distance);
```

### 4. 检查是否应该进行识别

```cpp
if (state_machine.shouldDetect()) {
    // 进行识别
} else {
    // 跳过识别
}
```

## 配置

### 方式1：使用YAML配置文件

创建配置文件（参考 `config/vision_state_machine_config.yaml.example`）：

```yaml
camera_6mm:
  device_serial_number: "DA1564615"
  camera_info_url: "package://rm_bringup/config/camera_info.yaml"
  camera_name: "camera_6mm"
  target_frame: "camera_optical_frame"
  acquisition_frame_rate: 200.0
  exposure_time: 1800.0
  gain: 1.0
  pixel_format: "BayerRG8"
  adc_bit_depth: "ADCBitDepth_8"
  switch_to_8mm_distance: 5.0
  switch_to_6mm_distance: 3.0

camera_8mm:
  device_serial_number: "DA1041822"
  camera_info_url: "package://rm_bringup/config/camera_info_copy.yaml"
  camera_name: "camera_8mm"
  target_frame: "camera_optical_frame2"
  acquisition_frame_rate: 200.0
  exposure_time: 1800.0
  gain: 1.0
  pixel_format: "BayerRG8"
  adc_bit_depth: "ADCBitDepth_8"
  switch_to_8mm_distance: 5.0
  switch_to_6mm_distance: 3.0
```

### 方式2：使用ROS2参数

在launch文件或节点参数中设置：

```yaml
camera_6mm:
  device_serial_number: "DA1564615"
  camera_info_url: "package://rm_bringup/config/camera_info.yaml"
  # ... 其他参数
```

## 状态说明

### VisionMode（视觉模式）
- `AUTO_AIM_RED`: 自瞄红
- `AUTO_AIM_BLUE`: 自瞄蓝
- `SMALL_RUNE_RED`: 小符红
- `SMALL_RUNE_BLUE`: 小符蓝
- `BIG_RUNE_RED`: 大符红
- `BIG_RUNE_BLUE`: 大符蓝

### TrackerState（跟踪器状态）
- `LOST`: 完全丢失目标
- `DETECTING`: 短暂识别到目标，需要更多帧确认
- `TRACKING`: 正常跟踪目标
- `TEMP_LOST`: 短暂丢失目标，通过预测继续跟踪

### CameraType（相机类型）
- `CAMERA_6MM`: 6mm相机
- `CAMERA_8MM`: 8mm相机

## 相机切换逻辑

1. **默认状态**：使用6mm相机
2. **切换到8mm**：当目标距离 > `switch_to_8mm_distance` 时
3. **切换回6mm**：
   - 当目标距离 < `switch_to_6mm_distance` 时
   - 当跟踪状态变为 `LOST` 时

## 注意事项

1. **单例模式**：确保整个系统中只有一个状态机实例
2. **线程安全**：所有状态访问都是线程安全的，可以在多线程环境中使用
3. **相机SDK依赖**：需要hik-camera包的SDK，确保SDK路径正确配置
4. **初始化顺序**：状态机必须在识别节点启动时初始化
5. **图像获取**：`getCurrentImage()` 返回的是当前激活相机的图像

## 集成到现有代码

### armor_detector节点

1. 移除hik-camera节点的订阅
2. 在节点初始化时初始化状态机
3. 从状态机获取图像和相机内参
4. 在SetMode回调中更新视觉模式

### armor_solver节点

1. 在tracker更新后同步跟踪状态到状态机
2. 更新目标距离到状态机
3. 从状态机获取相机内参（如果需要）

## 故障排除

1. **相机初始化失败**：检查相机序列号是否正确，相机是否连接
2. **图像获取失败**：确保状态机已正确初始化，检查相机是否正常运行
3. **相机切换不工作**：检查距离阈值配置，确保`updateTargetDistance()`被正确调用


