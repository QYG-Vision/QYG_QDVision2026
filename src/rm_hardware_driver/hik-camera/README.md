# hik_camera 海康相机驱动包

## hik_camera::HikCameraNode
相机驱动节点
## 使用方法
该包`Params`使用`hik_camera/config/`
```
ros2 launch hik_camera_first hik_camera.launch.py
```
## 发布话题
- `image_raw` (`sensor_msgs/msg/Image`) - 相机采集到的图像
- `camera_info` (`sensor_msgs/msg/CameraInfo`) - 相机内参
## 参数

- `exposure_time`（`double`，default：5000）：曝光时间
- `gain`（`double`，default：0）：增益
- `camera_info_url`（`string`，default：package://hik_camera/config/camera_info.yaml）:相机内参文件路径
- `camera_name`（`string`，default："hik_camera"）：相机名称
- `target_frame`（`string`，default："camera_optical_frame"）：发布话题的`header.frame_id`
- `use_DeviceSerialNumber`（`sting`，default：false）：是否根据相机序列号启动相机
- `DeviceSerialNumber`（`string`，default："DA1564615"）：要启动的相机的相机序列号
