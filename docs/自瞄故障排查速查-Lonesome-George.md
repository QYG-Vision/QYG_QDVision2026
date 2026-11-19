# 自瞄故障排查速查 - Lonesome George

更新日期：2026-11-19。适用当前仓库、ROS 2 Humble、Jetson GPU 容器，默认无命名空间。

本文按现象定位问题，不是另一套上车流程。首次接硬件按 [自瞄上车验收流程](自瞄上车验收流程-Lonesome-George.md)执行；GPU 环境和 engine 问题看 [部署与软件验收](Jetson部署与软件验收-Lonesome-George.md)，本机版本见 [环境总览](Jetson当前环境总览-Lonesome-George.md)。排查真实电控前先禁用自动跟随和发射，不要向云台话题随意发布测试指令。

## 1. 先确认环境和实际配置

宿主机用 `rv` 或 `docker exec -it rv_devel_ bash` 进入容器。每个新容器终端执行：

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
source /opt/openvino/setupvars.sh
source install/setup.bash
export ROS_DOMAIN_ID=0
export RM_VISION_PROFILE=jetson
export RM_VISION_INFERENCE_BACKEND=tensorrt
```

先启动主程序，再在另一个同环境终端查询；`docker compose up -d` 只启动开发容器，不会自动启动自瞄。

```bash
ros2 node list
ros2 param get /armor_detector yolo.backend
ros2 param get /armor_detector yolo.model
```

| 容易弄错的地方 | 当前规则 |
| --- | --- |
| 改了通用开关却没生效 | Jetson 的视频、虚拟串口和桥接开关以 `config/jetson/launch_params.yaml` 覆盖项为准 |
| GPU 构建/运行 | 三文件 Compose；构建显式 `ENABLE_TENSORRT=ON bash scripts/jetson/build.sh` |
| 换了代码但表现没变 | 停旧程序，重新构建、source、重启；Git 提交不会更新运行进程 |
| 修改配置或内参 | 重建 `rm_bringup`、source、重启；在线参数通常不会自动写回文件 |
| 查不到节点或话题 | 先查程序是否退出、终端是否在同一 `ROS_DOMAIN_ID`；命名空间非空时调整下文路径 |

## 2. 没有图像、颜色不对或帧率低

```bash
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once --qos-reliability best_effort
ros2 param get /camera_driver camera_6mm.acquisition_frame_rate
```

- 先确认使用话题相机模式：`video_player: false`、`use_state_machine_camera: false`、`replay: false`。视频和状态机模式没有 `/camera_driver`；状态机模式通常没有 `/image_raw`。
- 查相机 USB 连接、供电、线缆和 SDK 日志；用 `lsusb -t` 查看协商链路速率，不能只凭输出帧率判断 USB2/USB3。
- 单相机可关闭序列号选择；多相机必须配置真实序列号。确保没有独立预览和主程序抢同一个设备。
- Jetson 主程序相机覆盖当前限帧 **30Hz**。检查 `config/jetson/camera_driver_params.yaml`，不要只改通用相机文件中的帧率。
- 颜色异常先查实际像素格式与 Bayer 排列；模糊先查对焦、运动拖影和曝光。镜头按工作距离调好后固定，不要套用“焦距/光圈拧到最大”。改变光学设置或工作分辨率后重新核对标定。
- `/camera_info` 必须对应实际尺寸和镜头；有消息不代表内参正确。

独立话题相机节点只支持当前相机的曝光、增益在线调整，使用浮点数：

```bash
ros2 param set /camera_driver camera_6mm.exposure_time 1000.0
ros2 param set /camera_driver camera_6mm.gain 16.9
```

其他采集参数改 YAML 后重启；软件接受参数也不保证设备接受该数值，检查 SDK 日志与画面。

## 3. Foxglove 连不上或看不到结果图

`config/jetson/launch_params.yaml` 中 `foxglove_bridge: false` 时，在已准备好环境的容器终端手动运行：

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

若开关为 `true`，主程序自动启动桥接，不要再运行第二个。客户端连接本机 `ws://localhost:8765`，外部电脑用 `ws://车载电脑IP:8765`，并检查网络和端口访问。

```bash
ros2 node list
ros2 param get /foxglove_bridge port
ros2 topic info /armor_solver/result_img/compressed --verbose
```

- 看图选择 **Image** 面板：原图 `/image_raw`，结果图 `/armor_solver/result_img/compressed`；3D 面板用于 TF 和 Marker。
- 检测器的 Jetson `debug` 当前为 `false`。需要结果图时同时核对检测器、解算器的 `debug`，修改配置后重启。
- 没有图像订阅者时，当前解算器会跳过结果绘图；没有面板订阅时，Foxglove 的 `Hz` 显示 `--` 不代表程序没运行。
- 桥接没有 `/foxglove_bridge/heartbeat`。传统检测的二值图、数字图不适用于当前 YOLO 路线。

## 4. 有图但不识别、不跟踪或目标位置不对

```bash
ros2 topic echo /serial/receive --once --qos-reliability best_effort
ros2 topic echo /armor_solver/armors --once --qos-reliability best_effort
ros2 topic echo /armor_solver/target --once --qos-reliability best_effort
ros2 topic echo /armor_solver/cmd_gimbal --once --qos-reliability best_effort
```

- 模式 `0` 识别红板，`1` 识别蓝板。虚拟模式用 `ros2 param set /virtual_serial vision_mode 0` 切换；真实模式核对电控下发的 `/serial/receive.mode`。不要只改检测器颜色，串口会持续同步模式。
- `/armor_detector/armors` 是调试输出，受检测器 `debug` 控制；优先看 `/armor_solver/armors` 和 `/armor_solver/target`。
- 看日志是否提示 `camera_info is not ready`、TF 外推或持续变换失败。核对图像时间、姿态时间和坐标系，不要直接改成当前时间掩盖时差。
- 距离不对先查工作分辨率、内参、装甲板尺寸；目标随云台转动漂移先查姿态方向、时间对齐和外参。不要先用 `angle_offset` 大幅补偿。

## 5. 真实串口没有数据或方向不对

先停主程序，将 Jetson 覆盖的 `virtual_serial` 设为 `false`；核对 `config/node_params/serial_driver_params.yaml`。当前端口为 `/dev/ttyUSB0`，波特率为 `921600`，必须与实际设备和电控一致。

```bash
ls -l /dev/ttyUSB0
lsusb
ros2 topic hz /serial/receive
ros2 topic echo /serial/receive --qos-reliability best_effort
```

- `/dev/rm_usb0` 只有配置了对应 udev 规则才存在，不是必备路径；使用别名时同步修改 `port_name`。
- 无话题输出可能是端口、权限、波特率、帧格式或解析错误，不能直接断定电控没发送。必要时在串口 YAML 中启用 `enable_data_print` 后重启，检查原始数据。
- 当前主启动在 `rune: false` 时强制用 `hero` 实现。按 [电控协议](infantry-jeson串口协议_电控对接.md)核对 **24 字节单帧、float32 大端序、角度 rad、弹速 m/s**。
- 禁用跟随，手动转云台：yaw 向左为正，pitch 向下为正。确认反馈与控制目标采用一致参考系；有数据不等于单位和协议正确。

## 6. 断流后仍有指令、相机掉线或程序退出

- `/armor_solver/cmd_gimbal` 是定时发布，目标失效后仍有消息是正常的。看内容是否为 `id="0"`、`distance=-1`、`fire_advice=false`，以及目标 `tracking=false`，不要只看频率。
- `tracker.input_timeout` 当前为 0.5 秒，从最后一次有效处理结果开始计时；超时后清空旧跟踪。恢复输入后需重新确认目标。
- 当前独立相机节点连续取帧失败超过 5 次会调用 ROS shutdown，组合主程序可能整体退出；没有保证 USB 拔插后自动恢复。先恢复设备、确认旧程序退出，再重新启动。
- 程序已退出就不会再发送无效指令，电控必须有自己的视觉失联/串口超时保护。
- `scripts/jetson/check_stream_recovery.py` 只用于视频加虚拟电控的软件测试，不在真实硬件联调时运行。

## 7. GPU、延迟和跟随异常

- `yolo.backend` 应为 `tensorrt`，`yolo.model` 应指向本机 FP16 engine；不是把 OpenVINO 的 `device` 改成 `GPU`。
- engine 缺失、换机或 TensorRT/CUDA 环境变化，按部署与软件验收文档重新生成。CUDA Graph 是否启用看日志 `CUDA graph enabled`；捕获失败回退普通 TensorRT，不等于回退 CPU。
- CPU 对照：停主程序，在准备好环境的 GPU 容器终端设置 `export RM_VISION_INFERENCE_BACKEND=openvino` 后重启主 launch。恢复 GPU 时设置为 `tensorrt`；不必为一次对照重编译或重建容器。
- `/image_raw` 是采集率，`/armor_solver/target` 是结果输出率，云台指令率是定时器频率，三者不能混为模型 FPS。心跳也不是通用吞吐指标。
- 记录原图时间到结果的延迟、长间隔和 CPU/内存负载；多个图像订阅器会增加开销，比较时保持观测条件一致。性能脚本见部署与软件验收文档。
- 当前 `solver.predict2send_delay=0.11` 需要在真实 GPU 链路上重新测量；先查成像、标定、姿态和时间，再调预测与弹道，每次只改一类参数。

## 8. 保留哪些证据

保存提交号、启动环境、实际参数、完整报错、异常时间和录包。按 [验收流程第 8 节](自瞄上车验收流程-Lonesome-George.md#8-记录与收尾)记录原图、内参、姿态、TF、目标与指令。

当前仓库不提供旧笔记中的 `zero_order_gimbal_test` launch、`set_gimbal_target` 可执行程序或 `calculateError` 工具，不再把它们作为排查步骤。需要固定角度控制测试时，先与电控另行确认工具、单位和安全条件。
