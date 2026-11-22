# 自瞄上车验收流程 - Lonesome George

更新日期：2026-11-19。适用当前仓库、Orin Nano Super、ROS 2 Humble、GPU 容器 `rv_devel_`，先验收单相机装甲板自瞄。

本文回答“先做什么、怎样算通过”；异常时看 [自瞄故障排查速查](自瞄故障排查速查-Lonesome-George.md)。版本与配置见 [环境总览](Jetson当前环境总览-Lonesome-George.md)，环境和 engine 生成按 [部署与软件验收](Jetson部署与软件验收-Lonesome-George.md)。

**当前状态：** TensorRT FP16/CUDA Graph 视频与虚拟电控链路已验证；接相机前的相关三包构建成功，39 个测试用例通过。真实相机、串口和云台闭环尚未验收。最新相机修复只在独立目录构建验证，Git 提交不会自动更新正式 `install/`。

**顺序：更新程序 → 相机预览 → 内参 → 虚拟电控识别 → 真实通信与外参 → 异常退出 → 无发射跟随 → 后续闭环。前一步通过再继续。**

## 1. 更新程序和环境

宿主机用 `rv` 进入容器；未配置别名时用 `docker exec -it rv_devel_ bash`，需要 Docker 访问权限。以下命令除特别注明外，均在容器内执行。

先停止旧主程序和独立相机预览，再更新正式构建：

```bash
cd /ros_ws
ENABLE_TENSORRT=ON bash scripts/jetson/build.sh
```

此后每个新容器终端先执行：

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
source /opt/openvino/setupvars.sh
source install/setup.bash
export ROS_DOMAIN_ID=0
export RM_VISION_PROFILE=jetson
export RM_VISION_INFERENCE_BACKEND=tensorrt
```

确认 `/ros_ws/record-test/models/0526-fp16.engine` 存在。首次部署或更换设备、CUDA、TensorRT 后，按部署与软件验收文档重新生成，不复制其他设备的 engine。GPU 容器应使用三个 Compose 文件，不能漏掉 `docker-compose.jetson.gpu.yaml`。

配置主要在以下位置；Jetson 覆盖项优先于通用配置：

| 要改什么 | `src/rm_bringup/config/` 下的文件 |
| --- | --- |
| 视频/相机、虚拟/真实电控、Foxglove 开关 | `jetson/launch_params.yaml` |
| 相机序列号、像素格式、曝光和增益 | `node_params/camera_driver_params.yaml` |
| 主程序相机限帧，目前为 30Hz | `jetson/camera_driver_params.yaml` |
| 检测调试开关、CPU 参数 / GPU engine | `jetson/armor_detector_params.yaml` / `jetson/armor_detector_tensorrt_params.yaml` |
| 串口设备与波特率 / 跟踪与预测参数 | `node_params/serial_driver_params.yaml` / `node_params/armor_solver_params.yaml` |
| 安装外参 `gimbal2camera.xyz/rpy` | 通用 `launch_params.yaml` |

**后续修改上述配置或内参后：** 停程序，执行 `colcon build --symlink-install --packages-select rm_bringup`，重新 `source install/setup.bash` 并启动。修改 C++ 则重新执行完整 GPU 构建。

## 2. 单独预览相机

先固定镜头、对焦和工作分辨率。核对通用相机参数：单相机可用 `camera_6mm.enable_DeviceSerialNumber: false`；多相机必须填写真实序列号并启用匹配。曝光、增益、像素格式以实际设备支持情况为准。

生成独立预览/标定参数，限帧 30Hz；这份临时文件不会自动加载 Jetson 覆盖：

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
v = yaml.safe_load(Path('src/rm_bringup/config/node_params/camera_driver_params.yaml').read_text())
v['camera_6mm']['enable_frame_rate'] = True
v['camera_6mm']['acquisition_frame_rate'] = 30.0
v['use_sensor_data_qos'] = False
Path('/tmp/intrinsics_camera_params.yaml').write_text(
    yaml.safe_dump({'/**': {'ros__parameters': v}}))
PY
ros2 run hik_camera hik_camera_node --ros-args \
  -r __node:=camera_driver --params-file /tmp/intrinsics_camera_params.yaml
```

另开已准备好环境的终端：

```bash
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once --qos-reliability best_effort
```

按第 4 节连接 Foxglove，用 Image 面板看 `/image_raw`。独立相机节点可在线调参：

```bash
ros2 param set /camera_driver camera_6mm.exposure_time 1000.0
ros2 param set /camera_driver camera_6mm.gain 16.9
```

数值用浮点数，是否被设备接受还要看 SDK 日志与画面；其余采集参数改 YAML 后重启。在线调整不会自动写回 YAML。

**通过：** 连续出图，颜色正确、画面清晰，尺寸与预期一致，曝光合适、帧率稳定。不要同时运行独立预览和主程序占用同一相机。

## 3. 标定内参

相机安装和工作分辨率确定后标定；之后改变对焦、裁切或缩放，需要重新核对内参。推荐棋盘格 **9×6 内角点、方格边长 0.03m**，打印后量尺寸，不要把方格数当内角点数。

标定需要图形窗口。若容器未启用 GUI，先停程序，在宿主机取消 `docker-compose.yaml` 中 `environment`、`DISPLAY`、`QT_X11_NO_MITSHM` 和 `/tmp/.X11-unix` 挂载的注释，再于宿主机仓库目录执行：

```bash
xhost +si:localuser:root
sudo docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml \
  -f docker-compose.jetson.gpu.yaml up -d --force-recreate rv_devel
```

重新进入容器、准备环境并运行第 2 节预览；临时参数文件丢失时重新生成。容器内用 `ros2 pkg prefix camera_calibration` 检查工具，缺少时执行 `apt update` 和 `apt install -y ros-humble-camera-calibration`。重建容器后需重新检查临时安装的软件。

另开已准备好环境的容器终端：

```bash
ros2 run camera_calibration cameracalibrator \
  --size 9x6 --square 0.03 --camera_name camera_6mm \
  --no-service-check -k 3 --ros-args -r image:=/image_raw
```

移动、倾斜标定板，覆盖中心、四角和不同距离；停稳后采清晰图像。覆盖充分后点 **CALIBRATE → SAVE**，使用导出文件，不依赖写入相机服务：

```bash
out="calibration_results/intrinsics_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$out"
tar -xzf /tmp/calibrationdata.tar.gz -C "$out"
cp src/rm_bringup/config/camera_info.yaml "$out/camera_info.before.yaml"
cp "$out/ost.yaml" src/rm_bringup/config/camera_info.yaml
```

按第 1 节重建 `rm_bringup`，重启相机，核对 `/camera_info` 的尺寸和参数。使用其他标定板时必须同步修改类型、数量和实测间距；双相机分别标定，远摄文件为 `camera_info_copy.yaml`，不要覆盖近摄结果。

**通过：** 标定覆盖充分、重投影误差较小，并且后续实际测距合理。低重投影误差不能单独证明整套自瞄准确。

## 4. 相机加虚拟电控，验证识别

停止独立预览。在 `config/jetson/launch_params.yaml` 中修改以下字段，保留其他配置：

```yaml
video_player: false
virtual_serial: true
use_state_machine_camera: false
replay: false
rune: false
record: false
auto_record: false
foxglove_bridge: false
```

保持 `hero_solver`、`navigation`、`enable_dynamic_camera_tf`、`enable_pitch_calibration` 为 `false`。需要结果图时，将 Jetson 检测器配置和通用解算器配置中的 `debug` 设为 `true`。按第 1 节重建配置并启动：

```bash
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

另开已准备好环境的终端检查并选择目标颜色：

```bash
ros2 param get /armor_detector yolo.backend
ros2 param get /armor_detector yolo.model
ros2 param set /virtual_serial vision_mode 0
ros2 topic echo /armor_solver/target --once --qos-reliability best_effort
ros2 topic echo /armor_solver/cmd_gimbal --once --qos-reliability best_effort
```

后端应为 `tensorrt`，模型应为本机 FP16 engine。模式 `0` 识别红板，`1` 识别蓝板；不要只改单个检测器颜色，虚拟电控会持续同步模式。虚拟弹速固定 25m/s，不能用于验证真实弹速或运动姿态。

**Foxglove：** 另开已准备好环境的终端运行下列桥接；若启动开关已设为 `true`，不要再手动启动：

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

客户端连接 `ws://localhost:8765`，外部电脑用 `ws://车载电脑IP:8765`。

| 面板 | 观察内容 |
| --- | --- |
| Image | `/image_raw` 或 `/armor_solver/result_img/compressed` |
| Raw Messages / Plot | `/serial/receive`、`/armor_solver/target`、`/armor_solver/cmd_gimbal` |
| 3D，固定坐标系 `odom` | TF 与 `/armor_solver/marker` |

**通过：** 静止、左右/上下移动、遮挡后重现都能合理识别跟踪，距离与实测接近，目标离开后退出跟踪。此阶段只证明图像与算法链路，不能证明真实云台闭环。

## 5. 真实串口与外参

**先由电控禁用自动跟随和发射。** 停程序，将 Jetson 启动配置的 `virtual_serial` 改为 `false`；串口配置按实际设备核对 `port_name` 和 `baud_rate`，当前为 `/dev/ttyUSB0`、`921600`。重建配置并重启。

当前 `rune: false` 会强制选择 `hero` 实现；与 `infantry` 一样使用 **24 字节单帧、float32 大端序、角度 rad**。按 [电控协议](infantry-jeson串口协议_电控对接.md)逐字段核对，不要只看是否有数据。

```bash
ros2 topic echo /serial/receive --qos-reliability best_effort
```

手动转云台，核对 yaw 向左为正、pitch 向下为正、角度单位 rad、弹速单位 m/s、模式 0/1 对应红/蓝自瞄，以及时间戳和数据连续性。

在通用 `config/launch_params.yaml` 的 `gimbal2camera.xyz/rpy` 填实测/标定外参，保持原字符串格式，单位为米和弧度。固定底盘和目标，手动改变云台 yaw/pitch，检查 `odom` 中目标位置基本不变、重投影贴合。注意 `camera_link` 与相机光学坐标系之间还有固定旋转。

**通过：** 姿态方向、幅值、模式、弹速和通信正确；目标位置没有明显随云台姿态漂移。未通过先查标定、坐标系和时间对齐，不要用大角度偏置掩盖问题。

## 6. 断流、退出和恢复

先在虚拟电控模式检查，接真实电控后仍禁用跟随和发射再复验：

- 遮挡或移走目标：正常退出跟踪；恢复后重新确认目标。
- 在安全条件下断开相机：仍运行的解算器在默认 `tracker.input_timeout=0.5` 秒超时后撤销旧目标，输出 `tracking=false`，云台指令为 `id="0"`、`distance=-1`、`fire_advice=false`。按最后一次有效处理结果计时，不要求拔线瞬间立即失效。
- **当前相机节点连续取帧失败超过 5 次会调用 ROS shutdown，主程序可能整体退出；没有承诺自动重连。** 重新插入后按需要重启，检查能否再次正常出图、跟踪。
- Ctrl+C 停止并再次启动，检查无卡住、段错误或相机持续被占用。程序停止后没有新消息，不能据此证明电控已停止跟随。
- 与电控确认：视觉退出、串口断开和无效目标时，电控自身超时保护能退出跟随、禁止发射。视觉的 0.5 秒超时不能替代电控保护。

**通过：** 没有持续使用旧目标，退出正常，设备恢复后可以重新启动并工作。记录恢复是否需要人工重启；软件视频断流脚本不能替代 USB 实测。

## 7. 无发射跟随与调参

与电控一起开启跟随，继续禁用发射。从小角度静止目标开始，再测不同距离、平移、遮挡和旋转目标；对比 `/armor_solver/cmd_gimbal` 与 `/serial/receive` 的 yaw/pitch，确认方向正确、收敛且无明显振荡。

GPU 提速后，当前 `solver.predict2send_delay: 0.11` 需按真实链路重新测量，不能直接当作推理耗时或凭感觉清零。先解决成像、标定、姿态和时间问题，再调滤波、预测与弹道；每次只改一类参数。

双相机状态机模式留到单相机通过后：核对两台序列号、各自内外参，再设置 `use_state_machine_camera: true`，重新验收远近切换。此模式通常不发布 `/image_raw`。

**通过：** 指令与反馈方向一致、跟随稳定；目标丢失、退出自瞄和视觉断连时电控安全退出。后续发射验收只在规定区域、人员撤离射界后进行，并确认有效实测弹速；`fire_advice` 只是视觉建议，不能替代电控发射许可。

## 8. 记录与收尾

在话题相机模式下另开终端录包，每次使用新目录：

```bash
ros2 bag record -o "/ros_ws/rosbag/aim_$(date +%Y%m%d_%H%M%S)" \
  /image_raw /camera_info /serial/receive /tf /tf_static \
  /armor_solver/result_img/compressed /armor_solver/target \
  /armor_solver/cmd_gimbal /armor_solver/ballistic_debug
```

保存提交号、参数、内外参、测试条件、异常时间和 bag 路径。录制用 Ctrl+C 正常结束；结束测试先退出跟随/发射，再关程序。

只观察录包时先停实时主程序，再用 `ros2 bag play <录包目录>`，避免重复发布。仓库 `replay: true` 使用专用记录格式，不等于播放任意 ROS bag；状态机模式缺少原始图像时，不能仅靠结果图重跑检测。
