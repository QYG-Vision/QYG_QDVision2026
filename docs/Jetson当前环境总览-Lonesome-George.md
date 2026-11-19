# Jetson 当前环境总览 - Lonesome George

核对日期：2026-11-19。来源为仓库配置、本机只读查询及已有验收记录。本页是核心构建/运行环境总览，不是完整逐包清单或整机备份；更换设备后需重新核对。

**看哪里：** 操作按 [部署与软件验收](Jetson部署与软件验收-Lonesome-George.md)，改动和测试数据看 [验证记录](Jetson改动与验证记录-Lonesome-George.md)，接硬件按 [上车验收](自瞄上车验收流程-Lonesome-George.md)，异常查 [排障速查](自瞄故障排查速查-Lonesome-George.md)。

## 1. 宿主机

| 项目 | 当前值 |
| --- | --- |
| 设备 | NVIDIA Jetson Orin Nano Super，8GB |
| 系统 / 架构 | Ubuntu 22.04.5 LTS / `aarch64` |
| JetPack / L4T | `6.2.3+b81` / `36.5.2-20260716114719` |
| GPU 架构 | SM 8.7，Orin CUDA 编译默认 `87`，不适用于老 Jetson Nano |
| Docker / Compose | `29.1.3` / `2.40.3` |
| 仓库位置 | `/home/rm/rm/QYG_QDVision2026` |
| 分支 / 环境核对对应源码提交 | `infantry-jetson-Lonesome-George` / `a9bac47`；后续提交以 Git 历史为准 |
| 内存交换 | 6 个约 633.9MiB 的 zram，合计约 3.7GiB，压缩算法 `lzo-rle` |
| swappiness | `100`；本机持久配置 `/etc/sysctl.d/99-zram-swappiness.conf` |

内存交换设置只是本机当前状态，不是所有 Jetson 的通用推荐；内存占用随进程变化，不作为固定基线。

## 2. 容器与依赖

| 项目 | 当前值 |
| --- | --- |
| 容器名 / Compose 服务名 | `rv_devel_` / `rv_devel` |
| 当前 GPU 镜像 / CPU 基础镜像 | `qyg-vision:jetson-jp6-gpu` / `qyg-vision:jetson-jp6` |
| 工作区 / 网络 / runtime | `/ros_ws` 挂载当前宿主机仓库 / host / nvidia |
| 共享内存 / 设备 | 1GiB / privileged，并挂载 `/dev` |
| ROS | ROS 2 Humble，`ros-base`；路径 `/opt/ros/humble` |
| CUDA Toolkit | `12.6.68`；路径 `/usr/local/cuda-12.6` |
| TensorRT | 系统包 `10.3.0.30-1+cuda12.5`，Python 模块 `10.3.0` |
| OpenVINO | `2025.3.0-19807`；路径 `/opt/openvino`，CPU 对照和编译仍需要 |
| OpenCV / Python | `4.5.4` / `3.10.12` |
| Foxglove bridge | `3.6.3`，已安装，默认不随主程序启动 |
| 相机标定工具 | 本次查询未安装 `ros-humble-camera-calibration`，需要时在容器内安装并启用 GUI |
| 海康 SDK | 仓库配套 AArch64 库与头文件，选择与来源见 [SDK 说明](../src/rm_utils/hikSDK/README.md) |

系统 TensorRT 包名带 `cuda12.5` 后缀不是这里写错；现有包与 CUDA 12.6 配套运行已验证。不要随意单独升级其中一个组件。

依赖定义以根目录 [Dockerfile](../Dockerfile)、[GPU Dockerfile](../Dockerfile.jetson-gpu)和 [模型工具依赖](../scripts/jetson/requirements-model-tools.txt)为准。模型工具主要为 `openvino2onnx 1.0.0`、`onnx 1.17.0`、`numpy >=1.21,<2`、`protobuf 4.25.9`；工具虚拟环境位于 `record-test/model-tools/`。

## 3. 默认运行方式

本次只读查询确认：容器和主程序运行中，节点使用 `tensorrt` 与本机 `0526-fp16.engine`，输入为视频和虚拟电控；没有据此重新做性能或硬件验收。

| 项目 | 默认配置 |
| --- | --- |
| Compose 顺序 | `docker-compose.yaml` → `docker-compose.jetson.yaml` → `docker-compose.jetson.gpu.yaml` |
| 主启动入口 | `ros2 launch rm_bringup bringup_SingleProcess.launch.py` |
| 构建入口 | `ENABLE_TENSORRT=ON bash scripts/jetson/build.sh`；包级并发 1、包内 2，默认不编测试 |
| 环境变量 | `RM_VISION_PROFILE=jetson`、`RM_VISION_INFERENCE_BACKEND=tensorrt`、通信域 `ROS_DOMAIN_ID=0` |
| 日志 / Graph | `RM_LOG_FILE_LEVEL=INFO`；默认尝试 CUDA Graph，是否成功以日志为准 |
| 视频 / 电控 | `/ros_ws/test.mp4`，30Hz 循环 / 虚拟姿态与固定 25m/s 弹速 |
| 模型 | 原始 `0526.xml/.bin` 转 ONNX，engine 为 `/ros_ws/record-test/models/0526-fp16.engine` |
| 输入/输出 | FP32 `[1,3,640,640]` / `[1,25200,22]`，内部允许 FP16；沿用 RGB 与原后处理 |
| CPU 工作 | 解码、缩放、解析、PnP、跟踪和弹道；不是全部计算都搬到 GPU |

容器启动命令为 `bash`，不会自动启动自瞄。`source` 不等于启动；源码提交、独立测试通过也不等于正式 `install/` 已更新，接硬件前需要正式重编并重启。

## 4. 配置去哪改

下表路径均相对 `src/rm_bringup/config/`。一般按通用参数 → Jetson 覆盖 → TensorRT 附加参数/launch 显式项加载，查询运行节点才能确认实际生效值。

| 目的 | 文件 / 当前值 |
| --- | --- |
| 输入与功能开关 | `jetson/launch_params.yaml`：视频、虚拟电控为 true；状态机相机、回放、打符、录像和桥接为 false |
| CPU 检测基线 | `jetson/armor_detector_params.yaml`：YOLO、OpenVINO CPU、debug false、图像队列 1、OpenCV 线程 1 |
| GPU 模型覆盖 | `jetson/armor_detector_tensorrt_params.yaml`：TensorRT 与 FP16 engine |
| 相机参数 / 主程序限帧 | `node_params/camera_driver_params.yaml` / `jetson/camera_driver_params.yaml`，近远相机均限 30Hz |
| 相机内参 | `camera_info.yaml`、`camera_info_copy.yaml`；保留值不能视为新相机已标定 |
| 安装外参 | 通用 `launch_params.yaml` 的 `gimbal2camera.xyz/rpy`，米/弧度；需要实测核对 |
| 串口 | `node_params/serial_driver_params.yaml`：`/dev/ttyUSB0`、921600；主启动 `rune=false` 时强制用 hero 实现 |
| 跟踪与预测 | `node_params/armor_solver_params.yaml`：有效输入超时 0.5s，`predict2send_delay=0.11` 待真实链路测量 |
| 端口与方向 | Foxglove 8765；yaw 左正、pitch 下正，协议见 [电控对接文档](infantry-jeson串口协议_电控对接.md) |

独立相机预览不自动加载 Jetson 覆盖；按上车文档生成专用参数。内外参和帧率修改后需重新核对实际出图与测距。

## 5. 哪些东西不会随 Git 保存

| 本机内容 | 重装或换机时注意 |
| --- | --- |
| `~/.bashrc` 的 `rv` | 本机为 `alias rv='sg docker -c "docker exec -it rv_devel_ bash"'`；需要 Docker 组授权，别名不随仓库恢复 |
| Docker runtime、用户组、代理和 Git 身份 | 属于宿主机/用户配置；仓库只保存部分操作说明，不是完整自动恢复脚本 |
| zram、swappiness、udev、GUI 授权 | 需按目标机器核对；GUI 挂载当前仍在基础 Compose 中注释 |
| engine、报告、构建、日志和录包 | `record-test/`、`build/`、`install/`、`log/` 等为本机产物；engine 应在目标设备重建 |
| 容器临时软件 | 重建容器可能丢失；长期需要的依赖应写入 Dockerfile |

## 6. 当前完成与待办

已完成 GPU 容器与此前 14 包构建、视频六项链路检查、CPU/GPU 样本对照、软件断流恢复和短时稳定性；最新相机加固相关三包独立构建及 39 个用例通过，证据与边界见验证记录。

仍需：更新正式构建，接真实相机检查成像/内参/时间戳/USB 恢复，接电控核对协议和外参，最后验收无发射跟随。实际相机掉线可能触发 ROS 退出，不保证自动重连；电控须自行处理视觉失联。当前总览不能替代这些硬件结果。
