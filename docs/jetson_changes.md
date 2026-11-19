# Jetson 适配改动说明

首次记录：2026-11-17；更新日期：2026-11-19。适配分支：`infantry-jetson-Lonesome-George`。

本文记录本次实际修改、参数变化及验证范围。测试和部署步骤见
[测试与 Orin Nano 部署手册](jetson.md)。

第 2～6 节汇总适配实现；[第 9 节](#9-2026-11-19-本次工作区改动与验收)
记录相对提交 `71460d6` 的 CPU 迁移；第 10 节记录相对 `468f9c6` 的 GPU 改动，
避免把此前已有的 TensorRT 后端算作本次新增；第 11 节为断流补充验证，第 12 节记录
最新的 YOLO GPU 硬件核对和 CUDA Graph 优化；第 13 节记录接相机前的取帧与退出修复。

## 1. 目标与当前状态

目标设备由用户提供的终端输出确认：

| 项目 | 环境 |
| --- | --- |
| 设备 | NVIDIA Jetson Orin Nano Developer Kit Super，用户确认 8GB |
| 系统 | 用户确认 Ubuntu 22.04，AArch64 |
| JetPack | `6.2.3+b81` |
| L4T | `36.5.2` |
| 宿主机 CUDA Toolkit | `12.6` |
| TensorRT | `10.3.0.30` |

两文件 Compose 保留 OpenVINO CPU 基线；新增第三个 GPU 覆盖文件选择
TensorRT/CUDA 路线，具体操作见 [GPU 手册](jetson_gpu.md)。已有 IR 已转换为 ONNX，
转换和推理结果的实际验证范围、性能数据及限制见 [第 10 节](#10-tensorrtcuda-适配与性能验收)。
真实相机、电控和云台闭环仍需接入硬件后验收。

首次适配代码已形成提交 `87540e4`。后续操作手册更新以 Git 历史为准；源码已提交
不等于 Orin 上已通过部署测试，目标设备结果按操作手册逐步验证和记录。

## 2. 海康相机 SDK

### 原问题

`src/rm_utils/hikSDK/lib/arm64/` 中原有的 `libMvCameraControl.so` 和
`libMvUsb3vTL.so` 实际是 x86-64 文件。目录名不能改变二进制架构，直接在 Jetson
链接会失败。

### 实际修改

| 文件或目录 | 修改 |
| --- | --- |
| `src/rm_utils/hikSDK/lib/arm64/libMvCameraControl.so` | 替换为 AArch64 库 |
| `src/rm_utils/hikSDK/lib/arm64/libMvUsb3vTL.so` | 替换为 AArch64 USB3 传输库 |
| 同目录的 `libFormatConversion.so`、`libMediaProcess.so`、`libMVRender.so` | 补充配套 ARM64 库 |
| `src/rm_utils/hikSDK/include/arm64/` | 增加与 ARM64 库配套的 7 个头文件 |
| `src/rm_utils/cmake/hik_sdk.cmake` | 统一选择 SDK、检查头文件和两个关键库的 ELF 架构 |
| `src/rm_utils/CMakeLists.txt` | 使用统一 SDK 检查，链接和安装选定架构的库与头文件 |
| `src/rm_hardware_driver/hik-camera/CMakeLists.txt` | 使用同一套 SDK 选择逻辑 |

ARM64 文件取自本机已有的 `open-cv_-calibration/hikSDK`，其中库文件最近提交为
`440a35d`；保留厂商文件原始内容。x86-64 继续使用原 `include/` 和 `lib/amd64/`。

构建时支持 `-DHIK_SDK_ROOT=/opt/MVS` 或同名环境变量，也支持官方单架构
`include/` + `lib/64/` 布局。来源与目录说明见
[SDK 说明](../src/rm_utils/hikSDK/README.md)。

## 3. 装甲板推理后端

| 文件 | 修改 |
| --- | --- |
| `armor_detector/include/armor_detector/tensorrt_inference.hpp` | 新增 TensorRT 推理接口 |
| `armor_detector/src/tensorrt_inference.cpp` | 新增 TensorRT 10 engine 加载、CUDA 缓冲区、推理和资源回收 |
| `armor_detector/include/armor_detector/yolov5_detector.hpp` | 增加后端选择及 TensorRT 实例 |
| `armor_detector/src/yolov5_detector.cpp` | 共用缩放、补边和原有后处理；增加输入、模型形状检查 |
| `armor_detector/src/armor_detector_node.cpp` | 将后端和模型路径改为可配置参数 |
| `armor_detector/CMakeLists.txt` | 增加 `ENABLE_TENSORRT` 编译开关，默认 `OFF`；启用时寻找 CUDA/TensorRT 开发库 |

表中 `armor_detector/` 的完整前缀为 `src/rm_auto_aim/armor_detector/`。
该包 CMake 最低版本调整为 3.18，用于使用 `FindCUDAToolkit`。

新增参数：

| 参数 | 含义 |
| --- | --- |
| `yolo.backend` | `openvino` 或 `tensorrt`，启动时选择 |
| `yolo.model` | OpenVINO XML 或 TensorRT engine 路径 |
| `yolo.weights` | OpenVINO BIN 路径；TensorRT 路径忽略 |

模型路径支持 `package://`、`file://` 和普通文件路径。`device` 仍用于 OpenVINO；
TensorRT 使用当前 CUDA 设备。后端及模型路径需要重启节点后生效。

TensorRT 实现使用 `setTensorAddress` / `enqueueV3`。输入为 FP32、线性布局
`[1,3,640,640]`；输出为 FP32 `[1,N,22]`。内部 FP16 计算可以在生成 engine 时
开启，输入输出仍保持 FP32。

两条路径沿用 RGB、除以 255、等比例缩放后左上角对齐补黑边的约定，并共用原来的
四角点、颜色、类别、置信度和 NMS 解析。现有 IR 的输出形状为 `[1,25200,22]`。
多输出、动态形状及其他布局会被拒绝，以免把普通目标框模型错误用于角点解析。

仍缺少 `0526` 对应的原始 ONNX/训练权重；本轮改为从已有 IR 转换并进行样本回归，
详见第 10 节。`lenet.onnx` 和 `mlp.onnx` 属于数字分类模型，不能替代装甲板模型。

## 4. Docker 与编译

| 文件 | 修改 |
| --- | --- |
| `Dockerfile` | 支持 `BASE_IMAGE`；原生架构 Ubuntu 为默认基础镜像 |
| `Dockerfile` | ARM64 使用官方 Ubuntu 22.04 OpenVINO 2025.3.0 预编译包，校验后解压到 `/opt/openvino` |
| `Dockerfile` | 编译并发默认从 16 降到 2，原先写死的 `make -j16` 改为读取参数 |
| `Dockerfile` | 补充 `libtbb-dev`、`libusb-1.0-0`，调整 ROS/OpenVINO/工作区环境加载 |
| `docker-compose.jetson.yaml` | 新增 ARM64 覆盖配置：NVIDIA runtime、1GB 共享内存、Jetson 参数环境变量 |
| `scripts/jetson/build.sh` | 新增容器内构建入口：同时编译 1 个包，包内默认 2 个编译任务 |
| `.dockerignore` | 排除 Git、构建、缓存、录像等目录 |
| `.gitignore` | 忽略生成的 `*.engine`、`*.plan` 和 `__pycache__/` |

**编译并发只控制构建过程，不限制 ROS 运行线程、OpenVINO 推理线程或 GPU 并行度。**
它可能增加编译时间，在相同代码与优化选项下不会因此降低运行性能。原有 `-O3`
优化保留。

Jetson Compose 默认基础镜像为 `ubuntu:22.04`，用于 CPU 验证。设置了 NVIDIA
runtime 并不等于容器已经具有 CUDA Toolkit 和 TensorRT 开发包。现在 GPU 路线通过
第三个 Compose 文件和 `Dockerfile.jetson-gpu` 补齐开发依赖，见第 10 节。

初始 CPU 部署时使用 `BUILD_TESTING=OFF` 避开解算包不存在的测试引用；该入口已在
第 11 节修复，解算测试现可正常运行。部署脚本仍默认关闭测试，检测与解算测试按需
分别开启，见操作手册。

## 5. 启动配置与参数覆盖

`src/rm_bringup/launch/bringup_SingleProcess.launch.py` 新增
`RM_VISION_PROFILE=jetson` 选择，加载顺序为通用参数、Jetson 覆盖参数和 launch
显式参数。Jetson Compose 自动设置该环境变量；普通开发配置默认仍可用。

新增覆盖文件：

| 文件（位于 `src/rm_bringup/config/jetson/`） | 默认行为 |
| --- | --- |
| `launch_params.yaml` | 关闭打符、录像、回放、自动标定和自动启动 Foxglove；当前使用视频、虚拟串口 |
| `armor_detector_params.yaml` | YOLO + OpenVINO CPU，加载已有 IR；检测器 `debug: false` |
| `camera_driver_params.yaml` | 两个相机均开启 30Hz 限帧 |

视频参数位于通用 `config/node_params/video_player_params.yaml`：当前路径为
`/ros_ws/test.mp4`，播放频率为 30Hz，循环播放。通用 `armor_detector_params.yaml`
中的 `device` 也由 `GPU` 改为 `CPU`；这两项是通用文件改动，会影响其他使用它们的启动方式。
视频播放频率和真实相机的采集限帧是独立参数。

**30Hz 是新增的首次 CPU 流程验证配置，会影响实际采集频率。** 原通用配置中，
6mm 相机开启 100Hz 限帧；8mm 相机参数为 200Hz，但未开启限帧。通用文件中的
这些值保持原样，启用 Jetson 配置时才由覆盖文件替换。恢复方法见操作手册。

另外修复相机参数加载：通用 `camera_driver_params.yaml` 是普通嵌套 YAML，直接
作为 ROS 参数文件会报 `Cannot have a value before ros__parameters`；现在由
launch 读为字典，再交给 ROS launch 展平为参数。

跟踪器、弹道补偿、串口协议、相机内参、手眼标定数值沿用原逻辑和原配置。
`rune: false` 原本就会让 launch 选择 `hero` 串口协议实现，本次沿用此行为。

## 6. 新增工具与测试

- `scripts/jetson/build_armor_engine.py`：在目标设备检查 ONNX 的输入输出约定并生成
  TensorRT 10 engine；支持指定工作空间和内部 FP16 计算。
- `armor_detector/test/test_yolo_backends.cpp`：检查未知后端、现有 IR 的 CPU 加载
  与非法输入处理，并提供参考图片上的 CPU/GPU 结果对比。
- GPU 对比检查检测数量、类别、装甲板类型、置信度和角点。当前测试容差为
  置信度差不超过 0.02、角点距离小于 2 像素；这属于回归门槛，不代表实战精度。
- 未设置 `ARMOR_TENSORRT_ENGINE` 时，GPU 对比项明确跳过。
- `scripts/jetson/check_pipeline.py`：新增只读链路检查工具，默认订阅 20 秒，检查
  图像、姿态弹速、非空装甲板结果、有效跟踪、有效云台指令和成功弹道补偿。
  每项至少观察到两条满足条件的数据才通过，结果保存到被 Git 忽略的 `record-test/`。
  它不发送控制命令，也不验证消息逐帧对应、瞄准精度、串口发送或真实云台响应。

## 7. 验证记录与边界

| 检查 | 已有结果 | 能说明什么 |
| --- | --- | --- |
| 前期 x86 Docker 完整部署构建 | 14 个包通过，历史记录 | 作为此前平台的构建记录 |
| 本机 Orin Nano ARM64 部署构建 | 2026-11-19，14 个包通过 | 目标设备容器中的工作区可构建，ROS 能定位当前安装包 |
| 本机原有传统检测测试 | `test_detector` 通过 | 参考图片上的传统检测回归通过 |
| 本机 CPU 后端测试 | 2 项通过 | 未知后端检查、现有 IR 加载、非法输入检查和有效图像基本推理通过 |
| GPU 对比测试 | 原始 gtest XML 中 `result="skipped"` | 尚未验证 GPU 推理或 CPU/GPU 结果一致性 |
| 视频与虚拟电控功能检查 | 20.03 秒观察，六项全部通过 | 数据流到达跟踪、弹道补偿和云台指令输出端 |
| TensorRT 10.3 官方头文件语法编译 | 此前通过 | C++ 接口兼容性检查，不含 GPU 链接或执行 |
| SDK 静态检查 | ARM64 架构正确，包含源码使用的 15 个相机 API | 不代表已经打开真实相机 |
| 配置检查 | Compose、普通/Jetson 参数覆盖通过 | 不代表硬件通信成功 |
| 本机测试汇总 | `6 tests, 0 errors, 0 failures, 0 skipped` | 汇总包含包装项，跳过状态以原始 gtest XML 为准 |

`colcon test-result` 汇总可能包含 CTest 包装项，并可能没有正确统计 gtest 的跳过
项目。应查看测试明细中的 `result="skipped"` 或直接运行测试程序；不能仅凭
`6 tests, 0 skipped` 判断 GPU 已验证。

待完成的目标设备检查：真实相机采集与内参、真实串口双向收发、外参与真实云台闭环、
目标丢失和恢复、持续运行，以及取得可用模型后的 TensorRT GPU 构建、结果对比与性能优化。

## 8. 2026-11-18 本机部署审查补充

- Dockerfile 改为读取容器内 `dpkg --print-architecture` 选择 ARM/Intel 路径，
  不依赖构建器是否提供 `TARGETARCH`。
- 修正 ARM Ubuntu 软件源替换，保留一份 `/ubuntu-ports`，同时兼容 `:80`。
  原替换会在已有 `/ubuntu-ports` 的地址前重复追加该路径。
- ARM OpenVINO 改用约 40 MB 的官方下载包，包含 CPU 插件、IR 和 ONNX 前端，
  省去在 Jetson 上克隆和编译 OpenVINO。即使运行配置关闭打符，完整工作区里的
  `rune_detector` 仍需要 ONNX 前端。Jetson 按 Dockerfile 的清单安装依赖，首次构建
  跳过 `rosdep update`。
- Jetson Compose 的构建阶段使用宿主机网络，并传入 HTTP/HTTPS 代理构建参数，
  以访问本机代理。使用 `sudo` 时显式传入代理的方法见部署手册第 3.4 节。
- 本机实测中科大 Ubuntu/ROS 镜像直连返回有效签名索引，经当前代理则返回
  502 或 TLS 错误。Dockerfile 为该镜像域名配置 apt `DIRECT`，Compose 同时将其
  加入大小写 `NO_PROXY` 构建参数，供 curl 和 rosdep 直连；其余地址继续按代理配置访问。
- Jetson profile 使用 `ros-humble-ros-base`，跳过 Desktop、PlotJuggler、图形相机
  标定及 clangd 等非部署工具。完整 workspace 仍安装 OpenVINO CPU runtime、IR/ONNX
  前端；GUI 标定与 PlotJuggler 可在需要时于容器内安装。
- 截至 2026-11-18，仅验证了 Compose 合并、ARM 源替换和构建脚本语法，尚未完成
  ARM 镜像及工作区构建。2026-11-19 已补充本机 CPU 构建与视频链路验收，见下节。
  现有 Pixi 配置仅列出 `linux-64`，本次 Jetson 部署继续使用 Docker 路线。

## 9. 2026-11-19 本次工作区改动与验收

### 9.1 对比范围

对比基准为 `71460d6`（`docs(jetson): 细化小电脑测试与部署步骤`）。以下为该提交
之后的工作区改动；本次没有修改 YOLO 解析、跟踪器、弹道或串口协议的 C++ 实现。
TensorRT 后端与 engine 构建工具在基准提交中已经存在，本次未启用或验证 GPU 推理。

### 9.2 文件与行为变化

| 文件或目录 | 本次变化及作用 |
| --- | --- |
| `Dockerfile` | 使用容器内架构判断修正 ARM 软件源；精简 Jetson 依赖；OpenVINO 从源码构建改为带 SHA-256 校验的官方预编译包，支持本地缓存；修复入口脚本参数传递及环境加载顺序 |
| `docker-compose.jetson.yaml` | 启用构建阶段宿主机网络，传入代理和镜像直连配置；选择精简 ROS `ros-base` 与 2 个构建任务 |
| `scripts/jetson/openvino-cache/.gitignore` | 保留缓存目录并忽略 `*.tgz`；安装包只供本地构建复用，不提交到 Git |
| `src/rm_bringup/config/jetson/launch_params.yaml` | 启用视频和虚拟串口，关闭自动启动 Foxglove，建立无相机、无电控的验收配置 |
| `src/rm_bringup/config/node_params/armor_detector_params.yaml` | 通用 OpenVINO 设备由 `GPU` 改为 `CPU`，与当前验证路线一致 |
| `src/rm_bringup/config/node_params/video_player_params.yaml` | 视频改为容器内绝对路径 `/ros_ws/test.mp4`，播放频率由 100 改为 30Hz |
| `scripts/jetson/check_pipeline.py` | 新增六项功能检查，生成可读结果及 JSON 报告 |
| `docs/` | 将原根目录的 8 个 Markdown 文件统一移入文档目录；修正文档与源码链接，补充本机部署和验收状态 |

迁移的 8 个文件是 `AGENTS.md`、`README.md`、`SERIAL_PROTOCOL.md`、
`QD26步兵视觉电控通信协议与联调说明.md`、`infantry-jeson串口协议_电控对接.md`、
`手眼标定说明与执行指南.md`、`自瞄上车测试步骤.md` 和 `运行自瞄测试前(1).md`。
移动后对说明、引用或旧配置描述的整理不涉及算法改动。

此外，本机 `~/.bashrc` 已将 `rv` 配置为 `sudo docker exec -it rv_devel_ bash`，
让新终端可使用该快捷命令。此设置位于仓库外，不随本次 Git 提交保存。

### 9.3 实际验收结果

1. Docker 构建完成，`rv_devel_` 容器启动，挂载当前仓库到 `/ros_ws`。
2. `bash scripts/jetson/build.sh` 完成 14 个包构建；`armor_detector` 和 `rm_bringup`
   的 ROS 包路径均位于 `/ros_ws/install/`。
3. 检测包两组 CTest 全部通过。原始 gtest XML 中，传统检测和两个 CPU 后端测试
   完成，`TensorRtMatchesCpuOnReferenceImage` 明确跳过。
4. 视频与虚拟电控链路观察 20.03 秒，结果如下；下列计数是检查脚本收到的消息数，
   不是模型帧率或严格逐帧对应数量。

| 环节 | 话题 | 收到 / 满足条件 |
| --- | --- | --- |
| 图像输入 | `/image_raw` | 338 / 338 |
| 虚拟姿态与弹速 | `/serial/receive` | 6008 / 6008 |
| 检测与坐标转换 | `/armor_solver/armors` | 89 / 89 |
| 目标跟踪 | `/armor_solver/target` | 89 / 89 |
| 云台指令输出 | `/armor_solver/cmd_gimbal` | 3053 / 3053 |
| 弹道补偿 | `/armor_solver/ballistic_debug` | 3051 / 3051 |

本机报告为 `record-test/pipeline-check-20261119-131953-023116.json`，原始测试结果位于
`build/armor_detector/test_results/armor_detector/`。这些是被 Git 忽略的本机产物；
本文保留关键结果，其他机器需按操作手册重新生成记录。

### 9.4 性能观察与未完成事项

此前一次独立的 20 秒日志观察中，CPU 推理耗时中位数为 206.31ms，推理完成频率约
4.35 帧/秒；图像到预测延迟中位数为 487.72ms、P95 为 635.62ms，解算耗时中位数
为 2.88ms。这是当前视频、参数及负载下的观察值，尚未进行受控基准测试，不能据此
推断原 NUC 的性能差异或 Jetson GPU 的能力。

启动时出现 2 次 TF 向过去外推错误，原因是最初图像时间早于 TF 缓冲区最早记录；
后续日志观察未新增该错误，并持续进行了跟踪和解算。这不代表所有输入中断和恢复
场景已经测试。

截至本节对应的 CPU 提交，CUDA/TensorRT 性能适配仍处于方案阶段，未进行对应代码修改。
当前仅有 `0526.xml/.bin`，缺少对应原始 ONNX、训练权重及导出代码；IR 转 ONNX
的可行性与数值一致性也尚未验证。后续先确认模型来源或转换可行性，再接通 GPU
环境、验证结果一致性并优化延迟。真实相机、电控和云台闭环仍需硬件接入后验收。

本次提交名：`完成 ARM64 CPU 部署与视频自瞄链路验收-Lonesome-George`。


## 10. TensorRT/CUDA 适配与性能验收

### 10.1 范围与设计

本节对比基准为 `468f9c6`（完成 ARM64 CPU 部署与视频自瞄链路验收）。
GPU 工作尚未另行提交。本机确认是 **Orin Nano Super**，CUDA 架构为 SM 8.7；
此配置不能直接用于旧款 Jetson Nano 的 JetPack 4 环境。

YOLO 卷积使用 TensorRT，FP16 允许利用 GPU 半精度计算；输入 RGB 排列和归一化使用
CUDA。缩放、补边、角点输出、颜色/类别解析、NMS、PnP、跟踪及弹道保持原有语义。
PnP 当前是少量角点的 CPU 求解，没有在本轮盲目搬到 GPU；先减少分配、排队和无用
可视化，再通过整链路测量判断后续瓶颈。没有承诺已达到设备理论最优性能。

### 10.2 每处改动与目的

| 文件（路径相对仓库根目录） | 改了什么 | 目的与原理 |
| --- | --- | --- |
| `Dockerfile.jetson-gpu` | 在已验证的 CPU 镜像上安装固定版本 CUDA 12.6 编译工具、TensorRT 10.3 开发/解析/Python 库及 cuDLA 运行库；声明 GPU 设备与驱动能力 | 保持容器编译和运行依赖匹配本机；NVIDIA runtime 将宿主机驱动库注入容器，避免仅安装 Python 模块却不能创建 GPU Builder |
| `docker-compose.jetson.gpu.yaml` | 新增第三层覆盖文件，选择 GPU 镜像、编译开关、TensorRT 后端和 INFO 文件日志 | 明确选择 GPU 环境，保留原两文件 CPU 环境作为对照与回退 |
| `scripts/jetson/build.sh` | 同时设置 Make 并发并验证任务数 | 修复 Humble colcon 显式 `-j6` 覆盖 CMake 并发限制的问题，默认真正按 2 个任务构建 |
| `scripts/jetson/requirements-model-tools.txt` | 固定 Python 3.10 可用的转换器及 ONNX 等依赖版本 | 避免新版本要求更高 Python 导致安装失败，便于重复转换 |
| `scripts/jetson/convert_armor_ir.py` | IR 转 ONNX；仅移除未被使用的 MaxPool 索引输出；验证形状、数值、类别、颜色、阈值结果并记录 SHA-256 | TensorRT 不直接读取 XML/BIN；确认转换没有改变已检查样本的有效候选语义，失败即停止 |
| `scripts/jetson/build_armor_engine.py` | FP32 对照关闭 TF32；保存精度、输入输出形状、TensorRT 版本和模型/engine 哈希 | 分清 FP32 与 FP16 误差来源，使 engine 可追溯；仍在目标设备生成，不跨设备复制 |
| `scripts/jetson/prepare_tensorrt.sh` | 一次完成工具环境、转换检查和两种 engine 生成 | 把验证顺序固化，避免转换未通过就直接启用 GPU |
| `scripts/jetson/benchmark_pipeline.py` | 新增只订阅的预热/采样脚本，输出输入、处理、跟踪和命令频率，以及带原图时间戳的消息延迟 | 区分模型处理率与定时云台命令率，为 CPU/GPU 比较提供同一种观测方式 |
| `src/rm_auto_aim/armor_detector/CMakeLists.txt` | 条件编译 CUDA 内核；Orin 默认架构 87；将 C++ 警告选项限制于 C++；加入 CUDA 测试 | NVCC 不接受部分 C++ 编译选项；仅启用 TensorRT 时依赖 CUDA，CPU 编译路径可继续使用 |
| `src/rm_auto_aim/armor_detector/include/armor_detector/cuda_preprocess.hpp` | 声明流内预处理接口及设备指针、容量契约 | 让 CUDA 代码和 C++ 推理后端职责清楚，沿用同一 CUDA 流 |
| `src/rm_auto_aim/armor_detector/src/cuda_preprocess.cu` | RGB uint8 HWC 转 FP32 NCHW，并乘以 1/255 | 将通道整理与归一化搬到 GPU，覆盖非线程块整数倍的像素边界 |
| `src/rm_auto_aim/armor_detector/src/tensorrt_inference.cpp` | 复用固定页主机内存、GPU 缓冲区和非阻塞流，异步传输→CUDA 预处理→TensorRT→结果回传，最后同步一次 | 输入按 uint8 传输，由 4.92MB 降至 1.23MB；减少逐帧分配并支持异步拷贝。单帧调用仍等待结果，没有虚称实现多帧流水 |
| `src/rm_auto_aim/armor_detector/src/armor_detector_node.cpp` | 图像 QoS 深度与 OpenCV 线程数可配置 | Jetson 使用最新帧减少陈旧队列，限制 OpenCV 嵌套线程竞争；线程数设置影响同一进程内 OpenCV |
| `src/rm_bringup/config/jetson/armor_detector_params.yaml` | 图像队列深度设 1、OpenCV 线程数设 1，保留 CPU 模型参数 | 为 Jetson 降低排队与 CPU 争用，原 IR 保留用于对照 |
| `src/rm_bringup/config/jetson/armor_detector_tensorrt_params.yaml` | 新增独立 TensorRT 参数，选择本机生成的 FP16 engine | GPU 覆盖文件独立于 CPU 基线，避免改错通用模型路径 |
| `src/rm_bringup/launch/bringup_SingleProcess.launch.py` | 校验后端环境变量，TensorRT 模式追加 GPU 参数文件 | GPU Compose 自动选择正确模型；CPU 排障可在 shell 切换，无需改源码 |
| `src/rm_auto_aim/armor_detector/src/armor_pose_estimator.cpp` | PnP 输出数组预留容量，同一帧内复用候选数组 | 减少小对象分配；求解器、候选选择、角度和坐标转换未改 |
| `src/rm_auto_aim/armor_solver/src/armor_solver_node.cpp` | 没有结果图订阅者时不排队绘图，不做仅供绘图的重投影 | 将 CPU 留给跟踪与弹道；新增订阅者后仍可显示结果图 |
| `src/rm_utils/include/rm_utils/logger/impl/logger_impl.hpp` | 在格式化前过滤无输出的日志，分别控制文件和终端阈值 | 减少高频 DEBUG 格式化与同步刷盘开销 |
| `src/rm_utils/src/logger/logger_impl.cpp` | 读取 `RM_LOG_FILE_LEVEL`，默认仍保留 DEBUG，GPU Compose 设 INFO | GPU 默认保留信息、警告和错误；排障时可设 DEBUG 后重启进程 |
| `src/rm_auto_aim/armor_detector/test/test_cuda_preprocess.cpp` | 对比 CUDA 与 OpenCV，包括 RGB 边界值、随机图像和线程块边界 | 检查归一化、通道顺序及越界保护，实际要求可用 GPU |
| `src/rm_auto_aim/armor_detector/test/test_yolo_backends.cpp` | 复用最终检测对比，新增 5 个视频位置与红/蓝筛选回归 | 不只验证 engine 能加载，还验证最终数量、类别、类型、置信度和角点 |
| `docs/jetson_gpu.md` | 新增完整 GPU 操作步骤、命令位置、原理、成功条件和回退方法 | 用户可复现模型生成、编译、回归和软件链路验收 |
| `docs/jetson_changes.md` | 本节记录全部变化、实测与边界 | 区分之前的 CPU 提交与本次 GPU 工作 |
| `docs/jetson.md`、`docs/README.md`、`docs/自瞄上车测试步骤.md` | 更新 GPU 入口和进度，去除过时的“只能等原始 ONNX”步骤 | 让部署文档与实际可用的 IR 转换路线保持一致 |

### 10.3 验证结果

本机为 Orin Nano Super 8GB，功耗模式查询为 `MAXN_SUPER`；本轮没有调整功耗或锁频。
测试使用仓库 `test.mp4`、30Hz、虚拟电控，真实相机/串口未接入。

**编译与正确性：**

- GPU 镜像构建成功，容器内 CUDA 12.6、TensorRT Python 10.3.0 可创建 GPU Builder。
  系统 TensorRT 包为 10.3.0.30，包版本带 `cuda12.5` 后缀，与本机 CUDA 12.6 配套运行已验证。
- 完整 14 个 ROS 包构建成功。补齐 Make 限制后，再次检查构建命令不再注入 `-j6`。
- IR→ONNX 七个样本全部通过，有效候选最大角点差约 0.00156 模型像素，最大置信度差约
  0.000154；颜色、类别、0.65 阈值筛选一致。只移除了无消费者的池化索引，没有更换模型。
- FP32 和 FP16 engine 的参考图与视频抽帧回归均通过，数量、号码、装甲板类型一致，
  置信度差不超过 0.02、角点差小于 2 原图像素。FP16 通过后才被设为 GPU 默认模型。
- CUDA 预处理边界/通道测试、传统检测节点测试通过。最终 3 个 CTest 对应 6 个 gtest
  用例，原始 XML 中 GPU 项均实际运行，没有跳过。`colcon test-result` 将包装测试也计入，
  汇总为 `9 tests, 0 errors, 0 failures, 0 skipped`。

**相同当前代码及观测脚本的性能对照：** 两次均预热 3 秒、采样 20 秒，依次运行 CPU
与 GPU，测试时没有其他构建或 engine 生成任务。CPU 对照也使用队列深度 1、OpenCV
线程数 1 和 INFO 文件日志，因此下面对比主要反映后端切换效果。

| 指标 | 当前 OpenVINO CPU 对照 | TensorRT FP16 |
| --- | ---: | ---: |
| 处理结果频率（armors） | 4.049 Hz | 28.998 Hz |
| 跟踪结果频率（target） | 4.049 Hz | 28.948 Hz |
| 图像到检测/坐标转换消息，中位延迟 | 260.895 ms | 23.861 ms |
| 图像到检测/坐标转换消息，P95 | 305.402 ms | 34.730 ms |
| 图像到跟踪结果，中位延迟 | 262.762 ms | 24.207 ms |
| 图像到跟踪结果，P95 | 306.429 ms | 35.153 ms |

处理结果频率约提高 **7.16 倍**，跟踪消息中位延迟下降约 **90.8%**。这是本机 Jetson
的 CPU/GPU 对照，没有原 NUC 的同条件数据，不能据此比较 NUC 与 Jetson 的性能。
视频限帧 30Hz，不能把 29Hz 当作 GPU 模型最大吞吐；后台桌面/IDE 保持运行，测试
并非隔离环境下的硬实时保证。外部脚本接收大图像时可能丢帧，因此其 `images` 接收率
低于进程内检测率并不表示重复检测；该值是观测者接收率，不是视频发布频率。

此前迁移前 CPU 基线为 3.949Hz、跟踪中位延迟 389.196ms；当时仍有额外图像话题
观测者，作为历史记录保留，主要结论以上表当前 CPU 对照为准。

**独立阶段计时：** 另开 12 秒 DEBUG 日志观测，去除启动前 3 秒，222 个样本中：
检测阶段中位数 17.175ms、P95 24.140ms；PnP 位姿提取中位数 **0.230ms**、
P95 **0.370ms**。前者含预处理、TensorRT、传输与解析，后者不是完整跟踪/弹道耗时；
此次 DEBUG 观测含日志开销，与上表端到端测量分开。当前少量角点的 PnP 没有显示为
主要瓶颈，因此保留 CPU 求解。未来硬件高帧率、多目标负载下需重新测量。

**GPU 软件功能链路：**

| 检查环节 | 收到 / 有效消息数 |
| --- | ---: |
| 图像输入 | 336 / 336 |
| 虚拟姿态与弹速 | 11684 / 11684 |
| 检测与坐标转换 | 512 / 512 |
| 目标跟踪 | 512 / 512 |
| 云台指令输出 | 3978 / 3978 |
| 弹道补偿 | 3974 / 3974 |

20 秒六项检查通过。检查工具会增加消息观测开销，各次窗口的消息计数不能混作同一
帧逐级对应或模型 FPS。启动最初出现 2 次向过去外推 TF 错误，后续观察未新增；
未使用最新 TF 或修改时间戳来掩盖时序问题。输入中断/恢复、长时间负载及真实硬件仍需验收。

### 10.4 本地产物、运行状态与下一步

以下文件被 Git 忽略；本文保留关键数字，其他设备需重新生成。

| 产物 | 路径 |
| --- | --- |
| 模型转换及哈希报告 | `record-test/models/0526.conversion.json` |
| FP32 / FP16 engine 与元数据 | `record-test/models/0526-fp32.engine[.json]`、`0526-fp16.engine[.json]` |
| 迁移前 CPU 基线 | `record-test/benchmark-20261119-140724-965973.json` |
| 当前 CPU 对照 | `record-test/benchmark-20261119-144437-233012.json` |
| TensorRT FP16 性能 | `record-test/benchmark-20261119-144529-844736.json` |
| GPU 六项功能检查 | `record-test/pipeline-check-20261119-144607-000069.json` |
| 阶段计时 | `record-test/gpu-stage-profile.json` |
| GPU 精度回归 | `record-test/yolo-fp32.xml`、`record-test/yolo-fp16.xml`、`record-test/cuda-preprocess.xml` |
| 完整构建记录 | `record-test/gpu-workspace-build.log` |
| 资源观测 | `record-test/tegra-cpu-current.log`、`record-test/tegra-tensorrt-fp16.log` |

验收后 `rv_devel_` 已使用 `qyg-vision:jetson-jp6-gpu`，默认后端 TensorRT FP16、
INFO 文件日志。临时测试容器已清理，CPU 基础镜像保留。
本轮后台启动的视频主程序日志位于 `record-test/gpu-runtime-final.log`；它不是开机自启服务，
容器重启后仍需按 GPU 手册启动，运行中不要再启动第二套主 launch。

宿主机 `~/.bashrc` 的 `rv` 已从 `sudo docker exec ...` 改为
`sg docker -c "docker exec -it rv_devel_ bash"`，使用用户已获授权的 Docker 组；
新终端实际进入 GPU 容器已验证，不需要 sudo 密码。原配置备份为
`~/.bashrc.qyg-before-gpu-20261119`。此变化位于仓库外，不随 Git 提交保存；旧终端可先
执行 `source ~/.bashrc` 重新加载别名。

下一步先接相机验收出图、内参、手眼外参、静态/运动角点和位姿精度，再接真实串口核对
协议及姿态方向，最后做禁止发射状态下的云台跟随与受控闭环测试。当前软件链路通过
不代表真实测距、瞄准、串口发送、发射安全或长时间实时性已经通过。

## 11. 2026-11-19 断流失效与恢复验证

### 11.1 复现的问题与处理方式

GPU 接通后继续验证“图像完全停止、虚拟串口仍正常”的场景。原跟踪器的丢失计数依赖
新图像帧推进；没有图像时不会触发正常的 LOST 转移，而定时解算仍会使用旧目标。
卸载视频组件的实测中，1 秒尾帧宽限后的 2 秒窗口收到了 **507 条旧目标有效指令**，
没有无效指令或 `tracking=false` 通知，视频恢复后才重新初始化。

本轮增加 `tracker.input_timeout`，默认 0.5 秒：最后一次成功处理图像观测后超时，
由独立定时器撤销目标、清空连续检测/丢失计数、发布 `tracking=false`，并沿用原有
无效指令格式 `id=0`、`distance=-1`、`fire_advice=false`。它不是新的串口协议，
真实电控收到这些字段后的行为仍需接入硬件核对。恢复输入时从 LOST 重新确认目标，
不把中断前的计数或预测结果当成新观测。

断流检测使用单调时钟，避免系统校时或 ROS 时间跳变影响超时计量；定时器和处理
线程共用原有状态锁。只有成功完成坐标转换及跟踪处理的输入刷新计时，持续 TF 失败
也不会无限保留旧目标。本轮没有改 YOLO、PnP、跟踪滤波方程或弹道算法。

### 11.2 本轮逐项改动

| 文件 | 改动 | 目的 |
| --- | --- | --- |
| `src/rm_auto_aim/armor_solver/include/armor_solver/armor_solver_node.hpp` | 保存输入超时秒数与最后成功处理时刻 | 在无新回调的情况下仍能判断输入是否失效 |
| `src/rm_auto_aim/armor_solver/src/armor_solver_node.cpp` | 校验超时参数，定时撤销超时目标，补齐无效指令时间戳，成功处理后刷新单调时钟 | 完全断流后不再持续输出旧目标，状态读写沿用同一把锁 |
| `src/rm_auto_aim/armor_solver/include/armor_solver/armor_tracker.hpp` | 增加带调用约束说明的 `reset()` 接口 | 统一定义从旧跟踪回到 LOST 的操作 |
| `src/rm_auto_aim/armor_solver/src/armor_tracker.cpp` | 重置状态、号码和连续检测/丢失计数 | 恢复后使用完整的新观测确认，不继承旧计数 |
| `src/rm_bringup/config/node_params/armor_solver_params.yaml` | 明确 `tracker.input_timeout: 0.5` | 使超时策略可配置；修改后需重启节点 |
| `src/rm_auto_aim/armor_solver/test/test_tracker_prediction.cpp` | 新增检测计数和丢失计数重置的两项回归 | 验证恢复跟踪的状态语义，保留原有预测测试 |
| `src/rm_auto_aim/armor_solver/CMakeLists.txt` | 将不存在的 `test_solver_motion_switches.cpp` 测试引用替换为仓库实际存在的 `test_spin_imm.cpp` | 修复解算包开启测试时的配置失败，运行现有测试，不跳过缺失项来掩盖错误 |
| `scripts/jetson/check_stream_recovery.py` | 保存实际视频参数、临时卸载视频组件、分阶段观测，再尝试按原参数恢复；失败也保存 JSON | 可重复复现和验收断流；脚本自身只订阅数据及调用组件服务，不发布云台指令 |
| `scripts/jetson/benchmark_pipeline.py` | 增加分段处理率、最大结果间隔、非递增时间戳和组件 RSS/Swap；可不订阅大图像 | 让持续运行检查能发现短时停顿和内存趋势，区分观测开销与实际处理能力 |
| `docs/jetson_gpu.md`、`docs/自瞄上车测试步骤.md` | 补充无硬件断流/持续运行步骤、原理和通过条件 | 与上车前的软件验证顺序衔接 |
| `docs/jetson_changes.md`、`docs/jetson.md` | 记录本轮复现、修复和实测，更新解算测试说明 | 保留可审查的前后对照和验证边界 |

### 11.3 本轮验收记录

- `armor_solver`、`rm_bringup` 编译成功。解算包 3 个测试程序内的 **13 个 gtest 用例**
  全部通过，包括新增 2 项；CTest 包装计数合计 `16 tests, 0 errors, 0 failures, 0 skipped`。
- 断流后约 **0.493 秒**收到第一条无效指令；此时间相对于卸载服务返回的时刻测量，
  不是硬实时响应保证。1 秒宽限期后收到 **0 条有效指令、509 条无效指令**，
  虚拟串口仍有 1990 条反馈。断流期间发布一次 `tracking=false`，恢复后收到
  146 条有效跟踪结果，视频按原参数恢复成功。
- 修复前报告：`record-test/stream-recovery-20261119-152049-670315.json`。
- 修复后报告：`record-test/stream-recovery-20261119-152649-255600.json`。
- 构建/测试记录：`record-test/input-watchdog-build.log`、`record-test/solver-tests-build.log`、
  `record-test/solver-tests.log`；运行日志：`record-test/gpu-watchdog-runtime.log`。

六项完整功能检查再次通过，报告为
`record-test/pipeline-check-20261119-152755-871463.json`。随后独立观察 240.006 秒，
报告为 `record-test/benchmark-20261119-153245-081925.json`：

| 指标 | 本次结果 |
| --- | ---: |
| 处理结果频率 | 30.278Hz |
| 8 个 30 秒窗口处理率 | 30.233～30.331Hz |
| 跟踪消息数 / 有效跟踪数 | 7268 / 7268 |
| 图像到跟踪中位延迟 / P95 | 24.175 / 38.753ms |
| 最大跟踪结果间隔 | 116.819ms |
| 非递增检测/跟踪时间戳 | 0 / 0 |
| 组件进程 RSS 起点 / 终点 | 502.24 / 512.54MiB |

测试期间视频循环一次，未新增 TF 错误或超时告警；日志里那一次超时来自此前有意
卸载视频的断流实验。RSS 起点到终点增加约 10.3MiB，最后两段基本持平；4 分钟观察
不足以排除缓慢泄漏。此次不订阅大图像，不能直接与前一轮 20 秒有图像观测的
CPU/GPU 数据混作同条件比较，也不能把约 30Hz 的限帧视频结果当成模型吞吐上限。

本节为主迁移外的补充验证，改动未提交。后续工作按用户优先级回到 YOLO 的
CUDA/TensorRT 依赖、GPU 推理与预处理性能；不继续扩展外围控制逻辑。

## 12. YOLO GPU 硬件核对与 CUDA Graph

### 12.1 当前究竟运行在哪个设备

本机 `/proc/device-tree/model` 为 Orin Nano Super，架构 `aarch64`；CUDA 驱动实际查询
计算能力为 **8.7**，编译参数包含 `compute_87`、`sm_87`。容器 CUDA 12.6.68、
TensorRT Python 10.3.0，系统包 10.3.0.30；使用本机生成的 `0526-fp16.engine`。
因此 YOLO 主网络已在 NVIDIA GPU 上运行，不能把仍有 CPU 占用理解成模型回到了 CPU。
原 OpenVINO CPU 路径仅保留作对照/回退；PnP、跟踪、视频解码等仍有 CPU 工作。

### 12.2 本次只针对 YOLO GPU 路径的改动

| 文件 | 改了什么 | 目的与原理 |
| --- | --- | --- |
| `src/rm_auto_aim/armor_detector/include/armor_detector/tensorrt_inference.hpp` | 构造器可选择 CUDA Graph，增加实际捕获状态查询 | 可明确比较普通 GPU 提交与执行图，测试必须证明图已捕获 |
| `src/rm_auto_aim/armor_detector/src/tensorrt_inference.cpp` | 首帧正常预热，随后捕获固定页内存传输、CUDA 归一化、TensorRT 和回传；后续调用复用图；捕获失败保留普通 GPU 提交，析构前释放图 | 减少 CPU 逐个提交小内核的开销。输入内容每帧更新，地址、形状和上下文固定，结果只同步一次 |
| `src/rm_auto_aim/armor_detector/src/yolov5_detector.cpp` | TensorRT 默认尝试 CUDA Graph；`RM_TENSORRT_CUDA_GRAPH=0` 可关闭，非法值报错 | 提供同一后端的性能对照与排障开关，不改变模型解析或切换到 CPU |
| `src/rm_auto_aim/armor_detector/test/test_yolo_backends.cpp` | 新增黑图、随机图、彩色图的执行图回放对比 | 验证每次使用当前输入，避免错误地重放旧图像；断言图真实捕获，原始输出最大差不超过 0.001 |
| `src/rm_auto_aim/armor_detector/test/benchmark_tensorrt.cpp` | 独立测量两种 GPU 提交方式，交替测 200 次并输出 JSON | 与 30Hz 视频限帧解耦，测量实际后端调用延迟，降低测试顺序偏差 |
| `src/rm_auto_aim/armor_detector/CMakeLists.txt` | 仅在测试与 TensorRT 开启时构建后端性能工具 | CPU 环境不增加 CUDA 依赖，也不将性能数值作为易波动的单元测试断言 |
| `docs/jetson_gpu.md`、`docs/jetson_changes.md` | 补充 GPU 使用状态、图执行原理、开关和实测边界 | 区分 GPU 网络执行、CPU 周边工作与整链路帧率 |

CUDA Graph 遵循 TensorRT 的预热、固定上下文与缓冲区约束，参见
[NVIDIA TensorRT 10.3 开发指南](https://docs.nvidia.com/deeplearning/tensorrt/archives/tensorrt-1030/pdf/TensorRT-Developer-Guide.pdf)。
捕获失败只回到普通 TensorRT GPU 提交；常规推理及同步错误仍会报告，不静默切到 CPU。
本轮不重建 engine，模型权重和共享 YOLO 解析保持不变。

### 12.3 测量与验证

暂停视频主程序后，使用 `trtexec` 分别预热 1 秒、采样 5 秒，禁用输入输出传输，
仅比较 engine 执行。GPU 未锁频，桌面仍运行，工具也提示结果存在波动。

| engine / 提交方式 | 平均 GPU 计算耗时 | 平均 CPU Enqueue 耗时 |
| --- | ---: | ---: |
| FP32 普通提交 | 9.727ms | 2.201ms |
| FP16 普通提交 | 3.487ms | 1.720ms |
| FP16 CUDA Graph | 3.030ms | 0.017ms |

以上不含图像解码、缩放、预处理、数据传输及解析，不能将 qps 当作实际自瞄 FPS。
日志和原始计时位于 `record-test/trtexec-{fp32,fp16,fp16-graph}.{log,json}`。

另用正式 C++ 后端进行配对测试：40 次交替预热后，各测量 200 次，包含主机拷贝、
H2D、CUDA 归一化、TensorRT、D2H 和同步，排除解码、缩放及 YOLO 解析：

| 正式后端路径 | 中位耗时 | P95 |
| --- | ---: | ---: |
| 普通 GPU 提交 | 3.373ms | 6.126ms |
| CUDA Graph | 2.973ms | 6.069ms |

中位耗时减少约 0.40ms（11.8%），P95 改善很小；不宣称整链路同比提速。
报告：`record-test/cuda-graph-backend-fp16.json`。FP32、FP16 的 5 项后端回归均通过，
包括执行图切换输入的实际回放；CUDA 预处理和传统检测测试也通过，无 GPU 跳过。
构建及测试日志：`record-test/cuda-graph-build-final.log`、`record-test/cuda-graph-tests.log`、
`record-test/cuda-graph-fp32.xml`。

CUDA Graph 正式视频链路重新预热 3 秒、观察 20.003 秒，含大图像订阅：处理与跟踪
29.646Hz，跟踪中位延迟 22.837ms、P95 29.794ms，未出现非递增时间戳。
报告：`record-test/benchmark-20261119-154332-242151.json`。视频仍限帧约 30Hz，
与第 10 节测量的系统负载不完全一致，不据此量化整链路的严格加速比。
当前后台程序使用本节新版本，日志为 `record-test/gpu-cuda-graph-runtime.log`，
需以这里为准，不再以第 10 节旧日志判断当前是否在运行。

CUDA Graph 版本六项软件链路复验全部通过，报告为
`record-test/pipeline-check-20261119-154510-793174.json`；运行节点实查后端为
`tensorrt`，模型为 `/ros_ws/record-test/models/0526-fp16.engine`，日志确认图执行已启用。

## 13. 接相机前的代码检查

### 13.1 本轮修复

本轮在已有 GPU 改动之上检查取帧和相机节点生命周期，新增以下修复：

| 文件 | 问题与处理 |
| --- | --- |
| `src/rm_utils/src/driver/HikCamera.cpp` | 原 RGB 图像分配 `宽×高×3` 字节，却向 SDK 声明了更大的输出容量；现在传入实际分配大小，并检查转换返回值和输出长度 |
| 同上 | SDK 转换失败、截断 Bayer 数据和非法尺寸不再作为有效图像返回；每次调用先清空旧图像与时间戳，取得 SDK 缓冲区后通过作用域回收覆盖失败与异常路径 |
| 同上 | OpenCV 只处理支持的 8 位 Bayer 格式，其他格式交给 SDK 转换；避免将 `RGB8Packed` 等格式用于 Bayer 查表时抛异常。四种 Bayer 排列保留原 RGB 语义，并用已知颜色样本核对 |
| 同上 | 设备枚举失败和序列号读取失败不再继续使用未确认的结果 |
| `src/rm_utils/include/rm_utils/driver/HikCamera.hpp` | 曝光中点偏移改为原子快照，取帧线程不再与在线曝光更新并发读写同一个普通数值；补齐直接取帧的输出和等待契约 |
| `src/rm_utils/include/rm_utils/vision_state_machine.hpp`、`src/rm_utils/src/vision_state_machine.cpp` | 增加所属节点释放相机资源的接口；在节点销毁前清理 SDK 句柄和标定服务，清除旧节点指针，允许重新初始化；拒绝另一个节点重复占用，并与切换逻辑互斥 |
| `src/rm_hardware_driver/hik-camera/src/hik_camera_node.cpp` | 在线曝光/增益参数名对齐实际声明的 `camera_6mm.*` 等参数；先校验整批参数再应用修改，拒绝更新非当前相机；初始化失败和退出时释放自己持有的相机资源 |
| `src/rm_auto_aim/armor_detector/src/armor_detector_node.cpp` | 直连模式检查相机初始化结果，失败时明确报错并清理资源；采集线程同时检查节点上下文和退出标志，避免卸载组件时只等待全局 ROS shutdown；析构前释放相机及标定服务 |
| `src/rm_utils/test/test_hik_camera.cpp` | 扩展模拟 SDK 测试，覆盖转换容量、输出所有权、失败清空、缓冲区归还、Bayer RGB 顺序、序列号失败和曝光偏移更新 |
| `src/rm_auto_aim/armor_detector/test/test_camera_lifecycle.cpp` | 新增模拟相机生命周期测试，在 ROS 上下文仍运行时连续创建/销毁检测节点两次，并拒绝第二个节点重复占用；测试不访问真实相机 |
| `docs/自瞄上车测试步骤.md` | GPU 构建显式保留 `ENABLE_TENSORRT=ON`，GUI 重建容器保留第三个 GPU Compose 文件，更新后端说明和实际调参命令 |

参数批量校验仅覆盖软件可判断的条件，不保证设备接受某个曝光/增益数值；设备范围、
写入结果和实际成像仍需核对 SDK 日志。相机时间戳仍沿用原来的毫秒解释及半曝光偏移，
本轮未凭模拟数据认定真实 SDK 时间戳已与 ROS/姿态时间对齐。

新增退出测试首次运行时，gtest 断言通过但整个测试进程以 `-11`（段错误）退出：
状态机单例仍持有已销毁节点的相机标定服务，资源一直留到进程静态析构。
因此本轮补齐了所属节点的显式资源释放；验收必须同时检查测试断言和进程退出码。

### 13.2 接相机后的验收范围

这些修复可以提前完成，但不能在无相机时证明整套硬件已可用。接相机后按
[上车步骤](自瞄上车测试步骤.md) 先独立预览，再使用真实相机和虚拟电控验证：

- 确认 USB 采集、实际像素格式、RGB 颜色、曝光与采集频率。
- 确认工作分辨率与内参标定一致；现有内参文件不能当作新相机已经标定的证据。
- 核对图像时间戳、TF 时间差、静态测距和运动场景；不要用当前时间替换采集时间来掩盖偏差。
- 验证真实拔插、相机超时、进程退出和重启；软件卸载视频组件的测试不能替代 USB 相机恢复。
- 相机识别通过后再接真实串口，核对姿态、协议和外参，最后验证云台闭环。

当前配置继续保留视频与虚拟电控。切换相机时把 Jetson 覆盖文件的 `video_player`
改为 `false`，保留 `use_state_machine_camera: false` 和 `virtual_serial: true`，
先走独立相机节点发布图像的路径。

### 13.3 本轮验证与运行版本

- 在目标容器中独立构建 `rm_utils`、`armor_detector`、`hik_camera`，开启测试及
  TensorRT，三个包全部构建成功。产物位于 `record-test/pre-camera-build/` 和
  `record-test/pre-camera-install/`；构建日志位于 `record-test/pre-camera-log/`。
- 最终运行 9 个测试程序、**39 个 gtest 用例**，全部通过，原始 XML 中没有失败、错误
  或跳过。包括 16 项模拟 SDK 测试、1 项相机节点生命周期测试、2 项弹道补偿测试、
  7 项检测/CUDA/后端测试，以及 13 项已有解算测试。
- GPU 回归显式设置本机 `0526-fp16.engine` 和 `test.mp4`，实际执行 CPU/GPU 最终
  检测对比、视频抽帧和 CUDA Graph 输入切换检查；本轮未重新做 FP32 engine 回归
  或性能基准。测试进程使用 `ROS_DOMAIN_ID=73`。
- 解算测试复用 `build/armor_solver` 中已有程序，并加载本轮独立构建的 `rm_utils`
  复测。CTest 记录分别位于两个独立构建包及原解算构建包的 `Testing/Temporary/`，
  原始 gtest XML 位于各包 `test_results/`。同时检查 CTest 退出码，不能只看 XML。

本轮未替换正式 `install/`，也未重启第 12 节的后台视频程序。接相机前先停止旧程序，
在容器内将源代码构建到正式工作区后再启动：

```bash
cd /ros_ws
ENABLE_TENSORRT=ON bash scripts/jetson/build.sh
source install/setup.bash
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

相机上线时的输入开关按第 13.2 节修改；不要让独立相机预览和主程序同时打开同一设备。
