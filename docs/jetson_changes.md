# Jetson 适配改动说明

首次记录：2026-11-17；更新日期：2026-11-19。适配分支：`infantry-jetson-Lonesome-George`。

本文记录本次实际修改、参数变化及验证范围。测试和部署步骤见
[测试与 Orin Nano 部署手册](jetson.md)。

第 2～6 节汇总适配实现；[第 9 节](#9-2026-11-19-本次工作区改动与验收)
单独记录相对提交 `71460d6` 的本次工作区改动，避免把此前已有的 TensorRT 后端算作本次新增。

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

当前默认路线是：ARM64 容器编译，使用已有 OpenVINO IR 模型进行 CPU 流程验证。
TensorRT 后端已增加到代码中，但 GPU 启用仍需要对应 ONNX、在目标设备生成的
engine，以及容器内匹配的 CUDA/TensorRT 开发环境。

截至 2026-11-19，本机 ARM64 Docker 镜像和容器已建立，14 个包构建成功，CPU
自动测试通过，视频与虚拟电控输入到云台指令输出的六项功能检查通过。当前没有接入
真实相机和电控；TensorRT GPU 推理、硬件闭环及实时性能均未完成验收。

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

当前缺少 `0526` 装甲板模型对应的原始 ONNX/训练权重；现有 `lenet.onnx` 和
`mlp.onnx` 属于数字分类模型，不能作为它的替代品。

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
runtime 并不等于容器已经具有 CUDA Toolkit 和 TensorRT 开发包。GPU 构建需要
显式提供已验证匹配的 `JETSON_BASE_IMAGE`；本次尚未完成该镜像的目标设备验证。

部署构建设置 `BUILD_TESTING=OFF`，原因是当前分支 `armor_solver/CMakeLists.txt`
引用了不存在的 `test/test_solver_motion_switches.cpp`。本次没有补写这个无关测试；
检测包测试可以单独开启，见操作手册。

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

截至本次记录，CUDA/TensorRT 性能适配仍处于方案阶段，未进行对应代码修改。
当前仅有 `0526.xml/.bin`，缺少对应原始 ONNX、训练权重及导出代码；IR 转 ONNX
的可行性与数值一致性也尚未验证。后续先确认模型来源或转换可行性，再接通 GPU
环境、验证结果一致性并优化延迟。真实相机、电控和云台闭环仍需硬件接入后验收。

本次提交名：`完成 ARM64 CPU 部署与视频自瞄链路验收-Lonesome-George`。
