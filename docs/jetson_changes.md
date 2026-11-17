# Jetson 适配改动说明

记录日期：2026-11-17。适配分支：`infantry-jetson-Lonesome-George`。

本文记录本次实际修改、参数变化及验证范围。测试和部署步骤见
[测试与 Orin Nano 部署手册](jetson.md)。

## 1. 目标与当前状态

目标设备由用户提供的终端输出确认：

| 项目 | 环境 |
| --- | --- |
| 设备 | NVIDIA Jetson Orin Nano Developer Kit Super，用户确认 8GB |
| 系统 | 用户确认 Ubuntu 22.04，AArch64 |
| JetPack | `6.2.3+b81` |
| L4T | `36.5.2` |
| TensorRT | `10.3.0.30` |

当前默认路线是：ARM64 容器编译，使用已有 OpenVINO IR 模型进行 CPU 流程验证。
TensorRT 后端已增加到代码中，但 GPU 启用仍需要对应 ONNX、在目标设备生成的
engine，以及容器内匹配的 CUDA/TensorRT 开发环境。

截至本文整理时，修改保留在本地工作区，尚未提交、推送或部署到 Orin。

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
| `Dockerfile` | ARM64 从官方源码构建 OpenVINO 2025.3.0 的 CPU 路径，关闭 Intel GPU/NPU、样例和测试 |
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
| `launch_params.yaml` | 关闭打符、录像、回放和自动标定；使用物理相机、物理串口 |
| `armor_detector_params.yaml` | YOLO + OpenVINO CPU，加载已有 IR；检测器 `debug: false` |
| `camera_driver_params.yaml` | 两个相机均开启 30Hz 限帧 |

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

## 7. 验证记录与边界

| 检查 | 已有结果 | 能说明什么 |
| --- | --- | --- |
| 当前 x86 Docker 完整部署构建 | 14 个包通过 | 当前平台可编译，不能替代 ARM64 构建 |
| 原有传统检测测试 | 通过 | 参考图片上的传统检测回归通过 |
| 新增 CPU 测试 | 2 项通过 | 后端检查、现有 IR 加载和基本推理通过 |
| GPU 对比测试 | 跳过 | 尚未验证 GPU 推理或 CPU/GPU 结果一致性 |
| TensorRT 10.3 官方头文件语法编译 | 通过 | C++ 接口兼容性检查，不含 GPU 链接或执行 |
| SDK 静态检查 | ARM64 架构正确，包含源码使用的 15 个相机 API | 不代表已经打开真实相机 |
| 配置检查 | Compose、普通/Jetson 参数覆盖通过 | 不代表硬件通信成功 |
| 用户当前电脑测试截图 | 汇总为 0 errors、0 failures | CPU 自动测试完成，未接相机 |

`colcon test-result` 汇总可能包含 CTest 包装项，并可能没有正确统计 gtest 的跳过
项目。应查看测试明细中的 `result="skipped"` 或直接运行测试程序；不能仅凭
`6 tests, 0 skipped` 判断 GPU 已验证。

待完成的目标设备检查：ARM64 镜像构建、海康相机采集、真实串口收发、运行延迟与
资源占用，以及取得对应模型后的 TensorRT engine 构建和 GPU 回归。
