# 测试与 Orin Nano 部署手册

更新日期：2026-11-17。分支：`infantry-jetson-Lonesome-George`。

具体源码、SDK 和参数改动见 [改动说明](jetson_changes.md)。本手册按实际执行顺序
组织：当前电脑自动测试 → 无相机视频测试 → 同步到 Orin → 相机与串口测试 → GPU
验证。所有构建和 ROS 运行命令在 Docker 内执行，宿主机命令另行标注。

## 1. 当前进度与容器约定

当前电脑已经完成一次自动测试，截图显示 0 errors、0 failures。无需相机即可进行
这些测试。测试结束会退出测试进程，不会保留常驻 ROS 节点；`ros2 node list` 和
`ros2 param get` 只查询已经运行的节点，不负责启动程序。

| 用途 | 容器名 | 说明 |
| --- | --- | --- |
| 当前电脑测试 | `qyg_test` | 显式挂载当前仓库，避免测到其他代码目录 |
| Orin Compose 部署 | `rv_devel_` | 由主 Compose 与 Jetson 覆盖配置共同创建 |

测试、启动和查询应使用同一套代码及容器。宿主机可以检查实际挂载：

```bash
docker ps --format '{{.Names}} {{.Image}}'
docker inspect qyg_test --format '{{range .Mounts}}{{if eq .Destination "/ros_ws"}}{{.Source}}{{end}}{{end}}'
```

如果要检查部署容器，把第二条中的 `qyg_test` 换成 `rv_devel_`。此前检查到当前
电脑的 `rv_devel_` 挂载过另一个 `QD_Vision2026` 目录，不能仅凭容器名判断代码来源。

## 2. 当前电脑：编译与 CPU 自动测试

### 2.1 创建或进入测试容器

在当前电脑宿主机进入仓库实际目录，例如：

```bash
cd ~/RM_Vision/QYG_QDVision2026
docker run --rm -it --name qyg_test \
  -v "$PWD:/ros_ws" \
  -w /ros_ws \
  --entrypoint /bin/bash \
  slirute/qidian:latest
```

如果 `qyg_test` 已经在运行，不要重复创建，在另一个宿主机终端执行：

```bash
docker exec -it qyg_test bash
```

该临时容器退出最后的主 shell 后会删除，仓库挂载目录中的文件会保留。这里无需
挂载相机或串口设备。

### 2.2 编译与执行测试

以下在 `qyg_test` 容器内执行：

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
bash scripts/jetson/build.sh

source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --symlink-install \
  --packages-select armor_detector \
  --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=ON -DENABLE_TENSORRT=OFF

source install/setup.bash
colcon test --packages-select armor_detector \
  --ctest-args -R '^test_(detector|yolo_backends)$' --output-on-failure
colcon test-result --verbose
```

第一步完整构建默认关闭测试，避免当前 `armor_solver` 引用缺失的
`test/test_solver_motion_switches.cpp` 导致配置失败；之后仅对检测包开启测试。

预期：原传统检测测试与新增 CPU 测试通过，未提供 engine 的 GPU 对比项跳过。
`1 package had stderr output` 单独出现时不能判断失败，要看具体内容、退出状态和
测试结果；当前环境会输出 ament 头文件安装目录提示。

需要确认跳过情况时，在容器内直接运行：

```bash
build/armor_detector/test_yolo_backends
```

应能看到 `TensorRtMatchesCpuOnReferenceImage` 为 `SKIPPED`。汇总中的
`6 tests, 0 skipped` 不能证明 GPU 已执行，汇总可能包含 CTest 包装项。

## 3. 当前电脑：无相机、无电控的视频测试

这一步需要一段 OpenCV 可读取的装甲板视频。没有视频时，可以先停在第 2 节；
自动测试已经能够验证基本 CPU 路径。视频测试用于检查常驻节点、图像输入和算法
链路，虚拟串口不能验证真实电控通信。

### 3.1 准备视频与测试参数

将视频放入仓库根目录的 `record-test/armor.avi`；该目录匹配现有 `record*/` 忽略
规则。容器中的绝对路径为 `/ros_ws/record-test/armor.avi`。

编辑 `src/rm_bringup/config/jetson/launch_params.yaml` 中的这几个键，保留其他键：

```yaml
use_state_machine_camera: false
video_player: true
virtual_serial: true
replay: false
rune: false
record: false
auto_record: false
foxglove_bridge: false
```

CPU 检测设置沿用 `config/jetson/armor_detector_params.yaml` 中的 `openvino`、
`CPU`、`0526.xml/.bin`。视频参数来自通用文件；将
`src/rm_bringup/config/node_params/video_player_params.yaml` 调整为：

```yaml
/**:
  ros__parameters:
    camera_info_url: package://rm_bringup/config/camera_info.yaml
    path: /ros_ws/record-test/armor.avi
    frame_rate: 30
    start_frame: 0
    frame_id: camera_optical_frame
    keep_looping: true
```

这里 `frame_rate` 使用整数。视频与内参匹配时才适合评估解算精度；任意视频配上
当前内参只能做流程检查。虚拟姿态来自通用 `virtual_serial_params.yaml`，按参考
视频核对敌方颜色和工作模式。

测试时使用 `RM_VISION_PROFILE=jetson`，所以切换输入源应修改上述 Jetson 覆盖文件。
只修改通用 `config/launch_params.yaml` 会被 Jetson 文件中的同名键覆盖。

### 3.2 终端 A：保持程序运行

在 `qyg_test` 容器内执行；如果之前只编译了检测包，先执行完整构建脚本：

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
bash scripts/jetson/build.sh
source install/setup.bash
export RM_VISION_PROFILE=jetson
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

保持这个终端运行，启动日志应包含 `YOLO backend: openvino`。按 `Ctrl+C` 停止。

### 3.3 终端 B：查询节点与数据

在另一个宿主机终端进入同一容器：

```bash
docker exec -it qyg_test bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 node list
ros2 param get /armor_detector yolo.backend
ros2 param get /armor_detector device
ros2 param get /video_player path
```

预期存在 `/video_player`、`/virtual_serial`、`/armor_detector` 和 `/armor_solver`；
检测后端为 `openvino`，设备为 `CPU`，视频路径为容器内的绝对路径。
视频模式不启动 `/camera_driver`，此时不要查询该物理相机节点的参数。

以下命令逐条运行，持续统计的命令用 `Ctrl+C` 结束后再运行下一条：

```bash
ros2 topic hz /image_raw
ros2 topic hz /armor_solver/armors
ros2 topic echo /armor_solver/target --once
```

验收：持续有图像，正确颜色的目标进入/离开时跟踪状态相应变化，没有持续的图像
或 TF 错误。CPU 检测频率可能低于输入视频频率，命令发布频率也不等于检测频率。
Jetson 配置关闭了检测器 `debug`，`/armor_detector/armors` 可能没有消息；检测结果
通过共享状态机传给解算器，优先观察 `/armor_solver/armors`。

完成后记录视频路径、后端、输入/输出频率和异常日志。下一步部署实物前，将输入
恢复为所需模式；不要把本机视频路径作为 Orin 的正式输入配置推送。

## 4. 将修改同步到 Orin

### 4.1 当前电脑：检查并提交推送

在宿主机仓库目录检查改动；特别确认已恢复临时视频参数：

```bash
git status --short --branch
git diff
```

将这次适配文件加入暂存区。以下路径包含源码、二进制 SDK 和两份文档；提交前
检查暂存区，避免混入其他工作：

```bash
git add .gitignore .dockerignore Dockerfile docker-compose.jetson.yaml README.md \
  docs/jetson_changes.md docs/jetson.md scripts/jetson/ \
  src/rm_auto_aim/armor_detector/ \
  src/rm_bringup/config/jetson/ \
  src/rm_bringup/launch/bringup_SingleProcess.launch.py \
  src/rm_hardware_driver/hik-camera/CMakeLists.txt \
  src/rm_utils/CMakeLists.txt src/rm_utils/cmake/hik_sdk.cmake src/rm_utils/hikSDK/
git diff --cached --stat
git diff --cached
git commit -m "适配 Orin Nano 的 ARM64 SDK、部署配置与 TensorRT 后端"
git push -u origin infantry-jetson-Lonesome-George
```

本手册整理时尚未执行这些提交和推送操作。

### 4.2 Orin：获取源码

在 Orin 宿主机已有仓库中执行：

```bash
git status --short --branch
git fetch origin
git switch infantry-jetson-Lonesome-George
git pull --ff-only
```

如果已有本机配置修改，先保存或提交再拉取，不要用强制重置覆盖。首次下载则在
选定父目录执行：

```bash
git clone -b infantry-jetson-Lonesome-George https://github.com/QYG-Vision/QYG_QDVision2026.git
cd QYG_QDVision2026
```

同步源码、配置及仓库内的 SDK，Orin 自行编译。x86 的 `build/`、`install/`、缓存和
engine 不应作为部署产物复制过去。

## 5. Orin：构建 CPU 验证环境

### 5.1 宿主机检查

在 Orin 宿主机仓库目录执行：

```bash
uname -m
cat /etc/nv_tegra_release
dpkg-query --show nvidia-jetpack nvidia-l4t-core 'libnvinfer*'
file src/rm_utils/hikSDK/lib/arm64/*.so
docker compose version
docker info --format '{{json .Runtimes}}'
```

预期架构为 `aarch64`，SDK 库为 ARM aarch64，Docker runtimes 中包含 `nvidia`。
本次目标环境为 JetPack `6.2.3+b81`、L4T `36.5.2`、TensorRT `10.3.0.30`。
包版本后缀不能单独确认实际 CUDA Toolkit，GPU 阶段还要检查容器内 `nvcc`。

按根 README 在 Orin 宿主机安装相机和串口 udev 规则，再重新插拔设备。
如果 `nvidia` runtime 未配置，需先按设备环境配置 NVIDIA Container Toolkit；
Compose 中的 `runtime: nvidia` 不会自动安装它。

### 5.2 构建与进入容器

确认仓库的 Jetson 输入配置符合接下来的硬件状态。宿主机执行：

```bash
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml config
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml up -d --build
docker exec -it rv_devel_ bash
```

若 `rv_devel_` 已存在，先检查第 1 节的挂载信息，确认旧容器归属后在旧部署目录
停止它，再创建当前部署。不要仅因名字冲突删除不明容器。

默认基础镜像为原生 ARM64 `ubuntu:22.04`，构建时从源码安装 OpenVINO CPU
后端，默认 2 个编译任务；首次下载和编译可能较久。此默认镜像不自动包含
TensorRT/CUDA 开发环境。

容器内执行：

```bash
cd /ros_ws
bash scripts/jetson/build.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
printenv RM_VISION_PROFILE
```

预期最后输出 `jetson`。构建脚本同时编译 1 个包、包内默认 2 个任务，只影响编译
速度和峰值内存，不限制运行时线程。构建成功后，可按第 2.2 节在 Orin 单独开启
CPU 检测测试，验证 ARM64 上的已有模型是否可加载、推理。

## 6. Orin：真实相机和串口

### 6.1 先接相机，使用虚拟串口

连接海康相机，在 `src/rm_bringup/config/jetson/launch_params.yaml` 中设置：

```yaml
use_state_machine_camera: false
video_player: false
virtual_serial: true
replay: false
rune: false
```

其余相机内参、安装外参、序列号和曝光增益，按实际相机核对。当前修改沿用原值，
并未替新硬件重新标定。

在容器终端 A 执行：

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RM_VISION_PROFILE=jetson
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

宿主机终端 B 执行：

```bash
docker exec -it rv_devel_ bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 node list
ros2 param get /armor_detector yolo.backend
ros2 param get /armor_detector device
ros2 param get /camera_driver camera_6mm.acquisition_frame_rate
```

预期有 `/camera_driver`、`/virtual_serial`、检测和解算节点；参数分别为
`openvino`、`CPU`、`30.0`。继续按第 3.3 节观察图像、解算输出和跟踪状态。

验收时分别记录采集频率和算法处理频率，检查画面、目标状态与延迟，连续运行
观察是否出现断流、进程退出、内存持续增长或明显延迟积累。宿主机可运行
`tegrastats` 观察资源占用；CPU 验证通过不代表已达到实战速度要求。

### 6.2 再接真实电控

停止程序，将 Jetson 覆盖配置的 `virtual_serial` 改为 `false`。核对
`config/node_params/serial_driver_params.yaml` 中的实际设备路径和波特率；目前
配置为 `/dev/ttyUSB0`、921600。

本分支采用收发均为 24 字节的 float32 弧度协议，按
[电控对接文档](../infantry-jeson串口协议_电控对接.md) 核对固件。配置文件中残留的
“16 字节”注释不能作为帧布局依据。`rune: false` 原本会让 launch 选择 `hero`
协议实现，不能只看 YAML 中的 `protocol: infantry` 判断最终使用的实现。

重启后，在同一部署容器的查询终端逐条执行：

```bash
ros2 topic hz /serial/receive
ros2 topic echo /serial/receive --once
ros2 topic echo /armor_solver/cmd_gimbal --once
```

验收：真实串口数据持续更新，板端姿态改变时数据对应变化，模式和角度单位正确，
电控能解析视觉发送的数据。出现本地命令话题只证明视觉生成了命令，不能替代
电控接收验证；虚拟串口发布的数据同样不能作为真实串口通过的证据。

## 7. Orin：切换 TensorRT GPU

### 7.1 当前待补条件

1. 取得与 `0526.xml/.bin` 对应的四角点装甲板 ONNX 或可导出的原始权重。
2. 提供在当前 L4T `36.5.2` 环境验证匹配的 GPU 开发镜像，容器内包含 CUDA
   Toolkit、TensorRT 10 开发库、ONNX parser 和 TensorRT Python 模块。

`lenet.onnx`、`mlp.onnx` 是数字分类器，不能替代装甲板 YOLO。模型约定见改动
说明：输入 FP32 `[1,3,640,640]`，输出 FP32 `[1,N,22]`，保留原始 logits 和四角点
含义，不内置额外预处理/NMS。形状相同也不能单独证明模型语义正确。

宿主机装有 TensorRT 不代表普通 Ubuntu 容器内已有完整开发包。GPU 开发镜像应
依据 [NVIDIA NGC 实际标签](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)
和宿主机兼容性选择；本次尚未验证匹配镜像，不假定存在 `r36.5.2` 同名标签。

### 7.2 构建 GPU 开发容器

在宿主机将环境变量设置为已经确认的镜像地址，再重建。下面占位地址需要替换：

```bash
export JETSON_BASE_IMAGE='替换为已验证的JetPack开发镜像地址'
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml up -d --build
docker exec -it rv_devel_ bash
```

容器内检查开发环境：

```bash
nvcc --version
dpkg-query --show 'libnvinfer*' 'libnvonnxparsers*'
python3 -c 'import tensorrt as trt; print(trt.__version__)'
```

预期为匹配设备的 CUDA Toolkit 和 TensorRT 10.x，适配目标为 10.3。只有头文件
或只有 runtime 不够，实际构建需要对应开发库。

### 7.3 编译并生成 engine

以下在目标 Orin 容器 `/ros_ws` 中执行，替换 ONNX 的实际路径：

```bash
cd /ros_ws
ENABLE_TENSORRT=ON bash scripts/jetson/build.sh
python3 scripts/jetson/build_armor_engine.py /实际路径/0526.onnx --fp16
```

默认生成 `src/rm_auto_aim/armor_detector/model/0526.engine`。`--fp16` 允许内部
FP16 计算，I/O 仍为 FP32；如需先做 FP32 基准，可省略此选项。

engine 在当前目标设备和容器内生成，更换 TensorRT/CUDA/设备环境后重新生成，
不把当前电脑的 engine 直接复制过去。engine 文件已被 Git 忽略。

### 7.4 先执行 CPU/GPU 对比测试

容器内执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --symlink-install --packages-select armor_detector --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=ON -DENABLE_TENSORRT=ON
source install/setup.bash

ARMOR_TENSORRT_ENGINE=/ros_ws/src/rm_auto_aim/armor_detector/model/0526.engine \
  colcon test --packages-select armor_detector \
  --ctest-args -R '^test_(detector|yolo_backends)$' --output-on-failure
colcon test-result --verbose
```

检查测试明细，GPU 对比项必须实际运行并通过，不能是 `SKIPPED`。它比较同一张
参考图片的检测数量、类别、类型、置信度和角点，属于基本回归，不替代多场景和
运动目标测试。若失败，应检查模型来源、RGB/归一化/补边、输出布局及 FP16 误差。

### 7.5 切换运行参数并实测

在 `src/rm_bringup/config/jetson/armor_detector_params.yaml` 中修改：

```yaml
    yolo.backend: tensorrt
    yolo.model: package://armor_detector/model/0526.engine
```

其他键保留。`device: CPU` 和 `yolo.weights` 此时仅属于 OpenVINO 配置，不控制
TensorRT。重启主 launch 后，日志应出现 `YOLO backend: tensorrt`，再按第 6 节检查
相机、跟踪、串口，并比较相同输入条件下的吞吐、延迟、资源占用和稳定性。

需要退回已有 IR 时，恢复 `yolo.backend: openvino` 和原 XML 路径，然后重启。

## 8. 恢复相机帧率与记录结果

Jetson 的 30Hz 是首次 CPU 流程验证设置，会影响运行时采集频率。若要恢复原通用
配置的帧率行为，在 `src/rm_bringup/config/jetson/camera_driver_params.yaml` 设置：

```yaml
/**:
  ros__parameters:
    camera_6mm.enable_frame_rate: true
    camera_6mm.acquisition_frame_rate: 100.0
    camera_8mm.enable_frame_rate: false
    camera_8mm.acquisition_frame_rate: 200.0
```

8mm 的 `false` 表示不启用该限帧值，不等于保证实际 200FPS。修改后重启相机节点，
用 `/image_raw` 实测。GPU 速度测试要明确是否仍受 30Hz 输入上限约束。

每轮测试建议记录：

| 项目 | 记录内容 |
| --- | --- |
| 版本 | Git 提交、镜像地址、JetPack/L4T、容器内 CUDA/TensorRT |
| 输入 | 相机或视频、分辨率、采集限帧、曝光和使用的内参 |
| 后端 | OpenVINO CPU 或 TensorRT，模型路径、engine 精度 |
| 结果 | 输入频率、解算输出频率、跟踪变化、延迟与日志 |
| 资源 | CPU/GPU、内存和交换空间占用、温度等实际观测 |
| 结论 | 哪一步通过、哪一步未测、失败日志与复现条件 |

## 9. 常见现象与定位

| 现象 | 检查方向 |
| --- | --- |
| 自动测试通过但 `ros2 node list` 无业务节点 | 测试进程已退出；主 launch 是否还在另一终端运行 |
| 进入容器后查不到预期节点/参数 | 容器名、工作区挂载、ROS 环境、`ROS_DOMAIN_ID=0`、节点是否启动 |
| 无相机时相机初始化失败 | 改用视频 + 虚拟串口；CPU 单元测试不需要启动主 launch |
| 改通用开关但行为未变 | `RM_VISION_PROFILE=jetson` 的同名键覆盖了通用配置 |
| 视频模式查询 `/camera_driver` 失败 | 查询 `/video_player`；该模式没有物理相机节点 |
| `/armor_detector/armors` 没有消息 | 检查检测器 `debug`；默认 Jetson 路径优先观察 `/armor_solver/armors` |
| `unknown or invalid runtime name: nvidia` | 宿主机 NVIDIA Container Runtime 配置 |
| ARM SDK 架构错误 | 检查 `HIK_SDK_ROOT` 是否指向正确 SDK，核对实际 ELF 架构 |
| 找不到 `NvInfer.h`、`nvinfer` 或 CUDA Toolkit | 检查容器内开发包；宿主机已安装不能替代容器内环境 |
| 完整构建报缺失 solver 测试源文件 | 部署用 `BUILD_TESTING=OFF`，按包单独开启检测测试 |
| 有图像但没有跟踪目标 | 检查敌方颜色、工作模式、相机内参、TF 和目标是否满足检测阈值 |
| GPU 测试汇总无错误 | 继续检查 GPU 项是否执行；跳过不能算 GPU 验证通过 |
