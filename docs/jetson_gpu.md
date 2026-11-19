# Jetson TensorRT/CUDA 运行与验收

更新日期：2026-11-19。适用本机 Orin Nano Super 8GB、L4T 36.5.2、CUDA 12.6、
TensorRT 10.3。改动逐项说明与实测结果见 [jetson_changes.md 第 10 节](jetson_changes.md#10-tensorrtcuda-适配与性能验收)。

目前使用视频和虚拟电控。软件链路验收不能代替真实相机、手眼标定、串口与云台闭环测试。
以下每个代码块都是一个步骤；宿主机提示符通常为 `rm@rm-desktop`，容器内为 `root@rm-desktop:/ros_ws`。

## 1. 在宿主机创建 GPU 环境

先用 Ctrl+C 停止正在运行的主程序，再在宿主机仓库目录执行：

```bash
docker image inspect qyg-vision:jetson-jp6 >/dev/null
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml -f docker-compose.jetson.gpu.yaml build rv_devel
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml -f docker-compose.jetson.gpu.yaml up -d --no-build
```

**作用与原理：** GPU 镜像以已构建的 CPU 镜像为基础，安装匹配本机的 NVIDIA 编译器、
TensorRT 开发库和 Python 绑定；第三个 Compose 文件选择 GPU 环境。重建容器不会删除挂载的仓库，
但容器内临时安装的工具会随旧容器移除。若第一条提示镜像不存在，先按 [CPU 部署手册](jetson.md)
构建两文件 Compose 的 CPU 基础镜像。GPU 的三文件命令不能省略第三个文件。

**成功标志：** 镜像构建完成，`rv_devel_` 使用 `qyg-vision:jetson-jp6-gpu`。
Docker 组修改后，旧终端可能仍保留旧权限；新登录会话生效前可对 Docker 命令使用 `sudo`。

```bash
rv
cd /ros_ws
nvcc --version
python3 -c 'import tensorrt as t; b=t.Builder(t.Logger()); print(t.__version__, bool(b))'
```

**作用与原理：** `rv` 通过已有别名进入容器；`nvcc` 检查 CUDA 编译器，Builder 同时检查
TensorRT 模块和 GPU 驱动可用性，仅能 import 不能证明 GPU 可用。
**成功标志：** CUDA 12.6、TensorRT 10.3，最后输出 `True`。
若没有配置 `rv`，使用 `docker exec -it rv_devel_ bash`。

## 2. 转换模型并生成 engine（首次或环境变化时）

在 GPU 容器内执行：

```bash
bash scripts/jetson/prepare_tensorrt.sh
```

**作用与原理：** 在 `record-test/model-tools` 建立转换工具环境，将仓库 `0526.xml/.bin`
转换为 ONNX；移除没有消费者的 MaxPool 索引输出，使其兼容 TensorRT。用原 IR 与 ONNX
在 OpenVINO CPU 上检查参考图、5 个视频位置和黑图的候选角点、置信度、类别及颜色。
通过后，分别生成禁用 TF32 的 FP32 engine 和允许内部 FP16 计算的 engine。
FP16 的输入输出仍是 FP32。只有样本检查通过才进入 engine 构建。

**成功标志：** `record-test/models/0526.conversion.json` 中 `passed` 为 true，
同目录存在 `0526-fp32.engine`、`0526-fp16.engine` 和对应 SHA-256 元数据。
这些是本地生成物，不提交 Git；换设备、CUDA 或 TensorRT 后重新生成。转换检查仅覆盖这些样本，
不能保证所有场景与原训练模型等价。首次安装工具需要联网。首次 engine 构建需要筛选计算方案，可能持续数分钟以上，
FP16 通常比 FP32 更久；保持程序运行并查看构建日志。

## 3. 编译并检查检测一致性

保持主程序停止，在 GPU 容器执行：

```bash
bash scripts/jetson/build.sh
source /opt/ros/humble/setup.bash
source /opt/openvino/setupvars.sh
source /ros_ws/install/setup.bash
MAKEFLAGS="-j2 -l2" CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --packages-select armor_detector --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON -DENABLE_TENSORRT=ON
```

**作用与原理：** 首先完整编译工作区，随后只给检测包打开测试。GPU 镜像默认
`ENABLE_TENSORRT=ON`；构建 CUDA 预处理和 TensorRT C++ 后端，每包最多 2 个编译任务。
**成功标志：** 工作区构建成功，生成三个检测测试程序。

```bash
for precision in fp32 fp16; do
  ARMOR_TENSORRT_ENGINE=/ros_ws/record-test/models/0526-${precision}.engine \
  ARMOR_TEST_VIDEO=/ros_ws/test.mp4 \
    build/armor_detector/test_yolo_backends --gtest_output=xml:record-test/yolo-${precision}.xml || break
done
build/armor_detector/test_cuda_preprocess --gtest_output=xml:record-test/cuda-preprocess.xml
build/armor_detector/test_detector
```

**作用与原理：** 对比 CPU 与两种 engine 的参考图、5 个视频位置和红蓝两种筛选结果，
检查数量、号码、装甲板类型一致，置信度差不超过 0.02，角点差小于 2 原图像素。
另一个测试验证 CUDA 的 RGB/HWC→归一化 NCHW 与 OpenCV 相同，并覆盖线程块末尾边界。
**成功标志：** 全部 OK，GPU 项实际运行且没有 SKIPPED。任何 FP16 检测差异都应先分析，
不能直接扩大容差或忽略失败。视频抽样回归仍需补充真实相机和运动场景。

## 4. 启动视频链路，再量性能

在容器终端 A 执行：

```bash
source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

**作用与原理：** GPU Compose 通过 `RM_VISION_INFERENCE_BACKEND=tensorrt` 给 Jetson
参数追加 GPU 覆盖文件，使用 `0526-fp16.engine`。图像由测试视频提供，姿态和弹速由
虚拟串口提供；PnP、跟踪、弹道和云台指令仍按原有算法运行。
**成功标志：** 日志出现 `YOLO backend: tensorrt`，没有持续 CUDA/TensorRT 或 TF 错误。

另开宿主机终端 B，执行：

```bash
rv
source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 param get /armor_detector yolo.backend
python3 scripts/jetson/check_pipeline.py
python3 scripts/jetson/benchmark_pipeline.py --label tensorrt-fp16
```

**作用与原理：** 参数确认实际后端；功能脚本检查 6 个环节；性能脚本预热 3 秒、
采样 20 秒，记录实际处理频率以及原图时间戳到检测和跟踪消息的延迟。
它们只订阅，不向电控发送指令。报告写入 `record-test/`。
**成功标志：** 后端为 tensorrt、6 项通过、性能报告有有效处理消息及延迟。
云台指令频率由定时器决定，不能当成模型 FPS；30Hz 视频也不代表模型吞吐上限。
同时运行多个图像订阅器会增加复制和序列化开销，比较时保持观测工具一致。

## 5. 回退 CPU 与硬件接入

停止主程序后，在同一 GPU 容器中执行：

```bash
export RM_VISION_INFERENCE_BACKEND=openvino
source /ros_ws/install/setup.bash
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

**作用与原理：** 本次 shell 改用原 IR 和 OpenVINO CPU，GPU 镜像仍保留 CPU 依赖，
无需重建即可排查。**成功标志：** 日志后端为 openvino，`yolo.model` 为原 XML 路径。
打开新 shell 后重新使用镜像默认 GPU 选择。

若要恢复纯 CPU 镜像，停止程序后在宿主机执行两文件 Compose `up -d --no-build`，
并在 CPU 容器里使用 `ENABLE_TENSORRT=OFF bash scripts/jetson/build.sh` 重新编译。
这会替换当前 `rv_devel_`，不能在主程序运行时操作。

接上硬件后的顺序：单独出图 → 内参/手眼标定及坐标方向 → 静态/运动检测和测距 →
真实串口协议核对 → 禁止发射状态下验证跟随 → 受控闭环测试。
详细通过条件沿用 [上车测试步骤](自瞄上车测试步骤.md)。

## 6. 无硬件断流与持续运行测试

保持视频＋虚拟串口的主程序运行，在另一个 GPU 容器终端执行：

```bash
source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
python3 scripts/jetson/check_stream_recovery.py
```

**作用与原理：** 脚本确认是视频和虚拟串口配置，保存视频节点的实际参数，临时卸载
`video_player` 3 秒，再按原参数加载。其他组件和虚拟串口保持运行；脚本自身不发布
云台指令。这样能区分“整个程序停了”和“只有图像输入断了”。异常或验收失败时也会
尝试恢复视频，并在 JSON 报告中记录 `restored` 和错误原因。

解算器使用 `tracker.input_timeout`（默认 0.5 秒）检测连续无有效图像处理结果的情况。
时间从最后一次成功处理观测开始，使用单调时钟；超时后清空目标和跟踪计数，发布
`tracking=false`，输出协议已有的无效指令（`id=0`、`distance=-1`、`fire_advice=false`）。
视频恢复后重新确认目标，不继承中断前的连续检测/丢失计数。此参数修改后需重启节点。
“无效指令”能否让真实电控按预期停止跟随，仍需接板后核对协议实现。

**成功标志：** 报告 `passed=true`、`restored=true`；1 秒尾帧宽限后没有有效目标指令，
虚拟串口继续发布，恢复后再次得到有效跟踪。报告位于 `record-test/stream-recovery-*.json`。
脚本只在无硬件测试配置中执行，不能用它替代实际相机重连测试。

随后可在同一观测终端执行：

```bash
python3 scripts/jetson/benchmark_pipeline.py --label gpu-stability --duration 240 --skip-images --progress-every 30
```

**作用与原理：** 连续观察 4 分钟，覆盖本机视频至少一次循环；每 30 秒输出处理率，
记录最大结果间隔、非递增时间戳及组件进程 RSS/Swap。`--skip-images` 避免额外复制
大图像，因此不能直接把这个结果与此前订阅图像的 CPU/GPU 20 秒结果混作相同条件。
RSS 是主机进程常驻内存，不是 GPU 总内存；短时稳定也不能证明不存在缓慢内存泄漏。

**成功标志：** 各段持续有结果，循环播放后没有超时失效或时间戳倒退；结合报告查看
长间隔、RSS 是否持续增长及运行日志。具体本机结果见 [改动记录](jetson_changes.md)。

如果修改了解算器代码，在停止主程序后于 GPU 容器内验证单元测试：

```bash
source /opt/ros/humble/setup.bash
source /opt/openvino/setupvars.sh
source /ros_ws/install/setup.bash
MAKEFLAGS="-j2 -l2" CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --packages-select armor_solver --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select armor_solver --ctest-args -R '^test_' --output-on-failure
colcon test-result --test-result-base build/armor_solver --verbose
```

**作用与原理：** 编译并运行现有滤波/弹速测试和新增跟踪重置回归，检查中断前的计数
不会污染重建跟踪。**成功标志：** 所有用例通过，无失败、错误或跳过。

## 7. 确认 GPU 执行与比较 CUDA Graph

GPU Compose 默认选择 TensorRT FP16。新版本还会在第一帧正常预热后尝试捕获执行图，
后续复用“传输→CUDA 预处理→TensorRT→回传”，减少 CPU 提交开销；内容每帧更新，
并非重放旧图像。日志出现 `CUDA graph enabled` 才代表已捕获。捕获失败时沿用普通
TensorRT GPU 推理，不会因此改成 CPU。

保持主程序运行，在容器内执行：

```bash
source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 param get /armor_detector yolo.backend
ros2 param get /armor_detector yolo.model
```

**作用与原理：** 查询正在运行的节点，而非仅查看磁盘配置。
**成功标志：** 后端为 `tensorrt`，模型指向 `0526-fp16.engine`。
CPU 仍负责解码、缩放、角点解析、PnP、跟踪等，CPU 占用不代表 YOLO 在 CPU 推理。

若需要比较 GPU 执行方式，先停止主程序，再在已按第 3 节编译测试工具的 GPU 容器执行：

```bash
build/armor_detector/benchmark_tensorrt record-test/models/0526-fp16.engine
```

**作用与原理：** 对普通 GPU 提交与 CUDA Graph 进行交替预热和 200 次测量，包含
传输、归一化、模型和同步。**成功标志：** 显示图捕获成功，并输出两组延迟 JSON。
此结果不包含解码、缩放、YOLO 解析及 ROS 传输，不能代替整链路延迟。

排障时可在启动主程序前设置：

```bash
export RM_TENSORRT_CUDA_GRAPH=0
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

**作用与原理：** 只关闭图复用，仍使用 TensorRT GPU。恢复默认时停止程序，执行
`unset RM_TENSORRT_CUDA_GRAPH` 后重新启动；也可显式设置为 `1`。不接受其他值。
