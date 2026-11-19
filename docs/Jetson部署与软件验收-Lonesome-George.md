# Jetson 部署与软件验收 - Lonesome George

更新：2026-11-19。适用当前 Orin Nano Super、JetPack 6.2.3，默认 TensorRT FP16。本文合并原 `jetson.md`、`jetson_gpu.md`，只保留部署与软件验证；接硬件看 [上车验收流程](自瞄上车验收流程-Lonesome-George.md)。

版本、路径和默认配置看 [当前环境总览](Jetson当前环境总览-Lonesome-George.md)；已做改动和测试数据看 [改动与验证记录](Jetson改动与验证记录-Lonesome-George.md)。

## 1. 确认宿主机环境

在 Jetson 宿主机的仓库根目录执行，不是在其他电脑或容器中：

```bash
uname -m
cat /etc/nv_tegra_release
docker compose version
docker info --format '{{json .Runtimes}}'
```

应为 `aarch64`、L4T 36.5.2，Docker runtime 包含 `nvidia`。权限不足时，对宿主机 Docker 命令加 `sudo`。缺少 runtime 时先安装与现有 JetPack 匹配的 NVIDIA Container Toolkit；已有 `nvidia-ctk` 后可配置：

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

重启 Docker 会中断其他容器，必须先停测试。不要为了缺依赖直接升级 JetPack。已有 `rv_devel_` 时，先用 `docker inspect rv_devel_` 确认 `/ros_ws` 挂载的是当前仓库，不要误删别人的开发容器。

## 2. 创建 GPU 容器

先停止旧自瞄程序，在宿主机仓库目录执行。首次先构建 CPU 基础镜像，再构建 GPU 镜像：

```bash
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml build rv_devel
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml \
  -f docker-compose.jetson.gpu.yaml build rv_devel
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml \
  -f docker-compose.jetson.gpu.yaml up -d --no-build rv_devel
docker exec -it rv_devel_ bash
```

已有镜像且 Dockerfile、Compose 未变时，不必重复 build。容器镜像应为 `qyg-vision:jetson-jp6-gpu`；重建会丢失容器可写层中的临时安装，但保留挂载的仓库。日常可用本机 `rv` 别名进入容器。

网络失败时检查 Docker daemon 拉镜像代理与 Compose 构建代理，它们不是同一层。Compose 会读取 `HTTP_PROXY`、`HTTPS_PROXY`，使用 `sudo` 时需显式保留；地址以实际监听为准，不照搬旧端口。OpenVINO 下载失败可将 Dockerfile 指定的安装包放入 `scripts/jetson/openvino-cache/`，构建仍会校验哈希。

## 3. 生成 engine 并编译

以下均在容器内。首次部署、模型更新或设备/CUDA/TensorRT 变化时生成模型：

```bash
cd /ros_ws
bash scripts/jetson/prepare_tensorrt.sh
ENABLE_TENSORRT=ON bash scripts/jetson/build.sh
```

转换脚本先验证现有 IR 转 ONNX 的样本结果，再生成 FP32/FP16 engine；首次需要联网和较长构建时间。确认 `record-test/models/0526.conversion.json` 的 `passed=true`，且两个 engine 与元数据存在。生成物不提交 Git，不直接跨设备复制。

构建默认同时处理 1 个包、包内 2 个任务，关闭测试；只限制编译，不限制运行帧率。源码变更后要停程序再编译，Git 提交不会更新运行进程。

## 4. 准备每个容器终端

每个新容器终端执行：

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
source /opt/openvino/setupvars.sh
source install/setup.bash
export ROS_DOMAIN_ID=0
export RM_VISION_PROFILE=jetson
export RM_VISION_INFERENCE_BACKEND=tensorrt
```

当前 GPU Compose 已设置后两项，这里显式写出便于核对。`source` 只作用于当前 shell；不要同时启动两套主程序。

## 5. 验证视频链路

在 `src/rm_bringup/config/jetson/launch_params.yaml` 中确认以下值，保留其他开关：

```yaml
video_player: true
virtual_serial: true
use_state_machine_camera: false
replay: false
rune: false
record: false
auto_record: false
```

视频为 `/ros_ws/test.mp4`，30Hz 循环；模式必须与画面里的装甲板颜色一致。配置更改后重建 `rm_bringup`、重新 source，再启动：

```bash
colcon build --symlink-install --packages-select rm_bringup
source install/setup.bash
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

另开按第 4 节准备好的终端：

```bash
ros2 param get /armor_detector yolo.backend
ros2 param get /armor_detector yolo.model
python3 scripts/jetson/check_pipeline.py
python3 scripts/jetson/benchmark_pipeline.py --label tensorrt-fp16
```

**通过：** 实际后端为 `tensorrt`、路径为本机 FP16 engine，六项链路检查通过，性能报告有有效结果。图像、跟踪与云台命令频率含义不同；30Hz 视频不能测出 GPU 吞吐上限。观测工具会增加开销，比较时保持订阅条件一致。

## 6. 断流与持续运行

仅在视频＋虚拟电控主程序运行时执行，不用于真实相机联调：

```bash
python3 scripts/jetson/check_stream_recovery.py
python3 scripts/jetson/benchmark_pipeline.py \
  --label gpu-stability --duration 240 --skip-images --progress-every 30
```

第一条保存视频参数、卸载视频组件再恢复；**通过条件为 `passed=true`、`restored=true`，并且断流宽限期后没有旧目标有效指令。** 当前输入超时为 0.5 秒，恢复后重新确认目标。

第二条观察结果间隔、时间戳和 RSS/Swap，检查循环播放后仍稳定。`--skip-images` 改变观测负载，不能与订阅大图像的结果直接比较；4 分钟稳定不能证明没有慢性泄漏。报告保存到 `record-test/`。

## 7. 修改代码后的回归

停止主程序，先按第 4 节加载环境，再在正式构建中打开相关测试：

```bash
MAKEFLAGS="-j2 -l2" CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --symlink-install --executor sequential \
  --packages-select rm_utils hik_camera armor_detector armor_solver \
  --cmake-args -DBUILD_TESTING=ON -DENABLE_TENSORRT=ON
source install/setup.bash
export ROS_DOMAIN_ID=73
export ARMOR_TENSORRT_ENGINE=/ros_ws/record-test/models/0526-fp16.engine
export ARMOR_TEST_VIDEO=/ros_ws/test.mp4
colcon test --packages-select rm_utils armor_detector armor_solver \
  --ctest-args -R '^test_' --output-on-failure
colcon test-result --test-result-base build --verbose
export ROS_DOMAIN_ID=0
```

检查本轮三个测试包的 CTest 退出状态和原始 gtest XML，不能只看汇总数字；其他包可能留有旧结果。GPU 用例必须实际执行，没有 SKIPPED。相机用例模拟 SDK，不访问真实硬件。

需要 FP32 对照时另行执行：

```bash
ARMOR_TENSORRT_ENGINE=/ros_ws/record-test/models/0526-fp32.engine \
  ARMOR_TEST_VIDEO=/ros_ws/test.mp4 build/armor_detector/test_yolo_backends
```

后端回归对比参考图和视频抽帧的数量、类别、类型、置信度及角点；容差是回归门槛，不是硬件精度保证。测试中出现失败或跳过先排查，不扩大容差掩盖问题。

## 8. CPU 对照与 CUDA Graph 排查

CPU 对照无需换镜像：停主程序，在已准备好的 GPU 容器终端执行：

```bash
export RM_VISION_INFERENCE_BACKEND=openvino
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

恢复 GPU 时停程序、设置 `RM_VISION_INFERENCE_BACKEND=tensorrt` 后重启。若确需恢复纯 CPU 镜像，使用两文件 Compose `up -d --no-build rv_devel`，进入 CPU 容器后以 `ENABLE_TENSORRT=OFF bash scripts/jetson/build.sh` 重编译，不能直接复用 GPU 编译产物。

GPU 日志出现 `CUDA graph enabled` 才说明图已捕获；捕获失败回到普通 TensorRT，不是 CPU。停程序后可设置 `RM_TENSORRT_CUDA_GRAPH=0` 再启动，比较时只改变这一项；恢复时 `unset RM_TENSORRT_CUDA_GRAPH`。已按第 7 节构建测试后，可在主程序停止时测后端：

```bash
build/armor_detector/benchmark_tensorrt record-test/models/0526-fp16.engine
```

该计时包含传输、CUDA 预处理、推理和同步，不含解码、缩放、解析及 ROS 传输，不能当作整链路延迟。

## 9. 接硬件与日常更新

软件通过后转到 [上车验收流程](自瞄上车验收流程-Lonesome-George.md)，异常查 [排障速查](自瞄故障排查速查-Lonesome-George.md)。先相机＋虚拟电控，后真实通信和无发射跟随，不在本文重复标定与闭环步骤。

| 改动 | 需要做什么 |
| --- | --- |
| 仅文档 | 不必编译或重启 |
| bringup 配置、内参 | 停程序，重建 `rm_bringup`、source、重启 |
| C++、CUDA、构建选项 | 停程序，重编译、source、重启 |
| Dockerfile、依赖、Compose | 停程序，重建对应镜像/容器，再编译；临时安装可能丢失 |
| 模型、设备、CUDA/TensorRT | 重新生成 engine 并做精度回归 |

不要提交 engine、构建目录、运行日志和本机报告；保存关键结果、参数和标定文件。操作失败先保留完整报错、提交号与配置，不连续叠加修改。
