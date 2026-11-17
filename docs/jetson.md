# Orin Nano 小电脑逐步测试与部署手册

更新日期：2026-11-19。分支：`infantry-jetson-Lonesome-George`。

**本文的编译、自动测试、视频测试、相机测试和串口测试，全部在小电脑上完成。**
大电脑仅用于把新改动提交、推送到 GitHub。具体代码改动见
[Jetson 适配改动说明](jetson_changes.md)。

当前先走 **OpenVINO CPU** 路线，使用仓库已有的 `0526.xml/.bin`。完成基础测试
之后，再处理 TensorRT GPU；当前还缺少对应装甲板 ONNX 和已验证匹配的 GPU
开发镜像，不能直接把 CPU 参数改成 GPU 就算完成加速。

**本机进度（2026-11-19）：** ARM64 镜像与容器已建立，14 个包构建成功，CPU
自动测试通过，视频与虚拟电控到云台指令输出的六项功能检查通过。当前工作区默认
启用视频和虚拟串口，视频为 `/ros_ws/test.mp4`、30Hz；Jetson 下自动启动 Foxglove
关闭，可按[上车测试步骤](自瞄上车测试步骤.md)第 5 节手动启动。真实相机、真实串口
及 TensorRT GPU 尚未验收。
详细结果和本次相对 `71460d6` 的改动见
[本次工作区改动与验收](jetson_changes.md#9-2026-11-19-本次工作区改动与验收)。

## 0. 先理解操作位置和测试顺序

### 0.1 只准备小电脑上的两个终端

| 名称 | 在哪里打开 | 作用 |
| --- | --- | --- |
| 终端 A | 小电脑的 VS Code 终端或系统终端 | 更新代码、创建容器、编译、启动主程序 |
| 终端 B | 小电脑上新开的第二个终端 | 进入同一容器，查询节点、参数和话题 |

一开始，两个终端都在小电脑宿主机上。进入 Docker 后，终端仍属于同一台小电脑，
只是软件环境变成了容器环境。

```text
宿主机提示符示例：rm@rm-desktop:~/rm/QYG_QDVision2026$
容器内提示符示例：root@rm-desktop:/ros_ws#
```

不要只靠主机名区分，因为本项目容器使用宿主机网络。可以输入 `pwd`，再检查：

```bash
test -f /.dockerenv && echo "当前在 Docker 容器内" || echo "当前在宿主机"
```

复制代码块时，不要复制 `$`、`#` 或终端提示符。带反斜杠的多行命令需要整块复制。
`python3 - <<'PY'` 到最后独立一行的 `PY` 也是一整块命令。

### 0.2 本教程验证什么

| 阶段 | 需要硬件 | 通过后能说明什么 |
| --- | --- | --- |
| 第 1～4 步 | 小电脑、网络 | 代码同步完整，ARM64 Docker 环境与工作区能构建 |
| 第 5 步 | 无需相机和电控 | ARM CPU 能加载现有模型并通过基本自动测试 |
| 第 6～8 步 | 无需相机和电控 | 仓库视频、虚拟姿态、检测与解算链路能运行 |
| 第 9～10 步 | 海康相机 | ARM64 SDK 能连接真实相机，图像进入算法 |
| 第 11 步 | 相机、电控、串口连接 | 真实姿态和命令能收发，协议与电控对接 |
| 第 12 步 | 对应 ONNX、GPU 开发环境 | TensorRT 能生成 engine、运行并与 CPU 结果对比 |

**编译通过、CPU 测试通过、相机采集成功、串口对接成功是四个不同结论。**
每一步都看完“通过条件”再继续。失败时先停在当前步骤，保存完整报错，不要跳到
后面反复修改配置。

### 0.3 先知道三件容易混淆的事

- Docker `up -d` 创建并启动开发容器，当前容器启动命令是 `bash`，不会自动运行
  自瞄。自瞄要通过 `ros2 launch ...` 单独启动。
- 自动测试运行结束会退出测试进程；此时查不到常驻业务节点是正常的。
- 编译并发默认 2，影响编译速度和内存；相机限帧影响运行时采集频率，两者独立。

## 1. 确认在小电脑，找到仓库

**作用：** 避免把命令输入到大电脑的 WSL，或在错误目录里拉代码。

**要验证：** 当前是 Orin 宿主机，架构为 AArch64，并且进入了正确仓库。

**操作位置：小电脑终端 A，宿主机。**

```bash
hostname
uname -m
cat /etc/nv_tegra_release
```

三条命令依次查看主机名、CPU 架构、Jetson 系统版本。预期架构为 `aarch64`；
你的设备此前输出 L4T `36.5.2`。如果是 `x86_64` 或提示符仍是 `LAPTOP-...`，
先确认终端确实连接到了小电脑；不要在这里构建 ARM64 部署环境。

按照你此前截图中的小电脑路径进入仓库：

```bash
cd ~/rm/QYG_QDVision2026
pwd
git rev-parse --show-toplevel
```

- `cd`：进入目录。这里的 `~/rm/...` 是小电脑路径，不是大电脑的 `~/RM_Vision/...`。
- `pwd`：显示当前目录。
- `git rev-parse --show-toplevel`：显示当前 Git 仓库根目录。

**通过条件：** 后两条显示的是同一个 `QYG_QDVision2026` 根目录。

**如果目录不存在：** 在小电脑 VS Code 中打开已有仓库，从该窗口新建终端，先
运行 `git rev-parse --show-toplevel` 找到实际路径，再 `cd` 到那个路径。本教程
后面的宿主机命令都在仓库根目录执行。

## 2. 获取适配分支，确认文件下载完整

**作用：** 把已经推到 GitHub 的适配代码同步到小电脑。

**要验证：** 小电脑使用正确远程仓库、正确分支，且包含 SDK、脚本和配置文件。

**操作位置：终端 A，宿主机仓库根目录。**

### 2.1 先查看本地修改

```bash
git status --short --branch
git remote -v
```

预期远程地址是 `https://github.com/QYG-Vision/QYG_QDVision2026.git`。`origin`
是这个远程地址的简称。状态中的 `M` 表示本地文件被改过，`??` 表示未跟踪文件。

你此前的小电脑有一处参数修改，可以这样查看内容：

```bash
git --no-pager diff -- src/rm_bringup/config/node_params/armor_detector_params.yaml
```

`--no-pager` 让结果直接输出，避免进入翻页界面。如果其他 Git 命令进入翻页界面，
底部出现百分比或 `(END)`，按 `q` 返回命令行；那不是下载进度。

**只有在这处本地参数需要暂时移开、以便拉取代码时**，执行：

```bash
git stash push -m "Orin参数_更新前临时保存" -- src/rm_bringup/config/node_params/armor_detector_params.yaml
git stash list
```

这是把指定文件的修改存到 Git 的临时保存区，不是删除，也没有上传。记录这次
保存的条目；如果状态还有其他本地改动，应先检查它们，不能用强制重置清空。
如果没有本地修改，直接跳过这组 `stash` 命令。

### 2.2 获取、切换、更新

```bash
git fetch origin
git switch infantry-jetson-Lonesome-George
git pull --ff-only
```

三条命令的作用分别是：

1. `fetch` 获取远程提交和分支信息，不自动合并当前代码。
2. `switch` 切换到适配分支；首次切换通常会自动建立同名本地分支及跟踪关系。
3. `pull --ff-only` 更新当前分支；如果本地历史和远程分叉，则停止，不自动合并。

如果出现你此前遇到的 `GnuTLS recv error`、`early EOF`、`unpack-objects 失败`，
说明下载没有完整结束。先重试这条临时使用 HTTP/1.1 的命令：

```bash
git -c http.version=HTTP/1.1 pull --ff-only
```

如果仍然断线，停在此处检查小电脑访问 GitHub 的网络或代理，不要继续编译半途
更新的目录，也不要把网络错误当作代码冲突处理。

如果之前本轮执行了 `stash`，更新成功后再恢复那一条。**未执行过临时保存则跳过：**

```bash
git stash list
git stash apply 'stash@{0}'
```

这里假定本轮保存的是最新条目；有多个条目时按之前记录的名称选择。`apply`
恢复后仍保留备份，若提示冲突，先处理冲突再继续。

### 2.3 验证分支和关键文件

```bash
git branch --show-current
git log -1 --oneline
ls docker-compose.jetson.yaml docs/jetson.md scripts/jetson/build.sh
file src/rm_utils/hikSDK/lib/arm64/*.so
```

**通过条件：**

- 分支名为 `infantry-jetson-Lonesome-George`。
- 已取得适配提交。最初适配提交是 `87540e4`，后续文档更新的提交号可能更晚。
- 三个关键文件都能列出；SDK 文件显示 `ARM aarch64`，而不是 `x86-64`。

如果找不到适配文件，可能远程只建立了分支名，没有提交代码；或本次下载仍未
成功。先检查大电脑是否已提交、推送，以及小电脑是否拉取了同一仓库和分支。

## 3. 检查 Docker，创建小电脑开发容器

**作用：** 建立后面所有测试统一使用的 ARM64 软件环境。

**要验证：** Docker 可用、NVIDIA runtime 可用，容器挂载的是当前小电脑仓库。

**操作位置：终端 A，宿主机仓库根目录。**

### 3.1 检查 Docker 与设备版本

```bash
docker compose version
docker info --format '{{json .Runtimes}}'
dpkg-query --show nvidia-jetpack nvidia-l4t-core 'libnvinfer*'
df -h .
```

作用依次是查看 Compose 是否可用、已注册运行时、宿主机 NVIDIA 软件包版本，
以及当前磁盘剩余空间。此前确认过 JetPack `6.2.3+b81`、TensorRT `10.3.0.30`。

**通过条件：** Docker 命令没有连接或权限错误，runtimes 中包含 `nvidia`。
磁盘应有空间容纳镜像和编译产物；遇到 `No space left on device` 时先解决空间，
不要反复重建。

若 Docker 提示权限不足，先用 `sudo docker info` 判断是否只是当前用户权限问题；
需要时，本教程宿主机上的 Docker 命令统一加 `sudo`。容器内不需要这样处理。

### 3.2 仅在缺少 NVIDIA runtime 时处理

如果上一步已包含 `nvidia`，跳过本小节。

先查看是否已有配置工具：

```bash
command -v nvidia-ctk
```

如果有输出路径，执行下面配置。它修改 Docker 的运行时配置，重启 Docker 会
中断其他正在运行的容器，因此应在本次测试启动前完成：

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker info --format '{{json .Runtimes}}'
```

命令及作用依据 [NVIDIA Container Toolkit 官方说明](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker)。

如果 `nvidia-ctk` 没有路径，先尝试使用现有 JetPack 软件源安装：

```bash
sudo apt update
sudo apt install nvidia-container-toolkit
```

安装成功后再运行上面的配置命令。若找不到安装包或存在版本依赖冲突，停在这里
记录错误，按当前 JetPack 的软件源环境处理；不要为了这一步重刷或升级现有系统。

### 3.3 检查是否已有同名容器

```bash
docker ps -a --format '{{.Names}} {{.Image}} {{.Status}}'
```

如果已有 `rv_devel_`，查看它挂载的目录：

```bash
docker inspect rv_devel_ --format '{{range .Mounts}}{{if eq .Destination "/ros_ws"}}{{.Source}}{{end}}{{end}}'
```

如果不存在，`No such object` 只表示尚未创建，直接进行下一小节。
如果挂载的是别的仓库，先确认旧容器用途，并在旧部署目录停止、移除其开发容器
后再创建本部署；不要误以为进入同名容器就进入了当前代码。

如果已经确认它是你不再使用的旧开发容器，并已保存容器内需要保留的文件，可在
宿主机执行以下两条，再进行下一小节：

```bash
docker stop rv_devel_
docker rm rv_devel_
```

`stop` 停止旧容器，`rm` 删除该容器记录和可写层。宿主机挂载仓库保留，但只安装
或只保存在旧容器内部的内容会丢失；若仍需要旧容器，先保留它并处理命名方案，
不要执行这两条。

### 3.4 检查配置并构建镜像

```bash
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml config
```

这条只显示合并后的配置，不启动算法。核对输出里包含：

```text
platform: linux/arm64
runtime: nvidia
RM_VISION_PROFILE: jetson
image: qyg-vision:jetson-jp6
```

Jetson 覆盖使用 ARM64 Ubuntu 22.04 和 ROS 2 Humble `ros-base`，不会安装完整
桌面套件、PlotJuggler 或图形相机标定工具；CPU 检测、Foxglove、相机和串口
运行所需依赖仍会安装。若之后需要标定或 PlotJuggler，可进入容器单独安装：

```bash
sudo docker exec -it rv_devel_ bash
apt update
apt install -y ros-humble-camera-calibration ros-humble-plotjuggler ros-humble-plotjuggler-ros
```

再执行真正的镜像构建和容器创建：

```bash
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml up -d --build
```

- 两个 `-f`：先加载主配置，再加载 Jetson 覆盖配置，不能漏掉后者。
- `--build`：按 Dockerfile 构建开发镜像。
- `-d`：容器在后台运行，不占住这个宿主机终端。

如果使用本机 Clash 代理，并且 Docker 命令需要 `sudo`，用下面这条替代上面的
构建命令（先确认代理实际监听端口；此处按本机 `7897`）：

```bash
sudo env HTTP_PROXY=http://127.0.0.1:7897 HTTPS_PROXY=http://127.0.0.1:7897 \
  NO_PROXY=localhost,127.0.0.1,::1 \
  docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml up -d --build
```

Docker daemon 的 systemd 代理负责拉基础镜像；Jetson Compose 另外将这几个代理
构建参数传入镜像构建过程，并设置 `build.network: host`，让构建阶段的
Git/curl/wget 能访问宿主机回环地址。`sudo` 通常会清除当前 shell 的代理环境，
因此这里显式传入。不要把相同的回环地址直接用于普通桥接网络的容器。
Clash 使用全局模式时，还须确认全局出口选中了实际代理节点；选 `DIRECT` 仍是直连。

首次会下载精简 ROS、项目依赖和约 40 MB 的官方 OpenVINO 2025.3.0 ARM64 预编译包。
Dockerfile 校验压缩包并解压到 `/opt/openvino`，无需在 Jetson 上编译 OpenVINO。
如果 Docker 在下载 OpenVINO 时遇到 TLS 错误，可在宿主机先下载到本地缓存，再重跑构建：

```bash
ov_archive=openvino_toolkit_ubuntu22_2025.3.0.19807.44526285f24_arm64.tgz
curl -fSL --retry 3 \
  "https://storage.openvinotoolkit.org/repositories/openvino/packages/2025.3/linux/$ov_archive" \
  -o "scripts/jetson/openvino-cache/$ov_archive"
```

看到 `Using local OpenVINO archive` 表示构建使用了该文件，仍会校验 SHA-256。
当前 `armor_detector` 仍链接 OpenVINO 并使用 0526 IR；即使启动时关闭打符，完整工作区的
`rune_detector` 仍需要 ONNX 前端。Jetson 镜像按 Dockerfile 的软件包列表安装依赖，跳过
`rosdep update`；以后需要用 `rosdep install` 时，先在容器内运行 `rosdep update`。
重新构建时保留 Docker 缓存，已完成且输入未变化的步骤可直接复用。

**通过条件：** 构建退出成功，并显示容器已启动。继续检查：

```bash
docker ps --filter name=rv_devel_ --format '{{.Names}} {{.Status}}'
```

预期容器为 `Up ...`。如果构建报错，停在当前步骤，根据第一条实际错误排查。
只有 `Running` 容器并不代表自瞄已启动，当前容器主命令仍是 `bash`。

本机宿主机的 `~/.bashrc` 已配置快捷命令：

```bash
alias rv='sudo docker exec -it rv_devel_ bash'
```

每个新开的宿主机 Bash 终端可以直接输入 `rv`，在已有容器中打开交互式 shell；
需要时输入登录密码。修改别名后的旧终端先运行 `source ~/.bashrc`。该别名不会
构建或启动容器，容器必须处于运行状态；它是本机设置，不会随仓库克隆自动配置。

## 4. 进入容器，编译整个工作区

**作用：** 在小电脑本机编译 ARM64 可执行程序和 ROS 包。

**要验证：** 环境、SDK 链接和包间依赖均能完成构建。

**操作位置：终端 A，先在宿主机执行第一条，随后都在容器内。**

```bash
docker exec -it rv_devel_ bash
```

`exec` 在已经运行的容器中打开一个 shell。看到容器提示符后输入：

```bash
cd /ros_ws
pwd
uname -m
printenv RM_VISION_PROFILE
source /opt/ros/humble/setup.bash
bash scripts/jetson/build.sh
```

- `/ros_ws` 是宿主机仓库在容器中的挂载位置。
- `uname -m` 应为 `aarch64`；环境变量应输出 `jetson`。
- `source ...setup.bash` 让当前 shell 能找到 ROS 命令及包。
- 构建脚本同时编译 1 个包，包内默认 2 个任务，默认关闭 TensorRT 编译。

脚本还设置 `BUILD_TESTING=OFF`。原因是当前解算包 CMake 引用了不存在的
`test_solver_motion_switches.cpp`；先编译部署程序，下一步再仅开启检测包测试。
这不等于自动测试已经通过。

**通过条件：** 最后出现 `Summary: ... packages finished`，没有 `Failed` 或
`Aborted`。当前工作区此前完整构建为 14 个包，实际数量以当前源码为准。
`had stderr output` 本身不能判断失败，须查看具体内容；ament 的头文件安装提示
与真正的编译错误应分别判断。

成功后加载本次工作区：

```bash
source /ros_ws/install/setup.bash
ros2 pkg prefix armor_detector
ros2 pkg prefix rm_bringup
```

预期输出位于 `/ros_ws/install/`。如果显示其他目录或找不到包，先检查是否正确
构建并加载了当前工作区。

## 5. 在小电脑运行 CPU 自动测试，不接相机和电控

**作用：** 在真正的 ARM CPU 上验证已有模型能加载、基本推理能执行。

**要验证：** 传统检测参考图片测试、未知后端检查、IR 加载和非法输入检查通过。

**操作位置：终端 A，容器内 `/ros_ws`。**

### 5.1 只给检测包开启测试

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --symlink-install \
  --packages-select armor_detector \
  --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=ON -DENABLE_TENSORRT=OFF
```

这会生成检测包的测试程序。`--packages-select` 只选该包，避免同时开启前面提到
的缺失解算测试；`ENABLE_TENSORRT=OFF` 表示此阶段不依赖 CUDA/TensorRT 开发库。

**通过条件：** 检测包编译成功。若找不到 OpenVINO CPU 插件或模型算子不支持，
记录完整错误；当前电脑上的测试通过不能替代这一步 ARM 实测。

### 5.2 执行测试并看明细

```bash
source /ros_ws/install/setup.bash
colcon test --packages-select armor_detector \
  --ctest-args -R '^test_(detector|yolo_backends)$' --output-on-failure
colcon test-result --verbose
```

第一条运行选中的两个测试程序，失败时输出详情；第二条汇总测试结果。

再直接运行新增的测试程序，明确查看跳过项：

```bash
build/armor_detector/test_yolo_backends
```

**通过条件：**

- `RejectsUnknownBackend` 显示 `OK`。
- `CpuLoadsExistingIrAndRejectsInvalidImages` 显示 `OK`。
- `TensorRtMatchesCpuOnReferenceImage` 此阶段应显示 `SKIPPED`，因为没有提供 engine。
- 原传统检测测试无失败，测试汇总无错误、无失败。

注意：汇总可能包含 CTest 包装项，也可能没有正确统计 gtest 的跳过项。
`6 tests, 0 skipped` 不能证明 GPU 已执行；以具体测试名称的结果为准。

**测试到这里结束后没有常驻自瞄节点是正常的。** 下一步单独启动视频链路，才需要
在终端 B 查询 `ros2 node list`。

## 6. 准备无相机、无电控的视频测试

**作用：** 用文件替代相机，用虚拟串口替代电控，逐步检查常驻运行链路。

**要验证：** 视频能解码、参数正确、算法使用 CPU；尚不验证真实相机或通信。

**操作位置：终端 A，容器内。此时主 launch 应未运行。**

### 6.1 检查仓库自带视频

仓库已经跟踪了根目录 `test.mp4`，无需先从大电脑拷视频。执行：

```bash
cd /ros_ws
ls -lh test.mp4
python3 -c 'import cv2; print(cv2.__version__)'
```

如果第二条报 `No module named cv2`，在这个容器内安装诊断用 Python OpenCV：

```bash
apt update
apt install -y python3-opencv
```

它用于下方的视频读取检查。此手动安装保存在当前容器中，重建容器时可能需要
重新安装，不能据此认为 Dockerfile 已包含该 Python 包。

继续执行整块命令，读取第一帧：

```bash
python3 - <<'PY'
import cv2
cap = cv2.VideoCapture('/ros_ws/test.mp4')
print('视频是否打开:', cap.isOpened())
ok, frame = cap.read()
print('首帧是否读取成功:', ok)
if ok:
    print('首帧尺寸:', frame.shape)
    print('视频记录的FPS:', cap.get(cv2.CAP_PROP_FPS))
cap.release()
if not ok:
    raise SystemExit('视频解码失败，先检查文件是否完整、是否支持此编码')
PY
```

**通过条件：** 视频打开和首帧读取都为 `True`，能打印尺寸。文件存在不等于
能够解码。此视频先用于通路检查，若内容没有可识别装甲板，不能要求跟踪为真。

### 6.2 自动备份并设置视频测试参数

下面命令会先将相关配置备份到被 Git 忽略的 `record-test/config-backups/`，然后：

- 开启视频输入和虚拟串口，关闭真实相机、真实串口输入。
- 保持打符、录制、回放和自动标定关闭，避免混入其他输入源或调试任务。
- 明确使用 OpenVINO CPU 和已有 IR。
- 将视频路径设为容器内 `/ros_ws/test.mp4`，播放频率设为 30。

**这里的 30 是视频播放频率，不会修改真实相机的帧率配置。**

在容器 A 整块执行：

```bash
python3 - <<'PY'
from datetime import datetime
from pathlib import Path
import shutil
import yaml

root = Path('/ros_ws')
files = [
    root / 'src/rm_bringup/config/jetson/launch_params.yaml',
    root / 'src/rm_bringup/config/jetson/armor_detector_params.yaml',
    root / 'src/rm_bringup/config/node_params/video_player_params.yaml',
    root / 'src/rm_bringup/config/jetson/camera_driver_params.yaml',
    root / 'src/rm_bringup/config/node_params/serial_driver_params.yaml',
]
backup = root / 'record-test/config-backups' / datetime.now().strftime('%Y%m%d-%H%M%S-%f')
backup.mkdir(parents=True)
for path in files:
    shutil.copy2(path, backup / path.name)

path = files[0]
config = yaml.safe_load(path.read_text())
config.update({
    'use_state_machine_camera': False,
    'video_player': True,
    'virtual_serial': True,
    'replay': False,
    'rune': False,
    'record': False,
    'auto_record': False,
    'foxglove_bridge': False,
    'enable_dynamic_camera_tf': False,
    'enable_pitch_calibration': False,
})
path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))

path = files[1]
config = yaml.safe_load(path.read_text())
config['/**']['ros__parameters'].update({
    'detector': 'yolo',
    'device': 'CPU',
    'yolo.backend': 'openvino',
    'yolo.model': 'package://armor_detector/model/IR/0526.xml',
    'yolo.weights': 'package://armor_detector/model/IR/0526.bin',
})
path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))

path = files[2]
config = yaml.safe_load(path.read_text())
config['/**']['ros__parameters'].update({
    'path': '/ros_ws/test.mp4',
    'frame_rate': 30,
    'start_frame': 0,
    'keep_looping': True,
})
path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
print('配置备份目录:', backup)
print('已设置：视频 + 虚拟串口 + OpenVINO CPU')
PY
```

**通过条件：** 打印备份路径和完成信息，没有 Python 错误。该操作只改配置，不会
改模型、跟踪器或标定数值。保存这个备份路径，恢复方法见第 14 步。

这些开关必须改 `config/jetson/` 的覆盖文件。启用 `RM_VISION_PROFILE=jetson`
时，通用 `config/launch_params.yaml` 中的同名键会被覆盖，只改通用文件可能不生效。

### 6.3 确认安装目录能读到刚改的配置

当前使用 `--symlink-install`，通常可以直接读到修改。先检查：

```bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
python3 - <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import yaml
path = Path(get_package_share_directory('rm_bringup')) / 'config/jetson/launch_params.yaml'
config = yaml.safe_load(path.read_text())
print('实际读取文件:', path)
print('video_player:', config['video_player'])
print('virtual_serial:', config['virtual_serial'])
assert config['video_player'] is True and config['virtual_serial'] is True
PY
```

如果断言失败，或安装目录没有这个配置，重新安装 bringup 包再重复检查：

```bash
colcon build --symlink-install --packages-select rm_bringup --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF
source /ros_ws/install/setup.bash
```

**通过条件：** 安装目录读取的两个开关都为 `True`。后续每次改输入模式时，都要
确认读取的是当前仓库；必要时重复上述 bringup 安装命令。

## 7. 启动视频链路，保持终端 A 运行

**作用：** 真正启动自瞄节点，让输入、检测、解算持续运行。

**要验证：** 节点能初始化，视频能持续供图，CPU 模型可用于实际图像。

**操作位置：终端 A，容器内。**

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
export RM_VISION_PROFILE=jetson
export ROS_DOMAIN_ID=0
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

前两个 `source` 加载 ROS 和当前工作区；`RM_VISION_PROFILE` 选择 Jetson 覆盖配置；
`ROS_DOMAIN_ID=0` 与项目 launch 使用的通信域一致；最后一条才启动业务程序。

**启动后不要关闭终端 A，也不要在它里面继续输入查询命令。** 程序正常时会持续
占用这个终端。下面用终端 B 查询。需要停止时，在终端 A 按 `Ctrl+C`，等提示符
重新出现后再修改模式或重启。

**通过条件：** 节点没有初始化异常，日志包含 `YOLO backend: openvino`，没有
`Video file ... does not exist` 或 `Video file open failed`，程序持续运行。
循环视频到结尾后重新打开属于配置行为。

如果看到物理相机初始化日志或物理串口打开失败，说明实际输入模式仍不正确，
先回到第 6.3 步检查参数来源。

## 8. 用终端 B 验证节点、参数和数据

**作用：** 分层检查“有进程”“有图像”“有算法结果”，而不是只看启动日志。

**要验证：** 查询的是终端 A 启动的同一个容器，参数符合视频测试配置。

### 8.1 打开查询终端

在小电脑新建终端 B，此时它最初在宿主机。执行：

```bash
docker exec -it rv_devel_ bash
```

进入容器后执行：

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 node list
```

**通过条件：** 列表中至少包含 `/video_player`、`/virtual_serial`、`/armor_detector`
和 `/armor_solver`。物理相机模式尚未开启，所以此时没有 `/camera_driver` 是正常的。

如果没有业务节点，先看终端 A 是否还在运行、有无错误，再检查终端 B 是否进入
同一个 `rv_devel_` 容器，是否加载了环境。

### 8.2 验证实际参数

在容器 B 逐条执行：

```bash
ros2 param get /armor_detector yolo.backend
ros2 param get /armor_detector device
ros2 param get /video_player path
ros2 param get /video_player frame_rate
ros2 param get /virtual_serial vision_mode
```

| 查询 | 预期 | 验证目的 |
| --- | --- | --- |
| `yolo.backend` | `openvino` | 当前没有误用 TensorRT |
| `device` | `CPU` | OpenVINO 不会尝试调用 Intel GPU |
| 视频 `path` | `/ros_ws/test.mp4` | 读的是容器内真实存在的视频 |
| 视频 `frame_rate` | `30` | 输入播放频率设置正确 |
| `vision_mode` | 默认 `1` | 当前虚拟模式为自瞄蓝色；`0` 表示自瞄红色 |

如果视频中的装甲板颜色不同，可临时选择检测颜色：

```bash
ros2 param set /virtual_serial vision_mode 0
```

上面改为检测红色；需要蓝色时改为 `1`。这是修改虚拟串口模拟的模式，主要用于
视频和“相机 + 虚拟串口”阶段。它不替代真实电控的模式指令，重启后恢复文件值。

### 8.3 验证图像进入系统

在容器 B 执行：

```bash
ros2 topic hz /image_raw
```

观察约 10 秒，再按 `Ctrl+C`。这条会持续统计频率，不会自动结束。

**通过条件：** 有持续图像消息。接近设置的 30Hz 说明播放器大致按目标频率运行；
实际值受解码、CPU 负载和 CLI 统计开销影响，不能要求完全等于 30。

若始终没有数据，优先看播放器错误、视频解码和终端 A 是否退出。不要先改跟踪器。

### 8.4 验证检测到解算的数据通路

```bash
ros2 topic hz /armor_solver/armors
```

同样观察约 10 秒后按 `Ctrl+C`。这验证解算器处理后的装甲板消息是否持续发布。
它的频率不一定与相机输入相同，也不能直接当成单独的神经网络推理耗时。

再取一条消息查看内容：

```bash
ros2 topic echo /armor_solver/armors --once --qos-reliability best_effort
```

`--once` 收到一条后退出；`best_effort` 与项目传感器消息的 QoS 相容。若等待过久，
按 `Ctrl+C` 回到命令行，并检查终端 A 的 TF、相机内参、模型和处理线程日志。

消息里的 `armors` 可以为空：它可能表示当前帧没有检测到正确颜色的目标，并不
自动代表程序故障。当前检测器 `debug: false` 时，`/armor_detector/armors` 不一定
发布，所以这里观察 `/armor_solver/armors`，内部仍通过共享状态机传递检测结果。

### 8.5 验证目标跟踪与命令生成

```bash
ros2 topic echo /armor_solver/target --once --qos-reliability best_effort
ros2 topic echo /armor_solver/cmd_gimbal --once --qos-reliability best_effort
```

逐条执行。第一条看目标状态，第二条看视觉生成的云台命令。这些输出不意味着
已经通过真实串口发送，当前使用的是虚拟电控输入。

**分两级验收：**

1. 视频解码和消息持续，节点不退出：说明基本运行通路建立。
2. 视频确有可识别装甲板，颜色/模式正确，内参与画面匹配时，观察目标进入/离开
   对应的 `tracking` 状态和合理坐标变化，才能进一步评价检测跟踪。

若根目录 `test.mp4` 内容不适合自瞄，换成同相机录制的装甲板视频，存到仓库
`record-test/armor.avi`。把视频参数 `path` 改成 `/ros_ws/record-test/armor.avi`，
重启主 launch 后重复本步。视频内参与姿态不匹配时，只验证通路，不评判瞄准精度。

### 8.6 一次检查视频与虚拟电控功能链路

保持终端 A 的主 launch 运行，在已经进入同一容器的终端 B 执行：

```bash
cd /ros_ws
source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
python3 scripts/jetson/check_pipeline.py
```

**原理与作用：** 脚本只订阅数据，不发送控制命令。默认观察 20 秒，依次检查图像、
虚拟姿态和有效弹速、`odom` 下的非空装甲板、有效跟踪、有效云台指令，以及成功的
弹道补偿。每项至少收到两条满足条件的消息才通过；可用 `--duration 30` 延长观察。

**通过条件：** 六项均显示 `[通过]`，最后输出功能链路检查通过。JSON 报告保存到
`/ros_ws/record-test/pipeline-check-日期时间.json`，该目录已被 Git 忽略。2026-11-19
本机已完成一次六项通过的验收，具体计数见改动说明。

未通过时先区分“没有收到话题”和“收到消息但没有满足条件的数据”。例如当前视频
没有可识别目标时，图像可能正常，而跟踪和有效指令条件未满足。
本检查不验证逐帧对应、目标丢失和恢复、瞄准精度、实时性能、串口发送或真实云台响应。

## 9. 接真实海康相机，继续用虚拟电控

**作用：** 只把图像来源换成真实相机，先独立确认 ARM64 SDK 和 USB 采集。

**要验证：** 相机能枚举、打开、采集，图像进入同一检测与解算链路。

### 9.1 先停止视频程序

在终端 A 按 `Ctrl+C`，等主 launch 退出。终端 B 的频率统计若仍运行，也按
`Ctrl+C` 停止。修改配置后需要重启节点，本教程不依赖动态更新这些启动参数。

### 9.2 宿主机配置设备权限并接入相机

终端 B 如果仍在容器中，输入：

```bash
exit
```

这只是退出 B 的 `docker exec` shell，不会停止整个开发容器。
在已回到小电脑宿主机的 B 中，按仓库 README 创建相机 udev 规则：

```bash
sudo tee /etc/udev/rules.d/rm_hikrobot.rules >/dev/null <<'EOF'
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules
```

它让系统识别对应 USB 厂商设备时应用访问权限。然后拔插海康相机，再检查：

```bash
lsusb
```

核对是否出现海康设备或厂商 ID `2bdf`。USB 枚举成功仍不等于 SDK 已打开相机，
后面要继续看启动日志和图像消息。

### 9.3 改为相机输入，并恢复原采集设置

回到容器 A，整块执行：

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
root = Path('/ros_ws/src/rm_bringup/config/jetson')
path = root / 'launch_params.yaml'
config = yaml.safe_load(path.read_text())
config.update({
    'use_state_machine_camera': False,
    'video_player': False,
    'virtual_serial': True,
    'replay': False,
})
path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))

path = root / 'camera_driver_params.yaml'
config = yaml.safe_load(path.read_text())
config['/**']['ros__parameters'].update({
    'camera_6mm.enable_frame_rate': True,
    'camera_6mm.acquisition_frame_rate': 100.0,
    'camera_8mm.enable_frame_rate': False,
    'camera_8mm.acquisition_frame_rate': 200.0,
})
path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
print('已设置：真实相机 + 虚拟串口；采集设置恢复为原通用配置的行为')
PY
```

本步骤恢复 6mm 限帧 100Hz、8mm 不开启限帧，以保持你原来的采集行为。仓库初始
Jetson 覆盖文件中的两个 30Hz 只是先前选的临时验证值，不是 Orin 的硬件限制。
8mm 的配置数值 200 不等于实际一定采到 200FPS。

相机内参、手眼标定、序列号、曝光和增益来自原配置。换相机、镜头或安装位置后
需核对这些值；移植计算平台本身不会重新标定。

为了确认安装目录读到改动，可重新安装 bringup：

```bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
colcon build --symlink-install --packages-select rm_bringup --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF
source /ros_ws/install/setup.bash
```

### 9.4 重新启动主程序

在容器 A 执行，保持终端运行：

```bash
export RM_VISION_PROFILE=jetson
export ROS_DOMAIN_ID=0
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

**通过条件：** 相机成功初始化，没有持续的取帧失败，主进程保持运行。
如果失败，先检查 USB 连接、设备是否被其他进程占用、序列号配置和 SDK 日志。
不要通过修改跟踪参数修复相机打开失败。

## 10. 查询真实相机和处理速度

**作用：** 确认模式切换已生效，并区分采集能力和算法处理能力。

**操作位置：终端 B，先从宿主机进入同一容器。**

```bash
docker exec -it rv_devel_ bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 node list
ros2 param get /camera_driver camera_6mm.enable_frame_rate
ros2 param get /camera_driver camera_6mm.acquisition_frame_rate
ros2 param get /armor_detector device
```

预期有 `/camera_driver`、`/virtual_serial`、检测与解算节点，不再有 `/video_player`；
参数分别为 `True`、`100.0`、`CPU`。如果仍为 30.0，检查是否重启主程序，以及
安装目录和当前源码参数是否一致。

逐条执行下面命令，每条观察约 10 秒后按 `Ctrl+C`：

```bash
ros2 topic hz /image_raw
ros2 topic hz /armor_solver/armors
```

第一条测采集发布频率，第二条测解算器装甲板输出频率。100Hz 是目标上限，不是
实际采集保证；曝光、USB、转换、CPU 负载都可能影响实测。CPU 推理较慢时，
算法处理频率低于采集频率很正常，要结合延迟一起判断。

接着按第 8.5 步查看目标和命令。用正确颜色装甲板做静止、移动、进入和离开
画面测试，分别记录结果。没有电控真实姿态时，这仍属于虚拟姿态下的算法验证。

若要查看资源，在 B 中先 `exit` 回到宿主机，再执行：

```bash
tegrastats
```

观察一段时间后按 `Ctrl+C`。它用于查看 Jetson 的 CPU/GPU、内存、温度等情况。
记录采集和处理频率、是否断流、延迟是否增长、进程是否退出。此阶段是 CPU
路线，不应期待 TensorRT GPU 推理负载。

如需对比 30Hz，可在停止主程序后将 `camera_driver_params.yaml` 的 6mm 限帧值
改成 `30.0`，重启再测同一组指标。它是明确的对照实验，会减少采样点；不要把
30Hz 当成正式上车指标，也不要把输入限帧后的结果当成 GPU 吞吐上限。

## 11. 接真实电控，验证串口协议

**作用：** 把模拟姿态替换为真实姿态，验证真实收发，而不是只看本地命令话题。

**要验证：** 端口正确、协议匹配，电控发来的数据能解析，视觉命令能被板端接收。

### 11.1 停止主程序，确认端口和规则

终端 A 按 `Ctrl+C`。终端 B 回到宿主机，先配置 README 中的串口规则：

```bash
sudo tee /etc/udev/rules.d/rm_usb.rules >/dev/null <<'EOF'
KERNEL=="ttyUSB*", MODE:="0777", SYMLINK+="rm_usb0"
KERNEL=="ttyACM*", MODE:="0777", SYMLINK+="rm_usb0"
EOF
sudo udevadm control --reload-rules
```

重新插拔串口设备，检查实际端口：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/rm_usb0
```

某种通配符没有匹配时，`ls` 可能对那一项报不存在，重点看实际出现的设备。
该规则按设备类型匹配，多个串口时不要仅凭自动生成的 `rm_usb0` 判断板子身份。

本分支收发为 24 字节、float32 弧度协议，对照
[电控对接文档](infantry-jeson串口协议_电控对接.md) 确认固件。
YAML 中残留的“16 字节”注释不是实际布局。`rune: false` 时，launch 原本就会
选择 `hero` 协议实现，所以也不能只看 YAML 的 `protocol: infantry` 判断。

### 11.2 修改输入模式和真实端口

在容器 A 执行，先只切换输入模式：

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
path = Path('/ros_ws/src/rm_bringup/config/jetson/launch_params.yaml')
config = yaml.safe_load(path.read_text())
config['virtual_serial'] = False
config['video_player'] = False
config['replay'] = False
path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
print('已设置：真实相机 + 真实串口')
PY
```

查看当前端口配置：

```bash
cat /ros_ws/src/rm_bringup/config/node_params/serial_driver_params.yaml
```

当前默认 `/dev/ttyUSB0`、921600。若实际不同，在容器内打开文件修改：

```bash
nano /ros_ws/src/rm_bringup/config/node_params/serial_driver_params.yaml
```

只按实际电控情况调整 `port_name`、`baud_rate` 等必要项。nano 中 `Ctrl+O` 保存，
按回车确认，再 `Ctrl+X` 退出。容器中的路径和宿主机 `/dev` 通过 Compose 挂载相通。

在容器 A 重新安装 bringup，保证刚改的模式和串口参数进入安装目录，再启动：

```bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
colcon build --symlink-install --packages-select rm_bringup --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF
source /ros_ws/install/setup.bash
export RM_VISION_PROFILE=jetson
export ROS_DOMAIN_ID=0
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

### 11.3 查询真实收发

在宿主机 B 进入容器后加载环境。如果 B 已在容器中，则跳过第一条：

```bash
docker exec -it rv_devel_ bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 node list
ros2 param get /serial_driver protocol
ros2 param get /serial_driver port_name
ros2 param get /serial_driver baud_rate
```

预期 `/virtual_serial` 消失，出现 `/serial_driver`；`rune: false` 时协议参数为
`hero`，端口与波特率应等于你刚核对的值。

逐条测试：

```bash
ros2 topic hz /serial/receive
ros2 topic echo /serial/receive --once --qos-reliability best_effort
ros2 topic echo /armor_solver/cmd_gimbal --once --qos-reliability best_effort
```

第一条观察约 10 秒后 `Ctrl+C`；后两条收到一条后退出。

**通过条件：**

- 真实反馈持续发布，云台姿态变化时 `roll/pitch/yaw` 对应变化，角度单位为 rad。
- 工作模式、弹速等字段与电控实际输入对应。
- 视觉生成命令，板端也确认收到并正确解析；本地存在 `cmd_gimbal` 话题不能
  单独证明实际发送和板端解析成功。

如果端口能打开却没有 `/serial/receive`，继续检查板端是否发数据、波特率、帧长、
校验和字段约定。先定位通信，再判断检测或跟踪表现。

## 12. TensorRT GPU 阶段：条件具备后再做

**作用：** 使用 Orin 的 NVIDIA GPU 执行装甲板模型，并验证与 CPU 路线的结果。

**当前停止条件：** 缺少以下任一项，就先完成前面的 CPU 和硬件测试，不要复制
不存在的镜像地址或把分类模型改名后当作装甲板模型。

1. 对应 `0526.xml/.bin` 的原始四角点装甲板 ONNX 或可导出的训练权重。
2. 在当前 L4T `36.5.2` 上已验证匹配的 GPU 开发镜像，容器内含 CUDA Toolkit、
   TensorRT 10 开发库、ONNX parser 和 TensorRT Python 模块。

模型输入为 FP32 `[1,3,640,640]`，输出为 FP32 `[1,N,22]`，保留原 logits 和四角点
含义。形状符合只是必要条件，仍要验证模型来源和输出语义。`lenet.onnx`、
`mlp.onnx` 是数字分类器，不能替代它。

宿主机安装了 TensorRT，不等于容器内已有开发包。当前默认 `ubuntu:22.04` 镜像
用于 CPU 验证。本次未确认匹配的 GPU 开发镜像；实际标签以
[NVIDIA NGC](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)
为准，不假定存在与 L4T 同名的镜像标签。

### 12.1 换用已验证的 GPU 开发镜像

先在 A 停止主 launch，退出 A、B 的容器 shell。在宿主机仓库目录执行：

```bash
read -r -p "输入已经确认匹配的GPU开发镜像地址: " JETSON_BASE_IMAGE
export JETSON_BASE_IMAGE
printf '本次基础镜像: %s\n' "$JETSON_BASE_IMAGE"
```

确认打印的是你实际验证过的地址，而不是空值或占位文字，再执行：

```bash
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml up -d --build
docker exec -it rv_devel_ bash
```

这会重建开发镜像并重建容器。仓库挂载内容保留，但容器里临时安装的软件可能
需要重新安装。后续重建应保持相同基础镜像设置；该环境变量只在当前宿主机 shell
及其子进程生效。

在新容器 A 中确认环境：

```bash
cd /ros_ws
nvcc --version
dpkg-query --show 'libnvinfer*' 'libnvonnxparsers*'
python3 -c 'import tensorrt as trt; print(trt.__version__)'
```

分别验证 CUDA 编译工具、TensorRT 开发包和 Python 绑定。适配目标为 TensorRT
10.3。若缺包，不继续开启 TensorRT 编译；先补齐同一套环境。

### 12.2 编译 GPU 后端并生成 engine

```bash
cd /ros_ws
ENABLE_TENSORRT=ON bash scripts/jetson/build.sh
```

这条打开 TensorRT 编译开关，必须真正链接 CUDA 和 TensorRT 库。通过后，再输入
容器内真实存在的 ONNX 路径：

```bash
read -r -p "输入容器内0526对应ONNX的绝对路径: " QYG_ONNX_PATH
ls -lh "$QYG_ONNX_PATH"
python3 scripts/jetson/build_armor_engine.py "$QYG_ONNX_PATH" --fp16
```

ONNX 可以放到挂载仓库的 `record-test/` 下，宿主机文件随后能以 `/ros_ws/...`
路径在容器内访问。脚本检查模型并在目标小电脑生成默认文件：

```text
/ros_ws/src/rm_auto_aim/armor_detector/model/0526.engine
```

`--fp16` 允许内部 FP16 计算，I/O 仍为 FP32。需要先建立 FP32 基准时省略该选项。
更换 TensorRT、CUDA 或设备环境后重新生成 engine，不复制大电脑的 engine。

### 12.3 执行 CPU/GPU 对比

主程序保持停止，在容器 A 中执行：

```bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --symlink-install --packages-select armor_detector --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=ON -DENABLE_TENSORRT=ON
source /ros_ws/install/setup.bash
ARMOR_TENSORRT_ENGINE=/ros_ws/src/rm_auto_aim/armor_detector/model/0526.engine \
  colcon test --packages-select armor_detector \
  --ctest-args -R '^test_(detector|yolo_backends)$' --output-on-failure
colcon test-result --verbose
```

`ARMOR_TENSORRT_ENGINE` 把实际 engine 路径传给测试进程。直接看新增测试明细时：

```bash
ARMOR_TENSORRT_ENGINE=/ros_ws/src/rm_auto_aim/armor_detector/model/0526.engine \
  build/armor_detector/test_yolo_backends
```

**通过条件：** `TensorRtMatchesCpuOnReferenceImage` 实际执行并为 `OK`，不是
`SKIPPED`。它检查同一参考图片上的数量、类别、类型、置信度和角点基本一致性，
不替代多距离、多姿态和运动目标测试。失败时先检查模型来源、预处理、输出布局
及 FP16 误差。

### 12.4 切换运行后端，再重复视频与硬件测试

在容器 A 修改参数：

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
path = Path('/ros_ws/src/rm_bringup/config/jetson/armor_detector_params.yaml')
config = yaml.safe_load(path.read_text())
config['/**']['ros__parameters'].update({
    'yolo.backend': 'tensorrt',
    'yolo.model': 'package://armor_detector/model/0526.engine',
})
path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
print('已选择 TensorRT；需要重启主 launch')
PY
```

`device: CPU` 和 `yolo.weights` 此时只属于 OpenVINO 配置，不控制 TensorRT。
在容器 A 安装更新后的配置并启动：

```bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
colcon build --symlink-install --packages-select rm_bringup --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF
source /ros_ws/install/setup.bash
export RM_VISION_PROFILE=jetson
export ROS_DOMAIN_ID=0
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

日志应包含 `YOLO backend: tensorrt`。终端 B 从宿主机进入同一容器，查询参数：

```bash
docker exec -it rv_devel_ bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 param get /armor_detector yolo.backend
```

预期为 `tensorrt`；主 launch 必须保持运行。

分别重复视频测试、真实相机测试和真实串口测试。比较 CPU/GPU 时保持输入、
分辨率、采集帧率、曝光等条件一致，同时记录结果一致性、处理频率、延迟和
资源占用。输入被 30Hz 限制时，不能据此断言 GPU 的最大处理能力只有 30FPS。

需要退回 CPU 时，把后端恢复为 `openvino`，模型恢复为
`package://armor_detector/model/IR/0526.xml`，然后重启。

## 13. 停止、再次启动和日常更新

### 13.1 正常结束本次测试

1. 在 A 按 `Ctrl+C` 停止主 launch，等待回到 shell 提示符。
2. 在 B 按 `Ctrl+C` 停止持续话题统计。
3. 在容器 shell 输入 `exit`，回到宿主机；`docker exec` 的退出不会停止开发容器。

只想停掉开发容器时，在宿主机仓库目录执行：

```bash
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml stop rv_devel
```

这里 `rv_devel` 是 Compose 服务名，`rv_devel_` 是容器名，不能混用。

### 13.2 下次继续测试

容器之前仅停止、镜像和配置未变时，在宿主机执行：

```bash
docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml up -d
docker exec -it rv_devel_ bash
```

进入容器后：

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
export RM_VISION_PROFILE=jetson
export ROS_DOMAIN_ID=0
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

先确认上次留下的是视频、虚拟串口还是实物模式。`restart: unless-stopped` 只会
保活当前开发容器，不会让手动运行的主 launch 自动成为开机服务。

### 13.3 拉取后怎样判断是否重建

更新前先停止主程序，在宿主机记录和保存本地测试配置，再更新 Git。不要直接用
强制重置覆盖本机参数。

| 更新内容 | 下一步 |
| --- | --- |
| 只改文档 | 不需要编译或重建镜像 |
| C++、接口或 CMake 改动 | 容器内重新运行构建脚本，并加载新工作区 |
| 只改 YAML | 检查安装目录是否同步；必要时安装 bringup，再重启主 launch |
| Dockerfile 或基础镜像 | 宿主机用 `up -d --build` 重建环境，再在容器内编译 |
| TensorRT/CUDA/设备环境变化 | 重新验证开发环境并生成 engine |

## 14. 配置备份、恢复与提交边界

第 6.2 步自动备份了测试前的输入模式、检测参数、视频参数、相机覆盖参数和串口
参数，共五个文件。列出备份：

```bash
ls -1 /ros_ws/record-test/config-backups
```

若要恢复，先停止主程序，在容器中输入**之前打印的那一个备份目录**：

```bash
read -r -p "输入要恢复的备份目录绝对路径: " QYG_CONFIG_BACKUP
ls "$QYG_CONFIG_BACKUP"
```

确认里面是你要恢复的五个文件，再逐条执行：

```bash
cp "$QYG_CONFIG_BACKUP/launch_params.yaml" /ros_ws/src/rm_bringup/config/jetson/launch_params.yaml
cp "$QYG_CONFIG_BACKUP/armor_detector_params.yaml" /ros_ws/src/rm_bringup/config/jetson/armor_detector_params.yaml
cp "$QYG_CONFIG_BACKUP/video_player_params.yaml" /ros_ws/src/rm_bringup/config/node_params/video_player_params.yaml
cp "$QYG_CONFIG_BACKUP/camera_driver_params.yaml" /ros_ws/src/rm_bringup/config/jetson/camera_driver_params.yaml
cp "$QYG_CONFIG_BACKUP/serial_driver_params.yaml" /ros_ws/src/rm_bringup/config/node_params/serial_driver_params.yaml
```

这五项不包括你额外修改的相机内参文件、手眼标定等其他配置。恢复后也应检查
安装目录并重启节点。查看本地全部配置差异，请退出到宿主机仓库目录执行：

```bash
git status --short
git --no-pager diff -- src/rm_bringup/config/
```

准备提交时，区分可共享的部署配置和仅对本机有效的串口、视频路径、标定值。
本教程的测试命令会改本机 YAML，并不自动提交或推送；录像、配置备份和 engine
也不应作为通用源码提交。

**本次新手册的更新需要从修改文档的电脑推送一次，小电脑才能拉到新版。**
以下仅在修改本手册的那台电脑、对应仓库根目录执行：

```bash
git add docs/jetson.md docs/jetson_changes.md
git --no-pager diff --cached --stat
git commit -m "docs(jetson): 细化小电脑测试与部署步骤"
git push origin infantry-jetson-Lonesome-George
```

小电脑保存好本地配置后执行 `git pull --ff-only` 即可更新文档。之后的编译和
测试都可留在小电脑，无需再依赖大电脑的测试容器。

## 15. 每一步失败时，保留什么信息

| 失败位置 | 优先记录与检查 |
| --- | --- |
| Git 下载 | 完整网络错误、远程地址、当前分支；`early EOF` 表示下载不完整 |
| Docker 构建 | 第一条实际失败的步骤、末尾错误、磁盘与内存情况 |
| 容器创建 | 是否存在同名旧容器、挂载目录、`nvidia` runtime 是否注册 |
| 工作区编译 | 首个失败包、具体编译/链接错误，不只截最后一行汇总 |
| CPU 测试 | 失败的测试名称、模型/插件报错；GPU 跳过是此阶段的预期 |
| 主程序启动 | 终端 A 初始化日志，以及实际读取的输入模式和模型路径 |
| 没有话题 | 先查 A 是否还活着、B 的容器和 ROS 环境，再查图像源与 TF |
| 有图像但没目标 | 画面是否有目标、模式 0/1 的颜色、阈值、标定与 TF |
| 相机失败 | USB 是否枚举、SDK 架构、设备占用、序列号和取帧错误 |
| 串口无反馈 | 实际端口、波特率、板端发送、24 字节协议和解码错误 |
| GPU 编译/推理 | 容器内 CUDA/TensorRT 开发包、模型来源、engine 生成环境 |

如果终端命令一直统计输出，先 `Ctrl+C`。如果进入 Git 翻页，按 `q`。
如果出现单独的 `>` 提示符，通常是引号或多行命令没有结束，按 `Ctrl+C`，再完整
复制当前代码块。不要在未结束的命令后继续粘贴下一步。

最后把每阶段结果填下来，而不是只记录“成功/失败”：

| 阶段 | 是否通过 | 实测与备注 |
| --- | --- | --- |
| 分支与适配文件同步 | 待填 | Git 提交号、SDK 架构 |
| ARM64 镜像和工作区构建 | 待填 | 镜像、首个失败包或构建成功 |
| ARM CPU 自动测试 | 待填 | CPU 项结果、GPU 项是否跳过 |
| 视频 + 虚拟串口 | 待填 | 输入视频、模式、图像/处理频率 |
| 相机 + 虚拟串口 | 待填 | 相机、采集设置、频率、稳定性 |
| 相机 + 真实串口 | 待填 | 端口、协议、板端收发验证 |
| TensorRT GPU | 待填 | 模型、CUDA/TensorRT、对比测试与延迟 |
