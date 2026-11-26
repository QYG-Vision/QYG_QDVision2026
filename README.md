# QD_Vision2026

> 当前串口已迁移 V2 通信接口：发送 40 字节、接收 32 字节，物理量统一为 float32 弧度及其导数，帧头 `0xFE`。
> 装甲板已接入约束 MPC：目标与弹道有效且规划成功时输出同一时刻的 p/v/a；能量机关仍不发送有效参考。
> 字段和接入条件见 [当前电控对接协议](infantry-cwr串口协议_电控对接.md)。

奇点战队2026赛季视觉项目仓库，该项目基于中南大学FYT战队24赛季开源

## 一、项目结构

```
.
│
├── rm_bringup (启动及参数文件)
│
├── rm_robot_description (机器人urdf文件，坐标系的定义)
│
├── rm_interfaces (自定义msg、srv)
│
├── rm_hardware_driver
│   │
│   ├── hik-camera (海康相机驱动)
│   │
│   └── rm_serial_driver (串口驱动)
│
├── rm_auto_aim (自瞄算法)
│
├── rm_rune (打符算法)
│
├── rm_utils (工具包) 
│   ├── math (包括PnP解算、弹道补偿等)
│   │
│   └── logger (日志库)
│
├── rm_upstart (自启动配置)
|
└── rm_auto_replay (自动播放录包)
```

## 二、环境配置

```bash
# 海康相机 udev 规则
sudo tee /etc/udev/rules.d/rm_hikrobot.rules >/dev/null <<'EOF'
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", MODE="0666", GROUP="plugdev"
EOF
# 串口 udev 规则
sudo tee /etc/udev/rules.d/rm_usb.rules >/dev/null <<'EOF'
KERNEL=="ttyUSB*", MODE:="0777", SYMLINK+="rm_usb0"
KERNEL=="ttyACM*", MODE:="0777", SYMLINK+="rm_usb0"
EOF
# 重新加载 udev 规则
sudo udevadm control --reload-rules
sudo udevadm trigger -v --action=add
```

### Docker

```bash
# x86
docker pull slirute/qidian:latest
# arm64
docker pull slirute/qidian:arm64v8 
```

使用 docker 部署环境，根据需要注释`docker-compose.yaml`里的`command`（只能有一条 command）

```bash
# 构建容器
docker compose up -d 
# 进入容器
docker exec -it rv_devel_ bash
# 进入工作空间
cd /ros_ws
```

### Local

#### 1. 基础

- Ubuntu 22.04
- ROS2 Humble

#### 2. APT 

```bash
sudo apt install libfmt-dev libceres-dev
```

### 3. openvino

参考[OpenVINO官方文档](https://docs.openvino.ai/2022.3/openvino_docs_install_guides_installing_openvino_from_archive_linux.html)，建议同时安装GPU相关依赖

### 4. ROS包

使用`rosdep`安装剩下依赖

```bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-humble-asio-cmake-module       #该包不会自动安装
```

如果还有欠缺的请看根目录下的Dockerfile文件

### Pixi

```bash
# 安装 pixi
curl -fsSL https://pixi.sh/install.sh | sh
# 安装依赖
pixi install
# 进入环境
pixi shell
```

## 三、编译与运行

修改[rm_bringup/config/launch_params.yaml](src/rm_bringup/config/launch_params.yaml)，选择需要启动的功能

```bash
# 使用 pixi 的话用这行编译
pixi run build
# 编译
# 本仓库包含的功能包过多，建议限制同时编译的线程数
# 推荐 8G 2线程 ，16G 4线程, 32G不用限制
colcon build --symlink-install --parallel-workers 4 

# 运行
source install/setup.bash
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

默认日志和内录视频路径为`qd2026-log/`

## 四、自启动

使用宿主机的 `rm.service` 管理 Docker 自瞄容器。开机等待 Docker 和网络就绪，
容器内自动加载 ROS 环境并运行 `bringup_SingleProcess.launch.py`；程序退出后由
systemd 间隔 10 秒重启。启动参数沿用 `src/rm_bringup/config/launch_params.yaml`。

先在容器中完成编译并确认能手动运行，再停止手动启动的自瞄。在**小电脑宿主机的仓库根目录**执行一次：

```bash
sudo bash src/rm_upstart/register_service.sh
```

注册脚本自动识别仓库绝对路径和当前用户，并立即启用、启动服务。
首次启动会按 `docker-compose.yaml` + `docker-compose.autostart.yaml` 重建 `rv_devel_`，
使用本地已有镜像，不自动编译或下载。容器内临时安装的依赖应先写入镜像；挂载的工作区会保留。

```bash
systemctl status rm.service                # 查看状态
journalctl -u rm.service -f                # 查看实时输出
sudo systemctl stop rm.service            # 临时停止，本次开机不再自动重启
sudo systemctl start rm.service           # 再次启动
sudo systemctl disable --now rm.service   # 停止并取消开机自启
```

用户须具有 Docker 权限。恢复手动启动、首次编译和重启验收步骤见
[开机自启动说明 - Lonesome George](docs/开机自启动说明-Lonesome-George.md)。不要在容器内运行注册脚本。

## 开源许可证

```
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

## 致谢

感谢以下开源项目：

- [FYT2024_vision](https://github.com/CSU-FYT-Vision/FYT2024_vision) fyt2024是本项目的基础，提供了一套可参考的，规范、易用、高效的视觉算法框架
- [北洋机甲2024视觉算法库](https://github.com/HHgzs/OpenRM-2024) 提供了三分法代码结构
- [上海交通大学](https://github.com/julyfun/rm.cv.fans) 提供三分法代价计算的代码和双模型切换思路
- [sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) 提供 EKF 前哨战适配思路
- [深圳大学](https://github.com/broalantaps/RobotDetectionModel) 提供神经网络模型

## 更新日志

- 移植上交提出的三分法求YAW
- 加入录包功能
- 加入双头功能
- 制作了docker容器
- 加入平移模型
- 改成同济的 EKF 并适配新前哨

## 未来计划

- [ ] fix:平移模型时打中一块 armor 会转到另一块 armor 打（原因是打中后装甲板灯灭了）
- [ ] fix:马氏距离匹配没有设置允许通过阈值，只按最小的跟踪
- [ ] fix:装甲板在边缘旋转时匹配出错，做 nis 发散检测重置滤波器
- [ ] 测试车在坡上、存在 roll 、枪管装歪对自瞄的影响，判断是否需要更全面的坐标系建模（云台、底盘、枪管）
- [ ] fix:整理线程，优化性能调度
- [ ] 测试 龙格-库塔（RK4） 弹道解算
- [ ] feat:加入能量机关自瞄
- [x] feat:接入装甲板双轴约束 MPC 与 V2 p/v/a 参考（Eigen 活动集 QP 实现）
- [ ] refactor:重构双相机共头代码，改用 FOV 重叠切换相机逻辑
- [ ] feat:加入 robot 选择功能，操作手能选第一优先级 robot
- [ ] refactor: 重构全向感知逻辑，更具鲁棒性，便于调试、观测
- [ ] 测试 [jlu_vision_26](https://github.com/Fskaaaaaaaa/jlu_vision_26) 提出的 uv 系作为 EKF 观测量坐标系
- [ ] feat:加入网页前端调试，降低部署门槛
- [ ] refactor:重构整套代码，移除 ros 核心依赖，ros 只作为外部调试接口使用

## slirute/qidian

- `2.5`:加入Intel GPU驱动和 `rsync`
- `2.4`:改用`ros-humble-desktop`以使用rviz，加入`plotjuggler`和`tf-transformations`包
- `2.3`：移除`G2O` 和 `Sophus` 库
- `2.2`：完善 ARM 支持与启动脚本自动化
- `2.1`：引入多架构支持（AMD64/ARM64）与多阶段构建
- `2.0`:仅针对`FYT2024_vision`制作的镜像

## 开发实用技巧

### rsync

使用`rsync`同步文件

```bash
rsync -rlptzvP --delete \
--exclude='build/' \
--exclude='install/' \
--exclude='log/' \
--exclude='MvSDKLog/' \
--exclude='qd2026-log/' \
--exclude='record/' \
--exclude='rosbag/' \
--exclude='watchdog/' \
./ qidian@192.168.137.x:~/rmvision2025/
```

### 进入容器

在`~/.bashrc`中加入以下内容，每次进入容器只需要输入`rv`即可进入

```bash
echo "alias rv='docker exec -it rv_devel_ bash'" >> ~/.bashrc 
```

### commit 规范

- 提交信息应包含：类型 (Type)、描述 (Subject) 以及可选的 范围（scope）、正文 (Body) 和 脚注 (Footer)

```
<type>(<scope>): <subject>

<body>

<footer>
```

- 常用类型 (Type)

    | 类型 | 描述 |
    | :--- | :--- | 
    | **feat** | 新增功能 (feature) |
    | **fix** | 修复 Bug |
    | **docs** | 文档修改 (Documentation) | 
    | **style** | 代码格式修改 (不影响逻辑，如空格、分号等) | 
    | **refactor** | 代码重构 (既不是修复 Bug 也不是新增功能) | 
    | **perf** | 性能优化 (Performance) | 
    | **test** | 增加或修改测试用例 | 
    | **chore** | 构建过程、辅助工具或依赖库的变动 | 
