# 开机自启动说明 - Lonesome George

`infantry-cwr` 的 Docker 开机自启动。宿主机 systemd 管理 `rm.service`，
服务以前台方式启动 `rv_devel_`，容器自动运行自瞄，不需要登录桌面或打开终端。

## 使用前准备

1. 在小电脑上准备本分支的 Docker 环境，确认 `/ros_ws` 挂载当前仓库。
2. 在容器中编译并手动验证自瞄，使用真实相机、串口还是视频测试，由现有启动参数决定。
3. 停止手动启动的主程序，再注册服务。

容器内的编译、验证命令：

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 4
source install/setup.bash
ros2 launch rm_bringup bringup_SingleProcess.launch.py
# 确认正常后 Ctrl+C 停止，exit 返回宿主机。
```

`register_service.sh` 现在用于 Docker 部署，不再注册旧的宿主机原生 ROS 看门狗。
服务用户需要具有 Docker 权限，可先在宿主机用 `docker info` 检查；如需添加用户组：

```bash
sudo usermod -aG docker "$USER"
# 退出登录后重新登录，再检查 docker info。
```

## 一次注册

在**小电脑宿主机、当前仓库根目录**执行：

```bash
sudo bash src/rm_upstart/register_service.sh
```

脚本自动识别仓库路径、调用 sudo 的用户和 Docker 路径，写入
`/etc/systemd/system/rm.service`，启用 Docker 和自瞄的开机启动，并立即启动自瞄。
它会先检查编译产物、Compose 配置、服务用户的 Docker 权限、本地镜像以及同名容器的挂载路径。
仓库移动或更换用户后需重新注册。可以从任意目录使用脚本的绝对路径注册。

只预览服务配置、不安装或启动：

```bash
bash src/rm_upstart/register_service.sh --print-unit
```

首次启动会使用基础 Compose 和 `docker-compose.autostart.yaml` 重建同名容器，
保留 `/dev`、工作区和日志挂载，使用本地已有镜像。不会构建镜像或编译代码，
容器可写层中临时安装的依赖会丢失，因此注册前应将所需依赖固化到镜像。
这套默认注册配置使用当前 cwr 分支的基础镜像，没有叠加 Jetson GPU 配置。

## 启停与日志

```bash
systemctl is-enabled rm.service            # 是否开机启动
systemctl status rm.service                # 当前服务状态
journalctl -u rm.service -b -f             # 本次开机实时日志
docker logs --tail 100 rv_devel_           # 容器最近输出

sudo systemctl stop rm.service             # 临时停止，不会因 Restart=always 被重新拉起
sudo systemctl start rm.service            # 启动
sudo systemctl restart rm.service          # 重启
sudo systemctl disable --now rm.service    # 停止并取消开机启动
```

`start_vision.sh` 会加载 ROS Humble、可用的 OpenVINO 环境和工作区环境，
等待 5 秒后执行 `ros2 launch rm_bringup bringup_SingleProcess.launch.py`。
如需调整等待时间，在自启动 Compose 的 `rv_devel` 下添加
`environment: {RM_VISION_START_DELAY: "10"}`，随后重启服务。
固定延时不是设备就绪检测；相机或串口初始化失败导致主进程退出时，服务会再次尝试。

主组件进程退出会触发整个 launch 关闭，避免仅剩 Foxglove 时服务仍显示运行。
systemd 在服务退出后等待 10 秒重启；Docker 本身的重启策略在自启动配置中设为 `no`，
因此停止服务后容器不会自行复活。停止操作只针对该 Compose 的 `rv_devel` 服务。
此方式监控进程退出，不检测进程存活但数据停滞的情况。

`active (running)` 只说明服务正在运行，还应查看日志和实际 ROS 话题确认自瞄就绪。
自定义项目日志仍在 `qd2026-log/`，容器标准输出日志按 20 MB × 3 个文件轮转，
ROS 自身的文件日志仍需按机器的磁盘容量定期清理。

## 开机验收与恢复手动启动

注册后先检查日志和相机、串口状态。随后在小电脑宿主机执行 `sudo reboot`，
重启后检查：

```bash
systemctl is-enabled rm.service
systemctl status rm.service
journalctl -u rm.service -b --no-pager -n 100
```

确认没有手动输入启动命令，自瞄已输出正常图像和通信数据。
修改启动配置后按仓库惯例重新编译 `rm_bringup`，再重启服务；编译期间先停止服务。

恢复开发用的交互式容器：

```bash
sudo systemctl disable --now rm.service
docker compose -f docker-compose.yaml up -d --no-build --pull never rv_devel
docker exec -it rv_devel_ bash
```

恢复开机启动可以重新运行注册脚本。未恢复基础容器前，单独 `docker start rv_devel_`
仍会使用上次保存的自瞄命令；自启动服务运行期间也不要再手动启动第二套自瞄。

## 原有脚本

`rm_watch_dog.sh` 和 `rm_clean_up.sh` 保留供旧部署查阅，新服务不调用它们。
旧脚本含固定路径、额外 Foxglove 启动以及 `pkill -f ros` 的宽泛清理，
不应与新服务同时运行。
