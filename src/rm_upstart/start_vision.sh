#!/usr/bin/env bash
# 容器前台入口：加载运行环境后，将进程交给 ROS launch。
set -e

vision_workspace="${RM_VISION_WORKSPACE:-/ros_ws}"
vision_start_delay="${RM_VISION_START_DELAY:-5}"

if [[ ! "$vision_start_delay" =~ ^[0-9]+$ ]]; then
    echo "[rm_upstart] RM_VISION_START_DELAY 必须为非负整数（秒）。" >&2
    exit 1
fi

if [[ ! -r /opt/ros/humble/setup.bash || ! -r "$vision_workspace/install/setup.bash" ]]; then
    echo "[rm_upstart] 缺少 ROS Humble 或工作区 install/setup.bash，请先完成编译。" >&2
    exit 1
fi

# ROS/OpenVINO 的环境脚本不保证兼容 nounset，因此此处只启用 errexit。
source /opt/ros/humble/setup.bash
if [[ -r /opt/openvino/setupvars.sh ]]; then
    source /opt/openvino/setupvars.sh
elif [[ -r /usr/local/setupvars.sh ]]; then
    source /usr/local/setupvars.sh
fi
source "$vision_workspace/install/setup.bash"
cd -- "$vision_workspace"

echo "[rm_upstart] 等待 ${vision_start_delay} 秒后启动自瞄。"
sleep "$vision_start_delay"
exec ros2 launch rm_bringup bringup_SingleProcess.launch.py
