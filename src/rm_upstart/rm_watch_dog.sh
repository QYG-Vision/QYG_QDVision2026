#!/bin/bash
# watch_dog.sh

# 设置 USB 内存限制，多相机必备
echo 4000 > /sys/module/usbcore/parameters/usbfs_memory_mb

WORKING_DIR="/ros_ws" # 代码目录
TIMEOUT=10  # 设定超时时间为10秒
NAMESPACE="" # 命名空间 例如 "/infantry_3" 注意要有"/"
NODE_NAMES=("armor_detector" "armor_solver" "serial_driver")  # 列出所有需要监控的节点名称，注意是用空格分隔


# 启动 launch 文件配置
CONFIG_FILE="$WORKING_DIR/src/rm_bringup/config/launch_params.yaml"
USE_STATE_MACHINE=$(grep "use_state_machine_camera:" "$CONFIG_FILE" | awk '{print $2}')
LAUNCH_FILE="rm_bringup bringup_SingleProcess.launch.py" # launch 文件
if [ "$USE_STATE_MACHINE" == "false" ]; then
    NODE_NAMES+=("camera_driver")
fi

# 日志输出目录
SESSION_TIME=$(date +'%Y-%m-%d_%H-%M-%S')
LOG_DIR="$WORKING_DIR/watchdog/$SESSION_TIME"
mkdir -p "$LOG_DIR"

# ROS 环境配置

rmw="rmw_fastrtps_cpp" #RMW
export RMW_IMPLEMENTATION="$rmw" # RMW实现

USER="$(whoami)" #用户名
HOME_DIR=$(eval echo ~$USER)
export ROS_HOSTNAME=$(hostname)
export ROS_HOME=${ROS_HOME:=$HOME_DIR/.ros}
export ROS_LOG_DIR="/tmp"

source /opt/ros/humble/setup.bash
source $WORKING_DIR/install/setup.bash

# 中间件配置
rmw_config=""
if [[ "$rmw" == "rmw_fastrtps_cpp" ]]
then
  if [[ ! -z $rmw_config ]]
  then
    export FASTRTPS_DEFAULT_PROFILES_FILE=$rmw_config
  fi
elif [[ "$rmw" == "rmw_cyclonedds_cpp" ]]
then
  if [[ ! -z $rmw_config ]]
  then
    export CYCLONEDDS_URI=$rmw_config
  fi
fi

function log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_DIR/watchdog.log"
}

function bringup() {
    LAUNCH_TIME=$(date +'%Y-%m-%d_%H-%M-%S')
    ROS_LOG_FILE="$LOG_DIR/ros2_launch_$LAUNCH_TIME.log"
    log "Starting ROS 2 system, logging to $ROS_LOG_FILE..."
    source /opt/ros/humble/setup.bash
    source $WORKING_DIR/install/setup.bash
    nohup ros2 launch $LAUNCH_FILE > "$ROS_LOG_FILE" 2>&1 &
    nohup ros2 launch foxglove_bridge foxglove_bridge_launch.xml 2>&1 & # 调试用
}

function restart() {
    log "Performing cleanup of ROS 2 processes..."
    pkill -f ros  # 杀掉所有ROS2进程
    ros2 daemon stop
    ros2 daemon start
    bringup
}

restart
sleep $TIMEOUT

# 监控每个节点的心跳
while true; do
    for node in "${NODE_NAMES[@]}"; do
        topic="$NAMESPACE/$node/heartbeat"
        log "- Check $node"
        if ros2 topic list 2>/dev/null | grep -q $topic 2>/dev/null; then
            data_value=$(timeout 10 ros2 topic echo $topic --once | grep -o "data: [0-9]*" | awk '{print $2}' 2>/dev/null)
            if [ ! -z "$data_value" ]; then
                log "    $node is OK! Heartbeat FPS: $data_value"
            else
                log "    Heartbeat lost for $topic, restarting all nodes..."
                restart
                break 
            
            fi
        else
            log "    Heartbeat topic $topic does not exist, restarting all nodes..."
            restart
            break
        fi
    done
    sleep $TIMEOUT
done
