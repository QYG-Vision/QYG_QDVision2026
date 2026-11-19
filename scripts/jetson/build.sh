#!/usr/bin/env bash
set -euo pipefail

# 在 /ros_ws 容器内执行，包级与包内并发分开限制，避免 8GB 设备耗尽内存。
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_dir"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble is missing; run this script inside the development container." >&2
    exit 1
fi
if [[ ! "${ENABLE_TENSORRT:-OFF}" =~ ^(ON|OFF)$ ]]; then
    echo "ENABLE_TENSORRT must be ON or OFF." >&2
    exit 1
fi

# ROS 环境脚本可能读取尚未定义的变量。
set +u
source /opt/ros/humble/setup.bash
if [[ -f /opt/openvino/setupvars.sh ]]; then
    source /opt/openvino/setupvars.sh
fi
set -u
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
if [[ ! "$CMAKE_BUILD_PARALLEL_LEVEL" =~ ^[1-9][0-9]*$ ]]; then
    echo "CMAKE_BUILD_PARALLEL_LEVEL must be a positive integer." >&2
    exit 1
fi
# Humble 的 colcon 会显式传入 -jCPU核数，需同时限制 Make，才能真正控制包内并发。
export MAKEFLAGS="${MAKEFLAGS:--j${CMAKE_BUILD_PARALLEL_LEVEL} -l${CMAKE_BUILD_PARALLEL_LEVEL}}"
colcon build --symlink-install --parallel-workers 1 \
    --cmake-args -DBUILD_TESTING=OFF "-DENABLE_TENSORRT=${ENABLE_TENSORRT:-OFF}"
