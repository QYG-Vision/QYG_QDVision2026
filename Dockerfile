# 普通开发使用原生架构 Ubuntu；Jetson 可传入配套的 L4T JetPack 开发镜像。
ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE} AS final

# 创建 workspace
RUN mkdir -p /ros_ws
WORKDIR /ros_ws/

# 设置非交互式环境避免卡在时区选择
ENV DEBIAN_FRONTEND=noninteractive
# 设置默认时区
ENV TZ=Asia/Shanghai

# 设置线程数
ARG THREADS=2
ENV THREADS=${THREADS}
ENV CMAKE_BUILD_PARALLEL_LEVEL=${THREADS}
ARG JETSON_MINIMAL=OFF
ARG ROS_BASE_PACKAGE=ros-humble-desktop

# 更换软件源为中科大源
RUN \
    if [ -f /etc/apt/sources.list ]; then \
        if [ "$(dpkg --print-architecture)" = "arm64" ]; then \
            sed -i \
                -e 's@//ports.ubuntu.com\(:80\)\?/ubuntu-ports@//mirrors.ustc.edu.cn/ubuntu-ports@g' \
                /etc/apt/sources.list; \
        else \
            sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list; \
        fi; \
    else \
        if [ "$(dpkg --print-architecture)" = "arm64" ]; then \
            sed -i 's@//ports.ubuntu.com\(:80\)\?/ubuntu-ports@//mirrors.ustc.edu.cn/ubuntu-ports@g' /etc/apt/sources.list.d/ubuntu.sources; \
        else \
            sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list.d/ubuntu.sources; \
        fi; \
    fi && \
    # 国内镜像走直连，避免全局代理节点访问镜像站返回 502。
    printf '%s\n' \
        'Acquire::http::Proxy::mirrors.ustc.edu.cn "DIRECT";' \
        'Acquire::https::Proxy::mirrors.ustc.edu.cn "DIRECT";' \
        > /etc/apt/apt.conf.d/99qyg-mirror-direct

# apt 依赖
RUN \
    apt update && apt install -y \
    # ROS 仓库和构建的基础工具
    locales curl lsb-release \
    # 编译工具
    git cmake build-essential g++ \
    # 依赖
    libeigen3-dev libceres-dev libopencv-dev libfmt-dev libtbb-dev libusb-1.0-0 \
    # OpenVINO 构建与仓库工具
    wget gnupg bash-completion && \
    if [ "$JETSON_MINIMAL" != "ON" ]; then \
        apt install -y nano htop rsync clangd software-properties-common; \
    fi && \
    apt clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* && \
    # 设置语言环境
    locale-gen zh_CN.UTF-8 && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=zh_CN.UTF-8

ENV LANG=zh_CN.UTF-8
ENV LC_ALL=zh_CN.UTF-8

# 安装 ROS2 Humble 
RUN \
    # 安装密钥
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.ustc.edu.cn/ros2/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    # 安装 ROS2 Humble
    apt update && \
    apt install -y "$ROS_BASE_PACKAGE" ros-humble-foxglove-bridge python3-colcon-common-extensions python3-rosdep python3-argcomplete && \
    apt clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# openvino
# ARM64 优先使用宿主机准备的预编译包，避免容器构建时重复下载。
RUN --mount=type=bind,source=scripts/jetson/openvino-cache,target=/mnt/openvino-cache \
    set -eu; \
    if [ "$(dpkg --print-architecture)" = "arm64" ]; then \
        archive=openvino_toolkit_ubuntu22_2025.3.0.19807.44526285f24_arm64.tgz; \
        archive_path="/mnt/openvino-cache/$archive"; \
        if [ -f "$archive_path" ]; then \
            echo "Using local OpenVINO archive: $archive"; \
        else \
            archive_path="/tmp/$archive"; \
            curl -fSL --retry 3 --retry-all-errors --connect-timeout 20 \
                "https://storage.openvinotoolkit.org/repositories/openvino/packages/2025.3/linux/$archive" \
                -o "$archive_path"; \
        fi; \
        printf '%s  %s\n' '470f5c0ffbbf7c6983bd73c9cabf540fbc869043f5cc12f296e26011089464c0' "$archive_path" \
            | sha256sum -c - && \
        mkdir -p /opt/openvino && \
        tar -xzf "$archive_path" --strip-components=1 -C /opt/openvino && \
        rm -f "/tmp/$archive"; \
    else \
        echo "Installing OpenVINO from Intel APT repository for amd64..." && \
        wget -O /tmp/intel-gpg-key.pub https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
        gpg --dearmor < /tmp/intel-gpg-key.pub > /etc/apt/trusted.gpg.d/intel-openvino.gpg && \
        echo "deb https://apt.repos.intel.com/openvino ubuntu22 main" > /etc/apt/sources.list.d/intel-openvino.list && \
        apt update && apt install -y openvino-2025.3.0 && \
        apt clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* ; \
    fi

ENV OpenVINO_DIR=/opt/openvino/runtime/cmake

# rmvision
RUN \
    # rosdep
    mkdir -p /etc/ros/rosdep/sources.list.d/ && \
    curl -o /etc/ros/rosdep/sources.list.d/20-default.list https://mirrors.ustc.edu.cn/rosdistro/rosdep/sources.list.d/20-default.list && \
    sed -i 's#raw.githubusercontent.com/ros/rosdistro/master#mirrors.ustc.edu.cn/rosdistro#g' /etc/ros/rosdep/sources.list.d/20-default.list && \
    # Jetson 的依赖已在下方列出，首次构建无需下载 rosdep 索引。
    if [ "$JETSON_MINIMAL" != "ON" ]; then \
        ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml rosdep update; \
    fi && apt update && \
    # 安装rmvision依赖
    apt install -y ros-humble-asio-cmake-module \
    ros-humble-vision-opencv \
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration-parsers \
    ros-humble-serial-driver \
    ros-humble-udp-msgs \
    ros-humble-io-context \
    ros-humble-ament-cmake-clang-format \
    ros-humble-angles \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-tf-transformations \
    ros-humble-xacro \
    ros-humble-rosbag2-storage-mcap \
    && if [ "$JETSON_MINIMAL" != "ON" ]; then \
        apt install -y ros-humble-camera-calibration ros-humble-plotjuggler ros-humble-plotjuggler-ros; \
    fi && \
    apt clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# 添加环境变量 到 bashrc
RUN echo "if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi" >> /root/.bashrc \
    && echo "if [ -f /opt/openvino/setupvars.sh ]; then source /opt/openvino/setupvars.sh; fi" >> /root/.bashrc \
    && echo "if [ -f /ros_ws/install/setup.bash ]; then source /ros_ws/install/setup.bash; fi" >> /root/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /root/.bashrc \
    && echo "if [ -f /usr/share/bash-completion/bash_completion ]; then source /usr/share/bash-completion/bash_completion; fi" >> /root/.bashrc  \
    && echo 'export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml' >> /root/.bashrc

# 安装Intel GPU驱动
RUN \
    if [ "$(dpkg --print-architecture)" = "amd64" ]; then \
        echo "Installing Intel GPU Drivers for amd64..." && \
        # 添加Intel Graphics官方GPG密钥
        wget -qO - https://repositories.intel.com/gpu/intel-graphics.key | gpg --dearmor --output /usr/share/keyrings/intel-graphics.gpg && \
        # 添加Intel Graphics官方软件源
        echo "deb [arch=amd64 signed-by=/usr/share/keyrings/intel-graphics.gpg] https://repositories.intel.com/gpu/ubuntu jammy client" | tee /etc/apt/sources.list.d/intel-gpu-jammy.list && \
        apt update && \
        # 安装核心驱动组件：
        apt install -y \
            intel-opencl-icd \
            intel-level-zero-gpu \
            level-zero \
            ocl-icd-libopencl1 && \
        apt clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* ; \
    else \
        echo "Skipping Intel GPU Drivers for non-amd64 architecture" ; \
    fi

# 启动环境变量
RUN cat <<'EOF' > /rm_entrypoint.sh
#!/bin/bash
set -e

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
fi

if [ -f "/opt/openvino/setupvars.sh" ]; then
    # 官方脚本会解析并移除位置参数，在无参数的函数内加载，保留容器启动命令。
    load_openvino() { source "/opt/openvino/setupvars.sh"; }
    load_openvino
fi

if [ -f "/ros_ws/install/setup.bash" ]; then
    source "/ros_ws/install/setup.bash"
fi

exec "$@"
EOF
RUN chmod +x /rm_entrypoint.sh
ENTRYPOINT ["/rm_entrypoint.sh"]

# 允许 Git 信任挂载的开发目录
RUN git config --global --add safe.directory /ros_ws
