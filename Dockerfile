# 使用 ROS 2 Humble 作为基础镜像
FROM osrf/ros:humble-desktop

# 设置环境变量，避免交互式安装时的提示
ENV DEBIAN_FRONTEND=noninteractive

# 1. 安装 ESP-IDF 和 micro-ROS 所需的基础依赖
RUN apt-get update && apt-get install -y \
    git \
    wget \
    flex \
    bison \
    gperf \
    python3 \
    python3-pip \
    python3-venv \
    cmake \
    ninja-build \
    ccache \
    libffi-dev \
    libssl-dev \
    dfu-util \
    libusb-1.0-0 \
    udev \
    && rm -rf /var/lib/apt/lists/*

# 2. 安装 ESP-IDF v5.1.1
RUN mkdir -p /opt/esp && cd /opt/esp && \
    git clone --recursive --branch v5.1.1 https://github.com/espressif/esp-idf.git && \
    cd esp-idf && \
    ./install.sh esp32s3 && \
    echo "source /opt/esp/esp-idf/export.sh" >> /etc/bash.bashrc

# 3. 配置 micro-ROS Agent (ROS 2 环境)
RUN mkdir -p /root/microros_ws/src && cd /root/microros_ws && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    . install/local_setup.sh && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    echo "source /root/microros_ws/install/local_setup.sh" >> /etc/bash.bashrc

# 4. 在容器构建阶段为 ESP-IDF 的 venv 安装好依赖
RUN bash -c ". /opt/esp/esp-idf/export.sh && pip3 install catkin_pkg lark-parser empy==3.3.4 colcon-common-extensions"

# 5. 设置工作目录
WORKDIR /workspace

# 默认启动 bash
CMD ["/bin/bash"]
