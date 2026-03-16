# 采用 ROS2 Humble 桌面版官方基础镜像 (基于 Ubuntu 22.04)
FROM osrf/ros:humble-desktop

# 避免 apt 安装时出现时区交互提示
ENV DEBIAN_FRONTEND=noninteractive

# 1. 安装 ESP-IDF 和 micro-ROS 需要的基础系统依赖
RUN apt-get update && apt-get install -y \
    git wget flex bison gperf python3 python3-pip python3-venv cmake \
    ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 2. 部署 ESP-IDF v5.1.1 (针对 ESP32-S3 进行精简安装以节省空间)
ENV IDF_PATH=/opt/esp/idf
ENV IDF_TOOLS_PATH=/opt/esp/tools
RUN mkdir -p /opt/esp && \
    git clone -b v5.1.1 --recursive  https://gh.927223.xyz/https://github.com/espressif/esp-idf.git $IDF_PATH && \
    $IDF_PATH/install.sh esp32s3

# 3. 部署 micro-ROS 工作空间并构建 micro-ROS Agent
WORKDIR /uros_ws
# 使用 bash -c 以便使用 source 命令
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    rm -rf /var/lib/apt/lists/*"

# 4. 配置环境变量
# 默认进入容器即加载 ROS2 和 micro-ROS 环境
# ESP-IDF 环境通过快捷命令 `get_idf` 随时按需激活，防止与 ROS 的 CMake 冲突
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /uros_ws/install/local_setup.bash" >> ~/.bashrc && \
    echo "alias get_idf='. $IDF_PATH/export.sh'" >> ~/.bashrc

# 设置默认工作目录 (后续把宿主机项目代码映射到此处)
WORKDIR /workspace
CMD ["/bin/bash"]