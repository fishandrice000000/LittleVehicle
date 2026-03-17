# 使用 ROS 2 Humble 作为基础镜像
FROM osrf/ros:humble-desktop

# 避免交互式提示
ENV DEBIAN_FRONTEND=noninteractive

# 1. 基础系统依赖
RUN apt-get update && apt-get install -y \
    git wget flex bison gperf python3 python3-pip python3-venv cmake \
    ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 \
    udev python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 2. 部署 ESP-IDF v5.1.1 (规范化安装到 /opt/esp)
ENV IDF_PATH=/opt/esp/idf
ENV IDF_TOOLS_PATH=/opt/esp/tools
RUN mkdir -p /opt/esp && \
    git clone -b v5.1.1 --recursive  https://gh.927223.xyz/https://github.com/espressif/esp-idf.git $IDF_PATH && \
    $IDF_PATH/install.sh esp32s3

# 3. 关键修复：向 ESP-IDF 的虚拟环境中注入 micro-ROS 构建所需的 ROS 2 Python 依赖
# (吸纳了你之前 Dockerfile 的正确思路，配合后续环境变量防止 ament 报错)
RUN bash -c "source $IDF_PATH/export.sh && pip3 install catkin_pkg lark-parser empy==3.3.4 colcon-common-extensions"

# 4. 构建 micro-ROS Agent
WORKDIR /uros_ws
RUN bash -c "source /opt/ros/humble/setup.bash && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    rm -rf /var/lib/apt/lists/*"

# 5. 配置环境变量
# 关键：执行 get_idf 时，必须同时激活 ROS 2 和 ESP-IDF 环境！
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /uros_ws/install/local_setup.bash" >> ~/.bashrc && \
    echo "alias get_idf='source /opt/ros/humble/setup.bash && source $IDF_PATH/export.sh'" >> ~/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]