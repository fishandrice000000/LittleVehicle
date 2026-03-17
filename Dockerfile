# 使用 ROS 2 Humble 作为基础镜像
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# 1. 基础系统依赖
RUN apt-get update && apt-get install -y \
    git wget flex bison gperf python3 python3-pip python3-venv cmake \
    ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 \
    udev python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 2. 部署 ESP-IDF v5.1.1
ENV IDF_PATH=/opt/esp/idf
ENV IDF_TOOLS_PATH=/opt/esp/tools
RUN mkdir -p /opt/esp && \
    git clone -b v5.1.1 --recursive  https://gh.927223.xyz/https://github.com/espressif/esp-idf.git $IDF_PATH && \
    $IDF_PATH/install.sh esp32s3

# 3. 为 ESP-IDF 的虚拟环境注入编译 micro-ROS 固件所需的 Python 依赖
RUN bash -c "source $IDF_PATH/export.sh && pip3 install catkin_pkg lark-parser empy==3.3.4 colcon-common-extensions"

# 4. 配置环境变量别名
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "alias get_idf='source /opt/ros/humble/setup.bash && source $IDF_PATH/export.sh'" >> ~/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]