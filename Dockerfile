# 使用 ROS 2 Humble 作为基础镜像
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# 1. 基础系统依赖
RUN apt-get update && apt-get install -y \
    git wget flex bison gperf python3 python3-pip python3-venv cmake \
    ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 \
    udev python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 2. 部署 ESP-IDF v5.1.1 (使用官方默认路径)
# 容器内的 root 用户默认 HOME 是 /root，因此相当于官方教程的 ~/esp/esp-idf
ENV IDF_PATH=/root/esp/esp-idf
RUN mkdir -p /root/esp && \
    git clone -b v5.1.1 --recursive https://gitproxy.mrhjx.cn/https://github.com/espressif/esp-idf.git $IDF_PATH && \
    $IDF_PATH/install.sh esp32s3

# 3. 为 ESP-IDF 的虚拟环境注入编译 micro-ROS 固件所需的 Python 依赖
RUN bash -c "source $IDF_PATH/export.sh && pip3 install catkin_pkg lark-parser empy==3.3.4 colcon-common-extensions"

# 4. 配置环境变量别名
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "alias get_idf='source /opt/ros/humble/setup.bash && source $IDF_PATH/export.sh'" >> /root/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]