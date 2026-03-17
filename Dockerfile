# 必须使用 v5.2 甚至更高版本，以确保底层是 Ubuntu 22.04
FROM espressif/idf:v5.2.1

ENV DEBIAN_FRONTEND=noninteractive

# 1. 基础系统设置 (ROS 2 强依赖正确的 locale 和 tzdata 设置)
RUN apt-get update && apt-get install -y \
    locales curl software-properties-common tzdata \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# 2. 添加 ROS 2 官方 GPG 密钥和 apt 源 (明确指定 jammy)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装 ROS 2 Humble (如果不需要 Rviz/Gazebo，可以将 desktop 换成 ros-humble-ros-base 以大幅缩小镜像体积) 
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-pip \
    libusb-1.0-0 \
    udev \
    && rm -rf /var/lib/apt/lists/*

# 4. 在 ESP-IDF 的 Python 虚拟环境中注入 micro-ROS 编译依赖
# 注意：官方 espressif/idf 镜像中，IDF_PATH 默认配置为 /opt/esp/idf
RUN bash -c "source /opt/esp/idf/export.sh && pip3 install catkin_pkg lark-parser empy==3.3.4 colcon-common-extensions"

# 5. 配置环境变量别名
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "alias get_idf='source /opt/ros/humble/setup.bash && source /opt/esp/idf/export.sh'" >> /root/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]