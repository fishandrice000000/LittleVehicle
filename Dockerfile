# --------------------------------------------------------
# 基础镜像: ROS 2 Humble Desktop (包含 RViz2 和相关的 GUI 依赖)
# --------------------------------------------------------
FROM osrf/ros:humble-desktop

# 设置非交互式安装，防止 tzdata 等包安装时卡住
ENV DEBIAN_FRONTEND=noninteractive

# --------------------------------------------------------
# 1. 安装基础工具、ESP-IDF 前置依赖以及 Gazebo 相关的 ROS 包
# --------------------------------------------------------
RUN apt-get update && apt-get install -y \
    git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 \
    ros-humble-gazebo-ros-pkgs ros-humble-xacro \
    iputils-ping net-tools usbutils \
    nano vim curl \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------
# 2. 安装与配置 ESP-IDF 工具链
# --------------------------------------------------------
ENV IDF_PATH=/opt/esp/idf
ENV IDF_TOOLS_PATH=/opt/esp/tools

RUN mkdir -p /opt/esp && \
    cd /opt/esp && \
    # 克隆 ESP-IDF (此处选择 v5.1.2，是当前 micro-ROS 兼容性极好且稳定的版本)
    git clone -b v5.1.2 --recursive https://github.com/espressif/esp-idf.git idf 

RUN cd /opt/esp/idf && \
    # 运行安装脚本 (all 表示安装支持所有芯片的工具链，如 esp32, esp32s3, esp32c3 等)
    ./install.sh all

# --------------------------------------------------------
# 3. 安装 micro-ROS 和 ROS2 开发可能用到的 Python 拓展
# --------------------------------------------------------
RUN pip3 install catkin_pkg lark-parser empy colcon-common-extensions pyserial

# --------------------------------------------------------
# 4. 配置终端环境
# --------------------------------------------------------
# 将 ROS 2 和 ESP-IDF 的环境变量注入到 bashrc 中
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    # 创建一个快捷指令 `get_idf` 用于激活 ESP-IDF 环境 (不默认激活，防止与 ROS colcon 冲突)
    echo "alias get_idf='. /opt/esp/idf/export.sh'" >> /root/.bashrc && \
    # 自动 source ROS 2 工作空间（假设你未来在 /workspace 编译）
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc

# 设置你的工作目录
WORKDIR /workspace

# 启动默认命令
CMD ["/bin/bash"]