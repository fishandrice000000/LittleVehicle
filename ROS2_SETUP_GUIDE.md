# ROS2 上位机 IMU 融合配置指南

本文档指导在 ROS2 上位机上配置 EKF 节点，将 ESP32 发布的 `/odom`（编码器）和 `/imu`（ICM42670P）融合为 `/odom_filtered`。

## 前提

- ROS2 Humble 已安装
- micro-ROS Agent 已启动并能收到 `/odom` 和 `/imu` 话题
- 本项目的 `esp32_vehicle_control` 模块已可用

## 1. 安装 robot_localization

```bash
sudo apt install ros-humble-robot-localization
```

## 2. 创建 EKF 配置文件

在 ROS2 功能包中创建 `config/ekf.yaml`：

```yaml
### ekf.yaml — EKF 融合 /odom + /imu → /odom_filtered

ekf_filter_node:
  ros__parameters:
    frequency: 30.0

    # 坐标系
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 传感器 0: 编码器里程计 /odom
    odom0: /odom
    odom0_config:
      # 只融合位置 x, y 和朝向 yaw
      [true,  true,  false,   # x, y, z
       false, false, true,    # roll, pitch, yaw
       false, false, false,   # vx, vy, vz
       false, false, false,   # vroll, vpitch, vyaw
       false, false, false]   # ax, ay, az
    odom0_differential: false

    # 传感器 1: IMU /imu
    imu0: /imu
    imu0_config:
      # 主要融合 vyaw (绕 Z 轴角速度)，可选择融合加速度的 roll/pitch
      [false, false, false,   # x, y, z
       false, false, false,   # roll, pitch, yaw
       false, false, false,   # vx, vy, vz
       false, false, true,    # vroll, vpitch, vyaw  ← 关键: 用 IMU 角速度修正 yaw
       false, false, false]   # ax, ay, az
    imu0_differential: false

    # 初始状态协方差
    initial_state: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # 发布 odom->base_link 的 TF 变换
    publish_tf: true

    # 输出话题
    # 默认: /odometry/filtered, 可重映射为 /odom_filtered
```

## 3. 创建 Launch 文件

创建 `launch/real.robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # EKF 融合节点
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['config/ekf.yaml'],
            remappings=[
                ('/odometry/filtered', '/odom_filtered'),
            ],
        ),

        # RViz2 可视化（可选）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'config/real_robot.rviz'],
        ),
    ])
```

## 4. 修改控制节点订阅 `/odom_filtered`

修改 `base_motion.py` 中 `rotate_angle` 和 `move_distance` 的默认 odom 话题：

```python
class BaseMotionNode(Node):
    def __init__(self,
                 cmd_vel_topic: str = '/cmd_vel',
                 odom_topic: str = '/odom_filtered'):  # ← 改为融合后的话题
        ...
```

或者在调用时不传默认值，显式指定：
```python
node = BaseMotionNode(odom_topic='/odom_filtered')
```

## 5. 启动与测试

### 启动顺序

```bash
# 终端 1: 启动 micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
    microros/micro-ros-agent:humble udp4 --port 8090 -v4

# 终端 2: 启动 EKF + RViz2
cd ~/ros2_workspace
source install/setup.bash
ros2 launch my_robot_gazebo_rviz_imu real.robot.launch.py

# 终端 3: 运行测试
ros2 run path_controller_pkg test_rotate -a 360
```

### 验证步骤

1. 确认话题存在：
   ```bash
   ros2 topic list | grep -E 'odom|imu'
   # 应看到: /odom, /imu, /odom_filtered
   ```

2. 检查发布频率：
   ```bash
   ros2 topic hz /odom
   ros2 topic hz /imu
   ros2 topic hz /odom_filtered
   ```

3. 在 RViz2 中添加 Odometry，分别订阅 `/odom` 和 `/odom_filtered`，观察融合后朝向是否更稳定

4. 运行 `test_rotate 360°`，确认欠转问题消除

## 故障排查

| 问题 | 可能原因 |
|------|----------|
| `/odom_filtered` 无输出 | 检查 `/odom` 和 `/imu` 的 `frame_id`：odom 需为 `"odom"`，imu 需为 `"imu_link"` |
| EKF 报 "dropping topic" | 协方差矩阵全零，ESP32 固件需为非零值 |
| TF 树断裂 | 检查 URDF 中是否定义了 `imu_link` |
