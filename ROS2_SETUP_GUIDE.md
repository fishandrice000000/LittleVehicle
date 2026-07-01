# ROS2 上位机 IMU 融合配置指南

ESP32 固件发布 `/odom`（编码器）和 `/imu`（ICM42670P），上位机通过 `robot_localization` 的 EKF 节点融合为 `/odom_filtered`。

## 前提

- ROS2 Humble 已安装
- micro-ROS Agent 已启动
- `esp32_vehicle_control` 模块可用（已默认订阅 `/odom_filtered`）

## 1. 安装 robot_localization

```bash
sudo apt install ros-humble-robot-localization
```

## 2. 创建 EKF 配置文件

```bash
mkdir -p ~/LittleVehicle/config
```

创建 `~/LittleVehicle/config/ekf.yaml`：

```yaml
### ekf.yaml — EKF 融合 /odom + /imu → /odom_filtered

ekf_filter_node:
  ros__parameters:
    frequency: 30.0

    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 传感器 0: 编码器里程计 /odom
    odom0: /odom
    odom0_config:
      [true,  true,  false,   # x, y, z
       false, false, false,    # roll, pitch, yaw
       false, false, false,   # vx, vy, vz
       false, false, false,   # vroll, vpitch, vyaw
       false, false, false]   # ax, ay, az
    odom0_differential: false

    # 传感器 1: IMU /imu
    imu0: /imu
    imu0_config:
      [false, false, false,   # x, y, z
       false, false, false,   # roll, pitch, yaw
       false, false, false,   # vx, vy, vz
       false, false, true,    # vroll, vpitch, vyaw  ← 关键: IMU 角速度修正 yaw
       false, false, false]   # ax, ay, az
    imu0_differential: false

    publish_tf: true
```

## 3. 启动 EKF 节点

不用 launch 文件，直接 `ros2 run`：

```bash
ros2 run robot_localization ekf_node --ros-args \
  --params-file ~/LittleVehicle/config/ekf.yaml \
  -r /odometry/filtered:=/odom_filtered
```

EKF 是常驻服务，开一个终端挂着就行，不用每次关。

## 4. base_motion.py（已修改好）

默认订阅已改为 `/odom_filtered`，测试脚本无需传参：

```python
node = BaseMotionNode()                              # 默认 /odom_filtered
node = BaseMotionNode(odom_topic='/odom')            # 如 EKF 未启动，可临时用纯 odom
```

## 5. 启动与测试

### 终端布局

```
终端1: micro-ROS Agent    (docker run ...)
终端2: EKF 节点           (ros2 run ...  放着不动)
终端3: Python 测试脚本     (每次跑)
```

### 测试

```bash
# 在项目 scripts/ 目录下
python3 -m test_rotate -a 360 -s 1.0
```

### 验证步骤

```bash
ros2 topic list | grep -E 'odom|imu'
# 应看到: /odom, /imu, /odom_filtered

ros2 topic hz /odom_filtered
# 应有稳定输出 (~30Hz)
```

## 故障排查

| 问题 | 可能原因 |
|------|----------|
| `/odom_filtered` 不存在 | EKF 未启动 |
| EKF 报 "dropping topic" | 协方差矩阵全零，ESP32 固件需为非零值 |
| TF 树断裂 | `/odom` frame_id 需为 `"odom"`，`/imu` frame_id 需为 `"imu_link"` |
