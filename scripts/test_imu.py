#!/usr/bin/env python3
"""
IMU 精度测试: 手动转动小车, 对比实际转角与 gyro z 积分结果。

用法:
  python3 -m test_imu

操作:
  1. 车放平不动, 启动脚本
  2. 手动将车精确转动目标角度 (如 90°, 180°, 360°)
  3. Ctrl+C 停止, 查看累积 yaw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time


class ImuTest(Node):
    def __init__(self):
        super().__init__('imu_test')
        self.yaw = 0.0
        self.last_time = None
        self.sub = self.create_subscription(Imu, '/imu', self.callback, 10)
        self.get_logger().info(
            'IMU 测试已启动。手动转动小车, 完成后 Ctrl+C 停止'
        )

    def callback(self, msg):
        now = time.time()
        if self.last_time is not None:
            dt = now - self.last_time
            if dt < 0.5:  # 忽略长时间无数据的间隔
                self.yaw += msg.angular_velocity.z * dt
        self.last_time = now


def main():
    rclpy.init()
    node = ImuTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'累积 yaw: {node.yaw:.3f} rad = {math.degrees(node.yaw):.1f}°'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
