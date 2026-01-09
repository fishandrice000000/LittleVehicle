#!/usr/bin/env python3
import rclpy
import time
from esp32_vehicle_control.base_motion import BaseMotionNode

# 测试小车以 0.5m/s 的速度直线行驶 1m。

def main():
    rclpy.init()
    
    # 初始化节点
    node = BaseMotionNode()
    
    try:
        print("Waiting for Odometry...")
        # 必须先等待里程计数据，确保 micro-ROS 连接正常且数据已上报
        if not node.wait_for_odom(timeout_sec=10.0):
            print("Error: Could not get Odometry data. Check micro-ROS agent connection.")
            return

        print("Start: Moving Straight 1.0m at 0.5m/s")
        
        # 调用 move_distance
        # distance=1.0 (米), speed=0.5 (米/秒)
        node.move_distance(distance=1.0, speed=0.5)
        
        print("Done.")

    except KeyboardInterrupt:
        print("Test interrupted by user.")
    finally:
        # 确保退出时停车
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()