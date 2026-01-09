#!/usr/bin/env python3
import rclpy
import math
from esp32_vehicle_control.base_motion import BaseMotionNode

# 测试小车以 0.5m/s 的速度沿半径为 0.5m 的圆行驶一圈。

def main():
    rclpy.init()
    node = BaseMotionNode()
    
    try:
        print("Waiting for Odometry...")
        if not node.wait_for_odom(timeout_sec=10.0):
            print("Error: No Odom.")
            return

        # 参数计算
        radius = 0.5            # 半径 0.5m
        speed = 0.5             # 线速度 0.5m/s
        
        # 计算行驶一圈所需的弧长：L = 2 * pi * r
        arc_length = 2 * math.pi * radius
        
        print(f"Start: Moving in a circle (R={radius}m) for {arc_length:.2f}m length")
        
        # radius > 0 代表左转圆，radius < 0 代表右转圆
        node.move_arc(radius=radius, distance=arc_length, speed=speed)
        
        print("Done.")

    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()