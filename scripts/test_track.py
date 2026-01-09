#!/usr/bin/env python3
import rclpy
import math
import time
from esp32_vehicle_control.base_motion import BaseMotionNode

# 测试轨道：直线 0.5m -> 圆弧半圈 (R=0.5m) -> 直线 0.5m -> 圆弧半圈 (R=0.5m) -> 回到起点。全程 0.5m/s。

def main():
    rclpy.init()
    node = BaseMotionNode()
    
    try:
        print("Waiting for Odometry...")
        if not node.wait_for_odom(timeout_sec=10.0):
            print("Error: No Odom.")
            return

        speed = 0.5
        radius = 0.5
        straight_len = 0.5
        # 半圆弧长 = pi * r
        semi_circle_len = math.pi * radius

        print("=== Step 1: Straight 0.5m ===")
        node.move_distance(distance=straight_len, speed=speed)
        time.sleep(0.5) # 动作之间稍作停顿，让惯性消除（可选）

        print("=== Step 2: U-Turn (Semi-circle) ===")
        # radius=0.5 (左转)
        node.move_arc(radius=radius, distance=semi_circle_len, speed=speed)
        time.sleep(0.5)

        print("=== Step 3: Straight 0.5m (Returning) ===")
        node.move_distance(distance=straight_len, speed=speed)
        time.sleep(0.5)

        print("=== Step 4: U-Turn (Semi-circle to Start) ===")
        node.move_arc(radius=radius, distance=semi_circle_len, speed=speed)
        
        print("Test Complete. Should be back at start.")

    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()