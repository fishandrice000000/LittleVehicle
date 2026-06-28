#!/usr/bin/env python3
import rclpy
import math
import sys
import os
import argparse

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, '..'))
sys.path.append(project_root)
from esp32_vehicle_control.base_motion import BaseMotionNode

def main():
    # === 解析命令行参数 ===
    parser = argparse.ArgumentParser(description='Test circular motion.')
    parser.add_argument('-r', '--radius', type=float, default=0.5, help='Circle radius in meters (default: 0.5)')
    parser.add_argument('-s', '--speed', type=float, default=0.25, help='Linear speed in m/s (default: 0.25)')
    args = parser.parse_args()

    rclpy.init()
    node = BaseMotionNode()
    
    try:
        print("Waiting for Odometry...")
        if not node.wait_for_odom(timeout_sec=10.0):
            print("Error: No Odom.")
            return

        # 计算参数
        radius = args.radius
        speed = args.speed
        
        # 计算行驶一圈所需的弧长：L = 2 * pi * r
        arc_length = 2 * math.pi * abs(radius)
        
        print(f"Start: Moving in a circle (R={radius}m) for {arc_length:.2f}m length at {speed}m/s")
        
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