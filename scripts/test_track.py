#!/usr/bin/env python3
import rclpy
import math
import time
import sys
import os
import argparse

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, '..'))
sys.path.append(project_root)
from esp32_vehicle_control.base_motion import BaseMotionNode

def main():
    # === 解析命令行参数 ===
    parser = argparse.ArgumentParser(description='Test track (oval) motion.')
    parser.add_argument('-l', '--length', type=float, default=0.5, help='Length of straight section in meters (default: 0.5)')
    parser.add_argument('-r', '--radius', type=float, default=0.5, help='Radius of turn in meters (default: 0.5)')
    parser.add_argument('-s', '--speed', type=float, default=0.25, help='Speed in m/s (default: 0.25)')
    args = parser.parse_args()

    rclpy.init()
    node = BaseMotionNode()
    
    try:
        print("Waiting for Odometry...")
        if not node.wait_for_odom(timeout_sec=10.0):
            print("Error: No Odom.")
            return

        speed = args.speed
        radius = args.radius
        straight_len = args.length
        # 半圆弧长 = pi * r
        semi_circle_len = math.pi * abs(radius)

        print(f"Config: Speed={speed}m/s, Straight={straight_len}m, Radius={radius}m")

        print("=== Step 1: Straight ===")
        node.move_distance(distance=straight_len, speed=speed)
        time.sleep(0.5)

        print("=== Step 2: U-Turn (Semi-circle) ===")
        node.move_arc(radius=radius, distance=semi_circle_len, speed=speed)
        time.sleep(0.5)

        print("=== Step 3: Straight (Returning) ===")
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