#!/usr/bin/env python3
import rclpy
import sys
import os
import argparse

# 添加父目录到路径以便导入模块
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, '..'))
sys.path.append(project_root)
from esp32_vehicle_control.base_motion import BaseMotionNode

def main():
    # === 解析命令行参数 ===
    parser = argparse.ArgumentParser(description='Test straight line motion.')
    parser.add_argument('-d', '--distance', type=float, default=1.0, help='Target distance in meters (default: 1.0)')
    parser.add_argument('-s', '--speed', type=float, default=0.25, help='Target speed in m/s (default: 0.25)')
    args = parser.parse_args()

    rclpy.init()
    node = BaseMotionNode()
    
    try:
        print("Waiting for Odometry...")
        if not node.wait_for_odom(timeout_sec=10.0):
            print("Error: Could not get Odometry data.")
            return

        print(f"Start: Moving Straight {args.distance}m at {args.speed}m/s")
        
        # 使用命令行参数
        node.move_distance(distance=args.distance, speed=args.speed)
        
        print("Done.")

    except KeyboardInterrupt:
        print("Test interrupted by user.")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()