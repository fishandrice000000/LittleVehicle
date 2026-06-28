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
    parser = argparse.ArgumentParser(description='Test rotation accuracy.')
    # 默认转 360 度 (2 * pi)
    parser.add_argument('-a', '--angle', type=float, default=360.0, help='Rotation angle in degrees (default: 360)')
    parser.add_argument('-s', '--speed', type=float, default=1.0, help='Angular speed in rad/s (default: 1.0)')
    args = parser.parse_args()

    rclpy.init()
    node = BaseMotionNode()
    
    try:
        print("Waiting for Odometry...")
        if not node.wait_for_odom(timeout_sec=10.0):
            print("Error: No Odom.")
            return

        target_rad = math.radians(args.angle)
        print(f"Start: Rotating {args.angle} degrees ({target_rad:.2f} rad) at {args.speed} rad/s")
        
        # 地面标记起点方向，或者车头贴个胶带指向地板纹路
        node.rotate_angle(angle_rad=target_rad, angular_speed=args.speed)
        
        print("Done. Please check the heading error.")

    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()