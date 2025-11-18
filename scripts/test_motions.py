# scripts/test_motion.py
import rclpy
import math
from esp32_vehicle_control.base_motion import BaseMotionNode

def main():
    rclpy.init()

    node = BaseMotionNode()

    # 建议先等一下 /odom，保险一点
    # node.wait_for_odom(timeout_sec=5.0)

    # 目标：半径 0.25 m 的半圆
    R = 0.25
    FULL_ARC = math.pi * R * 2   # 半圆的弧长 = π * R

    # 直行 1.0 m
    node.move_distance(1.0, speed=1.0)

    # # 第一个半圆
    node.move_arc(radius=R, distance=FULL_ARC, speed=1.0)

    # 直行 1.0 m
    node.move_distance(1.0, speed=1.0)

    # 第二个半圆
    node.move_arc(radius=R, distance=FULL_ARC, speed=1.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
