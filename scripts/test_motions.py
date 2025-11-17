# scripts/test_motion.py
import rclpy
from esp32_vehicle_control.base_motion import BaseMotionNode


def main():
    rclpy.init()
    node = BaseMotionNode()

    # 直线 0.5m -> 左转 90° -> 再走 0.5m
    node.move_distance(0.5, speed=1.0)
    node.rotate_angle(1.5708, angular_speed=1.0)
    node.move_distance(0.5, speed=1.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
