# scripts/test_motion.py
import rclpy
import math
from esp32_vehicle_control.base_motion import BaseMotionNode

def main():
    rclpy.init()

    node = BaseMotionNode()

    R = 0.25
    HALF_ARC = math.pi * R  

    # node.move_distance(1.0, speed=0.2)
    node.move_arc(radius=R, distance=HALF_ARC, speed=0.2)
    # node.move_distance(1.0, speed=0.2)
    node.move_arc(radius=R, distance=HALF_ARC, speed=0.2)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()