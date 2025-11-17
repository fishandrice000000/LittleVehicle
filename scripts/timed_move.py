#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


def main():
    rclpy.init()
    node = Node('simple_timed_move')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # ====== 在这里配置你想要的小车运动 ======
    vx = 1.0        # 线速度 (m/s)，正数前进，负数后退
    wz = 0.0        # 角速度 (rad/s)，正数左转，负数右转
    duration = 10.0  # 持续时间 (秒)
    rate = 20.0     # 发布频率 (Hz)
    # =====================================

    twist = Twist()
    twist.linear.x = vx
    twist.angular.z = wz

    dt = 1.0 / rate
    start = time.time()
    now = start

    node.get_logger().info(
        f'Start move: vx={vx:.2f} m/s, wz={wz:.2f} rad/s, duration={duration:.2f} s'
    )

    # 在 duration 秒内持续往 /cmd_vel 发速度
    while rclpy.ok() and (now - start) < duration:
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(dt)
        now = time.time()

    # 时间到了，发一次停止命令
    stop = Twist()
    pub.publish(stop)
    node.get_logger().info('Motion finished, stop cmd sent')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
