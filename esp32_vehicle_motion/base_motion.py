# esp32_car_control/base_motion.py
import time
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """
    从四元数中计算 yaw（绕 Z 轴角度，rad）
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class BaseMotionNode(Node):
    """
    ESP32 小车的高层运动控制封装节点。

    提供的主要接口：
      - move_for(vx, wz, duration): 以给定线速度/角速度跑一段时间（不依赖里程计）
      - move_distance(distance, speed): 走指定距离（依赖 /odom）
      - rotate_angle(angle_rad, angular_speed): 转指定角度（依赖 /odom）
      - follow_waypoints(waypoints): 按序列点依次走（简单版本）
    """

    def __init__(self,
                 cmd_vel_topic: str = '/cmd_vel',
                 odom_topic: str = '/odom'):
        super().__init__('esp32_base_motion')

        # 发布速度命令
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, 10
        )

        self._last_odom: Optional[Odometry] = None

    # ========== 基础工具函数 ==========

    def _odom_callback(self, msg: Odometry):
        self._last_odom = msg

    def _wait_for_odom(self, timeout_sec: float = 2.0) -> bool:
        """
        等待第一次 /odom 消息到来，超时返回 False。
        """
        start = time.time()
        while rclpy.ok() and self._last_odom is None and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._last_odom is not None

    def _publish_twist(self, vx: float, wz: float):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_pub.publish(msg)

    def stop(self):
        """
        立即发送 0 速度。
        """
        self._publish_twist(0.0, 0.0)

    # ========== 高层 API 1：按时间运动（不依赖里程计） ==========

    def move_for(self,
                 vx: float,
                 wz: float,
                 duration: float,
                 rate_hz: float = 50.0):
        """
        以给定线速度/角速度运动 duration 秒。
        不依赖 /odom，适合你只想“按时间粗略跑一下”的场景。

        vx: 线速度 m/s（>0 前进，<0 后退）
        wz: 角速度 rad/s（>0 左转，<0 右转）
        duration: 持续时间（秒）
        rate_hz: 发布 /cmd_vel 的频率
        """
        # 简单限幅，你可以根据车子能力再调
        vx = max(min(vx, 1.0), -1.0)
        wz = max(min(wz, 5.0), -5.0)

        dt = 1.0 / rate_hz
        start = time.time()
        now = start

        self.get_logger().info(
            f'[move_for] vx={vx:.3f}, wz={wz:.3f}, duration={duration:.3f}s'
        )

        while rclpy.ok() and (now - start) < duration:
            self._publish_twist(vx, wz)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)
            now = time.time()

        self.stop()
        self.get_logger().info('[move_for] done, stop sent')

    # ========== 高层 API 2：走指定路程（依赖里程计） ==========

    def move_distance(self,
                      distance: float,
                      speed: float = 0.2,
                      rate_hz: float = 50.0,
                      wait_odom: bool = True):
        """
        沿车体前进方向走指定距离（m），基于 /odom 闭环。

        distance > 0: 向前；distance < 0: 向后。
        speed: 标称线速度大小（m/s），符号由 distance 决定。
        """
        if wait_odom and not self._wait_for_odom():
            self.get_logger().error('[move_distance] No odom received, abort')
            return

        if self._last_odom is None:
            self.get_logger().error('[move_distance] No odom at all, abort')
            return

        # 起点坐标
        start_odom = self._last_odom
        sx = start_odom.pose.pose.position.x
        sy = start_odom.pose.pose.position.y

        direction = 1.0 if distance >= 0.0 else -1.0
        speed = abs(speed) * direction
        target_dist = abs(distance)

        dt = 1.0 / rate_hz

        self.get_logger().info(
            f'[move_distance] distance={distance:.3f}m, speed={speed:.3f}m/s'
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if self._last_odom is None:
                self.get_logger().warn('[move_distance] no odom yet, keep waiting')
                time.sleep(dt)
                continue

            cur = self._last_odom.pose.pose.position
            dx = cur.x - sx
            dy = cur.y - sy
            traveled = math.hypot(dx, dy)

            if traveled >= target_dist:
                break

            self._publish_twist(speed, 0.0)
            time.sleep(dt)

        self.stop()
        self.get_logger().info('[move_distance] done, stop sent')

    # ========== 高层 API 3：原地旋转（依赖里程计） ==========

    def rotate_angle(self,
                     angle_rad: float,
                     angular_speed: float = 0.8,
                     rate_hz: float = 50.0,
                     wait_odom: bool = True):
        """
        原地旋转指定角度（rad），基于 /odom 闭环。

        angle_rad > 0: 左转；angle_rad < 0: 右转。
        angular_speed: 角速度大小（rad/s），符号由 angle_rad 决定。
        """
        if wait_odom and not self._wait_for_odom():
            self.get_logger().error('[rotate_angle] No odom received, abort')
            return

        if self._last_odom is None:
            self.get_logger().error('[rotate_angle] No odom at all, abort')
            return

        start_odom = self._last_odom
        q = start_odom.pose.pose.orientation
        start_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        direction = 1.0 if angle_rad >= 0.0 else -1.0
        speed = abs(angular_speed) * direction
        target_angle = abs(angle_rad)

        dt = 1.0 / rate_hz

        self.get_logger().info(
            f'[rotate_angle] angle={angle_rad:.3f}rad, wz={speed:.3f}rad/s'
        )

        def angle_diff(a, b):
            # 归一化到 [-pi, pi]
            d = a - b
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            return abs(d)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if self._last_odom is None:
                self.get_logger().warn('[rotate_angle] no odom yet, keep waiting')
                time.sleep(dt)
                continue

            q = self._last_odom.pose.pose.orientation
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            diff = angle_diff(yaw, start_yaw)

            if diff >= target_angle:
                break

            self._publish_twist(0.0, speed)
            time.sleep(dt)

        self.stop()
        self.get_logger().info('[rotate_angle] done, stop sent')

    # ========== 高层 API 4：简单跟随一串航点（粗糙版） ==========

    def follow_waypoints(self,
                          waypoints: List[Tuple[float, float]],
                          forward_speed: float = 0.2,
                          angular_speed: float = 0.8,
                          pos_tolerance: float = 0.05,
                          yaw_tolerance: float = 0.05):
        """
        粗糙版：在 odom 平面坐标系下依次跟随一串 (x, y) 航点。
        只用“转向 + 直走”的分步方式，日后你可以替换成更高级的控制器。

        waypoints: [(x1, y1), (x2, y2), ...]，单位 m
        """
        if not self._wait_for_odom():
            self.get_logger().error('[follow_waypoints] No odom received, abort')
            return

        for idx, (tx, ty) in enumerate(waypoints):
            self.get_logger().info(f'[follow_waypoints] Go to waypoint {idx}: ({tx:.3f}, {ty:.3f})')

            # 读取当前位姿
            if self._last_odom is None:
                self.get_logger().error('[follow_waypoints] odom lost, abort')
                return

            start = self._last_odom
            px = start.pose.pose.position.x
            py = start.pose.pose.position.y
            q = start.pose.pose.orientation
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

            # 先转向目标点方向
            dx = tx - px
            dy = ty - py
            target_yaw = math.atan2(dy, dx)
            delta_yaw = target_yaw - yaw
            # 归一化
            while delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            while delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi

            self.rotate_angle(delta_yaw, angular_speed=angular_speed, wait_odom=True)

            # 再直线走过去
            distance = math.hypot(dx, dy)
            # 留一点富余，靠容差终止
            self.move_distance(distance, speed=forward_speed, wait_odom=True)

            # 简单检查是否在容差范围内
            rclpy.spin_once(self, timeout_sec=0.0)
            if self._last_odom is None:
                continue
            cur = self._last_odom.pose.pose.position
            err = math.hypot(cur.x - tx, cur.y - ty)
            self.get_logger().info(f'[follow_waypoints] waypoint {idx} reached, pos_err={err:.3f}m')

        self.stop()
        self.get_logger().info('[follow_waypoints] all waypoints done')
