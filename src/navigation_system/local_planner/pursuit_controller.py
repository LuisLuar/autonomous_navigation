#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32


def yaw_from_quaternion(q):
    """Extrae yaw (rad) de quaternion"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # ----------------------------
        # Parámetros
        # ----------------------------
        self.declare_parameter('lookahead_min', 1.5)
        self.declare_parameter('lookahead_max', 6.0)
        self.declare_parameter('lookahead_gain', 1.2)

        self.declare_parameter('omega_max', 1.2)     # rad/s
        self.declare_parameter('omega_filter', 0.85) # suavizado

        self.lookahead_min = self.get_parameter('lookahead_min').value
        self.lookahead_max = self.get_parameter('lookahead_max').value
        self.lookahead_gain = self.get_parameter('lookahead_gain').value

        self.omega_max = self.get_parameter('omega_max').value
        self.alpha = self.get_parameter('omega_filter').value

        # ----------------------------
        # Estado
        # ----------------------------
        self.lane_points = []
        self.robot_pose = None
        self.robot_yaw = 0.0
        self.robot_speed = 0.0

        self.prev_omega = 0.0

        # ----------------------------
        # Subs
        # ----------------------------
        self.create_subscription(
            Path,
            '/lane_local',
            self.lane_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odometry/local',
            self.odom_callback,
            10
        )

        # ----------------------------
        # Pub
        # ----------------------------
        self.omega_pub = self.create_publisher(
            Float32,
            '/pure_pursuit/angular_z',
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("✅ Pure Pursuit Controller iniciado")

    # ==========================================================
    def lane_callback(self, msg: Path):
        self.lane_points = [
            (p.pose.position.x, p.pose.position.y)
            for p in msg.poses
        ]

    def odom_callback(self, msg: Odometry):
        self.robot_pose = msg.pose.pose.position
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_speed = math.hypot(vx, vy)

    # ==========================================================
    def control_loop(self):
        if self.robot_pose is None or len(self.lane_points) < 2:
            return

        # ----------------------------
        # Lookahead adaptativo
        # ----------------------------
        Ld = self.lookahead_gain * self.robot_speed + self.lookahead_min
        Ld = np.clip(Ld, self.lookahead_min, self.lookahead_max)

        target = self.find_lookahead_point(Ld)
        if target is None:
            return

        # ----------------------------
        # Transformar target a marco robot
        # ----------------------------
        dx = target[0] - self.robot_pose.x
        dy = target[1] - self.robot_pose.y

        cos_y = math.cos(self.robot_yaw)
        sin_y = math.sin(self.robot_yaw)

        x_r =  cos_y * dx + sin_y * dy
        y_r = -sin_y * dx + cos_y * dy

        if x_r <= 0.001:
            return

        # ----------------------------
        # Pure Pursuit
        # ----------------------------
        alpha = math.atan2(y_r, x_r)

        # Curvatura
        kappa = (2.0 * math.sin(alpha)) / Ld

        # Velocidad angular
        omega = self.robot_speed * kappa

        # Saturación
        omega = np.clip(omega, -self.omega_max, self.omega_max)

        # Suavizado (CRÍTICO)
        omega = self.alpha * self.prev_omega + (1.0 - self.alpha) * omega
        self.prev_omega = omega

        # ----------------------------
        # Publish
        # ----------------------------
        msg = Float32()
        msg.data = float(omega)
        self.omega_pub.publish(msg)

    # ==========================================================
    def find_lookahead_point(self, Ld):
        rx = self.robot_pose.x
        ry = self.robot_pose.y

        acc_dist = 0.0
        prev = (rx, ry)

        for p in self.lane_points:
            d = math.hypot(p[0] - prev[0], p[1] - prev[1])
            acc_dist += d
            if acc_dist >= Ld:
                return p
            prev = p

        return self.lane_points[-1]


def main():
    rclpy.init()
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
