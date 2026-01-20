#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticStatus


class LidarSpeedLimiter(Node):

    def __init__(self):
        super().__init__('lidar_speed_limiter')

        # ---------- Subscriber ----------
        self.sub_scan = self.create_subscription(LaserScan,'/scan',self.scan_callback,1)
        self.create_subscription(DiagnosticStatus, '/status/rplidar', self.rplidar_status_cb, 10)

        # ---------- Publisher ----------
        self.pub_alpha = self.create_publisher(Float32,'/alpha/lidar',1)
        self.pub_active = self.create_publisher(Bool,'/active/lidar_front',1)

        # ---------- Parameters ----------
        self.front_angle = np.deg2rad(30.0)
        self.side_angle  = np.deg2rad(60.0)

        self.front_stop_dist = 1.50
        self.front_slow_dist = 4.50

        self.side_stop_dist = 1.0
        self.side_slow_dist = 2.0

        self.range_max = 8.0  # RPLIDAR A1 típico

        # ---------- State ----------
        self.rplidar_status = 2  # Inicialmente error
        #self.get_logger().info("Lidar speed limiter ready")

    # =====================================================
    def rplidar_status_cb(self, msg: DiagnosticStatus):
        self.rplidar_status = self.get_message_level(msg)
        active = True 
        if self.rplidar_status == 2:
            active = False

        msg_active = Bool()
        msg_active.data = active
        self.pub_active.publish(msg_active)


    def scan_callback(self, scan: LaserScan):
        active = True
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges)

        # Filtrar inválidos
        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) < 50 or self.rplidar_status == 2:
            active = False

        msg_active = Bool()
        msg_active.data = active
        self.pub_active.publish(msg_active)

        if not active:
            return


        # Limitar rango máximo
        ranges = np.clip(ranges, 0.0, self.range_max)

        # ---------- Sectores ----------
        front_mask = np.abs(angles) <= self.front_angle
        left_mask  = (angles > self.front_angle) & (angles <= self.side_angle)
        right_mask = (angles < -self.front_angle) & (angles >= -self.side_angle)

        alpha_vals = []

        # ---------- Frontal ----------
        if np.any(front_mask):
            d_front = np.min(ranges[front_mask])
            alpha_front = self.compute_alpha(
                d_front,
                self.front_stop_dist,
                self.front_slow_dist
            )
            alpha_vals.append(alpha_front)

        # ---------- Lateral ----------
        for mask in [left_mask, right_mask]:
            if np.any(mask):
                d_side = np.min(ranges[mask])
                alpha_side = self.compute_alpha(
                    d_side,
                    self.side_stop_dist,
                    self.side_slow_dist
                )
                alpha_vals.append(alpha_side)

        alpha = min(alpha_vals) if alpha_vals else 1.0

        msg = Float32()
        msg.data = float(alpha)
        self.pub_active.publish(Bool(data=True))
        self.pub_alpha.publish(msg)

    # =====================================================

    def get_message_level(self, msg):
        """Extrae el nivel de un mensaje de forma segura."""
        level_raw = msg.level
        if isinstance(level_raw, bytes):
            return int.from_bytes(level_raw, byteorder='little', signed=False)
        else:
            try:
                return int(level_raw)
            except Exception:
                return 2
            
    @staticmethod
    def compute_alpha(d, d_stop, d_slow):
        if d <= d_stop:
            return 0.0
        if d >= d_slow:
            return 1.0
        return (d - d_stop) / (d_slow - d_stop)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarSpeedLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass 


if __name__ == '__main__':
    main()
