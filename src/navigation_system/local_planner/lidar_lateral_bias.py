#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
from diagnostic_msgs.msg import DiagnosticStatus


class LidarLateralBias(Node):

    def __init__(self):
        super().__init__('lidar_lateral_bias')

        # ---------- Subscriber ----------
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(DiagnosticStatus, '/status/rplidar', self.rplidar_status_cb, 10)

        self.pub_omega = self.create_publisher(Float32, '/omega/lidar', 10)
        self.pub_active = self.create_publisher(Bool, '/active/lidar_lateral', 10)

        # ---------- Params ----------
        self.side_angle = np.deg2rad(130.0)
        self.front_exclusion = np.deg2rad(30.0)

        self.d_safe = 1.0        # [m]
        self.k_lidar = 0.25      # gain
        self.omega_max = 0.3     # rad/s

        #self.get_logger().info("Lidar lateral bias READY")

    # =====================================================
    def rplidar_status_cb(self, msg: DiagnosticStatus):
        self.rplidar_status = self.get_message_level(msg)
        if self.rplidar_status == 2:
            active = False

        msg_active = Bool()
        msg_active.data = active
        self.pub_active.publish(msg_active)

    def scan_cb(self, scan: LaserScan):

        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges)

        valid = np.isfinite(ranges)
        angles = angles[valid]
        ranges = ranges[valid]

        # Lateral sectors
        left = (angles > self.front_exclusion) & (angles <= self.side_angle)
        right = (angles < -self.front_exclusion) & (angles >= -self.side_angle)

        if not np.any(left) or not np.any(right) or self.rplidar_status == 2:
            self.pub_active.publish(Bool(data=False))
            self.pub_omega.publish(Float32(data=0.0))
            return

        d_left = np.min(ranges[left])
        d_right = np.min(ranges[right])

        if d_left > self.d_safe and d_right > self.d_safe:
            self.pub_active.publish(Bool(data=False))
            self.pub_omega.publish(Float32(data=0.0))
            return

        # -------- Lateral bias --------
        e = (1.0 / max(d_right, 0.01)) - (1.0 / max(d_left, 0.01))
        omega = self.k_lidar * e
        omega = np.clip(omega, -self.omega_max, self.omega_max)

        self.pub_active.publish(Bool(data=True))
        self.pub_omega.publish(Float32(data=float(omega)))

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

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = LidarLateralBias()
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
