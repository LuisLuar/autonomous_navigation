#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import LaneModel
import csv
import os

class UnifiedLaneLogger(Node):
    def __init__(self):
        super().__init__('unified_lane_logger')
        
        # Nombre del archivo único
        self.filename = 'lane_comparison_data4.csv'
        
        # Inicializar el CSV con encabezados claros
        self.init_csv()

        # Suscriptores
        self.create_subscription(LaneModel, '/lane/model_raw', self.cb_raw, 10)
        self.create_subscription(LaneModel, '/lane/model_filtered', self.cb_filtered, 10)

        self.get_logger().info(f"Registrando datos en: {self.filename}")

    def init_csv(self):
        with open(self.filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            # 'source' indicará si es de la cámara (RAW) o del EKF (FILTERED)
            writer.writerow(['timestamp', 'source', 'd_lat', 'yaw', 'curvature', 'confidence'])

    def cb_raw(self, msg):
        # Convertir stamp a float (segundos.nanosegundos)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.save_row(t, 'RAW', msg.d_lat, msg.yaw, msg.curvature, msg.confidence)

    def cb_filtered(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.save_row(t, 'FILTERED', msg.d_lat, msg.yaw, msg.curvature, msg.confidence)

    def save_row(self, t, source, d, y, k, c):
        with open(self.filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([t, source, d, y, k, c])

def main():
    rclpy.init()
    node = UnifiedLaneLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()