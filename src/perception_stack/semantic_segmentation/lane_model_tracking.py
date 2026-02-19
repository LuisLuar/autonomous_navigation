#!/usr/bin/env python3
"""
LaneTrackerEKF - Versión Profesional (Paper-aligned)
---------------------------------------------------
Implementa el modelo cinemático de Dickmanns:
x = [d_lat, psi, kappa]^T
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Odometry
from custom_interfaces.msg import LaneModel
from std_msgs.msg import Header

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from sensor_msgs.msg import PointField
import struct


class LaneTrackerEKF(Node):

    def __init__(self):
        super().__init__('lane_tracker_ekf')

        # ---------------- Parámetros de Ruido (Sintonización) ----------------
        self.declare_parameters('', [
            ('q_d_lat', 0.001),      # Ruido de proceso
            ('q_psi', 0.001),
            ('q_kappa', 0.00001),
            ('r_d_lat', 0.1),      # Ruido de medición (Confianza cámara)
            ('r_psi', 0.01),
            ('r_kappa', 0.08),
        ])

        # ---------------- Estado del Filtro (x) ----------------
        # x = [d_lat, psi, kappa]
        self.x = np.zeros((3, 1)) 
        self.P = np.eye(3) * 0.5 

        # Variables de Odometría
        self.v = 0.0      # v_x
        self.omega = 0.0  # yaw_rate
        self.last_time = self.get_clock().now()

        # ROS
        self.create_subscription(Odometry, '/odometry/local', self.cb_odom, 10)
        self.create_subscription(LaneModel, '/lane/model_raw', self.cb_lane_measurement, 10)
        self.pub_filtered = self.create_publisher(LaneModel, '/lane/model_filtered', 10)

        # DEBUG
        self.pub_centerline_pc = self.create_publisher(PointCloud2,'/lane/centerline_pc',10)

        # Predicción a alta frecuencia (30Hz)
        self.timer = self.create_timer(0.03333333, self.predict_step)

        self.get_logger().info("LaneTracker EKF: Modelo de Dickmanns Activado.")

    def cb_odom(self, msg):
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def predict_step(self):
        """ 
        PREDICCIÓN ASÍNCRONA (Dead Reckoning)
        Basado en x_k+1 = F*x_k + B*u
        """
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0: return

        v = self.v
        w = self.omega

        # 1. Matriz de Transición de Estado (F) - Refleja acoplamiento v*kappa
        # d_lat = d_lat + v*dt*psi
        # psi   = psi + v*dt*kappa - w*dt
        # kappa = kappa
        F = np.array([
            [1.0,  v*dt,  0.0],
            [0.0,  1.0,   v*dt],
            [0.0,  0.0,   1.0]
        ])

        # 2. Vector de control externo (u)
        # El giro del robot (omega) es una entrada externa al carril
        u = np.array([
            [0.0],
            [-w * dt],
            [0.0]
        ])

        # 3. Predicción de Estado
        self.x = F @ self.x + u

        # 4. Predicción de Covarianza (P = FPF' + Q)
        Q = np.diag([
            self.get_parameter('q_d_lat').value,
            self.get_parameter('q_psi').value,
            self.get_parameter('q_kappa').value
        ])
        self.P = F @ self.P @ F.T + Q

        self.publish_state()

    def cb_lane_measurement(self, msg):
        """ 
        CORRECCIÓN (Update)
        Llega cuando la visión detecta algo (10-15Hz)
        """
        # Ignorar si la confianza es nula
        if msg.confidence < 0.01: return

        # Vector de medición z
        z = np.array([
            [msg.d_lat],
            [msg.yaw],
            [msg.curvature]
        ])

        # Matriz de observación H (Identidad porque el Nodo 1 ya entrega el estado)
        H = np.eye(3)

        # Ruido de medición R (ajustado por confianza)
        R = np.diag([
            self.get_parameter('r_d_lat').value,
            self.get_parameter('r_psi').value,
            self.get_parameter('r_kappa').value
        ]) / (msg.confidence + 0.0001)

        # Ganancia de Kalman
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Actualización
        y = z - H @ self.x # Residual
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

    def publish_centerline_pointcloud(self):
        """
        Publica la línea central del carril estimado como PointCloud2
        (solo debug, basado en el estado EKF)
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_footprint"

        d_lat = self.x[0, 0]
        yaw = self.x[1, 0]
        curvature = self.x[2, 0]

        xs = np.linspace(0.0, 8.0, 50)  # horizonte visual
        points = []

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        r, g, b = 0, 255, 0  # Verde

        for x in xs:
            y = (
                d_lat +
                math.tan(yaw) * x +
                0.5 * curvature * x**2
            )
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
            points.append([float(x), float(y), 0.0, rgb])
        

        cloud = point_cloud2.create_cloud(header, fields, points)
        self.pub_centerline_pc.publish(cloud)

    def publish_state(self):
        m = LaneModel()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_footprint"
        m.d_lat = float(self.x[0, 0])
        m.yaw = float(self.x[1, 0])
        m.curvature = float(self.x[2, 0])
        m.confidence = 1.0
        self.pub_filtered.publish(m)

        # Debug visual: centerline como PointCloud2
        self.publish_centerline_pointcloud()

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = LaneTrackerEKF()
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