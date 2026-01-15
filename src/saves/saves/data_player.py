#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
import csv
import time

import numpy as np
import math

class DataPlayer(Node):
    def __init__(self):
        super().__init__('data_player')

        self.odom_pub = self.create_publisher(Odometry, '/odom/unfiltered', 10)
        self.pose_pub = self.create_publisher(Odometry, '/robot/pose', 10)
        self.imu_pub  = self.create_publisher(Imu, '/imu/unfiltered', 10)
        self.gps_pub  = self.create_publisher(NavSatFix, '/gps/fix', 10)

        self.data = []
        with open('logs/ok1dataset.csv') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.data.append(row)

        self.data.sort(key=lambda x: float(x['stamp']))
        self.index = 0
        self.start_time = time.time()
        self.first_stamp = float(self.data[0]['stamp'])

        self.timer = self.create_timer(0.001, self.play)

        self.get_logger().info('▶️ Reproduciendo datos...')

    def add_z_rotation(self, q, angle_degrees):
        """
        Suma una rotación de 'angle_degrees' grados en el eje Z al cuaternión q
        
        Args:
            q: cuaternión original [x, y, z, w]
            angle_degrees: ángulo en grados
        
        Returns:
            Nuevo cuaternión [x, y, z, w]
        """
        # Convertir ángulo a radianes
        angle_rad = math.radians(angle_degrees)
        
        # Crear cuaternión de rotación en Z
        # Para rotación pura en Z: x=0, y=0, z=sin(θ/2), w=cos(θ/2)
        half_angle = angle_rad / 2.0
        q_rot_z = np.array([
            0.0,                    # x
            0.0,                    # y
            math.sin(half_angle),   # z
            math.cos(half_angle)    # w
        ])
        
        # Multiplicación de cuaterniones (q_resultado = q_rot_z * q)
        # q_rot_z * q
        x1, y1, z1, w1 = q_rot_z
        x2, y2, z2, w2 = q
        
        # Fórmula de multiplicación de cuaterniones
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        
        return np.array([x, y, z, w])

    def play(self):
        if self.index >= len(self.data):
            self.get_logger().info('⏹ Fin del archivo')
            self.timer.cancel()
            return

        row = self.data[self.index]
        now = time.time()
        if now - self.start_time < float(row['stamp']) - self.first_stamp:
            return

        if row['topic'] == 'odom':
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_footprint'

            msg.pose.pose.position.x = float(row['x'])
            msg.pose.pose.position.y = float(row['y'])
            msg.pose.pose.position.z = 0.0

            msg.pose.pose.orientation.x = float(row['qx'])
            msg.pose.pose.orientation.y = float(row['qy'])
            msg.pose.pose.orientation.z = float(row['qz'])
            msg.pose.pose.orientation.w = float(row['qw'])

            msg.twist.twist.linear.x = float(row['vx'])
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = 0.0

            msg.twist.twist.angular.x = 0.0
            msg.twist.twist.angular.y = 0.0
            msg.twist.twist.angular.z = float(row['wz'])

            msg.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.1, 0.0, 0.0,     
                                    0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
                    
            msg.twist.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.1]

            self.odom_pub.publish(msg)

        elif row['topic'] == 'pose':
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_footprint'

            msg.pose.pose.position.x = float(row['x'])
            msg.pose.pose.position.y = float(row['y'])
            msg.pose.pose.position.z = 0.0

            self.pose_pub.publish(msg)

        elif row['topic'] == 'imu':
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            msg.linear_acceleration.x = float(row['ax'])
            msg.linear_acceleration.y = float(row['ay'])
            msg.linear_acceleration.z = float(row['az'])

            """msg.orientation.x = float(row['qx'])
            msg.orientation.y = float(row['qy'])
            msg.orientation.z = float(row['qz'])
            msg.orientation.w = float(row['qw'])"""

            # Crear array con el cuaternión original
            q_original = np.array([
                float(row['qx']),
                float(row['qy']),
                float(row['qz']),
                float(row['qw'])
            ])

            # Aplicar rotación de 90 grados en Z
            q_rotado = self.add_z_rotation(q_original, 0)

            # Asignar de vuelta al mensaje
            msg.orientation.x = q_rotado[0]
            msg.orientation.y = q_rotado[1]
            msg.orientation.z = q_rotado[2]
            msg.orientation.w = q_rotado[3]

            msg.angular_velocity.x = float(row['wx'])
            msg.angular_velocity.y = float(row['wy'])
            msg.angular_velocity.z = float(row['wz'])

            msg.orientation_covariance = [0.0001, 0.0, 0.0,
                                            0.0, 0.0001, 0.0,
                                            0.0, 0.0, 0.001]
            msg.angular_velocity_covariance = [0.001, 0.0, 0.0,
                                                0.0, 0.001, 0.0,
                                                0.0, 0.0, 0.001]
            msg.linear_acceleration_covariance = [0.1, 0.0, 0.0,
                                                    0.0, 0.1, 0.0,
                                                    0.0, 0.0, 0.1]

            self.imu_pub.publish(msg)


        elif row['topic'] == 'gps':
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gnss_frame'  # o el que uses en tu URDF

            msg.latitude  = float(row['lat'])
            msg.longitude = float(row['lon'])
            msg.altitude  = float(row['alt'])
            msg.position_covariance[0] = float(row['cl1'])
            msg.position_covariance[4] = float(row['cl2'])
            msg.position_covariance[8] = float(row['cl3'])

            self.gps_pub.publish(msg)

        self.index += 1

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = DataPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        pass
    finally:        
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
