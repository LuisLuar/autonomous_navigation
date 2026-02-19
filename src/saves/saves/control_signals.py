#!/usr/bin/env python3
"""
Data Logger Node for ROS2 Humble
Guarda datos de LaneModel, Odometry y Twist en un archivo CSV
"""

import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from custom_interfaces.msg import LaneModel
from std_msgs.msg import Header


class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger')
        
        # ---------------- Parámetros ----------------
        self.declare_parameters('', [
            ('output_file', 'robot_data.csv'),
            ('save_path', ''),
        ])
        
        # Obtener ruta del archivo
        save_path = self.get_parameter('save_path').value
        filename = self.get_parameter('output_file').value
        
        if save_path:
            # Crear directorio si no existe
            os.makedirs(save_path, exist_ok=True)
            self.csv_file = os.path.join(save_path, filename)
        else:
            # Usar directorio actual
            self.csv_file = filename
        
        # ---------------- Variables para almacenar datos ----------------
        self.lane_data = {
            'timestamp': 0.0,
            'd_lat': 0.0,
            'yaw': 0.0,
            'curvature': 0.0,
            'confidence': 0.0
        }
        
        self.odom_data = {
            'timestamp': 0.0,
            'linear_x': 0.0,
            'angular_z': 0.0
        }
        
        self.cmd_vel_data = {
            'timestamp': 0.0,
            'linear_x': 0.0,
            'angular_z': 0.0
        }
        
        # Banderas para saber si tenemos datos nuevos
        self.new_lane_data = False
        self.new_odom_data = False
        self.new_cmd_vel_data = False
        
        # ---------------- Subscriptores ----------------
        self.create_subscription(
            LaneModel, 
            '/lane/model_filtered', 
            self.cb_lane_model, 
            10
        )
        
        self.create_subscription(
            Odometry, 
            '/odometry/local', 
            self.cb_odometry, 
            10
        )
        
        self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cb_cmd_vel, 
            10
        )
        
        # Timer para guardar datos cada 100ms (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Inicializar archivo CSV con headers
        self.init_csv_file()
        
        self.get_logger().info(f"Data Logger iniciado - Guardando en: {self.csv_file}")
        self.get_logger().info("Suscrito a: /lane/model_filtered, /odometry/local, /cmd_vel")

    def init_csv_file(self):
        """Inicializa el archivo CSV con los encabezados"""
        file_exists = os.path.isfile(self.csv_file)
        
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            
            # Si el archivo no existe, escribir encabezados
            if not file_exists or os.path.getsize(self.csv_file) == 0:
                writer.writerow([
                    'timestamp',
                    'lane_d_lat',
                    'lane_yaw',
                    'lane_curvature',
                    'lane_confidence',
                    'odom_linear_x',
                    'odom_angular_z',
                    'cmd_vel_linear_x',
                    'cmd_vel_angular_z'
                ])
                self.get_logger().info("Archivo CSV creado con encabezados")

    def cb_lane_model(self, msg):
        """Callback para datos de LaneModel"""
        # Convertir timestamp a segundos
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        self.lane_data = {
            'timestamp': timestamp,
            'd_lat': msg.d_lat,
            'yaw': msg.yaw,
            'curvature': msg.curvature,
            'confidence': msg.confidence
        }
        self.new_lane_data = True
        
        self.get_logger().debug(f"LaneModel recibido - d_lat: {msg.d_lat:.3f}")

    def cb_odometry(self, msg):
        """Callback para datos de Odometry"""
        # Convertir timestamp a segundos
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        self.odom_data = {
            'timestamp': timestamp,
            'linear_x': msg.twist.twist.linear.x,
            'angular_z': msg.twist.twist.angular.z
        }
        self.new_odom_data = True
        
        self.get_logger().debug(f"Odometry recibido - v: {msg.twist.twist.linear.x:.3f}")

    def cb_cmd_vel(self, msg):
        """Callback para datos de Twist (cmd_vel)"""
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        
        self.cmd_vel_data = {
            'timestamp': timestamp,
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        }
        self.new_cmd_vel_data = True
        
        self.get_logger().debug(f"CmdVel recibido - v: {msg.linear.x:.3f}")

    def timer_callback(self):
        """Timer para guardar datos periódicamente"""
        
        # Solo guardar si tenemos al menos un tipo de dato nuevo
        if not (self.new_lane_data or self.new_odom_data or self.new_cmd_vel_data):
            return
        
        # Usar el timestamp más reciente disponible
        timestamps = []
        if self.new_lane_data:
            timestamps.append(self.lane_data['timestamp'])
        if self.new_odom_data:
            timestamps.append(self.odom_data['timestamp'])
        if self.new_cmd_vel_data:
            timestamps.append(self.cmd_vel_data['timestamp'])
        
        current_time = max(timestamps) if timestamps else self.get_clock().now().nanoseconds * 1e-9
        
        # Guardar en CSV
        try:
            with open(self.csv_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    f"{current_time:.6f}",
                    f"{self.lane_data['d_lat']:.6f}",
                    f"{self.lane_data['yaw']:.6f}",
                    f"{self.lane_data['curvature']:.6f}",
                    f"{self.lane_data['confidence']:.6f}",
                    f"{self.odom_data['linear_x']:.6f}",
                    f"{self.odom_data['angular_z']:.6f}",
                    f"{self.cmd_vel_data['linear_x']:.6f}",
                    f"{self.cmd_vel_data['angular_z']:.6f}"
                ])
            
            # Log cada 10 escrituras aprox (1 segundo)
            if hasattr(self, 'write_count'):
                self.write_count += 1
            else:
                self.write_count = 1
            
            if self.write_count % 10 == 0:
                self.get_logger().info(f"Guardados {self.write_count} registros en CSV")
            
        except Exception as e:
            self.get_logger().error(f"Error guardando en CSV: {e}")
        
        # Reiniciar banderas
        self.new_lane_data = False
        self.new_odom_data = False
        self.new_cmd_vel_data = False

    def destroy_node(self):
        """Limpieza al destruir el nodo"""
        self.get_logger().info(f"Datos guardados en: {self.csv_file}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Deteniendo Data Logger por Ctrl+C")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()