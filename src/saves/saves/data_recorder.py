#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, String
import csv
import os
from pathlib import Path
import time

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        # Estado de logging
        self.is_logging_enabled = False
        self.current_log_path = None
        self.csv_writer = None
        self.csv_file = None
        self.node_name = "raw_sensors"
        
        # Buffer para datos
        self.data_buffer = []
        self.buffer_size = 50  # Aumentado para mejor performance
        self.last_flush_time = time.time()
        
        # Subscripciones a los datos de sensores
        self.create_subscription(
            Odometry, '/odom/unfiltered', self.odom_cb, 10)
        
        self.create_subscription(
            Imu, '/imu/unfiltered', self.imu_cb, 10)

        self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_cb, 10)
            
        # Subscripciones a las señales del manager
        self.create_subscription(
            Bool, '/logging_enabled', self.logging_enabled_cb, 10)
            
        self.create_subscription(
            String, '/current_log_path', self.log_path_cb, 10)
        
        # Encabezado del CSV
        self.header = [
            'stamp', 'topic', 
            'position_x', 'position_y', 'position_z',
            'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
            'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
            'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
            'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
            'latitude', 'longitude', 'altitude',
            'cov_latitude', 'cov_longitude', 'cov_altitude'
        ]
    

    def logging_enabled_cb(self, msg):
        """Callback para habilitar/deshabilitar logging"""
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            
            if self.is_logging_enabled:
                if self.current_log_path:
                    self._start_logging()
                else:
                    self.get_logger().warning('Ruta de logging no recibida aún')
            else:
                self._stop_logging()

    def log_path_cb(self, msg):
        """Callback para recibir la ruta de logging"""
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            
            if self.is_logging_enabled and not self.csv_file:
                self._start_logging()

    def _start_logging(self):
        """Inicia el logging creando el archivo CSV"""
        if not self.current_log_path:
            self.get_logger().error('No hay ruta de logging definida')
            return
            
        try:
            # Crear directorio si no existe
            log_dir = Path(self.current_log_path)
            log_dir.mkdir(parents=True, exist_ok=True)
            
            # Nombre del archivo
            filename = log_dir / f"{self.node_name}.csv"
            
            # Abrir archivo CSV para escritura
            self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Escribir encabezado
            self.csv_writer.writerow(self.header)
            self.csv_file.flush()
            
            # Limpiar buffer
            self.data_buffer.clear()
            self.write_count = 0
            
        except Exception as e:
            self.get_logger().error(f'Error al iniciar logging: {e}')
            self.csv_file = None
            self.csv_writer = None

    def _stop_logging(self):
        """Detiene el logging y cierra el archivo"""
        self._flush_buffer()
        
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception as e:
                self.get_logger().error(f'Error al cerrar archivo: {e}')
        
        self.csv_file = None
        self.csv_writer = None
        self.data_buffer.clear()

    def _flush_buffer(self):
        """Escribe los datos del buffer al archivo"""
        if self.csv_writer and self.data_buffer:
            try:
                self.csv_writer.writerows(self.data_buffer)
                self.csv_file.flush()
                self.data_buffer.clear()
                self.last_flush_time = time.time()
            except Exception as e:
                self.get_logger().error(f'Error al escribir en CSV: {e}')

    def _write_data(self, row_data):
        """Escribe una fila de datos"""
        if not self.is_logging_enabled or not self.csv_writer:
            return
            
        # Verificar que la fila tenga la longitud correcta
        if len(row_data) != len(self.header):
            self.get_logger().error(f"Error: Fila tiene {len(row_data)} columnas, se esperaban {len(self.header)}")
            return
            
        self.data_buffer.append(row_data)
        
        # Escribir a disco si el buffer está lleno
        if len(self.data_buffer) >= self.buffer_size:
            self._flush_buffer()

    def _create_empty_row(self, stamp, topic):
        """Crea una fila vacía con el timestamp y topic"""
        return [stamp, topic] + [None] * (len(self.header) - 2)

    def odom_cb(self, msg):
        """Callback para datos de odometría"""
        if not self.is_logging_enabled:
            return
        
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        row = [
            stamp, 'odom',
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
            None, None, None,  # aceleración
            None, None, None,  # gps
            None, None, None   # covarianzas
        ]
        
        self._write_data(row)

    def imu_cb(self, msg):
        """Callback para datos IMU"""
        if not self.is_logging_enabled:
            return
        
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        row = [
            stamp, 'imu',
            None, None, None,  # posición
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            None, None, None,  # velocidad lineal
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            None, None, None,  # gps
            None, None, None   # covarianzas
        ]
        
        self._write_data(row)

    def gps_cb(self, msg):
        """Callback para datos GPS"""
        if not self.is_logging_enabled:
            return
        
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Extraer covarianzas diagonales
        cov_lat = None
        cov_lon = None
        cov_alt = None
        if len(msg.position_covariance) >= 9:
            cov_lat = msg.position_covariance[0]
            cov_lon = msg.position_covariance[4]
            cov_alt = msg.position_covariance[8]
        
        row = [
            stamp, 'gps',
            None, None, None,  # posición cartesiana
            None, None, None, None,  # orientación
            None, None, None,  # velocidad lineal
            None, None, None,  # velocidad angular
            None, None, None,  # aceleración
            msg.latitude,
            msg.longitude,
            msg.altitude,
            cov_lat,
            cov_lon,
            cov_alt
        ]
        
        self._write_data(row)

    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        self._stop_logging()
        super().destroy_node()


def main():
    rclpy.init()
    node = DataRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Deteniendo DataRecorder por Ctrl+C")
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