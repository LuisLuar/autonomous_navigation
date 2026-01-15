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
        self.node_name = "raw_sensors"  # Nombre para identificar este nodo en los archivos
        
        # Buffer para datos (opcional, para mejorar performance)
        self.data_buffer = []
        self.buffer_size = 10  # Número de mensajes antes de escribir a disco
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
            'stamp', 'topic', 'x', 'y', 'z',
            'qx', 'qy', 'qz', 'qw',
            'vx', 'vy', 'vz',  # linear velocities
            'wx', 'wy', 'wz',  # angular velocities
            'ax', 'ay', 'az',  # linear accelerations
            'lat', 'lon', 'alt',  # GPS coordinates
            'cov_lat', 'cov_lon', 'cov_alt'  # GPS covariances
        ]

        #self.get_logger().info('📥 DataRecorder inicializado - Esperando señal de logging...')

    def logging_enabled_cb(self, msg):
        """Callback para habilitar/deshabilitar logging"""
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            
            if self.is_logging_enabled:
                #self.get_logger().info('🚀 Logging HABILITADO - Comenzando grabación...')
                if self.current_log_path:
                    self._start_logging()
                else:
                    #self.get_logger().warning('Ruta de logging no recibida aún')
                    pass
            else:
                #self.get_logger().info('🛑 Logging DESHABILITADO - Deteniendo grabación...')
                self._stop_logging()

    def log_path_cb(self, msg):
        """Callback para recibir la ruta de logging"""
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            #self.get_logger().info(f'📁 Ruta de logging recibida: {self.current_log_path}')
            
            # Si el logging está habilitado pero aún no hemos abierto el archivo
            if self.is_logging_enabled and not self.csv_file:
                self._start_logging()

    def _start_logging(self):
        """Inicia el logging creando el archivo CSV"""
        if not self.current_log_path:
            #self.get_logger().error('No hay ruta de logging definida')
            return
            
        try:
            # Crear directorio si no existe
            log_dir = Path(self.current_log_path)
            log_dir.mkdir(parents=True, exist_ok=True)
            
            # Nombre del archivo específico para este nodo
            filename = log_dir / f"{self.node_name}.csv"
            
            # Abrir archivo CSV para escritura
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Escribir encabezado
            self.csv_writer.writerow(self.header)
            self.csv_file.flush()  # Forzar escritura inmediata
            
            # Limpiar buffer si tenía datos pendientes
            self.data_buffer.clear()
            
            #self.get_logger().info(f'💾 Archivo creado: {filename}')
            
        except Exception as e:
            #self.get_logger().error(f'Error al iniciar logging: {e}')
            self.csv_file = None
            self.csv_writer = None

    def _stop_logging(self):
        """Detiene el logging y cierra el archivo"""
        # Escribir cualquier dato pendiente en el buffer
        self._flush_buffer()
        
        if self.csv_file:
            try:
                self.csv_file.close()
                #self.get_logger().info('📂 Archivo CSV cerrado correctamente')
            except Exception as e:
                #self.get_logger().error(f'Error al cerrar archivo: {e}')
                pass
        
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
                #self.get_logger().error(f'Error al escribir en CSV: {e}')
                pass

    def _write_data(self, row_data):
        """Escribe una fila de datos (usa buffer para mejor performance)"""
        if not self.is_logging_enabled or not self.csv_writer:
            return
            
        self.data_buffer.append(row_data)
        
        # Escribir a disco si el buffer está lleno o ha pasado mucho tiempo
        if (len(self.data_buffer) >= self.buffer_size or 
            time.time() - self.last_flush_time > 1.0):  # Máximo 1 segundo en buffer
            self._flush_buffer()

    def odom_cb(self, msg):
        """Callback para datos de odometría"""
        if not self.is_logging_enabled:
            return
            
        row = [
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'odom',
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,  # Incluimos z si está disponible
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
            None, None, None,  # No hay aceleración en odometría
            None, None, None,  # No hay GPS
            None, None, None   # No hay covarianzas
        ]
        self._write_data(row)

    def imu_cb(self, msg):
        """Callback para datos IMU"""
        if not self.is_logging_enabled:
            return
            
        # Extraer aceleración angular y lineal del IMU
        row = [
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'imu',
            None, None, None,  # No hay posición en IMU
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            None, None, None,  # No hay velocidad lineal en IMU puro
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            None, None, None,  # No hay GPS
            None, None, None   # No hay covarianzas
        ]
        self._write_data(row)

    def gps_cb(self, msg):
        """Callback para datos GPS (con covarianzas)"""
        if not self.is_logging_enabled:
            return
            
        # Para GPS, guardamos las covarianzas diagonales (posición)
        # La matriz de covarianza es 3x3, guardamos los elementos diagonales
        row = [
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'gps',
            None, None, None,  # No hay posición cartesiana
            None, None, None, None,  # No hay orientación
            None, None, None,  # No hay velocidad
            None, None, None,  # No hay velocidad angular
            None, None, None,  # No hay aceleración
            msg.latitude,
            msg.longitude,
            msg.altitude,
            msg.position_covariance[0] if len(msg.position_covariance) > 0 else None,
            msg.position_covariance[4] if len(msg.position_covariance) > 4 else None,
            msg.position_covariance[8] if len(msg.position_covariance) > 8 else None
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
        pass
    finally:
        try:
            # Asegurarse de que todo se cierre correctamente
            node._stop_logging()
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass 

if __name__ == '__main__':
    main()