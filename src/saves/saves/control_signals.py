#!/usr/bin/env python3
"""
Data Logger Node for ROS2 Humble
Guarda datos de LaneModel, Odometry y Twist en archivos CSV
Sincronizado con logging_manager
"""

import rclpy
from rclpy.node import Node
import csv
import os
from pathlib import Path
import time
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from custom_interfaces.msg import LaneModel
from std_msgs.msg import Bool, String


class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger')
        
        # ---------------- Estado de logging ----------------
        self.is_logging_enabled = False
        self.current_log_path = None
        self.csv_writer = None
        self.csv_file = None
        
        # ---------------- Buffer para datos ----------------
        self.data_buffer = []
        self.buffer_size = 10
        self.last_flush_time = time.time()
        
        # ---------------- Contadores para estadísticas ----------------
        self.write_count = 0
        
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
        
        # ---------------- Subscripciones a señales del manager ----------------
        self.create_subscription(
            Bool, '/logging_enabled', self.logging_enabled_cb, 10)
        
        self.create_subscription(
            String, '/current_log_path', self.log_path_cb, 10)
        
        # Timer para guardar datos periódicamente (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)


    # =====================================================
    # Callbacks de señales del manager
    # =====================================================
    def logging_enabled_cb(self, msg):
        """Callback para habilitar/deshabilitar logging"""
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            
            if self.is_logging_enabled:
                if self.current_log_path:
                    self._start_logging()
                else:
                    self.get_logger().warning("Ruta de logging no recibida aún")
            else:
                self._stop_logging()


    def log_path_cb(self, msg):
        """Callback para recibir la ruta de logging"""
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            
            # Resetear contadores para nueva sesión
            self.write_count = 0
            
            # Si el logging está habilitado pero aún no hemos abierto el archivo
            if self.is_logging_enabled and not self.csv_file:
                self._start_logging()

    # =====================================================
    # Callbacks de datos
    # =====================================================
    def cb_lane_model(self, msg):
        """Callback para datos de LaneModel"""
        if not self.is_logging_enabled:
            return
            
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

    def cb_odometry(self, msg):
        """Callback para datos de Odometry"""
        if not self.is_logging_enabled:
            return
            
        # Convertir timestamp a segundos
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        self.odom_data = {
            'timestamp': timestamp,
            'linear_x': msg.twist.twist.linear.x,
            'angular_z': msg.twist.twist.angular.z
        }
        self.new_odom_data = True

    def cb_cmd_vel(self, msg):
        """Callback para datos de Twist (cmd_vel)"""
        if not self.is_logging_enabled:
            return
            
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        
        self.cmd_vel_data = {
            'timestamp': timestamp,
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        }
        self.new_cmd_vel_data = True

    # =====================================================
    # Funciones de logging
    # =====================================================
    def _start_logging(self):
        """Inicia el logging creando el archivo CSV"""
        if not self.current_log_path:
            self.get_logger().error("No hay ruta de logging definida para DATA")
            return
            
        try:
            # Crear directorio si no existe
            log_dir = Path(self.current_log_path)
            log_dir.mkdir(parents=True, exist_ok=True)
            
            # Nombre del archivo específico para datos
            filename = log_dir / f"robot_data.csv"
            
            # Abrir archivo CSV para escritura
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Escribir encabezados
            headers = [
                'timestamp',
                'lane_d_lat',
                'lane_yaw',
                'lane_curvature',
                'lane_confidence',
                'odom_linear_x',
                'odom_angular_z',
                'cmd_vel_linear_x',
                'cmd_vel_angular_z'
            ]
            self.csv_writer.writerow(headers)
            
            # Forzar escritura inmediata
            self.csv_file.flush()
            
            # Limpiar buffer
            self.data_buffer.clear()
            
            # Resetear contadores
            self.write_count = 0
            
        except Exception as e:
            self.get_logger().error(f"Error al iniciar logging DATA: {e}")
            self.csv_file = None
            self.csv_writer = None

    def _stop_logging(self):
        """Detiene el logging y cierra el archivo"""
        # Escribir cualquier dato pendiente en el buffer
        self._flush_buffer()
        
        # Cerrar archivo
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception as e:
                self.get_logger().error(f"Error al cerrar archivo DATA: {e}")
        
        # Resetear variables
        self.csv_file = None
        self.csv_writer = None
        self.data_buffer.clear()
        
        # Resetear banderas
        self.new_lane_data = False
        self.new_odom_data = False
        self.new_cmd_vel_data = False

    def _flush_buffer(self):
        """Escribe los datos del buffer al archivo"""
        if self.csv_writer and self.data_buffer:
            try:
                self.csv_writer.writerows(self.data_buffer)
                self.csv_file.flush()
                self.data_buffer.clear()
            except Exception as e:
                self.get_logger().error(f"Error al escribir en CSV DATA: {e}")
        
        self.last_flush_time = time.time()

    def _write_data(self, row_data):
        """Escribe una fila de datos al archivo (usa buffer)"""
        if not self.is_logging_enabled or not self.csv_writer:
            return
            
        self.data_buffer.append(row_data)
        self.write_count += 1
        
        # Escribir a disco si el buffer está lleno o ha pasado mucho tiempo
        if (len(self.data_buffer) >= self.buffer_size or 
            time.time() - self.last_flush_time > 1.0):
            self._flush_buffer()

    # =====================================================
    # Timer callback
    # =====================================================
    def timer_callback(self):
        """Timer para guardar datos periódicamente"""
        
        if not self.is_logging_enabled:
            return
        
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
        
        # Crear fila de datos
        row = [
            f"{current_time:.6f}",
            f"{self.lane_data['d_lat']:.6f}",
            f"{self.lane_data['yaw']:.6f}",
            f"{self.lane_data['curvature']:.6f}",
            f"{self.lane_data['confidence']:.6f}",
            f"{self.odom_data['linear_x']:.6f}",
            f"{self.odom_data['angular_z']:.6f}",
            f"{self.cmd_vel_data['linear_x']:.6f}",
            f"{self.cmd_vel_data['angular_z']:.6f}"
        ]
        
        # Guardar en CSV con buffer
        self._write_data(row)
        
        
        # Reiniciar banderas
        self.new_lane_data = False
        self.new_odom_data = False
        self.new_cmd_vel_data = False

    # =====================================================
    # Cleanup
    # =====================================================
    def destroy_node(self):
        """Limpieza al destruir el nodo"""
        self._stop_logging()
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