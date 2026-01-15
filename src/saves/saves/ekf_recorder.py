#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import csv
import os
from pathlib import Path
import time
import numpy as np

class EKFRecorder(Node):
    def __init__(self):
        super().__init__('ekf_recorder')

        # Estado de logging
        self.is_logging_enabled = False
        self.current_log_path = None
        self.csv_writer_local = None
        self.csv_writer_global = None
        self.csv_file_local = None
        self.csv_file_global = None
        self.node_name = "ekf_filters"
        
        # Buffer para datos
        self.local_buffer = []
        self.global_buffer = []
        self.buffer_size = 10
        self.last_flush_time = time.time()
        
        # Contadores para estadísticas
        self.local_msg_count = 0
        self.global_msg_count = 0
        
        # Subscripciones a los datos EKF
        self.create_subscription(
            Odometry, '/odometry/local', self.local_ekf_cb, 10)
        
        self.create_subscription(
            Odometry, '/odometry/global', self.global_ekf_cb, 10)
            
        # Subscripciones a las señales del manager
        self.create_subscription(
            Bool, '/logging_enabled', self.logging_enabled_cb, 10)
            
        self.create_subscription(
            String, '/current_log_path', self.log_path_cb, 10)
        
        # Encabezados de los CSVs
        self.local_header = [
            'stamp', 'frame_id', 'child_frame_id',
            'position_x', 'position_y', 'position_z',
            'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
            'velocity_x', 'velocity_y', 'velocity_z',
            'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
            # Covarianzas de posición (6x6 matriz - 36 elementos)
            'pose_covariance_00', 'pose_covariance_01', 'pose_covariance_02', 
            'pose_covariance_03', 'pose_covariance_04', 'pose_covariance_05',
            'pose_covariance_06', 'pose_covariance_07', 'pose_covariance_08',
            'pose_covariance_09', 'pose_covariance_10', 'pose_covariance_11',
            'pose_covariance_12', 'pose_covariance_13', 'pose_covariance_14',
            'pose_covariance_15', 'pose_covariance_16', 'pose_covariance_17',
            'pose_covariance_18', 'pose_covariance_19', 'pose_covariance_20',
            'pose_covariance_21', 'pose_covariance_22', 'pose_covariance_23',
            'pose_covariance_24', 'pose_covariance_25', 'pose_covariance_26',
            'pose_covariance_27', 'pose_covariance_28', 'pose_covariance_29',
            'pose_covariance_30', 'pose_covariance_31', 'pose_covariance_32',
            'pose_covariance_33', 'pose_covariance_34', 'pose_covariance_35',
            # Covarianzas de velocidad (6x6 matriz - 36 elementos)
            'twist_covariance_00', 'twist_covariance_01', 'twist_covariance_02',
            'twist_covariance_03', 'twist_covariance_04', 'twist_covariance_05',
            'twist_covariance_06', 'twist_covariance_07', 'twist_covariance_08',
            'twist_covariance_09', 'twist_covariance_10', 'twist_covariance_11',
            'twist_covariance_12', 'twist_covariance_13', 'twist_covariance_14',
            'twist_covariance_15', 'twist_covariance_16', 'twist_covariance_17',
            'twist_covariance_18', 'twist_covariance_19', 'twist_covariance_20',
            'twist_covariance_21', 'twist_covariance_22', 'twist_covariance_23',
            'twist_covariance_24', 'twist_covariance_25', 'twist_covariance_26',
            'twist_covariance_27', 'twist_covariance_28', 'twist_covariance_29',
            'twist_covariance_30', 'twist_covariance_31', 'twist_covariance_32',
            'twist_covariance_33', 'twist_covariance_34', 'twist_covariance_35'
        ]
        
        # Para datos globales, añadimos campos de incertidumbre (si es necesario)
        self.global_header = [
            'stamp', 'frame_id', 'child_frame_id',
            'position_x', 'position_y', 'position_z',
            'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
            'velocity_x', 'velocity_y', 'velocity_z',
            'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
            # Covarianzas de posición
            'pose_covariance_00', 'pose_covariance_01', 'pose_covariance_02',
            'pose_covariance_03', 'pose_covariance_04', 'pose_covariance_05',
            'pose_covariance_06', 'pose_covariance_07', 'pose_covariance_08',
            'pose_covariance_09', 'pose_covariance_10', 'pose_covariance_11',
            'pose_covariance_12', 'pose_covariance_13', 'pose_covariance_14',
            'pose_covariance_15', 'pose_covariance_16', 'pose_covariance_17',
            'pose_covariance_18', 'pose_covariance_19', 'pose_covariance_20',
            'pose_covariance_21', 'pose_covariance_22', 'pose_covariance_23',
            'pose_covariance_24', 'pose_covariance_25', 'pose_covariance_26',
            'pose_covariance_27', 'pose_covariance_28', 'pose_covariance_29',
            'pose_covariance_30', 'pose_covariance_31', 'pose_covariance_32',
            'pose_covariance_33', 'pose_covariance_34', 'pose_covariance_35',
            # Covarianzas de velocidad
            'twist_covariance_00', 'twist_covariance_01', 'twist_covariance_02',
            'twist_covariance_03', 'twist_covariance_04', 'twist_covariance_05',
            'twist_covariance_06', 'twist_covariance_07', 'twist_covariance_08',
            'twist_covariance_09', 'twist_covariance_10', 'twist_covariance_11',
            'twist_covariance_12', 'twist_covariance_13', 'twist_covariance_14',
            'twist_covariance_15', 'twist_covariance_16', 'twist_covariance_17',
            'twist_covariance_18', 'twist_covariance_19', 'twist_covariance_20',
            'twist_covariance_21', 'twist_covariance_22', 'twist_covariance_23',
            'twist_covariance_24', 'twist_covariance_25', 'twist_covariance_26',
            'twist_covariance_27', 'twist_covariance_28', 'twist_covariance_29',
            'twist_covariance_30', 'twist_covariance_31', 'twist_covariance_32',
            'twist_covariance_33', 'twist_covariance_34', 'twist_covariance_35',
            # Campos adicionales para análisis
            'position_uncertainty',  # Incertidumbre total en posición (norma de diagonal)
            'orientation_uncertainty',  # Incertidumbre en orientación
            'velocity_uncertainty'  # Incertidumbre en velocidad
        ]

        #self.get_logger().info('📥 EKFRecorder inicializado - Esperando señal de logging...')
        #self.get_logger().info('🗺️  Suscrito a: /odometry/local y /odometry/global')

    def logging_enabled_cb(self, msg):
        """Callback para habilitar/deshabilitar logging"""
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            
            if self.is_logging_enabled:
                #self.get_logger().info('🚀 Logging EKF HABILITADO - Comenzando grabación...')
                if self.current_log_path:
                    self._start_logging()
                else:
                    #self.get_logger().warning('Ruta de logging no recibida aún')
                    pass
            else:
                #self.get_logger().info('🛑 Logging EKF DESHABILITADO - Deteniendo grabación...')
                self._stop_logging()
                # Mostrar estadísticas
                self._log_statistics()

    def log_path_cb(self, msg):
        """Callback para recibir la ruta de logging"""
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            #self.get_logger().info(f'📁 Ruta de logging EKF recibida: {self.current_log_path}')
            
            # Resetear contadores para nueva sesión
            self.local_msg_count = 0
            self.global_msg_count = 0
            
            # Si el logging está habilitado pero aún no hemos abierto los archivos
            if self.is_logging_enabled and (not self.csv_file_local or not self.csv_file_global):
                self._start_logging()

    def _calculate_uncertainty(self, covariance_matrix):
        """Calcula métricas de incertidumbre a partir de la matriz de covarianza"""
        if len(covariance_matrix) != 36:
            return 0.0, 0.0, 0.0
            
        # Convertir a numpy array
        cov = np.array(covariance_matrix).reshape(6, 6)
        
        # Diagonal de la matriz
        diagonal = np.diag(cov)
        
        # Incertidumbre en posición (primeros 3 elementos)
        position_uncertainty = np.sqrt(np.sum(diagonal[0:3]))
        
        # Incertidumbre en orientación (elementos 3-5)
        orientation_uncertainty = np.sqrt(np.sum(diagonal[3:6]))
        
        # Si la matriz es 6x6, también podemos calcular para velocidad
        velocity_uncertainty = 0.0
        if len(covariance_matrix) == 36:
            # Para twist covariance
            pass
            
        return position_uncertainty, orientation_uncertainty, velocity_uncertainty

    def _start_logging(self):
        """Inicia el logging creando los archivos CSV"""
        if not self.current_log_path:
            #self.get_logger().error('No hay ruta de logging definida para EKF')
            return
            
        try:
            # Crear directorio si no existe
            log_dir = Path(self.current_log_path)
            log_dir.mkdir(parents=True, exist_ok=True)
            
            # Nombres de archivos específicos para EKF
            filename_local = log_dir / f"ekf_local.csv"
            filename_global = log_dir / f"ekf_global.csv"
            
            # Abrir archivos CSV para escritura
            self.csv_file_local = open(filename_local, 'w', newline='')
            self.csv_writer_local = csv.writer(self.csv_file_local)
            
            self.csv_file_global = open(filename_global, 'w', newline='')
            self.csv_writer_global = csv.writer(self.csv_file_global)
            
            # Escribir encabezados
            self.csv_writer_local.writerow(self.local_header)
            self.csv_writer_global.writerow(self.global_header)
            
            # Forzar escritura inmediata
            self.csv_file_local.flush()
            self.csv_file_global.flush()
            
            # Limpiar buffers
            self.local_buffer.clear()
            self.global_buffer.clear()
            
            #self.get_logger().info(f'💾 Archivos EKF creados:')
            #self.get_logger().info(f'   Local: {filename_local}')
            #self.get_logger().info(f'   Global: {filename_global}')
            
            # Resetear contadores
            self.local_msg_count = 0
            self.global_msg_count = 0
            
        except Exception as e:
            #self.get_logger().error(f'Error al iniciar logging EKF: {e}')
            self.csv_file_local = None
            self.csv_writer_local = None
            self.csv_file_global = None
            self.csv_writer_global = None

    def _stop_logging(self):
        """Detiene el logging y cierra los archivos"""
        # Escribir cualquier dato pendiente en los buffers
        self._flush_buffers()
        
        # Cerrar archivo local
        if self.csv_file_local:
            try:
                self.csv_file_local.close()
                #self.get_logger().info('📂 Archivo EKF local cerrado correctamente')
            except Exception as e:
                #self.get_logger().error(f'Error al cerrar archivo local: {e}')
                pass
        
        # Cerrar archivo global
        if self.csv_file_global:
            try:
                self.csv_file_global.close()
                #self.get_logger().info('📂 Archivo EKF global cerrado correctamente')
            except Exception as e:
                #self.get_logger().error(f'Error al cerrar archivo global: {e}')
                pass
        
        # Resetear variables
        self.csv_file_local = None
        self.csv_writer_local = None
        self.csv_file_global = None
        self.csv_writer_global = None
        self.local_buffer.clear()
        self.global_buffer.clear()

    def _flush_buffers(self):
        """Escribe los datos de los buffers a los archivos"""
        current_time = time.time()
        
        # Flush buffer local
        if self.csv_writer_local and self.local_buffer:
            try:
                self.csv_writer_local.writerows(self.local_buffer)
                self.csv_file_local.flush()
                self.local_buffer.clear()
            except Exception as e:
                #self.get_logger().error(f'Error al escribir en CSV local: {e}')
                pass
        
        # Flush buffer global
        if self.csv_writer_global and self.global_buffer:
            try:
                self.csv_writer_global.writerows(self.global_buffer)
                self.csv_file_global.flush()
                self.global_buffer.clear()
            except Exception as e:
                #self.get_logger().error(f'Error al escribir en CSV global: {e}')
                pass
        
        self.last_flush_time = current_time

    def _write_local_data(self, row_data):
        """Escribe una fila de datos al archivo local (usa buffer)"""
        if not self.is_logging_enabled or not self.csv_writer_local:
            return
            
        self.local_buffer.append(row_data)
        self.local_msg_count += 1
        
        # Escribir a disco si el buffer está lleno o ha pasado mucho tiempo
        if (len(self.local_buffer) >= self.buffer_size or 
            time.time() - self.last_flush_time > 1.0):
            self._flush_buffers()

    def _write_global_data(self, row_data):
        """Escribe una fila de datos al archivo global (usa buffer)"""
        if not self.is_logging_enabled or not self.csv_writer_global:
            return
            
        self.global_buffer.append(row_data)
        self.global_msg_count += 1
        
        # Escribir a disco si el buffer está lleno o ha pasado mucho tiempo
        if (len(self.global_buffer) >= self.buffer_size or 
            time.time() - self.last_flush_time > 1.0):
            self._flush_buffers()

    def local_ekf_cb(self, msg):
        """Callback para datos EKF local"""
        if not self.is_logging_enabled:
            return
            
        # Extraer timestamp
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Crear fila con todos los datos y covarianzas
        row = [timestamp, msg.header.frame_id, msg.child_frame_id]
        
        # Posición
        row.extend([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # Orientación (cuaternión)
        row.extend([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        
        # Velocidad lineal
        row.extend([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # Velocidad angular
        row.extend([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])
        
        # Covarianza de pose (36 elementos)
        row.extend(list(msg.pose.covariance))
        
        # Covarianza de twist (36 elementos)
        row.extend(list(msg.twist.covariance))
        
        self._write_local_data(row)

    def global_ekf_cb(self, msg):
        """Callback para datos EKF global"""
        if not self.is_logging_enabled:
            return
            
        # Extraer timestamp
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Crear fila con todos los datos y covarianzas
        row = [timestamp, msg.header.frame_id, msg.child_frame_id]
        
        # Posición
        row.extend([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # Orientación (cuaternión)
        row.extend([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        
        # Velocidad lineal
        row.extend([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # Velocidad angular
        row.extend([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])
        
        # Covarianza de pose (36 elementos)
        row.extend(list(msg.pose.covariance))
        
        # Covarianza de twist (36 elementos)
        row.extend(list(msg.twist.covariance))
        
        # Calcular métricas de incertidumbre para análisis
        pos_unc, orient_unc, vel_unc = self._calculate_uncertainty(msg.pose.covariance)
        row.extend([pos_unc, orient_unc, vel_unc])
        
        self._write_global_data(row)

    def _log_statistics(self):
        """Muestra estadísticas de la sesión de logging"""
        #self.get_logger().info('📊 Estadísticas EKF de la sesión:')
        #self.get_logger().info(f'   Mensajes EKF local: {self.local_msg_count}')
        #self.get_logger().info(f'   Mensajes EKF global: {self.global_msg_count}')
        #self.get_logger().info(f'   Total mensajes: {self.local_msg_count + self.global_msg_count}')

    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        self._stop_logging()
        super().destroy_node()

def main():
    rclpy.init()
    node = EKFRecorder()

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