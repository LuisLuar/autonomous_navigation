#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import csv
import os
from pathlib import Path
import time
from collections import deque

class ControlRecorder(Node):
    def __init__(self):
        super().__init__('control_recorder')

        # Estado de logging
        self.is_logging_enabled = False
        self.current_log_path = None
        self.csv_writer = None
        self.csv_file = None
        self.node_name = "control_signals"
        
        # Buffer para datos
        self.data_buffer = []
        self.buffer_size = 10
        self.last_flush_time = time.time()
        
        # Variables para almacenar últimos valores
        self.last_timestamp = None
        self.values = {
            # Lane detection - valores originales
            'omega_lane': None,
            'active_lane': None,
            
            # Lane detection - nuevos valores
            'error_lateral': None,
            'error_heading': None,
            'curvature': None,
            'omega_lane_orientation': None,
            'controller_mode': None,  # 1:FRAME, 2:TRANSITION, 3:LANE, 0:otros
            
            # Lidar lateral
            'omega_lidar': None,
            'active_lidar_lateral': None,
            
            # Lidar frontal
            'alpha_lidar': None,
            'active_lidar_front': None,
            
            # Vision
            'alpha_vision': None,
            'active_vision': None,
            
            # OSM obstacles
            'alpha_osm_obstacles': None,
            'active_osm_obstacles': None,
            
            # OSM
            'alpha_osm': None,
            'active_osm': None,
            
            # Command velocity
            'cmd_vel_linear_x': None,
            'cmd_vel_angular_z': None,
            
            # Timestamp
            'timestamp': None
        }
        
        # Historial para promedios móviles
        self.history = {key: deque(maxlen=10) for key in [
            'omega_lane', 'omega_lidar', 'alpha_lidar', 
            'alpha_vision', 'alpha_osm_obstacles', 'alpha_osm',
            'cmd_vel_linear_x', 'cmd_vel_angular_z',
            'error_lateral', 'error_heading', 'curvature', 'omega_lane_orientation'
        ]}
        
        # Subscripciones a los temas de control
        # Lane detection - originales
        self.create_subscription(Float32, '/omega/lane', self.lane_cb, 10)
        self.create_subscription(Bool, '/active/lane', self.lane_active_cb, 10)
        
        # Lane detection - nuevos valores
        self.create_subscription(Float32, '/lane/error_lateral', self.e_lat_cb, 10)
        self.create_subscription(Float32, '/lane/error_heading', self.e_head_cb, 10)
        self.create_subscription(Float32, '/lane/curvature', self.curvature_cb, 10)
        self.create_subscription(Float32, '/omega/lane_orientation', self.lane_orientation_cb, 10)
        
        # Modo del controlador (SUBSCRIPCIÓN, no publicación)
        self.create_subscription(Int32, '/lane/controller_mode', self.controller_mode_cb, 10)

        # Lidar lateral
        self.create_subscription(Float32, '/omega/lidar', self.lidar_omega_cb, 10)
        self.create_subscription(Bool, '/active/lidar_lateral', self.lidar_lateral_active_cb, 10)

        # Lidar frontal
        self.create_subscription(Float32, '/alpha/lidar', self.lidar_alpha_cb, 10)
        self.create_subscription(Bool, '/active/lidar_front', self.lidar_front_active_cb, 10)
        
        # Vision
        self.create_subscription(Float32, '/alpha/vision', self.vision_alpha_cb, 10)
        self.create_subscription(Bool, '/active/vision', self.vision_active_cb, 10)

        # OSM obstacles
        self.create_subscription(Float32, '/alpha/osm_obstacles', self.osm_obstacles_alpha_cb, 10)
        self.create_subscription(Bool, '/active/osm_obstacles', self.osm_obstacles_active_cb, 10)

        # OSM
        self.create_subscription(Float32, '/alpha/osm', self.osm_alpha_cb, 10)
        self.create_subscription(Bool, '/active/osm', self.osm_active_cb, 10)
        
        # Command velocity
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        
        # Subscripciones a las señales del manager
        self.create_subscription(Bool, '/logging_enabled', self.logging_enabled_cb, 10)
        self.create_subscription(String, '/current_log_path', self.log_path_cb, 10)
        
        # Timer para guardar datos periódicamente (cuando tenemos valores)
        self.save_timer = self.create_timer(0.05, self.save_data_timer_cb)  # 20 Hz
        
        # Encabezado del CSV actualizado
        self.header = [
            'timestamp',
            # Lane detection - originales
            'omega_lane', 'active_lane',
            # Lane detection - nuevos valores
            'error_lateral', 'error_heading', 'curvature',
            'omega_lane_orientation', 'controller_mode',
            # Lidar lateral
            'omega_lidar', 'active_lidar_lateral',
            # Lidar frontal
            'alpha_lidar', 'active_lidar_front',
            # Vision
            'alpha_vision', 'active_vision',
            # OSM obstacles
            'alpha_osm_obstacles', 'active_osm_obstacles',
            # OSM
            'alpha_osm', 'active_osm',
            # Command velocity
            'cmd_vel_linear_x', 'cmd_vel_angular_z',
            # Flags calculados
            'any_vision_active',  # Si lane, lidar_front o vision están activos
            'any_lidar_active',   # Si lidar_lateral o lidar_front están activos
            'any_osm_active',     # Si osm_obstacles o osm están activos
            # Totales activos
            'total_active_sensors'
        ]

    def controller_mode_cb(self, msg):
        """Callback para recibir el modo del controlador de lane"""
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['controller_mode'] = msg.data

    # Callbacks para los nuevos valores de lane
    def e_lat_cb(self, msg):
        """Callback para error lateral"""
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['error_lateral'] = msg.data
        self.history['error_lateral'].append(msg.data)

    def e_head_cb(self, msg):
        """Callback para error de heading"""
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['error_heading'] = msg.data
        self.history['error_heading'].append(msg.data)

    def curvature_cb(self, msg):
        """Callback para curvatura"""
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['curvature'] = msg.data
        self.history['curvature'].append(msg.data)

    def lane_orientation_cb(self, msg):
        """Callback para omega de orientación de lane"""
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['omega_lane_orientation'] = msg.data
        self.history['omega_lane_orientation'].append(msg.data)

    def logging_enabled_cb(self, msg):
        """Callback para habilitar/deshabilitar logging"""
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            
            if self.is_logging_enabled:
                if self.current_log_path:
                    self._start_logging()
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
            
            # Limpiar buffer
            self.data_buffer.clear()
            
            # Resetear valores
            self._reset_values()
            
        except Exception as e:
            self.csv_file = None
            self.csv_writer = None

    def _stop_logging(self):
        """Detiene el logging y cierra el archivo"""
        # Escribir cualquier dato pendiente en el buffer
        self._flush_buffer()
        
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass
        
        self.csv_file = None
        self.csv_writer = None
        self.data_buffer.clear()

    def _reset_values(self):
        """Resetea todos los valores a None para nueva sesión"""
        for key in self.values:
            self.values[key] = None
        for key in self.history:
            self.history[key].clear()

    def _flush_buffer(self):
        """Escribe los datos del buffer al archivo"""
        if self.csv_writer and self.data_buffer:
            try:
                self.csv_writer.writerows(self.data_buffer)
                self.csv_file.flush()
                self.data_buffer.clear()
                self.last_flush_time = time.time()
            except Exception:
                pass

    def _calculate_flags(self):
        """Calcula flags derivados de los estados activos"""
        # Flags de categorías
        any_vision_active = any([
            self.values['active_lane'],
            self.values['active_lidar_front'],
            self.values['active_vision']
        ])
        
        any_lidar_active = any([
            self.values['active_lidar_lateral'],
            self.values['active_lidar_front']
        ])
        
        any_osm_active = any([
            self.values['active_osm_obstacles'],
            self.values['active_osm']
        ])
        
        # Contar sensores activos
        active_sensors = [
            self.values['active_lane'],
            self.values['active_lidar_lateral'],
            self.values['active_lidar_front'],
            self.values['active_vision'],
            self.values['active_osm_obstacles'],
            self.values['active_osm']
        ]
        total_active = sum(1 for s in active_sensors if s is True)
        
        return any_vision_active, any_lidar_active, any_osm_active, total_active

    def _prepare_row_data(self):
        """Prepara una fila de datos para guardar"""
        # Calcular flags
        any_vision_active, any_lidar_active, any_osm_active, total_active = self._calculate_flags()
        
        # Crear fila con todos los valores
        row = [
            self.values['timestamp'],
            # Lane detection - originales
            self.values['omega_lane'],
            1 if self.values['active_lane'] else 0 if self.values['active_lane'] is not None else None,
            # Lane detection - nuevos valores
            self.values['error_lateral'],
            self.values['error_heading'],
            self.values['curvature'],
            self.values['omega_lane_orientation'],
            self.values['controller_mode'],
            # Lidar lateral
            self.values['omega_lidar'],
            1 if self.values['active_lidar_lateral'] else 0 if self.values['active_lidar_lateral'] is not None else None,
            # Lidar frontal
            self.values['alpha_lidar'],
            1 if self.values['active_lidar_front'] else 0 if self.values['active_lidar_front'] is not None else None,
            # Vision
            self.values['alpha_vision'],
            1 if self.values['active_vision'] else 0 if self.values['active_vision'] is not None else None,
            # OSM obstacles
            self.values['alpha_osm_obstacles'],
            1 if self.values['active_osm_obstacles'] else 0 if self.values['active_osm_obstacles'] is not None else None,
            # OSM
            self.values['alpha_osm'],
            1 if self.values['active_osm'] else 0 if self.values['active_osm'] is not None else None,
            # Command velocity
            self.values['cmd_vel_linear_x'],
            self.values['cmd_vel_angular_z'],
            # Flags calculados
            1 if any_vision_active else 0,
            1 if any_lidar_active else 0,
            1 if any_osm_active else 0,
            total_active
        ]
        
        return row

    def save_data_timer_cb(self):
        """Timer callback para guardar datos periódicamente"""
        if not self.is_logging_enabled or not self.csv_writer:
            return
        
        # Solo guardar si tenemos al menos un valor no-None (excepto timestamp)
        has_data = any(
            self.values[key] is not None 
            for key in self.values 
            if key != 'timestamp'
        )
        
        if has_data and self.values['timestamp'] is not None:
            # Actualizar timestamp si no se ha actualizado recientemente
            current_time = time.time()
            if current_time - self.values['timestamp'] > 0.05:  # 20 Hz máximo
                self.values['timestamp'] = current_time
            
            # Preparar y guardar datos
            row = self._prepare_row_data()
            self.data_buffer.append(row)
            
            # Escribir a disco si el buffer está lleno
            if len(self.data_buffer) >= self.buffer_size:
                self._flush_buffer()

    # Callbacks originales (sin cambios)
    def lane_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['omega_lane'] = msg.data
        self.history['omega_lane'].append(msg.data)

    def lane_active_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['active_lane'] = msg.data

    def lidar_omega_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['omega_lidar'] = msg.data
        self.history['omega_lidar'].append(msg.data)

    def lidar_lateral_active_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['active_lidar_lateral'] = msg.data

    def lidar_alpha_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['alpha_lidar'] = msg.data
        self.history['alpha_lidar'].append(msg.data)

    def lidar_front_active_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['active_lidar_front'] = msg.data

    def vision_alpha_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['alpha_vision'] = msg.data
        self.history['alpha_vision'].append(msg.data)

    def vision_active_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['active_vision'] = msg.data

    def osm_obstacles_alpha_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['alpha_osm_obstacles'] = msg.data
        self.history['alpha_osm_obstacles'].append(msg.data)

    def osm_obstacles_active_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['active_osm_obstacles'] = msg.data

    def osm_alpha_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['alpha_osm'] = msg.data
        self.history['alpha_osm'].append(msg.data)

    def osm_active_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['active_osm'] = msg.data

    def cmd_vel_cb(self, msg):
        current_time = time.time()
        self.values['timestamp'] = current_time
        self.values['cmd_vel_linear_x'] = msg.linear.x
        self.values['cmd_vel_angular_z'] = msg.angular.z
        self.history['cmd_vel_linear_x'].append(msg.linear.x)
        self.history['cmd_vel_angular_z'].append(msg.angular.z)

    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        self._stop_logging()
        super().destroy_node()

def main():
    rclpy.init()
    node = ControlRecorder()

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