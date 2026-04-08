#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import csv
from pathlib import Path
import time
from datetime import datetime

class SystemHealthRecorder(Node):
    def __init__(self):
        super().__init__('system_health_recorder')
        
        self.is_logging_enabled = False
        self.current_log_path = None
        self.node_name = "jetson_health" # Cambiado para claridad
        
        self.csv_file = None
        self.csv_writer = None
        self.data_buffer = []
        self.buffer_size = 5 # Flush más frecuente para evitar pérdida en Jetson
        
        # Diccionario actualizado para Jetson Orin Nano
        self.current_values = {
            'timestamp': None,
            'cpu_temp': 0.0,
            'gpu_temp': 0.0,
            'soc_temp': 0.0,
            'vin_voltage': 0.0,
            'power_watt': 0.0,
            'cpu_usage': 0.0,
            'gpu_usage': 0.0,
            'ram_usage_pct': 0.0,
            'fan_speed_pct': 0.0,
            # Niveles de diagnóstico
            'cpu_level': 0,
            'gpu_level': 0,
            'soc_level': 0,
            'vin_level': 0,
            'ram_level': 0
        }
        
        # 1. SUSCRIPCIONES CORREGIDAS (Coinciden con JetsonHealthMonitor)
        topics = [
            ('/status/cpu_temperature', self.cpu_temp_cb),
            ('/status/gpu_temperature', self.gpu_temp_cb),
            ('/status/soc_temperature', self.soc_temp_cb),
            ('/status/input_voltage', self.vin_cb),
            ('/status/power_usage', self.power_cb),
            ('/status/cpu_usage', self.cpu_usage_cb),
            ('/status/ram_usage', self.ram_cb),
            ('/status/fan_speed', self.fan_cb)
        ]
        
        for topic, callback in topics:
            self.create_subscription(DiagnosticStatus, topic, callback, 10)
        
        # Señales del Manager
        self.create_subscription(Bool, '/logging_enabled', self.logging_enabled_cb, 10)
        self.create_subscription(String, '/current_log_path', self.log_path_cb, 10)
        
        self.save_timer = self.create_timer(1.0, self.save_health_data)
        #self.get_logger().info(' Recorder de salud para Jetson iniciado.')

    # --- Callbacks de Datos (Mapeados a las llaves de jtop) ---
    def _extract(self, msg, key, default='0.0'):
        for kv in msg.values:
            if kv.key == key: return kv.value
        return default

    def cpu_temp_cb(self, msg):
        self.current_values['cpu_temp'] = float(self._extract(msg, 'temp'))
        self.current_values['cpu_level'] = int.from_bytes(msg.level, 'big')
        self.current_values['timestamp'] = time.time()

    def gpu_temp_cb(self, msg):
        self.current_values['gpu_temp'] = float(self._extract(msg, 'temp'))
        self.current_values['gpu_level'] = int.from_bytes(msg.level, 'big')

    def soc_temp_cb(self, msg):
        self.current_values['soc_temp'] = float(self._extract(msg, 'temp'))
        self.current_values['soc_level'] = int.from_bytes(msg.level, 'big')

    def vin_cb(self, msg):
        self.current_values['vin_voltage'] = float(self._extract(msg, 'voltage'))
        self.current_values['vin_level'] = int.from_bytes(msg.level, 'big')

    def power_cb(self, msg):
        # Convertimos mW a W para el log
        p_mw = float(self._extract(msg, 'p_mw'))
        self.current_values['power_watt'] = p_mw / 1000.0

    def cpu_usage_cb(self, msg):
        self.current_values['cpu_usage'] = float(self._extract(msg, 'usage'))

    def ram_cb(self, msg):
        self.current_values['ram_usage_pct'] = float(self._extract(msg, 'usage'))
        self.current_values['ram_level'] = int.from_bytes(msg.level, 'big')

    def fan_cb(self, msg):
        self.current_values['fan_speed_pct'] = float(self._extract(msg, 'speed'))

    # --- Gestión de Archivos ---
    def logging_enabled_cb(self, msg):
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            if self.is_logging_enabled and self.current_log_path:
                self._start_logging()
            else:
                self._stop_logging()

    def log_path_cb(self, msg):
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            if self.is_logging_enabled:
                self._start_logging()

    def _start_logging(self):
        try:
            log_dir = Path(self.current_log_path)
            log_dir.mkdir(parents=True, exist_ok=True)
            filename = log_dir / f"{self.node_name}.csv"
            
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Header optimizado para Jetson
            header = [
                'timestamp', 'cpu_temp', 'gpu_temp', 'soc_temp', 
                'vin_v', 'power_w', 'cpu_pct', 'ram_pct', 'fan_pct',
                'cpu_lvl', 'gpu_lvl', 'soc_lvl', 'vin_lvl', 'ram_lvl'
            ]
            self.csv_writer.writerow(header)
            self.csv_file.flush()
            #self.get_logger().info(f' Grabando salud en: {filename}')
        except Exception as e:
            self.get_logger().error(f'Error CSV: {e}')

    def _stop_logging(self):
        if self.csv_file:
            self._flush_buffer()
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            #self.get_logger().info(' Log de salud cerrado.')

    def _flush_buffer(self):
        if self.csv_writer and self.data_buffer:
            self.csv_writer.writerows(self.data_buffer)
            self.csv_file.flush()
            self.data_buffer.clear()

    def save_health_data(self):
        if not self.is_logging_enabled or not self.csv_writer:
            return
        
        # Solo guardamos si hemos recibido al menos un timestamp
        if self.current_values['timestamp'] is None:
            return

        row = [
            self.current_values['timestamp'],
            round(self.current_values['cpu_temp'], 2),
            round(self.current_values['gpu_temp'], 2),
            round(self.current_values['soc_temp'], 2),
            round(self.current_values['vin_voltage'], 3),
            round(self.current_values['power_watt'], 2),
            round(self.current_values['cpu_usage'], 1),
            round(self.current_values['ram_usage_pct'], 1),
            round(self.current_values['fan_speed_pct'], 1),
            self.current_values['cpu_level'],
            self.current_values['gpu_level'],
            self.current_values['soc_level'],
            self.current_values['vin_level'],
            self.current_values['ram_level']
        ]
        
        self.data_buffer.append(row)
        if len(self.data_buffer) >= self.buffer_size:
            self._flush_buffer()

    def destroy_node(self):
        self._stop_logging()
        super().destroy_node()

def main():
    rclpy.init()
    node = SystemHealthRecorder()
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