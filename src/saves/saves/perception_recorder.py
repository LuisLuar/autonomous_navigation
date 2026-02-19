#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from pathlib import Path
import csv
import time
import cv2
import numpy as np
from datetime import datetime

class PerceptionRecorder(Node):
    def __init__(self):
        super().__init__('perception_recorder')
        
        # Parámetro de muestreo
        self.declare_parameter('save_every_n', 3)
        
        # Estado
        self.is_recording = False
        self.current_log_path = None
        self.frame_count = 0
        self.saved_frames = 0
        
        # Bridge
        self.bridge = CvBridge()
        
        # QoS Best Effort para la imagen
        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # ÚNICA SUSCRIPCIÓN: Procesa y guarda directamente cuando llega la imagen
        self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.image_callback, 
            best_effort_qos
        )
        
        # Señales del manager
        self.create_subscription(Bool, '/logging_enabled', self.recording_enabled_cb, 10)
        self.create_subscription(String, '/current_log_path', self.log_path_cb, 10)
        
        # Archivos
        self.csv_file = None
        self.csv_writer = None
        self.current_session_dir = None
        
        self.get_logger().info('PerceptionRecorder iniciado - Modo SIMPLE: guarda al llegar')

    def recording_enabled_cb(self, msg):
        if msg.data != self.is_recording:
            self.is_recording = msg.data
            if self.is_recording:
                self.get_logger().info('Grabación HABILITADA')
                if self.current_log_path:
                    self._start_recording()
            else:
                self.get_logger().info('Grabación DESHABILITADA')
                self._stop_recording()

    def log_path_cb(self, msg):
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            self.get_logger().info(f'Ruta recibida: {self.current_log_path}')
            if self.is_recording and not self.current_session_dir:
                self._start_recording()

    def image_callback(self, msg):
        """PROCESAMIENTO DIRECTO: Cuando llega imagen, la guarda (si está grabando)"""
        if not self.is_recording:
            return
        
        try:
            # Incrementar contador de frames recibidos
            self.frame_count += 1
            
            # Verificar muestreo
            save_every_n = self.get_parameter('save_every_n').value
            if self.frame_count % save_every_n != 0:
                return
            
            # Descomprimir imagen AHORA MISMO
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                self.get_logger().error('Imagen decodificada es None')
                return
            
            # Guardar imagen AHORA MISMO
            self.saved_frames += 1
            frame_id = f"frame_{self.saved_frames:06d}"
            timestamp = time.time()
            
            # Guardar JPEG
            img_filename = f"{frame_id}.jpg"
            img_path = self.current_session_dir / "images" / img_filename
            cv2.imwrite(str(img_path), image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            
            # Guardar en CSV
            if self.csv_writer:
                row = [timestamp, frame_id, str(img_filename)]
                self.csv_writer.writerow(row)
                self.csv_file.flush()
            
            # Log cada 100 frames
            if self.saved_frames % 100 == 0:
                self.get_logger().info(f'✅ Guardadas: {self.saved_frames} imágenes')
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def _start_recording(self):
        if not self.current_log_path:
            return
            
        try:
            base_dir = Path(self.current_log_path)
            self.current_session_dir = base_dir / "perception"
            self.current_session_dir.mkdir(parents=True, exist_ok=True)
            
            # Crear carpeta de imágenes
            images_dir = self.current_session_dir / "images"
            images_dir.mkdir(exist_ok=True)
            
            # CSV
            csv_path = self.current_session_dir / "perception_data.csv"
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'frame_id', 'image_filename'])
            self.csv_file.flush()
            
            # Resetear contadores
            self.frame_count = 0
            self.saved_frames = 0
            
            self.get_logger().info(f'📁 Grabando en: {self.current_session_dir}')
            
        except Exception as e:
            self.get_logger().error(f'Error iniciando: {e}')

    def _stop_recording(self):
        self.get_logger().info('Deteniendo grabación...')
        
        if self.saved_frames > 0:
            self.get_logger().info(f'✅ Total guardadas: {self.saved_frames} imágenes')
            
            # Resumen simple
            try:
                summary_path = self.current_session_dir / "summary.txt"
                with open(summary_path, 'w') as f:
                    f.write(f"Total imágenes: {self.saved_frames}\n")
                    f.write(f"Fecha: {datetime.now()}\n")
                self.get_logger().info(f'📊 Resumen: {summary_path}')
            except:
                pass
        
        # Cerrar CSV
        if self.csv_file:
            self.csv_file.close()
        
        # Resetear
        self.csv_file = None
        self.csv_writer = None
        self.current_session_dir = None
        self.frame_count = 0
        self.saved_frames = 0

    def destroy_node(self):
        self._stop_recording()
        super().destroy_node()

def main():
    rclpy.init()
    node = PerceptionRecorder()
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