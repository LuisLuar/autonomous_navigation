#!/usr/bin/env python3
"""
DataReplayer SIMPLIFICADO - Publica imágenes a frecuencia configurable
=====================================================================
- Publica imágenes secuencialmente a la frecuencia especificada
- Parámetro 'image_rate' controla los Hz de publicación
- Ejemplo: image_rate=30.0 publica 30 imágenes por segundo
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import csv
from pathlib import Path
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DataReplayer(Node):
    def __init__(self):
        super().__init__('image_replayer')
        
        # ÚNICO PARÁMETRO: frecuencia de publicación de imágenes
        self.declare_parameter('image_rate', 30.0)  # Default: 30 Hz
        self.image_rate = self.get_parameter('image_rate').value
        
        # Estado
        self.current_log_path = None
        self.image_data = []
        self.current_image_index = 0
        
        # Publicador para cámara frontal
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.image_pub = self.create_publisher(CompressedImage, '/image_raw/compressed', camera_qos)
        
        # Temporizador para imágenes (se configura después de cargar)
        self.image_timer = None
        
        # Suscriptor para la ruta
        self.create_subscription(String, '/replay_log_path', self.replay_path_cb, 10)
        
        # Parámetros de compresión JPEG
        self.compression_params = [cv2.IMWRITE_JPEG_QUALITY, 85]
        
        self.get_logger().info('🚀 DataReplayer SIMPLIFICADO inicializado')
        self.get_logger().info(f'📷 Frecuencia de imágenes: {self.image_rate} Hz')

    def replay_path_cb(self, msg):
        """Callback para recibir la ruta y comenzar"""
        if msg.data == self.current_log_path:
            return
            
        self.current_log_path = msg.data
        self.get_logger().info(f'📁 Ruta recibida: {self.current_log_path}')
        
        # Cargar imágenes
        self._load_images()
        
        # Iniciar reproducción
        self._start_playback()

    def _load_images(self):
        """Carga todas las imágenes del directorio"""
        self.image_data = []
        base_path = Path(self.current_log_path)
        
        # Buscar en carpetas posibles
        possible_paths = [
            base_path / "perception" / "images",
            base_path / "perception_frontal" / "images"
        ]
        
        for image_dir in possible_paths:
            if image_dir.exists():
                self._load_images_from_dir(image_dir)
                if self.image_data:
                    break
        
        # Ordenar por nombre (secuencial)
        self.image_data.sort(key=lambda x: x['path'])
        
        self.get_logger().info(f'📊 CARGADAS: {len(self.image_data)} imágenes')
        
        if len(self.image_data) == 0:
            self.get_logger().error('❌ No se encontraron imágenes')

    def _load_images_from_dir(self, image_dir):
        """Carga imágenes del directorio"""
        extensions = ['.jpg', '.jpeg', '.png']
        for ext in extensions:
            for img_path in sorted(image_dir.glob(f'*{ext}')):
                self.image_data.append({
                    'path': str(img_path)
                })

    def _start_playback(self):
        """Inicia la reproducción a la frecuencia configurada"""
        if not self.image_data:
            self.get_logger().error('❌ No hay imágenes para reproducir')
            return
        
        # Resetear índice
        self.current_image_index = 0
        
        # Crear temporizador con la frecuencia deseada
        timer_period = 1.0 / self.image_rate  # Ej: 1/30 = 0.0333 segundos
        self.image_timer = self.create_timer(timer_period, self.publish_next_image)
        
        self.get_logger().info(f'▶️ REPRODUCIENDO {len(self.image_data)} imágenes a {self.image_rate} Hz')
        self.get_logger().info(f'⏱️  Intervalo: {timer_period*1000:.1f} ms entre imágenes')

    def publish_next_image(self):
        """Publica la siguiente imagen en la secuencia"""
        if self.current_image_index >= len(self.image_data):
            # Terminamos
            self.get_logger().info('✅ Reproducción completada')
            self.image_timer.cancel()
            return
        
        # Publicar imagen actual
        img_data = self.image_data[self.current_image_index]
        self._publish_image(img_data)
        
        # Avanzar al siguiente índice
        self.current_image_index += 1
        
        # Log cada 30 imágenes
        if self.current_image_index % 30 == 0:
            self.get_logger().info(f'📸 Publicadas {self.current_image_index}/{len(self.image_data)} imágenes')

    def _publish_image(self, img_data):
        """Publica una imagen en formato comprimido"""
        try:
            cv_image = cv2.imread(img_data['path'])
            if cv_image is None:
                self.get_logger().warning(f'⚠️ No se pudo leer: {img_data["path"]}')
                return
            
            # Redimensionar si es necesario
            if cv_image.shape[1] != 640 or cv_image.shape[0] != 360:
                cv_image = cv2.resize(cv_image, (640, 360), interpolation=cv2.INTER_AREA)
            
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            msg.format = 'jpeg'
            
            success, encoded = cv2.imencode('.jpg', cv_image, self.compression_params)
            if success:
                msg.data = encoded.tobytes()
                self.image_pub.publish(msg)
                
        except Exception as e:
            pass

def main():
    rclpy.init()
    node = DataReplayer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 Deteniendo...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()