#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from pathlib import Path
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DataReplayerManual(Node):
    def __init__(self):
        super().__init__('data_replayer_manual')
        
        # Estado
        self.current_log_path = None
        self.image_data = []
        self.current_image_index = 0
        
        # Publicador
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.image_pub = self.create_publisher(CompressedImage, '/image_raw/compressed', camera_qos)
        
        # Suscriptor para la ruta
        self.create_subscription(String, '/replay_log_path', self.replay_path_cb, 10)
        
        # Parámetros de compresión JPEG
        self.compression_params = [cv2.IMWRITE_JPEG_QUALITY, 85]
        
        # Timer corto solo para refrescar la ventana de OpenCV y capturar teclado
        # (Sin esto, la ventana de CV2 se congela en ROS2)
        self.ui_timer = self.create_timer(0.01, self.update_ui)
        
        self.get_logger().info('🚀 Replayer Manual Inicializado')
        self.get_logger().info('Controles: [Flecha Derecha] Siguiente, [Flecha Izquierda] Anterior, [Q] Salir')

    def replay_path_cb(self, msg):
        """Callback cuando llega una nueva ruta"""
        if msg.data == self.current_log_path:
            return
            
        self.current_log_path = msg.data
        self.get_logger().info(f'📁 Nueva ruta recibida: {self.current_log_path}')
        
        # Cargar y reiniciar
        self._load_images()
        self.current_image_index = 0
        
        if self.image_data:
            self._display_and_publish()

    def _load_images(self):
        self.image_data = []
        base_path = Path(self.current_log_path)
        
        # Rutas según tu estructura
        possible_paths = [
            base_path / "perception" / "images",
            base_path / "perception_frontal" / "images"
        ]
        
        for image_dir in possible_paths:
            if image_dir.exists():
                extensions = ['.jpg', '.jpeg', '.png']
                for ext in extensions:
                    for img_path in sorted(image_dir.glob(f'*{ext}')):
                        self.image_data.append(str(img_path))
                if self.image_data:
                    break
        
        self.image_data.sort()
        self.get_logger().info(f'📊 CARGADAS: {len(self.image_data)} imágenes')

    def _display_and_publish(self):
        """Lee la imagen actual, la muestra y la publica"""
        if not self.image_data:
            return

        img_path = self.image_data[self.current_image_index]
        cv_image = cv2.imread(img_path)
        
        if cv_image is None:
            self.get_logger().warning(f'⚠️ Error al leer: {img_path}')
            return

        # Redimensionar para consistencia
        cv_image = cv2.resize(cv_image, (640, 360), interpolation=cv2.INTER_AREA)

        # 1. Mostrar en ventana
        # Añadir texto informativo en la imagen
        display_img = cv_image.copy()
        info_text = f"Img: {self.current_image_index + 1}/{len(self.image_data)}"
        cv2.putText(display_img, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("Data Replayer - Manual Mode", display_img)

        # 2. Publicar a ROS2
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.format = 'jpeg'
        
        success, encoded = cv2.imencode('.jpg', cv_image, self.compression_params)
        if success:
            msg.data = encoded.tobytes()
            self.image_pub.publish(msg)

    def update_ui(self):
        """Maneja los eventos del teclado"""
        key = cv2.waitKey(1) & 0xFF
        
        if not self.image_data:
            return

        # Tecla Flecha Derecha (o 'd')
        if key == 83 or key == ord('d'): # 83 es el código de flecha derecha en muchos sistemas
            self.current_image_index = (self.current_image_index + 1) % len(self.image_data)
            self._display_and_publish()
            
        # Tecla Flecha Izquierda (o 'a')
        elif key == 81 or key == ord('a'): # 81 es flecha izquierda
            self.current_image_index = (self.current_image_index - 1) % len(self.image_data)
            self._display_and_publish()
            
        # Salir con 'q'
        elif key == ord('q'):
            self.get_logger().info('Cerrando...')
            rclpy.shutdown()

def main():
    rclpy.init()
    node = DataReplayerManual()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()