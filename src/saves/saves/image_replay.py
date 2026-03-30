#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from pathlib import Path
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DataReplayerCircular(Node):
    def __init__(self):
        super().__init__('image_replayer_circular')
        
        # Parámetro de frecuencia: Muy importante para tu comparativa
        self.declare_parameter('image_rate', 30.0) 
        self.image_rate = self.get_parameter('image_rate').value
        
        self.bridge = CvBridge()
        self.image_paths = []
        self.current_image_index = 0
        self.current_log_path = None
        
        # QoS optimizado para la Orin (Best Effort evita bloqueos por red)
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Publicamos en RAW para que el nodo de TensorRT no pierda tiempo de CPU descomprimiendo
        self.image_pub = self.create_publisher(Image, '/image_raw', camera_qos)
        
        # Timer de publicación (se inicializa vacío)
        self.timer = None
        
        # Suscriptor para activar el replayer
        self.create_subscription(String, '/replay_log_path', self.path_callback, 10)
        
        self.get_logger().info(f'🚀 Replayer Circular inicializado a {self.image_rate} Hz')

    def path_callback(self, msg):
        path_str = msg.data.strip()
        if path_str == self.current_log_path:
            return
            
        self.current_log_path = path_str
        self._load_image_list()
        
        if self.image_paths:
            self._start_circular_playback()

    def _load_image_list(self):
        self.image_paths = []
        base_path = Path(self.current_log_path)
        
        search_dirs = [base_path / "perception/images", base_path / "perception_frontal/images"]
        
        for directory in search_dirs:
            if directory.exists():
                for ext in ['.jpg', '.jpeg', '.png']:
                    self.image_paths.extend(list(directory.glob(f'*{ext}')))
                if self.image_paths: break
        
        self.image_paths.sort()
        self.get_logger().info(f'📊 {len(self.image_paths)} imágenes listas para bucle circular.')

    def _start_circular_playback(self):
        # Si ya había un timer, lo cancelamos para reiniciar
        if self.timer is not None:
            self.timer.cancel()
        
        self.current_image_index = 0
        period = 1.0 / self.image_rate
        self.timer = self.create_timer(period, self.publish_cycle)
        self.get_logger().info(f'▶️ Iniciando bucle infinito a {self.image_rate} Hz')

    def publish_cycle(self):
        if not self.image_paths:
            return

        # 1. Leer imagen
        img_path = str(self.image_paths[self.current_image_index])
        cv_image = cv2.imread(img_path)
        
        if cv_image is not None:
            # 2. Redimensionar al estándar de tu red (640x360 o 512x288)
            # Para TwinLiteNet/YOLOPv2 en tu tesis, usa la que definiste en el nodo C++
            cv_image = cv2.resize(cv_image, (640, 360))

            # 3. Convertir y Publicar
            try:
                ros_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                ros_msg.header.stamp = self.get_clock().now().to_msg()
                ros_msg.header.frame_id = 'camera_link'
                self.image_pub.publish(ros_msg)
            except Exception as e:
                self.get_logger().error(f'Error en loop: {e}')

        # 4. Lógica CIRCULAR: Volver al inicio si llegamos al final
        self.current_image_index = (self.current_image_index + 1) % len(self.image_paths)

def main():
    rclpy.init()
    node = DataReplayerCircular()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()