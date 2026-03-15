#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from custom_interfaces.msg import PixelPoint, ObjectInfoArray
from cv_bridge import CvBridge
import time

class ImprovedVisualizerNode(Node):
    def __init__(self):
        super().__init__('improved_visualizer')
        self.bridge = CvBridge()
        
        # Almacenes de datos con sus marcas de tiempo
        self.data_store = {
            'image': {'msg': None, 'time': 0},
            'lane_pixels': {'msg': None, 'time': 0},
            'centerline_pixels': {'msg': None, 'time': 0}
        }

        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Suscripciones
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, best_effort_qos)
        self.pixels_lane_sub = self.create_subscription(PixelPoint, '/lane/pixel_candidates', self.lane_callback, 10)
        self.pixels_center_sub = self.create_subscription(PixelPoint, '/lane/ipm_inverse_pixel_points', self.center_callback, 10)
        
        # Publicador de salida
        self.viz_pub = self.create_publisher(Image, '/debug/camera_front', 10)
        
        # Timer para visualización constante (Evita saturar la CPU)
        self.create_timer(1.0/30.0, self.render_loop)
        
        self.last_time = time.time()
        self.get_logger().info("Visualizador optimizado iniciado - Esperando datos sincronizados...")

    def image_callback(self, msg):
        self.data_store['image']['msg'] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.data_store['image']['time'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def lane_callback(self, msg):
        self.data_store['lane_pixels']['msg'] = msg
        self.data_store['lane_pixels']['time'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def center_callback(self, msg):
        self.data_store['centerline_pixels']['msg'] = msg
        self.data_store['centerline_pixels']['time'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def draw_fast_pixels(self, canvas, pixel_msg, color, threshold=0.1):
        """Dibuja miles de puntos instantáneamente usando NumPy si el tiempo coincide"""
        if pixel_msg is None or canvas is None:
            return canvas

        # Validar sincronía: Si el punto es más viejo/nuevo que 100ms respecto a la imagen, ignorar
        msg_time = pixel_msg.header.stamp.sec + pixel_msg.header.stamp.nanosec * 1e-9
        if abs(self.data_store['image']['time'] - msg_time) > threshold:
            return canvas # Desfase temporal detectado, no dibujar para evitar "fantasmas"

        # Convertir a array de NumPy para velocidad máxima
        u = np.array(pixel_msg.u, dtype=np.int32)
        v = np.array(pixel_msg.v, dtype=np.int32)

        # Máscara de límites para evitar crash si el punto se sale de la imagen
        h, w = canvas.shape[:2]
        mask = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        
        # Dibujar todos los puntos de una sola vez (Asignación de matriz)
        canvas[v[mask], u[mask]] = color
        
        # Si quieres puntos más gruesos sin perder velocidad, dibuja los vecinos:
        # canvas[np.clip(v[mask]+1, 0, h-1), u[mask]] = color
        
        return canvas

    def render_loop(self):
        img = self.data_store['image']['msg']
        if img is None:
            return

        # Crear copia para no alterar el original
        display_img = img.copy()

        # Dibujar Carriles (Verde BGR: 0, 255, 0)
        display_img = self.draw_fast_pixels(display_img, self.data_store['lane_pixels']['msg'], (0, 255, 0))
        
        # Dibujar Línea Central (Magenta BGR: 255, 0, 255)
        display_img = self.draw_fast_pixels(display_img, self.data_store['centerline_pixels']['msg'], (255, 0, 255))

        # Añadir FPS real de visualización
        now = time.time()
        fps = 1.0 / (now - self.last_time)
        self.last_time = now
        cv2.putText(display_img, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        # Publicar resultado
        out_msg = self.bridge.cv2_to_imgmsg(display_img, encoding='bgr8')
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.viz_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImprovedVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()