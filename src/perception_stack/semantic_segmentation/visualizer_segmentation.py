#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from custom_interfaces.msg import SegmentationData
from cv_bridge import CvBridge

class SegmentationVisualizerNode(Node):
    def __init__(self):
        super().__init__('segmentation_visualizer')
        
        self.bridge = CvBridge()
        
        # Buffers para datos
        self.current_image = None
        self.current_seg_mask = None
        
        # Configuración de visualización para YOLOPv2
        # Colores según la codificación:
        # 0: Fondo, 1: Área transitable, 2: Carriles
        self.seg_colors = {
            0: [0, 0, 0],        # Negro - Fondo
            1: [0, 200, 0],      # Verde - Área transitable
            2: [0, 255, 255]     # Amarillo - Carriles
        }
        
        # Transparencia para overlay
        self.alpha = 0.4
        
        # Suscripciones
        self.image_sub = self.create_subscription(Image, '/camera/rgb/image_raw', self.image_callback, 10)
        self.segmentation_sub = self.create_subscription(SegmentationData, '/segmentation/data', self.segmentation_callback, 10)
        
        # Publicador de imagen visualizada
        self.viz_pub = self.create_publisher(Image, '/segmentation/overlay', 10)
        
        #self.get_logger().info("Nodo de visualización de segmentación inicializado")
    
    def image_callback(self, msg):
        """Recibe imagen de cámara"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.visualize_and_publish()
        except Exception as e:
            #self.get_logger().error(f"Error procesando imagen: {e}")
            pass
    
    def segmentation_callback(self, msg):
        """Recibe datos de segmentación"""
        try:
            # Reconstruir máscara desde bytes
            mask_flat = np.frombuffer(msg.mask_data, dtype=np.uint8)
            self.current_seg_mask = mask_flat.reshape((msg.height, msg.width))
            
            # Visualizar si ya tenemos imagen
            if self.current_image is not None:
                self.visualize_and_publish()
        except Exception as e:
            #self.get_logger().error(f"Error procesando segmentación: {e}")
            pass
    
    def create_colored_mask(self, mask):
        """Crear máscara coloreada a partir de la máscara de segmentación"""
        # Crear imagen en blanco para la máscara coloreada
        colored = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
        
        # Aplicar colores según clase
        for class_id, color in self.seg_colors.items():
            colored[mask == class_id] = color
        
        return colored
    
    def draw_lane_contours(self, image, mask):
        """Dibujar contornos para los carriles"""
        # Extraer máscara solo de carriles (clase 2)
        lane_mask = (mask == 2).astype(np.uint8) * 255
        
        # Encontrar contornos
        contours, _ = cv2.findContours(
            lane_mask, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Dibujar contornos significativos
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filtrar contornos pequeños
                cv2.drawContours(
                    image, 
                    [contour], 
                    -1, 
                    (0, 255, 255),  # Amarillo para contornos
                    2
                )
        
        return image
    
    def visualize_and_publish(self):
        """Crear visualización y publicarla"""
        if self.current_image is None or self.current_seg_mask is None:
            return
        
        try:
            # Crear copia de la imagen original
            display_image = self.current_image.copy()
            
            # Crear máscara coloreada
            colored_mask = self.create_colored_mask(self.current_seg_mask)
            
            # Aplicar overlay para área transitable (clase 1)
            drivable_mask = (self.current_seg_mask == 1)
            if np.any(drivable_mask):
                # Overlay semi-transparente para área transitable
                overlay = display_image.copy()
                overlay[drivable_mask] = colored_mask[drivable_mask]
                display_image = cv2.addWeighted(
                    overlay, self.alpha,
                    display_image, 1 - self.alpha, 0
                )
            
            # Dibujar contornos para carriles (clase 2)
            display_image = self.draw_lane_contours(display_image, self.current_seg_mask)
            
            # Añadir texto informativo
            self.add_info_overlay(display_image)
            
            # Publicar imagen visualizada
            viz_msg = self.bridge.cv2_to_imgmsg(display_image, 'bgr8')
            viz_msg.header.stamp = self.get_clock().now().to_msg()
            self.viz_pub.publish(viz_msg)
            
        except Exception as e:
            #self.get_logger().error(f"Error en visualización: {e}")
            pass
    
    def add_info_overlay(self, image):
        """Añadir información de estado a la imagen"""
        height, width = image.shape[:2]
        
        # Fondo semitransparente para texto
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (350, 110), (0, 0, 0), -1)
        image = cv2.addWeighted(overlay, 0.6, image, 0.4, 0)
        
        # Texto informativo
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        
        y_pos = 40
        line_spacing = 30
        
        # Título
        cv2.putText(
            image, 
            "YOLOPv2 - Lane & Drivable Area", 
            (20, y_pos), 
            font, 
            font_scale, 
            (255, 255, 255), 
            thickness
        )
        
        # Estadísticas
        if self.current_seg_mask is not None:
            total_pixels = self.current_seg_mask.size
            
            # Porcentaje de área transitable
            drivable_pixels = np.sum(self.current_seg_mask == 1)
            drivable_percent = (drivable_pixels / total_pixels) * 100
            
            # Porcentaje de carriles
            lane_pixels = np.sum(self.current_seg_mask == 2)
            lane_percent = (lane_pixels / total_pixels) * 100
            
            # Información de área transitable
            cv2.putText(
                image, 
                f"Drivable Area: {drivable_percent:.1f}%", 
                (20, y_pos + line_spacing), 
                font, 
                font_scale * 0.8, 
                (0, 255, 0),  # Verde
                thickness
            )
            
            # Información de carriles
            cv2.putText(
                image, 
                f"Lane Area: {lane_percent:.1f}%", 
                (20, y_pos + 2 * line_spacing), 
                font, 
                font_scale * 0.8, 
                (0, 255, 255),  # Amarillo
                thickness
            )
        
        return image
    
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info("Nodo de visualización detenido por usuario")
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown

if __name__ == '__main__':
    main()