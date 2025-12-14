#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from custom_interfaces.msg import LaneArray, SegmentationData
from cv_bridge import CvBridge

class UnifiedVisualizerNode(Node):
    def __init__(self):
        super().__init__('unified_visualizer')
        
        self.bridge = CvBridge()
        
        # Buffers para datos
        self.current_image = None
        self.current_seg_mask = None
        self.current_lanes = None
        
        # Configuración de visualización
        self.show_segmentation = True  # Mostrar overlay de segmentación
        self.show_lanes = True         # Mostrar carriles detectados
        self.show_original = True     # Mostrar imagen original también
        self.alpha = 0.6               # Transparencia segmentación
        
        # Colores para segmentación (mismos que en segmentador)
        self.seg_colors = np.array([
            [128, 64,128], [244, 35,232], [ 70, 70, 70], [102,102,156],
            [190,153,153], [153,153,153], [250,170, 30], [220,220,  0],
            [107,142, 35], [152,251,152], [ 70,130,180], [220, 20, 60],
            [255,  0,  0], [  0,  0,142], [  0,  0, 70], [  0, 60,100],
            [  0, 80,100], [  0,  0,230], [119, 11, 32]
        ], dtype=np.uint8)
        
        # Colores para carriles
        self.lane_colors = [
            (0, 255, 0),    # Verde
            (255, 0, 0),    # Azul
            (0, 0, 255),    # Rojo
            (255, 255, 0),  # Cian
            (255, 0, 255),  # Magenta
            (0, 255, 255)   # Amarillo
        ]
        
        # Suscripciones
        self.create_subscription(Image, '/camera/rgb/image_raw', self.image_callback, 10)
        self.create_subscription(SegmentationData, '/segmentation/data', self.segmentation_callback, 10)
        self.create_subscription(LaneArray, '/lanes/detected', self.lanes_callback, 10)
        
        # Publicador opcional de imagen visualizada
        self.pub_visualization = self.create_publisher(Image, '/segmentation/overlay', 10)
        
        # Timer para mostrar (30 FPS)
        self.timer = self.create_timer(0.033, self.visualize_callback)
        

    
    def image_callback(self, msg):
        """Recibe imagen de cámara"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")
    
    def segmentation_callback(self, msg):
        """Recibe datos de segmentación"""
        try:
            # Reconstruir máscara desde bytes
            mask_flat = np.frombuffer(msg.mask_data, dtype=np.uint8)
            self.current_seg_mask = mask_flat.reshape((msg.height, msg.width))
        except Exception as e:
            self.get_logger().error(f"Error procesando segmentación: {e}")
    
    def lanes_callback(self, msg):
        """Recibe carriles detectados"""
        self.current_lanes = msg
    
    
    def apply_segmentation_overlay(self, image):
        """Aplicar overlay de segmentación a la imagen"""
        if self.current_seg_mask is None:
            return image
        
        # Crear máscara coloreada
        colored_mask = self.seg_colors[self.current_seg_mask]
        colored_mask_bgr = cv2.cvtColor(colored_mask, cv2.COLOR_RGB2BGR)
        
        # Aplicar overlay solo donde no es fondo (clase 0)
        non_bg = self.current_seg_mask != 0
        if np.any(non_bg):
            if self.show_original:
                # Mezcla con imagen original
                image[non_bg] = cv2.addWeighted(
                    colored_mask_bgr[non_bg], self.alpha,
                    image[non_bg], 1 - self.alpha, 0
                )
            else:
                # Solo máscara
                image[non_bg] = colored_mask_bgr[non_bg]
        
        return image
    
    def draw_lanes(self, image):
        """Dibujar carriles detectados"""
        if self.current_lanes is None or len(self.current_lanes.lanes) == 0:
            return image
        
        height, width = image.shape[:2]
        
        for i, lane in enumerate(self.current_lanes.lanes):
            if len(lane.points_x) != len(lane.points_y):
                continue
            
            color = self.lane_colors[i % len(self.lane_colors)]
            
            # Convertir puntos normalizados a píxeles
            points = []
            for x_norm, y_norm in zip(lane.points_x, lane.points_y):
                x = int(x_norm * width)
                y = int(y_norm * height)
                points.append((x, y))
            
            # Dibujar línea que conecta puntos
            if len(points) >= 2:
                for j in range(len(points) - 1):
                    cv2.line(image, points[j], points[j+1], color, 3)
            
            # Dibujar puntos
            for point in points:
                cv2.circle(image, point, 5, color, -1)
                cv2.circle(image, point, 7, (255, 255, 255), 1)  # Borde blanco
        
        return image
    
    def draw_status_overlay(self, image):
        """Dibujar información de estado en la imagen"""
        height, width = image.shape[:2]
        
        # Fondo semitransparente para texto
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (300, 120), (0, 0, 0), -1)
        image = cv2.addWeighted(overlay, 0.5, image, 0.5, 0)
        
        # Información de estado
        y_offset = 30
        line_height = 25
        
        # Estado segmentación
        seg_status = "ON" if self.show_segmentation else "OFF"
        seg_color = (0, 255, 0) if self.show_segmentation else (0, 0, 255)
        cv2.putText(image, f"Segmentation: {seg_status}", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, seg_color, 2)
        
        # Estado carriles
        lanes_status = "ON" if self.show_lanes else "OFF"
        lanes_color = (0, 255, 0) if self.show_lanes else (0, 0, 255)
        cv2.putText(image, f"Lanes: {lanes_status}", (10, y_offset + line_height),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, lanes_color, 2)
        
        # Estado imagen original
        orig_status = "ON" if self.show_original else "OFF"
        orig_color = (0, 255, 0) if self.show_original else (0, 0, 255)
        cv2.putText(image, f"Original: {orig_status}", (10, y_offset + 2*line_height),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, orig_color, 2)
        
        # Contador de carriles
        if self.current_lanes:
            lane_count = len(self.current_lanes.lanes)
            cv2.putText(image, f"Lanes detected: {lane_count}", 
                       (10, y_offset + 3*line_height),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return image
    
    def visualize_callback(self):
        """Visualización principal"""
        if self.current_image is None:
            return
        
        # Crear imagen base según configuración
        if self.show_original:
            display = self.current_image.copy()
        else:
            # Fondo negro
            display = np.zeros_like(self.current_image)
        
        # Aplicar overlay de segmentación si está habilitado
        if self.show_segmentation and self.current_seg_mask is not None:
            display = self.apply_segmentation_overlay(display)
        
        # Dibujar carriles si están habilitados
        if self.show_lanes and self.current_lanes is not None:
            display = self.draw_lanes(display)
        
        
        # Publicar imagen visualizada (opcional)
        try:
            viz_msg = self.bridge.cv2_to_imgmsg(display, 'bgr8')
            viz_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_visualization.publish(viz_msg)
        except:
            pass

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info("Supervisor de RPLIDAR apagado por usuario")
        pass
    except Exception as e:
        node.get_logger().error(f"Error fatal: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()