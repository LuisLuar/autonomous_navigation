#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from custom_interfaces.msg import SegmentationData, ObjectInfoArray
from cv_bridge import CvBridge
import time
from collections import defaultdict


class CombinedVisualizerNode(Node):
    def __init__(self):
        super().__init__('combined_visualizer')
        
        self.bridge = CvBridge()
        
        # Configuración de segmentación
        self.seg_colors = {
            0: [0, 0, 0],        # Negro - Fondo
            1: [0, 200, 0],      # Verde - Área transitable
            2: [0, 255, 255]     # Amarillo - Carriles
        }
        self.seg_alpha = 0.3
        
        # Configuración de objetos
        self.class_colors = {
            'person': (0, 220, 255),      # Amarillo
            'car': (0, 127, 255),         # Naranja
            'bicycle': (255, 191, 0),     # Azul claro
            'motorcycle': (255, 64, 0),   # Azul
            'bus': (0, 0, 255),           # Rojo
            'truck': (0, 0, 255),         # Rojo
        }
        
        # Buffers
        self.current_image = None
        self.current_seg_mask = None
        self.current_objects = None
        self.last_fps_time = time.time()
        self.frame_count = 0
        
        # Tracks para trayectorias simples
        self.trajectories = defaultdict(list)
        self.max_trajectory_points = 10
        
        # Suscripciones
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10
        )
        self.segmentation_sub = self.create_subscription(
            SegmentationData, '/segmentation/data', self.segmentation_callback, 10
        )
        self.objects_sub = self.create_subscription(
            ObjectInfoArray, '/objects/fused_info', self.objects_callback, 10
        )
        
        # Publicador único
        self.viz_pub = self.create_publisher(Image, '/debug/camera_front', 10)
        
        self.get_logger().info("Visualizador combinado inicializado")
    
    def image_callback(self, msg):
        """Recibe imagen de cámara"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_and_publish()
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}", throttle_duration_sec=5.0)
    
    def segmentation_callback(self, msg):
        """Recibe datos de segmentación"""
        try:
            # Reconstruir máscara desde bytes
            mask_flat = np.frombuffer(msg.mask_data, dtype=np.uint8)
            self.current_seg_mask = mask_flat.reshape((msg.height, msg.width))
            self.process_and_publish()
        except Exception as e:
            self.get_logger().error(f"Error procesando segmentación: {e}", throttle_duration_sec=5.0)
    
    def objects_callback(self, msg):
        """Recibe objetos detectados"""
        self.current_objects = msg
        self.process_and_publish()
    
    def process_and_publish(self):
        """Procesa y publica la imagen combinada"""
        if self.current_image is None:
            return
        
        try:
            # Crear copia para visualización
            display_image = self.current_image.copy()
            height, width = display_image.shape[:2]
            
            # 1. Aplicar segmentación (si existe)
            if self.current_seg_mask is not None:
                display_image = self.apply_segmentation(display_image)
            
            # 2. Dibujar objetos (si existen)
            if self.current_objects is not None:
                display_image = self.draw_objects(display_image)
            
            # 3. Añadir información de estado
            display_image = self.add_status_overlay(display_image)
            
            # 4. Calcular y mostrar FPS
            display_image = self.add_fps_display(display_image)
            
            # Publicar
            viz_msg = self.bridge.cv2_to_imgmsg(display_image, 'bgr8')
            viz_msg.header.stamp = self.get_clock().now().to_msg()
            self.viz_pub.publish(viz_msg)
            
            # Actualizar FPS
            self.update_fps()
            
        except Exception as e:
            self.get_logger().error(f"Error en visualización: {e}", throttle_duration_sec=5.0)
    
    def apply_segmentation(self, image):
        """Aplicar overlay de segmentación"""
        # Crear máscara coloreada
        colored_mask = np.zeros((self.current_seg_mask.shape[0], 
                               self.current_seg_mask.shape[1], 3), dtype=np.uint8)
        
        for class_id, color in self.seg_colors.items():
            colored_mask[self.current_seg_mask == class_id] = color
        
        # Overlay para área transitable
        drivable_mask = (self.current_seg_mask == 1)
        if np.any(drivable_mask):
            overlay = image.copy()
            overlay[drivable_mask] = colored_mask[drivable_mask]
            image = cv2.addWeighted(overlay, self.seg_alpha, image, 1 - self.seg_alpha, 0)
        
        # Contornos para carriles
        lane_mask = (self.current_seg_mask == 2).astype(np.uint8) * 255
        contours, _ = cv2.findContours(lane_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 50:
                cv2.drawContours(image, [contour], -1, (0, 255, 255), 1)
        
        return image
    
    def draw_objects(self, image):
        """Dibujar objetos detectados"""
        height, width = image.shape[:2]
        
        for obj in self.current_objects.objects:
            # Coordenadas del bounding box
            x1, y1, x2, y2 = obj.x1, obj.y1, obj.x2, obj.y2
            
            # Validar coordenadas
            if x1 >= x2 or y1 >= y2 or x2 > width or y2 > height:
                continue
            
            # Obtener color según clase
            color = self.get_object_color(obj.class_name, obj.track_id)
            
            # Dibujar bounding box
            thickness = 2 if obj.track_id > 0 else 1
            cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
            
            # Centro del objeto
            center_x = int(obj.center_x)
            center_y = int(obj.center_y)
            cv2.circle(image, (center_x, center_y), 2, color, -1)
            
            # Etiqueta simplificada
            label = self.get_object_label(obj)
            if label:
                # Fondo para texto
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.5
                thickness = 1
                
                (tw, th), _ = cv2.getTextSize(label, font, font_scale, thickness)
                label_y = max(y1 - 3, th + 3)
                label_x = max(x1, 5)
                
                if label_x + tw + 6 < width:
                    cv2.rectangle(
                        image,
                        (label_x - 2, label_y - th - 2),
                        (label_x + tw + 2, label_y + 2),
                        color,
                        -1
                    )
                    
                    cv2.putText(
                        image,
                        label,
                        (label_x, label_y),
                        font,
                        font_scale,
                        (255, 255, 255),
                        thickness,
                        cv2.LINE_AA
                    )
            
            # Actualizar trayectoria si tiene track_id
            if obj.track_id > 0:
                self.update_trajectory(obj.track_id, center_x, center_y)
        
        # Dibujar trayectorias
        image = self.draw_trajectories(image)
        
        return image
    
    def get_object_color(self, class_name, track_id=0):
        """Obtener color para un objeto"""
        class_lower = class_name.lower()
        
        # Buscar color por clase
        for key, color in self.class_colors.items():
            if key in class_lower:
                base_color = color
                break
        else:
            # Color por defecto basado en hash
            h = hash(class_lower) & 0xFF
            base_color = (h % 200, (h * 37) % 200, (h * 73) % 200)
        
        # Variar color si tiene track_id
        if track_id > 0:
            variation = track_id % 3
            if variation == 1:
                return (min(255, base_color[0] + 50), base_color[1], base_color[2])
            elif variation == 2:
                return (base_color[0], min(255, base_color[1] + 50), base_color[2])
        
        return base_color
    
    def get_object_label(self, obj):
        """Crear etiqueta simplificada para objeto"""
        parts = []
        
        # Nombre de clase abreviado
        class_name = obj.class_name
        if len(class_name) > 8:
            class_name = class_name[:6] + ".."
        parts.append(class_name)
        
        # ID de track si existe
        if obj.track_id > 0:
            parts.append(f"#{obj.track_id}")
        
        # Confianza si es alta
        if obj.confidence > 0.7:
            parts.append(f"{int(obj.confidence * 100)}%")
        
        # Distancia si está disponible
        if obj.distance_valid and obj.distance > 0:
            parts.append(f"{obj.distance:.1f}m")
        
        return " ".join(parts)
    
    def update_trajectory(self, track_id, x, y):
        """Actualizar trayectoria de un objeto"""
        if track_id in self.trajectories:
            self.trajectories[track_id].append((x, y))
            if len(self.trajectories[track_id]) > self.max_trajectory_points:
                self.trajectories[track_id].pop(0)
        else:
            self.trajectories[track_id] = [(x, y)]
        
        # Limpiar trayectorias antiguas
        if len(self.trajectories) > 20:
            # Mantener solo los 15 tracks más recientes
            keys = list(self.trajectories.keys())
            if len(keys) > 15:
                for key in keys[:-15]:
                    del self.trajectories[key]
    
    def draw_trajectories(self, image):
        """Dibujar trayectorias de objetos trackeados"""
        for track_id, points in self.trajectories.items():
            if len(points) < 2:
                continue
            
            # Obtener color para este track
            color = self.get_object_color("car", track_id)
            
            # Dibujar línea conectando puntos
            for i in range(len(points) - 1):
                pt1 = (int(points[i][0]), int(points[i][1]))
                pt2 = (int(points[i+1][0]), int(points[i+1][1]))
                cv2.line(image, pt1, pt2, color, 1, cv2.LINE_AA)
        
        return image
    
    def add_status_overlay(self, image):
        """Añadir información de estado"""
        height, width = image.shape[:2]
        
        # Fondo para texto
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (300, 110), (0, 0, 0), -1)
        image = cv2.addWeighted(overlay, 0.6, image, 0.4, 0)
        
        # Texto
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 1
        y_pos = 35
        line_spacing = 25
        
        # Título
        cv2.putText(
            image,
            "CAMERA FRONT",
            (20, y_pos),
            font,
            font_scale,
            (255, 255, 255),
            thickness + 1
        )
        
        # Información de segmentación
        if self.current_seg_mask is not None:
            total_pixels = self.current_seg_mask.size
            drivable_pct = (np.sum(self.current_seg_mask == 1) / total_pixels) * 100
            lane_pct = (np.sum(self.current_seg_mask == 2) / total_pixels) * 100
            
            cv2.putText(
                image,
                f"Drivable: {drivable_pct:.1f}%",
                (20, y_pos + line_spacing),
                font,
                font_scale * 0.8,
                (0, 255, 0),
                thickness
            )
            
            cv2.putText(
                image,
                f"Lanes: {lane_pct:.1f}%",
                (20, y_pos + 2 * line_spacing),
                font,
                font_scale * 0.8,
                (0, 255, 255),
                thickness
            )
        
        # Información de objetos
        if self.current_objects is not None:
            obj_count = len(self.current_objects.objects)
            track_count = len(set(obj.track_id for obj in self.current_objects.objects if obj.track_id > 0))
            
            cv2.putText(
                image,
                f"Objects: {obj_count} (Tracks: {track_count})",
                (20, y_pos + 3 * line_spacing),
                font,
                font_scale * 0.8,
                (255, 200, 0),
                thickness
            )
        
        return image
    
    def add_fps_display(self, image):
        """Añadir display de FPS"""
        # Calcular FPS aproximado
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        # Mostrar en esquina inferior derecha
        fps_text = f"FPS: {fps:.1f}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 1
        
        (tw, th), _ = cv2.getTextSize(fps_text, font, font_scale, thickness)
        pos_x = image.shape[1] - tw - 20
        pos_y = image.shape[0] - 10
        
        # Fondo
        cv2.rectangle(
            image,
            (pos_x - 5, pos_y - th - 5),
            (pos_x + tw + 5, pos_y + 5),
            (0, 0, 0),
            -1
        )
        
        # Texto
        cv2.putText(
            image,
            fps_text,
            (pos_x, pos_y),
            font,
            font_scale,
            (0, 255, 255),
            thickness + 1
        )
        
        return image
    
    def update_fps(self):
        """Actualizar contador de FPS"""
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 2.0:
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def destroy_node(self):
        """Cleanup"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CombinedVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Visualizador detenido por usuario")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()