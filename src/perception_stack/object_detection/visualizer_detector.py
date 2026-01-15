#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_interfaces.msg import DetectionArray
import cv2
import numpy as np


class YoloVisualizerNode(Node):
    def __init__(self):
        super().__init__('yolo_visualizer')
        
        # ============ CONFIGURACIÓN VISUAL ============
        # Declarar parámetros simples primero
        self.declare_parameter('show_confidence', True)
        self.declare_parameter('show_track_id', True)
        self.declare_parameter('line_thickness', 2)
        self.declare_parameter('font_scale', 0.5)
        self.declare_parameter('alpha_overlay', 0.15)
        self.declare_parameter('fps_display', False)
        # class_colors lo manejaremos como string JSON o lo eliminamos
        
        self.bridge = CvBridge()
        self.last_detections = None
        self.frame_count = 0
        self.last_time = self.get_clock().now()
        
        # Colores predefinidos para clases comunes (BGR)
        self.color_palette = {
            'person': (0, 220, 255),      # Amarillo
            'car': (0, 127, 255),         # Naranja
            'bicycle': (255, 191, 0),     # Azul claro
            'motorcycle': (255, 64, 0),   # Azul
            'bus': (0, 0, 255),           # Rojo
            'truck': (0, 0, 255),         # Rojo
            'dog': (255, 0, 255),         # Magenta
            'cat': (255, 128, 255),       # Rosa
        }
        
        # ============ SUBSCRIPTORES ============
        self.sub_image = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.callback_image, 10
        )
        
        self.sub_detections = self.create_subscription(
            DetectionArray, '/detection/results', self.callback_detections, 10
        )
        
        # ============ PUBLICADORES ============
        self.pub_annotated = self.create_publisher(
            Image, '/detection/annotated_image', 10
        )
        
        #self.get_logger().info(" Visualizador YOLO inicializado")

    def callback_detections(self, msg: DetectionArray):
        """Almacenar detecciones más recientes"""
        self.last_detections = msg

    def callback_image(self, msg: Image):
        """Procesar y anotar imagen"""
        if self.last_detections is None:
            return
        
        try:
            # Convertir imagen
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            overlay = img.copy()  # Para transparencias
        except Exception as e:
            #self.get_logger().warn(f"Error CV bridge: {e}", throttle_duration_sec=5.0)
            return
        
        # ============ DIBUJAR DETECCIONES ============
        detections_by_class = {}
        
        for det in self.last_detections.detections:
            # Obtener color según clase
            bbox_color = self.get_class_color(det.class_name)
            
            # Crear bbox con transparencia
            overlay_filled = overlay.copy()
            cv2.rectangle(
                overlay_filled, 
                (det.x1, det.y1), 
                (det.x2, det.y2), 
                bbox_color, 
                -1  # Relleno
            )
            
            # Aplicar transparencia al overlay
            alpha = self.get_parameter('alpha_overlay').value
            cv2.addWeighted(overlay_filled, alpha, overlay, 1 - alpha, 0, overlay)
            
            # Borde del bbox (sin transparencia)
            cv2.rectangle(
                img, 
                (det.x1, det.y1), 
                (det.x2, det.y2), 
                bbox_color, 
                self.get_parameter('line_thickness').value
            )
            
            # ============ ETIQUETA MINIMALISTA ============
            label_parts = []
            
            # Nombre de clase (abreviado si es largo)
            class_display = det.class_name
            if len(det.class_name) > 10:
                # Abreviar palabras largas manteniendo legibilidad
                if '_' in det.class_name:
                    # Si tiene underscores, tomar primeras letras de cada parte
                    parts = det.class_name.split('_')
                    class_display = ''.join(p[0].upper() for p in parts if p)
                else:
                    class_display = det.class_name[:8] + '..'
            
            # Añadir track_id si está configurado y disponible
            if self.get_parameter('show_track_id').value and hasattr(det, 'track_id') and det.track_id > 0:
                label_parts.append(f"{class_display} #{det.track_id}")
            else:
                label_parts.append(class_display)
                
            # Añadir confianza si está configurado
            if self.get_parameter('show_confidence').value:
                # Mostrar como porcentaje sin decimales
                conf_percent = int(det.confidence * 100)
                label_parts.append(f"{conf_percent}%")
            
            label = " ".join(label_parts)
            
            # Calcular tamaño del texto
            font_scale = self.get_parameter('font_scale').value
            (tw, th), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1
            )
            
            # Posición de la etiqueta (arriba del bbox)
            label_y = max(det.y1 - 5, th + 5)
            label_x = det.x1
            
            # Asegurar que la etiqueta no se salga de la imagen
            if label_x + tw + 10 > img.shape[1]:
                label_x = img.shape[1] - tw - 10
            if label_x < 0:
                label_x = 5
            
            # Fondo de etiqueta
            cv2.rectangle(
                img,
                (label_x, label_y - th - 5),
                (label_x + tw + 10, label_y + 5),
                bbox_color,
                -1
            )
            
            # Texto de etiqueta
            cv2.putText(
                img,
                label,
                (label_x + 5, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (255, 255, 255),  # Blanco
                1,
                cv2.LINE_AA
            )
            
            # Punto central (opcional, pequeño)
            center_x = (det.x1 + det.x2) // 2
            center_y = (det.y1 + det.y2) // 2
            cv2.circle(img, (center_x, center_y), 3, bbox_color, -1)
            
            # Contar por clase para estadísticas
            if det.class_name not in detections_by_class:
                detections_by_class[det.class_name] = 0
            detections_by_class[det.class_name] += 1
        
        # ============ APLICAR OVERLAY CON TRANSPARENCIA ============
        alpha = self.get_parameter('alpha_overlay').value
        cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
        
        # ============ PUBLICAR IMAGEN ANOTADA ============
        try:
            out_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            out_msg.header = msg.header
            self.pub_annotated.publish(out_msg)
        except Exception as e:
            #self.get_logger().warn(f"Error publicando imagen: {e}", throttle_duration_sec=5.0)
            pass

    def get_class_color(self, class_name):
        """Obtener color para una clase"""
        # Primero buscar en la paleta predefinida
        if class_name in self.color_palette:
            return self.color_palette[class_name]
        
        # Si no está, buscar coincidencia parcial (para nombres como "person 0.5")
        for key in self.color_palette:
            if key in class_name.lower():
                return self.color_palette[key]
        
        # Por defecto, generar color consistente
        return self.class_color_hash(class_name)
    
    @staticmethod
    def class_color_hash(name: str):
        """Generar color consistente basado en hash del nombre"""
        # Hash simple del nombre
        h = 0
        for ch in name:
            h = (h * 31 + ord(ch)) & 0xFFFFFFFF
        
        # Usar el hash para generar componentes RGB
        r = (h >> 16) & 0xFF
        g = (h >> 8) & 0xFF
        b = h & 0xFF
        
        # Asegurar que el color no sea demasiado oscuro o claro
        r = max(50, min(200, r))
        g = max(50, min(200, g))
        b = max(50, min(200, b))
        
        # Devolver en formato BGR para OpenCV
        return (b, g, r)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisualizerNode()
    try:
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            try:
                node.destroy_node()
                rclpy.shutdown()
            except:
                pass
        


if __name__ == '__main__':
    main()