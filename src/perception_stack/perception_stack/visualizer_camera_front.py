#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from custom_interfaces.msg import PixelPoint, DetectionArray, Detection
from cv_bridge import CvBridge
import time
from collections import defaultdict
import math

class CombinedVisualizerNode(Node):
    def __init__(self):
        super().__init__('combined_visualizer')
        self.bridge = CvBridge()
        
        # Colores para Tracking (Inyectado)
        self.class_colors = {
            0: (255, 0, 0),     #0: person - Azul
            1: (0, 255, 0),     #1: bicycle - Verde
            2: (0, 0, 255),     #2: car - Rojo
            3: (255, 255, 0),   #3: motorcycle - Cyan
            4: (255, 0, 255),   #4: bus - Magenta
            5: (0, 255, 255),   #5: truck - Amarillo
            6: (128, 0, 128),   #6: speed_bump - Púrpura
            7: (255, 128, 0),   #7: crosswalk - Naranja
            8: (0, 128, 255),   #8: speed_bump_signage - Azul claro
            9: (128, 128, 0)    #9: crosswalk_signage - Verde oliva
        }

        self.class_names = {
            0: "person",
            1: "bicycle", 
            2: "car",
            3: "motorcycle",
            4: "bus",
            5: "truck",
            6: "speed_bump",
            7: "crosswalk",
            8: "speed_bump_signage",
            9: "crosswalk_signage"
        }

        self.current_detections = None # Buffer para tracking
        
        # --- BUFFERS DE DATOS ---
        self.current_image = None
        self.current_objects = None
        self.current_lane_pixels = None       # Puntos verdes (TensorRT)
        self.current_centerline_pixels = None  # Puntos magenta (IPM)
        
        # --- MÉTRICAS ---
        self.last_fps_time = time.time()
        self.frame_count = 0
        self.fps = 0.0

        # --- SUSCRIPCIONES (QoS adaptado a Orin Nano) ---
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        self.create_subscription(PixelPoint, '/lane/pixel_candidates', self.lane_callback, 10)
        self.create_subscription(PixelPoint, '/lane/ipm_inverse_pixel_points', self.centerline_callback, 10)
        self.det_sub = self.create_subscription(DetectionArray, '/detection/results', self.detections_callback, 10)

        # Publicador de Debug
        self.viz_pub = self.create_publisher(Image, '/debug/camera_front', 10)
        #self.get_logger().info("Visualizador Completo iniciado en Orin Nano.")

    # --- CALLBACKS ---
    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_and_publish()

    def lane_callback(self, msg): self.current_lane_pixels = list(zip(msg.u, msg.v))
    def centerline_callback(self, msg): self.current_centerline_pixels = list(zip(msg.u, msg.v))
    def detections_callback(self, msg):
        self.current_detections = msg.detections

    # --- LÓGICA DE PROCESAMIENTO ---
    def process_and_publish(self):
        if self.current_image is None: return
        
        try: 
            vis_img = self.current_image.copy()

            # 2. Dibujar Centerline (Magenta - calculado)
            if self.current_centerline_pixels:
                vis_img = self.draw_lane_selection_overlay(vis_img)
            
            # 1. Dibujar Carriles (Verde - directo de TensorRT)
            if self.current_lane_pixels:
                for u, v in self.current_lane_pixels:
                    if v < vis_img.shape[0] and u < vis_img.shape[1]:
                        cv2.circle(vis_img, (int(u), int(v)), 2, (0, 255, 0), -1)

            
                """for u, v in self.current_centerline_pixels:
                    if v < vis_img.shape[0] and u < vis_img.shape[1]:
                        cv2.circle(vis_img, (int(u), int(v)), 3, (255, 0, 255), -1)"""
            

            # 4. Dibujar detecciones de tracking
            if self.current_detections is not None:
                for det in self.current_detections:
                    color = self.class_colors.get(det.class_id, (255, 255, 255))
                    label_class = self.class_names.get(det.class_id, f"class_{det.class_id}")
                    
                    # 1. Dibujar el bounding box principal
                    cv2.rectangle(vis_img, (det.u1, det.v1), (det.u2, det.v2), color, 2)
                    
                    # 2. Preparar el texto de la etiqueta
                    label = f"{label_class}: {det.track_id} || {det.confidence:.2f}%"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    thickness = 1 # Grosor del texto (1 suele verse mejor en etiquetas pequeñas)

                    # 3. Calcular el tamaño del texto para el fondo
                    (w, h), baseline = cv2.getTextSize(label, font, font_scale, thickness)
                    
                    # 4. Dibujar el rectángulo relleno (Fondo de la etiqueta)
                    # El rectángulo va desde la esquina superior izquierda (u1, v1) 
                    # hacia arriba para no tapar el objeto
                    cv2.rectangle(vis_img, (det.u1, det.v1 - h - baseline - 5), (det.u1 + w, det.v1), color, -1)
                    
                    # 5. Dibujar el texto en color NEGRO (0, 0, 0) sobre el recuadro
                    cv2.putText(vis_img, label, (det.u1, det.v1 - 5), 
                                font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)


            # 4. Overlay de Información y FPS
            vis_img = self.add_info_overlay(vis_img)
            
            # Publicar
            self.viz_pub.publish(self.bridge.cv2_to_imgmsg(vis_img, "bgr8"))
            self.update_fps()

        except Exception as e:
            self.get_logger().error(f"Falla en procesamiento: {e}")

    def add_info_overlay(self, img):
        # Fondo oscuro para telemetría
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (220, 80), (0,0,0), -1)
        img = cv2.addWeighted(overlay, 0.4, img, 0.6, 0)
        
        cv2.putText(img, f"FPS: {self.fps:.1f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        status = "LANES: OK" if self.current_lane_pixels else "LANES: LOST"
        cv2.putText(img, status, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return img
    
    def draw_lane_selection_overlay(self, image):
        """Dibuja overlay de selección de carril con blend 50/50 solo en la intersección"""
        if self.current_centerline_pixels is None or len(self.current_centerline_pixels) < 2:
            return image
        
        height, width = image.shape[:2]
        points_by_v = defaultdict(list)
        
        for u, v in self.current_centerline_pixels:
            v_int, u_int = int(v), int(u)
            if 0 <= v_int < height:
                u_clamped = max(0, min(u_int, width - 1))
                points_by_v[v_int].append(u_clamped)
        
        if not points_by_v: 
            return image
        
        sorted_vs = sorted(points_by_v.keys())
        min_v_detected = sorted_vs[0]
        left_side, right_side = [], []
        last_min_u = min(points_by_v[min_v_detected])
        last_max_u = max(points_by_v[min_v_detected])
        
        for v in range(min_v_detected, height):
            if v in points_by_v:
                curr_min, curr_max = min(points_by_v[v]), max(points_by_v[v])
                if last_min_u <= 1: 
                    curr_min = 0
                if last_max_u >= width - 2: 
                    curr_max = width - 1
                last_min_u, last_max_u = curr_min, curr_max
            else:
                if last_min_u < 15: 
                    last_min_u = 0
                if last_max_u > width - 15: 
                    last_max_u = width - 1
            left_side.append([last_min_u, v])
            right_side.append([last_max_u, v])
        
        polygon_points = np.array(left_side + right_side[::-1], dtype=np.int32)
        
        # Crear máscara binaria
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [polygon_points], 255)
        
        # Crear overlay color
        overlay = np.full_like(image, (235, 183, 0), dtype=np.uint8)
        
        # Blend 50/50 usando operaciones vectorizadas
        alpha = 0.6
        mask_3ch = mask[:, :, np.newaxis] / 255.0  # Expandir a 3 canales y normalizar
        
        # Aplicar blend solo donde máscara es 1
        image = (image * (1 - alpha * mask_3ch) + overlay * (alpha * mask_3ch)).astype(np.uint8)
        
        return image

    def update_fps(self):
        self.frame_count += 1
        now = time.time()
        if now - self.last_fps_time >= 1.0:
            self.fps = self.frame_count / (now - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = now

def main(args=None):
    rclpy.init(args=args)
    node = CombinedVisualizerNode()
    
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