#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from custom_interfaces.msg import SegmentationData, DetectionArray, Detection
from cv_bridge import CvBridge
import time
from collections import defaultdict
from custom_interfaces.msg import PixelPoint

# IMPORTANTE: Message Filters para sincronización
from message_filters import Subscriber, ApproximateTimeSynchronizer

class CombinedVisualizerNode(Node):
    def __init__(self):
        super().__init__('combined_visualizer')
        
        self.bridge = CvBridge()
        
        # Colores para Tracking
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
        
        # Estado actual
        self.current_image = None
        self.current_centerline_pixels = None
        self.current_lane_pixels = None
        self.current_lane_selection_pixels = None
        self.current_detections = None
        
        self.last_fps_time = time.time()
        self.frame_count = 0
        
        # QoS para suscripciones
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # --- CREAR SUBSCRIBERS CON message_filters ---
        # 1. Subscriber para imagen comprimida
        self.image_sub = Subscriber(
            self, 
            CompressedImage, 
            '/image_raw/compressed',
            qos_profile=qos
        )
        
        # 2. Subscriber para detecciones
        self.detection_sub = Subscriber(
            self,
            DetectionArray,
            '/detection/results',
            qos_profile=qos
        )
        
        # 3. Subscriber para candidatos de carril segmentacion
        self.lane_candidates_sub = Subscriber(
            self,
            PixelPoint,
            '/lane/pixel_candidates',
            qos_profile=qos
        )
        
        # 4. Subscriber para centerline
        """self.centerline_sub = Subscriber(
            self,
            PixelPoint,
            '/lane/ipm_inverse_CL',
            qos_profile=qos
        )"""
        
        # 5. Subscriber para lane selection
        self.lane_selection_sub = Subscriber(
            self,
            PixelPoint,
            '/lane/ipm_inverse_pixel_points',
            qos_profile=qos
        )
        
        # --- SINCRONIZADOR APROXIMADO ---
        # Sincroniza imagen + detecciones + candidatos de carril
        # Puedes ajustar el tamaño de la cola (slop) según la latencia de tu sistema
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.detection_sub, self.lane_candidates_sub, 
              self.lane_selection_sub],
            queue_size=10,     # Tamaño de la cola de mensajes
            slop=0.05         # Tolerancia de 50ms entre timestamps
        )
        self.sync.registerCallback(self.sync_callback)
        
        # Publicador
        self.viz_pub = self.create_publisher(Image, '/debug/camera_front', 10)

    def sync_callback(self, img_msg, det_msg, lane_candidates_msg, lane_selection_msg):
        """
        Callback sincronizado. Todos los mensajes tienen timestamps aproximados.
        """
        try:
            # 1. Decodificar imagen
            np_arr = np.frombuffer(img_msg.data, np.uint8)
            self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if self.current_image is None:
                return
            
            # 2. Procesar detecciones
            if det_msg and hasattr(det_msg, 'detections'):
                self.current_detections = det_msg.detections
            else:
                self.current_detections = None
            
            # 3. Procesar candidatos de carril
            if lane_candidates_msg and len(lane_candidates_msg.u) > 0:
                self.current_lane_pixels = list(zip(lane_candidates_msg.u, lane_candidates_msg.v))
            else:
                self.current_lane_pixels = None
            
            # 4. Procesar centerline
            """if centerline_msg and len(centerline_msg.u) > 0:
                self.current_centerline_pixels = list(zip(centerline_msg.u, centerline_msg.v))
            else:
                self.current_centerline_pixels = None"""
            
            # 5. Procesar lane selection
            if lane_selection_msg and len(lane_selection_msg.u) > 0:
                self.current_lane_selection_pixels = list(zip(lane_selection_msg.u, lane_selection_msg.v))
            else:
                self.current_lane_selection_pixels = None
            
            # 6. Visualizar
            self.process_and_publish()
            
            # Log de sincronización (cada 30 frames)
            """if self.frame_count % 30 == 0:
                img_ts = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9
                det_ts = det_msg.header.stamp.sec + det_msg.header.stamp.nanosec * 1e-9 if det_msg else 0
                self.get_logger().info(
                    f"Sincronización - Img: {img_ts:.3f} | "
                    f"Det: {det_ts:.3f} | "
                    f"Diff: {(det_ts - img_ts)*1000:.1f}ms"
                )"""
            
            self.update_fps()
            
        except Exception as e:
            self.get_logger().error(f"Error en sync_callback: {e}")

    def process_and_publish(self):
        """Procesa y publica la imagen visualizada"""
        if self.current_image is None:
            return
        
        try:
            display_image = self.current_image.copy()
            
            # 1. Dibujar lane selection overlay
            if self.current_lane_selection_pixels is not None:
                display_image = self.draw_lane_selection_overlay(display_image)
            
            # 2. Dibujar píxeles de carril
            if self.current_lane_pixels is not None:
                display_image = self.draw_pixels(display_image, self.current_lane_pixels, (0, 255, 0))
            
            # 3. Dibujar centerline
            """if self.current_centerline_pixels is not None:
                display_image = self.draw_pixels(display_image, self.current_centerline_pixels, (255, 0, 255))"""

            # 4. Dibujar detecciones de tracking
            if self.current_detections is not None:
                for det in self.current_detections:
                    color = self.class_colors.get(det.class_id, (255, 255, 255))
                    # Dibujar bounding box
                    cv2.rectangle(display_image, (det.u1, det.v1), (det.u2, det.v2), color, 2)
                    # Etiqueta ID
                    label = f"ID:{det.track_id}"
                    cv2.putText(display_image, label, (det.u1, det.v1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Publicar imagen visualizada
            viz_msg = self.bridge.cv2_to_imgmsg(display_image, 'bgr8')
            viz_msg.header.stamp = self.get_clock().now().to_msg()
            self.viz_pub.publish(viz_msg)
        except Exception as e:
            self.get_logger().error(f"Error en visualización: {e}")

    def draw_lane_selection_overlay(self, image):
        """Dibuja overlay de selección de carril"""
        if self.current_lane_selection_pixels is None or len(self.current_lane_selection_pixels) < 2:
            return image
        height, width = image.shape[:2]
        points_by_v = defaultdict(list)
        for u, v in self.current_lane_selection_pixels:
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
        cv2.fillPoly(image, [polygon_points], (235, 183, 0)) 
        return image
    
    def draw_pixels(self, image, pixels, color=(255, 255, 0)):
        """Dibuja puntos en la imagen"""
        if pixels is None or len(pixels) == 0: 
            return image
        pixel_color = (color[2], color[1], color[0])
        for u, v in pixels:
            x, y = int(u), int(v)
            if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                cv2.circle(image, (x, y), 3, pixel_color, -1)
        return image
    
    def update_fps(self):
        """Actualiza y muestra FPS"""
        self.frame_count += 1
        curr = time.time()
        if curr - self.last_fps_time >= 2.0:
            #self.get_logger().info(f"FPS: {self.frame_count / (curr - self.last_fps_time):.1f}")
            self.frame_count = 0
            self.last_fps_time = curr

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