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

     
     

        # Buffers
        self.current_image = None
        self.current_detections = None # Buffer para tracking
        self.last_fps_time = time.time()
        self.frame_count = 0
        
        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # --- TUS SUSCRIPCIONES ORIGINALES ---
        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, best_effort_qos)
        
        self.pixels_sub = self.create_subscription(
            PixelPoint, '/lane/ipm_inverse_CL', self.pixels_centerline_callback, 10)

        self.pixels_lane_sub = self.create_subscription(
            PixelPoint, '/segmentation_data', self.pixels_lane_callback, 10)
        
        self.lane_selection_sub = self.create_subscription(
            PixelPoint, '/lane/ipm_inverse_pixel_points', self.lane_selection_callback, 10)
        
        # --- NUEVA SUSCRIPCIÓN TRACKING (Sin dañar el resto) ---
        self.det_sub = self.create_subscription(
            DetectionArray, '/detection/results', self.detections_callback, 10)

        self.current_centerline_pixels = None
        self.current_lane_pixels = None
        self.current_lane_selection_pixels = None
        
        self.viz_pub = self.create_publisher(Image, '/debug/camera_front', 10)
        self.get_logger().info("Visualizador combinado con Tracking inyectado listo")

    def detections_callback(self, msg):
        self.current_detections = msg.detections

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_and_publish()
        except Exception as e:
            self.get_logger().error(f"Error imagen: {e}")

    def pixels_centerline_callback(self, msg):
        if len(msg.u) == 0: self.current_centerline_pixels = None
        else: self.current_centerline_pixels = list(zip(msg.u, msg.v))

    def pixels_lane_callback(self, msg):
        if len(msg.u) == 0: self.current_lane_pixels = None
        else: self.current_lane_pixels = list(zip(msg.u, msg.v))
    
    def lane_selection_callback(self, msg):
        if len(msg.u) == 0: self.current_lane_selection_pixels = None
        else: self.current_lane_selection_pixels = list(zip(msg.u, msg.v))

    def process_and_publish(self):
        if self.current_image is None: return
        
        try:
            display_image = self.current_image.copy()
            
            # 1. TU DIBUJO DE CARRIL ORIGINAL (INTACTO)
            if self.current_lane_selection_pixels is not None:
                display_image = self.draw_lane_selection_overlay(display_image)
            
            # 2. TUS PÍXELES ORIGINALES
            if self.current_lane_pixels is not None:
                display_image = self.draw_pixels(display_image, self.current_lane_pixels, (0, 255, 0))
            if self.current_centerline_pixels is not None:
                display_image = self.draw_pixels(display_image, self.current_centerline_pixels, (255, 0, 255))

            # 3. NUEVO: CAPA DE TRACKING (Inyectada al final)
            if self.current_detections is not None:
                for det in self.current_detections:
                    color = self.class_colors.get(det.class_id, (255, 255, 255))
                    # Dibujar Bbox
                    cv2.rectangle(display_image, (det.u1, det.v1), (det.u2, det.v2), color, 2)
                    # Etiqueta ID
                    label = f"ID:{det.track_id}"
                    cv2.putText(display_image, label, (det.u1, det.v1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            viz_msg = self.bridge.cv2_to_imgmsg(display_image, 'bgr8')
            viz_msg.header.stamp = self.get_clock().now().to_msg()
            self.viz_pub.publish(viz_msg)
            self.update_fps()
            
        except Exception as e:
            self.get_logger().error(f"Error en visualización: {e}")

    # --- TU FUNCIÓN ORIGINAL INTACTA ---
    def draw_lane_selection_overlay(self, image):
        if self.current_lane_selection_pixels is None or len(self.current_lane_selection_pixels) < 2:
            return image
        height, width = image.shape[:2]
        points_by_v = defaultdict(list)
        for u, v in self.current_lane_selection_pixels:
            v_int, u_int = int(v), int(u)
            if 0 <= v_int < height:
                u_clamped = max(0, min(u_int, width - 1))
                points_by_v[v_int].append(u_clamped)
        if not points_by_v: return image
        sorted_vs = sorted(points_by_v.keys())
        min_v_detected = sorted_vs[0]
        left_side, right_side = [], []
        last_min_u = min(points_by_v[min_v_detected])
        last_max_u = max(points_by_v[min_v_detected])
        for v in range(min_v_detected, height):
            if v in points_by_v:
                curr_min, curr_max = min(points_by_v[v]), max(points_by_v[v])
                if last_min_u <= 1: curr_min = 0
                if last_max_u >= width - 2: curr_max = width - 1
                last_min_u, last_max_u = curr_min, curr_max
            else:
                if last_min_u < 15: last_min_u = 0
                if last_max_u > width - 15: last_max_u = width - 1
            left_side.append([last_min_u, v])
            right_side.append([last_max_u, v])
        polygon_points = np.array(left_side + right_side[::-1], dtype=np.int32)
        cv2.fillPoly(image, [polygon_points], (235, 183, 0)) 
        return image
    
    # --- TU FUNCIÓN ORIGINAL INTACTA ---
    def draw_pixels(self, image, pixels, color=(255, 255, 0)):
        if pixels is None or len(pixels) == 0: return image
        pixel_color = (color[2], color[1], color[0])
        for u, v in pixels:
            x, y = int(u), int(v)
            if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                cv2.circle(image, (x, y), 3, pixel_color, -1)
        return image
    
    def update_fps(self):
        self.frame_count += 1
        curr = time.time()
        if curr - self.last_fps_time >= 2.0:
            self.frame_count = 0
            self.last_fps_time = curr

def main(args=None):
    rclpy.init(args=args)
    node = CombinedVisualizerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()