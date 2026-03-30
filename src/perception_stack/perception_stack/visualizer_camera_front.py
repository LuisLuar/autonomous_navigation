#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from custom_interfaces.msg import ObjectInfoArray, PixelPoint
from cv_bridge import CvBridge
import time
from collections import defaultdict

class CombinedVisualizerNode(Node):
    def __init__(self):
        super().__init__('combined_visualizer')
        self.bridge = CvBridge()
        
        # --- CONFIGURACIÓN DE COLORES Y ESTILOS ---
        self.class_colors = {
            'person': (0, 220, 255), 'car': (0, 127, 255),
            'bicycle': (255, 191, 0), 'bus': (0, 0, 255), 'truck': (0, 0, 255),
        }
        self.trajectories = defaultdict(list)
        self.max_trajectory_points = 10
        
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
        self.create_subscription(ObjectInfoArray, '/objects/fused_info', self.objects_callback, 10)
        self.create_subscription(PixelPoint, '/lane/pixel_candidates', self.lane_callback, 10)
        self.create_subscription(PixelPoint, '/lane/ipm_inverse_pixel_points', self.centerline_callback, 10)

        # Publicador de Debug
        self.viz_pub = self.create_publisher(Image, '/debug/camera_front', 10)
        self.get_logger().info("Visualizador Completo iniciado en Orin Nano.")

    # --- CALLBACKS ---
    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_and_publish()

    def objects_callback(self, msg): self.current_objects = msg
    def lane_callback(self, msg): self.current_lane_pixels = list(zip(msg.u, msg.v))
    def centerline_callback(self, msg): self.current_centerline_pixels = list(zip(msg.u, msg.v))

    # --- LÓGICA DE PROCESAMIENTO ---
    def process_and_publish(self):
        if self.current_image is None: return
        
        vis_img = self.current_image.copy()
        
        # 1. Dibujar Carriles (Verde - directo de TensorRT)
        if self.current_lane_pixels:
            for u, v in self.current_lane_pixels:
                if v < vis_img.shape[0] and u < vis_img.shape[1]:
                    cv2.circle(vis_img, (int(u), int(v)), 2, (0, 255, 0), -1)

        # 2. Dibujar Centerline (Magenta - calculado)
        if self.current_centerline_pixels:
            for u, v in self.current_centerline_pixels:
                if v < vis_img.shape[0] and u < vis_img.shape[1]:
                    cv2.circle(vis_img, (int(u), int(v)), 3, (255, 0, 255), -1)

        # 3. Dibujar Objetos y Trayectorias
        if self.current_objects:
            vis_img = self.draw_objects(vis_img)

        # 4. Overlay de Información y FPS
        vis_img = self.add_info_overlay(vis_img)
        
        # Publicar
        self.viz_pub.publish(self.bridge.cv2_to_imgmsg(vis_img, "bgr8"))
        self.update_fps()

    def draw_objects(self, img):
        for obj in self.current_objects.objects:
            color = self.class_colors.get(obj.class_name.lower(), (0, 255, 0))
            # Bounding Box
            cv2.rectangle(img, (obj.x1, obj.y1), (obj.x2, obj.y2), color, 2)
            
            # Etiqueta
            label = f"{obj.class_name} {obj.distance:.1fm}"
            cv2.putText(img, label, (obj.x1, obj.y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # Trayectorias (si hay track_id)
            if obj.track_id > 0:
                cx, cy = int(obj.center_x), int(obj.center_y)
                self.trajectories[obj.track_id].append((cx, cy))
                if len(self.trajectories[obj.track_id]) > self.max_trajectory_points:
                    self.trajectories[obj.track_id].pop(0)
        
        # Dibujar líneas de trayectoria
        for tid in self.trajectories:
            points = self.trajectories[tid]
            for i in range(len(points) - 1):
                cv2.line(img, points[i], points[i+1], (255, 255, 255), 1)
        return img

    def add_info_overlay(self, img):
        # Fondo oscuro para telemetría
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (220, 80), (0,0,0), -1)
        img = cv2.addWeighted(overlay, 0.4, img, 0.6, 0)
        
        cv2.putText(img, f"FPS: {self.fps:.1f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        status = "LANES: OK" if self.current_lane_pixels else "LANES: LOST"
        cv2.putText(img, status, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return img

    def update_fps(self):
        self.frame_count += 1
        now = time.time()
        if now - self.last_fps_time >= 1.0:
            self.fps = self.frame_count / (now - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = now

def main():
    rclpy.init()
    rclpy.spin(CombinedVisualizerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()