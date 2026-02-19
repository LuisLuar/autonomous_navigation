#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2  # ¡AGREGADO PARA CORRECCIÓN DE DISTORSIÓN!
import json
import math

from custom_interfaces.msg import PixelPoint
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener

class PixelToMeterTransformLeft(Node):

    def __init__(self):
        super().__init__('pixel_to_meter_transform_left')

        # =================== CARGA DE CALIBRACIÓN ===================
        calibration_path = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration_left.json"
        try:
            with open(calibration_path, 'r') as f:
                calib = json.load(f)

            self.h = float(calib['camera_height'])
            self.pitch = float(calib['camera_pitch'])
            
            # Parámetros intrínsecos
            fx = float(calib['intrinsics']['fx'])
            fy = float(calib['intrinsics']['fy'])
            cx = float(calib['intrinsics']['cx'])
            cy = float(calib['intrinsics']['cy'])
            
            # ¡COEFICIENTES DE DISTORSIÓN - AGREGADOS!
            k1 = float(calib['distortion']['k1'])
            k2 = float(calib['distortion']['k2'])
            k3 = float(calib['distortion']['k3'])
            p1 = float(calib['distortion']['p1'])
            p2 = float(calib['distortion']['p2'])
            
            # Crear matriz de cámara K
            self.K = np.array([[fx, 0, cx],
                               [0, fy, cy],
                               [0, 0, 1]], dtype=np.float32)
            
            # Crear vector de distorsión
            self.dist = np.array([[k1, k2, p1, p2, k3]], dtype=np.float32)
            
            # Precalcular mapas de corrección (se inicializan después)
            self.mapx = None
            self.mapy = None
            self.new_K = None
            self.roi = None
            
            #self.get_logger().info(" Parámetros de calibración cargados (incluyendo distorsión)")

        except Exception as e:
            #self.get_logger().error(f" Error cargando calibración: {e}")
            rclpy.shutdown()
            return

        # =================== PARÁMETROS ===================
        self.declare_parameter("min_distance", 0.5)
        self.declare_parameter("max_distance", 8.0)
        self.declare_parameter("image_width", 640)      # ¡AGREGADO!
        self.declare_parameter("image_height", 480)     # ¡AGREGADO!
        self.declare_parameter("use_distortion_correction", True)  # ¡AGREGADO!

        self.declare_parameter("camera_offset_x", 0.18)
        self.declare_parameter("camera_offset_y", 0.42)
        self.declare_parameter("camera_yaw", 24)  # En deg

        self.min_distance = self.get_parameter("min_distance").value
        self.max_distance = self.get_parameter("max_distance").value
        self.cam_x = self.get_parameter("camera_offset_x").value
        self.cam_y = self.get_parameter("camera_offset_y").value
        self.cam_yaw = math.radians(self.get_parameter("camera_yaw").value)  # En radianes

        self.use_distortion_correction = self.get_parameter("use_distortion_correction").value

        # Precalcular seno y coseno del yaw para eficiencia
        self.cos_yaw = math.cos(self.cam_yaw)
        self.sin_yaw = math.sin(self.cam_yaw)

        # Inicializar mapas de corrección de distorsión
        image_width = self.get_parameter("image_width").value
        image_height = self.get_parameter("image_height").value
        self.init_undistort_maps(image_width, image_height)

        # ========== TRANSFORMADA A ODOM =============================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # =================== PUBLICADORES ===================
        self.pub_lane_candidates = self.create_publisher(PointCloud2, '/lane/meter_candidates_left', 10)

        # =================== SUBSCRIPCIONES ===================
        self.create_subscription(
            PixelPoint, '/lane/pixel_candidates_left',
            lambda msg: self.process_and_publish(msg, self.pub_lane_candidates), 10)

    def init_undistort_maps(self, width, height):
        """Precalcular mapas de corrección de distorsión para eficiencia"""
        if not self.use_distortion_correction:
            #self.get_logger().info(" Corrección de distorsión DESACTIVADA")
            self.new_K = self.K  # Usar la matriz original
            return
            
        try:
            # Calcular nueva matriz de cámara óptima
            self.new_K, self.roi = cv2.getOptimalNewCameraMatrix(
                self.K, self.dist, (width, height), 1, (width, height)
            )
            
            # Generar mapas de corrección (solo una vez)
            self.mapx, self.mapy = cv2.initUndistortRectifyMap(
                self.K, self.dist, None, self.new_K, (width, height), cv2.CV_32FC1
            )
            
            #self.get_logger().info(f" Mapas de corrección inicializados para {width}x{height}")
            
        except Exception as e:
            #self.get_logger().error(f" Error inicializando mapas de corrección: {e}")
            self.use_distortion_correction = False
            self.new_K = self.K

    def undistort_pixel(self, u, v):
        """Corregir distorsión de un píxel individual"""
        if not self.use_distortion_correction or self.mapx is None or self.mapy is None:
            return u, v  # Retornar sin corrección
        
        try:
            # Convertir a enteros y asegurar dentro de los límites
            x = int(np.clip(u, 0, self.mapx.shape[1] - 1))
            y = int(np.clip(v, 0, self.mapx.shape[0] - 1))
            
            # Obtener coordenadas corregidas
            u_corrected = float(self.mapx[y, x])
            v_corrected = float(self.mapy[y, x])
            
            return u_corrected, v_corrected
            
        except Exception as e:
            #self.get_logger().warn(f" Error corrigiendo píxel ({u}, {v}): {e}")
            return u, v

    # =================== IPM EXACTA ===================
    def pixel_to_meters(self, u, v):
        # PASO 1: Corregir distorsión del píxel (¡AGREGADO!)
        if self.use_distortion_correction:
            u_corr, v_corr = self.undistort_pixel(u, v)
        else:
            u_corr, v_corr = u, v
        
        # Pixel corregido → rayo en cámara
        # Usar new_K si está disponible, sino usar K original
        K_to_use = self.new_K if self.new_K is not None else self.K
        
        x = (u_corr - K_to_use[0, 2]) / K_to_use[0, 0]
        y = -(v_corr - K_to_use[1, 2]) / K_to_use[1, 1]
        
        ray = np.array([x, y, 1.0])

        # Rotación por pitch (MISMA que calibrador)
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)

        R = np.array([
            [1,  0,   0],
            [0,  cp,  sp],
            [0, -sp,  cp]
        ])

        ray_w = R @ ray

        # Intersección con suelo Y = 0
        if ray_w[1] >= 0:
            return None

        t = self.h / -ray_w[1]

        Xc = ray_w[0] * t   # lateral (derecha +) EN SISTEMA CÁMARA
        Zc = ray_w[2] * t   # adelante EN SISTEMA CÁMARA

        return Xc, Zc

    # =================== PIPELINE ===================
    def process_and_publish(self, msg: PixelPoint, publisher):
        if not msg.is_valid or len(msg.x_coordinates) == 0:
            return

        points = []
        
        # Contadores para debug
        total_points = len(msg.x_coordinates)
        corrected_points = 0

        for u, v in zip(msg.x_coordinates, msg.y_coordinates):
            # ¡Ahora usa pixel_to_meters que incluye corrección!
            res = self.pixel_to_meters(u, v)
            if res is None:
                continue

            Xc, Zc = res                
            
            if not (self.min_distance <= Zc <= self.max_distance):
                continue

            corrected_points += 1
            
            # ========== TRANSFORMACIÓN CON YAW ==========
            # Paso 1: Aplicar rotación del yaw de la cámara
            X_rotated = Xc * self.cos_yaw - Zc * self.sin_yaw
            Z_rotated = Xc * self.sin_yaw + Zc * self.cos_yaw
            
            # Paso 2: Aplicar offset de posición
            x_base = Z_rotated + self.cam_x   # Adelante del robot
            y_base = -X_rotated + self.cam_y  # Lateral (negado para ROS: Y+ = izquierda)

            points.append((x_base, y_base, 0.0))

        # Debug opcional
        """if total_points > 0:
            self.get_logger().debug(
                f"Puntos: {corrected_points}/{total_points} corregidos "
                f"({(corrected_points/total_points*100):.1f}%)"
            )"""

        if not points:
            return

        # === Publicar en base_footprint ===
        header = Header()
        header.stamp = msg.header.stamp 
        header.frame_id = "base_footprint"

        cloud_base = point_cloud2.create_cloud_xyz32(header, points)
        publisher.publish(cloud_base)

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = PixelToMeterTransformLeft()
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