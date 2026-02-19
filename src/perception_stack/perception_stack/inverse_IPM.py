#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import json
import math

from custom_interfaces.msg import PixelPoint
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class IPMInverseNode(Node):
    """Nodo que implementa la transformación inversa de IPM (metros a píxeles)"""

    def __init__(self):
        super().__init__('ipm_inverse_node')

        # =================== CARGA DE CALIBRACIÓN ===================
        calibration_path = "/home/raynel/autonomous_navigation/src/perception_stack/params/camera_calibration.json"
        try:
            with open(calibration_path, 'r') as f:
                calib = json.load(f)

            self.h = float(calib['camera_height'])
            self.pitch = float(calib['camera_pitch'])
            self.fx = float(calib['intrinsics']['fx'])
            self.fy = float(calib['intrinsics']['fy'])
            self.cx = float(calib['intrinsics']['cx'])
            self.cy = float(calib['intrinsics']['cy'])

            #self.get_logger().info(f"Calibración cargada: h={self.h}, pitch={self.pitch}")

        except Exception as e:
            #self.get_logger().error(f"Error cargando calibración: {e}")
            rclpy.shutdown()
            return

        # =================== PARÁMETROS ===================
        self.declare_parameter("camera_offset_x", -0.216)
        self.declare_parameter("camera_offset_y", 0.0)

        self.cam_x = self.get_parameter("camera_offset_x").value
        self.cam_y = self.get_parameter("camera_offset_y").value

        # =================== VARIABLES DE ESTADO ===================
        self.latest_centerline_points = None

        # =================== SUSCRIPTORES ===================
        # Suscripción al topic de PointCloud2 en base_footprint
        self.create_subscription(
            PointCloud2,
            '/lane/debug_fitted_lines',
            self.centerline_callback,
            10
        )

        # =================== PUBLICADORES ===================
        # Para publicar puntos en píxeles
        self.pub_pixel_points = self.create_publisher(
            PixelPoint,
            '/lane/ipm_inverse_pixel_points',
            10
        )

    # =================== CALLBACK DE CENTERLINE ===================
    def centerline_callback(self, msg: PointCloud2):
        """Procesa la nube de puntos del centerline"""
        
        # Extraer puntos de la nube - CORREGIDO
        points_gen = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        points = np.array([[p[0], p[1], p[2]] for p in points_gen], dtype=np.float32)
        
        if points.size == 0:
            return
                
        # Guardar puntos para procesamiento
        self.latest_centerline_points = points
        
        # Procesar transformación
        self.process_transformation()

    # =================== TRANSFORMACIÓN INVERSA IPM CORRECTA ===================
    def meters_to_pixels(self, X_base, Y_base):
        """
        Transforma coordenadas en base_footprint a píxeles en imagen.
        INVERSA MATEMÁTICA EXACTA del IPM original.
        
        Args:
            X_base: Coordenada X en base_footprint (adelante)
            Y_base: Coordenada Y en base_footprint (lateral, izquierda +)
            
        Returns:
            (u, v): Coordenadas en píxeles, o None si no es visible
        """
        try:
            # ============================================
            # PASO 1: base_footprint → coordenadas cámara
            # ============================================
            # Esta transformación debe ser INVERSA de la que usas en IPM original
            
            # En tu IPM original haces:
            # x_base = Zc + self.cam_x  (adelante)
            # y_base = -Xc + self.cam_y (lateral)
            
            # Por lo tanto, la inversa es:
            # Zc = X_base - self.cam_x  (adelante en cámara)
            # Xc = -(Y_base - self.cam_y)  (lateral en cámara, derecha +)
            
            # Pero OJO: en IPM original usas Y_cam = -h para el suelo
            
            Z_cam = X_base - self.cam_x      # Adelante en cámara
            X_cam = -(Y_base - self.cam_y)   # Lateral derecha en cámara
            
            # ============================================
            # PASO 2: Crear punto en suelo (Y = -h en frame cámara)
            # ============================================
            # Esto es CRÍTICO: el punto debe estar en el PLANO del suelo
            # Y_cam = -self.h (porque cámara mira hacia adelante/abajo)
            
            Y_cam = -self.h
            
            # ============================================
            # PASO 3: Rotación inversa por pitch
            # ============================================
            # En IPM original usas:
            # R = [[1,  0,   0],
            #      [0,  cp,  sp],
            #      [0, -sp,  cp]]
            #
            # Por lo tanto, la inversa es la transpuesta:
            
            cp = math.cos(self.pitch)
            sp = math.sin(self.pitch)
            
            # Matriz inversa (transpuesta)
            R_inv = np.array([
                [1.0,  0.0,   0.0],
                [0.0,   cp,   -sp],
                [0.0,   sp,    cp]
            ])
            
            # Punto en frame cámara sin rotación pitch
            point_cam = np.array([X_cam, Y_cam, Z_cam])
            
            # Aplicar rotación inversa
            # Esto lleva el punto del frame cámara (con pitch) al frame sin pitch
            point_no_pitch = R_inv @ point_cam
            
            # ============================================
            # PASO 4: Proyección perspectiva
            # ============================================
            # Ahora proyectamos como una cámara normal
            
            if point_no_pitch[2] <= 0:
                # Detrás de la cámara
                return None
            
            # Coordenadas normalizadas
            x_norm = point_no_pitch[0] / point_no_pitch[2]  # X/Z
            y_norm = point_no_pitch[1] / point_no_pitch[2]  # Y/Z
            
            # ============================================
            # PASO 5: A píxeles
            # ============================================
            # IMPORTANTE: El signo negativo en y_norm depende de tu convención
            # En tu IPM original tienes: y = -(v - cy) / fy
            # Esto significa que v aumenta hacia ABAJO en imagen
            
            u = self.fx * x_norm + self.cx
            v = -self.fy * y_norm + self.cy  # Negativo porque Y de imagen va hacia abajo
            
            # ============================================
            # PASO 6: Convertir a enteros y validar
            # ============================================
            u_int = int(round(u))
            v_int = int(round(v))
            
            # Validación básica: píxeles no pueden ser negativos
            if u_int < 0 or v_int < 0:
                return None
            
            return (u_int, v_int)
                
        except Exception as e:
            #self.get_logger().warn(f"Error en transformación inversa IPM: {e}")
            return None

    # =================== PROCESAMIENTO PRINCIPAL ===================
    def process_transformation(self):
        """Procesa todos los puntos del centerline"""
        
        if self.latest_centerline_points is None:
            return
            
        pixel_u = []
        pixel_v = []
        
        # Procesar cada punto
        for point in self.latest_centerline_points:
            X_base, Y_base, _ = point[0], point[1], point[2]
            
            # Aplicar transformación inversa IPM
            pixel = self.meters_to_pixels(X_base, Y_base)
            
            if pixel is not None:
                u, v = pixel
                # Ya están validados en meters_to_pixels
                pixel_u.append(u)
                pixel_v.append(v)
        
        # Publicar puntos en píxeles
        if pixel_u:
            pixel_msg = PixelPoint()
            pixel_msg.header.stamp = self.get_clock().now().to_msg()
            pixel_msg.header.frame_id = "camera"
            pixel_msg.is_valid = True
            
            # IMPORTANTE: Usar int puro de Python
            pixel_msg.x_coordinates = pixel_u  # Ya son int
            pixel_msg.y_coordinates = pixel_v  # Ya son int
            
            self.pub_pixel_points.publish(pixel_msg)

def main():
    rclpy.init()
    node = IPMInverseNode()
    
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