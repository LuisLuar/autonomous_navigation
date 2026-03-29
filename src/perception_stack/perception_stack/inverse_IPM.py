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
    def __init__(self):
        super().__init__('ipm_inverse_node')

        # =================== CARGA DE CALIBRACIÓN ===================
        calibration_path = "/home/raynel/autonomous_navigation/src/params/camera_calibration.json"
        try:
            with open(calibration_path, 'r') as f:
                calib = json.load(f)

            self.cam_z = calib["camera_z"]
            self.cam_x = calib["camera_x"]
            self.cam_y = calib["camera_y"]
            self.roll = float(calib['camera_roll'])
            self.fx = float(calib['intrinsics']['fx'])
            self.fy = float(calib['intrinsics']['fy'])
            self.cx = float(calib['intrinsics']['cx'])
            self.cy = float(calib['intrinsics']['cy'])
            
            self.img_width = 640
            self.img_height = 360

        except Exception as e:
            self.get_logger().error(f"Fallo al cargar calibración: {e}")
            rclpy.shutdown()
            return

        # =================== CONSTRUCCIÓN DE HOMOGRAFÍA ===================
        K = np.array([
            [self.fx,  0.0,     self.cx],
            [0.0,      self.fy,     self.cy],
            [0.0,      0.0,     1.0]
        ])
        
        R_align = np.array([
            [0.0, -1.0,  0.0],
            [0.0,  0.0, -1.0],
            [1.0,  0.0,  0.0]
        ])

        cp = math.cos(self.roll)
        sp = math.sin(self.roll)
        R_pitch = np.array([
            [1.0,  0.0,  0.0],
            [0.0,   cp,   sp],
            [0.0,  -sp,   cp]
        ])
        
        rotation_matrix = R_pitch @ R_align
        cam_position = np.array([self.cam_x, self.cam_y, self.cam_z])
        translation = -rotation_matrix @ cam_position

        # Homografía Directa (Mundo Z=0 -> Imagen)
        self.H = K @ np.column_stack((rotation_matrix[:, 0], 
                                      rotation_matrix[:, 1], 
                                      translation))

        # =================== COMUNICACIONES ===================
        self.create_subscription(
            PointCloud2,
            '/lane/debug_fitted_lines', #/lane/centerline_pc
            self.centerline_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.pub_pixel_points = self.create_publisher(
            PixelPoint,
            '/lane/ipm_inverse_pixel_points',
            10
        )

    def centerline_callback(self, msg: PointCloud2):
        # 1. Extraer puntos de forma segura
        # Usamos list() para sacar los datos del generador de ROS
        pts_list = list(point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True))
        
        if not pts_list:
            return

        # --- CORRECCIÓN CRUCIAL ---
        # En lugar de pasar la lista directo a np.array, extraemos los valores
        # Esto elimina cualquier rastro del dtype estructurado ('x', 'y')
        try:
            # Convertimos la lista de tuplas/objetos a una matriz de floats pura
            pts = np.array([ [p[0], p[1]] for p in pts_list ], dtype=np.float64)
        except (IndexError, TypeError):
            # Fallback en caso de que la estructura sea distinta
            return

        # Aseguramos que sea (N, 2)
        if pts.ndim == 1:
            pts = pts.reshape(-1, 2)
        
        # 2. Preparar matriz homogénea (3, N)
        n_points = pts.shape[0]
        input_pts = np.ones((3, n_points))
        input_pts[0, :] = pts[:, 0]  # Coordenada X
        input_pts[1, :] = pts[:, 1]  # Coordenada Y

        # 3. Proyección a Imagen usando la Homografía
        # [u_hom, v_hom, w] = H @ [X, Y, 1]
        projected = self.H @ input_pts

        # 4. Normalización por W (Coordenadas homogéneas a píxeles)
        w = projected[2, :]
        
        # Filtro de seguridad: Solo puntos delante de la cámara (W positivo)
        mask_valid = w > 0.1
        if not np.any(mask_valid):
            return

        u_pixels = projected[0, mask_valid] / w[mask_valid]
        v_pixels = projected[1, mask_valid] / w[mask_valid]

        # 5. Filtrar puntos que caen dentro de la resolución de la imagen
        img_mask = (u_pixels >= 0) & (u_pixels < self.img_width) & \
                   (v_pixels >= 0) & (v_pixels < self.img_height)

        final_u = u_pixels[img_mask].astype(int)
        final_v = v_pixels[img_mask].astype(int)

        # 6. Publicar mensaje PixelPoint
        if final_u.size > 0:
            out_msg = PixelPoint()
            out_msg.header = msg.header
            # ROS2 no acepta numpy.int64, convertimos a int de Python
            out_msg.u = [int(u) for u in final_u]
            out_msg.v = [int(v) for v in final_v]
            self.pub_pixel_points.publish(out_msg)

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