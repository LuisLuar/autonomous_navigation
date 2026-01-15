#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import math
import json

from sensor_msgs.msg import CameraInfo
from custom_interfaces.msg import SegmentationData
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class IPMNode(Node):
    def __init__(self):
        super().__init__('ipm_node')
        
        # ===== CARGAR CALIBRACIÓN =====
        calibration_path = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration.json"
        try:
            with open(calibration_path, 'r') as f:
                calib = json.load(f)
            
            # PARÁMETROS CALIBRADOS EXACTAMENTE
            self.h = float(calib['camera_height'])        # 0.38 m
            self.pitch = float(calib['camera_pitch'])     # 0.1312 rad (7.52°)
            self.fx = float(calib['intrinsics']['fx'])    # 574.1
            self.fy = float(calib['intrinsics']['fy'])    # 574.1
            self.cx = float(calib['intrinsics']['cx'])    # 320.0
            self.cy = float(calib['intrinsics']['cy'])    # 240.0
            
            #self.get_logger().info("=" * 60)
            #self.get_logger().info("✅ IPM CON CALIBRACIÓN EXACTA")
            #self.get_logger().info(f"  Pitch: {self.pitch:.4f} rad ({math.degrees(self.pitch):.2f}°)")
            #self.get_logger().info(f"  Altura: {self.h} m")
            #self.get_logger().info(f"  fx: {self.fx:.1f}, fy: {self.fy:.1f}")
            #self.get_logger().info(f"  cx: {self.cx:.1f}, cy: {self.cy:.1f}")
            #self.get_logger().info("=" * 60)
            
        except Exception as e:
            #self.get_logger().error(f"❌ Error cargando calibración: {e}")
            rclpy.shutdown()
            return

        # ===== PARÁMETROS BEV (ajustables) =====
        self.declare_parameter("roi_ratio", 0.45)      # ROI más arriba
        self.declare_parameter("bev_scale", 40.0)      # px/m
        self.declare_parameter("max_distance", 12.0)    # m
        
        self.roi_ratio = self.get_parameter("roi_ratio").value
        self.scale = self.get_parameter("bev_scale").value
        self.max_distance = self.get_parameter("max_distance").value

        # ===== BEV DIMENSIONES =====
        self.bev_w = 400
        self.bev_h = int(self.max_distance * self.scale) + 100
        
        #self.get_logger().info(f"BEV: {self.bev_w}x{self.bev_h}, Escala: {self.scale} px/m")
        #self.get_logger().info(f"ROI ratio: {self.roi_ratio}, Distancia máxima: {self.max_distance}m")

        # ===== ROS =====
        self.bridge = CvBridge()
        self.create_subscription(
            CameraInfo,
            "/camera/rgb/camera_info",
            self.camera_info_cb,
            1
        )
        self.create_subscription(
            SegmentationData,
            "/segmentation/data",
            self.seg_cb,
            1
        )
        self.bev_pub = self.create_publisher(Image, "/ipm/bev", 1)
        self.pc_pub = self.create_publisher(PointCloud2, "/ipm/points", 1)

        
        # Bandera de ready (aunque ya tenemos intrínsecos)
        self.ready = True

    def camera_info_cb(self, msg):
        """Verificar que coincidan los intrínsecos"""
        if abs(self.fx - msg.k[0]) > 1.0 or abs(self.fy - msg.k[4]) > 1.0:
            pass
            #self.get_logger().warn(f"Intrínsecos ROS diferentes a calibración:")
            #self.get_logger().warn(f"  Calib: fx={self.fx:.1f}, ROS: fx={msg.k[0]:.1f}")
            #self.get_logger().warn(f"  Calib: fy={self.fy:.1f}, ROS: fy={msg.k[4]:.1f}")

    def seg_cb(self, msg):
        """Transformación IPM con parámetros calibrados"""
        H, W = msg.height, msg.width
        seg = np.array(msg.mask_data, dtype=np.uint8).reshape(H, W)

        bev = np.zeros((self.bev_h, self.bev_w), dtype=np.uint8)
        v0 = int(H * self.roi_ratio)

        # Pre-calcular valores
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)
        
        # MATRIZ CORRECTA para pitch POSITIVO (7.52°)
        R = np.array([
            [1,  0,   0],
            [0,  cp,  sp],     # sp sin signo negativo
            [0, -sp,  cp]      # -sp aquí
        ])

        # Estadísticas
        points_mapped = 0
        distances = []
        points = []
        
        # Procesar cada 2 píxeles para velocidad
        for v in range(v0, H, 2):
            for u in range(0, W, 2):
                cls = seg[v, u]
                if cls == 0:
                    continue

                # Rayo en coordenadas cámara
                x = (u - self.cx) / self.fx
                y = -(v - self.cy) / self.fy  # Y negativo porque crece hacia abajo
                ray = np.array([x, y, 1.0])

                # Rotar según pitch
                ray_w = R @ ray

                # CONDICIÓN: rayo debe apuntar hacia el suelo (Y negativo)
                # Con pitch positivo 7.52°, esto debería funcionar
                if ray_w[1] >= 0:
                    continue

                # Distancia al suelo
                t = self.h / -ray_w[1]
                X = ray_w[0] * t      # Lateral (derecha/izquierda)
                Z = ray_w[2] * t      # Adelante

                # Filtrar por distancia
                if Z <= 0 or Z > self.max_distance:
                    continue
                
                distances.append(Z)
                points.append([Z, -X, 0.0])          

                # Mapear a BEV
                bx = int(self.bev_w / 2 + X * self.scale)
                by = int(self.bev_h - Z * self.scale)

                if 0 <= bx < self.bev_w and 0 <= by < self.bev_h:
                    points_mapped += 1
                    if cls == 2:      # Líneas de carril
                        bev[by, bx] = 255
                    elif cls == 1:    # Área transitable
                        bev[by, bx] = 180
                    else:             # Otros
                        bev[by, bx] = 100



        # Log informativo
        seg_pixels = np.count_nonzero(seg[v0:, :])
        if points_mapped > 0 and distances:
            avg_dist = np.mean(distances)
            #self.get_logger().info(f"✅ BEV: {points_mapped} pts | Dist media: {avg_dist:.1f}m | ROI seg: {seg_pixels}")
        #else:
            #self.get_logger().warn(f"❌ BEV: 0 pts | ROI seg: {seg_pixels}")

        # Publicar BEV
        if points_mapped > 0:
            bev_msg = self.bridge.cv2_to_imgmsg(bev, encoding="mono8")
            bev_msg.header.stamp = self.get_clock().now().to_msg()
            bev_msg.header.frame_id = "base_footprint"
            self.bev_pub.publish(bev_msg)
        
        if points:
            header = msg.header
            header.frame_id = "base_footprint"

            pc_msg = point_cloud2.create_cloud_xyz32(header, points)
            self.pc_pub.publish(pc_msg)


        # ===== VISUALIZACIÓN =====
        #self.visualize_results(seg, bev, H, W, v0, points_mapped, distances)

    def visualize_results(self, seg, bev, H, W, v0, points_mapped, distances):
        """Visualización mejorada"""
        # 1. Segmentación con ROI
        seg_vis = cv2.cvtColor(seg * 120, cv2.COLOR_GRAY2BGR)
        cv2.line(seg_vis, (0, v0), (W, v0), (0, 255, 0), 2)
        
        # Info en segmentación
        pitch_deg = math.degrees(self.pitch)
        cv2.putText(seg_vis, f"Pitch: {pitch_deg:.1f}°", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(seg_vis, f"ROI: y > {v0}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(seg_vis, f"BEV pts: {points_mapped}", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Dibujar línea del horizonte teórico
        v_horizon = self.cy + self.fy * math.tan(self.pitch)
        if 0 <= v_horizon < H:
            cv2.line(seg_vis, (0, int(v_horizon)), (W, int(v_horizon)),
                    (0, 255, 255), 1, cv2.LINE_AA)

        # 2. BEV
        bev_vis = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
        
        # Eje del robot
        cv2.line(bev_vis,
                 (self.bev_w // 2, self.bev_h),
                 (self.bev_w // 2, 0),
                 (0, 255, 0), 1)
        
        # Delantera del robot
        cv2.line(bev_vis,
                 (self.bev_w // 2 - 15, self.bev_h),
                 (self.bev_w // 2 + 15, self.bev_h),
                 (0, 0, 255), 2)

        # Rejilla métrica (cada metro)
        max_z = int(self.max_distance)
        for z in range(1, max_z + 1):
            y = int(self.bev_h - z * self.scale)
            if y >= 0:
                color = (60, 60, 60) if z % 2 == 0 else (40, 40, 40)
                cv2.line(bev_vis, (0, y), (self.bev_w, y), color, 1)
                if z % 2 == 0:
                    cv2.putText(bev_vis, f"{z}m", (5, y - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

        # Información en BEV
        cv2.putText(bev_vis, f"Pitch: {pitch_deg:.1f}°", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if distances:
            min_dist, max_dist = np.min(distances), np.max(distances)
            cv2.putText(bev_vis, f"Dist: {min_dist:.1f}-{max_dist:.1f}m", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        cv2.putText(bev_vis, f"Scale: {self.scale} px/m", (10, 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        # Mostrar ventanas
        cv2.imshow("1. Segmentation", seg_vis)
        cv2.imshow(f"2. IPM BEV (max {max_z}m)", bev_vis)
        
        # Controles
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            #self.get_logger().info("ESC pressed - Shutting down")
            cv2.destroyAllWindows()
            rclpy.shutdown()
        elif key == ord('+'):
            self.scale = min(self.scale * 1.1, 100.0)
            #self.get_logger().info(f"Escala aumentada a: {self.scale:.1f} px/m")
        elif key == ord('-'):
            self.scale = max(self.scale * 0.9, 20.0)
            #self.get_logger().info(f"Escala reducida a: {self.scale:.1f} px/m")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = IPMNode()
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

if __name__ == "__main__":
    main()