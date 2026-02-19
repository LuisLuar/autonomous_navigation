#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import struct
import colorsys
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from custom_interfaces.msg import LaneModel

class QuadraticLaneEstimator(Node):
    def __init__(self):
        super().__init__('quadratic_lane_estimator')
        
        # Parámetros
        self.expected_width = 3.0      
        self.min_points = 15           
        self.cluster_tol = 0.4         # Aumentado un poco para curvas
        self.max_yaw_angle = 0.8       # Límite de orientación inicial

        # Suscripción
        self.sub = self.create_subscription(PointCloud2, '/lane/meter_candidates', self.cb_points, 10)
        
        # Publicadores
        self.pub_model = self.create_publisher(LaneModel, '/lane/model_raw', 10)
        self.pub_debug_clusters = self.create_publisher(PointCloud2, '/lane/debug_clusters', 10)
        self.pub_debug_lines = self.create_publisher(PointCloud2, '/lane/debug_fitted_lines', 10)

        self.get_logger().info("Estimador Parabólico (Grado 2) Iniciado")

    def create_pc2_msg(self, header, points_rgb):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        return point_cloud2.create_cloud(header, fields, points_rgb)

    def get_rgb_uint32(self, r, g, b):
        return struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]

    def cb_points(self, msg):
        pts = np.array([(p[0], p[1]) for p in point_cloud2.read_points(msg, field_names=['x', 'y'], skip_nans=True)])
        if len(pts) < self.min_points: return

        # 1. Agrupamiento por Bins
        y_bins = np.round(pts[:, 1] / self.cluster_tol)
        unique_bins = np.unique(y_bins)
        
        candidates = []
        cluster_cloud_pts = []
        
        for i, b in enumerate(unique_bins):
            cluster_mask = (y_bins == b)
            cluster_pts = pts[cluster_mask]
            if len(cluster_pts) < self.min_points: continue
            
            # Color para debug
            hue = (i * 0.618) % 1.0
            rgb = colorsys.hsv_to_rgb(hue, 0.8, 1.0)
            color_int = self.get_rgb_uint32(int(rgb[0]*255), int(rgb[1]*255), int(rgb[2]*255))
            
            for p in cluster_pts:
                cluster_cloud_pts.append([float(p[0]), float(p[1]), 0.0, color_int])

            # 2. AJUSTE DE GRADO 2 (y = c2*x^2 + c1*x + c0)
            # c2: curvatura, c1: yaw inicial, c0: offset lateral
            coeffs = np.polyfit(cluster_pts[:, 0], cluster_pts[:, 1], 2)
            c2, c1, c0 = coeffs
            
            # Filtro de sensatez: El ángulo en el origen (x=0) no debe ser extremo
            if abs(np.arctan(c1)) < self.max_yaw_angle:
                candidates.append({'c0': c0, 'c1': c1, 'c2': c2, 'pts': cluster_pts})

        if cluster_cloud_pts:
            self.pub_debug_clusters.publish(self.create_pc2_msg(msg.header, cluster_cloud_pts))

        if not candidates: return

        # 3. Clasificación Izquierda / Derecha (por el intercepto c0)
        l_cands = [c for c in candidates if c['c0'] > 0]
        r_cands = [c for c in candidates if c['c0'] < 0]
        best_l = min(l_cands, key=lambda x: x['c0']) if l_cands else None
        best_r = max(r_cands, key=lambda x: x['c0']) if r_cands else None

        # 4. Construcción del Mensaje y Debug Visual
        out = LaneModel()
        out.header = msg.header
        line_debug_pts = []
        c_left = self.get_rgb_uint32(0, 255, 0)
        c_right = self.get_rgb_uint32(0, 0, 255)
        c_center = self.get_rgb_uint32(255, 0, 0)

        def add_curve_to_debug(c0, c1, c2, color):
            # Dibujamos una curva suave de 0 a 12 metros
            for x in np.linspace(0, 12, 30):
                y = c2*(x**2) + c1*x + c0
                line_debug_pts.append([float(x), float(y), 0.05, color])

        if best_l and best_r:
            out.lane_width = float(abs(best_l['c0'] - best_r['c0']))
            out.d_lat = float((best_l['c0'] + best_r['c0']) / 2.0)
            # Promediamos el ángulo local y la curvatura para la línea central
            avg_yaw = (best_l['c1'] + best_r['c1']) / 2.0
            avg_curv = (best_l['c2'] + best_r['c2']) / 2.0
            out.curvature = float(2.0 * avg_curv)
            out.yaw = float(np.arctan(avg_yaw))
            out.confidence = 1.0
            
            add_curve_to_debug(best_l['c0'], best_l['c1'], best_l['c2'], c_left)
            add_curve_to_debug(best_r['c0'], best_r['c1'], best_r['c2'], c_right)
            # Línea central
            add_curve_to_debug(out.d_lat, avg_yaw, avg_curv, c_center)
            
        elif best_l or best_r:
            ref = best_l if best_l else best_r
            sign = -1 if best_l else 1
            out.lane_width = self.expected_width
            out.d_lat = float(ref['c0'] + sign * (self.expected_width / 2.0))
            out.yaw = float(np.arctan(ref['c1']))
            out.curvature = float(2.0 * ref['c2']) 
            out.confidence = 0.5
            
            color = c_left if best_l else c_right
            add_curve_to_debug(ref['c0'], ref['c1'], ref['c2'], color)
            # Línea central asumiendo misma curvatura que la única línea vista
            add_curve_to_debug(out.d_lat, ref['c1'], ref['c2'], c_center)

        if out.confidence > 0:
            self.pub_model.publish(out)
            self.pub_debug_lines.publish(self.create_pc2_msg(msg.header, line_debug_pts))

def main(args=None):
    rclpy.init(args=args)
    node = QuadraticLaneEstimator()

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