#!/usr/bin/env python3
"""
LanePreprocessorNode - Multi-Línea
=================================
Procesa 4 líneas de carril simultáneamente:
1. left_border    (borde izquierdo)
2. lane_dividing  (línea divisoria)  
3. right_lane     (carril derecho)
4. right_border   (borde derecho)

Para cada línea:
- Filtrado temporal (max_age específico por línea)
- Filtrado espacial (voxel específico por línea)
- Extracción de línea central (slices)
- Publicación: PointCloud2 (control) + Marker (debug) con colores distintos
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
from collections import deque

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header

import math


class LanePreprocessorNode(Node):
    def __init__(self):
        super().__init__('lane_preprocessor_node')

        # ==================== PARÁMETROS GLOBALES ====================
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('timer_period', 0.1)     # segundos (10 Hz)
        
        self.frame_id = self.get_parameter('frame_id').value
        
        # ==================== DEFINICIÓN DE LAS 4 LÍNEAS CON PARÁMETROS INDIVIDUALES ====================
        self.lines_config = {
            'left_border': {
                'input_topic': '/memory/left_border',
                'output_pc2': '/centerline/left_border',
                'max_age': 20.0,      # 30 segundos para borde izquierdo
                'voxel_size': 0.08,   # 8 cm para bordes
                'output_marker': '/debug/left_border',
                'color': (0.0, 1.0, 0.0),      # VERDE
                'marker_id': 0
            },
            'lane_dividing': {
                'input_topic': '/memory/lane_dividing',
                'output_pc2': '/centerline/lane_dividing',
                'max_age': 15.0,      # 15 segundos para divisoria
                'voxel_size': 0.03,   # 3 cm para mayor precisión
                'output_marker': '/debug/lane_dividing',
                'color': (1.0, 1.0, 0.0),      # AMARILLO
                'marker_id': 1
            },
            'right_lane': {
                'input_topic': '/memory/right_lane',
                'output_pc2': '/centerline/right_lane',
                'max_age': 25.0,      # 25 segundos para carril derecho
                'voxel_size': 0.15,   # 5 cm
                'output_marker': '/debug/right_lane',
                'color': (0.0, 0.0, 1.0),      # AZUL
                'marker_id': 2
            },
            'right_border': {
                'input_topic': '/memory/right_border',
                'output_pc2': '/centerline/right_border',
                'max_age': 20.0,      # 30 segundos para borde derecho
                'voxel_size': 0.08,   # 8 cm para bordes
                'output_marker': '/debug/right_border',
                'color': (1.0, 0.0, 0.0),      # ROJO
                'marker_id': 3
            }
        }
        
        # ==================== ESTADO POR LÍNEA ====================
        self.line_memories = {name: deque(maxlen=5000) for name in self.lines_config}
        self.current_centerlines = {name: np.empty((0, 3)) for name in self.lines_config}
        
        # ==================== SUSCRIPTORES POR LÍNEA ====================
        self.line_subscriptions = {}
        for line_name, config in self.lines_config.items():
            self.line_subscriptions[line_name] = self.create_subscription(
                PointCloud2,
                config['input_topic'],
                lambda msg, ln=line_name: self.cloud_callback(msg, ln),
                10
            )
        
        # ==================== PUBLICADORES POR LÍNEA ====================
        self.publishers_pc2 = {}
        self.publishers_marker = {}
        
        for line_name, config in self.lines_config.items():
            self.publishers_pc2[line_name] = self.create_publisher(
                PointCloud2,
                config['output_pc2'],
                10
            )
            
            self.publishers_marker[line_name] = self.create_publisher(
                Marker,
                config['output_marker'],
                10
            )
        
        # ==================== TIMER ÚNICO ====================
        timer_period = self.get_parameter('timer_period').value
        self.create_timer(timer_period, self.timer_callback)
        
        #self.get_logger().info(f"LanePreprocessorNode iniciado con {len(self.lines_config)} líneas")
        
        # Mostrar parámetros específicos por línea
        """for line_name, config in self.lines_config.items():
            self.get_logger().info(
                f"Línea '{line_name}': max_age={config['max_age']}s, "
                f"voxel_size={config['voxel_size']}m"
            )"""

    # ==================== CALLBACKS ====================
    def cloud_callback(self, msg: PointCloud2, line_name: str):
        """Acumula puntos en memoria específica de cada línea"""
        now = time.time()
        memory = self.line_memories[line_name]
        
        try:
            for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                memory.append([p[0], p[1], p[2], now])
        except Exception as e:
            #self.get_logger().warn(f"Error leyendo puntos de {line_name}: {e}")
            pass
    
    # ==================== TIMER CALLBACK ====================
    def timer_callback(self):
        """Procesamiento periódico de todas las líneas"""
        for line_name in self.lines_config.keys():
            self.process_line(line_name)
        
        self.publish_all_results()
    
    # ==================== PROCESAMIENTO POR LÍNEA ====================
    def process_line(self, line_name: str):
        """
        Procesa una línea individual usando sus parámetros específicos:
        - Limpia memoria vieja (con max_age específico)
        - Filtra por voxel (con voxel_size específico)
        - Ordena y genera centerline estable
        """
        memory = self.line_memories[line_name]
        config = self.lines_config[line_name]

        # 0. Verificar memoria mínima
        if len(memory) < 10:
            self.current_centerlines[line_name] = np.empty((0, 3))
            return

        # 1. Eliminar puntos viejos usando max_age específico
        recent_points = self.forget_old_line(memory, config['max_age'])

        if len(recent_points) < 5:
            self.current_centerlines[line_name] = np.empty((0, 3))
            return

        # 2. Filtro voxel usando voxel_size específico
        filtered_cloud = self.voxel_filter_line(recent_points, config['voxel_size'])

        if len(filtered_cloud) < 5:
            self.current_centerlines[line_name] = np.empty((0, 3))
            return

        # 3. Ordenar puntos por eje longitudinal (PCA)
        filtered_cloud = self.order_points_along_principal_axis(filtered_cloud)

        # 4. Crear centerline 2D con Z=0
        centerline_2d = filtered_cloud[:, :2]

        if len(centerline_2d) < 2:
            self.current_centerlines[line_name] = np.empty((0, 3))
            return

        self.current_centerlines[line_name] = np.column_stack([
            centerline_2d[:, 0],
            centerline_2d[:, 1],
            np.zeros(len(centerline_2d))
        ])

    def forget_old_line(self, memory: deque, max_age: float) -> list:
        """Filtra puntos viejos usando max_age específico de la línea"""
        now = time.time()
        recent_points = []

        for x, y, z, t in memory:
            if now - t < max_age:
                recent_points.append([x, y, z])

        return recent_points

    def voxel_filter_line(self, points: list, voxel_size: float) -> np.ndarray:
        """Filtrado por voxel usando voxel_size específico de la línea"""
        voxels = {}
        
        for x, y, z in points:
            if np.isnan(x) or np.isnan(y) or np.isnan(z):
                continue

            key = (
                math.floor(x / voxel_size),
                math.floor(y / voxel_size),
                math.floor(z / voxel_size)
            )

            voxels.setdefault(key, []).append((x, y, z))
        
        filtered_points = []
        for v_pts in voxels.values():
            if len(v_pts) > 0:
                mean_pt = np.mean(v_pts, axis=0)
                filtered_points.append(mean_pt)
        
        return np.array(filtered_points) if filtered_points else np.empty((0, 3))
    
    def order_points_along_principal_axis(self, points: np.ndarray) -> np.ndarray:
        """Ordena puntos 3D según el eje principal (PCA) en XY."""
        if len(points) < 2:
            return points

        pts_xy = points[:, :2]
        mean_xy = np.mean(pts_xy, axis=0)
        centered = pts_xy - mean_xy

        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        principal_axis = eigvecs[:, np.argmax(eigvals)]

        projections = centered @ principal_axis
        order = np.argsort(projections)

        return points[order]

    # ==================== PUBLICACIÓN ====================
    def publish_all_results(self):
        """Publica resultados de todas las líneas procesadas"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        
        for line_name, config in self.lines_config.items():
            centerline = self.current_centerlines[line_name]
            
            if len(centerline) > 0:
                pc2_msg = point_cloud2.create_cloud_xyz32(
                    header,
                    centerline.tolist()
                )
                self.publishers_pc2[line_name].publish(pc2_msg)
            
            self.publish_debug_marker(line_name, header, centerline)
    
    def publish_debug_marker(self, line_name: str, header: Header, centerline: np.ndarray):
        """Publica marcador de debug para una línea específica"""
        if len(centerline) == 0:
            return
        
        config = self.lines_config[line_name]
        marker = Marker()
        
        marker.header = header
        marker.ns = f'centerline_{line_name}'
        marker.id = config['marker_id']
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        
        r, g, b = config['color']
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0
        
        for x, y, z in centerline:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = float(z)
            marker.points.append(p)
        
        self.publishers_marker[line_name].publish(marker)
    
    # ==================== ESTADÍSTICAS ====================
    def get_statistics(self):
        """Retorna estadísticas de procesamiento con parámetros específicos"""
        stats = {}
        for line_name in self.lines_config.keys():
            config = self.lines_config[line_name]
            stats[line_name] = {
                'memory': len(self.line_memories[line_name]),
                'centerline': len(self.current_centerlines[line_name]),
                'max_age': config['max_age'],
                'voxel_size': config['voxel_size']
            }
        return stats

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LanePreprocessorNode()
    
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