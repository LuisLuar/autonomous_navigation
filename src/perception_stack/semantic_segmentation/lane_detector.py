#!/usr/bin/env python3
"""
LaneDetector - Versión simplificada para detección de líneas de carril
======================================================
- Eliminado DBSCAN
- Detección simple basada en agrupación por Y
- Enfocado en líneas horizontales (paralelas a eje X)
- Líneas separadas por offset en Y
"""

import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque
import math
from typing import List, Dict

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, Float32, Bool
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Importar mensajes personalizados
from custom_interfaces.msg import LaneLines, LaneLine  # Ajusta el nombre de tu paquete

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        
        # =================== PARÁMETROS SIMPLIFICADOS ===================
        self.declare_parameters('', [
            ('x_range_min', 1.50),
            ('x_range_max', 5.0),
            ('max_lateral_distance', 5.0),
            
            # Parámetros simplificados para detección de líneas
            ('y_bin_size', 0.4),           # Tamaño de bin en Y para agrupar puntos
            ('min_points_per_line', 10),    # Mínimo de puntos para considerar una línea
            ('min_line_length', 1.0),       # Longitud mínima de línea en X
            ('max_line_width', 0.1),        # Ancho máximo permitido (std en Y)
            
            # Parámetros para líneas de carril
            ('expected_y_positions', [4.2, 1.4, -1.4, -4.2]),  # Posiciones Y esperadas de líneas (derecha, izquierda)
            ('y_position_tolerance', 0.25),  # Tolerancia para posiciones Y esperadas
            ('lane_width', 2.8),            # Ancho de carril esperado
            
            # Parámetros de ángulo
            ('max_angle_deviation', 15.0),  # Máxima desviación angular en grados (líneas horizontales)
            
            # Parámetros de visualización
            ('visualize_candidates', True),
            ('line_thickness', 0.08),
            ('publish_rate', 10.0),
            ('debug', True),
        ])
        
        # Detección de curvas simplificada - Asegurar que es bool puro
        self.is_in_curve = False
        
        # =================== ROS ===================
        # Suscriptor al pointcloud fusionado
        self.create_subscription(
            PointCloud2, 
            '/lane/merged_candidates',
            self.cb_merged_points, 
            10
        )
        
        # =================== PUBLICADORES ===================
        self.pub_lines = self.create_publisher(
            LaneLines, 
            '/lane/detected_lines', 
            10
        )
        
        self.pub_markers = self.create_publisher(
            MarkerArray, 
            '/lane/debug/candidates', 
            10
        )
        
        self.pub_filtered_points = self.create_publisher(
            PointCloud2,
            '/lane/debug/filtered_points',
            10
        )
        
        self.pub_lines_count = self.create_publisher(Float32, '/lane/debug/line_count', 10)
        self.pub_is_curve = self.create_publisher(Bool, '/lane/debug/is_curve', 10)
        
        # Timer para publicación periódica
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self.publish_debug_info
        )
        
        ##self.get_logger().info(" LaneDetector simplificado inicializado")
        #self.get_logger().info(" Enfoque en líneas horizontales (paralelas a eje X)")
    
    # =================== PROCESAMIENTO PRINCIPAL ===================
    def cb_merged_points(self, msg):
        pts = list(point_cloud2.read_points(msg, ('x','y','z'), skip_nans=True))
        if len(pts) < 5:
            return
        
        # 1. FILTRADO DE PUNTOS
        filtered = self.filter_points(pts)
        
        if len(filtered) > 0:
            self.publish_filtered_points(filtered)
        
        if len(filtered) < 5:
            return
        
        # 2. DETECCIÓN SIMPLIFICADA DE LÍNEAS
        lines_base = self.detect_lines_simple(filtered)
        
        # 3. DETECCIÓN DE CURVAS SIMPLIFICADA
        if lines_base:
            # Asegurar que es bool puro
            self.is_in_curve = bool(self.detect_curve_simple(lines_base))
        
        # 4. PUBLICAR MENSAJE PERSONALIZADO
        if lines_base:
            self.publish_lines_msg(lines_base, msg.header)
        
        # 5. PUBLICAR INFORMACIÓN BÁSICA
        self.publish_line_count(len(lines_base))
        self.publish_curve_status()
        
        # 6. VISUALIZACIÓN EN RVIZ
        if self.get_parameter('visualize_candidates').value and lines_base:
            self.visualize_lines_as_markers(lines_base)
        
        # 7. LOGS DE DEBUG
        if self.get_parameter('debug').value and len(lines_base) > 0:
            self.log_detection_info(lines_base)
    
    # =================== MÉTODOS SIMPLIFICADOS ===================
    
    def filter_points(self, pts):
        """Filtra puntos en base_footprint"""
        filtered = []
        x_min = self.get_parameter('x_range_min').value
        x_max = self.get_parameter('x_range_max').value
        y_max = self.get_parameter('max_lateral_distance').value
        
        for x, y, z in pts:
            if x_min <= x <= x_max and abs(y) <= y_max:
                filtered.append((x, y, z))
        
        return filtered
    
    def detect_lines_simple(self, pts):
        """Detección simplificada basada en agrupación por Y"""
        if len(pts) < self.get_parameter('min_points_per_line').value:
            return []
        
        points_array = np.array(pts)
        
        # 1. Agrupar puntos por su posición Y (líneas horizontales)
        y_bin_size = self.get_parameter('y_bin_size').value
        y_min, y_max = -self.get_parameter('max_lateral_distance').value, self.get_parameter('max_lateral_distance').value
        n_bins = int((y_max - y_min) / y_bin_size)
        
        lines = []
        line_id = 0
        
        # Crear bins en Y
        for i in range(n_bins):
            y_bin_start = y_min + i * y_bin_size
            y_bin_end = y_bin_start + y_bin_size
            
            # Filtrar puntos en este bin de Y
            mask = (points_array[:, 1] >= y_bin_start) & (points_array[:, 1] < y_bin_end)
            bin_points = points_array[mask]
            
            if len(bin_points) < self.get_parameter('min_points_per_line').value:
                continue
            
            # 2. Verificar que sea una línea (cobertura en X)
            x_coverage = bin_points[:, 0].max() - bin_points[:, 0].min()
            if x_coverage < self.get_parameter('min_line_length').value:
                continue
            
            # 3. Verificar que sea estrecha (pequeña desviación en Y)
            y_std = np.std(bin_points[:, 1])
            if y_std > self.get_parameter('max_line_width').value:
                continue
            
            # 4. Ajustar línea (horizontal por definición)
            mean_y = np.mean(bin_points[:, 1])
            mean_z = np.mean(bin_points[:, 2]) if len(bin_points[0]) > 2 else 0.0
            
            # Calcular ángulo (casi horizontal)
            angle = self.calculate_line_angle(bin_points)
            angle_deg = abs(math.degrees(angle)) % 180
            
            # 5. Filtrar por ángulo (debe ser casi horizontal)
            if angle_deg > self.get_parameter('max_angle_deviation').value and (180 - angle_deg) > self.get_parameter('max_angle_deviation').value:
                continue
            
            # 6. Calcular calidad simplificada
            quality = self.calculate_simple_quality(bin_points, mean_y, angle, x_coverage, y_std)
            
            # 7. Priorizar líneas en posiciones Y esperadas (líneas de carril)
            expected_positions = self.get_parameter('expected_y_positions').value
            y_tolerance = self.get_parameter('y_position_tolerance').value
            
            is_expected_position = False
            for expected_y in expected_positions:
                if abs(mean_y - expected_y) <= y_tolerance:
                    is_expected_position = True
                    quality *= 1.2  # Bonus por estar en posición esperada
                    break
            
            # 8. Crear datos de línea
            lines.append({
                'id': line_id,
                'points': bin_points.tolist(),
                'mean_rel_y': float(mean_y),
                'length': float(x_coverage),
                'angle': float(angle),
                'perp_std': float(y_std),
                'quality': float(quality),
                'n_points': int(len(bin_points)),
                'x_mean': float(np.mean(bin_points[:, 0])),
                'y_mean': float(mean_y),
                'z_mean': float(mean_z),
                'is_expected_position': bool(is_expected_position),  # Asegurar bool
            })
            
            line_id += 1
        
        # 9. Ordenar líneas por calidad y por posición Y
        lines.sort(key=lambda x: (-x['quality'], abs(x['mean_rel_y'])))
        
        # 10. Limitar a las mejores líneas (máximo 4)
        if len(lines) > 4:
            lines = lines[:4]
        
        return lines
    
    def calculate_line_angle(self, points):
        """Calcula ángulo de la línea usando regresión lineal simple"""
        if len(points) < 2:
            return 0.0
        
        x = points[:, 0]
        y = points[:, 1]
        
        # Regresión lineal
        A = np.vstack([x, np.ones(len(x))]).T
        try:
            slope, intercept = np.linalg.lstsq(A, y, rcond=None)[0]
        except:
            return 0.0
        
        # Ángulo de la línea
        angle = math.atan(slope)
        
        # Para líneas de carril, el ángulo debe estar cerca de 0 (horizontal)
        # Normalizar a [-π/2, π/2]
        if angle > math.pi/2:
            angle -= math.pi
        elif angle < -math.pi/2:
            angle += math.pi
        
        return angle
    
    def calculate_simple_quality(self, points, mean_y, angle, x_coverage, y_std):
        """Calidad simplificada basada en varios factores"""
        
        # Factor 1: Número de puntos
        n_points = len(points)
        points_factor = min(n_points / 50.0, 1.0)  # Normalizar a [0, 1]
        
        # Factor 2: Cobertura en X
        length_factor = min(x_coverage / 3.0, 1.0)
        
        # Factor 3: Rectitud (baja desviación en Y)
        straightness_factor = 1.0 - min(y_std / 0.2, 1.0)  # Máximo 0.2m de desviación
        
        # Factor 4: Horizontalidad
        angle_deg = abs(math.degrees(angle))
        horizontal_factor = 1.0 - min(angle_deg / 90.0, 1.0)
        
        # Ponderación
        quality = (
            points_factor * 0.25 +
            length_factor * 0.30 +
            straightness_factor * 0.30 +
            horizontal_factor * 0.15
        )
        
        return max(0.0, min(1.0, quality))
    
    def detect_curve_simple(self, lines):
        """Detección simplificada de curva - Retorna bool explícito"""
        if len(lines) < 2:
            return False
        
        # Para detección simple de curva, verificamos si las líneas
        # se están desviando mucho de la horizontal
        angles = []
        for line in lines:
            angle_deg = abs(math.degrees(line['angle']))
            if angle_deg > 90:
                angle_deg = 180 - angle_deg
            angles.append(angle_deg)
        
        # Si el ángulo promedio es > 10 grados, posible curva
        avg_angle = np.mean(angles)
        # Retornar bool explícito
        return bool(avg_angle > 10.0)
    
    # =================== MÉTODOS DE PUBLICACIÓN ===================
    
    def publish_lines_msg(self, lines_base, input_header):
        """Publica líneas en mensaje personalizado para Nodo 2"""
        msg = LaneLines()
        
        msg.header = input_header
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Asegurar que es bool puro
        msg.is_in_curve = bool(self.is_in_curve)
        msg.frame_id = 0  # Simplificado
        
        for line in lines_base:
            lane_line = LaneLine()
            
            lane_line.points = []
            for point in line['points']:
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = float(point[2]) if len(point) > 2 else 0.0
                lane_line.points.append(p)
            
            # Asignar valores
            lane_line.mean_rel_y = float(line['mean_rel_y'])
            lane_line.quality = float(line['quality'])
            lane_line.angle = float(line['angle'])
            lane_line.length = float(line['length'])
            lane_line.perp_std = float(line['perp_std'])
            lane_line.n_points = int(line['n_points'])
            lane_line.x_mean = float(line['x_mean'])
            
            msg.lines.append(lane_line)
        
        self.pub_lines.publish(msg)
        
        """if self.get_parameter('debug').value:
            self.get_logger().debug(
                f" Publicadas {len(lines_base)} líneas en /lane/detected_lines"
            )"""
    
    def visualize_lines_as_markers(self, lines):
        """Visualización simplificada en RViz"""
        marker_array = MarkerArray()
        
        for i, line in enumerate(lines):
            line_marker = Marker()
            line_marker.header.frame_id = "base_footprint"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "lane_lines"
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            line_marker.points = []
            points_array = np.array(line['points'])
            
            # Ordenar por X para visualización continua
            sorted_indices = np.argsort(points_array[:, 0])
            sorted_points = points_array[sorted_indices]
            
            for point in sorted_points:
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = 0.0
                line_marker.points.append(p)
            
            line_marker.scale.x = self.get_parameter('line_thickness').value
            
            # Color según posición Y
            if line.get('is_expected_position', False):
                # Línea en posición esperada - Verde
                line_marker.color.r = 0.0
                line_marker.color.g = 1.0
                line_marker.color.b = 0.0
            else:
                # Otra línea - Amarillo
                line_marker.color.r = 1.0
                line_marker.color.g = 1.0
                line_marker.color.b = 0.0
            
            line_marker.color.a = 0.8
            
            line_marker.pose.orientation.w = 1.0
            line_marker.lifetime.sec = 1
            
            marker_array.markers.append(line_marker)
        
        self.pub_markers.publish(marker_array)
    
    def publish_filtered_points(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_footprint"
        
        points_xyz = [(p[0], p[1], p[2]) for p in points]
        cloud = point_cloud2.create_cloud_xyz32(header, points_xyz)
        self.pub_filtered_points.publish(cloud)
    
    def publish_line_count(self, count):
        msg = Float32()
        msg.data = float(count)
        self.pub_lines_count.publish(msg)
    
    def publish_curve_status(self):
        msg = Bool()
        # Asegurar que es bool puro
        msg.data = bool(self.is_in_curve)
        self.pub_is_curve.publish(msg)
    
    def publish_debug_info(self):
        pass
        """self.get_logger().debug(
            f" LaneDetector activo - "
            f"Curva: {'Sí' if self.is_in_curve else 'No'}"
        )"""
    
    def log_detection_info(self, lines):
        if not self.get_parameter('debug').value:
            return
        
        """self.get_logger().info(
            f" Detectadas {len(lines)} líneas:"
        )"""
        
        for i, line in enumerate(lines):
            angle_deg = math.degrees(line['angle'])
            if angle_deg > 90:
                angle_deg -= 180
            
            pos_type = "ESPERADA" if line.get('is_expected_position', False) else "OTRA"
            
            """self.get_logger().info(
                f"  L{i} ({pos_type}): "
                f"Y={line['mean_rel_y']:.2f}m, "
                f"Q={line['quality']:.2f}, "
                f"∠={angle_deg:.1f}°, "
                f"L={line['length']:.1f}m, "
                f"P={line['n_points']}"
            )"""

def main():
    rclpy.init()
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(" LaneDetector detenido")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()