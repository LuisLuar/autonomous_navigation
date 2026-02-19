#!/usr/bin/env python3
"""
LANE EDGE ESTIMATOR WITH IPM TRANSFORMATION - ADAPTIVE
=========================================================
Parámetros adaptativos:
1. Base ratio: calcula el punto más arriba con lane/road mask
2. Range ratio: se ajusta para evitar la parte inferior donde road mask ocupa todo
3. Cálculo del ángulo del borde del carril
4. Cálculo de errores para control PD
5. Publicacion de puntos de borde en odom para control geoemtrico 
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import json
import math
from collections import deque

from custom_interfaces.msg import SegmentationData
from std_msgs.msg import Float32MultiArray, Header, Float32
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


class LaneEdgeIPMNodeAdaptive(Node):
    def __init__(self):
        super().__init__('lane_edge_ipm_adaptive')
        
        # =================== PARÁMETROS IPM ===================
        calibration_path = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration_front.json"
        try:
            with open(calibration_path, 'r') as f:
                calib = json.load(f)
            
            # Parámetros calibrados
            self.h = float(calib['camera_height'])        
            self.pitch = float(calib['camera_pitch'])     
            self.fx = float(calib['intrinsics']['fx'])    
            self.fy = float(calib['intrinsics']['fy'])    
            self.cx = float(calib['intrinsics']['cx'])    
            self.cy = float(calib['intrinsics']['cy'])    
            
        except Exception as e:
            #self.get_logger().error(f"Error cargando calibración: {e}")
            rclpy.shutdown()
            return

        # =================== PARÁMETROS ADAPTATIVOS INICIALES ===================
        # PARA MÉTODO ADAPTATIVO (lane + road)
        self.base_ratio_adaptive = 0.65  # Valor inicial
        self.range_ratio_adaptive = 0.2  # Valor inicial
        self.roi_ratio_adaptive = 0.6   # ROI para método adaptativo - comienza en 72.5%
        
        # PARA MÉTODO LANE-ONLY (solo lane)
        self.base_ratio_laneonly = 0.7   # Valor inicial
        self.range_ratio_laneonly = 0.2  # Valor inicial
        self.roi_ratio_laneonly = 0.75

        # Parámetros BEV
        self.declare_parameter("max_distance", 12.0)
        self.declare_parameter("min_distance", 0.5)
        
        self.max_distance = self.get_parameter("max_distance").value
        self.min_distance = self.get_parameter("min_distance").value

        # =================== NUEVO: PARÁMETROS PARA NORMALIZACIÓN ===================
        self.declare_parameter('max_angle_range', 170.0)  # Grados máximo para normalización

        self.max_angle_range = self.get_parameter('max_angle_range').value
        self.filtered_error = 0.0  # Inicialización del error filtrado  

        # =================== FILTRADO TEMPORAL ===================
        self.declare_parameter('ema_alpha', 0.002)  # Suavizado exponencial
        
        # =================== PARÁMETROS LANE DETECTION ===================
        self.declare_parameter('num_horizons', 5)
        self.declare_parameter('min_points_per_horizon', 5)
        self.declare_parameter('use_lane_mask', True)
        self.declare_parameter('use_road_mask', True)
        self.declare_parameter('density_road_mask', 0.8)
        
        # =================== PARÁMETROS POLYNOMIAL FIT ===================
        self.declare_parameter('poly_degree', 2)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('show_debug', True)
        
        # =================== NUEVO: PARÁMETRO PARA ÁNGULO ===================
        self.declare_parameter('lookahead_distance', 3.0)  # Distancia para calcular ángulo
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        # =================== NUEVO: PARÁMETROS PARA ERRORES PD ===================
        self.declare_parameter('max_angle_error', 3.0)  # Ángulo máximo en grados antes de generar error
        self.max_angle_error = self.get_parameter('max_angle_error').value
        
        # =================== ESTADO ===================
        self.mask = None
        self.bridge = CvBridge()
        self.angle_error = 0.0
        
        # Historial para suavizado adaptativo
        self.base_ratio_adaptive_history = deque(maxlen=10)
        self.range_ratio_adaptive_history = deque(maxlen=10)
        self.base_ratio_laneonly_history = deque(maxlen=10)
        self.range_ratio_laneonly_history = deque(maxlen=10)


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lane_map_points = deque(maxlen=5000)
        self.lane_mask_map_points = deque(maxlen=5000) 

        # =================== NUEVO: ESTADO DE CALIDAD ===================
        self.lane_quality = 0.0
        self.lane_valid = False

        # Freeze last good lane
        self.last_good_coeffs = None
        self.last_good_points = None
        self.last_good_time = None
        
        # =================== NUEVO: HISTORIAL PARA ÁNGULO ===================
        self.angle_history = deque(maxlen=5)
        
        # =================== NUEVO: HISTORIAL PARA ERROR DE ÁNGULO ===================
        self.angle_error_history = deque(maxlen=5)
        
        # =================== ROS SETUP ===================
        # Suscripciones
        self.create_subscription(SegmentationData, '/segmentation/data', self.seg_cb, 1)
        
        # Publicadores
        self.pub_poly_coeffs = self.create_publisher(Float32MultiArray, '/lane/polynomial_coeffs', 10)
        self.pub_lane_points = self.create_publisher(PointCloud2, '/lane/edge_points', 10)
        self.pub_lane_mask_points = self.create_publisher(PointCloud2, '/lane/mask_points', 10)  
        self.pub_debug = self.create_publisher(Image, '/lane_debug', 10)
        self.pub_lane_path = self.create_publisher(Path, '/lane/edge_path', 10)
        
        # ===================  PUBLICADOR PARA ERROR DE ÁNGULO (Float32) ===================
        self.pub_angle_error = self.create_publisher(Float32, '/omega/lane_orientation', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.get_parameter('publish_rate').value,self.timer_cb)

    def _update_lane_map_points(self, points_meters, use_rightmost=True):
        """
        Actualiza los puntos del mapa principal.
        """
        if use_rightmost:
            # Filtrar para mantener solo los puntos más a la derecha
            if len(points_meters) > 0:
                x_values = points_meters[:, 1]
                rightmost_threshold = np.percentile(x_values, 30)  # Tomar el 30% más a la derecha
                mask = x_values <= rightmost_threshold
                points_meters = points_meters[mask]
        
        points_odom = self.transform_points_to_odom(points_meters)
        for p in points_odom:
            self.lane_map_points.append(p)

    def _update_lane_mask_points(self, points_meters):
        """
        Actualiza los puntos del lane mask.
        """
        points_odom = self.transform_points_to_odom(points_meters)
        for p in points_odom:
            self.lane_mask_map_points.append(p)

    def _publish_point_clouds(self):
        """
        Publica ambos PointClouds.
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "odom"
        
        # Publicar puntos del borde principal
        if len(self.lane_map_points) > 0:
            pc_msg = point_cloud2.create_cloud_xyz32(header, list(self.lane_map_points))
            self.pub_lane_points.publish(pc_msg)
        
        # Publicar puntos del lane mask
        if len(self.lane_mask_map_points) > 0:
            pc_mask_msg = point_cloud2.create_cloud_xyz32(header, list(self.lane_mask_map_points))
            self.pub_lane_mask_points.publish(pc_mask_msg)
    
    def _fit_lane_mask_polynomial(self, lane_mask_points):
        """
        Ajusta un polinomio a los puntos del lane mask.
        """
        if len(lane_mask_points) < 3:
            return None
        
        try:
            degree = self.get_parameter('poly_degree').value
            coeffs = np.polyfit(lane_mask_points[:, 0], lane_mask_points[:, 1], degree)
            return coeffs
        except:
            return None
    
    def _compute_lane_mask_quality(self, lane_mask_points):
        """
        Evalúa la calidad del lane mask basado en:
        - Cantidad de puntos
        - Consistencia espacial
        - Rango de distancias
        """
        if len(lane_mask_points) < 10:
            return 0.0
        
        # Calcular dispersión de puntos
        if len(lane_mask_points) > 1:
            z_values = lane_mask_points[:, 0]
            x_values = lane_mask_points[:, 1]
            
            # Rango de distancias (debe cubrir un buen rango)
            z_range = np.max(z_values) - np.min(z_values)
            z_range_score = min(z_range / 5.0, 1.0)  # Normalizar a máximo 5 metros
            
            # Dispersión lateral (no debe ser muy disperso)
            x_std = np.std(x_values)
            x_std_score = max(0, 1.0 - (x_std / 1.0))  # Penalizar si std > 1m
            
            # Cantidad de puntos
            point_count_score = min(len(lane_mask_points) / 50.0, 1.0)
            
            # Calidad total (ponderada)
            quality = 0.4 * z_range_score + 0.3 * x_std_score + 0.3 * point_count_score
            return quality
        return 0.0

    def _compare_edges(self, adaptive_points, lane_mask_points):
        """
        Compara los puntos detectados por los dos métodos.
        Retorna: 
        - same_edge: True si los bordes son similares
        - rightmost_edge_points: puntos del borde más a la derecha
        - other_points: puntos del otro bordes
        - lane_mask_quality: calidad del lane mask
        """
        # Convertir a arrays numpy si no lo están
        adaptive_array = np.array(adaptive_points) if len(adaptive_points) > 0 else np.array([])
        lane_mask_array = np.array(lane_mask_points) if len(lane_mask_points) > 0 else np.array([])
        
        # Calcular calidad del lane mask
        lane_mask_quality = self._compute_lane_mask_quality(lane_mask_array)
        
        if len(adaptive_array) == 0 and len(lane_mask_array) == 0:
            return False, None, None, lane_mask_quality
        
        if len(adaptive_array) == 0:
            return False, lane_mask_array, None, lane_mask_quality
        
        if len(lane_mask_array) == 0:
            return False, adaptive_array, None, lane_mask_quality
        
        # Calcular el promedio lateral de cada conjunto de puntos
        adaptive_x_avg = np.mean(adaptive_array[:, 1]) if len(adaptive_array) > 0 else 0
        lane_mask_x_avg = np.mean(lane_mask_array[:, 1]) if len(lane_mask_array) > 0 else 0
        
        # Diferencia entre bordes
        diff = abs(adaptive_x_avg - lane_mask_x_avg)
        
        # Si la diferencia es pequeña (< 0.5m), considerar que es el mismo borde
        same_edge = diff < 0.5
        
        # Determinar cuál es más a la derecha (X más negativo en coordenadas del robot)
        if adaptive_x_avg < lane_mask_x_avg:  # adaptive está más a la derecha
            rightmost_points = adaptive_array
            other_points = lane_mask_array
        else:
            rightmost_points = lane_mask_array
            other_points = adaptive_array
        
        return same_edge, rightmost_points, other_points, lane_mask_quality
    
    def _extract_lane_mask_points(self, mask):
        """
        Extrae puntos directamente de la lane mask (valor 2) y los transforma a metros.
        """
        if mask is None:
            return []
        
        h, w = mask.shape
        
        # Buscar puntos de lane mask en el lado derecho
        roi_start_y = int(h * self.roi_ratio_adaptive)
        
        # Crear máscara para lane mask en el área de interés
        lane_mask_area = mask[roi_start_y:, w//2:w]
        
        # Encontrar todos los píxeles de lane mask
        ys, xs = np.where(lane_mask_area == 2)
        
        if len(ys) < 10:  # Mínimo de puntos
            return []
        
        # Muestrear puntos (no todos para no saturar)
        step = max(1, len(ys) // 20)  # Tomar aproximadamente 20 puntos
        sampled_indices = range(0, len(ys), step)
        
        points_meters = []
        
        for idx in sampled_indices:
            y_local = ys[idx]
            x_local = xs[idx]
            
            # Convertir a coordenadas globales de imagen
            y_global = y_local + roi_start_y
            x_global = x_local + w // 2
            
            # Transformar a metros
            result = self._pixel_to_meters(x_global, y_global)
            if result is not None:
                X, Z = result
                if self.min_distance <= Z <= self.max_distance:
                    points_meters.append((Z, X))
        
        return np.array(points_meters)

    def transform_points_to_odom(self, points_meters):
        points_odom = []

        try:
            tf = self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time()
            )
        except Exception as e:
            return points_odom

        for Z, X in points_meters:
            p = PointStamped()
            p.header.frame_id = 'base_footprint'
            p.header.stamp = self.get_clock().now().to_msg()

            p.point.x = Z
            p.point.y = -X
            p.point.z = 0.0

            p_odom = do_transform_point(p, tf)
            points_odom.append((
                p_odom.point.x,
                p_odom.point.y,
                p_odom.point.z
            ))

        return points_odom

    # =================== NUEVO: VALIDACIÓN DE CALIDAD DEL BORDE ===================

    def check_perspective_consistency(self, horizon_data, min_dx_px=6):
        """
        Un borde real debe tener inclinación en la imagen (no vertical).
        Evita falsas detecciones cuando road mask ocupa todo.
        """
        if len(horizon_data) < 3:
            return False

        xs = [h['x'] for h in horizon_data]
        ys = [h['y'] for h in horizon_data]

        dx = max(xs) - min(xs)
        dy = max(ys) - min(ys)

        if dy < 20:  # Muy poco rango vertical
            return False

        return dx >= min_dx_px


    def check_monotonicity(self, horizon_data):
        """
        El borde debe desplazarse de forma consistente (no zig-zag).
        """
        if len(horizon_data) < 3:
            return False

        xs = np.array([h['x'] for h in horizon_data])
        diffs = np.diff(xs)

        return np.all(diffs >= -3) or np.all(diffs <= 3)


    def compute_global_confidence(self, horizon_data):
        """
        Confianza promedio de todos los horizontes.
        """
        if not horizon_data:
            return 0.0

        return float(np.mean([h['confidence'] for h in horizon_data]))


    def compute_lane_quality(self, horizon_data):
        """
        Score final de calidad [0.0 – 1.0]
        """
        quality = 0.0

        if self.check_perspective_consistency(horizon_data):
            quality += 0.4

        if self.check_monotonicity(horizon_data):
            quality += 0.3

        quality += 0.3 * self.compute_global_confidence(horizon_data)

        return min(1.0, quality)

    # =================== MÉTODO PARA CALCULAR ERROR DE ÁNGULO ===================
    def calculate_angle_error(self, coeffs):
        """
        Calcula el error de ángulo para el controlador PD normalizado entre -1 y 1.
        Solo genera error si el ángulo excede los límites.
        """
        if coeffs is None or len(coeffs) < 3:
            return 0.0
        
        # 1. CALCULAR ÁNGULO ACTUAL
        angle_deg = self.calculate_lane_angle(coeffs, self.lookahead_distance)
        
        # Inicializar error de ángulo en 0
        angle_error = 0.0
        
        # Solo generar error si el ángulo excede los límites
        if abs(angle_deg) > self.max_angle_error:
            # Error con signo contrario al ángulo
            # Ángulo negativo (gira derecha) → error positivo
            # Ángulo positivo (gira izquierda) → error negativo
            angle_error = -angle_deg

        alpha = self.get_parameter('ema_alpha').value
        self.filtered_error = (
            alpha * angle_error + 
            (1 - alpha) * self.filtered_error
        )
        
        # Normalización
        angle_error_normalized = np.clip(self.filtered_error / self.max_angle_range, -1.0, 1.0)
        
        # Suavizar error con historial
        self.angle_error_history.append(angle_error_normalized)
        
        if len(self.angle_error_history) > 0:
            angle_error_normalized = np.mean(self.angle_error_history)
        
        return angle_error_normalized
    
    # =================== NUEVO: MÉTODO PARA CALCULAR ÁNGULO ===================
    def calculate_lane_angle(self, coeffs, lookahead_distance=3.0):
        """
        Calcula el ángulo que debe girar el robot para seguir el borde del carril.
        
        Args:
            coeffs: Coeficientes del polinomio [a, b, c] para X = a*Z² + b*Z + c
            lookahead_distance: Distancia adelante donde evaluar (metros)
            
        Returns:
            angle_deg: Ángulo en grados (positivo = gira izquierda, negativo = gira derecha)
        """
        if coeffs is None or len(coeffs) < 3:
            return 0.0
        
        # Calcular pendiente del polinomio en la distancia lookahead
        a, b, c = coeffs[0], coeffs[1], coeffs[2]
        slope = 2 * a * lookahead_distance + b
        
        # Ángulo = dirección opuesta a la pendiente del borde
        # slope > 0: borde va a derecha → robot gira izquierda (ángulo positivo)
        # slope < 0: borde va a izquierda → robot gira derecha (ángulo negativo)
        angle_rad = -math.atan(slope)
        angle_deg = math.degrees(angle_rad)
        
        # Suavizado con historial
        self.angle_history.append(angle_deg)
        if len(self.angle_history) > 0:
            smoothed_angle = np.mean(self.angle_history)
        else:
            smoothed_angle = angle_deg
            
        return smoothed_angle
    
    # =================== CÁLCULO DE PARÁMETROS ADAPTATIVOS ===================
    def _calculate_adaptive_ratios(self, mask):
        """
        Calcula ratios PARA MÉTODO ADAPTATIVO (lane + road) - SIN LÍMITES ESTRICTOS
        """
        if mask is None:
            return self.base_ratio_adaptive, self.range_ratio_adaptive
        
        h, w = mask.shape
        roi_start = int(h * self.roi_ratio_adaptive)
        
        # ================= BASE_RATIO_ADAPTIVE =================
        # Buscar la PRIMERA fila válida hacia arriba (lane O road)
        search_region = mask[roi_start:, w//2:w]
        
        lane_rows, _ = np.where(search_region == 2)
        road_rows, _ = np.where(search_region == 1)
        
        min_lane = np.min(lane_rows) if len(lane_rows) > 0 else h
        min_road = np.min(road_rows) if len(road_rows) > 0 else h
        min_row = min(min_lane, min_road)
        
        if min_row < (h - roi_start):
            new_base = (roi_start + min_row) / h
            
            # QUITAR LÍMITE DE 0.85 - Permitir buscar más arriba
            # Solo asegurar que esté dentro de [roi_ratio + 0.02, 0.95]
            new_base = np.clip(new_base, self.roi_ratio_adaptive + 0.02, 0.95)  # Cambiado 0.85 → 0.95
            
            self.base_ratio_adaptive_history.append(new_base)
            self.base_ratio_adaptive = np.mean(self.base_ratio_adaptive_history)
        else:
            self.base_ratio_adaptive = self.roi_ratio_adaptive + 0.1
        
        # ================= RANGE_RATIO_ADAPTIVE =================
        # Buscar HASTA DONDE ROAD DOMINA (>70%)
        problem_row_ratio = 0.95  # Valor por defecto alto (95%)
        
        for row in range(roi_start, h, 10):
            row_slice = mask[row:min(row+10, h), w//2:w]  # Evitar overflow
            if row_slice.size > 0:
                density = np.sum(row_slice == 1) / row_slice.size
                if density > 0.05:  # Road ocupa > 70%
                    problem_row_ratio = row / h
                    break
        
        margin = 0.08
        max_allowed = max(0.1, problem_row_ratio - self.base_ratio_adaptive - margin)
        
        last_range = self.range_ratio_adaptive
        max_step = 0.03  # Un poco más rápido
        
        new_range = np.clip(
            max_allowed,
            last_range - max_step,
            last_range + max_step
        )
        # PERMITIR RANGOS MÁS GRANDES
        new_range = np.clip(new_range, 0.05, 0.5)  # Cambiado 0.4 → 0.5
        
        self.range_ratio_adaptive_history.append(new_range)
        self.range_ratio_adaptive = np.mean(self.range_ratio_adaptive_history)
        
        return self.base_ratio_adaptive, self.range_ratio_adaptive



    def _calculate_lane_only_ratios_right(self, mask):
        """
        Calcula ratios PARA MÉTODO LANE-ONLY - COMPLETAMENTE INDEPENDIENTE
        """
        if mask is None:
            return None, None
        
        h, w = mask.shape
        roi_start = int(h * self.roi_ratio_laneonly)
        
        # NO LIMITAR POR ADAPTATIVE - buscar en toda la imagen desde ROI hacia arriba
        # Buscar lane mask en TODO el lado derecho desde ROI hacia arriba
        search_region = mask[roi_start:, w//2:w]
        lane_rows, _ = np.where(search_region == 2)
        
        if len(lane_rows) < 5:  # Menos estricto que 10
            return None, None
        
        # lane_rows está relativo al ROI
        min_lane_row = np.min(lane_rows)
        max_lane_row = np.max(lane_rows)
        
        base_ratio = (roi_start + min_lane_row) / h
        top_ratio = (roi_start + max_lane_row) / h
        
        # Protección con límites MÁS FLEXIBLES
        base_ratio = np.clip(base_ratio, self.roi_ratio_laneonly, 0.9)  # Desde ROI
        top_ratio = np.clip(top_ratio, base_ratio + 0.05, 0.95)  # Hasta 95%
        
        range_ratio = top_ratio - base_ratio
        
        # Si el rango es muy pequeño, expandirlo
        if range_ratio < 0.1:
            range_ratio = 0.15
        
        # Guardar en historial lane-only
        self.base_ratio_laneonly_history.append(base_ratio)
        self.range_ratio_laneonly_history.append(range_ratio)
        
        self.base_ratio_laneonly = np.mean(self.base_ratio_laneonly_history)
        self.range_ratio_laneonly = np.mean(self.range_ratio_laneonly_history)
        
        return self.base_ratio_laneonly, self.range_ratio_laneonly


    # =================== CALLBACKS ROS ===================    
    def seg_cb(self, msg: SegmentationData):
        """Recibe la máscara de segmentación."""
        try:
            self.mask = np.frombuffer(
                msg.mask_data, np.uint8
            ).reshape(msg.height, msg.width)
        except Exception as e:
            #self.get_logger().error(f"Error reshaping mask: {e}")
            self.mask = None
    
    # =================== EXTRACCIÓN DE PUNTOS ADAPTATIVA ===================
    def _extract_horizon_points_adaptive(self, mask):
        """Extrae puntos usando PARÁMETROS ADAPTATIVOS - SIN LÍMITE SUPERIOR ESTRICTO"""
        if mask is None:
            return []
        
        # 1. Calcular ratios adaptativos
        self._calculate_adaptive_ratios(mask)
        
        h, w = mask.shape
        num_horizons = self.get_parameter('num_horizons').value
        
        horizon_data = []
        
        # Usar parámetros ADAPTATIVOS
        roi_start_y = int(h * self.roi_ratio_adaptive)
        
        # SIN LÍMITE SUPERIOR ESTRICTO - buscar hasta donde sea necesario
        # Usar un límite razonable basado en los parámetros
        max_search_y = int(h * min(0.95, self.base_ratio_adaptive + self.range_ratio_adaptive * 1.5))
        
        for i in range(num_horizons):
            ratio = i / max(1, num_horizons - 1)
            
            # Usar base_ratio_adaptive y range_ratio_adaptive
            y_center = int(h * (self.base_ratio_adaptive + ratio * self.range_ratio_adaptive))
            
            # VERIFICACIÓN MÁS FLEXIBLE
            if y_center < roi_start_y:
                continue  # Aún debe estar dentro del ROI
            
            # Franja vertical dinámica
            slice_height = int(h * 0.06)
            y1 = max(roi_start_y, y_center - slice_height // 2)
            y2 = min(h - 10, y_center + slice_height // 2)  # h-10 para no salirse
            
            # Solo lado derecho (donde está el carril)
            slice_mask = mask[y1:y2, w//2:w]
            
            # Extraer borde
            edge_x, confidence = self._extract_edge_robust(slice_mask)
            
            if edge_x is not None:
                global_x = edge_x + w // 2
                
                horizon_data.append({
                    'horizon_idx': i,
                    'y': y_center,
                    'x': global_x,
                    'confidence': confidence,
                    'slice_bounds': (y1, y2),
                    'distance_ratio': ratio,
                })
        
        return horizon_data
    
    def _extract_lane_only_points_right(self, mask, num_points=5):
        """
        Extrae puntos usando SOLO lane_mask - SIN LIMITARSE POR ADAPTIVE
        """
        base, range_ = self._calculate_lane_only_ratios_right(mask)
        
        if base is None or range_ is None:
            return []
        
        h, w = mask.shape
        roi_start_y = int(h * self.roi_ratio_laneonly)
        
        # NO LIMITAR POR ADAPTIVE - buscar donde lane esté
        # Límite superior: donde termina lane o 95% de la imagen
        max_search_y = int(h * 0.95)  # 95% de la imagen
        
        points = []
        
        for i in range(num_points):
            ratio = i / max(1, num_points - 1)
            
            # Usar base_ratio_laneonly y range_ratio_laneonly
            y_center = int(h * (self.base_ratio_laneonly + ratio * self.range_ratio_laneonly))
            
            # Verificar límites razonables
            if y_center < roi_start_y or y_center >= h - 10:
                continue

            slice_h = max(6, int(h * 0.02))  # 2% de altura
            y1 = max(roi_start_y, y_center - slice_h // 2)
            y2 = min(h - 10, y_center + slice_h // 2)

            slice_mask = mask[y1:y2, w//2:w]
            ys, xs = np.where(slice_mask == 2)

            if len(xs) < 3:  # Más flexible que 5
                continue

            # borde MÁS A LA IZQUIERDA dentro de la mitad derecha
            edge_x_local = int(np.median(xs))
            edge_x = edge_x_local + w // 2

            points.append({
                'x': edge_x,
                'y': y_center
            })

        return points


    def _extract_edge_robust(self, slice_mask):
        """Extrae el borde del carril - SIEMPRE el más a la DERECHA."""
        min_points = self.get_parameter('min_points_per_horizon').value
        
        # Combinar lane (2) y road (1) - buscar TODOS los puntos válidos
        valid_pixels = (slice_mask == 1) | (slice_mask == 2)
        ys, xs = np.where(valid_pixels)
        
        if len(xs) < min_points:
            return None, 0.0
        
        # Filtrar outliers
        x_low = np.percentile(xs, 5)
        x_high = np.percentile(xs, 95)
        
        valid_mask = (xs >= x_low) & (xs <= x_high)
        filtered_xs = xs[valid_mask]
        
        if len(filtered_xs) < min_points:
            return None, 0.0
        
        # Tomar percentil ALTO para asegurar borde derecho
        if len(filtered_xs) > 100:
            percentile = 97
        elif len(filtered_xs) > 50:
            percentile = 95
        else:
            percentile = 93
        
        edge_x = int(np.percentile(filtered_xs, percentile))
        
        # Confianza
        total_pixels = slice_mask.shape[0] * slice_mask.shape[1]
        density = len(filtered_xs) / total_pixels
        
        confidence = min(1.0, density * 12)
        
        return edge_x, confidence
    
    # =================== TRANSFORMACIÓN IPM ===================
    def _pixel_to_meters(self, u, v):
        """Transforma un punto (u, v) de píxeles a coordenadas métricas."""
        # Rayo en coordenadas cámara
        x = (u - self.cx) / self.fx
        y = -(v - self.cy) / self.fy
        ray = np.array([x, y, 1.0])
        
        # Matriz de rotación para pitch
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)
        
        R = np.array([
            [1,  0,   0],
            [0,  cp,  sp],
            [0, -sp,  cp]
        ])
        
        # Rotar según pitch
        ray_w = R @ ray
        
        # Condición: rayo debe apuntar hacia el suelo
        if ray_w[1] >= 0:
            return None
        
        # Distancia al suelo
        t = self.h / -ray_w[1]
        X_camera = ray_w[0] * t
        Z_camera = ray_w[2] * t
        
        return X_camera, Z_camera
    
    def _transform_points_to_meters(self, horizon_data):
        """Transforma todos los puntos detectados a coordenadas métricas."""
        points_meters = []
        
        for h_data in horizon_data:
            u = h_data['x']
            v = h_data['y']
            
            result = self._pixel_to_meters(u, v)
            if result is not None:
                X, Z = result
                
                # Filtrar por distancia mínima y máxima
                if self.min_distance <= Z <= self.max_distance:
                    points_meters.append((Z, X))
        
        return np.array(points_meters)
    
    # =================== AJUSTE POLINÓMICO ===================
    def _fit_polynomial(self, points_meters):
        """Ajusta un polinomio de grado 2 a los puntos en metros."""
        if len(points_meters) < 3:
            return None
        
        Z = points_meters[:, 0]  # Distancia adelante
        X = points_meters[:, 1]  # Posición lateral
        
        try:
            degree = self.get_parameter('poly_degree').value
            coeffs = np.polyfit(Z, X, degree)
            return coeffs
        except:
            return None
    
    # =================== CREACIÓN DE PATHS ===================
    def _create_polynomial_path(self, coeffs, offset=0.0):
        """
        Crea un Path desde los coeficientes del polinomio con un offset.
        
        Args:
            coeffs: Coeficientes del polinomio [a, b, c]
            offset: Desplazamiento lateral en metros (positivo = izquierda)
        """
        if coeffs is None or len(coeffs) < 3:
            return None
        
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "base_footprint"
        
        # Generar puntos a lo largo del camino
        for z in np.arange(0.5, 5.0, 0.1):  # De 0.5m a 5m cada 10cm
            # Posición del borde en este punto Z
            x_borde = np.polyval(coeffs, z)
            
            # Aplicar offset
            x_position = x_borde - offset  # offset positivo = hacia la izquierda
            
            # Crear PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            
            # En base_footprint:
            # X = adelante (Z del polinomio)
            # Y = izquierda (-X del polinomio porque Y apunta a izquierda)
            pose_stamped.pose.position.x = float(z)
            pose_stamped.pose.position.y = float(-x_position)
            pose_stamped.pose.position.z = 0.0
            
            # Orientación: tangente al camino
            dx_dz = 2 * coeffs[0] * z + coeffs[1]
            yaw = math.atan2(dx_dz, 1.0)
            
            # Quaternion simple para yaw
            pose_stamped.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0)
            )
            
            path_msg.poses.append(pose_stamped)
        
        return path_msg
    
    # =================== VISUALIZACIÓN ===================
    def _create_debug_image(self, mask, horizon_data, points_meters, coeffs,
                       lane_mask_points, lane_mask_coeffs, coeffs_for_angle, angle_source):
        """Crea imagen de debug con información adaptativa."""
        if mask is None:
            return None
        
        h, w = mask.shape
        
        # Crear imagen base
        img = np.full((h, w, 3), 225, dtype=np.uint8)
        
        # Colorear máscara de segmentación
        lane_pixels = (mask == 2)
        road_pixels = (mask == 1)
        img[lane_pixels] = [0, 100, 200]  # Azul para lane
        img[road_pixels] = [50, 50, 50]   # Gris para road
        
        # ===== DIBUJAR ROI (solo línea) =====
        # ROI adaptativo (línea amarilla)
        roi_line_y_adaptive = int(h * self.roi_ratio_adaptive)
        cv2.line(img, (0, roi_line_y_adaptive), (w, roi_line_y_adaptive), 
                (0, 255, 255), 2, cv2.LINE_AA)  # Amarillo
        cv2.putText(img, f"ROI-ADAPT", (10, roi_line_y_adaptive - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # ROI lane-only (línea magenta)
        roi_line_y_laneonly = int(h * self.roi_ratio_laneonly)
        cv2.line(img, (0, roi_line_y_laneonly), (w, roi_line_y_laneonly), 
                (255, 0, 255), 2, cv2.LINE_AA)  # Magenta
        cv2.putText(img, f"ROI-LANE", (10, roi_line_y_laneonly - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        # ===== DIBUJAR PUNTOS DETECTADOS =====
        colors = [(0, 255, 255), (0, 255, 128), (0, 255, 0), 
                (128, 255, 0), (255, 128, 0)]
        
        for i, h_data in enumerate(horizon_data):
            color = colors[min(i, len(colors)-1)]
            confidence = h_data.get('confidence', 1.0)
            y1, y2 = h_data.get('slice_bounds', (0, 0))
            
            # Dibujar franja de búsqueda (solo contorno)
            cv2.rectangle(img, (w//2, y1), (w, y2), 
                        (color[0]//3, color[1]//3, color[2]//3), 1)
            
            # Punto detectado
            radius = int(4 + confidence * 4)
            cv2.circle(img, (h_data['x'], h_data['y']), radius, color, -1)
            cv2.circle(img, (h_data['x'], h_data['y']), radius+2, (255, 255, 255), 1)
            
            # Label
            text = f"H{i}"
            cv2.putText(img, text, (h_data['x'] + 10, h_data['y'] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
        
        # ===== INFORMACIÓN TEXTUAL =====
         # ===== MOSTRAR PARÁMETROS DE AMBOS MÉTODOS =====
        info_y = 30
        spacing = 22
        
        # Título
        cv2.putText(img, "ADAPTIVE LANE DETECTION - DUAL PARAMS", (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        info_y += spacing
        
        # Parámetros ADAPTATIVOS (lane+road)
        adaptive_text = f"ADAPTIVE: Base={self.base_ratio_adaptive:.3f} Range={self.range_ratio_adaptive:.3f}"
        cv2.putText(img, adaptive_text, (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)  # Amarillo
        info_y += spacing
        
        # Parámetros LANE-ONLY
        laneonly_text = f"LANE-ONLY: Base={self.base_ratio_laneonly:.3f} Range={self.range_ratio_laneonly:.3f}"
        cv2.putText(img, laneonly_text, (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)  # Magenta
        info_y += spacing
        
          # ROI adaptativo
        roi_adaptive_text = f"ROI-ADAPT: {self.roi_ratio_adaptive:.3f}"
        cv2.putText(img, roi_adaptive_text, (20, info_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        info_y += spacing
        
        # ROI lane-only
        roi_laneonly_text = f"ROI-LANE: {self.roi_ratio_laneonly:.3f}"
        cv2.putText(img, roi_laneonly_text, (20, info_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        info_y += spacing
    
        
        # Puntos válidos
        points_text = f"Points: {len(points_meters)}/{len(horizon_data)}"
        point_color = (0, 255, 0) if len(points_meters) >= 3 else (0, 0, 255)
        cv2.putText(img, points_text, (20, info_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, point_color, 1)
        info_y += spacing
        
        # Coeficientes (si existen)
        if coeffs is not None:
            coeff_text = f"Poly: {coeffs[0]:.4f}z² + {coeffs[1]:.4f}z + {coeffs[2]:.4f}"
            cv2.putText(img, coeff_text, (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 0), 1)
            
            # ===== NUEVO: MOSTRAR ÁNGULO =====
            info_y += spacing
            angle_deg = self.calculate_lane_angle(coeffs, self.lookahead_distance)
            angle_text = f"Angle: {angle_deg:+.1f}° @ {self.lookahead_distance}m"
            angle_color = (0, 255, 0) if abs(angle_deg) < self.max_angle_error else (0, 165, 255)
            cv2.putText(img, angle_text, (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, angle_color, 1)
            
            # ===== NUEVO: MOSTRAR ERROR DE ÁNGULO =====
            info_y += spacing
            angle_error = self.angle_error
            angle_error_text = f"Angle Error: {angle_error:+.3f}"
            
            cv2.putText(img, angle_error_text, (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 1)
            
            # Indicador de dirección
            info_y += spacing
            if abs(angle_deg) > self.max_angle_error:
                if angle_deg > 0:
                    direction_text = "CORRECT LEFT"
                    direction_color = (0, 165, 255)
                else:
                    direction_text = "CORRECT RIGHT"
                    direction_color = (0, 100, 255)
            else:
                direction_text = "STRAIGHT"
                direction_color = (0, 255, 0)
            
            cv2.putText(img, direction_text, (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, direction_color, 2)
        
        # ===== DISTANCIAS EN METROS =====
        if len(points_meters) > 0:
            for i, (Z, X) in enumerate(points_meters[:3]):  # Mostrar primeros 3
                if i < len(horizon_data):
                    point_y = horizon_data[i]['y']
                    dist_text = f"Z={Z:.1f}m, X={X:.2f}m"
                    cv2.putText(img, dist_text, (w - 200, point_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # ===== NUEVO: DIBUJAR FLECHA DE DIRECCIÓN =====
        if coeffs is not None:
            angle_deg = self.calculate_lane_angle(coeffs, self.lookahead_distance)
            angle_rad = math.radians(angle_deg)
            
            center_x = w // 2
            bottom_y = h - 20
            line_length = 100
            
            end_x = center_x - int(line_length * math.sin(angle_rad))
            end_y = bottom_y - int(line_length * math.cos(angle_rad))
            
            cv2.arrowedLine(img, (center_x, bottom_y), (end_x, end_y),
                           (0, 255, 0), 3, cv2.LINE_AA, tipLength=0.3)
            
            arrow_text = f"{abs(angle_deg):.1f}°"
            text_size = cv2.getTextSize(arrow_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            cv2.putText(img, arrow_text, (end_x - text_size[0]//2, end_y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # ===== DEBUG: LANE ONLY LEFT =====
        lane_only_points = self._extract_lane_only_points_right(mask)

        for p in lane_only_points:
            cv2.circle(img, (p['x'], p['y']), 6, (255, 0, 0), -1)
            cv2.circle(img, (p['x'], p['y']), 8, (255, 255, 255), 1)

        if lane_only_points:
            cv2.putText(img, "LANE-ONLY LEFT (DEBUG)", (20, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        
        return img
    
    # =================== PUBLICACIÓN ===================  
    def _publish_results(self, coeffs_for_angle, points_meters, debug_img):
        """Publica todos los resultados usando los coeficientes priorizados."""
            
        # Coeficientes del polinomio (publicar los del borde principal)
        if coeffs_for_angle is not None:
            coeffs_msg = Float32MultiArray()
            coeffs_msg.data = coeffs_for_angle.tolist()
            self.pub_poly_coeffs.publish(coeffs_msg)

            # ===== PUBLICAR ERROR DE ÁNGULO (Float32) USANDO COEFICIENTES PRIORIZADOS =====
            angle_error = self.calculate_angle_error(coeffs_for_angle)
            self.angle_error = angle_error  # Guardar para acceso externo si es necesario
            angle_error_msg = Float32()
            angle_error_msg.data = float(angle_error)
            self.pub_angle_error.publish(angle_error_msg)
        
        # Paths (borde y robot) - usar borde principal
        if coeffs_for_angle is not None:
            # Path del borde del carril (offset = 0)
            lane_path = self._create_polynomial_path(coeffs_for_angle, offset=0.0)
            if lane_path is not None:
                self.pub_lane_path.publish(lane_path)
        
        # Imagen de debug (opcional)
        # if debug_img is not None:
        #     self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
        
    # =================== MAIN LOOP ===================
    def timer_cb(self):
        """Loop principal."""
        if self.mask is None:
            return
        
        # 1. Extraer puntos con parámetros adaptativos
        horizon_data = self._extract_horizon_points_adaptive(self.mask)
        
        # 2. Extraer puntos directamente del lane mask
        lane_mask_points = self._extract_lane_mask_points(self.mask)
        
        # ===== CALIDAD DEL CARRIL =====
        self.lane_quality = self.compute_lane_quality(horizon_data)
        self.lane_valid = self.lane_quality > 0.6

        adaptive_points = np.array([])
        adaptive_coeffs = None
        lane_mask_coeffs = None
        
        if self.lane_valid and len(horizon_data) >= 3:
            adaptive_points = self._transform_points_to_meters(horizon_data)
            
            if len(adaptive_points) >= 3:
                adaptive_coeffs = self._fit_polynomial(adaptive_points)
                
                # ===== GUARDAR ÚLTIMO CARRIL BUENO =====
                if adaptive_coeffs is not None:
                    self.last_good_coeffs = adaptive_coeffs
                    self.last_good_points = adaptive_points
                    self.last_good_time = self.get_clock().now()

        # Ajustar polinomio del lane mask si hay suficientes puntos
        if len(lane_mask_points) >= 3:
            lane_mask_coeffs = self._fit_lane_mask_polynomial(lane_mask_points)

        # 3. COMPARAR BORDES Y MANEJAR LAS 3 CIRCUNSTANCIAS
        same_edge, rightmost_points, other_points, lane_mask_quality = self._compare_edges(
            adaptive_points, lane_mask_points
        )
        
        # ===== DECISIÓN DE QUÉ ÁNGULO USAR =====
        coeffs_for_angle = None
        angle_source = "EDGE"  # Para debug
        
        # Priorizar lane mask si es de buena calidad (> 0.6) y tiene coeficientes
        if lane_mask_coeffs is not None and lane_mask_quality > 0.6:
            coeffs_for_angle = lane_mask_coeffs
            angle_source = "MASK (high quality)"
        # Si lane mask no es bueno pero hay borde adaptativo
        elif adaptive_coeffs is not None:
            coeffs_for_angle = adaptive_coeffs
            angle_source = "EDGE"
        # Si no hay borde adaptativo pero lane mask tiene algún valor
        elif lane_mask_coeffs is not None:
            coeffs_for_angle = lane_mask_coeffs
            angle_source = "MASK (fallback)"
        
        # Circunstancia 1: Mismo borde
        if same_edge and len(lane_mask_points) > 0:
            # Actualizar ambos topicos con los mismos puntos
            points_for_both = lane_mask_points if len(lane_mask_points) > len(adaptive_points) else adaptive_points
            self._update_lane_map_points(points_for_both, use_rightmost=False)
            self._update_lane_mask_points(points_for_both)
            
        # Circunstancia 2: Bordes diferentes
        elif not same_edge and len(adaptive_points) > 0 and len(lane_mask_points) > 0:
            # Borde derecho (más a la derecha) va a '/lane/edge_points'
            if rightmost_points is not None:
                self._update_lane_map_points(rightmost_points, use_rightmost=True)
            
            # Borde de lane mask va a '/lane/mask_points'
            if other_points is not None:
                self._update_lane_mask_points(other_points)
        
        # Circunstancia 3: Solo borde derecho
        elif len(adaptive_points) > 0 and len(lane_mask_points) == 0:
            self._update_lane_map_points(adaptive_points, use_rightmost=True)
        
        # Circunstancia adicional: Solo lane mask
        elif len(adaptive_points) == 0 and len(lane_mask_points) > 0:
            self._update_lane_mask_points(lane_mask_points)
            # También actualizar el borde principal si no hay borde adaptativo
            self._update_lane_map_points(lane_mask_points, use_rightmost=True)

        # 4. Publicar PointClouds
        self._publish_point_clouds()
        
        # 5. Crear imagen de debug
        debug_img = self._create_debug_image(self.mask, horizon_data, adaptive_points, adaptive_coeffs,
                                            lane_mask_points, lane_mask_coeffs, coeffs_for_angle, angle_source)
        
        # 6. Publicar resultados restantes (incluyendo ángulo priorizado)
        self._publish_results(coeffs_for_angle, adaptive_points, debug_img)
        
        # 7. Mostrar debug (opcional)
        if self.get_parameter('show_debug').value and debug_img is not None:
            cv2.imshow("Adaptive Lane Detection", debug_img)
            cv2.waitKey(1)
        
    def destroy_node(self):
        """Cleanup."""
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = LaneEdgeIPMNodeAdaptive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            pass


if __name__ == '__main__':
    main()