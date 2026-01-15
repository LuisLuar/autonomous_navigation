#!/usr/bin/env python3
"""
LANE EDGE ESTIMATOR WITH IPM TRANSFORMATION - ADAPTIVE
=========================================================
Parámetros adaptativos:
1. Base ratio: calcula el punto más arriba con lane/road mask
2. Range ratio: se ajusta para evitar la parte inferior donde road mask ocupa todo
3. Cálculo del ángulo del borde del carril
4. Cálculo de errores para control PD
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
from std_msgs.msg import Bool

class LaneEdgeIPMNodeAdaptive(Node):
    def __init__(self):
        super().__init__('lane_edge_ipm_adaptive')
        
        # =================== PARÁMETROS IPM ===================
        calibration_path = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration.json"
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
            
            #self.get_logger().info("=" * 60)
            #self.get_logger().info("IPM CON CALIBRACIÓN EXACTA")
            #self.get_logger().info(f"  Pitch: {self.pitch:.4f} rad ({math.degrees(self.pitch):.2f}°)")
            #self.get_logger().info(f"  Altura: {self.h} m")
            #self.get_logger().info("=" * 60)
            
        except Exception as e:
            #self.get_logger().error(f"Error cargando calibración: {e}")
            rclpy.shutdown()
            return

        # =================== PARÁMETROS ADAPTATIVOS INICIALES ===================
        self.base_ratio = 0.65  # Valor inicial, se calculará dinámicamente
        self.range_ratio = 0.2  # Valor inicial, se calculará dinámicamente
        self.roi_ratio = 0.45   # Fijo - comenzar desde 45% de la imagen
        
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
        
        # =================== PARÁMETROS POLYNOMIAL FIT ===================
        self.declare_parameter('poly_degree', 2)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('show_debug', True)

        # =================== PARÁMETROS DE CONTROL ===================
        self.declare_parameter('path_offset', 1.5)  # Desplazamiento del robot desde el borde
        self.path_offset = self.get_parameter('path_offset').value
        
        # =================== NUEVO: PARÁMETRO PARA ÁNGULO ===================
        self.declare_parameter('lookahead_distance', 3.0)  # Distancia para calcular ángulo
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        # =================== NUEVO: PARÁMETROS PARA ERRORES PD ===================
        self.declare_parameter('max_angle_error', 3.0)  # Ángulo máximo en grados antes de generar error
        self.max_angle_error = self.get_parameter('max_angle_error').value
        
        # =================== ESTADO ===================
        self.mask = None
        self.bridge = CvBridge()
        
        # Historial para suavizado adaptativo
        self.base_ratio_history = deque(maxlen=10)
        self.range_ratio_history = deque(maxlen=10)
        
        # =================== NUEVO: HISTORIAL PARA ÁNGULO ===================
        self.angle_history = deque(maxlen=5)
        
        # =================== NUEVO: HISTORIAL PARA ERROR DE ÁNGULO ===================
        self.angle_error_history = deque(maxlen=5)
        
        # =================== ROS SETUP ===================
        # Suscripciones
        self.create_subscription(
            SegmentationData, '/segmentation/data', 
            self.seg_cb, 1
        )
        
        # Publicadores
        self.pub_poly_coeffs = self.create_publisher(Float32MultiArray, '/lane/polynomial_coeffs', 10)
        self.pub_lane_points = self.create_publisher(PointCloud2, '/lane/edge_points', 10)
        self.pub_debug = self.create_publisher(Image, '/lane_debug', 10)
        self.pub_lane_path = self.create_publisher(Path, '/lane/edge_path', 10)
        self.pub_robot_path = self.create_publisher(Path, '/lane/offset_path', 10)
        
        # =================== NUEVO: PUBLICADOR PARA ERROR DE ÁNGULO (Float32) ===================
        self.pub_angle_error = self.create_publisher(Float32, '/omega/lane_orientation', 10)
        

        # =================== NUEVO: PARÁMETRO PARA CONFIANZA ===================
        self.declare_parameter('min_confidence_points', 2)  # Mínimo de puntos para considerar confiable
        self.min_confidence_points = self.get_parameter('min_confidence_points').value

        # En ROS SETUP (después de la línea 109):

        # En la sección de estado (después de línea 91):
        self.lane_active = True  # Por defecto activo
        self.valid_lane_detected = False
        
        # Timer
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self.timer_cb
        )
        
        #self.get_logger().info("Lane Edge IPM Node ADAPTIVE READY")
    
    # =================== NUEVO: MÉTODO PARA CALCULAR ERROR DE ÁNGULO ===================
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
        angle_error_normalized = np.clip(angle_error / self.max_angle_range, -1.0, 1.0)
        
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
        Calcula base_ratio y range_ratio de forma adaptativa basado en la máscara.
        
        Estrategia:
        1. base_ratio: Punto más arriba donde hay lane/road mask válida
        2. range_ratio: Se ajusta para evitar zona donde road mask ocupa todo el ancho
        """
        if mask is None:
            return self.base_ratio, self.range_ratio
        
        h, w = mask.shape
        
        # ===== 1. CALCULAR BASE_RATIO (punto más arriba con datos válidos) =====
        # Analizar solo el lado derecho de la imagen DESPUÉS del ROI
        roi_start = int(h * self.roi_ratio)
        right_half = mask[roi_start:, w//2:w]  # Solo área del ROI hacia abajo
        
        # Encontrar la fila más alta con lane mask (2) o road mask (1)
        lane_rows, _ = np.where(right_half == 2)
        road_rows, _ = np.where(right_half == 1)
        
        min_lane_row = np.min(lane_rows) if len(lane_rows) > 0 else h
        min_road_row = np.min(road_rows) if len(road_rows) > 0 else h
        
        # Tomar la fila más alta (menor valor) entre lane y road
        min_row = min(min_lane_row, min_road_row)
        
        if min_row < h - roi_start:  # Se encontraron datos válidos en el ROI
            # Convertir a ratio global (0 en parte superior, 1 en inferior)
            new_base_ratio = (roi_start + min_row) / h
            
            # Aplicar límites
            new_base_ratio = max(self.roi_ratio + 0.05, min(0.8, new_base_ratio))
            
            # Suavizado con historial
            self.base_ratio_history.append(new_base_ratio)
            if len(self.base_ratio_history) > 0:
                self.base_ratio = np.mean(self.base_ratio_history)
        else:
            # Fallback: usar valor ligeramente arriba del ROI
            self.base_ratio = self.roi_ratio + 0.1
        
        # ===== 2. CALCULAR RANGE_RATIO (evitar zona inferior problemática) =====
        # Analizar distribución vertical de road mask SOLO en el ROI
        road_density_by_row = []
        
        # Analizar densidad de road mask por fila (en el lado derecho del ROI)
        for row in range(roi_start, h, 10):  # Desde ROI hacia abajo
            row_slice = mask[row:min(row+10, h), w//2:w]
            road_pixels = np.sum(row_slice == 1)
            total_pixels = row_slice.size
            density = road_pixels / total_pixels if total_pixels > 0 else 0
            road_density_by_row.append((row/h, density))
        
        # Encontrar donde road mask empieza a ocupar mucho ancho (densidad > 0.7)
        problem_row_ratio = 0.9  # Valor por defecto
        for row_ratio, density in road_density_by_row:
            if density > 0.7:  # Road mask ocupa más del 70% del ancho
                problem_row_ratio = row_ratio
                break
        
        # Calcular range_ratio para que los puntos estén ARRIBA de la zona problemática
        margin = 0.08  # Margen de seguridad
        max_allowed_range = max(0.05, problem_row_ratio - self.base_ratio - margin)
        
        # Ajustar range_ratio
        new_range_ratio = min(0.25, max(0.08, max_allowed_range))
        
        # Suavizado con historial
        self.range_ratio_history.append(new_range_ratio)
        if len(self.range_ratio_history) > 0:
            self.range_ratio = np.mean(self.range_ratio_history)
        
        return self.base_ratio, self.range_ratio
    
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
        """Extrae puntos usando parámetros adaptativos, RESPETANDO EL ROI."""
        if mask is None:
            return []
        
        # 1. Calcular ratios adaptativos basados en la máscara actual
        self._calculate_adaptive_ratios(mask)
        
        h, w = mask.shape
        num_horizons = self.get_parameter('num_horizons').value
        
        horizon_data = []
        
        # Calcular ROI absoluto
        roi_start_y = int(h * self.roi_ratio)
        
        for i in range(num_horizons):
            # Ratio va de 0.0 (más lejano) a 1.0 (más cercano)
            ratio = i / max(1, num_horizons - 1)
            
            # Calcular posición vertical usando parámetros adaptativos
            y_center = int(h * (self.base_ratio + ratio * self.range_ratio))
            
            # VERIFICACIÓN CRÍTICA: El punto DEBE estar dentro del ROI
            if y_center < roi_start_y:
                # Saltar este punto, está fuera del ROI
                continue
            
            # Franja vertical dinámica
            slice_height = int(h * 0.06)  # 6% de la altura fijo
            y1 = max(roi_start_y, y_center - slice_height // 2)
            y2 = min(h, y_center + slice_height // 2)
            
            # Solo lado derecho (donde está el carril)
            slice_mask = mask[y1:y2, w//2:w]
            
            # Extraer borde
            edge_x, confidence = self._extract_edge_robust(slice_mask)
            
            if edge_x is not None:
                # Ajustar a coordenadas globales
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
    def _create_debug_image(self, mask, horizon_data, points_meters, coeffs):
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
        roi_line_y = int(h * self.roi_ratio)
        cv2.line(img, (0, roi_line_y), (w, roi_line_y), 
                (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"ROI", (10, roi_line_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # ===== DIBUJAR LÍNEAS DE BASE Y TOPE =====
        # Línea de base (calculada adaptativamente)
        base_line_y = int(h * self.base_ratio)
        cv2.line(img, (0, base_line_y), (w, base_line_y),
                (255, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f"Base", (w - 60, base_line_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
        
        # Línea de tope (calculada adaptativamente)
        top_line_y = int(h * (self.base_ratio + self.range_ratio))
        if top_line_y < h:
            cv2.line(img, (0, top_line_y), (w, top_line_y),
                    (255, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(img, f"Top", (w - 50, top_line_y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
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
        info_y = 30
        spacing = 22
        
        # Título
        cv2.putText(img, "ADAPTIVE LANE DETECTION", (20, info_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        info_y += spacing
        
        # Parámetros adaptativos
        param_text = f"Base: {self.base_ratio:.3f}  Range: {self.range_ratio:.3f}  ROI: {self.roi_ratio:.2f}"
        cv2.putText(img, param_text, (20, info_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
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
            angle_error = self.calculate_angle_error(coeffs)
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
        
        return img
    
    # =================== PUBLICACIÓN ===================  
    def _publish_results(self, coeffs, points_meters, debug_img):
        """Publica todos los resultados."""
            
        # Coeficientes del polinomio
        if coeffs is not None:
            coeffs_msg = Float32MultiArray()
            coeffs_msg.data = coeffs.tolist()
            self.pub_poly_coeffs.publish(coeffs_msg)
            

            # ===== PUBLICAR ERROR DE ÁNGULO (Float32) =====
            angle_error = self.calculate_angle_error(coeffs)
            angle_error_msg = Float32()
            angle_error_msg.data = float(angle_error)
            self.pub_angle_error.publish(angle_error_msg)
        
        # PointCloud2 con puntos del borde
        if len(points_meters) > 0:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "base_footprint"
            
            points_cloud = []
            for Z, X in points_meters:
                points_cloud.append([Z, -X, 0.0])  # Y negativo porque apunta a izquierda
            
            pc_msg = point_cloud2.create_cloud_xyz32(header, points_cloud)
            self.pub_lane_points.publish(pc_msg)
        
        # Paths (borde y robot)
        if coeffs is not None:
            # Path del borde del carril (offset = 0)
            lane_path = self._create_polynomial_path(coeffs, offset=0.0)
            if lane_path is not None:
                self.pub_lane_path.publish(lane_path)
            
            # Path del robot (desplazado 1.5m del borde)
            robot_path = self._create_polynomial_path(coeffs, offset=self.path_offset)
            if robot_path is not None:
                self.pub_robot_path.publish(robot_path)
        
        # Imagen de debug (opcional)
        # if debug_img is not None:
        #     self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
    
    # =================== MAIN LOOP ===================
    def timer_cb(self):
        """Loop principal."""
        if self.mask is None:
            return
        
        # 1. Extraer puntos con parámetros adaptativos (respetando ROI)
        horizon_data = self._extract_horizon_points_adaptive(self.mask)
        
        coeffs = None
        points_meters = np.array([])
        
        if len(horizon_data) >= 3:
            # 2. Transformar puntos a metros
            points_meters = self._transform_points_to_meters(horizon_data)
            
            if len(points_meters) >= 3:
                # 3. Ajustar polinomio
                coeffs = self._fit_polynomial(points_meters)
        
        # 4. Crear imagen de debug
        debug_img = self._create_debug_image(
            self.mask, horizon_data, points_meters, coeffs
        )
        
        # 5. Publicar resultados
        self._publish_results(coeffs, points_meters, debug_img)
        
        # 6. Mostrar debug (opcional)
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
            print(f"Shutdown error: {e}")


if __name__ == '__main__':
    main()