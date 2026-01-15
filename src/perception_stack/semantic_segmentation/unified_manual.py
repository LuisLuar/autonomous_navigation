#!/usr/bin/env python3
"""
LANE EDGE ESTIMATOR WITH IPM TRANSFORMATION - INTERACTIVE
=========================================================
Agrega trackbars para ajustar en tiempo real:
1. Base ratio (posición del punto más cercano)
2. Range ratio (distancia entre puntos)
3. ROI ratio (desde dónde empezar a buscar)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import json
import math
from collections import deque

from custom_interfaces.msg import SegmentationData
from std_msgs.msg import Float32, Float32MultiArray, Bool, Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class LaneEdgeIPMNodeInteractive(Node):
    def __init__(self):
        super().__init__('lane_edge_ipm_node_interactive')
        
        # =================== PARÁMETROS IPM ===================
        calibration_path = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration.json"
        try:
            with open(calibration_path, 'r') as f:
                calib = json.load(f)
            
            # Parámetros calibrados
            self.h = float(calib['camera_height'])        # 0.38 m
            self.pitch = float(calib['camera_pitch'])     # 0.1312 rad (7.52°)
            self.fx = float(calib['intrinsics']['fx'])    # 574.1
            self.fy = float(calib['intrinsics']['fy'])    # 574.1
            self.cx = float(calib['intrinsics']['cx'])    # 320.0
            self.cy = float(calib['intrinsics']['cy'])    # 240.0
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ IPM CON CALIBRACIÓN EXACTA")
            self.get_logger().info(f"  Pitch: {self.pitch:.4f} rad ({math.degrees(self.pitch):.2f}°)")
            self.get_logger().info(f"  Altura: {self.h} m")
            self.get_logger().info("=" * 60)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error cargando calibración: {e}")
            rclpy.shutdown()
            return

        # =================== PARÁMETROS INTERACTIVOS ===================
        # Valores iniciales (puedes ajustar estos según necesites)
        self.base_ratio = 0.69  # 65% de la altura (punto más cercano)
        self.range_ratio = 0.08  # 20% de extensión hacia arriba
        self.roi_ratio = 0.64   # Comenzar a procesar desde 45% de la imagen
        
        # Configuración de trackbars
        self.window_name = "Lane Points Configuration"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)
        
        # Crear trackbars
        cv2.createTrackbar('Base Ratio (%)', self.window_name, 
                          int(self.base_ratio * 100), 100, 
                          self.on_base_ratio_change)
        cv2.createTrackbar('Range Ratio (%)', self.window_name, 
                          int(self.range_ratio * 100), 50,
                          self.on_range_ratio_change)
        cv2.createTrackbar('ROI Ratio (%)', self.window_name, 
                          int(self.roi_ratio * 100), 100,
                          self.on_roi_ratio_change)
        cv2.createTrackbar('Slice Height', self.window_name, 10, 50,
                          self.on_slice_height_change)
        
        # Parámetros adicionales
        self.slice_height_div = 50  # División para altura de franja
        
        # Parámetros BEV
        self.declare_parameter("bev_scale", 40.0)
        self.declare_parameter("max_distance", 12.0)
        self.declare_parameter("min_distance", 0.5)
        
        self.scale = self.get_parameter("bev_scale").value
        self.max_distance = self.get_parameter("max_distance").value
        self.min_distance = self.get_parameter("min_distance").value
        
        # =================== PARÁMETROS LANE DETECTION ===================
        self.declare_parameter('num_horizons', 5)
        self.declare_parameter('ref_ratio', 0.8)
        self.declare_parameter('min_confidence_ratio', 0.3)
        self.declare_parameter('outlier_percentile_low', 5)
        self.declare_parameter('outlier_percentile_high', 90)
        self.declare_parameter('min_points_per_horizon', 5)
        self.declare_parameter('use_lane_mask', True)
        self.declare_parameter('use_road_mask', True)
        
        # =================== PARÁMETROS POLYNOMIAL FIT ===================
        self.declare_parameter('poly_degree', 2)
        self.declare_parameter('evaluation_distance', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('show_debug', True)
        
        # =================== ESTADO ===================
        self.mask = None
        self.camera_status = 0
        self.bridge = CvBridge()
        
        # Buffers para filtrado
        self.error_buffer = deque(maxlen=5)
        self.poly_coeffs_buffer = deque(maxlen=3)
        
        # Variables de control
        self.lane_points_pixel = []
        self.lane_points_meters = []
        self.poly_coeffs = None
        
        # =================== ROS SETUP ===================
        # Suscripciones
        self.create_subscription(
            SegmentationData, '/segmentation/data', 
            self.seg_cb, 1
        )
        self.create_subscription(
            DiagnosticStatus, '/status/camera', 
            self.cb_camera_status, 10
        )
        
        # Publicadores
        self.pub_error_meters = self.create_publisher(Float32, '/lane/error_meters', 10)
        self.pub_error_pixels = self.create_publisher(Float32, '/lane/error_pixels', 10)
        self.pub_poly_coeffs = self.create_publisher(Float32MultiArray, '/lane/polynomial_coeffs', 10)
        self.pub_lane_points = self.create_publisher(PointCloud2, '/lane/edge_points', 10)
        self.pub_valid = self.create_publisher(Bool, '/active/lane', 10)
        self.pub_debug = self.create_publisher(Image, '/lane_debug', 10)
        self.pub_bev = self.create_publisher(Image, '/ipm/bev_lane', 10)
        self.pub_lane_path = self.create_publisher(Path, '/lane/edge_path', 10)
        
        # Timer
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self.timer_cb
        )
        
        self.get_logger().info("🎮 Lane Edge IPM Node INTERACTIVE READY")
        self.get_logger().info("  Use trackbars to adjust point positions")
    
    # =================== CALLBACKS DE TRACKBARS ===================
    def on_base_ratio_change(self, val):
        """Cambia la posición base de los puntos (más cercano)."""
        self.base_ratio = val / 100.0
        self.get_logger().info(f"Base ratio ajustado a: {self.base_ratio:.2f}")
    
    def on_range_ratio_change(self, val):
        """Cambia la extensión entre puntos."""
        self.range_ratio = val / 100.0
        self.get_logger().info(f"Range ratio ajustado a: {self.range_ratio:.2f}")
    
    def on_roi_ratio_change(self, val):
        """Cambia desde dónde empezar a buscar."""
        self.roi_ratio = val / 100.0
        self.get_logger().info(f"ROI ratio ajustado a: {self.roi_ratio:.2f}")
    
    def on_slice_height_change(self, val):
        """Cambia la altura de la franja de búsqueda."""
        self.slice_height_div = max(2, val)
        self.get_logger().info(f"Slice height ajustado a: {self.slice_height_div}")
    
    # =================== CALLBACKS ROS ===================
    def cb_camera_status(self, msg: DiagnosticStatus):
        """Monitoreo del estado de la cámara."""
        self.camera_status = self._get_level(msg)
        if self.camera_status == 2:  # Error crítico
            self._publish_invalid()
    
    def seg_cb(self, msg: SegmentationData):
        """Recibe la máscara de segmentación."""
        try:
            self.mask = np.frombuffer(
                msg.mask_data, np.uint8
            ).reshape(msg.height, msg.width)
        except Exception as e:
            self.get_logger().error(f"Error reshaping mask: {e}")
            self.mask = None
    
    # =================== EXTRACCIÓN DE PUNTOS INTERACTIVA ===================
    def _extract_horizon_points_interactive(self, mask):
        """Extrae puntos usando parámetros interactivos."""
        if mask is None:
            return []
        
        h, w = mask.shape
        num_horizons = self.get_parameter('num_horizons').value
        
        horizon_data = []
        
        for i in range(num_horizons):
            # Ratio va de 0.0 (más lejano) a 1.0 (más cercano)
            ratio = i / max(1, num_horizons - 1)
            
            # Calcular posición vertical usando parámetros interactivos
            y_center = int(h * (self.base_ratio + ratio * self.range_ratio))
            
            # Franja vertical ajustable
            slice_height = h // self.slice_height_div
            y1 = max(int(h * self.roi_ratio), y_center - slice_height // 2)
            y2 = min(h, y_center + slice_height // 2)
            
            # Si el punto está fuera del ROI, saltar
            if y_center < int(h * self.roi_ratio):
                continue
            
            # Solo lado derecho (donde está el carril)
            slice_mask = mask[y1:y2, w//2:w]
            
            # Extraer borde
            edge_x, confidence = self._extract_edge_robust(slice_mask, y1, y2)
            
            if edge_x is not None:
                # Ajustar a coordenadas globales
                global_x = edge_x + w // 2
                
                horizon_data.append({
                    'horizon_idx': i,
                    'y': y_center,
                    'x': global_x,
                    'confidence': confidence,
                    'slice_bounds': (y1, y2),
                    'distance_ratio': ratio  # 0=lejano, 1=cercano
                })
        
        return horizon_data
    
    def _extract_edge_robust(self, slice_mask, y1, y2):
        """Extrae el borde del carril - Toma SIEMPRE el borde más a la DERECHA."""
        min_points = self.get_parameter('min_points_per_horizon').value
        use_lane = self.get_parameter('use_lane_mask').value
        use_road = self.get_parameter('use_road_mask').value
        
        # ========== ESTRATEGIA MEJORADA ==========
        # 1. Extraer todos los puntos válidos (lane=2, road=1)
        # 2. Filtrar outliers
        # 3. Tomar el percentil más alto (más a la derecha)
        # 4. Si lane y road están cerca, combinarlos
        
        all_points = []
        
        # Extraer lane points (valor 2)
        if use_lane:
            lane_ys, lane_xs = np.where(slice_mask == 2)
            for x, y in zip(lane_xs, lane_ys):
                all_points.append((x, y, 'lane', 1.0))  # Lane tiene mayor peso
        
        # Extraer road points (valor 1)
        if use_road:
            road_ys, road_xs = np.where(slice_mask == 1)
            for x, y in zip(road_xs, road_ys):
                all_points.append((x, y, 'road', 0.7))  # Road tiene menor peso
        
        if len(all_points) < min_points:
            return None, 0.0
        
        # Convertir a arrays
        xs = np.array([p[0] for p in all_points])
        ys = np.array([p[1] for p in all_points])
        types = np.array([p[2] for p in all_points])
        weights = np.array([p[3] for p in all_points])
        
        # Filtrar outliers - más agresivo
        low_pct = self.get_parameter('outlier_percentile_low').value
        high_pct = min(98, self.get_parameter('outlier_percentile_high').value)  # Máximo 98%
        
        x_low = np.percentile(xs, low_pct)
        x_high = np.percentile(xs, high_pct)
        
        valid_mask = (xs >= x_low) & (xs <= x_high)
        filtered_xs = xs[valid_mask]
        filtered_types = types[valid_mask]
        
        if len(filtered_xs) < min_points:
            return None, 0.0
        
        # Análisis de distribución
        unique_types, type_counts = np.unique(filtered_types, return_counts=True)
        
        # Calcular percentiles por tipo
        lane_xs = filtered_xs[filtered_types == 'lane']
        road_xs = filtered_xs[filtered_types == 'road']
        
        lane_percentile = np.percentile(lane_xs, 90) if len(lane_xs) > 0 else -1
        road_percentile = np.percentile(road_xs, 90) if len(road_xs) > 0 else -1
        
        # ESTRATEGIA DE DECISIÓN:
        # 1. Si hay lane points, usar lane (más confiable para borde)
        # 2. Si lane no tiene suficientes puntos pero road sí, usar road
        # 3. Si ambos tienen puntos, usar el que esté MÁS A LA DERECHA
        
        selected_edge = None
        selected_conf = 0.0
        selected_type = None
        
        # Umbral de proximidad para combinar (20 píxeles)
        proximity_threshold = 20
        
        # Caso 1: Ambos tienen puntos suficientes
        if len(lane_xs) >= min_points and len(road_xs) >= min_points:
            # Si están cerca, combinar (promedio ponderado)
            if abs(lane_percentile - road_percentile) < proximity_threshold:
                # Promedio ponderado (lane tiene más peso)
                selected_edge = int((lane_percentile * 0.7 + road_percentile * 0.3))
                selected_type = 'combined'
                selected_conf = min(1.0, (len(lane_xs) + len(road_xs)) / 
                                (slice_mask.shape[0] * slice_mask.shape[1]) * 15)
            else:
                # Tomar el que esté MÁS A LA DERECHA
                if lane_percentile > road_percentile:
                    selected_edge = int(lane_percentile)
                    selected_type = 'lane'
                    selected_conf = min(1.0, len(lane_xs) / 
                                    (slice_mask.shape[0] * slice_mask.shape[1]) * 12)
                else:
                    selected_edge = int(road_percentile)
                    selected_type = 'road'
                    selected_conf = min(1.0, len(road_xs) / 
                                    (slice_mask.shape[0] * slice_mask.shape[1]) * 10)
        
        # Caso 2: Solo lane tiene puntos suficientes
        elif len(lane_xs) >= min_points:
            selected_edge = int(lane_percentile)
            selected_type = 'lane'
            selected_conf = min(1.0, len(lane_xs) / 
                            (slice_mask.shape[0] * slice_mask.shape[1]) * 12)
        
        # Caso 3: Solo road tiene puntos suficientes
        elif len(road_xs) >= min_points:
            selected_edge = int(road_percentile)
            selected_type = 'road'
            selected_conf = min(1.0, len(road_xs) / 
                            (slice_mask.shape[0] * slice_mask.shape[1]) * 10)
        
        # Caso 4: Combinados alcanzan el mínimo
        elif len(filtered_xs) >= min_points * 2:  # Requiere más puntos si son combinados
            # Tomar percentil alto de todos los puntos combinados
            selected_edge = int(np.percentile(filtered_xs, 92))  # Percentil más alto
            selected_type = 'mixed'
            selected_conf = min(1.0, len(filtered_xs) / 
                            (slice_mask.shape[0] * slice_mask.shape[1]) * 8)
        
        if selected_edge is not None:
            # Debug info
            debug_msg = f"Borde seleccionado: {selected_type} en x={selected_edge}, "
            debug_msg += f"conf={selected_conf:.2f}, "
            debug_msg += f"lane_pts={len(lane_xs)}, road_pts={len(road_xs)}"
            self.get_logger().debug(debug_msg)
            
            return selected_edge, selected_conf
        
        return None, 0.0
    
    # =================== TRANSFORMACIÓN IPM ===================
    def _pixel_to_meters(self, u, v):
        """
        Transforma un punto (u, v) de píxeles a coordenadas métricas.
        """
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
        valid_points_pixel = []
        
        for h_data in horizon_data:
            u = h_data['x']
            v = h_data['y']
            
            result = self._pixel_to_meters(u, v)
            if result is not None:
                X, Z = result
                
                # Filtrar por distancia mínima y máxima
                if self.min_distance <= Z <= self.max_distance:
                    points_meters.append((Z, X))
                    valid_points_pixel.append((u, v))
        
        return np.array(points_meters), valid_points_pixel
    
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
    
    def _evaluate_polynomial(self, coeffs, Z):
        """Evalúa el polinomio en una distancia Z."""
        if coeffs is None:
            return 0.0
        return np.polyval(coeffs, Z)
    
    # =================== CÁLCULO DE ERROR ===================
    def _calculate_errors(self, coeffs, horizon_data):
        """Calcula error en metros y píxeles."""
        # Error en metros
        eval_distance = self.get_parameter('evaluation_distance').value
        error_meters = self._evaluate_polynomial(coeffs, eval_distance)
        
        # Error en píxeles
        error_pixels = 0.0
        if horizon_data:
            # Punto más cercano (mayor ratio)
            closest_point = max(horizon_data, key=lambda x: x.get('distance_ratio', 0))
            ref_x = self.cx * 2 * self.get_parameter('ref_ratio').value
            error_pixels = (ref_x - closest_point['x']) / (self.cx * 2)
            error_pixels = float(np.clip(error_pixels, -1.0, 1.0))
        
        return error_meters, error_pixels
    
    # =================== VISUALIZACIÓN MEJORADA ===================
    def _create_debug_image(self, mask, horizon_data, points_meters, coeffs, error_meters, error_pixels, valid):
        """Crea imagen de debug con controles visuales."""
        if mask is None:
            return None
        
        h, w = mask.shape
        
        # Crear imagen base
        img = np.full((h, w, 3), 225, dtype=np.uint8)
        
        # Colorear máscara de segmentación
        lane_pixels = (mask == 2)
        road_pixels = (mask == 1)
        img[lane_pixels] = [0, 100, 200]
        img[road_pixels] = [50, 50, 50]
        
        # ===== DIBUJAR LÍNEAS GUÍA =====
        # Línea de ROI
        roi_line_y = int(h * self.roi_ratio)
        cv2.line(img, (0, roi_line_y), (w, roi_line_y), 
                (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f"ROI: {self.roi_ratio:.2f}", 
                   (w - 100, roi_line_y - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        # Línea de base (punto más cercano)
        base_line_y = int(h * self.base_ratio)
        cv2.line(img, (0, base_line_y), (w, base_line_y),
                (255, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f"Base: {self.base_ratio:.2f}",
                   (w - 100, base_line_y - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
        
        # Línea de tope (punto más lejano)
        top_line_y = int(h * (self.base_ratio + self.range_ratio))
        if top_line_y < h:
            cv2.line(img, (0, top_line_y), (w, top_line_y),
                    (255, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(img, f"Top: {self.base_ratio + self.range_ratio:.2f}",
                       (w - 100, top_line_y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # ===== DIBUJAR PUNTOS DETECTADOS =====
        colors = [(0, 255, 255), (0, 255, 128), (0, 255, 0), 
                  (128, 255, 0), (255, 128, 0)]
        
        for i, h_data in enumerate(horizon_data):
            color = colors[min(i, len(colors)-1)]
            confidence = h_data.get('confidence', 1.0)
            y1, y2 = h_data.get('slice_bounds', (0, 0))
            
            # Dibujar franja de búsqueda
            cv2.rectangle(img, (w//2, y1), (w, y2), 
                         (color[0]//3, color[1]//3, color[2]//3), 1)
            
            # Punto detectado
            radius = int(4 + confidence * 4)
            cv2.circle(img, (h_data['x'], h_data['y']), radius, color, -1)
            cv2.circle(img, (h_data['x'], h_data['y']), radius+2, (255, 255, 255), 1)
            
            # Label con información
            dist_ratio = h_data.get('distance_ratio', 0)
            text = f"H{i}({dist_ratio:.1f})"
            cv2.putText(img, text, (h_data['x'] + 10, h_data['y'] - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
        
        # ===== INFORMACIÓN TEXTUAL =====
        info_y = 30
        spacing = 25
        
        # Parámetros actuales
        cv2.putText(img, f"Base: {self.base_ratio:.2f}, Range: {self.range_ratio:.2f}, ROI: {self.roi_ratio:.2f}", 
                   (20, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        info_y += spacing
        
        # Errores
        error_color = (0,255,0) if abs(error_meters) < 0.1 else (0,200,255) if abs(error_meters) < 0.3 else (0,0,255)
        cv2.putText(img, f"Error: {error_meters:+.3f} m", (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, error_color, 2)
        info_y += spacing
        
        cv2.putText(img, f"Error px: {error_pixels:+.3f}", (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += spacing
        
        # Coeficientes
        if coeffs is not None:
            coeff_text = f"Poly: {coeffs[0]:.4f}x² + {coeffs[1]:.4f}x + {coeffs[2]:.4f}"
            cv2.putText(img, coeff_text, (20, info_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
            info_y += spacing
        
        # Puntos válidos
        status_color = (0,255,0) if valid else (0,0,255)
        cv2.putText(img, f"Points: {len(points_meters)}/{len(horizon_data)}", 
                   (20, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 1)
        
        # ===== DISTANCIAS EN METROS =====
        if len(points_meters) > 0:
            dist_y = h - 30
            for i, (Z, X) in enumerate(points_meters[:3]):  # Mostrar primeros 3
                if i < len(horizon_data):
                    point_y = horizon_data[i]['y']
                    dist_text = f"Z={Z:.1f}m, X={X:.2f}m"
                    cv2.putText(img, dist_text, (w - 200, point_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # ===== BEV OVERLAY =====
        bev_overlay = self._create_bev_overlay(points_meters, coeffs)
        if bev_overlay is not None:
            bev_h, bev_w = bev_overlay.shape[:2]
            scale_factor = 0.3
            new_h, new_w = int(h * scale_factor), int(w * scale_factor)
            bev_resized = cv2.resize(bev_overlay, (new_w, new_h))
            img[10:10+new_h, w-new_w-10:w-10] = bev_resized
        
        return img
    
    def _create_lane_path(self, coeffs):
        """Crea un Path mensaje desde los coeficientes del polinomio."""
        if coeffs is None or len(coeffs) < 3:
            return None
        
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "base_footprint"
        
        # Generar puntos a lo largo del polinomio
        for z in np.arange(0.5, 5.0, 0.1):  # De 0.5m a 5m cada 10cm
            x = np.polyval(coeffs, z)  # Posición lateral
            
            # Crear PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            
            # La pose está en base_footprint: X=z (adelante), Y=-x (izquierda)
            pose_stamped.pose.position.x = float(z)      # Adelante
            pose_stamped.pose.position.y = float(-x)     # Izquierda (negativo de X)
            pose_stamped.pose.position.z = 0.0
            
            # Orientación mirando hacia adelante
            pose_stamped.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            path_msg.poses.append(pose_stamped)
        
        return path_msg

    def _create_bev_overlay(self, points_meters, coeffs):
        """Crea una vista BEV pequeña para overlay."""
        if len(points_meters) == 0:
            return None
        
        bev_h = 200
        bev_w = 200
        bev = np.zeros((bev_h, bev_w, 3), dtype=np.uint8)
        
        max_z = min(self.max_distance, 5.0)
        scale_z = bev_h / max_z
        scale_x = bev_w / 4.0
        
        center_x = bev_w // 2
        cv2.line(bev, (center_x, 0), (center_x, bev_h), (100, 100, 100), 1)
        
        # Dibujar puntos
        for Z, X in points_meters:
            if Z <= max_z:
                y = int(bev_h - Z * scale_z)
                x = int(center_x + X * scale_x)
                if 0 <= x < bev_w and 0 <= y < bev_h:
                    cv2.circle(bev, (x, y), 3, (0, 255, 0), -1)
        
        # Dibujar polinomio
        if coeffs is not None:
            poly_points = []
            for z in np.linspace(0, max_z, 50):
                x = np.polyval(coeffs, z)
                y_px = int(bev_h - z * scale_z)
                x_px = int(center_x + x * scale_x)
                if 0 <= x_px < bev_w and 0 <= y_px < bev_h:
                    poly_points.append((x_px, y_px))
            
            if len(poly_points) >= 2:
                for i in range(len(poly_points)-1):
                    cv2.line(bev, poly_points[i], poly_points[i+1], 
                            (0, 200, 255), 2)
        
        # Punto de evaluación
        eval_z = 1.0
        eval_y = int(bev_h - eval_z * scale_z)
        cv2.line(bev, (0, eval_y), (bev_w, eval_y), (255, 255, 0), 1)
        
        cv2.putText(bev, "BEV", (5, 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return bev
    
    # =================== PUBLICACIÓN ===================
    def _publish_invalid(self):
        """Publica estado inválido."""
        self.pub_valid.publish(Bool(data=False))
        self.pub_error_meters.publish(Float32(data=0.0))
        self.pub_error_pixels.publish(Float32(data=0.0))
    
    def _publish_results(self, error_meters, error_pixels, coeffs, points_meters, valid, debug_img):
        """Publica todos los resultados."""
        # Errores
        self.pub_error_meters.publish(Float32(data=float(error_meters)))
        self.pub_error_pixels.publish(Float32(data=float(error_pixels)))
        self.pub_valid.publish(Bool(data=valid))
        
        # Coeficientes
        if coeffs is not None:
            coeffs_msg = Float32MultiArray()
            coeffs_msg.data = coeffs.tolist()
            self.pub_poly_coeffs.publish(coeffs_msg)
        
        # PointCloud2
        if len(points_meters) > 0:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "base_footprint"
            
            points_cloud = []
            for Z, X in points_meters:
                points_cloud.append([Z, -X, 0.0])
            
            pc_msg = point_cloud2.create_cloud_xyz32(header, points_cloud)
            self.pub_lane_points.publish(pc_msg)
        
        # Después de publicar el PointCloud2
        if coeffs is not None:
            lane_path = self._create_lane_path(coeffs)
            if lane_path is not None:
                self.pub_lane_path.publish(lane_path)

        # Imagen de debug
        if debug_img is not None:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
    
    # =================== MAIN LOOP ===================
    def timer_cb(self):
        """Loop principal."""
        # Procesar eventos de OpenCV
        cv2.waitKey(1)
        
        if self.mask is None or self.camera_status == 2:
            self._publish_invalid()
            return
        
        # 1. Extraer puntos con parámetros interactivos
        horizon_data = self._extract_horizon_points_interactive(self.mask)
        
        valid = False
        error_meters = 0.0
        error_pixels = 0.0
        coeffs = None
        points_meters = np.array([])
        
        if len(horizon_data) >= 3:
            # 2. Transformar puntos a metros
            points_meters, valid_points_pixel = self._transform_points_to_meters(horizon_data)
            
            if len(points_meters) >= 3:
                # 3. Ajustar polinomio
                coeffs = self._fit_polynomial(points_meters)
                
                if coeffs is not None:
                    # 4. Calcular errores
                    error_meters, error_pixels = self._calculate_errors(coeffs, horizon_data)
                    valid = True
        
        # 5. Crear imagen de debug
        debug_img = self._create_debug_image(
            self.mask, horizon_data, points_meters, 
            coeffs, error_meters, error_pixels, valid
        )
        
        # 6. Publicar resultados
        self._publish_results(
            error_meters, error_pixels, coeffs, 
            points_meters, valid, debug_img
        )
        
        # 7. Mostrar ventana de configuración
        if debug_img is not None:
            cv2.imshow("Lane Detection", debug_img)
    
    # =================== UTILITIES ===================
    def _get_level(self, msg):
        """Extrae nivel de diagnóstico."""
        level = msg.level
        if isinstance(level, bytes):
            return int.from_bytes(level, byteorder='little', signed=False)
        try:
            return int(level)
        except:
            return 2
    
    def destroy_node(self):
        """Cleanup."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = LaneEdgeIPMNodeInteractive()
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