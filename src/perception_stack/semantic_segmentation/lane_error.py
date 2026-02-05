#!/usr/bin/env python3
"""
CONTROL PREDICTIVO ROBUSTO - VERSIÓN MEJORADA
==============================================
Optimizado para:
- Carriles irregulares
- Sombras y cambios de iluminación
- Control suave y estable
- Múltiples estrategias de detección

CONVENCIÓN DE SIGNOS:
- ERROR POSITIVO (+) → Robot debe ALEJARSE del carril → Giro ANTIHORARIO (Z+)
- ERROR NEGATIVO (-) → Robot debe ACERCARSE del carril → Giro HORARIO (Z-)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from collections import deque

from custom_interfaces.msg import SegmentationData
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticStatus


class LaneErrorPredictiveRobust(Node):

    def __init__(self):
        super().__init__('lane_error_predictive_robust')
        
        self.bridge = CvBridge()
        
        # =================== PARÁMETROS DE HORIZONTES ===================
        # Configuración ORIGINAL (EXACTAMENTE como en el código original)
        self.declare_parameter('num_horizons_1', 5)
        self.declare_parameter('ref_ratio_1', 0.77)  # IGUAL al original: 0.8
        
        # Pesos EXACTAMENTE iguales al original
        self.declare_parameter('horizon_weights_1', [0.002, 0.003, 0.04, 0.002, 0.001])
        
        # Rango vertical EXACTO al original: (0.7 + ratio * 0.2) → entre 70% y 90%
        self.declare_parameter('y_range_1_start', 0.71)
        self.declare_parameter('y_range_1_end', 0.77)
        
        # Configuración NUEVA (solo lane_mask, prioriza izquierda)
        self.declare_parameter('num_horizons_2', 5)
        self.declare_parameter('ref_ratio_2', 0.75)  # Misma ref_ratio que original
        self.declare_parameter('horizon_weights_2', [0.002, 0.003, 0.04, 0.002, 0.001])
        
        # Rango vertical ligeramente diferente para comparación
        self.declare_parameter('y_range_2_start', 0.75)
        self.declare_parameter('y_range_2_end', 0.95)
        
        # =================== PARÁMETROS COMUNES (iguales al original) ===================
        self.declare_parameter('curve_detection_threshold', 0.15)
        self.declare_parameter('curve_weight_boost', 1.5)
        
        # ROBUSTEZ A SOMBRAS
        self.declare_parameter('use_lane_mask_1', True)  # Config 1: igual al original
        self.declare_parameter('use_road_mask_1', True)  # Config 1: igual al original
        self.declare_parameter('min_confidence_ratio', 0.3)
        self.declare_parameter('outlier_percentile_low', 5)
        self.declare_parameter('outlier_percentile_high', 90)
        self.declare_parameter('min_points_per_horizon', 5)
        
        # Config 2 específica
        self.declare_parameter('use_lane_mask_2', True)   # Config 2: solo lane mask
        self.declare_parameter('use_road_mask_2', False)  # Config 2: NO road mask
        self.declare_parameter('mask_priority_2', 'leftmost')  # Prioriza puntos izquierda
        
        # FILTRADO TEMPORAL
        self.declare_parameter('ema_alpha', 0.2)
        self.declare_parameter('use_median_filter', True)
        self.declare_parameter('median_window_size', 5)
        
        # FALLBACK Y VALIDACIÓN
        self.declare_parameter('fail_count_threshold', 8)
        self.declare_parameter('fallback_decay', 0.85)
        self.declare_parameter('min_valid_horizons', 2)
        
        # PUBLICACIÓN
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('show_debug', True)
        self.declare_parameter('use_config_1', True)  # True: usa Config 1, False: usa Config 2
        
        # =================== ESTADO ===================
        self.mask = None
        self.camera_status = 0
        
        # Estados para cada configuración
        self.filtered_error_1 = 0.0
        self.filtered_error_2 = 0.0
        self.last_valid_error_1 = 0.0
        self.last_valid_error_2 = 0.0
        self.fail_count_1 = 0
        self.fail_count_2 = 0
        self.is_curve_1 = False
        self.is_curve_2 = False
        
        # Buffers para filtro de mediana
        median_size = self.get_parameter('median_window_size').value
        self.error_buffer_1 = deque(maxlen=median_size)
        self.error_buffer_2 = deque(maxlen=median_size)
        
        # Historial para detección de curvas
        self.horizon_x_history_1 = deque(maxlen=10)
        self.horizon_x_history_2 = deque(maxlen=10)
        
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
        self.pub_error = self.create_publisher(Float32, '/omega/lane', 10)
        self.pub_valid = self.create_publisher(Bool, '/active/lane', 10)
        self.pub_debug = self.create_publisher(Image, '/lane_debug_image', 10)
        
        # Timer
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self.timer_cb
        )
    
    # =================== CALLBACKS ===================
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
            self.mask = None
    
    # =================== EXTRACCIÓN DE PUNTOS ===================
    def _extract_horizon_points_robust(self, mask, config_num):
        """
        Extrae puntos igual que en el código original para Config 1.
        Para Config 2, usa lógica simplificada.
        """
        if config_num == 1:
            # CONFIG 1: EXACTAMENTE igual al código original
            return self._extract_horizon_points_original(mask)
        else:
            # CONFIG 2: Nueva lógica simplificada
            return self._extract_horizon_points_simple(mask, config_num)
    
    def _extract_horizon_points_original(self, mask):
        """
        EXACTA copia de la función original.
        """
        h, w = mask.shape
        num_horizons = self.get_parameter('num_horizons_1').value
        
        horizon_data = []

        # OBTENER LOS PARÁMETROS DE RANGO Y
        y_start = self.get_parameter('y_range_1_start').value
        y_end = self.get_parameter('y_range_1_end').value
        
        for i in range(num_horizons):
            # Distribución no lineal: más resolución cerca del robot
            # i=0 → bottom (más cercano, más peso)
            # i=N-1 → top (más lejano, menos peso)
            ratio = (num_horizons - i - 1) / num_horizons
            y_center = int(h * (y_start + ratio * (y_end - y_start)))
            
            # Franja vertical alrededor del horizonte
            slice_height = h // (num_horizons * 15)
            y1 = max(0, y_center - slice_height)
            y2 = min(h, y_center + slice_height)
            
            # SOLO lado derecho (donde está el carril)
            slice_mask = mask[y1:y2, w//2:w]
            
            # === EXTRACCIÓN ROBUSTA (igual al original) ===
            edge_x, confidence = self._extract_edge_robust(slice_mask, y1, y2)
            
            if edge_x is not None:
                # Ajustar a coordenadas globales
                global_x = edge_x + w // 2
                
                horizon_data.append({
                    'horizon_idx': i,
                    'y': y_center,
                    'x': global_x,
                    'confidence': confidence,
                    'slice_bounds': (y1, y2)
                })
        
        return horizon_data
    
    def _extract_horizon_points_simple(self, mask, config_num):
        """
        Nueva lógica para Config 2: solo lane_mask, prioriza izquierda.
        """
        h, w = mask.shape
        num_horizons = self.get_parameter('num_horizons_2').value
        y_start = self.get_parameter('y_range_2_start').value
        y_end = self.get_parameter('y_range_2_end').value
        
        horizon_data = []
        
        for i in range(num_horizons):
            # Distribución no lineal
            ratio = (num_horizons - i - 1) / num_horizons
            y_center = int(h * (y_start + ratio * (y_end - y_start)))
            
            # Franja vertical
            slice_height = h // (num_horizons * 15)
            y1 = max(0, y_center - slice_height)
            y2 = min(h, y_center + slice_height)
            
            # SOLO lado derecho (igual que original)
            slice_mask = mask[y1:y2, w//2:w]
            
            # Extraer solo lane_mask
            edge_x, confidence = self._extract_edge_simple(slice_mask, y1, y2)
            
            if edge_x is not None:
                # Ajustar a coordenadas globales
                global_x = edge_x + w // 2
                
                horizon_data.append({
                    'horizon_idx': i,
                    'y': y_center,
                    'x': global_x,
                    'confidence': confidence,
                    'slice_bounds': (y1, y2),
                    'config_num': config_num
                })
        
        return horizon_data
    
    def _extract_edge_robust(self, slice_mask, y1, y2):
        """
        EXACTA copia de la función original.
        """
        min_points = self.get_parameter('min_points_per_horizon').value
        use_lane = self.get_parameter('use_lane_mask_1').value
        use_road = self.get_parameter('use_road_mask_1').value
        
        # Extraer datos de ambas máscaras por separado
        lane_xs, lane_ys = None, None
        road_xs, road_ys = None, None
        
        if use_lane:
            lane_ys, lane_xs = np.where(slice_mask == 2)
        
        if use_road:
            road_ys, road_xs = np.where(slice_mask == 1)
        
        # Estrategia inteligente: ¿cuál está más a la derecha?
        lane_rightmost = np.percentile(lane_xs, 85) if lane_xs is not None and len(lane_xs) > 0 else -1
        road_rightmost = np.percentile(road_xs, 85) if road_xs is not None and len(road_xs) > 0 else -1
        
        # Decidir qué máscara usar como PRIMARIA
        all_ys = []
        all_xs = []
        
        # CASO 1: Ambas máscaras tienen datos suficientes
        lane_has_enough = lane_xs is not None and len(lane_xs) >= min_points
        road_has_enough = road_xs is not None and len(road_xs) >= min_points
        
        if lane_has_enough and road_has_enough:
            # Usamos la que esté MÁS A LA DERECHA
            if lane_rightmost > road_rightmost:
                all_ys.extend(lane_ys)
                all_xs.extend(lane_xs)
            else:
                all_ys.extend(road_ys)
                all_xs.extend(road_xs)
        
        # CASO 2: Solo lane tiene datos suficientes
        elif lane_has_enough:
            all_ys.extend(lane_ys)
            all_xs.extend(lane_xs)
        
        # CASO 3: Solo road tiene datos suficientes
        elif road_has_enough:
            all_ys.extend(road_ys)
            all_xs.extend(road_xs)
        
        # CASO 4: Ninguna tiene suficientes datos individualmente, pero combinadas sí
        else:
            # Combinar ambas aunque sean escasas
            combined_ys = []
            combined_xs = []
            
            if lane_xs is not None:
                combined_ys.extend(lane_ys)
                combined_xs.extend(lane_xs)
            
            if road_xs is not None:
                combined_ys.extend(road_ys)
                combined_xs.extend(road_xs)
            
            if len(combined_xs) >= min_points:
                # Decide cuál usar basado en posición relativa
                if lane_rightmost > road_rightmost and lane_xs is not None:
                    all_ys.extend(lane_ys if len(lane_xs) > 0 else combined_ys)
                    all_xs.extend(lane_xs if len(lane_xs) > 0 else combined_xs)
                elif road_xs is not None:
                    all_ys.extend(road_ys if len(road_xs) > 0 else combined_ys)
                    all_xs.extend(road_xs if len(road_xs) > 0 else combined_xs)
                else:
                    all_ys = combined_ys
                    all_xs = combined_xs
        
        # Si no hay suficientes puntos, retornar None
        if len(all_xs) < min_points:
            return None, 0.0
        
        all_xs = np.array(all_xs)
        all_ys = np.array(all_ys)
        
        # === FILTRADO DE OUTLIERS ===
        low_pct = self.get_parameter('outlier_percentile_low').value
        high_pct = self.get_parameter('outlier_percentile_high').value
        
        x_low = np.percentile(all_xs, low_pct)
        x_high = np.percentile(all_xs, high_pct)
        
        # Filtrar puntos dentro del rango
        valid_mask = (all_xs >= x_low) & (all_xs <= x_high)
        filtered_xs = all_xs[valid_mask]
        filtered_ys = all_ys[valid_mask]
        
        if len(filtered_xs) < min_points:
            return None, 0.0
        
        # El "borde" es el percentil alto (lado derecho)
        edge_x = int(np.percentile(filtered_xs, 85))
        
        # Confianza basada en número de puntos y coherencia
        total_pixels = slice_mask.shape[0] * slice_mask.shape[1]
        point_ratio = len(filtered_xs) / total_pixels
        
        # Confianza adicional si lane y road están de acuerdo
        agreement_bonus = 0.0
        if lane_xs is not None and road_xs is not None:
            # Calcular cuán similares son sus posiciones
            if len(lane_xs) > 0 and len(road_xs) > 0:
                lane_median = np.median(lane_xs)
                road_median = np.median(road_xs)
                diff = abs(lane_median - road_median)
                # Menor diferencia = mayor acuerdo = más confianza
                if diff < 20:  # Si están a menos de 20 píxeles
                    agreement_bonus = 0.2
        
        confidence = min(1.0, point_ratio * 10 + agreement_bonus)
        
        return edge_x, confidence
    
    def _extract_edge_simple(self, slice_mask, y1, y2):
        """
        Extracción simple para Config 2: solo lane_mask, prioriza izquierda.
        """
        min_points = self.get_parameter('min_points_per_horizon').value
        
        # Solo lane mask
        lane_ys, lane_xs = np.where(slice_mask == 2)
        
        if len(lane_xs) < min_points:
            return None, 0.0
        
        # Priorizar puntos a la izquierda (percentil bajo)
        edge_x = int(np.percentile(lane_xs, 15))  # 15% = izquierda
        
        # Confianza simple
        total_pixels = slice_mask.shape[0] * slice_mask.shape[1]
        point_ratio = len(lane_xs) / total_pixels
        confidence = min(1.0, point_ratio * 10)
        
        return edge_x, confidence
    
    # =================== DETECCIÓN DE CURVAS ===================
    def _detect_curve_adaptive(self, horizon_data, config_num):
        """EXACTA copia de la función original."""
        if len(horizon_data) < 3:
            return False
        
        x_positions = [h['x'] for h in horizon_data]
        
        # Guardar historial
        if config_num == 1:
            self.horizon_x_history_1.append(x_positions)
        else:
            self.horizon_x_history_2.append(x_positions)
        
        # Calcular variación en X
        x_std = np.std(x_positions)
        x_range = np.max(x_positions) - np.min(x_positions)
        
        # Normalizar por ancho de imagen
        if len(horizon_data) > 0:
            img_width = horizon_data[0]['x'] * 2  # Aproximación
            normalized_variation = x_range / img_width
        else:
            normalized_variation = 0
        
        threshold = self.get_parameter('curve_detection_threshold').value
        is_curve = normalized_variation > threshold
        
        return is_curve
    
    # =================== CÁLCULO DE ERROR ===================
    def _calculate_predictive_error(self, horizon_data, w, is_curve, config_num):
        """
        EXACTA copia de la función original para Config 1.
        Adaptada para Config 2.
        """
        if not horizon_data:
            return None, []
        
        if config_num == 1:
            ref_x = w * self.get_parameter('ref_ratio_1').value
            weights = self.get_parameter('horizon_weights_1').value
        else:
            ref_x = w * self.get_parameter('ref_ratio_2').value
            weights = self.get_parameter('horizon_weights_2').value
        
        # === AJUSTE ADAPTATIVO EN CURVAS ===
        if is_curve:
            boost = self.get_parameter('curve_weight_boost').value
            adjusted_weights = []
            
            for i, w_orig in enumerate(weights[:len(horizon_data)]):
                if i < 2:  # Horizontes cercanos (reacción rápida)
                    adjusted_weights.append(w_orig * boost)
                else:  # Horizontes lejanos
                    adjusted_weights.append(w_orig / boost)
            
            # Normalizar para que sumen 1
            total = sum(adjusted_weights)
            weights = [w / total for w in adjusted_weights]
        else:
            weights = weights[:len(horizon_data)]
        
        # === CÁLCULO DE ERROR PONDERADO ===
        errors = []
        weighted_errors = []
        
        for h_data, weight in zip(horizon_data, weights):
            detected_x = h_data['x']
            
            # Error normalizado (IGUAL fórmula para ambas)
            error = (ref_x - detected_x) / (w / 2.0)
            
            # Ponderar por confianza del horizonte
            confidence = h_data.get('confidence', 1.0)
            effective_weight = weight * confidence
            
            errors.append(error)
            weighted_errors.append(error * effective_weight)
        
        # Error final
        total_weight = sum([w * h['confidence'] for w, h in zip(weights, horizon_data)])
        if total_weight > 0:
            weighted_error = sum(weighted_errors) / total_weight
        else:
            weighted_error = 0.0
        
        return float(np.clip(weighted_error, -1.0, 1.0)), errors
    
    # =================== FILTRADO ===================
    def _apply_filters(self, raw_error, config_num):
        """
        EXACTA copia de la función original.
        """
        # Filtro de mediana
        if self.get_parameter('use_median_filter').value:
            if config_num == 1:
                self.error_buffer_1.append(raw_error)
                buffer = self.error_buffer_1
            else:
                self.error_buffer_2.append(raw_error)
                buffer = self.error_buffer_2
            
            if len(buffer) >= 3:
                median_error = np.median(list(buffer))
            else:
                median_error = raw_error
        else:
            median_error = raw_error
        
        # EMA
        alpha = self.get_parameter('ema_alpha').value
        if config_num == 1:
            self.filtered_error_1 = (
                alpha * median_error + 
                (1 - alpha) * self.filtered_error_1
            )
            return float(np.clip(self.filtered_error_1, -1.0, 1.0))
        else:
            self.filtered_error_2 = (
                alpha * median_error + 
                (1 - alpha) * self.filtered_error_2
            )
            return float(np.clip(self.filtered_error_2, -1.0, 1.0))
    
    # =================== FALLBACK ===================
    def _get_fallback_error(self, config_num):
        """
        EXACTA copia de la función original.
        """
        threshold = self.get_parameter('fail_count_threshold').value
        decay = self.get_parameter('fallback_decay').value
        
        if config_num == 1:
            fail_count = self.fail_count_1
            last_error = self.last_valid_error_1
        else:
            fail_count = self.fail_count_2
            last_error = self.last_valid_error_2
        
        if fail_count < threshold:
            error = last_error * decay
            return error, True
        else:
            return 0.0, False
    
    # =================== VISUALIZACIÓN ===================
    def _create_debug_image(self, mask, horizon_data_1, horizon_data_2, 
                           error_1, error_2, valid_1, valid_2, 
                           is_curve_1, is_curve_2, raw_errors_1, raw_errors_2):
        """
        Crea imagen de debug con ambas configuraciones.
        """
        h, w = mask.shape
        
        # Crear imagen base (IGUAL al original)
        img = np.full((h, w, 3), 225, dtype=np.uint8)
        
        # Colorear máscara de segmentación (IGUAL)
        lane_pixels = (mask == 2)
        road_pixels = (mask == 1)
        img[lane_pixels] = [0, 100, 200]  # Naranja oscuro = carril
        img[road_pixels] = [50, 50, 50]   # Gris = carretera
        
        # Colores para cada configuración
        color_1 = (0, 255, 255)    # Amarillo - Config 1 (igual al original)
        color_2 = (255, 0, 255)    # Magenta - Config 2
        
        # === LÍNEAS DE REFERENCIA PARA AMBAS CONFIGURACIONES ===
        ref_x_1 = int(w * self.get_parameter('ref_ratio_1').value)
        ref_x_2 = int(w * self.get_parameter('ref_ratio_2').value)
        
        # Línea de referencia Config 1 (azul como en el original)
        cv2.line(img, (ref_x_1, 0), (ref_x_1, h), (255, 0, 0), 2)
        cv2.putText(img, "REF_C1", (ref_x_1 + 5, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Línea de referencia Config 2 (magenta)
        cv2.line(img, (ref_x_2, 0), (ref_x_2, h), color_2, 2)
        cv2.putText(img, "REF_C2", (ref_x_2 + 5, 45),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_2, 2)
        
        # === DIBUJAR HORIZONTES CONFIG 1 (igual al original) ===
        colors = [
            (0, 255, 255),    # Cian (cercano)
            (0, 255, 128),    # Verde-amarillo
            (0, 255, 0),      # Verde
            (128, 255, 0),    # Verde-azul
            (255, 128, 0)     # Azul (lejano)
        ]
        
        for h_data in horizon_data_1:
            idx = h_data['horizon_idx']
            color = colors[min(idx, len(colors)-1)]
            confidence = h_data.get('confidence', 1.0)
            
            # Línea horizontal del horizonte
            y = h_data['y']
            alpha = int(80 * confidence)
            cv2.line(img, (w//2, y), (w, y), (alpha, alpha, alpha), 1)
            
            # Punto detectado (IGUAL al original)
            radius = int(4 + confidence * 4)
            cv2.circle(img, (h_data['x'], y), radius, color, -1)
            cv2.circle(img, (h_data['x'], y), radius+2, (255, 255, 255), 1)
            
            # Label
            text = f"H{idx}"
            if len(raw_errors_1) > idx:
                text += f" ({raw_errors_1[idx]:+.2f})"
            cv2.putText(img, text, (h_data['x'] + 10, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
        
        # === DIBUJAR HORIZONTES CONFIG 2 ===
        for h_data in horizon_data_2:
            idx = h_data['horizon_idx']
            y = h_data['y']
            confidence = h_data.get('confidence', 1.0)
            
            # Punto detectado Config 2 (diferente marcador)
            radius = int(4 + confidence * 4)
            x, y_pt = h_data['x'], h_data['y']
            
            # Dibujar como X
            cv2.line(img, (x-radius, y_pt-radius), (x+radius, y_pt+radius), color_2, 2)
            cv2.line(img, (x-radius, y_pt+radius), (x+radius, y_pt-radius), color_2, 2)
            
            # Label
            text = f"C2-H{idx}"
            if len(raw_errors_2) > idx:
                text += f" ({raw_errors_2[idx]:+.2f})"
            cv2.putText(img, text, (x + 10, y_pt - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, color_2, 1)
        
        # === BARRA DE ERROR CONFIG 1 (igual al original) ===
        center_x = w // 2
        bar_x_1 = int(center_x + error_1 * w / 4)
        
        # Centro
        cv2.rectangle(img, (center_x - 2, h-20), (center_x + 2, h), (255,255,255), -1)
        
        # Barra coloreada según magnitud
        if abs(error_1) < 0.2:
            bar_color = (0, 255, 0)  # Verde: OK
        elif abs(error_1) < 0.5:
            bar_color = (0, 200, 255)  # Naranja: Moderado
        else:
            bar_color = (0, 0, 255)  # Rojo: Grande
        
        cv2.rectangle(img, (bar_x_1 - 5, h-35), (bar_x_1 + 5, h-10), bar_color, -1)
        
        # Flecha indicando dirección de corrección
        if error_1 > 0.05:  # Alejarse (antihorario)
            cv2.arrowedLine(img, (bar_x_1, h-45), (bar_x_1 + 15, h-45), (255, 255, 0), 2)
            dir_text_1 = "ALEJARSE"
        elif error_1 < -0.05:  # Acercarse (horario)
            cv2.arrowedLine(img, (bar_x_1, h-45), (bar_x_1 - 15, h-45), (255, 255, 0), 2)
            dir_text_1 = "ACERCARSE"
        else:
            dir_text_1 = "OK"
        
        # === INFORMACIÓN TEXTUAL MEJORADA ===
        y_text = 70  # Aumentado para que no se superponga con REF_C2
        spacing = 25
        
        # Error Config 1
        error_color_1 = (0,255,0) if abs(error_1) < 0.3 else (0,200,255) if abs(error_1) < 0.6 else (0,0,255)
        use_config_1 = self.get_parameter('use_config_1').value
        
        active_text = " (ACTIVE)" if use_config_1 else ""
        cv2.putText(img, f"Config 1 [ORIGINAL]: {error_1:+.3f} {dir_text_1}{active_text}", 
                   (20, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.6, error_color_1, 2)
        y_text += spacing
        
        # Error Config 2
        error_color_2 = (0,255,0) if abs(error_2) < 0.3 else (0,200,255) if abs(error_2) < 0.6 else (0,0,255)
        active_text = " (ACTIVE)" if not use_config_1 else ""
        
        if error_2 > 0.05:
            dir_text_2 = "ALEJARSE"
        elif error_2 < -0.05:
            dir_text_2 = "ACERCARSE"
        else:
            dir_text_2 = "OK"
            
        cv2.putText(img, f"Config 2 [NEW]: {error_2:+.3f} {dir_text_2}{active_text}", 
                   (20, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.6, error_color_2, 2)
        y_text += spacing
        
        # Valid status
        cv2.putText(img, f"Valid: C1={valid_1} | C2={valid_2}", (20, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        y_text += spacing
        
        # Curves
        cv2.putText(img, f"Curve: C1={'YES' if is_curve_1 else 'NO'} | C2={'YES' if is_curve_2 else 'NO'}", 
                   (20, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        y_text += spacing
        
        # Points
        cv2.putText(img, f"Points: C1={len(horizon_data_1)} | C2={len(horizon_data_2)}", 
                   (20, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 255), 2)
        y_text += spacing
        
        # Fallos
        if self.fail_count_1 > 0 or self.fail_count_2 > 0:
            cv2.putText(img, f"Fails: C1={self.fail_count_1} | C2={self.fail_count_2}", 
                       (20, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
            y_text += spacing
        
        # === LEYENDA ===
        legend_y = h - 60
        cv2.putText(img, "SIGN: (+)=Away (-)=Closer", (w - 250, legend_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        
        return img
    
    # =================== PUBLICACIÓN ===================
    def _publish_invalid(self):
        """Publica estado inválido."""
        self.pub_valid.publish(Bool(data=False))
        self.pub_error.publish(Float32(data=0.0))
    
    def _publish_results(self, error, valid, debug_img):
        """Publica todos los resultados."""
        self.pub_error.publish(Float32(data=error))
        self.pub_valid.publish(Bool(data=valid))
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
    
    # =================== MAIN LOOP ===================
    def timer_cb(self):
        """Loop principal."""
        if self.mask is None or self.camera_status == 2:
            self._publish_invalid()
            return
        
        h, w = self.mask.shape
        
        # Procesar Config 1 (ORIGINAL - EXACTAMENTE igual)
        horizon_data_1 = self._extract_horizon_points_original(self.mask)
        self.is_curve_1 = self._detect_curve_adaptive(horizon_data_1, 1)
        
        min_horizons = self.get_parameter('min_valid_horizons').value
        valid_1 = False
        error_1 = 0.0
        raw_errors_1 = []
        
        if len(horizon_data_1) >= min_horizons:
            raw_error_1, raw_errors_1 = self._calculate_predictive_error(
                horizon_data_1, w, self.is_curve_1, 1
            )
            
            if raw_error_1 is not None:
                error_1 = self._apply_filters(raw_error_1, 1)
                valid_1 = True
                self.last_valid_error_1 = error_1
                self.fail_count_1 = 0
        else:
            self.fail_count_1 += 1
            error_1, valid_1 = self._get_fallback_error(1)
        
        # Procesar Config 2 (NUEVA)
        horizon_data_2 = self._extract_horizon_points_simple(self.mask, 2)
        self.is_curve_2 = self._detect_curve_adaptive(horizon_data_2, 2)
        
        valid_2 = False
        error_2 = 0.0
        raw_errors_2 = []
        
        if len(horizon_data_2) >= min_horizons:
            raw_error_2, raw_errors_2 = self._calculate_predictive_error(
                horizon_data_2, w, self.is_curve_2, 2
            )
            
            if raw_error_2 is not None:
                error_2 = self._apply_filters(raw_error_2, 2)
                valid_2 = True
                self.last_valid_error_2 = error_2
                self.fail_count_2 = 0
        else:
            self.fail_count_2 += 1
            error_2, valid_2 = self._get_fallback_error(2)
        
        # Decidir qué error publicar
        use_config_1 = self.get_parameter('use_config_1').value

        if valid_1 and valid_2:
            # Ambas válidas → elegir menor magnitud
            min_error = min(abs(error_1), abs(error_2))  
            if min_error == abs(error_1):
                final_error = error_1
                final_valid = valid_1
            else:
                final_error = error_2
                final_valid = valid_2
        elif valid_1:
            # Solo Config 1 válida
            final_error = error_1
            final_valid = valid_1
        elif valid_2:
            # Solo Config 2 válida
            final_error = error_2
            final_valid = valid_2
        else:
            # Ninguna válida
            final_error = 0.0
            final_valid = False
                
        """if use_config_1:
            final_error = error_1
            final_valid = valid_1
        else:
            final_error = error_2
            final_valid = valid_2"""
        
        # Crear imagen de debug
        debug_img = self._create_debug_image(
            self.mask, horizon_data_1, horizon_data_2,
            error_1, error_2, valid_1, valid_2,
            self.is_curve_1, self.is_curve_2,
            raw_errors_1, raw_errors_2
        )
        
        # Publicar
        self._publish_results(final_error, final_valid, debug_img)
        
        # Debug window
        if self.get_parameter('show_debug').value:
            cv2.imshow("Lane Error - 2 Configurations", debug_img)
            cv2.waitKey(1)
    
    # =================== UTILITIES ===================
    def _get_level(self, msg):
        """Extrae nivel de diagnóstico de forma segura."""
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
    node = LaneErrorPredictiveRobust()
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