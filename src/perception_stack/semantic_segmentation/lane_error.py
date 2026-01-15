#!/usr/bin/env python3
"""
CONTROL PREDICTIVO ROBUSTO - VERSIÓN FINAL
==========================================
Optimizado para:
- Carriles irregulares
- Sombras y cambios de iluminación
- Control suave y estable

CONVENCIÓN DE SIGNOS:
- ERROR POSITIVO (+) → Robot debe ALEJARSE del carril → Giro ANTIHORARIO (Z+)
- ERROR NEGATIVO (-) → Robot debe ACERCARSE al carril → Giro HORARIO (Z-)
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
        self.declare_parameter('num_horizons', 5)
        self.declare_parameter('ref_ratio', 0.8)  # Línea de referencia (75% = lado derecho)
        
        # Pesos para cada horizonte [cercano → lejano]
        # Cercano: más importante para reacción inmediata
        # Lejano: importante para anticipación
        self.declare_parameter('horizon_weights', [0.001, 0.004, 0.05, 0.006, 0.005])#[0.05, 0.03, 0.015, 0.004, 0.001])
        
        # =================== DETECCIÓN DE CURVAS ===================
        self.declare_parameter('curve_detection_threshold', 0.15)
        self.declare_parameter('curve_weight_boost', 1.5)
        
        # =================== ROBUSTEZ A SOMBRAS ===================
        # Usar múltiples valores de máscara para ser más robusto
        self.declare_parameter('use_lane_mask', True)      # Valor 2 (carril)
        self.declare_parameter('use_road_mask', True)      # Valor 1 (carretera)
        self.declare_parameter('min_confidence_ratio', 0.3)  # Mínimo 30% de puntos buenos
        
        # Filtrado de outliers
        self.declare_parameter('outlier_percentile_low', 5)   # Descartar 10% más bajo
        self.declare_parameter('outlier_percentile_high', 90)  # Descartar 10% más alto
        self.declare_parameter('min_points_per_horizon', 5)
        
        # =================== FILTRADO TEMPORAL ===================
        self.declare_parameter('ema_alpha', 0.2)  # Suavizado exponencial
        self.declare_parameter('use_median_filter', True)
        self.declare_parameter('median_window_size', 5)
        
        # =================== FALLBACK Y VALIDACIÓN ===================
        self.declare_parameter('fail_count_threshold', 8)  # Más tolerante
        self.declare_parameter('fallback_decay', 0.85)
        self.declare_parameter('min_valid_horizons', 2)
        
        # =================== PUBLICACIÓN ===================
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('show_debug', True)
        
        # =================== ESTADO ===================
        self.mask = None
        self.camera_status = 0
        self.filtered_error = 0.0
        self.last_valid_error = 0.0
        self.fail_count = 0
        self.is_curve = False
        
        # Buffer para filtro de mediana
        median_size = self.get_parameter('median_window_size').value
        self.error_buffer = deque(maxlen=median_size)
        
        # Historial para detección de curvas
        self.horizon_x_history = deque(maxlen=10)
        
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
        
        #self.get_logger().info("✅ Lane Error Predictive Robust READY")
        #self.get_logger().info("📐 CONVENCIÓN: (+) = Alejarse del carril, (-) = Acercarse al carril")
    
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
            #self.get_logger().error(f"Error reshaping mask: {e}")
            self.mask = None
    
    # =================== EXTRACCIÓN DE PUNTOS ROBUSTA ===================
    def _extract_horizon_points_robust(self, mask):
        """
        Extrae puntos en múltiples horizontes con robustez a sombras.
        
        ESTRATEGIA:
        1. Divide la imagen en N horizontes verticales
        2. En cada horizonte, busca el borde del carril
        3. Usa TANTO lane mask (2) COMO road mask (1) para robustez
        4. Filtra outliers usando percentiles
        5. Valida que haya suficientes puntos
        
        Returns:
            Lista de diccionarios con info de cada horizonte
        """
        h, w = mask.shape
        num_horizons = self.get_parameter('num_horizons').value
        
        horizon_data = []
        
        for i in range(num_horizons):
            # Distribución no lineal: más resolución cerca del robot
            # i=0 → bottom (más cercano, más peso)
            # i=N-1 → top (más lejano, menos peso)
            ratio = (num_horizons - i - 1) / num_horizons
            y_center = int(h * (0.7 + ratio * 0.2))  # (0.4 + ratio * 0.5)) Entre 40% y 90%
            
            # Franja vertical alrededor del horizonte
            slice_height = h // (num_horizons * 10)#2)
            y1 = max(0, y_center - slice_height)
            y2 = min(h, y_center + slice_height)
            
            # SOLO lado derecho (donde está el carril)
            slice_mask = mask[y1:y2, w//2:w]
            
            # === EXTRACCIÓN ROBUSTA ===
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
    
    def _extract_edge_robust(self, slice_mask, y1, y2):
        """
        Extrae el borde del carril de forma robusta.
        
        ESTRATEGIA MEJORADA:
        1. Compara lane_mask vs road_mask para ver cuál está más a la DERECHA
        2. Usa la máscara que esté MÁS A LA DERECHA como referencia principal
        3. Si ambas tienen datos, combina inteligentemente
        4. Fallback a la otra si la principal falla
        """
        min_points = self.get_parameter('min_points_per_horizon').value
        use_lane = self.get_parameter('use_lane_mask').value
        use_road = self.get_parameter('use_road_mask').value
        
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
                # Lane mask está más a la derecha → usar lane
                #self.get_logger().debug(f"Usando LANE (más a la derecha: {lane_rightmost:.1f} vs {road_rightmost:.1f})")
                all_ys.extend(lane_ys)
                all_xs.extend(lane_xs)
            else:
                # Road mask está más a la derecha → usar road
                #self.get_logger().debug(f"Usando ROAD (más a la derecha: {road_rightmost:.1f} vs {lane_rightmost:.1f})")
                all_ys.extend(road_ys)
                all_xs.extend(road_xs)
        
        # CASO 2: Solo lane tiene datos suficientes
        elif lane_has_enough:
            all_ys.extend(lane_ys)
            all_xs.extend(lane_xs)
            #self.get_logger().debug("Usando LANE (única con datos)")
        
        # CASO 3: Solo road tiene datos suficientes
        elif road_has_enough:
            all_ys.extend(road_ys)
            all_xs.extend(road_xs)
            #self.get_logger().debug("Usando ROAD (única con datos)")
        
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
                    # Preferir lane si está más a la derecha
                    all_ys.extend(lane_ys if len(lane_xs) > 0 else combined_ys)
                    all_xs.extend(lane_xs if len(lane_xs) > 0 else combined_xs)
                    #self.get_logger().debug("Combinado: prefiriendo LANE por posición")
                elif road_xs is not None:
                    all_ys.extend(road_ys if len(road_xs) > 0 else combined_ys)
                    all_xs.extend(road_xs if len(road_xs) > 0 else combined_xs)
                    #self.get_logger().debug("Combinado: prefiriendo ROAD")
                else:
                    all_ys = combined_ys
                    all_xs = combined_xs
                    #self.get_logger().debug("Combinado: usando ambas")
        
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
    
    # =================== DETECCIÓN DE CURVAS ===================
    def _detect_curve_adaptive(self, horizon_data):
        """
        Detecta si estamos en una curva analizando la variación entre horizontes.
        
        MÉTODO:
        - Compara posiciones X de horizontes cercanos vs lejanos
        - Si hay mucha diferencia → curva
        - Si son similares → recta
        """
        if len(horizon_data) < 3:
            return False
        
        x_positions = [h['x'] for h in horizon_data]
        
        # Guardar historial para análisis temporal
        self.horizon_x_history.append(x_positions)
        
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
    def _calculate_predictive_error(self, horizon_data, w, is_curve):
        """
        Calcula error ponderado considerando múltiples horizontes.
        
        LÓGICA DE SIGNOS:
        - ref_x está a la DERECHA (75% del ancho)
        - Si el carril detectado está más a la IZQUIERDA que ref_x:
          → error POSITIVO → robot debe ALEJARSE (girar antihorario)
        - Si el carril detectado está más a la DERECHA que ref_x:
          → error NEGATIVO → robot debe ACERCARSE (girar horario)
        
        ERROR = (ref_x - detected_x) / normalización
        
        Si detected_x < ref_x → error > 0 → alejarse
        Si detected_x > ref_x → error < 0 → acercarse
        """
        if not horizon_data:
            return None, []
        
        ref_x = w * self.get_parameter('ref_ratio').value
        weights = self.get_parameter('horizon_weights').value
        
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
            
            # Error normalizado
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
    def _apply_filters(self, raw_error):
        """
        Aplica filtros temporales para suavizar el error.
        
        1. Filtro de mediana móvil (elimina spikes)
        2. EMA (suavizado exponencial)
        """
        # Filtro de mediana
        if self.get_parameter('use_median_filter').value:
            self.error_buffer.append(raw_error)
            if len(self.error_buffer) >= 3:
                median_error = np.median(list(self.error_buffer))
            else:
                median_error = raw_error
        else:
            median_error = raw_error
        
        # EMA
        alpha = self.get_parameter('ema_alpha').value
        self.filtered_error = (
            alpha * median_error + 
            (1 - alpha) * self.filtered_error
        )
        
        return float(np.clip(self.filtered_error, -1.0, 1.0))
    
    # =================== FALLBACK ===================
    def _get_fallback_error(self):
        """
        Lógica de fallback cuando la detección falla temporalmente.
        """
        threshold = self.get_parameter('fail_count_threshold').value
        decay = self.get_parameter('fallback_decay').value
        
        if self.fail_count < threshold:
            # Usar último error válido con decaimiento gradual
            error = self.last_valid_error * decay
            return error, True
        else:
            # Demasiados fallos → detener
            return 0.0, False
    
    # =================== VISUALIZACIÓN ===================
    def _create_debug_image(self, mask, horizon_data, error, valid, is_curve, raw_errors):
        """
        Crea imagen de debug con toda la información visual.
        """
        h, w = mask.shape
        
        # Crear imagen base (máscara coloreada)
        img = np.full((h, w, 3), 225, dtype=np.uint8)
        
        # Colorear máscara de segmentación
        lane_pixels = (mask == 2)
        road_pixels = (mask == 1)
        img[lane_pixels] = [0, 100, 200]  # Naranja oscuro = carril
        img[road_pixels] = [50, 50, 50]   # Gris = carretera
        
        # === DIBUJAR HORIZONTES ===
        colors = [
            (0, 255, 255),    # Cian (cercano) - MÁS IMPORTANTE
            (0, 255, 128),    # Verde-amarillo
            (0, 255, 0),      # Verde
            (128, 255, 0),    # Verde-azul
            (255, 128, 0)     # Azul (lejano)
        ]
        
        for h_data in horizon_data:
            idx = h_data['horizon_idx']
            color = colors[min(idx, len(colors)-1)]
            confidence = h_data.get('confidence', 1.0)
            
            # Línea horizontal del horizonte
            y = h_data['y']
            alpha = int(80 * confidence)
            cv2.line(img, (w//2, y), (w, y), (alpha, alpha, alpha), 1)
            
            # Punto detectado (más grande = más confianza)
            radius = int(4 + confidence * 4)
            cv2.circle(img, (h_data['x'], y), radius, color, -1)
            cv2.circle(img, (h_data['x'], y), radius+2, (255, 255, 255), 1)
            
            # Label
            text = f"H{idx}"
            if len(raw_errors) > idx:
                text += f" ({raw_errors[idx]:+.2f})"
            cv2.putText(img, text, (h_data['x'] + 10, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
        
        # === LÍNEA DE REFERENCIA ===
        ref_x = int(w * self.get_parameter('ref_ratio').value)
        cv2.line(img, (ref_x, 0), (ref_x, h), (255, 0, 0), 2)
        cv2.putText(img, "REF", (ref_x + 5, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # === BARRA DE ERROR ===
        center_x = w // 2
        bar_x = int(center_x + error * w / 4)
        
        # Centro
        cv2.rectangle(img, (center_x - 2, h-20), (center_x + 2, h), (255,255,255), -1)
        
        # Barra coloreada según magnitud
        if abs(error) < 0.2:
            bar_color = (0, 255, 0)  # Verde: OK
        elif abs(error) < 0.5:
            bar_color = (0, 200, 255)  # Naranja: Moderado
        else:
            bar_color = (0, 0, 255)  # Rojo: Grande
        
        cv2.rectangle(img, (bar_x - 5, h-35), (bar_x + 5, h-10), bar_color, -1)
        
        # Flecha indicando dirección de corrección
        if error > 0.05:  # Alejarse (antihorario)
            cv2.arrowedLine(img, (bar_x, h-45), (bar_x + 15, h-45), (255, 255, 0), 2)
            dir_text = "ALEJARSE"
        elif error < -0.05:  # Acercarse (horario)
            cv2.arrowedLine(img, (bar_x, h-45), (bar_x - 15, h-45), (255, 255, 0), 2)
            dir_text = "ACERCARSE"
        else:
            dir_text = "OK"
        
        # === INFORMACIÓN TEXTUAL ===
        y_text = 30
        spacing = 30
        
        # Error
        error_color = (0,255,0) if abs(error) < 0.3 else (0,200,255) if abs(error) < 0.6 else (0,0,255)
        cv2.putText(img, f"Error: {error:+.3f} ({dir_text})", (20, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, error_color, 2)
        y_text += spacing
        
        # Estado
        status_color = (0,255,0) if valid else (0,0,255)
        cv2.putText(img, f"Valid: {valid} | Points: {len(horizon_data)}", (20, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        y_text += spacing
        
        # Curva
        curve_color = (255, 200, 0) if is_curve else (100,100,100)
        cv2.putText(img, f"Curve: {'YES' if is_curve else 'NO'}", (20, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, curve_color, 2)
        y_text += spacing
        
        # Fallos
        if self.fail_count > 0:
            cv2.putText(img, f"Fails: {self.fail_count}", (20, y_text),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
        
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
        
        # Validaciones
        self.get_logger
        if self.mask is None or self.camera_status == 2:
            self._publish_invalid()
            return
        
        h, w = self.mask.shape
        
        # 1. Extraer puntos en horizontes (ROBUSTO)
        horizon_data = self._extract_horizon_points_robust(self.mask)
        
        # 2. Detectar curva
        self.is_curve = self._detect_curve_adaptive(horizon_data)
        
        valid = False
        error = 0.0
        raw_errors = []
        
        min_horizons = self.get_parameter('min_valid_horizons').value
        
        if len(horizon_data) >= min_horizons:
            # 3. Calcular error predictivo
            raw_error, raw_errors = self._calculate_predictive_error(
                horizon_data, w, self.is_curve
            )
            
            if raw_error is not None:
                # 4. Aplicar filtros
                error = self._apply_filters(raw_error)
                valid = True
                
                self.last_valid_error = error
                self.fail_count = 0
        else:
            # Fallback
            self.fail_count += 1
            error, valid = self._get_fallback_error()
        
        # 5. Visualizar
        debug_img = self._create_debug_image(
            self.mask, horizon_data, error, valid, self.is_curve, raw_errors
        )
        
        # 6. Publicar
        self._publish_results(error, valid, debug_img)
        
        # 7. Debug window (opcional)
        if self.get_parameter('show_debug').value:
            cv2.imshow("Lane Error Predictive Robust", debug_img)
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
            return 2  # Error por defecto
    
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
            print(f"Shutdown error: {e}")


if __name__ == '__main__':
    main()