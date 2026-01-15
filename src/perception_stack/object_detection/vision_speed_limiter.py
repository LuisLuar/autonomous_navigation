#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from custom_interfaces.msg import ObjectInfoArray
from std_msgs.msg import Float32, Bool
from diagnostic_msgs.msg import DiagnosticStatus


class VisionSpeedLimiter(Node):
    def __init__(self):
        super().__init__('vision_speed_limiter')

        # ---------------- Subscribers ----------------
        self.create_subscription(
            ObjectInfoArray,
            '/objects/fused_info',
            self.fused_objects_cb,
            10
        )
        self.create_subscription(
            DiagnosticStatus, 
            '/status/camera', 
            self.cb_camera_status, 
            10
        )

        # ---------------- Publishers ----------------
        self.pub_alpha = self.create_publisher(Float32, '/alpha/vision', 10)
        self.pub_active = self.create_publisher(Bool, '/active/vision', 10)

        # ---------------- Parameters ----------------
        self.stop_dist_person = 1.5
        self.slow_dist_person = 5.0

        self.stop_dist_vehicle = 2.0
        self.slow_dist_vehicle = 8.0

        self.stop_dist_bicycle = 1.2
        self.slow_dist_bicycle = 4.0

        self.min_quality_score = 0.3  # Mínima calidad para considerar válido
        self.min_distance_confidence = 0.8  # Confianza en medición de distancia
        
        # ---------------- State ----------------
        self.camera_status = 2  # 0: OK, 1: WARN, 2: ERROR
        self.last_fused_data = None
        self.robot_speed = 0.0

        #self.get_logger().info("Vision speed limiter READY - Using fused objects")

    # ======================================================
    def cb_camera_status(self, msg: DiagnosticStatus):
        self.camera_status = self.get_message_level(msg)
        if self.camera_status == 2:
            self.pub_active.publish(Bool(data=False))
            self.pub_alpha.publish(Float32(data=0.0))  # Parar si cámara falla
            return

    # ======================================================
    def fused_objects_cb(self, msg: ObjectInfoArray):
        """Procesar objetos fusionados con posición 3D"""
        
        # Actualizar velocidad del robot
        self.robot_speed = msg.robot_speed
        self.last_fused_data = msg
        
        # Verificar estado de cámara
        if self.camera_status == 2:
            self.pub_active.publish(Bool(data=False))
            self.pub_alpha.publish(Float32(data=0.0))
            return

        active = False
        alpha_vals = []

        # Procesar cada objeto fusionado
        for obj in msg.objects:
            # Verificar calidad mínima
            if obj.quality_score < self.min_quality_score:
                continue
            
            # Verificar que la distancia sea válida
            if not obj.distance_valid or obj.distance <= 0:
                continue
                
            # Verificar confianza en medición de distancia
            # Si la fuente es depth (1) o IPM (2), es confiable
            if obj.distance_source not in [1, 2]:
                # Si es estimación por tamaño (3), verificar confianza
                if obj.confidence < self.min_distance_confidence:
                    continue

            d = obj.distance  # Distancia en metros desde el objeto fusionado
            active = True

            # --- Determinar umbrales por clase ---
            class_name_lower = obj.class_name.lower()
            
            # Personas
            if any(term in class_name_lower for term in ['person', 'pedestrian']):
                d_stop = self.stop_dist_person
                d_slow = self.stop_dist_person + self.slow_dist_person
                
            # Bicicletas y similares
            elif any(term in class_name_lower for term in ['bicycle', 'bike', 'motorcycle']):
                d_stop = self.stop_dist_bicycle
                d_slow = self.stop_dist_bicycle + self.slow_dist_bicycle
                
            # Vehículos
            elif any(term in class_name_lower for term in ['car', 'truck', 'bus', 'vehicle']):
                d_stop = self.stop_dist_vehicle
                d_slow = self.stop_dist_vehicle + self.slow_dist_vehicle
                
            # Caso por defecto (otros objetos)
            else:
                d_stop = self.stop_dist_vehicle
                d_slow = self.stop_dist_vehicle + self.slow_dist_vehicle

            # --- Ajustar umbrales si el objeto está en movimiento ---
            if not obj.is_static:
                # Aumentar distancia de seguridad para objetos en movimiento
                if obj.is_moving_toward:
                    # Objeto acercándose - aumentar distancia de parada
                    d_stop *= 1.5
                    d_slow *= 1.3
                elif obj.is_moving_away:
                    # Objeto alejándose - reducir distancia de parada
                    d_stop *= 0.8
            
            # --- Ajustar por Tiempo hasta Colisión (TTC) ---
            if obj.time_to_collision < 5.0 and obj.is_moving_toward:
                # TTC crítico - parar más lejos
                ttc_factor = min(2.0, 5.0 / max(obj.time_to_collision, 0.1))
                d_stop *= ttc_factor
            
            # --- Verificar si está en la trayectoria ---
            if obj.is_in_path:
                # Objeto directamente en el camino - usar umbrales normales
                alpha = self.compute_alpha(d, d_stop, d_slow)
            else:
                # Objeto fuera del camino - reducir prioridad
                path_factor = 0.6  # Reducir efecto de objetos fuera del camino
                alpha = min(1.0, self.compute_alpha(d, d_stop * 1.2, d_slow * 1.2) * path_factor)
            
            alpha_vals.append(alpha)

        # Publicar estado de actividad
        self.pub_active.publish(Bool(data=active))

        if not alpha_vals:
            # No hay objetos detectados - velocidad normal
            self.pub_alpha.publish(Float32(data=1.0))
            return

        # Tomar el valor más conservador (mínimo alpha)
        alpha_final = float(min(alpha_vals))
        
        # Ajuste basado en velocidad del robot
        if self.robot_speed > 2.0:
            # Robot rápido - ser más conservador
            alpha_final *= 0.8
        elif self.robot_speed < 0.5:
            # Robot lento - ser menos conservador
            alpha_final = min(1.0, alpha_final * 1.2)
        
        # Asegurar valores en rango [0, 1]
        alpha_final = np.clip(alpha_final, 0.0, 1.0)
        
        self.pub_alpha.publish(Float32(data=alpha_final))

    # ======================================================
    def get_message_level(self, msg):
        """Extrae el nivel de un mensaje de forma segura."""
        level_raw = msg.level
        if isinstance(level_raw, bytes):
            return int.from_bytes(level_raw, byteorder='little', signed=False)
        else:
            try:
                return int(level_raw)
            except Exception:
                return 2

    @staticmethod
    def compute_alpha(d, d_stop, d_slow):
        """Calcular factor de velocidad basado en distancia"""
        if d <= d_stop:
            return 0.0  # Detener completamente
        if d >= d_slow:
            return 1.0  # Velocidad normal
        # Interpolación lineal entre d_stop y d_slow
        return (d - d_stop) / (d_slow - d_stop)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionSpeedLimiter()
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