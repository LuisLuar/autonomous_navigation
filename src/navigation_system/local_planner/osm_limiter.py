#!/usr/bin/env python3
"""
Nodo de Control de Velocidad por Obstáculos OSM
================================================
Suscribe a /osm/nearby_features y controla la velocidad según:
- Reductores de velocidad (traffic_calming)
- Cruces peatonales (crossing)
- Intersecciones
"""

import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque

# Mensajes personalizados
try:
    from custom_interfaces.msg import NearbyOSMElements, OSMElement
    CUSTOM_INTERFACE_AVAILABLE = True
except ImportError as e:
    print(f"ADVERTENCIA: No se pudo importar custom_interfaces: {e}")
    CUSTOM_INTERFACE_AVAILABLE = False

# Mensajes ROS estándar
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry


class OSMObstacleSpeedLimiter(Node):
    def __init__(self):
        super().__init__('osm_obstacle_speed_limiter')
        
        # ---------------- Parámetros Configurables ----------------
        self.declare_parameters(namespace='', parameters=[
            # Distancias de activación
            ('traffic_calming_activation_dist', 8.0),    # Distancia para detectar reductores
            ('crossing_activation_dist', 10.0),          # Distancia para detectar cruces
            
            # Distancias de frenado
            ('traffic_calming_stop_dist', 1.5),          # Detener completamente a esta distancia
            ('traffic_calming_slow_dist', 5.0),          # Reducir velocidad desde aquí
            
            ('crossing_stop_dist', 2.0),                 # Detener en cruce peatonal
            ('crossing_slow_dist', 6.0),                 # Reducir velocidad antes del cruce
            
            # Factores de velocidad
            ('traffic_calming_max_alpha', 0.5),          # Máxima velocidad sobre reductor (%)
            ('crossing_max_alpha', 0.4),                 # Máxima velocidad en cruce (%)
            
            # Tiempo de permanencia
            ('feature_cooldown_time', 3.0),              # Segundos para considerar mismo obstáculo
            ('min_confidence', 0.6),                     # Confianza mínima en detección
            
            # Historial
            ('history_size', 10),                        # Tamaño de historial para suavizado
            
            # Umbral para considerar activo
            ('min_features_for_active', 1),              # Mínimo de características para activar
        ])
        
        # ---------------- Obtener Parámetros ----------------
        self.traffic_calming_activation_dist = self.get_parameter('traffic_calming_activation_dist').value
        self.crossing_activation_dist = self.get_parameter('crossing_activation_dist').value
        
        self.traffic_calming_stop_dist = self.get_parameter('traffic_calming_stop_dist').value
        self.traffic_calming_slow_dist = self.get_parameter('traffic_calming_slow_dist').value
        
        self.crossing_stop_dist = self.get_parameter('crossing_stop_dist').value
        self.crossing_slow_dist = self.get_parameter('crossing_slow_dist').value
        
        self.traffic_calming_max_alpha = self.get_parameter('traffic_calming_max_alpha').value
        self.crossing_max_alpha = self.get_parameter('crossing_max_alpha').value
        
        self.feature_cooldown_time = self.get_parameter('feature_cooldown_time').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.history_size = self.get_parameter('history_size').value
        self.min_features_for_active = self.get_parameter('min_features_for_active').value
        
        # ---------------- Variables de Estado ----------------
        self.current_alpha = 1.0  # Factor de velocidad actual (1.0 = velocidad normal)
        self.is_active = False    # Si hay obstáculos OSM activos
        
        self.robot_speed = 0.0    # Velocidad actual del robot
        self.robot_pose = None    # Pose del robot
        
        # Historial para suavizado
        self.alpha_history = deque(maxlen=self.history_size)
        self.active_history = deque(maxlen=self.history_size)
        
        # Registro de características procesadas
        self.processed_features = {}  # feature_id -> timestamp
        
        # ---------------- Suscriptores ----------------
        if CUSTOM_INTERFACE_AVAILABLE:
            self.create_subscription(
                NearbyOSMElements,
                '/osm/nearby_features',
                self.osm_features_callback,
                10
            )
        #else:
            #self.get_logger().warn('No se puede suscribir a /osm/nearby_features - interfaz no disponible')
        
        # Suscribir a odometría para velocidad del robot
        self.create_subscription(
            Odometry,
            '/odometry/local',
            self.odometry_callback,
            10
        )
        
        # ---------------- Publicadores ----------------
        self.pub_alpha = self.create_publisher(Float32, '/alpha/osm_obstacles', 10)
        self.pub_active = self.create_publisher(Bool, '/active/osm_obstacles', 10)
        
        # ---------------- Timer para publicación periódica ----------------
        self.publish_timer = self.create_timer(0.1, self.publish_status)  # 10 Hz
        
        #self.get_logger().info('OSM Obstacle Speed Limiter inicializado')
    
    # ======================================================
    def odometry_callback(self, msg):
        """Actualizar velocidad del robot"""
        # Calcular velocidad lineal del robot
        linear_vel = msg.twist.twist.linear
        self.robot_speed = np.sqrt(linear_vel.x**2 + linear_vel.y**2)
        
        # Guardar pose actual
        self.robot_pose = msg.pose.pose
    
    # ======================================================
    def osm_features_callback(self, msg):
        """Procesar características OSM cercanas"""
        if not CUSTOM_INTERFACE_AVAILABLE or len(msg.nearby_elements) == 0:
            self.is_active = False
            self.current_alpha = 1.0
            return
        
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        active_features_count = 0
        alphas = []
        
        # Procesar cada característica OSM
        for element in msg.nearby_elements:
            # Verificar que la característica sea válida
            if not self.is_valid_feature(element):
                continue
            
            # Generar ID único para esta característica
            feature_id = f"{element.osm_id}_{int(element.longitudinal_distance)}"
            
            # Verificar cooldown (evitar procesar misma característica repetidamente)
            if feature_id in self.processed_features:
                last_time = self.processed_features[feature_id]
                if current_time - last_time < self.feature_cooldown_time:
                    continue
            
            # Marcar como procesada ahora
            self.processed_features[feature_id] = current_time
            
            # Calcular factor de velocidad para esta característica
            feature_alpha = self.calculate_feature_alpha(element)
            
            if feature_alpha < 1.0:  # Solo considerar si afecta la velocidad
                active_features_count += 1
                alphas.append(feature_alpha)
        
        # Limpiar características antiguas
        self.cleanup_old_features(current_time)
        
        # Determinar si hay obstáculos activos
        if active_features_count >= self.min_features_for_active:
            self.is_active = True
            if alphas:
                # Tomar el valor más conservador (menor alpha)
                self.current_alpha = min(alphas)
            else:
                self.current_alpha = 1.0
        else:
            self.is_active = False
            self.current_alpha = 1.0
        
        # Añadir al historial para suavizado
        self.alpha_history.append(self.current_alpha)
        self.active_history.append(self.is_active)
    
    # ======================================================
    def is_valid_feature(self, element):
        """Verificar si la característica es válida para procesar"""
        # Verificar tipo de característica
        if not hasattr(element, 'feature_type'):
            return False
        
        # Solo procesar reductores y cruces
        if element.feature_type not in [OSMElement.FEATURE_TRAFFIC_CALMING, 
                                       OSMElement.FEATURE_CROSSING]:
            return False
        
        # Verificar que esté adelante del robot
        if not hasattr(element, 'is_ahead') or not element.is_ahead:
            return False
        
        # Verificar distancia longitudinal
        if not hasattr(element, 'longitudinal_distance'):
            return False
        
        # Verificar distancia de activación según tipo
        if element.feature_type == OSMElement.FEATURE_TRAFFIC_CALMING:
            if element.longitudinal_distance > self.traffic_calming_activation_dist:
                return False
        elif element.feature_type == OSMElement.FEATURE_CROSSING:
            if element.longitudinal_distance > self.crossing_activation_dist:
                return False
        
        return True
    
    # ======================================================
    def calculate_feature_alpha(self, element):
        """
        Calcular factor de velocidad (alpha) para una característica OSM.
        alpha = 0.0 -> detener completamente
        alpha = 1.0 -> velocidad normal
        """
        distance = element.longitudinal_distance
        
        if element.feature_type == OSMElement.FEATURE_TRAFFIC_CALMING:
            # Reductor de velocidad
            if distance <= self.traffic_calming_stop_dist:
                # Detener completamente si está muy cerca
                return 0.0
            elif distance >= self.traffic_calming_slow_dist:
                # Reducción máxima permitida
                return self.traffic_calming_max_alpha
            else:
                # Interpolación lineal entre stop_dist y slow_dist
                t = (distance - self.traffic_calming_stop_dist) / \
                    (self.traffic_calming_slow_dist - self.traffic_calming_stop_dist)
                alpha = self.traffic_calming_max_alpha * t
                # Asegurar mínimo
                return max(0.0, min(self.traffic_calming_max_alpha, alpha))
        
        elif element.feature_type == OSMElement.FEATURE_CROSSING:
            # Paso peatonal
            if distance <= self.crossing_stop_dist:
                # Detener en el cruce
                return 0.0
            elif distance >= self.crossing_slow_dist:
                # Reducción máxima antes del cruce
                return self.crossing_max_alpha
            else:
                # Interpolación lineal entre stop_dist y slow_dist
                t = (distance - self.crossing_stop_dist) / \
                    (self.crossing_slow_dist - self.crossing_stop_dist)
                alpha = self.crossing_max_alpha * t
                # Asegurar mínimo
                return max(0.0, min(self.crossing_max_alpha, alpha))
        
        # Característica desconocida - no afectar velocidad
        return 1.0
    
    # ======================================================
    def cleanup_old_features(self, current_time):
        """Limpiar características procesadas antiguas"""
        to_remove = []
        for feature_id, timestamp in self.processed_features.items():
            if current_time - timestamp > self.feature_cooldown_time * 2:
                to_remove.append(feature_id)
        
        for feature_id in to_remove:
            del self.processed_features[feature_id]
    
    # ======================================================
    def publish_status(self):
        """Publicar estado actual"""
        # Aplicar suavizado con historial
        smoothed_alpha = 1.0
        smoothed_active = False
        
        if self.alpha_history:
            smoothed_alpha = np.mean(list(self.alpha_history))
        
        if self.active_history:
            # Considerar activo si la mayoría de las últimas lecturas lo indican
            active_count = sum(1 for active in self.active_history if active)
            smoothed_active = (active_count / len(self.active_history)) > 0.5
        
        # Publicar alpha (asegurar rango [0, 1])
        alpha_msg = Float32()
        alpha_msg.data = float(np.clip(smoothed_alpha, 0.0, 1.0))
        self.pub_alpha.publish(alpha_msg)
        
        # Publicar estado activo
        active_msg = Bool()
        active_msg.data = bool(smoothed_active)
        self.pub_active.publish(active_msg)
        
        # Log informativo (reducir frecuencia)
        """if self.get_clock().now().nanoseconds % 20 == 0:
            if smoothed_active:
                self.get_logger().info(
                    f'OSM Obstacles: ACTIVO | Alpha: {alpha_msg.data:.2f} | '
                    f'Robot speed: {self.robot_speed:.2f} m/s'
                )
            elif len(self.alpha_history) > 0:
                self.get_logger().debug(
                    f'OSM Obstacles: INACTIVO | Alpha: {alpha_msg.data:.2f}'
                )"""
    
    # ======================================================
    def destroy_node(self):
        """Limpieza al destruir nodo"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OSMObstacleSpeedLimiter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info('Nodo OSM Obstacle Speed Limiter detenido')
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()