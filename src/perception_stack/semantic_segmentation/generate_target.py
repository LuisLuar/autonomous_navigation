#!/usr/bin/env python3
"""
LaneTargetGeneratorNode (Versión Mejorada - FIXED)
==================================================
- Target se ubica correctamente entre lane_dividing y lane_right
- No gira con el robot, mantiene posición fija en el carril
"""

import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque
from math import cos, sin, atan2
import time

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion


class KalmanFilter:
    """Filtro de Kalman simple para suavizar mediciones"""
    def __init__(self, initial_value, process_variance=0.01, measurement_variance=0.1):
        self.value = initial_value
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate_variance = 1.0
    
    def update(self, measurement):
        # Predicción
        prediction = self.value
        
        # Actualización
        kalman_gain = self.estimate_variance / (self.estimate_variance + self.measurement_variance)
        self.value = prediction + kalman_gain * (measurement - prediction)
        self.estimate_variance = (1 - kalman_gain) * self.estimate_variance + self.process_variance
        
        return self.value


class LaneTargetGeneratorNode(Node):
    def __init__(self):
        super().__init__('lane_target_generator')

        self.lookahead = 2.0
        self.frame_id = 'odom'
        
        # ===== Parámetros de suavizado =====
        self.alpha_offset = 0.2      # Suavizado para offset
        self.alpha_width = 0.1       # Suavizado para ancho
        self.min_confidence = 0.3    # Confianza mínima para usar estructura
        self.history_size = 30       # Tamaño del historial
        self.width_min = 2.5         # Ancho mínimo válido (m)
        self.width_max = 3.5         # Ancho máximo válido (m)
        
        # ===== Estado =====
        self.robot_pose = None
        self.lines = {
            'right_lane': None,      # Límite DERECHO del carril
            'lane_dividing': None,   # Límite IZQUIERDO del carril  
            'left_border': None,     # Borde izquierdo de la vía
            'right_border': None     # Borde derecho de la vía
        }

        # === MEMORIA ESTRUCTURAL DEL CARRIL ===
        self.lane_offset_lat = None      # offset lateral del CENTRO del carril
        self.lane_width = None           # ancho estimado del carril
        self.structure_confidence = 0.0  # confianza en la estructura (0-1)
        self.lane_locked = False         # TRUE cuando carril está definido
        
        # Historial para estabilización
        self.offset_history = deque(maxlen=self.history_size)
        self.width_history = deque(maxlen=self.history_size)
        
        # Historial para curvatura
        self.target_history = deque(maxlen=10)
        self.heading_history = deque(maxlen=5)
        
        # Filtros de Kalman
        self.target_filter = KalmanFilter(0.0, process_variance=0.05, measurement_variance=0.2)
        self.offset_filter = KalmanFilter(0.0, process_variance=0.02, measurement_variance=0.1)
        self.curvature_filter = KalmanFilter(0.0, process_variance=0.01, measurement_variance=0.15)
        
        # Último target calculado
        self.last_target = None

        # ===== Subs =====
        for ln in self.lines.keys():
            self.create_subscription(
                PointCloud2,
                f'/centerline/{ln}',
                lambda msg, name=ln: self.line_cb(msg, name),
                10
            )

        self.create_subscription(Odometry, '/odometry/local', self.odom_cb, 10)

        # ===== Pubs =====
        self.pub_target = self.create_publisher(Marker, '/lane/target', 10)
        
        # Nuevos publishers para el controlador
        self.pub_error_lat = self.create_publisher(Float32, '/lane/error_lateral', 10)
        self.pub_error_head = self.create_publisher(Float32, '/lane/error_heading', 10)
        self.pub_curvature = self.create_publisher(Float32, '/lane/curvature', 10)

        self.create_timer(0.1, self.timer_cb)

    # ==========================================================
    def odom_cb(self, msg):
        """Actualiza pose del robot"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_pose = np.array([p.x, p.y, yaw])

    def line_cb(self, msg, name):
        """Procesa una línea detectada"""
        pts = [(x, y) for x, y, _ in point_cloud2.read_points(msg, skip_nans=True)]
        if len(pts) >= 2:
            self.lines[name] = np.array(pts)
        else:
            self.lines[name] = None

    # ==========================================================
    def timer_cb(self):
        """Loop principal de control"""
        if self.robot_pose is None:
            return

        target = self.compute_target_enhanced()
        if target is not None:
            # 1. Publicar target visual
            self.publish_marker(target)
            
            # 2. Calcular y publicar errores para el controlador
            self.publish_control_errors(target)

    # ==========================================================
    def find_line_point_at_distance(self, line_points, robot_pose, lookahead_distance):
        """Encuentra el punto en la línea a una distancia lookahead del robot"""
        if line_points is None or len(line_points) < 2:
            return None
            
        robot_pos = robot_pose[:2]
        yaw = robot_pose[2]
        forward = np.array([cos(yaw), sin(yaw)])
        
        # Punto objetivo adelante
        target_pos = robot_pos + forward * lookahead_distance
        
        # Encontrar el punto más cercano en la línea al target_pos
        distances = np.linalg.norm(line_points - target_pos, axis=1)
        idx = np.argmin(distances)
        
        if distances[idx] < 3.0:  # Umbral máximo de 3 metros
            return line_points[idx]
        
        # Alternativa: interpolar entre puntos cercanos
        if idx > 0 and idx < len(line_points) - 1:
            # Promediar puntos adyacentes
            return (line_points[idx-1] + line_points[idx] + line_points[idx+1]) / 3.0
        
        return line_points[idx]

    # ==========================================================
    def compute_target_enhanced(self):
        """Versión corregida - target entre lane_dividing y lane_right"""
        if self.robot_pose is None:
            return None
        
        rp = self.robot_pose
        robot_pos = rp[:2]
        yaw = rp[2]
        
        forward = np.array([cos(yaw), sin(yaw)])
        left = np.array([-sin(yaw), cos(yaw)])
        
        # ===== CASO IDEAL: ambas líneas disponibles =====
        if (self.lines['lane_dividing'] is not None and 
            self.lines['right_lane'] is not None):
            
            # Encontrar puntos en ambas líneas a la distancia lookahead
            ld_point = self.find_line_point_at_distance(
                self.lines['lane_dividing'], rp, self.lookahead
            )
            rl_point = self.find_line_point_at_distance(
                self.lines['right_lane'], rp, self.lookahead
            )
            
            if ld_point is not None and rl_point is not None:
                # Calcular el PUNTO MEDIO entre las dos líneas
                target_point = (ld_point + rl_point) / 2.0 - 0.3
                
                # Actualizar ancho estimado
                current_width = np.linalg.norm(ld_point - rl_point)
                if self.width_min < current_width < self.width_max:
                    if self.lane_width is None:
                        self.lane_width = current_width
                    else:
                        self.lane_width = self.alpha_width * current_width + (1 - self.alpha_width) * self.lane_width
                
                self.structure_confidence = 1.0
                self.lane_locked = True
                
                # Aplicar filtro de Kalman
                filtered_target = np.array(self.target_filter.update(target_point))
                self.last_target = filtered_target
                return filtered_target
        
        # ===== CASO 2: Solo lane_dividing disponible =====
        elif self.lines['lane_dividing'] is not None:
            ld_point = self.find_line_point_at_distance(
                self.lines['lane_dividing'], rp, self.lookahead
            )
            
            if ld_point is not None:
                # Si tenemos ancho estimado, calcular punto medio
                if self.lane_width is not None and self.lane_width > 0:
                    # Vector normalizado perpendicular a la línea
                    line_dir = forward  # Asumir dirección hacia adelante
                    perp = np.array([-line_dir[1], line_dir[0]])  # Rotar 90 grados
                    
                    # Mover desde lane_dividing hacia la derecha (mitad del ancho)
                    target_point = ld_point - perp * (self.lane_width / 2.0) - 0.3
                else:
                    # Sin ancho estimado, usar línea - 1.5m (ancho estándar)
                    target_point = ld_point - left * 1.5 - 0.3
                
                self.structure_confidence = 0.7
                
                filtered_target = np.array(self.target_filter.update(target_point))
                self.last_target = filtered_target
                return filtered_target
        
        # ===== CASO 3: Solo right_lane disponible =====
        elif self.lines['right_lane'] is not None:
            rl_point = self.find_line_point_at_distance(
                self.lines['right_lane'], rp, self.lookahead
            )
            
            if rl_point is not None:
                # Si tenemos ancho estimado, calcular punto medio
                if self.lane_width is not None and self.lane_width > 0:
                    # Vector normalizado perpendicular a la línea
                    line_dir = forward  # Asumir dirección hacia adelante
                    perp = np.array([-line_dir[1], line_dir[0]])  # Rotar 90 grados
                    
                    # Mover desde right_lane hacia la izquierda (mitad del ancho)
                    target_point = rl_point + perp * (self.lane_width / 2.0)
                else:
                    # Sin ancho estimado, usar línea + 1.5m (ancho estándar)
                    target_point = rl_point + left * 1.5 +0.3
                
                self.structure_confidence = 0.7
                
                filtered_target = np.array(self.target_filter.update(target_point))
                self.last_target = filtered_target
                return filtered_target
        
        # ===== CASO 4: Fallback a posición predicha =====
        elif self.last_target is not None:
            # Predicción simple basada en último target
            # Asumir que el carril continúa recto
            target_point = self.last_target + forward * 0.1  # Pequeño avance
            
            self.structure_confidence *= 0.9  # Disminuir confianza
            if self.structure_confidence < 0.1:
                self.structure_confidence = 0.1
            
            self.last_target = target_point
            return target_point
        
        return None

    # ==========================================================
    def compute_control_errors(self, target):
        """Calcula los 3 errores para el controlador"""
        if self.robot_pose is None or target is None:
            return None, None, None
        
        rp = self.robot_pose
        yaw = rp[2]
        
        # Vector izquierdo del robot
        left = np.array([-sin(yaw), cos(yaw)])
        
        # 1. ERROR LATERAL (e_lat) - positivo = target a la izquierda
        dx = target[0] - rp[0]
        dy = target[1] - rp[1]
        e_lat = dx * left[0] + dy * left[1]
        
        # 2. ERROR HEADING (e_head) - diferencia de orientación
        target_yaw = atan2(dy, dx)
        e_head = self.normalize_angle(target_yaw - yaw)
        
        # 3. CURVATURA (kappa) - derivada del heading deseado
        # Guardar target actual en historial
        self.target_history.append(target)
        
        if len(self.target_history) >= 3:
            # Calcular cambio de heading usando targets anteriores
            targets = list(self.target_history)
            
            # Heading entre target n-2 y target n-1
            dx1 = targets[-2][0] - targets[-3][0]
            dy1 = targets[-2][1] - targets[-3][1]
            yaw1 = atan2(dy1, dx1)
            
            # Heading entre target n-1 y target actual
            dx2 = targets[-1][0] - targets[-2][0]
            dy2 = targets[-1][1] - targets[-2][1]
            yaw2 = atan2(dy2, dx2)
            
            # Distancia entre targets
            dist = np.sqrt(dx2*dx2 + dy2*dy2)
            
            if dist > 0.1:  # Evitar division por cero
                # Cambio de heading por distancia = curvatura
                d_yaw = self.normalize_angle(yaw2 - yaw1)
                curvature = d_yaw / dist
                
                # Filtrar curvatura (suavizar)
                self.heading_history.append(curvature)
                if len(self.heading_history) > 1:
                    curvature = np.mean(self.heading_history)
            else:
                curvature = 0.0
        else:
            curvature = 0.0
        
        # Aplicar filtro de Kalman a la curvatura
        curvature = self.curvature_filter.update(curvature)
        
        return e_lat, e_head, curvature

    # ==========================================================
    def normalize_angle(self, angle):
        """Normaliza ángulo a [-π, π]"""
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        return angle

    # ==========================================================
    def publish_control_errors(self, target):
        """Publica los 3 errores para el controlador"""
        e_lat, e_head, curvature = self.compute_control_errors(target)
        
        if e_lat is not None:
            # Publicar error lateral
            msg_lat = Float32()
            msg_lat.data = float(e_lat)
            self.pub_error_lat.publish(msg_lat)
            
            # Publicar error heading
            msg_head = Float32()
            msg_head.data = float(e_head)
            self.pub_error_head.publish(msg_head)
            
            # Publicar curvatura
            msg_curv = Float32()
            msg_curv.data = float(curvature)
            self.pub_curvature.publish(msg_curv)

    # ==========================================================
    def publish_marker(self, target):
        """Publica marker del target"""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'lane_target'
        m.id = 0
        m.type = Marker.SPHERE
        m.scale.x = m.scale.y = m.scale.z = 0.3
        
        # Color según confianza
        if self.structure_confidence > 0.7:
            m.color.r, m.color.g, m.color.b = (0.0, 1.0, 0.0)  # Verde (alta confianza)
        elif self.structure_confidence > 0.4:
            m.color.r, m.color.g, m.color.b = (1.0, 1.0, 0.0)  # Amarillo (media confianza)
        else:
            m.color.r, m.color.g, m.color.b = (1.0, 0.0, 0.0)  # Rojo (baja confianza)
        m.color.a = 1.0
        
        m.pose.position.x = float(target[0])
        m.pose.position.y = float(target[1])
        m.pose.position.z = 0.0
        
        self.pub_target.publish(m)

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = LaneTargetGeneratorNode()
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