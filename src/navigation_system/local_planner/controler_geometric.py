#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry, Path
import numpy as np
import math
import time
from collections import deque
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path



class GeometricPathController(Node):

    def __init__(self):
        super().__init__('geometric_path_controller')
        # ===== PATH FOLLOWING =====
        self.create_subscription(Path, '/global_path', self.path_cb, 10)

        self.global_path = []
        self.path_received = False

        self.declare_parameter('k_y', 1.2)
        self.declare_parameter('k_psi', 2.0)
        self.declare_parameter('lookahead_distance', 0.8)

        

    # =================== CALLBACKS ===================
    
    def path_cb(self, msg: Path):
        """Procesa el camino global y precalcula todo."""
        if len(msg.poses) < 2:
            self.get_logger().warn("Camino demasiado corto")
            return
            
        self.get_logger().info(f"📐 Nuevo camino recibido: {len(msg.poses)} puntos")
        
        # Extraer poses
        self.path_poses = []
        self.path_lengths = [0.0]
        cumulative_length = 0.0
        
        # Primer punto
        p0 = msg.poses[0].pose.position
        yaw0 = self._get_yaw_from_pose(msg.poses[0].pose)
        self.path_poses.append([p0.x, p0.y, yaw0, 0.0])
        
        # Procesar todos los puntos
        for i in range(1, len(msg.poses)):
            p_prev = msg.poses[i-1].pose.position
            p_curr = msg.poses[i].pose.position
            
            # Distancia entre puntos
            dx = p_curr.x - p_prev.x
            dy = p_curr.y - p_prev.y
            segment_length = math.sqrt(dx*dx + dy*dy)
            cumulative_length += segment_length
            
            # Yaw del segmento (tangente)
            yaw = math.atan2(dy, dx)
            
            # Calcular curvatura (discreta)
            if i >= 2:
                p_prev2 = msg.poses[i-2].pose.position
                curvature = self._calculate_curvature(
                    [p_prev2.x, p_prev2.y],
                    [p_prev.x, p_prev.y],
                    [p_curr.x, p_curr.y]
                )
            else:
                curvature = 0.0
                
            self.path_poses.append([p_curr.x, p_curr.y, yaw, curvature])
            self.path_lengths.append(cumulative_length)
        
        self.total_path_length = cumulative_length
        self.closest_idx = 0
        self.geometric_active = True
        
        self.get_logger().info(f"✅ Camino procesado: {self.total_path_length:.2f}m")

    def odom_cb(self, msg: Odometry):
        """Actualiza pose y velocidad actual del robot."""
        # Posición
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Orientación (yaw)
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.current_pose = [x, y, yaw]
        
        # Velocidad
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_velocity = math.sqrt(vx*vx + vy*vy)

    def lane_cb(self, msg):
        self.omega_lane = msg.data

    def lane_orientation_cb(self, msg):
        self.omega_lane_orientation = msg.data

    def lane_active_cb(self, msg):
        self.active_lane = msg.data

    def emergency_cb(self, msg):
        self.emergency = msg.data

    def manual_cb(self, msg):
        self.manual = msg.data

    # =================== CÁLCULOS GEOMÉTRICOS ===================
    
    def _get_yaw_from_pose(self, pose: Pose):
        """Extrae yaw de un mensaje Pose."""
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw
    
    def _calculate_curvature(self, p1, p2, p3):
        """Calcula curvatura discreta de 3 puntos."""
        # Vectores
        v1 = np.array(p2) - np.array(p1)
        v2 = np.array(p3) - np.array(p2)
        
        # Longitudes
        l1 = np.linalg.norm(v1)
        l2 = np.linalg.norm(v2)
        
        if l1 < 1e-6 or l2 < 1e-6:
            return 0.0
            
        # Ángulo entre vectores
        cos_angle = np.dot(v1, v2) / (l1 * l2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = math.acos(cos_angle)
        
        # Signo de la curvatura (usando producto cruz)
        cross = np.cross(v1, v2)
        sign = 1.0 if cross >= 0 else -1.0
        
        # Curvatura = cambio de ángulo / longitud
        avg_length = (l1 + l2) / 2.0
        if avg_length < 1e-6:
            return 0.0
            
        curvature = sign * angle / avg_length
        return curvature
    
    def _find_closest_point(self, robot_pose):
        """Encuentra el punto más cercano en el camino."""
        if not self.path_poses or robot_pose is None:
            return 0, 0.0, 0.0, 0.0
            
        rx, ry, _ = robot_pose
        
        # Buscar alrededor del último índice conocido (optimización)
        search_range = min(20, len(self.path_poses) - self.closest_idx - 1)
        start_idx = max(0, self.closest_idx - 5)
        end_idx = min(len(self.path_poses), self.closest_idx + search_range)
        
        min_dist = float('inf')
        closest_idx = self.closest_idx
        
        for i in range(start_idx, end_idx):
            px, py, _, _ = self.path_poses[i]
            dist = math.sqrt((rx-px)**2 + (ry-py)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        self.closest_idx = closest_idx
        
        # Obtener datos del punto más cercano
        px, py, pyaw, pcurv = self.path_poses[closest_idx]
        
        return closest_idx, min_dist, pyaw, pcurv
    
    def _calculate_lateral_error(self, robot_pose, path_idx):
        """Calcula error lateral perpendicular al camino."""
        if path_idx >= len(self.path_poses) - 1:
            return 0.0
            
        rx, ry, _ = robot_pose
        
        # Puntos del segmento
        x1, y1, yaw1, _ = self.path_poses[path_idx]
        x2, y2, yaw2, _ = self.path_poses[path_idx + 1]
        
        # Vector del segmento
        segment_vec = np.array([x2 - x1, y2 - y1])
        segment_length = np.linalg.norm(segment_vec)
        
        if segment_length < 1e-6:
            return 0.0
            
        # Vector del punto inicial al robot
        robot_vec = np.array([rx - x1, ry - y1])
        
        # Proyección escalar
        t = np.dot(robot_vec, segment_vec) / (segment_length**2)
        t = np.clip(t, 0.0, 1.0)
        
        # Punto más cercano en el segmento
        closest_x = x1 + t * (x2 - x1)
        closest_y = y1 + t * (y2 - y1)
        
        # Vector error (del punto más cercano al robot)
        error_vec = np.array([rx - closest_x, ry - closest_y])
        
        # Orientación del segmento (tangente)
        segment_yaw = math.atan2(segment_vec[1], segment_vec[0])
        
        # Componente lateral (perpendicular al camino)
        # Rotar error_vec por -segment_yaw
        cos_yaw = math.cos(-segment_yaw)
        sin_yaw = math.sin(-segment_yaw)
        
        rotated_x = error_vec[0] * cos_yaw - error_vec[1] * sin_yaw
        rotated_y = error_vec[0] * sin_yaw + error_vec[1] * cos_yaw
        
        # El componente Y es el error lateral
        lateral_error = rotated_y
        
        return lateral_error
    
    def _get_lookahead_point(self, distance):
        """Obtiene punto a cierta distancia adelante en el camino."""
        if self.closest_idx >= len(self.path_poses) - 1:
            return self.path_poses[-1] if self.path_poses else None
        
        # Buscar punto a distancia 'lookahead' desde el punto más cercano
        target_distance = self.path_lengths[self.closest_idx] + distance
        target_idx = self.closest_idx
        
        # Avanzar hasta alcanzar la distancia
        while (target_idx < len(self.path_lengths) - 1 and 
               self.path_lengths[target_idx] < target_distance):
            target_idx += 1
        
        if target_idx >= len(self.path_poses):
            target_idx = len(self.path_poses) - 1
        
        return self.path_poses[target_idx]

    # =================== CONTROL STANLEY ===================
    
    def calculate_stanley_control(self):
        """Calcula comando de control usando Stanley Controller."""
        if not self.geometric_active or not self.current_pose:
            return 0.0, 0.0, False
        
        rx, ry, ryaw = self.current_pose
        v = max(self.current_velocity, self.get_parameter('min_speed').value)
        
        # 1. Encontrar punto más cercano
        closest_idx, dist, pyaw, pcurv = self._find_closest_point(self.current_pose)
        
        # 2. Calcular error lateral
        e_y = self._calculate_lateral_error(self.current_pose, closest_idx)
        self.lateral_error = e_y
        
        # 3. Calcular error de orientación respecto al camino
        e_psi = self._normalize_angle(ryaw - pyaw)
        
        # 4. Obtener punto lookahead
        lookahead_dist = self.get_parameter('lookahead_distance').value
        lookahead_point = self._get_lookahead_point(lookahead_dist)
        
        if lookahead_point:
            lx, ly, lyaw, lcurv = lookahead_point
            self.lookahead_point = [lx, ly]
            self.current_curvature = lcurv
        else:
            lyaw = pyaw
            self.current_curvature = pcurv
        
        # 5. Fórmula Stanley mejorada
        K_y = self.get_parameter('Kp_y').value
        K_psi = self.get_parameter('Kp_psi').value
        k_soft = self.get_parameter('softening_k').value
        
        # Término lateral (suavizado a baja velocidad)
        lateral_term = math.atan2(K_y * e_y, k_soft + v)
        
        # Término de orientación
        orientation_term = K_psi * e_psi
        
        # Término de curvatura (feedforward)
        curvature_term = self.current_curvature * lookahead_dist
        
        # Control total
        delta = lateral_term + orientation_term + curvature_term
        
        # Limitar
        delta = np.clip(delta, 
                       -self.get_parameter('max_steer').value,
                       self.get_parameter('max_steer').value)
        
        # 6. Velocidad adaptativa
        cruise_speed = self.get_parameter('cruise_speed').value
        k_curve = self.get_parameter('curvature_speed_gain').value
        k_lat = self.get_parameter('lateral_error_speed_gain').value
        
        # Reducción por curvatura
        speed_curve = 1.0 / (1.0 + k_curve * abs(self.current_curvature))
        
        # Reducción por error lateral
        speed_lat = 1.0 / (1.0 + k_lat * abs(e_y))
        
        # Velocidad deseada
        v_des = cruise_speed * min(speed_curve, speed_lat)
        v_des = max(v_des, self.get_parameter('min_speed').value)
        
        return delta, v_des, True

    # =================== FUSIÓN CON VISIÓN ===================
    
    def fuse_with_vision(self, w_geometric, v_geometric):
        """Fusiona control geométrico con control por visión."""
        alpha = self.get_parameter('fusion_alpha').value
        
        # Si no hay visión activa, usar solo control geométrico
        if not self.get_parameter('use_vision_when_active').value or not self.active_lane:
            return w_geometric, v_geometric
        
        # Convertir error de visión a comando angular aproximado
        # omega_lane es error lateral en píxeles, escalar a rad/s
        K_vision = 0.3  # Ganancia empírica
        w_vision = K_vision * self.omega_lane
        
        # omega_lane_orientation es error de orientación ya en rad
        w_orientation = self.omega_lane_orientation
        
        # Comando de visión combinado
        w_vision_total = w_vision + w_orientation
        
        # Fusionar: mezcla ponderada
        w_fused = alpha * w_geometric + (1.0 - alpha) * w_vision_total
        
        # Velocidad: usar la menor entre visión y camino
        v_min = min(v_geometric, self.get_parameter('cruise_speed').value * 0.8)
        v_fused = v_min if self.active_lane else v_geometric
        
        return w_fused, v_fused

    # =================== CONTROL LOOP ===================
    
    def control_loop(self):
        """Loop principal de control."""
        # Validaciones
        if self.manual or self.emergency:
            self._publish_stop()
            return
        
        # Calcular control geométrico
        w_geo, v_geo, geo_valid = self.calculate_stanley_control()
        
        if not geo_valid:
            self.geometric_active = False
            self._publish_stop()
            return
        
        # Fusionar con visión si está disponible
        w_cmd, v_cmd = self.fuse_with_vision(w_geo, v_geo)
        
        # Aplicar límites
        w_max = self.get_parameter('max_steer').value
        w_cmd = np.clip(w_cmd, -w_max, w_max)
        
        # Publicar comando
        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        
        self.pub_cmd.publish(cmd)
        
        # Publicar estado
        active_msg = Bool(data=self.geometric_active)
        self.pub_mode.publish(active_msg)
        
        # Debug
        self._publish_debug()
        
        # Log periódico
        self._log_status(v_cmd, w_cmd)

    # =================== UTILIDADES ===================
    
    def _normalize_angle(self, angle):
        """Normaliza ángulo a [-π, π]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def _publish_stop(self):
        """Publica comando de parada."""
        cmd = Twist()
        self.pub_cmd.publish(cmd)
        
        active_msg = Bool(data=False)
        self.pub_mode.publish(active_msg)
    
    def _publish_debug(self):
        """Publica información de debug."""
        debug_msg = Float32()
        debug_msg.data = self.lateral_error
        self.pub_debug.publish(debug_msg)
    
    def _log_status(self, v, w):
        """Log del estado actual."""
        self.get_logger().info(
            f"🛣️  Geometric: v={v:.2f}m/s, w={w:.3f}rad/s | "
            f"Lateral error: {self.lateral_error:.3f}m | "
            f"Vision active: {self.active_lane} | "
            f"Closest idx: {self.closest_idx}/{len(self.path_poses)}"
        )

    def destroy_node(self):
        """Cleanup."""
        self.get_logger().info("🛑 Geometric Path Controller shutting down")
        super().destroy_node()


def main():
    rclpy.init()
    node = GeometricPathController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()