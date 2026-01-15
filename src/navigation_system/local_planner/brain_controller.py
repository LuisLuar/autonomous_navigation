#!/usr/bin/env python3
"""
Brain Controller - Nodo principal de control de navegación
"""

import rclpy
from rclpy.node import Node
import math
import numpy as np
from collections import deque
from enum import Enum
import time

# Mensajes ROS
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticStatus
from custom_interfaces.msg import ObjectInfoArray, ObjectInfo, NearbyOSMElements, OSMElement

# =================== ENUMS DE ESTADOS ===================
class RobotState(Enum):
    IDLE = 0
    RUNNING = 1
    SLOW = 2
    STOPPED = 3
    EMERGENCY = 4
    GOAL_REACHED = 5

class ObstacleType(Enum):
    NONE = 0
    PEDESTRIAN = 1
    VEHICLE = 2
    BICYCLE = 3

# =================== CLASE PRINCIPAL ===================
class BrainController(Node):
    def __init__(self):
        super().__init__('brain_controller')
        
        # ============ PARÁMETROS ============
        self.declare_parameters(namespace='', parameters=[
            ('max_linear_speed', 1.0),
            ('min_linear_speed', 0.0),
            ('max_angular_speed', 0.5),
            ('slow_speed_linear', 0.4),
            ('slow_speed_angular', 0.0),
            ('max_linear_accel', 0.05),  # AUMENTADO: Aceleración más agresiva
            ('max_linear_decel', 0.1),  # AUMENTADO: Desaceleración más agresiva
            ('control_frequency', 20.0),
            ('min_pedestrian_distance', 2.5),
            ('min_vehicle_distance', 3.0),
            ('min_bicycle_distance', 2.0),
            ('crosswalk_slow_distance', 5.0),
            ('crosswalk_stop_distance', 5.0),
            ('speedbump_slow_distance', 8.0),
            ('crosswalk_wait_time', 3.0),
            ('stop_timeout', 5.0),
            ('obstacle_confirmation_time', 1.0),
            ('object_timeout', 2.0),
            ('lane_width', 3.5),
            # En la lista de parámetros, añade:
            ('max_angular_accel', 1.0),  # Aceleración angular máxima (rad/s²)
            ('max_angular_decel', 1.0),  # Desaceleración angular máxima (rad/s²)

        ])

        # Añade esta variable junto con las otras de control:
        self.current_cmd_angular_speed = 0.0  # Velocidad angular del último comando        
        # ============ VARIABLES DE ESTADO ============
        self.current_state = RobotState.IDLE
        self.last_state = RobotState.IDLE
        self.obstacle_detected = ObstacleType.NONE
        self.obstacle_distance = float('inf')
        
        # OSM
        self.near_crosswalk = False
        self.near_speedbump = False
        self.crosswalk_has_pedestrian = False
        self.crosswalk_element = None
        
        # Control - VELOCIDADES DE COMANDO (no de odometría)
        self.target_linear_speed = 0.0
        self.current_cmd_linear_speed = 0.0  # Velocidad del último comando enviado
        self.target_angular_speed = 0.0
        self.last_odom_time = time.time()
        
        # Tiempos
        self.state_start_time = time.time()
        self.crosswalk_wait_start = None
        self.last_object_time = time.time()
        self.last_osm_time = time.time()
        self.last_debug_print = time.time()
        
        # Datos
        self.robot_velocity = 0.0  # Velocidad REAL del EKF (solo para info)
        self.robot_velocity_x = 0.0
        self.robot_velocity_y = 0.0
        self.robot_position = None
        self.robot_yaw = 0.0
        self.objects_data = None
        self.osm_data = None
        
        # Flags
        self.emergency_stop = False
        self.lid_closed = True
        self.goal_reached = False
        self.system_error = False
        self.system_warning = False
        self.has_received_odom = False
        
        # Historial
        self.speed_history = deque(maxlen=10)
 
        # Para debug
        self.cycle_count = 0
        self.last_cmd_vel_time = time.time()
        
        # ============ SUSCRIPTORES ============
        self.create_subscription(
            Float32, '/pure_pursuit/angular_z',
            self.pure_pursuit_callback, 10
        )

        
        # Estado del sistema
        self.create_subscription(
            Bool, '/goal_reached',
            self.goal_reached_callback, 10
        )
        
        self.create_subscription(
            Bool, '/emergency',
            self.emergency_callback, 10
        )
        
        self.create_subscription(
            Bool, '/lid_closed',
            self.lid_callback, 10
        )
        
        # Diagnóstico del sistema (opcional)
        self.create_subscription(
            DiagnosticStatus, '/diagnostics',
            self.diagnostics_callback, 10
        )
        
        # Datos de percepción
        self.create_subscription(
            ObjectInfoArray, '/objects/fused_info',
            self.objects_callback, 10
        )
        
        # Elementos OSM
        self.create_subscription(
            NearbyOSMElements, '/osm/nearby_features',
            self.osm_callback, 10
        )
        
        # Odometría del EKF (solo para información, NO para control de aceleración)
        self.create_subscription(
            Odometry, '/odometry/global',
            self.odometry_callback, 10
        )
        
        # ============ PUBLICADORES ============
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel2', 10
        )
        
        # Publicador para diagnóstico
        self.control_state_pub = self.create_publisher(
            Bool, '/brain_controller/active', 10
        )
        
        # ============ TIMERS ============
        control_rate = self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(1.0/control_rate, self.control_cycle)
        
        # Timer para limpieza
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_old_data)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Brain Controller inicializado")
        self.get_logger().info(f"Velocidad máxima: {self.get_parameter('max_linear_speed').value} m/s")
        self.get_logger().info(f"Aceleración: {self.get_parameter('max_linear_accel').value} m/s²")
        self.get_logger().info("=" * 60)
    
    # ============ CALLBACKS ============
    def apply_angular_acceleration_limit(self, target_angular_speed):
        """Aplica límite de aceleración angular"""
        max_accel = self.get_parameter('max_angular_accel').value
        max_decel = self.get_parameter('max_angular_decel').value
        dt = 1.0 / self.get_parameter('control_frequency').value
        max_angular = self.get_parameter('max_angular_speed').value
        
        # Calcular cambio máximo permitido
        if abs(target_angular_speed) > abs(self.current_cmd_angular_speed):
            # Acelerando (en magnitud)
            max_change = max_accel * dt
        else:
            # Desacelerando o cambiando dirección
            max_change = max_decel * dt
        
        # Limitar cambio
        angular_diff = target_angular_speed - self.current_cmd_angular_speed
        
        if abs(angular_diff) > max_change:
            if angular_diff > 0:
                new_angular = self.current_cmd_angular_speed + max_change
            else:
                new_angular = self.current_cmd_angular_speed - max_change
        else:
            new_angular = target_angular_speed
        
        # Saturación final
        return max(-max_angular, min(max_angular, new_angular))
    
    def pure_pursuit_callback(self, msg):
        self.target_angular_speed = msg.data

    
    def goal_reached_callback(self, msg):
        """Meta alcanzada"""
        if msg.data and not self.goal_reached:
            self.get_logger().info("¡Meta alcanzada!")
            self.goal_reached = True
            self.publish_command(0.0, 0.0)
            self.current_cmd_linear_speed = 0.0
        elif not msg.data and self.goal_reached:
            self.goal_reached = False
    
    def emergency_callback(self, msg):
        """Emergencia"""
        was_emergency = self.emergency_stop
        self.emergency_stop = False#msg.data
        
        if self.emergency_stop and not was_emergency:
            self.get_logger().warn("¡EMERGENCIA ACTIVADA!")
            self.publish_command(0.0, 0.0)
            self.current_cmd_linear_speed = 0.0
    
    def lid_callback(self, msg):
        """Estado de la tapa"""
        was_closed = self.lid_closed
        self.lid_closed = True#msg.data
        
        if not self.lid_closed and was_closed:
            self.get_logger().warn("Laptop abierta - Robot detenido")
            self.publish_command(0.0, 0.0)
            self.current_cmd_linear_speed = 0.0
    
    def diagnostics_callback(self, msg):
        """Diagnóstico del sistema"""
        if msg.level == DiagnosticStatus.ERROR:
            self.system_error = True
            self.system_warning = False
            self.get_logger().error(f"Error del sistema: {msg.message}")
        elif msg.level == DiagnosticStatus.WARN:
            self.system_warning = True
            self.system_error = False
            self.get_logger().warn(f"Advertencia: {msg.message}")
        else:
            self.system_error = False
            self.system_warning = False
    
    def objects_callback(self, msg):
        """Objetos detectados"""
        self.objects_data = msg
        self.last_object_time = time.time()
        self.analyze_objects()
    
    def osm_callback(self, msg):
        """Elementos OSM cercanos"""
        self.osm_data = msg
        self.last_osm_time = time.time()
        self.analyze_osm_elements()
    
    def odometry_callback(self, msg):
        """Odometría del EKF - SOLO PARA INFORMACIÓN"""
        self.has_received_odom = True
        self.last_odom_time = time.time()
        
        # Velocidad lineal del robot (solo para info/debug)
        self.robot_velocity_x = msg.twist.twist.linear.x
        self.robot_velocity_y = msg.twist.twist.linear.y
        
        # Velocidad total (magnitud)
        self.robot_velocity = math.sqrt(
            self.robot_velocity_x**2 + 
            self.robot_velocity_y**2
        )
        
        # Guardar en historial
        self.speed_history.append(self.robot_velocity)
        
        # Posición y orientación
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
        # Calcular yaw de la orientación
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # ============ ANÁLISIS DE DATOS ============
    
    def analyze_objects(self):
        """Analiza objetos detectados"""
        if not self.objects_data:
            self.obstacle_detected = ObstacleType.NONE
            self.obstacle_distance = float('inf')
            return
        
        min_ped_dist = self.get_parameter('min_pedestrian_distance').value
        min_veh_dist = self.get_parameter('min_vehicle_distance').value
        min_bike_dist = self.get_parameter('min_bicycle_distance').value
        lane_width = self.get_parameter('lane_width').value
        
        closest_distance = float('inf')
        closest_obstacle = ObstacleType.NONE
        
        for obj in self.objects_data.objects:
            if obj.distance <= 0 or not obj.distance_valid:
                continue
            
            # Verificar si está en trayectoria
            in_path = abs(obj.lateral_offset) < (lane_width / 2)
            if not in_path:
                continue
            
            obj_class = obj.class_name.lower()
            obj_distance = obj.distance
            
            # Personas
            if any(word in obj_class for word in ['person', 'pedestrian', 'people', 'human']):
                if obj_distance < min_ped_dist and obj_distance < closest_distance:
                    closest_distance = obj_distance
                    closest_obstacle = ObstacleType.PEDESTRIAN
            
            # Vehículos
            elif any(word in obj_class for word in ['car', 'truck', 'bus', 'vehicle', 'van']):
                if obj_distance < min_veh_dist and obj_distance < closest_distance:
                    closest_distance = obj_distance
                    closest_obstacle = ObstacleType.VEHICLE
            
            # Bicicletas y motos
            elif any(word in obj_class for word in ['bicycle', 'motorcycle', 'bike', 'moto', 'cyclist']):
                if obj_distance < min_bike_dist and obj_distance < closest_distance:
                    closest_distance = obj_distance
                    closest_obstacle = ObstacleType.BICYCLE
        
        self.obstacle_detected = closest_obstacle
        self.obstacle_distance = closest_distance
    
    def analyze_osm_elements(self):
        """Analiza elementos OSM cercanos"""
        self.near_crosswalk = False
        self.near_speedbump = False
        self.crosswalk_has_pedestrian = False
        self.crosswalk_element = None
        
        if not self.osm_data:
            return
        
        crosswalk_slow_dist = self.get_parameter('crosswalk_slow_distance').value
        speedbump_slow_dist = self.get_parameter('speedbump_slow_distance').value
        
        closest_crosswalk = None
        closest_crosswalk_dist = float('inf')
        
        closest_speedbump = None
        closest_speedbump_dist = float('inf')
        
        for element in self.osm_data.nearby_elements:
            # Cruz peatonal
            if element.feature_type == OSMElement.FEATURE_CROSSING:
                if element.longitudinal_distance < crosswalk_slow_dist and element.longitudinal_distance < closest_crosswalk_dist:
                    closest_crosswalk = element
                    closest_crosswalk_dist = element.longitudinal_distance
            
            # Reductor de velocidad
            elif element.feature_type == OSMElement.FEATURE_TRAFFIC_CALMING:
                if element.longitudinal_distance < speedbump_slow_dist and element.longitudinal_distance < closest_speedbump_dist:
                    closest_speedbump = element
                    closest_speedbump_dist = element.longitudinal_distance
        
        # Procesar cruces peatonales
        if closest_crosswalk:
            self.near_crosswalk = True
            self.crosswalk_element = closest_crosswalk
            
            # Verificar peatones cerca del cruce
            if self.objects_data:
                for obj in self.objects_data.objects:
                    if 'person' in obj.class_name.lower() or 'pedestrian' in obj.class_name.lower():
                        if abs(obj.distance - closest_crosswalk_dist) < 3.0:
                            self.crosswalk_has_pedestrian = True
                            break
        
        # Procesar reductores
        if closest_speedbump:
            self.near_speedbump = True
    
    # ============ LÓGICA DE CONTROL ============
    def calculate_target_angle(self, target_state):
        if target_state in [
            RobotState.EMERGENCY,
            RobotState.IDLE,
            RobotState.GOAL_REACHED,
            RobotState.STOPPED
        ]:
            return 0.0

        omega = self.target_angular_speed

        # Suavizar aún más en estados lentos
        if target_state == RobotState.SLOW:
            omega *= 0.6

        if self.near_crosswalk or self.near_speedbump:
            omega *= 0.7

        return omega


    def check_system_ready(self):
        """Verifica que el sistema esté listo para moverse"""
        if self.emergency_stop:
            return False
        
        if not self.lid_closed:
            return False
        
        if self.system_error:
            return False
        
        if self.goal_reached:
            return False
        
        if not self.has_received_odom:
            self.get_logger().warn("Esperando datos de odometría...", 
                                  throttle_duration_sec=5.0)
            return False
        
        
        if time.time() - self.last_odom_time > 1.0:
            self.get_logger().warn("Datos de odometría desactualizados")
            return False
        
        return True
    
    def determine_target_state(self):
        """Determina el estado objetivo"""
        if self.emergency_stop:
            return RobotState.EMERGENCY
        
        if not self.lid_closed:
            return RobotState.IDLE
        
        if self.goal_reached:
            return RobotState.GOAL_REACHED
        
        if not self.check_system_ready():
            return RobotState.IDLE
        
        # Obstáculos cercanos
        if self.obstacle_detected != ObstacleType.NONE:
            crosswalk_stop_dist = self.get_parameter('crosswalk_stop_distance').value
            
            if self.near_crosswalk and self.crosswalk_has_pedestrian:
                if self.obstacle_distance < crosswalk_stop_dist:
                    return RobotState.STOPPED
                else:
                    return RobotState.SLOW
            
            return RobotState.STOPPED
        
        # Elementos OSM
        if self.near_crosswalk:
            if self.crosswalk_has_pedestrian:
                return RobotState.STOPPED
            else:
                return RobotState.SLOW
        
        if self.near_speedbump:
            return RobotState.SLOW
        
        # Advertencia del sistema
        if self.system_warning:
            return RobotState.SLOW
        
        # Navegación normal
        return RobotState.RUNNING
    
    def calculate_target_speed(self, target_state):
        """Calcula velocidad objetivo"""
        if target_state in [RobotState.EMERGENCY, RobotState.STOPPED, 
                           RobotState.GOAL_REACHED, RobotState.IDLE]:
            return 0.0
        
        if target_state == RobotState.SLOW:
            slow_speed = self.get_parameter('slow_speed_linear').value
            
            if self.near_crosswalk and not self.crosswalk_has_pedestrian:
                if self.crosswalk_element and self.crosswalk_element.longitudinal_distance < 5.0:
                    return slow_speed * 0.7
            
            if self.near_speedbump:
                return slow_speed * 0.8
            
            return slow_speed
        
        if target_state == RobotState.RUNNING:
            return self.get_parameter('max_linear_speed').value
        
        return 0.0
    
    
    def apply_acceleration_limit(self, target_speed):
        """Aplica límite de aceleración BASADO EN COMANDOS ANTERIORES"""
        max_accel = self.get_parameter('max_linear_accel').value
        max_decel = self.get_parameter('max_linear_decel').value
        dt = 1.0 / self.get_parameter('control_frequency').value
        
        # Calcular cambio máximo permitido
        if target_speed > self.current_cmd_linear_speed:
            max_change = max_accel * dt
        else:
            max_change = max_decel * dt
        
        # Limitar cambio
        speed_diff = target_speed - self.current_cmd_linear_speed
        
        if abs(speed_diff) > max_change:
            if speed_diff > 0:
                new_speed = self.current_cmd_linear_speed + max_change
            else:
                new_speed = self.current_cmd_linear_speed - max_change
        else:
            new_speed = target_speed
        
        return max(0.0, new_speed)
    
    def handle_crosswalk_waiting(self):
        """Maneja espera en cruces peatonales"""
        if self.near_crosswalk and self.crosswalk_has_pedestrian:
            crosswalk_wait_time = self.get_parameter('crosswalk_wait_time').value
            
            if self.crosswalk_wait_start is None:
                self.crosswalk_wait_start = time.time()
                self.get_logger().info(f"Esperando en cruce peatonal ({self.obstacle_distance:.1f}m)...")
            
            wait_duration = time.time() - self.crosswalk_wait_start
            
            if wait_duration >= crosswalk_wait_time:
                self.get_logger().info("Tiempo de espera completado en cruce")
                self.crosswalk_wait_start = None
                return True
            
            return False
        
        self.crosswalk_wait_start = None
        return True
    
    def control_cycle(self):
        """Ciclo principal de control"""
        try:
            # Verificación inicial de condiciones críticas
            if self.emergency_stop or not self.lid_closed or self.goal_reached:
                self.publish_command(0.0, 0.0)
                self.current_cmd_linear_speed = 0.0
                if not self.lid_closed or self.emergency_stop:
                    pass
                return
            
            # Determinar estado
            target_state = self.determine_target_state()
            
            # Manejar casos especiales
            can_move = True
            if target_state == RobotState.STOPPED and self.near_crosswalk and self.crosswalk_has_pedestrian:
                can_move = self.handle_crosswalk_waiting()
            
            # Cambio de estado
            if target_state != self.current_state:
                self.last_state = self.current_state
                self.current_state = target_state
                self.state_start_time = time.time()
                
                state_names = {
                    RobotState.IDLE: "IDLE",
                    RobotState.RUNNING: "RUNNING",
                    RobotState.SLOW: "SLOW",
                    RobotState.STOPPED: "STOPPED",
                    RobotState.EMERGENCY: "EMERGENCY",
                    RobotState.GOAL_REACHED: "GOAL_REACHED"
                }
                
                self.get_logger().info(f"Cambio de estado: {state_names[self.last_state]} -> {state_names[self.current_state]}")
            
            # Calcular velocidades
            target_linear = 0.0
            if can_move or target_state != RobotState.STOPPED:
                target_linear = self.calculate_target_speed(target_state)
            else:
                self.publish_command(0.0, 0.0)
                self.current_cmd_linear_speed = 0.0
                return
            
            # Calcular ángulo
            target_angular = self.calculate_target_angle(target_state)
            
            # Aplicar aceleración limitada
            new_linear_speed = self.apply_acceleration_limit(target_linear)
            filtered_angular = self.apply_angular_acceleration_limit(target_angular)

            
            
            # Publicar comando
            self.publish_command(new_linear_speed, filtered_angular)
            
            # Actualizar velocidad del comando actual
            self.current_cmd_linear_speed = new_linear_speed
            self.current_cmd_angular_speed = filtered_angular 
            
            # Publicar estado activo
            active_msg = Bool()
            active_msg.data = (target_state not in [
                RobotState.IDLE, RobotState.EMERGENCY, RobotState.GOAL_REACHED
            ])
            self.control_state_pub.publish(active_msg)
            
            # Debug periódico
            current_time = time.time()
            if current_time - self.last_debug_print > 2.0:
                self.last_debug_print = current_time
                
                debug_info = [
                    f"Estado: {self.current_state.name}",
                    f"Cmd Vel: {new_linear_speed:.2f}m/s",
                    f"Real Vel: {self.robot_velocity:.2f}m/s",
                    f"Ang: {target_angular:.2f}rad/s",
                    f"OSM: C={self.near_crosswalk}",
                    f"R={self.near_speedbump}",
                    f"Obs: {self.obstacle_detected.name}",
                    f"{self.obstacle_distance:.1f}m"
                ]
                
                self.get_logger().info(" | ".join(debug_info))
            
        except Exception as e:
            self.get_logger().error(f"Error en control: {e}")
            self.publish_command(0.0, 0.0)
            self.current_cmd_linear_speed = 0.0
    
    def publish_command(self, linear_speed, angular_speed):
        """Publica comando de velocidad"""
        max_linear = self.get_parameter('max_linear_speed').value
        max_angular = self.get_parameter('max_angular_speed').value
        
        linear_speed = max(0.0, min(linear_speed, max_linear))
        angular_speed = max(-max_angular, min(angular_speed, max_angular))
            
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_speed)
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = -float(angular_speed)
        
        self.cmd_vel_pub.publish(twist_msg)
        self.last_cmd_vel_time = time.time()
    
    def cleanup_old_data(self):
        """Limpia datos viejos"""
        object_timeout = self.get_parameter('object_timeout').value
        current_time = time.time()
        
        if self.objects_data and (current_time - self.last_object_time) > object_timeout:
            self.objects_data = None
            self.obstacle_detected = ObstacleType.NONE
            self.obstacle_distance = float('inf')
        
        if self.osm_data and (current_time - self.last_osm_time) > object_timeout:
            self.osm_data = None
            self.near_crosswalk = False
            self.near_speedbump = False
            self.crosswalk_has_pedestrian = False
            self.crosswalk_element = None
    
    def destroy_node(self):
        """Limpieza al apagar"""
        self.publish_command(0.0, 0.0)
        self.get_logger().info("Brain Controller apagado")
        super().destroy_node()


# =================== FUNCIÓN PRINCIPAL ===================
def main(args=None):
    rclpy.init(args=args)
    node = BrainController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error fatal: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()