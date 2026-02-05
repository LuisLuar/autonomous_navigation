#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
import json
from collections import deque, defaultdict
from scipy.spatial.transform import Rotation as R

# Mensajes ROS
from custom_interfaces.msg import DetectionArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from custom_interfaces.msg import ObjectInfoArray, ObjectInfo

# Utilidades
from cv_bridge import CvBridge
import cv2


class ObjectFusionNodeLeft(Node):
    def __init__(self):
        super().__init__('object_fusion_left')
        
        # ============ CARGAR CALIBRACIÓN DEL JSON ============
        calibration_path = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration.json"
        try:
            with open(calibration_path, 'r') as f:
                calib = json.load(f)
            
            # PARÁMETROS CALIBRADOS EXACTAMENTE
            self.camera_height = float(calib['camera_height'])        # 0.38 m
            self.camera_pitch = float(calib['camera_pitch'])         # 0.1312 rad (7.52°)
            self.fx = float(calib['intrinsics']['fx'])               # 574.1
            self.fy = float(calib['intrinsics']['fy'])               # 574.1
            self.cx = float(calib['intrinsics']['cx'])               # 320.0
            self.cy = float(calib['intrinsics']['cy'])               # 240.0
            
        except Exception as e:
            #self.get_logger().error(f" Error cargando calibración: {e}")
            # Valores por defecto si falla
            self.camera_height = 0.38
            self.camera_pitch = 0.1312
            self.fx = 574.1
            self.fy = 574.1
            self.cx = 320.0
            self.cy = 240.0
        
        # ============ PARÁMETROS CONFIGURABLES ============
        self.declare_parameters(namespace='', parameters=[
            ('min_velocity_threshold', 0.1),     # m/s para considerar estático
            ('ttc_warning_threshold', 5.0),      # segundos para advertencia
            ('ttc_critical_threshold', 2.0),     # segundos para crítico
            ('path_width', 1.0),                 # ancho de trayectoria (m)
            ('max_track_age', 300),              # frames máx para track
            ('depth_valid_min', 0.3),            # m mínima de depth válida
            ('depth_valid_max', 12.0),           # m máxima de depth válida
            ('static_object_time', 2.0),         # segundos para confirmar estático
            ('ipm_enabled', True),               # usar IPM cuando falla depth
            ('publish_rate', 20.0),              # Hz de publicación
            ('use_rgb_camera_info', True),       # usar info de cámara RGB
            ('depth_scale', 0.001),              # escala de depth (mm a m)
        ])
        
        # ============ VARIABLES DE ESTADO ============
        self.robot_speed = 0.0           # m/s
        self.robot_yaw = 0.0             # radianes
        self.robot_position = np.array([0.0, 0.0, 0.0])  # x, y, z
        
        # Historial de tracks
        self.tracks_history = defaultdict(lambda: {
            'positions': deque(maxlen=30),      # (distance, lateral)
            'timestamps': deque(maxlen=30),
            'velocities': deque(maxlen=10),
            'first_seen': time.time(),
            'last_seen': time.time(),
            'static_confirmed': False,
            'static_counter': 0,
            'distance_measurements': deque(maxlen=10),
            'class_name': '',
        })
        
        # Últimos datos recibidos
        self.last_detections = None
        self.last_detections_time = None
        
        # ============ ROS SUBSCRIPTORES ============
        # Detecciones con tracking
        self.create_subscription(
            DetectionArray,
            '/detection/results_left',
            self.detection_callback,
            10
        )
        
        # Odometría del robot
        self.create_subscription(
            Odometry,
            '/odometry/local',
            self.odometry_callback,
            10
        )
        
        # ============ ROS PUBLISHERS ============
        self.pub_fused = self.create_publisher(
            ObjectInfoArray,
            '/objects/fused_info_left',
            10
        )
        
        # ============ UTILITIES ============
        self.bridge = CvBridge()
        
        # Timer para publicación periódica
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self.publish_fused_data
        )

    # ============ CALLBACKS ============
    
    def detection_callback(self, msg: DetectionArray):
        """Procesar detecciones con tracking"""
        self.last_detections = msg
        self.last_detections_time = time.time()
        
        # Actualizar tracks
        for det in msg.detections:
            track_id = det.track_id
            
            # Solo procesar si tiene ID válido
            if track_id == 0:
                continue
                
            # Obtener información de este track
            track_info = self.tracks_history[track_id]
            
            # Guardar nombre de clase
            track_info['class_name'] = det.class_name
            
            # Calcular posición actual en imagen
            center_x = det.center_x
            center_y = det.y2  # Usar base inferior para distancia
            
            # Obtener distancia (prioridad: 1.depth, 2.IPM, 3.estimación)
            distance, distance_source = self.estimate_distance(
                center_x, center_y, det.class_name, det.x1, det.y1, det.x2, det.y2
            )
            
            # Calcular posición en mundo relativa al robot
            lateral_offset = self.calculate_lateral_offset(center_x, distance)
            
            # Actualizar historial
            current_time = time.time()
            track_info['positions'].append((float(distance), float(lateral_offset)))  # <-- CONVERTIR a float
            track_info['timestamps'].append(current_time)
            track_info['last_seen'] = current_time
            
            # Guardar medición de distancia
            if distance > 0:
                track_info['distance_measurements'].append((float(distance), int(distance_source)))  # <-- CONVERTIR
    
    def odometry_callback(self, msg: Odometry):
        """Actualizar velocidad y pose del robot"""
        # Velocidad lineal
        self.robot_speed = float(math.sqrt(
            msg.twist.twist.linear.x ** 2 +
            msg.twist.twist.linear.y ** 2
        ))
        
        # Orientación (yaw)
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = R.from_quat(q).as_euler('zyx')
        self.robot_yaw = float(euler[0])
        
        # Posición
        self.robot_position[0] = float(msg.pose.pose.position.x)
        self.robot_position[1] = float(msg.pose.pose.position.y)
        self.robot_position[2] = float(msg.pose.pose.position.z)
    
    # ============ FUNCIONES DE ESTIMACIÓN ============
    
    def estimate_distance(self, center_x, center_y, class_name, x1, y1, x2, y2):
        """
        Estimar distancia con prioridades:
        1. Cámara de profundidad (si disponible y válida)
        2. IPM (usando calibración del JSON)
        3. Estimación basada en tamaño de objeto
        """
        distance = 0.0
        source = 0  # 0=none, 1=depth, 2=ipm, 3=size
        
        # 2. Si falla depth, usar IPM
        if distance <= 0 and self.get_parameter('ipm_enabled').value:
            ipm_distance = self.estimate_distance_ipm(center_y)
            if ipm_distance > 0:
                distance = float(ipm_distance)  # <-- CONVERTIR a float
                source = 2
        
        # 3. Si todo falla, estimación basada en tamaño
        if distance <= 0:
            size_distance = self.estimate_distance_from_size(class_name, y2 - y1)
            if size_distance > 0:
                distance = float(size_distance)  # <-- CONVERTIR a float
                source = 3
        
        return distance, source
    
    def estimate_distance_ipm(self, v):
        """Estimar distancia usando IPM con calibración del JSON"""
        h = self.camera_height
        fy = self.fy
        pitch = self.camera_pitch
        
        v0 = self.cy + fy * math.tan(pitch)
        
        if v > v0:
            distance = h * fy / ((v - v0) * math.cos(pitch))
            return float(max(0.1, distance))  # <-- CONVERTIR y mínimo 0.1m
        
        return 0.0
    
    def estimate_distance_from_size(self, class_name, height_px):
        """Estimar distancia basada en altura del objeto en píxeles"""
        typical_heights = {
            'person': 1.7,
            'pedestrian': 1.7,
            'car': 1.5,
            'truck': 2.5,
            'bus': 3.0,
            'bicycle': 1.1,
            'motorcycle': 1.3,
            'bike': 1.1,
        }
        
        for key in typical_heights:
            if key in class_name.lower():
                distance = (typical_heights[key] * self.fy) / max(height_px, 1)
                return float(distance)  # <-- CONVERTIR a float
        
        return 0.0
    
    def calculate_lateral_offset(self, center_x, distance):
        """Calcular desplazamiento lateral del objeto"""
        if distance <= 0:
            return 0.0
        
        x_normalized = (center_x - self.cx) / self.fx
        lateral_offset = distance * x_normalized
        
        return float(lateral_offset)  # <-- CONVERTIR a float
    
    def calculate_velocities(self, track_id):
        """Calcular velocidades del objeto basado en historial"""
        track_info = self.tracks_history[track_id]
        
        if len(track_info['positions']) < 2:
            return 0.0, 0.0, 0.0, 0.0  # vx, vy, speed, relative_speed
        
        positions = list(track_info['positions'])
        timestamps = list(track_info['timestamps'])
        
        # Calcular velocidad instantánea
        dt = timestamps[-1] - timestamps[-2]
        if dt > 0:
            dx = positions[-1][0] - positions[-2][0]  # cambio en distancia
            dy = positions[-1][1] - positions[-2][1]  # cambio en lateral
            
            vx = float(dx / dt)  # <-- CONVERTIR a float
            vy = float(dy / dt)  # <-- CONVERTIR a float
            speed = float(math.sqrt(vx**2 + vy**2))  # <-- CONVERTIR a float
            
            # Velocidad relativa al robot
            relative_speed = float(vx)  # <-- CONVERTIR a float
            
            track_info['velocities'].append((vx, vy, speed, relative_speed))
        
        # Promediar últimos N velocidades
        if len(track_info['velocities']) > 0:
            vx_list = [v[0] for v in track_info['velocities']]
            vy_list = [v[1] for v in track_info['velocities']]
            speed_list = [v[2] for v in track_info['velocities']]
            rel_speed_list = [v[3] for v in track_info['velocities']]
            
            vx_avg = float(np.mean(vx_list)) if vx_list else 0.0
            vy_avg = float(np.mean(vy_list)) if vy_list else 0.0
            speed_avg = float(np.mean(speed_list)) if speed_list else 0.0
            rel_speed_avg = float(np.mean(rel_speed_list)) if rel_speed_list else 0.0
            
            return vx_avg, vy_avg, speed_avg, rel_speed_avg
        
        return 0.0, 0.0, 0.0, 0.0
    
    # ============ PUBLICACIÓN ============
    
    def publish_fused_data(self):
        """Publicar datos fusionados periódicamente"""
        if self.last_detections is None:
            return
        
        start_time = time.time()
        
        msg = ObjectInfoArray()
        msg.header = self.last_detections.header
        msg.header.frame_id = "base_footprint"
        msg.robot_speed = float(self.robot_speed)
        
        static_count = 0
        moving_count = 0
        
        # Limpiar tracks viejos
        current_time = time.time()
        tracks_to_remove = []
        for track_id, track_info in self.tracks_history.items():
            if current_time - track_info['last_seen'] > 5.0:
                tracks_to_remove.append(track_id)
        
        for track_id in tracks_to_remove:
            del self.tracks_history[track_id]
        
        # Procesar cada detección actual
        for det in self.last_detections.detections:
            if det.track_id == 0:
                continue
            
            track_info = self.tracks_history[det.track_id]
            
            # Calcular velocidades
            vx, vy, speed, relative_speed = self.calculate_velocities(det.track_id)
            
            # Determinar si es estático
            is_static = False
            static_threshold = self.get_parameter('min_velocity_threshold').value
            
            # CONVERTIR speed a float de Python
            speed_float = float(speed)
            relative_speed_float = float(relative_speed)
            
            if speed_float < static_threshold:
                track_info['static_counter'] += 1
                if track_info['static_counter'] >= 10:
                    is_static = True
                    track_info['static_confirmed'] = True
                    static_count += 1
                else:
                    moving_count += 1
            else:
                track_info['static_counter'] = 0
                track_info['static_confirmed'] = False
                moving_count += 1
            
            # Calcular TTC si se acerca
            ttc = 999.0
            if relative_speed_float < -0.1:
                if track_info['distance_measurements']:
                    distances = [d[0] for d in track_info['distance_measurements'] if d[0] > 0]
                    if distances:
                        avg_distance = np.mean(distances[-5:]) if len(distances) >= 5 else np.mean(distances)
                        ttc = float(avg_distance / abs(relative_speed_float) if abs(relative_speed_float) > 0 else 999)
            
            # Determinar si está en trayectoria
            is_in_path = False
            if track_info['positions']:
                _, lateral = track_info['positions'][-1]
                path_width = self.get_parameter('path_width').value
                if abs(lateral) < path_width / 2:
                    is_in_path = True
            
            # ===== CREAR MENSAJE CON TIPOS CORRECTOS =====
            obj_info = ObjectInfo()
            
            # Identificación
            obj_info.track_id = int(det.track_id)
            obj_info.class_name = str(det.class_name)
            obj_info.class_id = int(det.class_id)
            obj_info.confidence = float(det.confidence)
            
            # Posición en imagen
            obj_info.x1 = int(det.x1)
            obj_info.y1 = int(det.y1)
            obj_info.x2 = int(det.x2)
            obj_info.y2 = int(det.y2)
            obj_info.center_x = float(det.center_x)
            obj_info.center_y = float(det.center_y)
            
            # Posición 3D
            if track_info['positions']:
                distance, lateral = track_info['positions'][-1]
                obj_info.distance = float(distance)
                obj_info.lateral_offset = float(lateral)
                obj_info.ground_distance = float(math.sqrt(float(distance)**2 + float(lateral)**2))
                obj_info.distance_valid = bool(float(distance) > 0)
                
                if track_info['distance_measurements']:
                    last_source = track_info['distance_measurements'][-1][1]
                    obj_info.distance_source = int(last_source)
                else:
                    obj_info.distance_source = 0
            else:
                obj_info.distance = 0.0
                obj_info.lateral_offset = 0.0
                obj_info.ground_distance = 0.0
                obj_info.distance_valid = False
                obj_info.distance_source = 0
            
            # Velocidades
            obj_info.velocity_x = float(vx)
            obj_info.velocity_y = float(vy)
            obj_info.speed = float(speed_float)
            obj_info.relative_speed = float(relative_speed_float)
            
            # ===== CAMPOS BOOLEANOS - IMPORTANTE! =====
            # Convertir explícitamente a bool de Python
            obj_info.is_static = bool(is_static)
            
            # Aquí está la corrección crítica:
            # relative_speed_float es un float, la comparación devuelve un bool de Python
            is_moving_toward_bool = relative_speed_float < -0.1
            obj_info.is_moving_toward = bool(is_moving_toward_bool)
            
            is_moving_away_bool = relative_speed_float > 0.1
            obj_info.is_moving_away = bool(is_moving_away_bool)
            
            obj_info.is_in_path = bool(is_in_path)
            obj_info.time_to_collision = float(ttc)
            
            # Calidad de track
            obj_info.track_age = int(len(track_info['positions']))
            obj_info.is_lost = False  # bool explícito
            obj_info.quality_score = float(min(1.0, float(det.confidence) * 0.7 + 
                                             min(float(len(track_info['positions']))/50.0, 0.3)))
            
            msg.objects.append(obj_info)
        
        msg.num_static_objects = int(static_count)
        msg.num_moving_objects = int(moving_count)
        msg.processing_time_ms = float((time.time() - start_time) * 1000.0)
        
        # Publicar
        try:
            self.pub_fused.publish(msg)
        except Exception as e:
           pass

    
    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectFusionNodeLeft()
    
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