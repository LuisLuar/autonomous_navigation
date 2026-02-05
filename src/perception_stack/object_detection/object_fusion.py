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


class ObjectFusionNode(Node):
    def __init__(self):
        super().__init__('object_fusion')
        
        # ============ CARGAR CALIBRACIÓN DEL JSON ============
        calibration_path = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration_front.json"
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
            self.get_logger().error(f" Error cargando calibración: {e}")
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
            ('publish_rate', 10.0),              # Hz de publicación
            ('use_rgb_camera_info', True),       # usar info de cámara RGB
            ('depth_scale', 0.001),              # escala de depth (mm a m)
            ('camera_offset_x', 0.31),           # offset cámara adelante (m) - CONSISTENTE CON SEGMENTACIÓN
            ('camera_offset_y', 0.0),            # offset cámara lateral (m)
            ('debug_ipm', False),                # modo debug para IPM
        ])
        
        # ============ ALTURAS TÍPICAS DE OBJETOS ============
        self.object_heights = {
            'person': 1.7,      # altura promedio persona
            'pedestrian': 1.7,
            'car': 1.5,         # altura centro de masa (no altura total)
            'truck': 2.5,
            'bus': 3.0,
            'bicycle': 1.1,     # altura sillín
            'motorcycle': 1.3,
            'bike': 1.1,
            # Añadir alturas para las señales (aunque no se usarán para IPM)
            'speed bump signage': 2.0,      # altura aproximada de señal
            'crosswalk signage': 2.0,       # altura aproximada de señal
            'speed bump': 0.15,             # altura del badén
            'crosswalk': 0.0,               # en el suelo
        }
        
        # ============ VARIABLES DE ESTADO ============
        self.robot_speed = 0.0           # m/s
        self.robot_yaw = 0.0             # radianes
        self.robot_position = np.array([0.0, 0.0, 0.0])  # x, y, z
        
        # Historial de tracks - ahora separados por fuente
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
            'source_history': deque(maxlen=5),  # historial de fuentes de distancia
            'detection_source': '',             # 'objects' o 'senaletica'
        })
        
        # Detecciones separadas por fuente
        self.last_detections_objects = None
        self.last_detections_senaletica = None
        self.last_detections_time_objects = None
        self.last_detections_time_senaletica = None
        
        # Para fusionar ambas fuentes
        self.all_detections = []
        
        # Cámara de profundidad
        self.depth_image = None
        self.depth_timestamp = None
        self.camera_info = None

        # ============ CLASES QUE NO ESTÁN AL NIVEL DEL SUELO ============
        self.aerial_classes = {
            'speed bump signage',
            'crosswalk signage',
            'Crosswalk signage',  # por si acaso
            'speed bump signage'
        }
        
        # ============ MAPEO DE CLASES PARA EVITAR CONFLICTOS ============
        # Vamos a re-mapear los IDs de las señales para evitar conflictos
        self.class_id_offset = 200  # Offset para IDs de señales
        
        # ============ ROS SUBSCRIPTORES ============
        # Detecciones con tracking de objetos (primera red)
        self.create_subscription(
            DetectionArray,
            '/detection/results',
            self.detection_callback_objects,
            10
        )

        # Detecciones de señalética (segunda red)
        self.create_subscription(
            DetectionArray,
            '/detection/results_senaletica',
            self.detection_callback_senaletica,
            10
        )
        
        # Odometría del robot
        self.create_subscription(
            Odometry,
            '/odometry/local',
            self.odometry_callback,
            10
        )
        
        # Cámara de profundidad
        self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # Info de cámara RGB (para verificar calibración)
        if self.get_parameter('use_rgb_camera_info').value:
            self.create_subscription(
                CameraInfo,
                '/camera/rgb/camera_info',
                self.camera_info_callback,
                10
            )
        
        # ============ ROS PUBLISHERS ============
        self.pub_fused = self.create_publisher(
            ObjectInfoArray,
            '/objects/fused_info',
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
    
    def detection_callback_objects(self, msg: DetectionArray):
        """Procesar detecciones de la primera red (objetos)"""
        self.last_detections_objects = msg
        self.last_detections_time_objects = time.time()
        self.process_detections(msg, 'objects')
    
    def detection_callback_senaletica(self, msg: DetectionArray):
        """Procesar detecciones de la segunda red (señalética)"""
        self.last_detections_senaletica = msg
        self.last_detections_time_senaletica = time.time()
        self.process_detections(msg, 'senaletica')
    
    def process_detections(self, msg: DetectionArray, source: str):
        """Procesar detecciones de una fuente específica"""
        for det in msg.detections:
            track_id = det.track_id
            
            # Para señales, modificar el track_id para evitar conflictos
            if source == 'senaletica':
                # Añadir offset al track_id para evitar colisiones
                track_id = track_id + 10000
                
                # También modificar el class_id para evitar conflictos
                # Pero mantener el nombre de clase original
                original_class_id = det.class_id
                det.class_id = original_class_id + self.class_id_offset
            
            # Solo procesar si tiene ID válido
            if track_id == 0:
                continue
                
            # Obtener información de este track
            track_info = self.tracks_history[track_id]
            
            # Guardar nombre de clase y fuente
            track_info['class_name'] = det.class_name
            track_info['detection_source'] = source
            
            # Verificar si es una clase aérea (no en suelo)
            is_aerial = det.class_name in self.aerial_classes
            
            # Para clases aéreas, no calcular distancia basada en IPM
            if is_aerial:
                # Solo intentar usar depth si está disponible
                center_x = det.center_x
                center_y = (det.y1 + det.y2) / 2  # Usar centro en lugar de suelo
                
                distance = 0.0
                lateral = 0.0
                source_code = 0
                
                # Intentar obtener distancia solo de depth
                if self.depth_image is not None and self.depth_timestamp is not None:
                    if time.time() - self.depth_timestamp < 0.5:
                        ix = int(np.clip(center_x, 0, self.depth_image.shape[1] - 1))
                        iy = int(np.clip(center_y, 0, self.depth_image.shape[0] - 1))

                        roi = self.depth_image[
                            max(iy-2, 0):min(iy+3, self.depth_image.shape[0]),
                            max(ix-2, 0):min(ix+3, self.depth_image.shape[1])
                        ]

                        valid = roi[
                            (roi > self.get_parameter('depth_valid_min').value) &
                            (roi < self.get_parameter('depth_valid_max').value)
                        ]

                        if valid.size > 0:
                            Z = float(np.median(valid))
                            X = (center_x - self.cx) / self.fx * Z
                            distance = Z
                            lateral = X
                            source_code = 1  # depth
            else:
                # Para objetos en suelo, calcular normalmente
                center_x = det.center_x
                
                # Obtener el punto de contacto con el suelo CORREGIDO
                ground_y = self.get_ground_contact_point(det, track_info)
                
                # Obtener distancia (prioridad: 1.depth, 2.IPM, 3.estimación)
                distance, lateral, source_code = self.estimate_position(
                    center_x, ground_y,
                    det.class_name,
                    det.x1, det.y1, det.x2, det.y2
                )
            
            # Guardar información
            if distance > 0 or is_aerial:
                track_info['positions'].append((distance, lateral))
                track_info['timestamps'].append(time.time())
                if distance > 0:
                    track_info['distance_measurements'].append((distance, source_code))
                track_info['source_history'].append(source_code)
                track_info['last_seen'] = time.time()

    
    def get_ground_contact_point(self, det, track_info):
        """Calcular punto de contacto con el suelo considerando altura del objeto"""
        # Para señales, devolver centro (no intentar encontrar suelo)
        if track_info['class_name'] in self.aerial_classes:
            return (det.y1 + det.y2) / 2
        
        # Por defecto usar borde inferior
        default_y = det.y2
        
        # Si tenemos historial de distancias, calcular corrección de altura
        if track_info['distance_measurements']:
            # Usar la última distancia válida
            last_distance = track_info['distance_measurements'][-1][0]
            
            # Obtener altura del objeto según su clase
            obj_height = 0.0
            for key in self.object_heights:
                if key in det.class_name.lower():
                    obj_height = self.object_heights[key]
                    break
            
            if obj_height > 0 and last_distance > 0:
                # Calcular cuántos píxeles está el suelo por debajo del borde inferior
                # Usando semejanza de triángulos
                pixels_to_ground = (obj_height * self.fy) / last_distance
                
                # El suelo está "pixels_to_ground" píxeles por debajo del borde inferior
                ground_y = det.y2 + pixels_to_ground
                
                # Limitar a un máximo razonable (no más del 30% de la altura del bbox)
                max_addition = 0.3 * (det.y2 - det.y1)
                ground_y = min(ground_y, det.y2 + max_addition)
                
                return det.y2#ground_y
        
        # Fallback: usar borde inferior ajustado según clase
        if 'person' in det.class_name.lower():
            # Para personas, asumir que los pies están a ~85% de la altura
            return det.y2 #+ 0.85 * (det.y2 - det.y1)
        elif any(word in det.class_name.lower() for word in ['car', 'truck', 'bus']):
            # Para vehículos, asumir que las ruedas están a ~95%
            return det.y2 #+ 0.95 * (det.y2 - det.y1)
        elif 'speed bump' in det.class_name.lower() and 'signage' not in det.class_name.lower():
            # Para badenes, están en el suelo
            return det.y2
        elif 'crosswalk' in det.class_name.lower() and 'signage' not in det.class_name.lower():
            # Para cruces peatonales, están en el suelo
            return det.y2
        else:
            # Por defecto usar borde inferior
            return default_y
    
    def estimate_position(self, center_x, ground_y, class_name, x1, y1, x2, y2):
        """
        Devuelve SIEMPRE:
            distance (Z, m)
            lateral (X, m)
            source (1=depth, 2=ipm, 3=size)
        """
        # Para señales aéreas, no usar IPM
        if class_name in self.aerial_classes:
            return 0.0, 0.0, 0
        
        # ================= DEPTH =================
        if self.depth_image is not None and self.depth_timestamp is not None:
            if time.time() - self.depth_timestamp < 0.5:
                ix = int(np.clip(center_x, 0, self.depth_image.shape[1] - 1))
                iy = int(np.clip(ground_y, 0, self.depth_image.shape[0] - 1))

                roi = self.depth_image[
                    max(iy-2, 0):min(iy+3, self.depth_image.shape[0]),
                    max(ix-2, 0):min(ix+3, self.depth_image.shape[1])
                ]

                valid = roi[
                    (roi > self.get_parameter('depth_valid_min').value) &
                    (roi < self.get_parameter('depth_valid_max').value)
                ]

                if valid.size > 0:
                    Z = float(np.median(valid))
                    X = (center_x - self.cx) / self.fx * Z
                    return Z, X, 1

        # ================= IPM 3D =================
        if self.get_parameter('ipm_enabled').value:
            X, Z = self.pixel_to_ground(center_x, ground_y)
            if X is not None and Z > 0:
                # Aplicar offset de cámara para consistencia con segmentación
                cam_offset_x = self.get_parameter('camera_offset_x').value
                cam_offset_y = self.get_parameter('camera_offset_y').value
                
                # Z ya es distancia adelante, X es lateral
                Z_with_offset = Z + cam_offset_x
                X_with_offset = -X + cam_offset_y
                
                return Z_with_offset, X_with_offset, 2

        # ================= SIZE FALLBACK =================
        Z = self.estimate_distance_from_size(class_name, y2 - y1)
        if Z > 0:
            X = (center_x - self.cx) / self.fx * Z
            return Z, X, 3

        return 0.0, 0.0, 0
    
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
    
    def depth_callback(self, msg: Image):
        """Guardar imagen de profundidad más reciente"""
        try:
            depth_scale = self.get_parameter('depth_scale').value
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
            self.depth_image = depth_image.astype(np.float32) * depth_scale
            self.depth_timestamp = time.time()
        except Exception as e:
            self.get_logger().warn(f"Error procesando depth: {e}")
    
    def camera_info_callback(self, msg: CameraInfo):
        """Verificar que coinciden los parámetros con el JSON"""
        if self.camera_info is None:
            self.camera_info = msg
            # Verificar coincidencia con JSON
            if abs(msg.k[0] - self.fx) > 0.1 or abs(msg.k[4] - self.fy) > 0.1:
                self.get_logger().warn(
                    f"Calibración diferente: JSON(fx={self.fx}, fy={self.fy}) "
                    f"vs CameraInfo(fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f})"
                )
    
    # ============ FUNCIONES DE ESTIMACIÓN ============      
    def estimate_distance_from_size(self, class_name, height_px):
        """Estimar distancia basada en altura del objeto en píxeles"""
        # Para señales aéreas, no estimar distancia por tamaño
        if class_name in self.aerial_classes:
            return 0.0
            
        for key in self.object_heights:
            if key in class_name.lower():
                distance = (self.object_heights[key] * self.fy) / max(height_px, 1)
                return float(distance)
        
        return 0.0
    
    def calculate_velocities(self, track_id):
        """Calcular velocidades del objeto basado en historial"""
        track_info = self.tracks_history[track_id]
        
        # Para señales aéreas, no calcular velocidades
        if track_info['class_name'] in self.aerial_classes:
            return 0.0, 0.0, 0.0, 0.0
        
        if len(track_info['positions']) < 2:
            return 0.0, 0.0, 0.0, 0.0  # vx, vy, speed, relative_speed
        
        positions = list(track_info['positions'])
        timestamps = list(track_info['timestamps'])
        
        # Calcular velocidad instantánea
        dt = timestamps[-1] - timestamps[-2]
        if dt > 0:
            dx = positions[-1][0] - positions[-2][0]  # cambio en distancia
            dy = positions[-1][1] - positions[-2][1]  # cambio en lateral
            
            vx = float(dx / dt)
            vy = float(dy / dt)
            speed = float(math.sqrt(vx**2 + vy**2))
            
            # Velocidad relativa al robot
            relative_speed = float(vx)  # positiva = se aleja, negativa = se acerca
            
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
        # Recolectar todas las detecciones activas
        all_detections = []
        
        # Agregar detecciones de objetos si están disponibles
        if self.last_detections_objects is not None:
            for det in self.last_detections_objects.detections:
                if det.track_id != 0:
                    all_detections.append(det)
        
        # Agregar detecciones de señalética si están disponibles
        if self.last_detections_senaletica is not None:
            for det in self.last_detections_senaletica.detections:
                if det.track_id != 0:
                    # Crear una copia para modificar IDs - CORREGIDO
                    from custom_interfaces.msg import Detection  # Asegúrate de importar
                    
                    modified_det = Detection()
                    # Copiar todos los campos manualmente
                    modified_det.track_id = det.track_id + 10000  # Offset para evitar conflictos
                    modified_det.class_id = det.class_id + self.class_id_offset
                    modified_det.class_name = det.class_name
                    modified_det.confidence = det.confidence
                    modified_det.x1 = det.x1
                    modified_det.y1 = det.y1
                    modified_det.x2 = det.x2
                    modified_det.y2 = det.y2
                    modified_det.center_x = det.center_x
                    modified_det.center_y = det.center_y
                    # Copiar header si existe en Detection
                    if hasattr(det, 'header'):
                        modified_det.header = det.header
                    
                    all_detections.append(modified_det)
                
        if not all_detections:
            return
        
        start_time = time.time()
        
        msg = ObjectInfoArray()
        # Usar el timestamp más reciente
        if self.last_detections_time_objects and self.last_detections_time_senaletica:
            latest_time = max(self.last_detections_time_objects, self.last_detections_time_senaletica)
        elif self.last_detections_time_objects:
            latest_time = self.last_detections_time_objects
        else:
            latest_time = self.last_detections_time_senaletica
            
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.robot_speed = float(self.robot_speed)
        
        static_count = 0
        moving_count = 0
        aerial_count = 0
        
        # Limpiar tracks viejos
        current_time = time.time()
        tracks_to_remove = []
        for track_id, track_info in self.tracks_history.items():
            if current_time - track_info['last_seen'] > 5.0:
                tracks_to_remove.append(track_id)
        
        for track_id in tracks_to_remove:
            del self.tracks_history[track_id]
        
        # Procesar cada detección
        for det in all_detections:
            if det.track_id == 0:
                continue
            
            track_info = self.tracks_history.get(det.track_id)
            if not track_info:
                continue
            
            # Verificar si es señal aérea
            is_aerial = track_info['class_name'] in self.aerial_classes
            
            if is_aerial:
                aerial_count += 1
                # Para señales aéreas, no calcular velocidades ni TTC
                vx, vy, speed, relative_speed = 0.0, 0.0, 0.0, 0.0
                is_static = True
                ttc = 999.0
            else:
                # Calcular velocidades para objetos en suelo
                vx, vy, speed, relative_speed = self.calculate_velocities(det.track_id)
                
                # Determinar si es estático
                is_static = False
                static_threshold = self.get_parameter('min_velocity_threshold').value
                
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
            
            # Determinar si está en trayectoria (solo para objetos en suelo)
            is_in_path = False
            if not is_aerial and track_info['positions']:
                _, lateral = track_info['positions'][-1]
                path_width = self.get_parameter('path_width').value
                if abs(lateral) < path_width / 2:
                    is_in_path = True
            
            # ===== CREAR MENSAJE CON TIPOS CORRECTOS =====
            obj_info = ObjectInfo()
            
            # Identificación
            obj_info.track_id = int(det.track_id)
            obj_info.class_name = str(track_info['class_name'])
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
                if distance > 0:
                    obj_info.ground_distance = float(math.sqrt(float(distance)**2 + float(lateral)**2))
                else:
                    obj_info.ground_distance = 0.0
                obj_info.distance_valid = bool(float(distance) > 0) and not is_aerial
                
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
            obj_info.speed = float(speed)
            obj_info.relative_speed = float(relative_speed)
            
            # ===== CAMPOS BOOLEANOS =====
            obj_info.is_static = bool(is_static or is_aerial)  # Señales siempre estáticas
            obj_info.is_moving_toward = bool(relative_speed < -0.1 and not is_aerial)
            obj_info.is_moving_away = bool(relative_speed > 0.1 and not is_aerial)
            obj_info.is_in_path = bool(is_in_path)
            obj_info.time_to_collision = float(ttc)
            
            # Calidad de track
            obj_info.track_age = int(len(track_info['positions']))
            obj_info.is_lost = False
            
            # Score basado en confianza y consistencia de fuente
            confidence_score = float(det.confidence) * 0.7
            age_score = min(float(len(track_info['positions']))/50.0, 0.3)
            
            # Bonificación por consistencia en fuente de distancia
            source_consistency = 0.0
            if len(track_info['source_history']) >= 3:
                unique_sources = len(set(track_info['source_history']))
                if unique_sources == 1:  # Misma fuente siempre
                    source_consistency = 0.2
            
            obj_info.quality_score = float(min(1.0, confidence_score + age_score + source_consistency))
            
            msg.objects.append(obj_info)
        
        msg.num_static_objects = int(static_count + aerial_count)  # Incluir señales en estáticos
        msg.num_moving_objects = int(moving_count)
        msg.processing_time_ms = float((time.time() - start_time) * 1000.0)
        
        # Publicar
        try:
            self.pub_fused.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error al publicar: {e}")
        

    # ============ IPM CONSISTENTE CON SEGMENTACIÓN ============
    def pixel_to_ground(self, u, v):
        """IPM idéntico al nodo de segmentación de carriles"""
        fx, fy = self.fx, self.fy
        cx, cy = self.cx, self.cy
        h = self.camera_height
        pitch = self.camera_pitch

        # Rayo cámara (IDÉNTICO a segmentación)
        x = (u - cx) / fx
        y = -(v - cy) / fy  # Negativo importante
        ray = np.array([x, y, 1.0])

        # Rotación pitch (IDÉNTICA a segmentación)
        cp = math.cos(pitch)
        sp = math.sin(pitch)

        R = np.array([
            [1,  0,   0],
            [0,  cp,  sp],
            [0, -sp,  cp]
        ])

        ray_w = R @ ray

        # Intersección con suelo Y = 0
        if ray_w[1] >= 0:
            return None, None

        t = h / -ray_w[1]

        Xc = ray_w[0] * t   # lateral (derecha +)
        Zc = ray_w[2] * t   # adelante

        return float(Xc), float(Zc)
    
    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectFusionNode()
    
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