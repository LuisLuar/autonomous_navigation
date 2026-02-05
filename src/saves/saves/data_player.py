#!/usr/bin/env python3
"""
DataReplayer CON CALIBRACIÓN - Repite datos con tiempo vivo
===========================================================
- Reproduce odometría, IMU, GPS e imágenes pregrabados
- Cuando detecta punto para calibración: mantiene posición pero tiempo sigue avanzando
- Al reanudar: continúa desde siguiente dato en tiempo actual
- Sin saltos, sin giros no deseados
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, CameraInfo, Image
from std_msgs.msg import Bool, String
import csv
import os
from pathlib import Path
import time
import numpy as np
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

class DataReplayer(Node):
    def __init__(self):
        super().__init__('data_replayer')
        
        # Parámetros de configuración
        self.declare_parameter('replay_speed', 1.0)  # Velocidad de reproducción
        self.replay_speed = self.get_parameter('replay_speed').value
        
        # Estado de reproducción
        self.is_replaying = True
        self.calibration_ready = True#False
        self.waiting_for_calibration = False
        self.found_stop_point = False
        self.current_log_path = None
        self.start_time = None
        self.real_start_time = None
        
        # Para modo calibración
        self.calibration_start_time = None
        self.frozen_pose = None
        self.frozen_yaw = None
        self.calibration_duration = 0.0
        self.min_yaw_variation = 0.001
        
        # Para seguimiento de qué dato publicamos
        self.last_published_index = -1  # Último índice publicado
        
        # Buffer de datos
        self.sensor_data = []
        self.image_data = []
        self.left_image_data = []
        self.right_image_data = []
        
        # Índices actuales
        self.current_data_index = 0
        self.current_image_index = 0
        self.current_left_index = 0
        self.current_right_index = 0
        
        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, '/odom/microros', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/microros', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Publicadores para cámaras
        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.rgb_image_pub = self.create_publisher(Image, '/camera/rgb/image_raw', camera_qos)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)
        self.left_image_pub = self.create_publisher(Image, '/camera/rgb/left', camera_qos)
        self.left_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/left/camera_info', 10)
        self.right_image_pub = self.create_publisher(Image, '/camera/rgb/right', camera_qos)
        self.right_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/right/camera_info', 10)
        
        # Configuración de cámaras
        fx, fy, cx, cy = 574.1, 574.1, 320.0, 240.0
        
        self.rgb_camera_info = CameraInfo()
        self.rgb_camera_info.header.frame_id = "camera_rgb_optical_frame"
        self.rgb_camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.rgb_camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.rgb_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.rgb_camera_info.distortion_model = "plumb_bob"
        self.rgb_camera_info.height = 480
        self.rgb_camera_info.width = 640
        
        self.left_camera_info = CameraInfo()
        self.left_camera_info.header.frame_id = "camera_left_optical_frame"
        self.left_camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.left_camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.left_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.left_camera_info.distortion_model = "plumb_bob"
        self.left_camera_info.height = 480
        self.left_camera_info.width = 640
        
        self.right_camera_info = CameraInfo()
        self.right_camera_info.header.frame_id = "camera_right_optical_frame"
        self.right_camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.right_camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.right_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.right_camera_info.distortion_model = "plumb_bob"
        self.right_camera_info.height = 480
        self.right_camera_info.width = 640
        
        # Para procesamiento de imágenes
        self.bridge = CvBridge()
        
        # Temporizador para reproducción
        self.replay_timer = self.create_timer(0.01, self.replay_callback)  # 100 Hz
        self.replay_timer.cancel()
        
        # Suscriptores
        self.create_subscription(Bool, '/replay_enabled', self.replay_enabled_cb, 10)
        self.create_subscription(String, '/replay_log_path', self.replay_path_cb, 10)
        self.create_subscription(Bool, '/lane_topology/calibration_ready', self.calibration_ready_cb, 10)
        
        self.get_logger().info('DataReplayer CON CALIBRACIÓN inicializado - Esperando habilitación...')

    # =================== CALLBACKS ===================

    def calibration_ready_cb(self, msg):
        """Callback para recibir señal de calibración lista - CORREGIDO"""
        if msg.data and not self.calibration_ready:
            self.calibration_ready = True
            self.waiting_for_calibration = False
            
            # SOLUCIÓN CORRECTA: NO ajustar start_time
            # Solo reanudar desde el siguiente dato
            self.get_logger().info('✓ Calibración recibida - Reanudando reproducción normal')
            self.get_logger().info(f'Índice actual: {self.current_data_index}, Próximo dato: {self.current_data_index + 1}')
            
            # Limpiar estado de calibración
            self.calibration_start_time = None
            self.frozen_pose = None
            self.frozen_yaw = None
            self.calibration_duration = 0.0
            
            # IMPORTANTE: Resincronizar el tiempo real
            self.real_start_time = self.get_clock().now().nanoseconds * 1e-9

    def replay_enabled_cb(self, msg):
        """Callback para habilitar/deshabilitar reproducción"""
        if msg.data != self.is_replaying:
            self.is_replaying = msg.data
            
            if self.is_replaying:
                self.get_logger().info('Reproducción HABILITADA')
                if self.current_log_path and not self.sensor_data:
                    self._load_all_data()
                    if self.sensor_data:
                        self.replay_timer.reset()
                elif self.sensor_data:
                    self.replay_timer.reset()
            else:
                self.get_logger().info('Reproducción DESHABILITADA')
                self.replay_timer.cancel()
                self._reset_indices()

    def replay_path_cb(self, msg):
        """Callback para recibir la ruta de logging"""
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            self.get_logger().info(f'Ruta de datos recibida: {self.current_log_path}')
            
            if self.is_replaying and not self.sensor_data:
                self._load_all_data()
                if self.sensor_data:
                    self.replay_timer.reset()

    # =================== CARGA DE DATOS ===================

    def _load_all_data(self):
        """Carga todos los datos"""
        try:
            sensor_file = Path(self.current_log_path) / "raw_sensors.csv"
            if not sensor_file.exists():
                self.get_logger().error(f'Archivo no encontrado: {sensor_file}')
                return
                
            self._load_sensor_data(sensor_file)
            self._load_all_camera_data()
            
            # Resetear índices
            self._reset_indices()
            
            if self.sensor_data:
                self.start_time = self.sensor_data[0]['stamp']
                self.real_start_time = self.get_clock().now().nanoseconds * 1e-9
                
                self.get_logger().info(f'✅ Datos cargados:')
                self.get_logger().info(f'   - Sensores: {len(self.sensor_data)} muestras')
                self.get_logger().info(f'   - Imágenes centrales: {len(self.image_data)}')
                self.get_logger().info(f'   - Imágenes izquierdas: {len(self.left_image_data)}')
                self.get_logger().info(f'   - Imágenes derechas: {len(self.right_image_data)}')
                
        except Exception as e:
            self.get_logger().error(f'Error al cargar datos: {e}')

    def _reset_indices(self):
        """Resetea todos los índices a cero"""
        self.current_data_index = 0
        self.current_image_index = 0
        self.current_left_index = 0
        self.current_right_index = 0
        self.last_published_index = -1
        self.found_stop_point = False
        self.waiting_for_calibration = False
        self.calibration_ready = False
        self.calibration_start_time = None
        self.frozen_pose = None
        self.frozen_yaw = None
        self.calibration_duration = 0.0

    # =================== DETECCIÓN DE PUNTO DE CALIBRACIÓN ===================

    def _check_stop_condition(self, odom_data):
        """Verifica si es un buen punto para calibración"""
        # Verificar velocidad baja
        speed = 0.0
        if odom_data['vx'] is not None and odom_data['vy'] is not None:
            speed = math.sqrt(odom_data['vx']**2 + odom_data['vy']**2)
        
        if speed > 0.2:  # > 20 cm/s
            return False
        
        # Verificar que tengamos imágenes de al menos 2 cámaras
        active_count = 0
        current_timestamp = odom_data['stamp']
        
        # Verificar cámara central
        if self.image_data:
            for img_data in self.image_data:
                if abs(img_data['stamp'] - current_timestamp) <= 0.5:  # ±500ms
                    if Path(img_data['path']).exists():
                        active_count += 1
                        break
        
        # Verificar cámara izquierda
        if self.left_image_data:
            for img_data in self.left_image_data:
                if abs(img_data['stamp'] - current_timestamp) <= 0.5:
                    if Path(img_data['path']).exists():
                        active_count += 1
                        break
        
        # Verificar cámara derecha
        if self.right_image_data:
            for img_data in self.right_image_data:
                if abs(img_data['stamp'] - current_timestamp) <= 0.5:
                    if Path(img_data['path']).exists():
                        active_count += 1
                        break
        
        self.get_logger().info(f'Verificando calibración: velocidad={speed:.3f} m/s, cámaras={active_count}')
        return active_count >= 3

    def _extract_yaw_from_quaternion(self, qx, qy, qz, qw):
        """Extrae yaw de un cuaternión"""
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def _quaternion_from_yaw(self, yaw):
        """Crea cuaternión desde yaw (roll=0, pitch=0)"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (0.0, 0.0, sy, cy)  # (x, y, z, w)

    # =================== REPRODUCCIÓN CON CALIBRACIÓN ===================

    def replay_callback(self):
        """Callback principal de reproducción - CORREGIDO"""
        if not self.sensor_data:
            return
        
        # Si estamos en modo calibración
        if self.waiting_for_calibration:
            self._handle_calibration_mode()
            return
        
        # Si terminamos los datos
        if self.current_data_index >= len(self.sensor_data):
            self.get_logger().info('✅ Reproducción completada')
            self.replay_timer.cancel()
            return
        
        # Si estamos recién saliendo de calibración, publicar inmediatamente
        if self.calibration_ready and self.current_data_index > 0:
            self._publish_next_data_after_calibration()
            self.calibration_ready = False
            return
        
        current_time = self.get_current_time()
        if current_time is None:
            return
        
        # Publicar datos de sensores hasta el tiempo actual
        while (self.current_data_index < len(self.sensor_data) and 
               self.sensor_data[self.current_data_index]['stamp'] <= current_time):
            
            data = self.sensor_data[self.current_data_index]
            
            # Verificar si es un buen punto para calibración
            if not self.found_stop_point and data['topic'] == 'odom':
                if self._check_stop_condition(data):
                    self.get_logger().info(f'🎯 Punto encontrado para calibración en índice {self.current_data_index}')
                    self.get_logger().info(f'   Tiempo: {data["stamp"]:.3f}s')
                    self.get_logger().info(f'   Posición: ({data["x"]:.2f}, {data["y"]:.2f})')
                    
                    # Guardar estado para calibración
                    self.found_stop_point = True
                    self.waiting_for_calibration = True
                    self.calibration_start_time = self.get_clock().now().nanoseconds * 1e-9
                    
                    # Guardar posición congelada
                    if all(v is not None for v in [data['x'], data['y'], data['z']]):
                        self.frozen_pose = (data['x'], data['y'], data['z'])
                    
                    # Guardar yaw base
                    if all(v is not None for v in [data['qx'], data['qy'], data['qz'], data['qw']]):
                        self.frozen_yaw = self._extract_yaw_from_quaternion(
                            data['qx'], data['qy'], data['qz'], data['qw']
                        )
                    else:
                        self.frozen_yaw = 0.0
                    
                    self.get_logger().info(f'   Yaw base: {math.degrees(self.frozen_yaw):.2f}°')
                    self.get_logger().info('🔧 Modo CALIBRACIÓN activo - Manteniendo posición con tiempo vivo')
                    
                    # Publicar este dato como inicio de calibración
                    self._publish_odom(data)
                    self.last_published_index = self.current_data_index
                    
                    # NO incrementar current_data_index - nos quedamos aquí
                    return
            
            # Publicar según tipo de dato
            if data['topic'] == 'odom':
                self._publish_odom(data)
            elif data['topic'] == 'imu':
                self._publish_imu(data)
            elif data['topic'] == 'gps':
                self._publish_gps(data)
            
            self.last_published_index = self.current_data_index
            self.current_data_index += 1
        
        # Publicar imágenes
        self._publish_camera_images(current_time)

    def _publish_next_data_after_calibration(self):
        """Publica el siguiente dato inmediatamente después de calibración"""
        if self.current_data_index >= len(self.sensor_data):
            return
        
        data = self.sensor_data[self.current_data_index]
        
        # Publicar según tipo de dato
        if data['topic'] == 'odom':
            self._publish_odom(data)
        elif data['topic'] == 'imu':
            self._publish_imu(data)
        elif data['topic'] == 'gps':
            self._publish_gps(data)
        
        self.last_published_index = self.current_data_index
        self.current_data_index += 1
        
        self.get_logger().info(f'Publicado dato {self.last_published_index} después de calibración')

    def _handle_calibration_mode(self):
        """Maneja el modo de calibración - tiempo vivo, posición fija"""
        if not self.waiting_for_calibration:
            return
        
        # Calcular duración de calibración
        current_real_time = self.get_clock().now().nanoseconds * 1e-9
        self.calibration_duration = current_real_time - self.calibration_start_time
        
        # Publicar odometría de calibración
        self._publish_calibration_odom()
        
        # Publicar IMU y GPS también durante calibración
        self._publish_calibration_other_sensors()
        
        # MODIFICACIÓN: Publicar imágenes periódicamente durante toda la calibración
        # Usar un intervalo fijo (ej: 10Hz = 0.1s) para no saturar
        if not hasattr(self, '_last_image_publish_time'):
            self._last_image_publish_time = 0.0
        
        # Publicar imágenes cada 0.1 segundos (10Hz)
        if self.calibration_duration - self._last_image_publish_time >= 0.1:
            if 0 <= self.current_data_index < len(self.sensor_data):
                calib_timestamp = self.sensor_data[self.current_data_index]['stamp']
                self._publish_calibration_images(calib_timestamp)
                self._last_image_publish_time = self.calibration_duration
        
        # Log cada 2 segundos (opcional)
        if int(self.calibration_duration) % 2 == 0:
            self.get_logger().info(f'🔧 Calibración: {self.calibration_duration:.1f}s')

    def _publish_calibration_odom(self):
        """Publica odometría durante calibración - posición fija, yaw con variación mínima"""
        if self.frozen_pose is None or self.frozen_yaw is None:
            return
        
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        
        # Posición fija
        x, y, z = self.frozen_pose
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        
        # Yaw con variación mínima
        variation = math.sin(self.calibration_duration * 0.1) * self.min_yaw_variation
        current_yaw = self.frozen_yaw + variation
        
        qx, qy, qz, qw = self._quaternion_from_yaw(current_yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        
        # Velocidad CERO
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0
        
        # Covarianzas pequeñas
        pose_covariance = [
            0.001, 0.0,  0.0,  0.0,   0.0,   0.0,
            0.0,  0.001, 0.0,  0.0,   0.0,   0.0,
            0.0,  0.0,  0.001, 0.0,   0.0,   0.0,
            0.0,  0.0,  0.0,  0.001,  0.0,   0.0,
            0.0,  0.0,  0.0,  0.0,   0.001,  0.0,
            0.0,  0.0,  0.0,  0.0,   0.0,   0.001
        ]
        
        twist_covariance = [
            0.001, 0.0,  0.0,  0.0,   0.0,   0.0,
            0.0,  0.001, 0.0,  0.0,   0.0,   0.0,
            0.0,  0.0,  0.001, 0.0,   0.0,   0.0,
            0.0,  0.0,  0.0,  0.001,  0.0,   0.0,
            0.0,  0.0,  0.0,  0.0,   0.001,  0.0,
            0.0,  0.0,  0.0,  0.0,   0.0,   0.001
        ]
        
        msg.pose.covariance = pose_covariance
        msg.twist.covariance = twist_covariance
        
        self.odom_pub.publish(msg)

    def _publish_calibration_other_sensors(self):
        """Publica IMU y GPS durante calibración"""
        if 0 <= self.current_data_index < len(self.sensor_data):
            calib_timestamp = self.sensor_data[self.current_data_index]['stamp']
            
            # Buscar IMU cercano
            for i in range(max(0, self.current_data_index - 10), min(len(self.sensor_data), self.current_data_index + 10)):
                data = self.sensor_data[i]
                if data['topic'] == 'imu' and abs(data['stamp'] - calib_timestamp) < 0.1:
                    self._publish_imu(data)
                    break
            
            # Buscar GPS cercano
            for i in range(max(0, self.current_data_index - 10), min(len(self.sensor_data), self.current_data_index + 10)):
                data = self.sensor_data[i]
                if data['topic'] == 'gps' and abs(data['stamp'] - calib_timestamp) < 0.5:
                    self._publish_gps(data)
                    break

    # =================== PUBLICACIÓN DE IMÁGENES ===================

    def _publish_calibration_images(self, timestamp):
        """Publica imágenes durante calibración"""
        # Publicar imagen central
        if self.image_data:
            closest_idx = self._find_closest_image(self.image_data, timestamp)
            if closest_idx >= 0:
                self._publish_single_image(self.image_data[closest_idx], 'central')
        
        # Publicar imagen izquierda
        if self.left_image_data:
            closest_idx = self._find_closest_image(self.left_image_data, timestamp)
            if closest_idx >= 0:
                self._publish_single_image(self.left_image_data[closest_idx], 'left')
        
        # Publicar imagen derecha
        if self.right_image_data:
            closest_idx = self._find_closest_image(self.right_image_data, timestamp)
            if closest_idx >= 0:
                self._publish_single_image(self.right_image_data[closest_idx], 'right')

    def _find_closest_image(self, image_list, timestamp):
        """Búsqueda binaria más eficiente"""
        if not image_list:
            return -1
        
        low, high = 0, len(image_list) - 1
        
        while low <= high:
            mid = (low + high) // 2
            mid_time = image_list[mid]['stamp']
            
            if abs(mid_time - timestamp) <= 0.05:  # ±50ms
                return mid
            elif mid_time < timestamp:
                low = mid + 1
            else:
                high = mid - 1
        
        # Si no encontramos exacto, devolver el más cercano
        if low < len(image_list) and abs(image_list[low]['stamp'] - timestamp) < 0.5:
            return low
        elif high >= 0 and abs(image_list[high]['stamp'] - timestamp) < 0.5:
            return high
        
        return -1

    def _publish_single_image(self, img_data, camera_name):
        """Publica una imagen específica"""
        try:
            cv_image = cv2.imread(img_data['path'])
            if cv_image is None:
                self.get_logger().warning(f'No se pudo cargar imagen: {img_data["path"]}')
                return
            
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = f'camera_{camera_name}_optical_frame'
            
            # Publicar imagen
            if camera_name == 'central':
                self.rgb_image_pub.publish(img_msg)
                self.rgb_camera_info.header.stamp = img_msg.header.stamp
                self.rgb_info_pub.publish(self.rgb_camera_info)
            elif camera_name == 'left':
                self.left_image_pub.publish(img_msg)
                self.left_camera_info.header.stamp = img_msg.header.stamp
                self.left_info_pub.publish(self.left_camera_info)
            elif camera_name == 'right':
                self.right_image_pub.publish(img_msg)
                self.right_camera_info.header.stamp = img_msg.header.stamp
                self.right_info_pub.publish(self.right_camera_info)
                
        except Exception as e:
            self.get_logger().debug(f'Error al publicar imagen {camera_name}: {e}')

    def _publish_camera_images(self, current_time):
        """Publica imágenes de todas las cámaras con frecuencia constante"""
        
        # Usar un tiempo de publicación mínimo (ej: 10Hz = 0.1s)
        current_clock = self.get_clock().now()
        if not hasattr(self, '_last_camera_publish'):
            self._last_camera_publish = {}
        
        # Solo publicar si ha pasado el intervalo mínimo para ALGUNA cámara
        should_publish = False
        camera_names = ['central', 'left', 'right']
        
        for cam_name in camera_names:
            if cam_name not in self._last_camera_publish:
                self._last_camera_publish[cam_name] = current_clock
                should_publish = True
            
            # Verificar si ha pasado el intervalo mínimo
            elapsed = (current_clock - self._last_camera_publish[cam_name]).nanoseconds * 1e-9
            if elapsed >= 0.1:  # 10Hz mínimo
                should_publish = True
                break
        
        if should_publish:
            # Imágenes centrales
            self._publish_camera_images_simple(
                current_time, self.image_data, self.current_image_index,
                self.rgb_image_pub, self.rgb_camera_info, "central"
            )
            
            # Imágenes izquierdas
            self._publish_camera_images_simple(
                current_time, self.left_image_data, self.current_left_index,
                self.left_image_pub, self.left_camera_info, "left"
            )
            
            # Imágenes derechas
            self._publish_camera_images_simple(
                current_time, self.right_image_data, self.current_right_index,
                self.right_image_pub, self.right_camera_info, "right"
            )
            
            # Actualizar tiempos de todas las cámaras
            for cam_name in camera_names:
                self._last_camera_publish[cam_name] = current_clock

    def _publish_camera_images_simple(self, current_time, image_list, current_index, 
                                 publisher, camera_info, camera_name):
        """Publica imágenes de una cámara"""
        if not image_list or current_index >= len(image_list):
            return
        
        # MODIFICACIÓN: Publicar todas las imágenes que correspondan al tiempo actual
        images_to_publish = []
        
        # Recopilar imágenes para este tiempo
        while (current_index < len(image_list) and 
            image_list[current_index]['stamp'] <= current_time):
            
            # Permitir un margen de ±0.05s para sincronización
            if abs(image_list[current_index]['stamp'] - current_time) <= 0.05:
                images_to_publish.append(image_list[current_index])
            
            current_index += 1
        
        # Publicar la imagen más cercana si encontramos alguna
        if images_to_publish:
            # Tomar la más cercana en tiempo
            closest = min(images_to_publish, key=lambda x: abs(x['stamp'] - current_time))
            self._publish_single_image(closest, camera_name)
            
            # Actualizar índice
            if camera_name == "central":
                self.current_image_index = current_index
            elif camera_name == "left":
                self.current_left_index = current_index
            elif camera_name == "right":
                self.current_right_index = current_index

    # =================== MÉTODOS DE CARGA ===================

    def _load_sensor_data(self, csv_file):
        """Carga datos de sensores desde CSV"""
        self.sensor_data = []
        
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            fieldnames = reader.fieldnames
            
            for row in reader:
                data = {
                    'stamp': float(row['stamp']),
                    'topic': row['topic'],
                    'x': float(row['x']) if row['x'] != '' and row['x'] != 'None' else None,
                    'y': float(row['y']) if row['y'] != '' and row['y'] != 'None' else None,
                    'z': float(row['z']) if row['z'] != '' and row['z'] != 'None' else None,
                    'qx': float(row['qx']) if row['qx'] != '' and row['qx'] != 'None' else None,
                    'qy': float(row['qy']) if row['qy'] != '' and row['qy'] != 'None' else None,
                    'qz': float(row['qz']) if row['qz'] != '' and row['qz'] != 'None' else None,
                    'qw': float(row['qw']) if row['qw'] != '' and row['qw'] != 'None' else None,
                    'vx': float(row['vx']) if row['vx'] != '' and row['vx'] != 'None' else None,
                    'vy': float(row['vy']) if row['vy'] != '' and row['vy'] != 'None' else None,
                    'vz': float(row['vz']) if row['vz'] != '' and row['vz'] != 'None' else None,
                    'wx': float(row['wx']) if row['wx'] != '' and row['wx'] != 'None' else None,
                    'wy': float(row['wy']) if row['wy'] != '' and row['wy'] != 'None' else None,
                    'wz': float(row['wz']) if row['wz'] != '' and row['wz'] != 'None' else None,
                    'ax': float(row['ax']) if row['ax'] != '' and row['ax'] != 'None' else None,
                    'ay': float(row['ay']) if row['ay'] != '' and row['ay'] != 'None' else None,
                    'az': float(row['az']) if row['az'] != '' and row['az'] != 'None' else None,
                    'lat': float(row['lat']) if row['lat'] != '' and row['lat'] != 'None' else None,
                    'lon': float(row['lon']) if row['lon'] != '' and row['lon'] != 'None' else None,
                    'alt': float(row['alt']) if row['alt'] != '' and row['alt'] != 'None' else None,
                }
                
                # Añadir covarianzas solo si existen
                if 'cov_lat' in fieldnames:
                    data['cov_lat'] = float(row['cov_lat']) if row['cov_lat'] != '' and row['cov_lat'] != 'None' else None
                if 'cov_lon' in fieldnames:
                    data['cov_lon'] = float(row['cov_lon']) if row['cov_lon'] != '' and row['cov_lon'] != 'None' else None
                if 'cov_alt' in fieldnames:
                    data['cov_alt'] = float(row['cov_alt']) if row['cov_alt'] != '' and row['cov_alt'] != 'None' else None
                
                self.sensor_data.append(data)
        
        self.get_logger().info(f'Cargados {len(self.sensor_data)} registros de sensores')

    def _load_all_camera_data(self):
        """Carga datos de todas las cámaras"""
        base_path = Path(self.current_log_path)
        
        # Cargar cámara central
        perception_dir = base_path / "perception"
        if perception_dir.exists():
            image_dir = perception_dir / "images"
            if image_dir.exists():
                self._load_camera_data_simple(image_dir, "central", self.image_data)
        
        # Cargar cámara izquierda
        perception_left_dir = base_path / "perception_left"
        if perception_left_dir.exists():
            left_image_dir = perception_left_dir / "images"
            if left_image_dir.exists():
                self._load_camera_data_simple(left_image_dir, "left", self.left_image_data)
        
        # Cargar cámara derecha
        perception_right_dir = base_path / "perception_right"
        if perception_right_dir.exists():
            right_image_dir = perception_right_dir / "images"
            if right_image_dir.exists():
                self._load_camera_data_simple(right_image_dir, "right", self.right_image_data)

    def _load_camera_data_simple(self, image_dir, camera_type, data_list):
        """Carga metadatos de imágenes"""
        try:
            perception_csv = image_dir.parent / "perception_data.csv"
            
            if perception_csv.exists():
                with open(perception_csv, 'r') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        try:
                            image_filename = row.get('image_filename', '')
                            timestamp_str = row.get('timestamp', '')
                            
                            if image_filename and timestamp_str:
                                timestamp = float(timestamp_str)
                                image_path = image_dir / image_filename
                                
                                if image_path.exists():
                                    data_list.append({
                                        'stamp': timestamp,
                                        'path': str(image_path),
                                        'filename': image_filename,
                                        'camera': camera_type
                                    })
                        except Exception as e:
                            continue
            
            # Si no hay CSV, buscar imágenes directamente
            if not data_list:
                image_extensions = ['.jpg', '.jpeg', '.png']
                image_files = []
                
                for ext in image_extensions:
                    image_files.extend(image_dir.glob(f'*{ext}'))
                
                for img_file in image_files:
                    try:
                        timestamp = self.start_time + len(data_list) * 0.2
                        
                        data_list.append({
                            'stamp': timestamp,
                            'path': str(img_file),
                            'filename': img_file.name,
                            'camera': camera_type
                        })
                    except Exception as e:
                        continue
            
            data_list.sort(key=lambda x: x['stamp'])
            self.get_logger().info(f'✅ Cargadas {len(data_list)} imágenes de cámara {camera_type}')
            
        except Exception as e:
            self.get_logger().error(f'Error cargando imágenes {camera_type}: {e}')

    # =================== UTILIDADES ===================

    def get_current_time(self):
        """Obtiene el tiempo actual de reproducción"""
        if self.real_start_time is None or self.start_time is None:
            return None
            
        current_real_time = self.get_clock().now().nanoseconds * 1e-9
        elapsed_real = current_real_time - self.real_start_time
        elapsed_simulated = elapsed_real * self.replay_speed
        
        return self.start_time + elapsed_simulated

    # =================== MÉTODOS DE PUBLICACIÓN ===================

    def _publish_odom(self, data):
        """Publica mensaje de odometría"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        
        # Posición
        if all(v is not None for v in [data['x'], data['y'], data['z']]):
            msg.pose.pose.position.x = data['x']
            msg.pose.pose.position.y = data['y']
            msg.pose.pose.position.z = data['z']
        
        # Orientación
        if all(v is not None for v in [data['qx'], data['qy'], data['qz'], data['qw']]):
            msg.pose.pose.orientation.x = data['qx']
            msg.pose.pose.orientation.y = data['qy']
            msg.pose.pose.orientation.z = data['qz']
            msg.pose.pose.orientation.w = data['qw']
        else:
            msg.pose.pose.orientation.w = 1.0
        
        # Velocidad lineal
        if all(v is not None for v in [data['vx'], data['vy'], data['vz']]):
            msg.twist.twist.linear.x = data['vx']
            msg.twist.twist.linear.y = data['vy']
            msg.twist.twist.linear.z = data['vz']
        
        # Velocidad angular
        if all(v is not None for v in [data['wx'], data['wy'], data['wz']]):
            msg.twist.twist.angular.x = data['wx']
            msg.twist.twist.angular.y = data['wy']
            msg.twist.twist.angular.z = data['wz']
        
        # Covarianzas
        pose_covariance = [
            0.01, 0.0,  0.0,  0.0,   0.0,   0.0,
            0.0,  0.01, 0.0,  0.0,   0.0,   0.0,
            0.0,  0.0,  0.05, 0.0,   0.0,   0.0,
            0.0,  0.0,  0.0,  0.05,  0.0,   0.0,
            0.0,  0.0,  0.0,  0.0,   0.05,  0.0,
            0.0,  0.0,  0.0,  0.0,   0.0,   0.02
        ]
        
        twist_covariance = [
            0.05, 0.0,  0.0,  0.0,   0.0,   0.0,
            0.0,  0.1,  0.0,  0.0,   0.0,   0.0,
            0.0,  0.0,  0.1,  0.0,   0.0,   0.0,
            0.0,  0.0,  0.0,  0.02,  0.0,   0.0,
            0.0,  0.0,  0.0,  0.0,   0.02,  0.0,
            0.0,  0.0,  0.0,  0.0,   0.0,   0.01
        ]
        
        msg.pose.covariance = pose_covariance
        msg.twist.covariance = twist_covariance
        
        self.odom_pub.publish(msg)

    def _publish_imu(self, data):
        """Publica mensaje IMU"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        if all(v is not None for v in [data['qx'], data['qy'], data['qz'], data['qw']]):
            msg.orientation.x = data['qx']
            msg.orientation.y = data['qy']
            msg.orientation.z = data['qz']
            msg.orientation.w = data['qw']
        
        if all(v is not None for v in [data['wx'], data['wy'], data['wz']]):
            msg.angular_velocity.x = data['wx']
            msg.angular_velocity.y = data['wy']
            msg.angular_velocity.z = data['wz']
        
        if all(v is not None for v in [data['ax'], data['ay'], data['az']]):
            msg.linear_acceleration.x = data['ax']
            msg.linear_acceleration.y = data['ay']
            msg.linear_acceleration.z = data['az']
        
        msg.orientation_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
        msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        
        self.imu_pub.publish(msg)

    def _publish_gps(self, data):
        """Publica mensaje GPS"""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gnss_frame'
        
        if all(v is not None for v in [data['lat'], data['lon'], data['alt']]):
            msg.latitude = data['lat']
            msg.longitude = data['lon']
            msg.altitude = data['alt']
            
            cov_matrix = [0.0] * 9
            if 'cov_lat' in data and data['cov_lat'] is not None:
                cov_matrix[0] = data['cov_lat']
            if 'cov_lon' in data and data['cov_lon'] is not None:
                cov_matrix[4] = data['cov_lon']
            if 'cov_alt' in data and data['cov_alt'] is not None:
                cov_matrix[8] = data['cov_alt']
            
            msg.position_covariance = cov_matrix
            msg.position_covariance_type = 2
        
        self.gps_pub.publish(msg)

    def destroy_node(self):
        """Cleanup"""
        self.replay_timer.cancel()
        super().destroy_node()

def main():
    rclpy.init()
    node = DataReplayer()

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