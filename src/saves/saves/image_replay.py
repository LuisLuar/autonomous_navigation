#!/usr/bin/env python3
"""
DataReplayer CON CALIBRACIÓN SIMPLIFICADA - Repite datos con tiempo vivo
========================================================================
- Reproduce odometría, IMU, GPS e imágenes pregrabados (solo cámara frontal)
- Cuando calibration_mode = True: mantiene la primera imagen publicando a 30fps
- Cuando calibration_mode = False: publica en orden de timestamp
- Funciona con cualquier combinación de datos (solo imágenes, solo sensores, o ambos)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, CameraInfo, CompressedImage
from std_msgs.msg import String
import csv
from pathlib import Path
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

class DataReplayer(Node):
    def __init__(self):
        super().__init__('data_replayer')
        
        # Parámetros de configuración
        self.declare_parameter('replay_speed', 1.0)
        self.replay_speed = self.get_parameter('replay_speed').value
        
        # Parámetro para modo calibración
        self.declare_parameter('calibration_mode', False)
        self.calibration_mode = self.get_parameter('calibration_mode').value
        
        # Estado de reproducción
        self.current_log_path = None
        self.start_time = None
        self.real_start_time = None
        self.has_sensors = False
        self.has_images = False
        
        # Para modo calibración
        self.first_image_path = None
        self.first_image_timestamp = None
        self.calibration_active = False
        
        # Buffer de datos
        self.sensor_data = []
        self.image_data = []  # Solo cámara frontal
        
        # Índices actuales
        self.current_data_index = 0
        self.current_image_index = 0
        
        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, '/odom/microros', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/microros', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Publicador para cámara frontal (best effort)
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.image_pub = self.create_publisher(CompressedImage, '/image_raw/compressed', camera_qos)
        
        # Temporizadores
        self.replay_timer = self.create_timer(0.01, self.replay_callback)  # 100 Hz
        self.replay_timer.cancel()
        
        self.calibration_timer = self.create_timer(0.033, self.calibration_callback)  # 30 Hz
        self.calibration_timer.cancel()
        
        # Suscriptor único para la ruta
        self.create_subscription(String, '/replay_log_path', self.replay_path_cb, 10)
        
        # Parámetros de compresión JPEG
        self.compression_params = [cv2.IMWRITE_JPEG_QUALITY, 85]
        
        self.get_logger().info('DataReplayer CON CALIBRACIÓN SIMPLIFICADA inicializado')
        self.get_logger().info(f'Modo calibración: {self.calibration_mode}')

    def replay_path_cb(self, msg):
        """Callback para recibir la ruta y comenzar automáticamente"""
        if msg.data == self.current_log_path:
            return
            
        self.current_log_path = msg.data
        self.get_logger().info(f'Ruta recibida: {self.current_log_path}')
        
        # Cargar datos y comenzar reproducción
        self._load_all_data()
        self._start_reproduction()

    def _start_reproduction(self):
        """Inicia la reproducción según el modo y datos disponibles"""
        # Configurar tiempo base
        self._setup_time_reference()
        
        # Resetear índices
        self.current_data_index = 0
        self.current_image_index = 0
        
        # Iniciar según modo
        if self.calibration_mode and self.has_images:
            self.calibration_active = True
            self.calibration_timer.reset()
            self.get_logger().info('▶️ Modo CALIBRACIÓN activado - Publicando primera imagen a 30fps')
        elif self.has_sensors or self.has_images:
            self.calibration_active = False
            self.replay_timer.reset()
            self.get_logger().info('▶️ Modo NORMAL activado - Publicando datos en orden temporal')
        else:
            #self.get_logger().error('❌ No hay datos para reproducir')
            pass

    def _setup_time_reference(self):
        """Establece la referencia de tiempo basada en los datos disponibles"""
        # Prioridad: sensores > imágenes
        if self.has_sensors and self.sensor_data:
            self.start_time = self.sensor_data[0]['stamp']
            self.get_logger().info(f'Referencia temporal: primer sensor en t={self.start_time:.3f}s')
        elif self.has_images and self.image_data:
            self.start_time = self.image_data[0]['stamp']
            self.get_logger().info(f'Referencia temporal: primera imagen en t={self.start_time:.3f}s')
        else:
            self.start_time = 0.0
            self.get_logger().info('Referencia temporal: 0.0 (sin datos)')
        
        self.real_start_time = self.get_clock().now().nanoseconds * 1e-9

    def _load_all_data(self):
        """Carga todos los datos disponibles (sensores e imágenes)"""
        try:
            # 1. Cargar sensores (si existen)
            sensor_file = Path(self.current_log_path) / "raw_sensors.csv"
            if sensor_file.exists():
                self._load_sensor_data(sensor_file)
                self.has_sensors = len(self.sensor_data) > 0
            else:
                self.get_logger().warning('Archivo de sensores no encontrado - Solo se publicarán imágenes')
                self.sensor_data = []
                self.has_sensors = False
            
            # 2. Cargar imágenes (si existen)
            self._load_camera_data()
            self.has_images = len(self.image_data) > 0
            
            # 3. Guardar primera imagen para modo calibración
            if self.has_images:
                self.first_image_path = self.image_data[0]['path']
                self.first_image_timestamp = self.image_data[0]['stamp']
            
            # 4. Reportar estado
            self.get_logger().info('📊 DATOS CARGADOS:')
            self.get_logger().info(f'   - Sensores: {len(self.sensor_data)} muestras')
            self.get_logger().info(f'   - Imágenes: {len(self.image_data)} archivos')
            
        except Exception as e:
            #self.get_logger().error(f'Error al cargar datos: {e}")
            pass

    # =================== REPRODUCCIÓN NORMAL (POR TIMESTAMP) ===================

    def replay_callback(self):
        """Publica datos en orden de timestamp"""
        current_time = self._get_current_time()
        if current_time is None:
            return
        
        # Publicar sensores (si hay)
        if self.has_sensors:
            self._publish_sensors_until_time(current_time)
        
        # Publicar imágenes (si hay y no estamos en calibración)
        if self.has_images and not self.calibration_active:
            self._publish_images_until_time(current_time)
        
        # Verificar si terminó
        sensors_finished = not self.has_sensors or self.current_data_index >= len(self.sensor_data)
        images_finished = not self.has_images or self.current_image_index >= len(self.image_data)
        
        if sensors_finished and images_finished:
            self.get_logger().info('✅ Reproducción completada')
            self.replay_timer.cancel()

    def _publish_sensors_until_time(self, current_time):
        """Publica sensores hasta el tiempo actual"""
        while (self.current_data_index < len(self.sensor_data) and 
               self.sensor_data[self.current_data_index]['stamp'] <= current_time):
            
            data = self.sensor_data[self.current_data_index]
            
            if data['topic'] == 'odom':
                self._publish_odom(data)
            elif data['topic'] == 'imu':
                self._publish_imu(data)
            elif data['topic'] == 'gps':
                self._publish_gps(data)
            
            self.current_data_index += 1

    def _publish_images_until_time(self, current_time):
        """Publica imágenes hasta el tiempo actual"""
        images_to_publish = []
        
        while (self.current_image_index < len(self.image_data) and 
               self.image_data[self.current_image_index]['stamp'] <= current_time):
            
            if abs(self.image_data[self.current_image_index]['stamp'] - current_time) <= 0.05:
                images_to_publish.append(self.image_data[self.current_image_index])
            
            self.current_image_index += 1
        
        if images_to_publish:
            closest = min(images_to_publish, key=lambda x: abs(x['stamp'] - current_time))
            self._publish_image_compressed(closest)

    # =================== MODO CALIBRACIÓN (30FPS CONSTANTES) ===================

    def calibration_callback(self):
        """Publica primera imagen a 30fps (modo calibración)"""
        if not self.calibration_active or not self.first_image_path:
            return
        
        # Publicar imagen
        self._publish_calibration_image()
        
        # Publicar sensores congelados (si hay)
        if self.has_sensors:
            self._publish_frozen_sensors()

    def _publish_calibration_image(self):
        """Publica la primera imagen en formato comprimido"""
        try:
            cv_image = cv2.imread(self.first_image_path)
            if cv_image is None:
                return
            
            # Redimensionar a 640x360
            if cv_image.shape[1] != 640 or cv_image.shape[0] != 360:
                cv_image = cv2.resize(cv_image, (640, 360), interpolation=cv2.INTER_AREA)
            
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            msg.format = 'jpeg'
            
            success, encoded = cv2.imencode('.jpg', cv_image, self.compression_params)
            if success:
                msg.data = encoded.tobytes()
                self.image_pub.publish(msg)
                
        except Exception as e:
            #self.get_logger().debug(f'Error en imagen calibración: {e}")
            pass

    def _publish_frozen_sensors(self):
        """Publica sensores congelados en la primera posición"""
        if not self.sensor_data:
            return
        
        # Buscar primer odom
        for data in self.sensor_data:
            if data['topic'] == 'odom':
                self._publish_frozen_odom(data)
                break
        
        # Buscar primer IMU y GPS (en los primeros 10 datos)
        for data in self.sensor_data[:10]:
            if data['topic'] == 'imu':
                self._publish_imu(data)
            elif data['topic'] == 'gps':
                self._publish_gps(data)

    def _publish_frozen_odom(self, data):
        """Publica odometría congelada"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        
        # Posición fija
        if all(v is not None for v in [data['x'], data['y'], data['z']]):
            msg.pose.pose.position.x = data['x']
            msg.pose.pose.position.y = data['y']
            msg.pose.pose.position.z = data['z']
        
        # Orientación fija
        if all(v is not None for v in [data['qx'], data['qy'], data['qz'], data['qw']]):
            msg.pose.pose.orientation.x = data['qx']
            msg.pose.pose.orientation.y = data['qy']
            msg.pose.pose.orientation.z = data['qz']
            msg.pose.pose.orientation.w = data['qw']
        
        # Velocidad cero
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0
        
        msg.pose.covariance = [0.001] * 36
        msg.twist.covariance = [0.001] * 36
        
        self.odom_pub.publish(msg)

    # =================== PUBLICACIÓN DE IMÁGENES ===================

    def _publish_image_compressed(self, img_data):
        """Publica una imagen en formato comprimido"""
        try:
            cv_image = cv2.imread(img_data['path'])
            if cv_image is None:
                return
            
            if cv_image.shape[1] != 640 or cv_image.shape[0] != 360:
                cv_image = cv2.resize(cv_image, (640, 360), interpolation=cv2.INTER_AREA)
            
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            msg.format = 'jpeg'
            
            success, encoded = cv2.imencode('.jpg', cv_image, self.compression_params)
            if success:
                msg.data = encoded.tobytes()
                self.image_pub.publish(msg)
                
        except Exception as e:
            #self.get_logger().debug(f'Error publicando imagen: {e}")
            pass

    # =================== CARGA DE DATOS ===================

    def _load_sensor_data(self, csv_file):
        """Carga datos de sensores"""
        self.sensor_data = []
        
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                data = {
                    'stamp': float(row['stamp']),
                    'topic': row['topic'],
                    'x': float(row['x']) if row['x'] not in ('', 'None') else None,
                    'y': float(row['y']) if row['y'] not in ('', 'None') else None,
                    'z': float(row['z']) if row['z'] not in ('', 'None') else None,
                    'qx': float(row['qx']) if row['qx'] not in ('', 'None') else None,
                    'qy': float(row['qy']) if row['qy'] not in ('', 'None') else None,
                    'qz': float(row['qz']) if row['qz'] not in ('', 'None') else None,
                    'qw': float(row['qw']) if row['qw'] not in ('', 'None') else None,
                    'vx': float(row['vx']) if row['vx'] not in ('', 'None') else None,
                    'vy': float(row['vy']) if row['vy'] not in ('', 'None') else None,
                    'vz': float(row['vz']) if row['vz'] not in ('', 'None') else None,
                    'wx': float(row['wx']) if row['wx'] not in ('', 'None') else None,
                    'wy': float(row['wy']) if row['wy'] not in ('', 'None') else None,
                    'wz': float(row['wz']) if row['wz'] not in ('', 'None') else None,
                    'ax': float(row['ax']) if row['ax'] not in ('', 'None') else None,
                    'ay': float(row['ay']) if row['ay'] not in ('', 'None') else None,
                    'az': float(row['az']) if row['az'] not in ('', 'None') else None,
                    'lat': float(row['lat']) if row['lat'] not in ('', 'None') else None,
                    'lon': float(row['lon']) if row['lon'] not in ('', 'None') else None,
                    'alt': float(row['alt']) if row['alt'] not in ('', 'None') else None,
                }
                self.sensor_data.append(data)
        
        self.sensor_data.sort(key=lambda x: x['stamp'])

    def _load_camera_data(self):
        """Carga datos de la cámara frontal"""
        self.image_data = []
        base_path = Path(self.current_log_path)
        
        # Buscar en carpetas posibles
        possible_paths = [
            base_path / "perception" / "images",
            base_path / "perception_frontal" / "images"
        ]
        
        for image_dir in possible_paths:
            if image_dir.exists():
                self._load_images_from_dir(image_dir)
                if self.image_data:
                    break
        
        self.image_data.sort(key=lambda x: x['stamp'])

    def _load_images_from_dir(self, image_dir):
        """Carga imágenes de un directorio"""
        # Intentar cargar desde CSV primero
        csv_file = image_dir.parent / "perception_data.csv"
        if csv_file.exists():
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        img_file = row.get('image_filename', '')
                        timestamp = float(row.get('timestamp', 0))
                        
                        if img_file and timestamp:
                            img_path = image_dir / img_file
                            if img_path.exists():
                                self.image_data.append({
                                    'stamp': timestamp,
                                    'path': str(img_path)
                                })
                    except:
                        continue
        
        # Si no hay CSV, cargar por nombre de archivo
        if not self.image_data:
            extensions = ['.jpg', '.jpeg', '.png']
            images = []
            for ext in extensions:
                images.extend(image_dir.glob(f'*{ext}'))
            
            images.sort()
            for i, img_path in enumerate(images):
                # Intentar extraer timestamp del nombre
                try:
                    name = img_path.stem
                    timestamp = float(name) if name.replace('.', '').isdigit() else i * 0.1
                except:
                    timestamp = i * 0.1
                
                self.image_data.append({
                    'stamp': timestamp,
                    'path': str(img_path)
                })

    # =================== UTILIDADES ===================

    def _get_current_time(self):
        """Obtiene el tiempo actual de reproducción"""
        if self.real_start_time is None or self.start_time is None:
            return None
        
        elapsed = (self.get_clock().now().nanoseconds * 1e-9) - self.real_start_time
        return self.start_time + (elapsed * self.replay_speed)

    # =================== PUBLICACIÓN DE SENSORES ===================

    def _publish_odom(self, data):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        
        if data['x'] is not None: msg.pose.pose.position.x = data['x']
        if data['y'] is not None: msg.pose.pose.position.y = data['y']
        if data['z'] is not None: msg.pose.pose.position.z = data['z']
        
        if all(v is not None for v in [data['qx'], data['qy'], data['qz'], data['qw']]):
            msg.pose.pose.orientation.x = data['qx']
            msg.pose.pose.orientation.y = data['qy']
            msg.pose.pose.orientation.z = data['qz']
            msg.pose.pose.orientation.w = data['qw']
        
        if data['vx'] is not None: msg.twist.twist.linear.x = data['vx']
        if data['vy'] is not None: msg.twist.twist.linear.y = data['vy']
        if data['vz'] is not None: msg.twist.twist.linear.z = data['vz']
        
        if data['wx'] is not None: msg.twist.twist.angular.x = data['wx']
        if data['wy'] is not None: msg.twist.twist.angular.y = data['wy']
        if data['wz'] is not None: msg.twist.twist.angular.z = data['wz']
        
        self.odom_pub.publish(msg)

    def _publish_imu(self, data):
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
        
        self.imu_pub.publish(msg)

    def _publish_gps(self, data):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gnss_frame'
        
        if data['lat'] is not None: msg.latitude = data['lat']
        if data['lon'] is not None: msg.longitude = data['lon']
        if data['alt'] is not None: msg.altitude = data['alt']
        
        self.gps_pub.publish(msg)

def main():
    rclpy.init()
    node = DataReplayer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()