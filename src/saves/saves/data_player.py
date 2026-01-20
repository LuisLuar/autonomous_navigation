#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, CameraInfo, Image
from std_msgs.msg import Bool, String
import csv
import os
from pathlib import Path
import time
from collections import deque
import numpy as np
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DataReplayer(Node):
    def __init__(self):
        super().__init__('data_replayer')
        
        # Parámetros de configuración
        self.declare_parameter('replay_speed', 1.0)  # Velocidad de reproducción
        self.replay_speed = self.get_parameter('replay_speed').value
        
        # Estado de reproducción
        self.is_replaying = True
        self.current_log_path = None
        self.start_time = None
        self.real_start_time = None
        
        # Buffer de datos
        self.sensor_data = []
        self.image_data = []
        self.current_data_index = 0
        self.current_image_index = 0
        self.last_image_index = -1
        
        # Timestamps para sincronización
        self.last_sensor_timestamp = None
        self.last_image_timestamp = None
        
        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, '/odom/unfiltered', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/unfiltered', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Publicadores para cámara con QoS apropiado para imágenes
        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.rgb_image_pub = self.create_publisher(Image, '/camera/rgb/image_raw', camera_qos)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)
        
        # Configuración de la cámara
        self.rgb_camera_info = CameraInfo()
        self.rgb_camera_info.header.frame_id = "camera_rgb_optical_frame"
        fx, fy, cx, cy = 574.1, 574.1, 320.0, 240.0
        self.rgb_camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.rgb_camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.rgb_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.rgb_camera_info.distortion_model = "plumb_bob"
        self.rgb_camera_info.height = 480
        self.rgb_camera_info.width = 640
        
        # Para procesamiento de imágenes
        self.bridge = CvBridge()
        
        # Temporizador para reproducción
        self.replay_timer = self.create_timer(0.01, self.replay_callback)  # 100 Hz
        self.replay_timer.cancel()  # Inicialmente desactivado
        
        # Suscriptores para control
        self.create_subscription(
            Bool, '/replay_enabled', self.replay_enabled_cb, 10)
            
        self.create_subscription(
            String, '/replay_log_path', self.replay_path_cb, 10)
        
        self.get_logger().info('🎬 DataReplayer inicializado - Esperando señal de reproducción...')

    def replay_enabled_cb(self, msg):
        """Callback para habilitar/deshabilitar reproducción"""
        if msg.data != self.is_replaying:
            self.is_replaying = msg.data
            
            if self.is_replaying:
                self.get_logger().info('🚀 Reproducción HABILITADA - Cargando datos...')
                if self.current_log_path:
                    self._start_replay()
                else:
                    self.get_logger().warning('Ruta de logging no recibida aún')
            else:
                self.get_logger().info('🛑 Reproducción DESHABILITADA')
                self._stop_replay()

    def replay_path_cb(self, msg):
        """Callback para recibir la ruta de logging"""
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            self.get_logger().info(f'📁 Ruta de datos recibida: {self.current_log_path}')
            
            # Si la reproducción está habilitada pero no hemos cargado datos
            if self.is_replaying and not self.sensor_data:
                self._start_replay()

    def _start_replay(self):
        """Inicia la reproducción cargando datos desde archivos"""
        if not self.current_log_path:
            self.get_logger().error('No hay ruta de datos definida')
            return
            
        try:
            # Cargar datos de sensores
            sensor_file = Path(self.current_log_path) / "raw_sensors.csv"
            if not sensor_file.exists():
                self.get_logger().error(f'Archivo de sensores no encontrado: {sensor_file}')
                return
                
            self._load_sensor_data(sensor_file)
            
            # Cargar imágenes RGB
            image_dir = Path(self.current_log_path) / "perception/images"
            if image_dir.exists():
                self._load_image_data(image_dir)
            else:
                self.get_logger().warning(f'Directorio de imágenes no encontrado: {image_dir}')
            
            # Inicializar índices
            self.current_data_index = 0
            self.current_image_index = 0
            self.last_image_index = -1
            
            # Establecer tiempo inicial
            if self.sensor_data:
                self.start_time = self.sensor_data[0]['stamp']
                self.real_start_time = self.get_clock().now().nanoseconds * 1e-9
                
                # Activar temporizador
                self.replay_timer.reset()
                
                self.get_logger().info(f'✅ Datos cargados: {len(self.sensor_data)} muestras de sensores, {len(self.image_data)} imágenes')
                self.get_logger().info(f'⏱️  Iniciando reproducción a {self.replay_speed}x velocidad')
            else:
                self.get_logger().error('No hay datos de sensores para reproducir')
                
        except Exception as e:
            self.get_logger().error(f'Error al iniciar reproducción: {e}')

    def _stop_replay(self):
        """Detiene la reproducción"""
        self.replay_timer.cancel()
        self.sensor_data.clear()
        self.image_data.clear()
        self.current_data_index = 0
        self.current_image_index = 0
        self.last_image_index = -1
        self.get_logger().info('📂 Reproducción detenida')

    def _load_sensor_data(self, csv_file):
        """Carga datos de sensores desde CSV"""
        self.sensor_data = []
        
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Convertir valores a float o mantener None
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
                    'cov_lat': float(row['cov_lat']) if row['cov_lat'] != '' and row['cov_lat'] != 'None' else None,
                    'cov_lon': float(row['cov_lon']) if row['cov_lon'] != '' and row['cov_lon'] != 'None' else None,
                    'cov_alt': float(row['cov_alt']) if row['cov_alt'] != '' and row['cov_alt'] != 'None' else None,
                }
                self.sensor_data.append(data)
        
        self.get_logger().info(f'📊 Cargados {len(self.sensor_data)} registros de sensores')

    def _load_image_data(self, image_dir):
        """Carga metadatos de imágenes desde directorio"""
        self.image_data = []
        
        try:
            # Buscar el archivo CSV de percepción
            perception_csv = Path(self.current_log_path) / "perception" / "perception_data.csv"
            
            if not perception_csv.exists():
                self.get_logger().warning(f'Archivo CSV no encontrado: {perception_csv}')
                return
            
            # Leer CSV para obtener timestamps
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
                                self.image_data.append({
                                    'stamp': timestamp,
                                    'path': str(image_path),
                                    'filename': image_filename
                                })
                            else:
                                # Buscar imagen aunque el nombre no coincida exactamente
                                for ext in ['.jpg', '.jpeg', '.png']:
                                    possible_path = image_dir / f"{Path(image_filename).stem}{ext}"
                                    if possible_path.exists():
                                        self.image_data.append({
                                            'stamp': timestamp,
                                            'path': str(possible_path),
                                            'filename': image_filename
                                        })
                                        break
                    except Exception as e:
                        continue
            
            # Si no encontramos datos en CSV, intentar usar nombres de archivo
            if not self.image_data:
                self.get_logger().info('Intentando extraer timestamps de nombres de archivo...')
                # Buscar archivos de imágenes
                image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']
                image_files = []
                
                for ext in image_extensions:
                    image_files.extend(image_dir.glob(f'*{ext}'))
                
                for img_file in image_files:
                    try:
                        # Extraer número de frame: frame_002800_image.jpg -> 2800
                        if img_file.name.startswith('frame_'):
                            frame_num_part = img_file.name.split('_')[1]
                            frame_num = int(frame_num_part)
                            
                            # Estimar timestamp basado en el tiempo inicial de los sensores
                            # Asumiendo 5 FPS: 0.2 segundos entre frames
                            timestamp = self.start_time + (frame_num * 0.2)
                            
                            self.image_data.append({
                                'stamp': timestamp,
                                'path': str(img_file),
                                'filename': img_file.name
                            })
                            self.get_logger().info(f'📸 Timestamp estimado para {img_file.name}: {timestamp}')
                    except:
                        self.get_logger().warning(f'No se pudo procesar: {img_file.name}')
            
            # Ordenar por timestamp
            self.image_data.sort(key=lambda x: x['stamp'])
            
            self.get_logger().info(f'🖼️  Cargadas {len(self.image_data)} imágenes')
            
            # Mostrar algunas imágenes cargadas para verificación
            for i, img in enumerate(self.image_data[:5]):
                self.get_logger().info(f'  {i+1}. {Path(img["path"]).name} -> {img["stamp"]:.3f}')
            
        except Exception as e:
            self.get_logger().error(f'Error cargando imágenes: {e}')

    def get_current_time(self):
        """Obtiene el tiempo actual de reproducción"""
        if self.real_start_time is None or self.start_time is None:
            return None
            
        current_real_time = self.get_clock().now().nanoseconds * 1e-9
        elapsed_real = current_real_time - self.real_start_time
        elapsed_simulated = elapsed_real * self.replay_speed
        
        return self.start_time + elapsed_simulated

    def replay_callback(self):
        """Callback del temporizador para publicar datos"""
        if not self.sensor_data or self.current_data_index >= len(self.sensor_data):
            self.get_logger().info('✅ Reproducción completada')
            self._stop_replay()
            return
        
        current_time = self.get_current_time()
        if current_time is None:
            return
        
        # Publicar datos de sensores hasta alcanzar el tiempo actual
        while (self.current_data_index < len(self.sensor_data) and 
               self.sensor_data[self.current_data_index]['stamp'] <= current_time):
            
            data = self.sensor_data[self.current_data_index]
            
            # Publicar según el tipo de dato
            if data['topic'] == 'odom':
                self._publish_odom(data)
            elif data['topic'] == 'imu':
                self._publish_imu(data)
            elif data['topic'] == 'gps':
                self._publish_gps(data)
            
            self.last_sensor_timestamp = data['stamp']
            self.current_data_index += 1
        
        # Publicar imágenes (mantener última hasta que llegue nueva)
        if self.image_data:
            # Buscar imagen correspondiente al tiempo actual
            image_to_publish = None
            
            # Buscar la imagen más reciente que no supere el tiempo actual
            while (self.current_image_index < len(self.image_data) and 
                   self.image_data[self.current_image_index]['stamp'] <= current_time):
                
                image_to_publish = self.image_data[self.current_image_index]
                self.last_image_timestamp = image_to_publish['stamp']
                self.current_image_index += 1
            
            # Si encontramos una nueva imagen, publicarla
            if image_to_publish and self.current_image_index - 1 != self.last_image_index:
                self._publish_image(image_to_publish)
                self.last_image_index = self.current_image_index - 1
                self.get_logger().info(f'📸 Imagen publicada: {Path(image_to_publish["path"]).name}')

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
        
        self.odom_pub.publish(msg)
        
        # Log informativo
        if self.current_data_index % 100 == 0:
            self.get_logger().info(f'📍 Odom publicado - Tiempo: {data["stamp"]:.3f}, '
                                 f'Posición: ({data["x"]:.2f}, {data["y"]:.2f}, {data["z"]:.2f})')

    def _publish_imu(self, data):
        """Publica mensaje IMU"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Orientación
        if all(v is not None for v in [data['qx'], data['qy'], data['qz'], data['qw']]):
            msg.orientation.x = data['qx']
            msg.orientation.y = data['qy']
            msg.orientation.z = data['qz']
            msg.orientation.w = data['qw']
        
        # Velocidad angular
        if all(v is not None for v in [data['wx'], data['wy'], data['wz']]):
            msg.angular_velocity.x = data['wx']
            msg.angular_velocity.y = data['wy']
            msg.angular_velocity.z = data['wz']
        
        # Aceleración lineal
        if all(v is not None for v in [data['ax'], data['ay'], data['az']]):
            msg.linear_acceleration.x = data['ax']
            msg.linear_acceleration.y = data['ay']
            msg.linear_acceleration.z = data['az']
        
        self.imu_pub.publish(msg)
        
        # Log informativo
        if self.current_data_index % 100 == 0:
            self.get_logger().info(f'📈 IMU publicado - Tiempo: {data["stamp"]:.3f}, '
                                 f'Aceleración: ({data["ax"]:.2f}, {data["ay"]:.2f}, {data["az"]:.2f})')

    def _publish_gps(self, data):
        """Publica mensaje GPS"""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gnss_frame'
        
        # Coordenadas
        if all(v is not None for v in [data['lat'], data['lon'], data['alt']]):
            msg.latitude = data['lat']
            msg.longitude = data['lon']
            msg.altitude = data['alt']
            
            # Covarianzas
            cov_matrix = [0.0] * 9
            if data['cov_lat'] is not None:
                cov_matrix[0] = data['cov_lat']
            if data['cov_lon'] is not None:
                cov_matrix[4] = data['cov_lon']
            if data['cov_alt'] is not None:
                cov_matrix[8] = data['cov_alt']
            
            msg.position_covariance = cov_matrix
            msg.position_covariance_type = 2  # COVARIANCE_TYPE_APPROXIMATED
        
        self.gps_pub.publish(msg)
        
        # Log informativo
        if self.current_data_index % 100 == 0:
            self.get_logger().info(f'🌍 GPS publicado - Tiempo: {data["stamp"]:.3f}, '
                                 f'Posición: ({data["lat"]:.6f}, {data["lon"]:.6f})')

    def _publish_image(self, image_data):
        """Publica imagen RGB"""
        try:
            # Cargar imagen
            img_path = image_data['path']
            cv_image = cv2.imread(img_path)
            
            if cv_image is None:
                self.get_logger().error(f'No se pudo cargar imagen: {img_path}')
                return
            
            # Convertir a mensaje ROS
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_rgb_optical_frame'
            
            # Publicar imagen
            self.rgb_image_pub.publish(img_msg)
            
            # Publicar información de la cámara
            self.rgb_camera_info.header.stamp = img_msg.header.stamp
            self.rgb_info_pub.publish(self.rgb_camera_info)
            
        except Exception as e:
            self.get_logger().error(f'Error al publicar imagen: {e}')

    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        self._stop_replay()
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