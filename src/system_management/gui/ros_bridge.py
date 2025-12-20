# gui/ros_bridge.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Path
from pyproj import Transformer
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool



from cv_bridge import CvBridge
from PySide6.QtGui import QImage, QPixmap
import numpy as np
import cv2

class ROSBridge(Node):
    def __init__(self, parent_node=None):
        """
        Inicializa el puente ROS2 para la interfaz gráfica.
        
        Args:
            parent_node: Nodo padre si se está utilizando un nodo existente
        """
        # Si se pasa un nodo padre, usarlo en lugar de crear uno nuevo
        if parent_node:
            self.node = parent_node
            self.get_logger = parent_node.get_logger
            self.get_clock = parent_node.get_clock
            self.create_subscription = parent_node.create_subscription
            self.create_publisher = parent_node.create_publisher
        else:
            super().__init__('gui_bridge_node')
            self.node = self

        # Últimos mensajes (inicializados a None)
        self.range_front = None
        self.range_left = None
        self.range_right = None
        self.imu = None
        self.odom = None
        self.odom_local = None
        self.odom_global = None
        self.battery_array = None
        self.motors_array = None
        self.gps_fix = None
        self.gps_info = None
        self.gps_raw = None

        # Estados de diagnóstico
        self.battery_12v_status = None
        self.voltage_5v_status = None
        self.motor_left_status = None
        self.motor_right_status = None
        self.esp32_safety_status = None
        self.esp32_control_status = None
        self.microros_agent_status = None
        self.components_status = None
        self.global_status = None
        self.camera_status = None
        self.gps_status = None
        self.rplidar_status = None

        #Estados de diagnostico LAPTOP
        self.cpu_temperature_status = None
        self.gpu_temperature_status = None
        self.battery_laptop_status = None
        self.ram_status = None
        self.cpu_usage_status = None
        self.gpu_usage_status = None
        self.disk_temperature_status = None
        self.uptime_status = None

        #camara
        self.bridge = CvBridge()
        #self.camera_rgb_qpixmap = None
        #self.camera_depth_qpixmap = None
        self.camera_segmentation_qpixmap = None
        self.camera_detection_qpixmap = None

        # Path global
        self.global_path = None
        self.map_origin = None   # (utm_x, utm_y)
        self.current_latlon = None


        # Conversor UTM → lat/lon (MISMO CRS que el planner)
        self.xy_to_ll = Transformer.from_crs(
            "EPSG:32717", "EPSG:4326", always_xy=True
        )


        # Bandera para indicar si recibimos al menos un mensaje
        self.any_msg_received = False

        # Configurar suscripciones
        self._setup_subscriptions()

        # Publisher para destino
        self.goal_latlon_pub = self.create_publisher(GeoPoint, '/goal_latlon', 10)

        #Publisher para limpiar destino
        self.clear_goal_pub = self.create_publisher(Bool, '/clear_path',10)

        #self.get_logger().info('[ROSBridge] Inicializado y suscrito a topics.')

    def _setup_subscriptions(self):
        """Configura todas las suscripciones a topics de ROS2."""
        # Sensores de distancia
        self.create_subscription(Range, '/range/front', self.cb_range_front, 10)
        self.create_subscription(Range, '/range/left', self.cb_range_left, 10)
        self.create_subscription(Range, '/range/right', self.cb_range_right, 10)
        
        # Sensores de orientación y posición
        self.create_subscription(Imu, '/imu/unfiltered', self.cb_imu, 10)
        self.create_subscription(Odometry, '/odom/unfiltered', self.cb_odom, 10)

        # Sensores de sistema
        self.create_subscription(Float32MultiArray, '/battery_array', self.cb_battery_array, 10)
        self.create_subscription(Float32MultiArray, '/motors_array', self.cb_motors_array, 10)

        #suscripciones de odometria local y global si existen
        self.create_subscription(Odometry, '/odometry/local', self.cb_odom_local, 10)
        self.create_subscription(Odometry, '/odometry/global', self.cb_odom_global, 10)
       
        # Suscripciones de diagnóstico
        self.create_subscription(DiagnosticStatus, 'status/battery_12v', self.cb_batt12v_status, 10)
        self.create_subscription(DiagnosticStatus, 'status/voltage_5v', self.cb_5v_status, 10)
        self.create_subscription(DiagnosticStatus, 'status/motor_left', self.cb_motor_left_status, 10)
        self.create_subscription(DiagnosticStatus, 'status/motor_right', self.cb_motor_right_status, 10)
        self.create_subscription(DiagnosticStatus, 'status/esp32_safety', self.cb_esp32_safety, 10)
        self.create_subscription(DiagnosticStatus, 'status/esp32_control', self.cb_esp32_control, 10)
        self.create_subscription(DiagnosticStatus, 'status/microros_agent', self.cb_microros_agent, 10)
        self.create_subscription(DiagnosticStatus, 'status/components', self.cb_components, 10)
        self.create_subscription(DiagnosticStatus, 'status/camera', self.cb_camera_status, 10)
        self.create_subscription(DiagnosticStatus, 'status/gps', self.cb_gps_status, 10)
        self.create_subscription(DiagnosticStatus, 'status/rplidar', self.cb_rplidar_status, 10)
        self.create_subscription(DiagnosticStatus, 'global_status', self.cb_global, 10)

        #Mensajes de diagnostico Laptopo
        self.create_subscription(DiagnosticStatus, '/status/cpu_temperature', self.cb_cpu_temp_status, 10)
        self.create_subscription(DiagnosticStatus, '/status/gpu_temperature', self.cb_gpu_temp_status, 10)
        self.create_subscription(DiagnosticStatus, '/status/battery_laptop', self.cb_battery_laptop_status, 10)
        self.create_subscription(DiagnosticStatus, '/status/ram', self.cb_ram_status, 10)
        self.create_subscription(DiagnosticStatus, '/status/cpu_usage', self.cb_cpu_usage_status, 10)
        self.create_subscription(DiagnosticStatus, '/status/gpu_usage', self.cb_gpu_usage_status, 10)
        self.create_subscription(DiagnosticStatus, '/status/disk_temperature', self.cb_disk_temp_status, 10)
        self.create_subscription(DiagnosticStatus, '/status/uptime', self.cb_uptime_status, 10)
        

        # Señal del gps
        self.create_subscription(NavSatFix, '/gps/filtered', self.cb_gps_fix,10)
        self.create_subscription(String, '/gps/raw', self.cb_gps_raw,10)
        self.create_subscription(String, '/gps/info', self.cb_gps_info,10)
        

        #Camara
        #self.create_subscription(Image, '/camera/rgb/image_raw', self.cb_camera_rgb, 1)
        #self.create_subscription(Image, '/camera/depth/image_raw', self.cb_camera_depth, 1) 

        # Redes neuronales
        self.create_subscription(Image, '/segmentation/overlay', self.cb_camera_segmentation, 1)
        self.create_subscription(Image, '/detection/annotated_image', self.cb_camera_detection, 1)

        # Path global
        self.create_subscription(Path, '/global_path', self.cb_global_path, 10)
        self.create_subscription(PointStamped,"/utm_map_origin",self.cb_map_origin,1)



    #FUNCIONES ADICIONALES
    def map_to_latlon(self, x_map, y_map):
        if self.map_origin is None:
            return None

        utm_x = self.map_origin[0] + x_map
        utm_y = self.map_origin[1] + y_map

        lon, lat = self.xy_to_ll.transform(utm_x, utm_y)
        return lat, lon


    # Callbacks: guardan el último mensaje recibido
    def cb_map_origin(self, msg: PointStamped):
        self.map_origin = (msg.point.x, msg.point.y)
        self.any_msg_received = True

    def cb_range_front(self, msg: Range):
        self.range_front = msg
        self.any_msg_received = True

    def cb_range_left(self, msg: Range):
        self.range_left = msg
        self.any_msg_received = True

    def cb_range_right(self, msg: Range):
        self.range_right = msg
        self.any_msg_received = True

    def cb_imu(self, msg: Imu):
        self.imu = msg
        self.any_msg_received = True

    def cb_odom(self, msg: Odometry):
        self.odom = msg
        self.any_msg_received = True

    def cb_battery_array(self, msg: Float32MultiArray):
        self.battery_array = msg
        self.any_msg_received = True

    def cb_motors_array(self, msg: Float32MultiArray):
        self.motors_array = msg
        self.any_msg_received = True

    def cb_batt12v_status(self, msg: DiagnosticStatus):
        self.battery_12v_status = msg
        self.any_msg_received = True

    def cb_5v_status(self, msg: DiagnosticStatus):
        self.voltage_5v_status = msg
        self.any_msg_received = True

    def cb_motor_left_status(self, msg: DiagnosticStatus):
        self.motor_left_status = msg
        self.any_msg_received = True

    def cb_motor_right_status(self, msg: DiagnosticStatus):
        self.motor_right_status = msg
        self.any_msg_received = True

    def cb_esp32_safety(self, msg: DiagnosticStatus):
        self.esp32_safety_status = msg
        self.any_msg_received = True

    def cb_esp32_control(self, msg: DiagnosticStatus):
        self.esp32_control_status = msg
        self.any_msg_received = True

    def cb_microros_agent(self, msg: DiagnosticStatus):
        self.microros_agent_status = msg
        self.any_msg_received = True

    def cb_components(self, msg: DiagnosticStatus):
        self.components_status = msg
        self.any_msg_received = True

    def cb_camera_status(self, msg: DiagnosticStatus):
        self.camera_status = msg
        self.any_msg_received = True
    
    def cb_gps_status(self, msg: DiagnosticStatus):
        self.gps_status = msg
        self.any_msg_received = True

    def cb_rplidar_status(self, msg: DiagnosticStatus):
        self.rplidar_status = msg
        self.any_msg_received = True
    
    def cb_global(self, msg: DiagnosticStatus):
        self.global_status = msg
        self.any_msg_received = True

    #Diagnostico de laptop
    def cb_cpu_temp_status(self, msg: DiagnosticStatus):
        self.cpu_temperature_status = msg
        self.any_msg_received = True

    def cb_gpu_temp_status(self, msg: DiagnosticStatus):
        self.gpu_temperature_status = msg
        self.any_msg_received = True

    def cb_battery_laptop_status(self, msg: DiagnosticStatus):
        self.battery_laptop_status= msg
        self.any_msg_received = True
    
    def cb_ram_status(self, msg: DiagnosticStatus):
        self.ram_status= msg
        self.any_msg_received = True
    
    def cb_cpu_usage_status(self, msg: DiagnosticStatus):
        self.cpu_usage_status = msg
        self.any_msg_received = True

    def cb_gpu_usage_status(self, msg: DiagnosticStatus):
        self.gpu_usage_status = msg
        self.any_msg_received = True

    def cb_disk_temp_status(self, msg: DiagnosticStatus):
        self.disk_temperature_status = msg
        self.any_msg_received = True

    def cb_uptime_status(self, msg: DiagnosticStatus):
        self.uptime_status= msg
        self.any_msg_received = True
    
    # Posición del robot    
    def cb_gps_fix(self, msg: NavSatFix):
        self.gps_fix = msg
        self.any_msg_received = True
    
    def cb_gps_raw(self, msg: String):
        self.gps_raw = msg
        self.any_msg_received = True

    def cb_gps_info(self, msg: String):
        self.gps_info = msg
        self.any_msg_received = True

    #Callbacks de posición global y local de odometría
    def cb_odom_local(self, msg: Odometry):
        self.odom_local = msg
        self.any_msg_received = True    
    
    def cb_odom_global(self, msg: Odometry):
        self.odom_global = msg

        if self.map_origin is None:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        latlon = self.map_to_latlon(x, y)
        if latlon:
            self.current_latlon = latlon

        self.any_msg_received = True

    

    
    def cb_camera_detection(self, msg: Image):
        try:
            det_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb = cv2.cvtColor(det_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb.copy().data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.camera_detection_qpixmap = QPixmap.fromImage(qimg)
            self.any_msg_received = True
            self.last_detection_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().warning(f"Error en cb_camera_detection: {e}")

    def cb_camera_segmentation(self, msg: Image):
        try:
            seg_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb = cv2.cvtColor(seg_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb.copy().data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.camera_segmentation_qpixmap = QPixmap.fromImage(qimg)
            self.any_msg_received = True
            self.last_segmentation_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().warning(f"Error en cb_camera_segmentation: {e}")

    def cb_global_path(self, msg: Path):
        if self.map_origin is None:
            return

        path_latlon = []

        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y

            latlon = self.map_to_latlon(x, y)
            if latlon:
                path_latlon.append(latlon)

        self.global_path = path_latlon
        self.any_msg_received = True

    def clear_path(self):
        """
        Publica una señal para limpiar el path actual.
        """
        clear_msg = Bool()
        clear_msg.data = True
        self.clear_goal_pub.publish(clear_msg)
        #self.get_logger().info(f'[ROSBridge] Publicado clear path signal.')


    def publish_goal_latlon(self, lat, lon):
        """
        Publica un destino en coordenadas geográficas.
        
        Args:
            lat: Latitud del destino
            lon: Longitud del destino
        """
        goal = GeoPoint()
        goal.latitude = float(lat)
        goal.longitude = float(lon)
        goal.altitude = 0.0  # Puedes usar la altitud actual si quieres
        
        self.goal_latlon_pub.publish(goal)
        #self.get_logger().info(f'[ROSBridge] Publicado goal lat/lon: {lat:.6f}, {lon:.6f}')