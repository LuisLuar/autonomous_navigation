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

        #camara
        self.camera_rgb_image = None
        self.camera_depth_image = None

        self.bridge = CvBridge()
        self.camera_rgb_qpixmap = None
        self.camera_depth_qpixmap = None


        # Bandera para indicar si recibimos al menos un mensaje
        self.any_msg_received = False

        # Configurar suscripciones
        self._setup_subscriptions()

        # Publisher de ejemplo para destino
        self.goal_pub = self.create_publisher(PoseStamped, '/gui/goal_pose', 10)

        self.get_logger().info('[ROSBridge] Inicializado y suscrito a topics.')

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

        # Señal del gps
        self.create_subscription(NavSatFix, '/gps/filtered', self.cb_gps_fix,10)
        self.create_subscription(String, '/gps/raw', self.cb_gps_raw,10)
        self.create_subscription(String, '/gps/info', self.cb_gps_info,10)

        #Camara
        self.create_subscription(Image, '/camera/rgb/image_raw', self.cb_camera_rgb, 1)
        self.create_subscription(Image, '/camera/depth/image_raw', self.cb_camera_depth, 1)


    # Callbacks: guardan el último mensaje recibido
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
    
    def cb_gps_fix(self, msg: NavSatFix):
        self.gps_fix = msg
        self.any_msg_received = True
    
    def cb_gps_raw(self, msg: String):
        self.gps_raw = msg
        self.any_msg_received = True

    def cb_gps_info(self, msg: String):
        self.gps_info = msg
        self.any_msg_received = True

    # Agregar los callbacks en ROSBridge:
    def cb_camera_rgb(self, msg: Image):

        try:
            # Convertir a cv2 (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convertir a RGB y crear QImage copiando los datos (evita referencias inválidas)
            rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb.copy().data, w, h, bytes_per_line, QImage.Format_RGB888)

            # Convertir a QPixmap (rápido) y guardarlo
            self.camera_rgb_qpixmap = QPixmap.fromImage(qimg)
            self.any_msg_received = True
            self.last_rgb_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().warning(f"Error en cb_camera_rgb: {e}")

    def cb_camera_depth(self, msg: Image):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            # Normalizar y colormap como haces en UI, pero aquí
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            h, w = depth_colormap.shape[:2]

            rgb = cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb.copy().data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.camera_depth_qpixmap = QPixmap.fromImage(qimg)
            self.any_msg_received = True
            self.last_depth_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().warning(f"Error en cb_camera_depth: {e}")


    def publish_goal(self, x, y, frame_id='map'):
        """
        Publica un destino en el mapa.
        
        Args:
            x: Coordenada X del destino
            y: Coordenada Y del destino
            frame_id: Sistema de referencia del destino
        """
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)
        self.get_logger().info(f'[ROSBridge] Publicado goal: {x:.2f}, {y:.2f}')