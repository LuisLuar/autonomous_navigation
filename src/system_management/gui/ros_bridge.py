# gui/ros_bridge.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped

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

        # Estados de diagnóstico
        self.battery_12v_status = None
        self.voltage_5v_status = None
        self.motor_left_status = None
        self.motor_right_status = None
        self.esp32_safety_status = None
        self.esp32_control_status = None
        self.microros_agent_status = None
        self.components_status = None

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
        self.create_subscription(Float32MultiArray, '/battery/array', self.cb_battery_array, 10)
        self.create_subscription(Float32MultiArray, '/motors/array', self.cb_motors_array, 10)
       
        # Suscripciones de diagnóstico
        self.create_subscription(DiagnosticStatus, 'status/battery_12v', self.cb_batt12v_status, 10)
        self.create_subscription(DiagnosticStatus, 'status/voltage_5v', self.cb_5v_status, 10)
        self.create_subscription(DiagnosticStatus, 'status/motor_left', self.cb_motor_left_status, 10)
        self.create_subscription(DiagnosticStatus, 'status/motor_right', self.cb_motor_right_status, 10)
        self.create_subscription(DiagnosticStatus, 'status/esp32_safety', self.cb_esp32_safety, 10)
        self.create_subscription(DiagnosticStatus, 'status/esp32_control', self.cb_esp32_control, 10)
        self.create_subscription(DiagnosticStatus, 'status/microros_agent', self.cb_microros_agent, 10)
        self.create_subscription(DiagnosticStatus, 'status/components', self.cb_components, 10)

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