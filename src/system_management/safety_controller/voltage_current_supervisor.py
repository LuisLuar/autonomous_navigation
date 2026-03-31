import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from gui.config.constants import Constants
import time

class VoltageCurrentSupervisor(Node):
    def __init__(self):
        super().__init__('supervisor_voltage_current')
        
        # Suscriptor único para datos de batería
        self.battery_sub = self.create_subscription(
            Float32MultiArray,
            '/battery_array',
            self.battery_callback,
            10
        )
        
        # Publicador único para estado de batería 12V
        self.battery_12v_pub = self.create_publisher(DiagnosticStatus, 'status/battery_12v', 10)
        
        # Publicador único para relé que controla ambos motores
        self.motors_relay_pub = self.create_publisher(Bool, 'start/motors', 10)
        
        # Variables de estado
        self.current_voltage_12v = 0.0
        
        # Variables para detección de timeout
        self.last_battery_time = None
        self.TIMEOUT_DURATION = 5.0  # segundos sin datos antes de considerar error
        
        # Estado de conexión
        self.battery_connected = False
        
        # Contador de mensajes para diagnóstico
        self.battery_msg_count = 0
        
        # Timer para publicación inmediata al inicio
        self.initial_publish_timer = self.create_timer(1.0, self.initial_status_publish)
        self.initial_publications_done = False
        
        # Timer para verificación periódica de timeouts
        self.timeout_timer = self.create_timer(2.0, self.check_timeouts)
        
        self.get_logger().info("Supervisor de Voltaje/Corriente iniciado")

    def initial_status_publish(self):
        """Publica estados iniciales inmediatamente al iniciar el nodo"""
        if not self.initial_publications_done:
            # Publicar estado inicial de batería
            self.publish_initial_battery_status()
            
            # Marcar que ya se hicieron las publicaciones iniciales
            self.initial_publications_done = True
            self.initial_publish_timer.cancel()
            self.get_logger().info("Estados iniciales publicados")

    def publish_initial_battery_status(self):
        """Publica estado inicial de batería (ESPERANDO DATOS)"""
        battery_status = DiagnosticStatus()
        battery_status.name = "Voltaje de la bateria"
        battery_status.level = DiagnosticStatus.ERROR
        battery_status.message = "ESPERANDO PRIMEROS DATOS"
        
        battery_status.values = [
            self.create_key_value("voltage", "0.00"),
            self.create_key_value("percentage", "0.0"),
            self.create_key_value("units", "V"),
            self.create_key_value("status", "INICIALIZANDO"),
            self.create_key_value("message_count", "0"),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        
        self.battery_12v_pub.publish(battery_status)

    def battery_callback(self, msg):
        """Callback para datos de batería con timestamp"""
        current_time = time.time()
        self.last_battery_time = current_time
        self.battery_msg_count += 1
        
        if len(msg.data) >= 1:
            self.current_voltage_12v = msg.data[0]
            
            # Solo procesar si es la primera vez o si se recuperó de timeout
            if not self.battery_connected:
                self.get_logger().info("Batería: Conexión establecida - Primeros datos recibidos")
                self.battery_connected = True
            
            self.publish_battery_status()
            self.check_emergency_shutdown()

    def check_timeouts(self):
        """Verifica timeout en el tópico de batería y publica estado de error"""
        current_time = time.time()
        
        # Verificar timeout de batería
        battery_timed_out = False
        if self.last_battery_time is not None:
            battery_timeout = current_time - self.last_battery_time > self.TIMEOUT_DURATION
            if battery_timeout and self.battery_connected:
                self.get_logger().error(f"TIMEOUT Batería: Sin datos por {self.TIMEOUT_DURATION}s")
                self.battery_connected = False
                battery_timed_out = True
        elif self.last_battery_time is None and self.initial_publications_done:
            # Si ya pasó el tiempo inicial y nunca recibió datos
            if not hasattr(self, '_battery_never_received_logged') or not self._battery_never_received_logged:
                self.get_logger().warn("Batería: Aún no se recibieron datos iniciales")
                self._battery_never_received_logged = True
        
        # Publicar estado de error por timeout
        if battery_timed_out:
            self.publish_battery_timeout_status()

    def publish_battery_timeout_status(self):
        """Publica estado de error por timeout de batería"""
        battery_status = DiagnosticStatus()
        battery_status.name = "Voltaje de la bateria"
        battery_status.level = DiagnosticStatus.ERROR
        battery_status.message = f"TIMEOUT: Sin datos por {self.TIMEOUT_DURATION}s"
        
        battery_status.values = [
            self.create_key_value("voltage", "N/A"),
            self.create_key_value("percentage", "N/A"),
            self.create_key_value("units", "V"),
            self.create_key_value("status", "TIMEOUT"),
            self.create_key_value("last_message_count", str(self.battery_msg_count)),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        
        self.battery_12v_pub.publish(battery_status)

    def publish_battery_status(self):
        """Publica estado normal de batería (solo si hay conexión)"""
        if not self.battery_connected:
            return
            
        # Calcular porcentaje de batería 12V
        battery_percentage = max(0, min(100, 
            (self.current_voltage_12v - Constants.BATTERY_12V_MIN) / 
            (Constants.BATTERY_12V_MAX - Constants.BATTERY_12V_MIN) * 100
        ))
        
        battery_status = DiagnosticStatus()
        battery_status.name = "Voltaje de la bateria"
        
        if battery_percentage < Constants.BATTERY_PERCENTAGE_MIN:
            battery_status.level = DiagnosticStatus.ERROR
            battery_status.message = f"CRÍTICO: {self.current_voltage_12v:.2f}V ({battery_percentage:.1f}%)"
        elif battery_percentage < Constants.BATTERY_PERCENTAGE_LIMIT:
            battery_status.level = DiagnosticStatus.WARN
            battery_status.message = f"BAJO: {self.current_voltage_12v:.2f}V ({battery_percentage:.1f}%)"
        else:
            battery_status.level = DiagnosticStatus.OK
            battery_status.message = f"ÓPTIMO: {self.current_voltage_12v:.2f}V ({battery_percentage:.1f}%)"
        
        battery_status.values = [
            self.create_key_value("voltage", f"{self.current_voltage_12v:.2f}"),
            self.create_key_value("percentage", f"{battery_percentage:.1f}"),
            self.create_key_value("units", "V"),
            self.create_key_value("status", "ACTIVO"),
            self.create_key_value("message_count", str(self.battery_msg_count)),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        
        self.battery_12v_pub.publish(battery_status)

    def check_emergency_shutdown(self):
        """Verificación de emergencia por voltaje (solo si hay datos válidos)"""
        if not self.battery_connected:
            return
            
        # Verificación de emergencia por voltaje bajo
        emergency = Bool()
        emergency.data = self.current_voltage_12v < Constants.BATTERY_12V_MIN
        
        self.motors_relay_pub.publish(emergency)
        
        if emergency.data:
            self.get_logger().warn(f"EMERGENCIA ACTIVADA: Voltaje bajo {self.current_voltage_12v:.2f}V")

    def create_key_value(self, key: str, value: str) -> KeyValue:
        """Helper function to create KeyValue objects"""
        kv = KeyValue()
        kv.key = key
        kv.value = value
        return kv

    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoltageCurrentSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Supervisor apagado por usuario")
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()