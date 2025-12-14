import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from gui.config.constants import Constants
import time

class VoltageCurrentSupervisor(Node):
    def __init__(self):
        super().__init__('supervisor_voltage_current')
        
        # Suscriptores
        self.battery_sub = self.create_subscription(
            Float32MultiArray,
            '/battery_array',
            self.battery_callback,
            10
        )
        
        self.motors_sub = self.create_subscription(
            Float32MultiArray,
            '/motors_array',
            self.motors_callback,
            10
        )
        
        # Publicadores para DiagnosticStatus
        self.battery_12v_pub = self.create_publisher(DiagnosticStatus, 'status/battery_12v', 10)
        self.voltage_5v_pub = self.create_publisher(DiagnosticStatus, 'status/voltage_5v', 10)
        self.motor_left_status_pub = self.create_publisher(DiagnosticStatus, 'status/motor_left', 10)
        self.motor_right_status_pub = self.create_publisher(DiagnosticStatus, 'status/motor_right', 10)
        
        # Publicadores para relés
        self.motor_left_relay_pub = self.create_publisher(Bool, 'start/motor_left', 10)
        self.motor_right_relay_pub = self.create_publisher(Bool, 'start/motor_right', 10)
        
        # Variables de estado
        self.current_voltage_12v = 0.0
        self.current_voltage_5v = 0.0
        self.current_left = 0.0
        self.current_right = 0.0
        
        # Variables para detección de timeout
        self.last_battery_time = None
        self.last_motors_time = None
        self.TIMEOUT_DURATION = 5.0  # segundos sin datos antes de considerar error
        
        # Estados de conexión
        self.battery_connected = False
        self.motors_connected = False
        
        # Contadores de mensajes para diagnóstico
        self.battery_msg_count = 0
        self.motors_msg_count = 0
        
        # 🔥 NUEVO: Timer para publicación inmediata al inicio
        self.initial_publish_timer = self.create_timer(1.0, self.initial_status_publish)
        self.initial_publications_done = False
        
        # Timer para verificación periódica de timeouts
        self.timeout_timer = self.create_timer(2.0, self.check_timeouts)
        
        #self.get_logger().info("🟢 Safety Monitor Inicializado - Enviando estados iniciales")

    def initial_status_publish(self):
        """Publica estados iniciales inmediatamente al iniciar el nodo"""
        if not self.initial_publications_done:
            #self.get_logger().info("📢 Publicando estados iniciales...")
            
            # Publicar estados de "ESPERANDO DATOS" para todos los componentes
            self.publish_initial_battery_status()
            self.publish_initial_motor_status()
            
            # Marcar que ya se hicieron las publicaciones iniciales
            self.initial_publications_done = True
            self.initial_publish_timer.cancel()  # Detener este timer después de la primera ejecución
            #self.get_logger().info("✅ Estados iniciales publicados")

    def publish_initial_battery_status(self):
        """Publica estado inicial de batería (ESPERANDO DATOS)"""
        # Batería 12V - Estado INICIAL
        battery_status = DiagnosticStatus()
        battery_status.name = "Voltaje de la bateria"
        battery_status.level = DiagnosticStatus.WARN  # 🔥 WARN en lugar de ERROR para estado inicial
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
        #self.get_logger().info("🔶 Batería 12V: Estado inicial publicado (ESPERANDO DATOS)")

        # Voltaje 5V - Estado INICIAL
        voltage_5v_status = DiagnosticStatus()
        voltage_5v_status.name = "Voltaje del sistema de control"
        voltage_5v_status.level = DiagnosticStatus.WARN  # 🔥 WARN en lugar de ERROR para estado inicial
        voltage_5v_status.message = "ESPERANDO PRIMEROS DATOS"
        
        voltage_5v_status.values = [
            self.create_key_value("voltage", "0.00"),
            self.create_key_value("units", "V"),
            self.create_key_value("status", "INICIALIZANDO"),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        
        self.voltage_5v_pub.publish(voltage_5v_status)
        #self.get_logger().info("🔶 Voltaje 5V: Estado inicial publicado (ESPERANDO DATOS)")

    def publish_initial_motor_status(self):
        """Publica estado inicial de motores (ESPERANDO DATOS)"""
        # Motor izquierdo - Estado INICIAL
        left_status = DiagnosticStatus()
        left_status.name = "Corriente del motor izquierdo"
        left_status.level = DiagnosticStatus.WARN  # 🔥 WARN en lugar de ERROR para estado inicial
        left_status.message = "ESPERANDO PRIMEROS DATOS"
        
        left_status.values = [
            self.create_key_value("current", "0.00"),
            self.create_key_value("units", "A"),
            self.create_key_value("status", "INICIALIZANDO"),
            self.create_key_value("message_count", "0"),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        self.motor_left_status_pub.publish(left_status)
        #self.get_logger().info("🔶 Motor izquierdo: Estado inicial publicado (ESPERANDO DATOS)")

        # Motor derecho - Estado INICIAL
        right_status = DiagnosticStatus()
        right_status.name = "Corriente del motor derecho"
        right_status.level = DiagnosticStatus.WARN  # 🔥 WARN en lugar de ERROR para estado inicial
        right_status.message = "ESPERANDO PRIMEROS DATOS"
        
        right_status.values = [
            self.create_key_value("current", "0.00"),
            self.create_key_value("units", "A"),
            self.create_key_value("status", "INICIALIZANDO"),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        self.motor_right_status_pub.publish(right_status)
        #self.get_logger().info("🔶 Motor derecho: Estado inicial publicado (ESPERANDO DATOS)")

    def battery_callback(self, msg):
        """Callback para datos de batería con timestamp"""
        current_time = time.time()
        self.last_battery_time = current_time
        self.battery_msg_count += 1
        
        if len(msg.data) >= 2:
            self.current_voltage_12v = msg.data[0]
            self.current_voltage_5v = msg.data[1]
            
            # Solo procesar si es la primera vez o si se recuperó de timeout
            if not self.battery_connected:
                #self.get_logger().info("🔋 Batería: Conexión establecida - Primeros datos recibidos")
                self.battery_connected = True
            
            self.publish_battery_status()
            self.check_emergency_shutdown()

    def motors_callback(self, msg):
        """Callback para datos de motores con timestamp"""
        current_time = time.time()
        self.last_motors_time = current_time
        self.motors_msg_count += 1
        
        if len(msg.data) >= 2:
            self.current_left = msg.data[0]
            self.current_right = msg.data[1]
            
            # Solo procesar si es la primera vez o si se recuperó de timeout
            if not self.motors_connected:
                #self.get_logger().info("⚙️ Motores: Conexión establecida - Primeros datos recibidos")
                self.motors_connected = True
            
            self.publish_motor_status()
            self.check_motor_protection()

    def check_timeouts(self):
        """Verifica timeouts en ambos tópicos y publica estados de error"""
        current_time = time.time()
        
        # Verificar timeout de batería
        battery_timed_out = False
        if self.last_battery_time is not None:
            battery_timeout = current_time - self.last_battery_time > self.TIMEOUT_DURATION
            if battery_timeout and self.battery_connected:
                #self.get_logger().error(f"🔴 TIMEOUT Batería: Sin datos por {self.TIMEOUT_DURATION}s")
                self.battery_connected = False
                battery_timed_out = True
        elif self.last_battery_time is None and self.initial_publications_done:
            # Si ya pasó el tiempo inicial y nunca recibió datos
            if not hasattr(self, '_battery_never_received_logged') or not self._battery_never_received_logged:
                #self.get_logger().warn("🔶 Batería: Aún no se recibieron datos iniciales")
                self._battery_never_received_logged = True
        
        # Verificar timeout de motores
        motors_timed_out = False
        if self.last_motors_time is not None:
            motors_timeout = current_time - self.last_motors_time > self.TIMEOUT_DURATION
            if motors_timeout and self.motors_connected:
                #self.get_logger().error(f"🔴 TIMEOUT Motores: Sin datos por {self.TIMEOUT_DURATION}s")
                self.motors_connected = False
                motors_timed_out = True
        elif self.last_motors_time is None and self.initial_publications_done:
            # Si ya pasó el tiempo inicial y nunca recibió datos
            if not hasattr(self, '_motors_never_received_logged') or not self._motors_never_received_logged:
                #self.get_logger().warn("🔶 Motores: Aún no se recibieron datos iniciales")
                self._motors_never_received_logged = True
        
        # Publicar estados de error por timeout
        if battery_timed_out:
            self.publish_battery_timeout_status()
        
        if motors_timed_out:
            self.publish_motors_timeout_status()
        
        # Verificar si AMBOS tópicos tienen timeout
        if battery_timed_out and motors_timed_out:
            #self.get_logger().error("🚨 CRÍTICO: Ambos tópicos (batería y motores) tienen timeout!")
            self.publish_global_timeout_status()

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

        # También para 5V
        voltage_5v_status = DiagnosticStatus()
        voltage_5v_status.name = "Voltaje del sistema de control"
        voltage_5v_status.level = DiagnosticStatus.ERROR
        voltage_5v_status.message = f"TIMEOUT: Sin datos por {self.TIMEOUT_DURATION}s"
        
        voltage_5v_status.values = [
            self.create_key_value("voltage", "N/A"),
            self.create_key_value("units", "V"),
            self.create_key_value("status", "TIMEOUT"),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        
        self.voltage_5v_pub.publish(voltage_5v_status)

    def publish_motors_timeout_status(self):
        """Publica estado de error por timeout de motores"""
        # Motor izquierdo
        left_status = DiagnosticStatus()
        left_status.name = "Corriente del motor izquierdo"
        left_status.level = DiagnosticStatus.ERROR
        left_status.message = f"TIMEOUT: Sin datos por {self.TIMEOUT_DURATION}s"
        
        left_status.values = [
            self.create_key_value("current", "N/A"),
            self.create_key_value("units", "A"),
            self.create_key_value("status", "TIMEOUT"),
            self.create_key_value("last_message_count", str(self.motors_msg_count)),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        self.motor_left_status_pub.publish(left_status)

        # Motor derecho
        right_status = DiagnosticStatus()
        right_status.name = "Corriente del motor derecho"
        right_status.level = DiagnosticStatus.ERROR
        right_status.message = f"TIMEOUT: Sin datos por {self.TIMEOUT_DURATION}s"
        
        right_status.values = [
            self.create_key_value("current", "N/A"),
            self.create_key_value("units", "A"),
            self.create_key_value("status", "TIMEOUT"),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        self.motor_right_status_pub.publish(right_status)

    def publish_global_timeout_status(self):
        """Publica estado crítico cuando ambos tópicos fallan"""
        # Log adicional para estado crítico
        #self.get_logger().error("💀 SISTEMA CRÍTICO: Sin datos de batería ni motores")
        pass

    def publish_battery_status(self):
        """Publica estado normal de batería (solo si hay conexión)"""
        if not self.battery_connected:
            return
            
        # Para batería de 12V
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

        # Para voltaje de 5V
        voltage_5v_status = DiagnosticStatus()
        voltage_5v_status.name = "Voltaje del sistema de control"
        
        if self.current_voltage_5v < Constants.CONTROL_VOLTAGE_MIN:
            voltage_5v_status.level = DiagnosticStatus.ERROR
            voltage_5v_status.message = f"CRÍTICO: {self.current_voltage_5v:.2f}V"
        elif self.current_voltage_5v < Constants.CONTROL_VOLTAGE_LIMIT:
            voltage_5v_status.level = DiagnosticStatus.WARN
            voltage_5v_status.message = f"BAJO: {self.current_voltage_5v:.2f}V"
        else:
            voltage_5v_status.level = DiagnosticStatus.OK
            voltage_5v_status.message = f"NORMAL: {self.current_voltage_5v:.2f}V"
        
        voltage_5v_status.values = [
            self.create_key_value("voltage", f"{self.current_voltage_5v:.2f}"),
            self.create_key_value("units", "V"),
            self.create_key_value("status", "ACTIVO"),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        
        self.voltage_5v_pub.publish(voltage_5v_status)

    def publish_motor_status(self):
        """Publica estado normal de motores (solo si hay conexión)"""
        if not self.motors_connected:
            return
            
        # Motor izquierdo
        left_status = DiagnosticStatus()
        left_status.name = "Corriente del motor izquierdo"
        
        if self.current_left >= Constants.MOTOR_CURRENT_MAX:
            left_status.level = DiagnosticStatus.ERROR
            left_status.message = f"CRÍTICA: {self.current_left:.2f}A"
        elif self.current_left >= Constants.MOTOR_CURRENT_LIMIT:
            left_status.level = DiagnosticStatus.WARN
            left_status.message = f"ELEVADA: {self.current_left:.2f}A"
        else:
            left_status.level = DiagnosticStatus.OK
            left_status.message = f"NORMAL: {self.current_left:.2f}A"
        
        left_status.values = [
            self.create_key_value("current", f"{self.current_left:.2f}"),
            self.create_key_value("units", "A"),
            self.create_key_value("status", "ACTIVO"),
            self.create_key_value("message_count", str(self.motors_msg_count)),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        self.motor_left_status_pub.publish(left_status)

        # Motor derecho
        right_status = DiagnosticStatus()
        right_status.name = "Corriente del motor derecho"
        
        if self.current_right >= Constants.MOTOR_CURRENT_MAX:
            right_status.level = DiagnosticStatus.ERROR
            right_status.message = f"CRÍTICA: {self.current_right:.2f}A"
        elif self.current_right >= Constants.MOTOR_CURRENT_LIMIT:
            right_status.level = DiagnosticStatus.WARN
            right_status.message = f"ELEVADA: {self.current_right:.2f}A"
        else:
            right_status.level = DiagnosticStatus.OK
            right_status.message = f"NORMAL: {self.current_right:.2f}A"
        
        right_status.values = [
            self.create_key_value("current", f"{self.current_right:.2f}"),
            self.create_key_value("units", "A"),
            self.create_key_value("status", "ACTIVO"),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        self.motor_right_status_pub.publish(right_status)

    def create_key_value(self, key: str, value: str) -> KeyValue:
        """Helper function to create KeyValue objects"""
        kv = KeyValue()
        kv.key = key
        kv.value = value
        return kv

    def check_motor_protection(self):
        """Protección de motores (solo si hay datos válidos)"""
        if not self.motors_connected:
            return
            
        # Protección individual por motor
        left_relay = Bool()
        left_relay.data = self.current_left >= Constants.MOTOR_CURRENT_MAX
        self.motor_left_relay_pub.publish(left_relay)

        right_relay = Bool()
        right_relay.data = self.current_right >= Constants.MOTOR_CURRENT_MAX
        self.motor_right_relay_pub.publish(right_relay)

    def check_emergency_shutdown(self):
        """Verificación de emergencia por voltaje (solo si hay datos válidos)"""
        if not self.battery_connected:
            return
            
        # Verificación de emergencia por voltaje
        if self.current_voltage_12v < Constants.BATTERY_12V_MIN:
            emergency_relay = Bool()
            emergency_relay.data = True
            
            self.motor_left_relay_pub.publish(emergency_relay)
            self.motor_right_relay_pub.publish(emergency_relay)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoltageCurrentSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info("Safety Monitor apagado por usuario")
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()