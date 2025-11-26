#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt8, Bool
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import time


class SafetyUnifierNode(Node):
    def __init__(self):
        super().__init__('unifier_safety_communication')

        # Parameters
        self.declare_parameter('microros_timeout_ms', 2000)
        self.declare_parameter('serial_timeout_ms', 800)
        self.declare_parameter('recover_stable_ms', 2000)
        self.declare_parameter('data_expire_ms', 1000)

        self.microros_timeout_ms = self.get_parameter('microros_timeout_ms').value
        self.serial_timeout_ms = self.get_parameter('serial_timeout_ms').value
        self.recover_stable_ms = self.get_parameter('recover_stable_ms').value
        self.data_expire_ms = self.get_parameter('data_expire_ms').value

        # Final unified publishers
        self.battery_array_pub = self.create_publisher(Float32MultiArray, 'battery_array', 10)
        self.motors_array_pub = self.create_publisher(Float32MultiArray, 'motors_array', 10)

        # Commands routing - para controlar qué bridge envía comandos
        self.use_serial_safety_pub = self.create_publisher(Bool, 'use_serial_safety', 10)

        # Diagnostic publisher
        self.diagnostic_pub = self.create_publisher(DiagnosticStatus, 'status/esp32_safety', 10)

        # Subscribe to all safety sources
        # Micro-ROS sources
        self.create_subscription(Float32MultiArray, 'battery_array/microros', self.cb_battery_microros, 10)
        self.create_subscription(Float32MultiArray, 'motors_array/microros', self.cb_motors_microros, 10)
        
        # Serial sources  
        self.create_subscription(Float32MultiArray, 'battery_array/serial', self.cb_battery_serial, 10)
        self.create_subscription(Float32MultiArray, 'motors_array/serial', self.cb_motors_serial, 10)

        # Heartbeats
        self.create_subscription(Bool, 'serial_safety/heartbeat', self.cb_serial_hb, 10)

        # Timestamps
        self.last_microros_data = 0.0  # Tiempo del último dato micro-ROS
        self.last_serial_hb = 0.0

        self.last_switch_time = 0.0
        self.current_source = 'microros'  # Priorizar micro-ROS por defecto

        # Last received data
        self.last_battery_microros = None
        self.last_battery_serial = None
        self.last_motors_microros = None
        self.last_motors_serial = None

        # Timestamps for data expiration
        self.last_rx_time = {
            'battery_m': 0, 'battery_s': 0,
            'motors_m': 0, 'motors_s': 0
        }

        # Diagnostic variables
        self.diagnostic_last_publish = 0.0
        self.diagnostic_publish_interval = 1.0  # 1 second
        self.message_counters = {
            'microros': 0,
            'serial': 0
        }
        self.last_activity_time = {
            'microros': None,
            'serial': None
        }

        # Timer: 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        #self.get_logger().info('🚀 Safety Unifier node started')

    # === Timestamps helper ===
    def now_ms(self):
        return self.get_clock().now().nanoseconds / 1e6

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def is_fresh(self, key):
        return (self.now_ms() - self.last_rx_time[key]) < self.data_expire_ms

    # === Callbacks with timestamp recording ===
    def cb_battery_microros(self, msg):
        self.last_battery_microros = msg
        self.last_rx_time['battery_m'] = self.now_ms()
        self.last_activity_time['microros'] = self.now_sec()
        self.last_microros_data = self.now_ms()
        self.message_counters['microros'] += 1

    def cb_motors_microros(self, msg):
        self.last_motors_microros = msg
        self.last_rx_time['motors_m'] = self.now_ms()
        self.last_activity_time['microros'] = self.now_sec()
        self.last_microros_data = self.now_ms()
        self.message_counters['microros'] += 1

    def cb_battery_serial(self, msg):
        self.last_battery_serial = msg
        self.last_rx_time['battery_s'] = self.now_ms()
        self.last_activity_time['serial'] = self.now_sec()
        self.message_counters['serial'] += 1

    def cb_motors_serial(self, msg):
        self.last_motors_serial = msg
        self.last_rx_time['motors_s'] = self.now_ms()
        self.last_activity_time['serial'] = self.now_sec()
        self.message_counters['serial'] += 1

    def cb_serial_hb(self, msg):
        if msg.data:  # Solo actualizar si el heartbeat es True
            self.last_serial_hb = self.now_ms()
            self.last_activity_time['serial'] = self.now_sec()

    # === Diagnostic Functions ===
    def get_connection_quality(self, source):
        """Evaluate connection quality based on data freshness"""
        if source == 'microros':
            fresh_count = sum([self.is_fresh('battery_m'), self.is_fresh('motors_m')])
        else:  # serial
            fresh_count = sum([self.is_fresh('battery_s'), self.is_fresh('motors_s')])
        
        total_possible = 2  # battery_array, motors_array
        percentage = (fresh_count / total_possible) * 100
        
        if percentage >= 80:
            return "EXCELENTE"
        elif percentage >= 60:
            return "BUENA"
        elif percentage >= 40:
            return "REGULAR"
        else:
            return "DEFICIENTE"

    def get_data_freshness_status(self):
        """Get detailed freshness status for both sources"""
        microros_fresh = sum([self.is_fresh('battery_m'), self.is_fresh('motors_m')])
        serial_fresh = sum([self.is_fresh('battery_s'), self.is_fresh('motors_s')])
        
        return microros_fresh, serial_fresh

    def publish_diagnostic(self):
        """Publish comprehensive diagnostic information"""
        current_time = self.now_sec()
        
        # Calculate time since last activity
        microros_activity_time = self.last_activity_time.get('microros')
        serial_activity_time = self.last_activity_time.get('serial')
        
        time_since_microros = "Nunca" if microros_activity_time is None else f"{current_time - microros_activity_time:.1f}s"
        time_since_serial = "Nunca" if serial_activity_time is None else f"{current_time - serial_activity_time:.1f}s"
        
        # Check if sources are alive
        microros_alive = (self.now_ms() - self.last_microros_data) < self.microros_timeout_ms
        serial_alive = (self.now_ms() - self.last_serial_hb) < self.serial_timeout_ms
        
        # Get data freshness
        microros_fresh, serial_fresh = self.get_data_freshness_status()
        
        # Determine overall status level
        if microros_alive and serial_alive:
            level = DiagnosticStatus.OK
            status_msg = "Ambas conexiones activas"
        elif microros_alive:
            level = DiagnosticStatus.WARN
            status_msg = "Solo micro-ROS activo"
        elif serial_alive:
            level = DiagnosticStatus.WARN
            status_msg = "Solo Serial activo"
        else:
            level = DiagnosticStatus.ERROR
            status_msg = "Sin conexiones activas"
        
        # Add source quality to message
        if microros_alive:
            status_msg += f" | micro-ROS: {self.get_connection_quality('microros')}"
        if serial_alive:
            status_msg += f" | Serial: {self.get_connection_quality('serial')}"
        
        # Create diagnostic message
        msg = DiagnosticStatus()
        msg.name = "ESP32 Safety"
        msg.hardware_id = "ESP32_Robot_Safety"
        msg.level = level
        msg.message = status_msg
        
        # Add detailed values
        msg.values = [
            KeyValue(key="current_source", value=self.current_source),
            KeyValue(key="microros_connected", value=str(microros_alive)),
            KeyValue(key="serial_connected", value=str(serial_alive)),
            KeyValue(key="microros_data_fresh", value=f"{microros_fresh}/2"),
            KeyValue(key="serial_data_fresh", value=f"{serial_fresh}/2"),
            KeyValue(key="microros_quality", value=self.get_connection_quality('microros')),
            KeyValue(key="serial_quality", value=self.get_connection_quality('serial')),
            KeyValue(key="last_microros_activity", value=time_since_microros),
            KeyValue(key="last_serial_activity", value=time_since_serial),
            KeyValue(key="microros_message_count", value=str(self.message_counters['microros'])),
            KeyValue(key="serial_message_count", value=str(self.message_counters['serial'])),
            KeyValue(key="use_serial_commands", value=str(self.current_source == 'serial')),
            KeyValue(key="data_expire_ms", value=str(self.data_expire_ms)),
            KeyValue(key="microros_timeout_ms", value=str(self.microros_timeout_ms)),
            KeyValue(key="serial_timeout_ms", value=str(self.serial_timeout_ms)),
            KeyValue(key="recover_stable_ms", value=str(self.recover_stable_ms)),
            KeyValue(key="timestamp", value=str(current_time)),
        ]
        
        self.diagnostic_pub.publish(msg)

    # === Main logic ===
    def control_loop(self):
        now = self.now_ms()

        # Detectar micro-ROS vivo basado en datos
        microros_alive = (now - self.last_microros_data) < self.microros_timeout_ms
        serial_alive = (now - self.last_serial_hb) < self.serial_timeout_ms

        prev_source = self.current_source

        # Select source with hysteresis - micro-ROS tiene prioridad
        if microros_alive:
            if self.current_source != 'microros' and \
               (now - self.last_switch_time > self.recover_stable_ms):
                self.current_source = 'microros'
                self.last_switch_time = now
        elif serial_alive:
            if self.current_source != 'serial':
                self.current_source = 'serial'
                self.last_switch_time = now

        # Notify safety bridge which input to use for commands
        use_serial_msg = Bool()
        use_serial_msg.data = (self.current_source == 'serial')
        self.use_serial_safety_pub.publish(use_serial_msg)

        # ========= PUBLICACIÓN SEGÚN FUENTE =========
        if self.current_source == 'microros':
            # Publicar datos de micro-ROS si están frescos
            if self.is_fresh('battery_m') and self.last_battery_microros:
                self.battery_array_pub.publish(self.last_battery_microros)
            if self.is_fresh('motors_m') and self.last_motors_microros:
                self.motors_array_pub.publish(self.last_motors_microros)
        else:
            # Publicar datos de serial si están frescos
            if self.is_fresh('battery_s') and self.last_battery_serial:
                self.battery_array_pub.publish(self.last_battery_serial)
            if self.is_fresh('motors_s') and self.last_motors_serial:
                self.motors_array_pub.publish(self.last_motors_serial)

        # === Caso crítico: no hay datos frescos de ninguna fuente ===
        if not any([
            self.is_fresh('battery_m'), self.is_fresh('battery_s'),
            self.is_fresh('motors_m'), self.is_fresh('motors_s')
        ]):
            # Manual throttle (cada 5 segundos)
            if not hasattr(self, "last_warn_time"):
                self.last_warn_time = 0

            if now - self.last_warn_time > 5000:  # 5000 ms
                #self.get_logger().warn("⚠️ NO hay datos frescos de safety - microros ni serial")
                self.last_warn_time = now

        # Log source changes
        if prev_source != self.current_source:
            #self.get_logger().info(f"🔄 Safety - Cambio de fuente: {prev_source} -> {self.current_source}")
            pass

        # Publish diagnostic information (throttled to 1Hz)
        current_sec = self.now_sec()
        if current_sec - self.diagnostic_last_publish >= self.diagnostic_publish_interval:
            self.publish_diagnostic()
            self.diagnostic_last_publish = current_sec


def main(args=None):
    rclpy.init(args=args)
    node = SafetyUnifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 Safety Unifier node apagado por usuario')
    except Exception as e:
        node.get_logger().error(f'🚨 Error fatal en Safety Unifier: {e}')
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()