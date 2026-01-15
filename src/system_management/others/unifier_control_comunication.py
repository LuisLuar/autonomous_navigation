#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import time
from geometry_msgs.msg import Quaternion
import numpy as np
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class UnifierNode(Node):
    def __init__(self):
        super().__init__('unifier_control_communication')

        # Parameters
        self.declare_parameter('microros_timeout_ms', 800)
        self.declare_parameter('serial_timeout_ms', 800)
        self.declare_parameter('recover_stable_ms', 2000)
        self.declare_parameter('data_expire_ms', 1000)
        
        # Nuevo parámetro para el offset de calibración del yaw (en grados)
        self.declare_parameter('imu_yaw_offset_deg', 0.0)  # Offset en grados

        self.microros_timeout_ms = self.get_parameter('microros_timeout_ms').value
        self.serial_timeout_ms = self.get_parameter('serial_timeout_ms').value
        self.recover_stable_ms = self.get_parameter('recover_stable_ms').value
        self.data_expire_ms = self.get_parameter('data_expire_ms').value
        
        # Obtener offset de calibración del yaw
        self.imu_yaw_offset_deg = self.get_parameter('imu_yaw_offset_deg').value
        
        # Convertir a radianes
        self.imu_yaw_offset = np.deg2rad(self.imu_yaw_offset_deg)
        
        self.get_logger().info(f'IMU Yaw Calibration offset: {self.imu_yaw_offset_deg}° ({self.imu_yaw_offset:.4f} rad)')

        # Final unified publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom/unfiltered', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/unfiltered', 10)
        self.range_front_pub = self.create_publisher(Range, 'range/front', 10)
        self.range_left_pub = self.create_publisher(Range, 'range/left', 10)
        self.range_right_pub = self.create_publisher(Range, 'range/right', 10)

        # Commands routing
        self.use_serial_pub = self.create_publisher(Bool, 'use_serial_control', 10)

        # Diagnostic publisher - mismo formato que el supervisor anterior
        self.diagnostic_pub = self.create_publisher(DiagnosticStatus, 'status/esp32_control', 10)

        # Subscribe to all sources (sin microros/heartbeat)
        self.create_subscription(Odometry, 'odom/microros', self.cb_odom_microros, 10)
        self.create_subscription(Odometry, 'odom/serial', self.cb_odom_serial, 10)
        self.create_subscription(Imu, 'imu/microros', self.cb_imu_microros, 10)
        self.create_subscription(Imu, 'imu/serial', self.cb_imu_serial, 10)
        self.create_subscription(Range, 'range/front/microros', self.cb_rf_microros, 10)
        self.create_subscription(Range, 'range/left/microros', self.cb_rl_microros, 10)
        self.create_subscription(Range, 'range/right/microros', self.cb_rr_microros, 10)
        self.create_subscription(Range, 'range/front/serial', self.cb_rf_serial, 10)
        self.create_subscription(Range, 'range/left/serial', self.cb_rl_serial, 10)
        self.create_subscription(Range, 'range/right/serial', self.cb_rr_serial, 10)

        # Solo serial heartbeat (microros heartbeat eliminado)
        self.create_subscription(Bool, 'serial/heartbeat', self.cb_serial_hb, 10)

        self.last_microros_data = 0.0  # Tiempo del último dato micro-ROS
        self.last_serial_hb = 0.0

        self.last_switch_time = 0.0
        self.current_source = 'microros'

        # Last received data
        self.last_odom_microros = None
        self.last_odom_serial = None
        self.last_imu_microros = None
        self.last_imu_serial = None
        self.last_ranges = {
            'front_m': None, 'left_m': None, 'right_m': None,
            'front_s': None, 'left_s': None, 'right_s': None
        }

        # Timestamps for data expiration
        self.last_rx_time = {
            'odom_m': 0, 'odom_s': 0,
            'imu_m': 0, 'imu_s': 0,
            'rf_m': 0, 'lf_m': 0, 'rr_m': 0,
            'rf_s': 0, 'lf_s': 0, 'rr_s': 0
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

        # Service router
        self.reset_service = self.create_service(SetBool, 'robot_control_reset', self.reset_router_cb)

    # === Funciones de calibración de la IMU ===
    def apply_yaw_calibration(self, imu_msg):
        """
        Aplica el offset de calibración al yaw de la IMU
        """
        if imu_msg is None:
            return None
        
        # Si el offset es 0, no hacer nada
        if abs(self.imu_yaw_offset) < 0.0001:  # ~0.0057 grados
            return imu_msg
        
        # Crear una copia del mensaje para no modificar el original
        calibrated_msg = Imu()
        calibrated_msg.header = imu_msg.header
        calibrated_msg.angular_velocity = imu_msg.angular_velocity
        calibrated_msg.linear_acceleration = imu_msg.linear_acceleration
        calibrated_msg.orientation_covariance = imu_msg.orientation_covariance
        calibrated_msg.angular_velocity_covariance = imu_msg.angular_velocity_covariance
        calibrated_msg.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
        
        # Extraer la orientación actual como cuaternión
        q_original = imu_msg.orientation
        q_original_list = [q_original.x, q_original.y, q_original.z, q_original.w]
        
        # Convertir a ángulos de Euler (roll, pitch, yaw)
        try:
            roll, pitch, yaw = euler_from_quaternion(q_original_list)
            
            # Aplicar offset SOLO al yaw
            yaw += self.imu_yaw_offset
            
            # Normalizar el yaw a [-π, π]
            while yaw > np.pi:
                yaw -= 2 * np.pi
            while yaw < -np.pi:
                yaw += 2 * np.pi
            
            # Convertir de vuelta a cuaternión
            q_calibrated_list = quaternion_from_euler(roll, pitch, yaw)
            
            # Asignar al mensaje calibrado
            calibrated_msg.orientation.x = q_calibrated_list[0]
            calibrated_msg.orientation.y = q_calibrated_list[1]
            calibrated_msg.orientation.z = q_calibrated_list[2]
            calibrated_msg.orientation.w = q_calibrated_list[3]
            
        except Exception as e:
            self.get_logger().warn(f'Error applying IMU yaw calibration: {e}')
            # En caso de error, mantener la orientación original
            calibrated_msg.orientation = imu_msg.orientation
        
        return calibrated_msg

    # === Timestamps helper ===
    def now_ms(self):
        return self.get_clock().now().nanoseconds / 1e6

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def is_fresh(self, key):
        return (self.now_ms() - self.last_rx_time[key]) < self.data_expire_ms

    # === Callbacks with timestamp recording ===
    def cb_odom_microros(self, msg):
        self.last_odom_microros = msg
        self.last_rx_time['odom_m'] = self.now_ms()
        self.last_activity_time['microros'] = self.now_sec()
        self.last_microros_data = self.now_ms()  # Actualizar tiempo del último dato micro-ROS
        self.message_counters['microros'] += 1

    def cb_odom_serial(self, msg):
        self.last_odom_serial = msg
        self.last_rx_time['odom_s'] = self.now_ms()
        self.last_activity_time['serial'] = self.now_sec()
        self.message_counters['serial'] += 1

    def cb_imu_microros(self, msg):
        self.last_imu_microros = msg
        self.last_rx_time['imu_m'] = self.now_ms()
        self.last_activity_time['microros'] = self.now_sec()
        self.last_microros_data = self.now_ms()  # Actualizar tiempo del último dato micro-ROS
        self.message_counters['microros'] += 1

    def cb_imu_serial(self, msg):
        self.last_imu_serial = msg
        self.last_rx_time['imu_s'] = self.now_ms()
        self.last_activity_time['serial'] = self.now_sec()
        self.message_counters['serial'] += 1

    def cb_rf_microros(self, msg):
        self.last_ranges['front_m'] = msg
        self.last_rx_time['rf_m'] = self.now_ms()
        self.last_activity_time['microros'] = self.now_sec()
        self.last_microros_data = self.now_ms()  # Actualizar tiempo del último dato micro-ROS
        self.message_counters['microros'] += 1

    def cb_rl_microros(self, msg):
        self.last_ranges['left_m'] = msg
        self.last_rx_time['lf_m'] = self.now_ms()
        self.last_activity_time['microros'] = self.now_sec()
        self.last_microros_data = self.now_ms()  # Actualizar tiempo del último dato micro-ROS
        self.message_counters['microros'] += 1

    def cb_rr_microros(self, msg):
        self.last_ranges['right_m'] = msg
        self.last_rx_time['rr_m'] = self.now_ms()
        self.last_activity_time['microros'] = self.now_sec()
        self.last_microros_data = self.now_ms()  # Actualizar tiempo del último dato micro-ROS
        self.message_counters['microros'] += 1

    def cb_rf_serial(self, msg):
        self.last_ranges['front_s'] = msg
        self.last_rx_time['rf_s'] = self.now_ms()
        self.last_activity_time['serial'] = self.now_sec()
        self.message_counters['serial'] += 1

    def cb_rl_serial(self, msg):
        self.last_ranges['left_s'] = msg
        self.last_rx_time['lf_s'] = self.now_ms()
        self.last_activity_time['serial'] = self.now_sec()
        self.message_counters['serial'] += 1

    def cb_rr_serial(self, msg):
        self.last_ranges['right_s'] = msg
        self.last_rx_time['rr_s'] = self.now_ms()
        self.last_activity_time['serial'] = self.now_sec()
        self.message_counters['serial'] += 1

    # Solo serial heartbeat (microros heartbeat eliminado)
    def cb_serial_hb(self, msg):
        self.last_serial_hb = self.now_ms()
        self.last_activity_time['serial'] = self.now_sec()

    # === Diagnostic Functions ===
    def get_connection_quality(self, source):
        """Evaluate connection quality based on data freshness"""
        if source == 'microros':
            fresh_count = sum([self.is_fresh('odom_m'), self.is_fresh('imu_m'), 
                             self.is_fresh('rf_m'), self.is_fresh('lf_m'), self.is_fresh('rr_m')])
        else:  # serial
            fresh_count = sum([self.is_fresh('odom_s'), self.is_fresh('imu_s'), 
                             self.is_fresh('rf_s'), self.is_fresh('lf_s'), self.is_fresh('rr_s')])
        
        total_possible = 5  # odom, imu, 3 ranges
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
        microros_fresh = sum([self.is_fresh('odom_m'), self.is_fresh('imu_m'), 
                            self.is_fresh('rf_m'), self.is_fresh('lf_m'), self.is_fresh('rr_m')])
        serial_fresh = sum([self.is_fresh('odom_s'), self.is_fresh('imu_s'), 
                          self.is_fresh('rf_s'), self.is_fresh('lf_s'), self.is_fresh('rr_s')])
        
        return microros_fresh, serial_fresh

    def publish_diagnostic(self):
        """Publish comprehensive diagnostic information"""
        current_time = self.now_sec()
        
        # Calculate time since last activity
        microros_activity_time = self.last_activity_time.get('microros')
        serial_activity_time = self.last_activity_time.get('serial')
        
        time_since_microros = "Nunca" if microros_activity_time is None else f"{current_time - microros_activity_time:.1f}s"
        time_since_serial = "Nunca" if serial_activity_time is None else f"{current_time - serial_activity_time:.1f}s"
        
        # Check if sources are alive (usando datos en lugar de heartbeat para micro-ROS)
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
        msg.name = "ESP32 Control"
        msg.hardware_id = "ESP32_Robot_Control"
        msg.level = level
        msg.message = status_msg
        
        # Add detailed values
        msg.values = [
            KeyValue(key="current_source", value=self.current_source),
            KeyValue(key="microros_connected", value=str(microros_alive)),
            KeyValue(key="serial_connected", value=str(serial_alive)),
            KeyValue(key="microros_data_fresh", value=f"{microros_fresh}/5"),
            KeyValue(key="serial_data_fresh", value=f"{serial_fresh}/5"),
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
            KeyValue(key="imu_yaw_offset_deg", value=str(self.imu_yaw_offset_deg)),
            KeyValue(key="timestamp", value=str(current_time)),
        ]
        
        self.diagnostic_pub.publish(msg)

    # === Main logic ===
    def control_loop(self):
        now = self.now_ms()

        # Detectar micro-ROS vivo basado en datos (en lugar de heartbeat)
        microros_alive = (now - self.last_microros_data) < self.microros_timeout_ms
        serial_alive = (now - self.last_serial_hb) < self.serial_timeout_ms

        prev_source = self.current_source

        # Select source with hysteresis
        if microros_alive:
            if self.current_source != 'microros' and \
               (now - self.last_switch_time > self.recover_stable_ms):
                self.current_source = 'microros'
                self.last_switch_time = now
        elif serial_alive:
            if self.current_source != 'serial':
                self.current_source = 'serial'
                self.last_switch_time = now

        # Notify bridge which input to use for commands
        use_serial_msg = Bool()
        use_serial_msg.data = (self.current_source == 'serial')
        self.use_serial_pub.publish(use_serial_msg)

        # ========= PUBLICACIÓN SEGÚN FUENTE CON CALIBRACIÓN =========
        if self.current_source == 'microros':
            if self.is_fresh('odom_m'): 
                self.odom_pub.publish(self.last_odom_microros)
            if self.is_fresh('imu_m') and self.last_imu_microros is not None: 
                # Aplicar calibración de yaw antes de publicar
                calibrated_imu = self.apply_yaw_calibration(self.last_imu_microros)
                self.imu_pub.publish(calibrated_imu)
            if self.is_fresh('rf_m') and self.last_ranges['front_m'] is not None: 
                self.range_front_pub.publish(self.last_ranges['front_m'])
            if self.is_fresh('lf_m') and self.last_ranges['left_m'] is not None: 
                self.range_left_pub.publish(self.last_ranges['left_m'])
            if self.is_fresh('rr_m') and self.last_ranges['right_m'] is not None: 
                self.range_right_pub.publish(self.last_ranges['right_m'])
        else:
            if self.is_fresh('odom_s') and self.last_odom_serial is not None: 
                self.odom_pub.publish(self.last_odom_serial)
            if self.is_fresh('imu_s') and self.last_imu_serial is not None: 
                # Aplicar calibración de yaw antes de publicar
                calibrated_imu = self.apply_yaw_calibration(self.last_imu_serial)
                self.imu_pub.publish(calibrated_imu)
            if self.is_fresh('rf_s') and self.last_ranges['front_s'] is not None: 
                self.range_front_pub.publish(self.last_ranges['front_s'])
            if self.is_fresh('lf_s') and self.last_ranges['left_s'] is not None: 
                self.range_left_pub.publish(self.last_ranges['left_s'])
            if self.is_fresh('rr_s') and self.last_ranges['right_s'] is not None: 
                self.range_right_pub.publish(self.last_ranges['right_s'])

        # === Caso crítico: no hay datos frescos de ninguna fuente ===
        if not any([
            self.is_fresh('odom_m'), self.is_fresh('odom_s'),
            self.is_fresh('imu_m'), self.is_fresh('imu_s'),
            self.is_fresh('rf_m'), self.is_fresh('rf_s'),
            self.is_fresh('lf_m'), self.is_fresh('lf_s'),
            self.is_fresh('rr_m'), self.is_fresh('rr_s')
        ]):
            # Manual throttle (cada 2 segundos)
            if not hasattr(self, "last_warn_time"):
                self.last_warn_time = 0

            if now - self.last_warn_time > 2000:  # 2000 ms
                #self.get_logger().warn("⚠️ NO hay datos frescos de microros ni serial — sistema en silencio.")
                self.last_warn_time = now

        # Log source changes
        if prev_source != self.current_source:
            #self.get_logger().info(f"🔄 Cambio de fuente: {prev_source} -> {self.current_source}")
            pass

        # Publish diagnostic information (throttled to 1Hz)
        current_sec = self.now_sec()
        if current_sec - self.diagnostic_last_publish >= self.diagnostic_publish_interval:
            self.publish_diagnostic()
            self.diagnostic_last_publish = current_sec

    # === Service router ===
    def reset_router_cb(self, request, response):

        # First try microROS service
        microros_cli = self.create_client(SetBool, 'robot_control_reset_microros')
        if microros_cli.wait_for_service(timeout_sec=0.2):
            req = SetBool.Request()
            req.data = request.data
            fut = microros_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=0.6)
            if fut.done() and fut.result():
                res = fut.result()
                response.success = res.success
                response.message = "(microros) " + res.message
                return response

        # Fallback to serial
        serial_cli = self.create_client(SetBool, 'robot_control_reset_serial')
        if serial_cli.wait_for_service(timeout_sec=0.5):
            req2 = SetBool.Request()
            req2.data = request.data
            fut2 = serial_cli.call_async(req2)
            rclpy.spin_until_future_complete(self, fut2, timeout_sec=1.0)
            if fut2.done() and fut2.result():
                res2 = fut2.result()
                response.success = res2.success
                response.message = "(serial) " + res2.message
                return response

        response.success = False
        response.message = "No hay servicios de reset disponibles"
        return response
    
    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnifierNode()
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