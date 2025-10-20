#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry
import time
from collections import deque

class ComponentDiagnosticNode(Node):
    def __init__(self):
        super().__init__('supervisor_component_ESP32_diagnostic')

        # Publicador de diagnóstico
        self.diagnostic_pub = self.create_publisher(DiagnosticStatus, 'status/components', 10)

        # Suscriptores para cada componente
        self.encoder_sub = self.create_subscription(
            Odometry, 
            '/odom/unfiltered', 
            self.encoder_callback, 
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu/unfiltered', 
            self.imu_callback, 
            10
        )
        
        self.sharp_front_sub = self.create_subscription(
            Range, 
            '/range/front', 
            self.sharp_front_callback, 
            10
        )
        
        self.sharp_left_sub = self.create_subscription(
            Range, 
            '/range/left', 
            self.sharp_left_callback, 
            10
        )
        
        self.sharp_right_sub = self.create_subscription(
            Range, 
            '/range/right', 
            self.sharp_right_callback, 
            10
        )

        # Temporizador para verificación periódica
        self.timer = self.create_timer(2.0, self.check_components_health)

        # Timeouts para cada componente (segundos)
        self.TIMEOUTS = {
            'encoder': 3.0,
            'imu': 3.0,
            'sharp_front': 3.0,
            'sharp_left': 3.0,
            'sharp_right': 3.0
        }

        # Timestamps de última recepción
        self.last_received = {
            'encoder': None,
            'imu': None,
            'sharp_front': None,
            'sharp_left': None,
            'sharp_right': None
        }

        # Contadores de mensajes
        self.message_counters = {
            'encoder': 0,
            'imu': 0,
            'sharp_front': 0,
            'sharp_left': 0,
            'sharp_right': 0
        }

        # Historial de datos para verificación de calidad
        self.encoder_data_history = deque(maxlen=10)
        self.imu_data_history = deque(maxlen=10)
        self.sharp_front_history = deque(maxlen=10)
        self.sharp_left_history = deque(maxlen=10)
        self.sharp_right_history = deque(maxlen=10)

        # Umbrales para detección de datos inválidos
        self.INVALID_THRESHOLDS = {
            'sharp_range_min': 0.02,  # 2cm
            'sharp_range_max': 1.50,  # 150cm
            'imu_angular_velocity_max': 10.0,  # rad/s
            'imu_linear_acceleration_max': 20.0  # m/s²
        }

        self.get_logger().info("🟢 Nodo de diagnóstico de componentes iniciado")

    # Callbacks para cada componente
    def encoder_callback(self, msg):
        current_time = time.time()
        self.last_received['encoder'] = current_time
        self.message_counters['encoder'] += 1
        
        # Verificar calidad de datos del encoder
        is_valid = self.validate_encoder_data(msg)
        self.encoder_data_history.append((current_time, is_valid))
        
        if not is_valid:
            self.get_logger().warn("⚠️ Datos de encoder potencialmente inválidos")

    def imu_callback(self, msg):
        current_time = time.time()
        self.last_received['imu'] = current_time
        self.message_counters['imu'] += 1
        
        # Verificar calidad de datos de IMU
        is_valid = self.validate_imu_data(msg)
        self.imu_data_history.append((current_time, is_valid))
        
        if not is_valid:
            self.get_logger().warn("⚠️ Datos de IMU potencialmente inválidos")

    def sharp_front_callback(self, msg):
        current_time = time.time()
        self.last_received['sharp_front'] = current_time
        self.message_counters['sharp_front'] += 1
        
        # Verificar calidad de datos del sensor sharp frontal
        is_valid = self.validate_sharp_data(msg, 'front')
        self.sharp_front_history.append((current_time, is_valid))
        
        if not is_valid:
            self.get_logger().warn("⚠️ Datos de Sharp frontal potencialmente inválidos")

    def sharp_left_callback(self, msg):
        current_time = time.time()
        self.last_received['sharp_left'] = current_time
        self.message_counters['sharp_left'] += 1
        
        # Verificar calidad de datos del sensor sharp izquierdo
        is_valid = self.validate_sharp_data(msg, 'left')
        self.sharp_left_history.append((current_time, is_valid))
        
        if not is_valid:
            self.get_logger().warn("⚠️ Datos de Sharp izquierdo potencialmente inválidos")

    def sharp_right_callback(self, msg):
        current_time = time.time()
        self.last_received['sharp_right'] = current_time
        self.message_counters['sharp_right'] += 1
        
        # Verificar calidad de datos del sensor sharp derecho
        is_valid = self.validate_sharp_data(msg, 'right')
        self.sharp_right_history.append((current_time, is_valid))
        
        if not is_valid:
            self.get_logger().warn("⚠️ Datos de Sharp derecho potencialmente inválidos")

    # Métodos de validación de datos
    def validate_encoder_data(self, msg):
        """Valida que los datos del encoder sean razonables"""
        try:
            # Verificar que la posición no sea NaN
            if (abs(msg.pose.pose.position.x) > 1000 or 
                abs(msg.pose.pose.position.y) > 1000 or
                abs(msg.pose.pose.position.z) > 1000):
                return False
            
            # Verificar que la orientación sea válida
            orientation = msg.pose.pose.orientation
            if (abs(orientation.x) > 1.0 or 
                abs(orientation.y) > 1.0 or 
                abs(orientation.z) > 1.0 or 
                abs(orientation.w) > 1.0):
                return False
                
            return True
        except:
            return False

    def validate_imu_data(self, msg):
        """Valida que los datos del IMU sean razonables"""
        try:
            # Verificar velocidades angulares
            av = msg.angular_velocity
            if (abs(av.x) > self.INVALID_THRESHOLDS['imu_angular_velocity_max'] or
                abs(av.y) > self.INVALID_THRESHOLDS['imu_angular_velocity_max'] or
                abs(av.z) > self.INVALID_THRESHOLDS['imu_angular_velocity_max']):
                return False
            
            # Verificar aceleraciones lineales
            la = msg.linear_acceleration
            if (abs(la.x) > self.INVALID_THRESHOLDS['imu_linear_acceleration_max'] or
                abs(la.y) > self.INVALID_THRESHOLDS['imu_linear_acceleration_max'] or
                abs(la.z) > self.INVALID_THRESHOLDS['imu_linear_acceleration_max']):
                return False
                
            return True
        except:
            return False

    def validate_sharp_data(self, msg, sensor_name):
        """Valida que los datos del sensor sharp sean razonables"""
        try:
            # Verificar rango válido
            if (msg.range < self.INVALID_THRESHOLDS['sharp_range_min'] or 
                msg.range > self.INVALID_THRESHOLDS['sharp_range_max']):
                return False
            
            # Verificar que no sea NaN o infinito
            if not (0 <= msg.range <= 10.0):  # Rango máximo extendido para verificación
                return False
                
            return True
        except:
            return False

    def check_component_health(self, component_name):
        """Verifica el estado de salud de un componente específico"""
        current_time = time.time()
        last_received = self.last_received[component_name]
        
        if last_received is None:
            return DiagnosticStatus.ERROR, "NUNCA_RECIBIDO", 0.0
        
        time_since_received = current_time - last_received
        
        # Verificar timeout
        if time_since_received > self.TIMEOUTS[component_name]:
            return DiagnosticStatus.ERROR, "TIMEOUT", time_since_received
        
        # Verificar calidad de datos basado en historial
        data_quality = self.check_data_quality(component_name)
        if not data_quality:
            return DiagnosticStatus.WARN, "DATOS_INVALIDOS", time_since_received
        
        return DiagnosticStatus.OK, "FUNCIONANDO_OK", time_since_received

    def check_data_quality(self, component_name):
        """Verifica la calidad de datos basado en el historial"""
        history_map = {
            'encoder': self.encoder_data_history,
            'imu': self.imu_data_history,
            'sharp_front': self.sharp_front_history,
            'sharp_left': self.sharp_left_history,
            'sharp_right': self.sharp_right_history
        }
        
        history = history_map.get(component_name, [])
        if not history:
            return True  # No hay datos suficientes para evaluar
        
        # Contar cuántos de los últimos datos fueron válidos
        valid_count = sum(1 for _, is_valid in history if is_valid)
        validity_ratio = valid_count / len(history)
        
        # Considerar buena calidad si al menos 80% de los datos son válidos
        return validity_ratio >= 0.8

    def check_components_health(self):
        """Verificación periódica de todos los componentes"""
        try:
            msg = DiagnosticStatus()
            msg.name = "Diagnóstico de Componentes"
            msg.hardware_id = "Sensores_Robot"
            
            components_status = []
            overall_level = DiagnosticStatus.OK
            overall_message = "Todos los componentes OK"
            
            # Verificar cada componente
            for component in ['encoder', 'imu', 'sharp_front', 'sharp_left', 'sharp_right']:
                level, status_text, time_since = self.check_component_health(component)
                
                # Mapear nombres para display
                display_name = {
                    'encoder': 'Encoder',
                    'imu': 'IMU', 
                    'sharp_front': 'Sharp Front',
                    'sharp_left': 'Sharp Left',
                    'sharp_right': 'Sharp Right'
                }[component]
                
                components_status.append({
                    'name': display_name,
                    'level': level,
                    'status': status_text,
                    'time_since': time_since,
                    'message_count': self.message_counters[component]
                })
                
                # Actualizar nivel general (el peor nivel prevalece)
                if level > overall_level:
                    overall_level = level
                    if level == DiagnosticStatus.ERROR:
                        overall_message = f"Error en {display_name}"
                    elif level == DiagnosticStatus.WARN:
                        overall_message = f"Advertencia en {display_name}"
            
            msg.level = overall_level
            msg.message = overall_message
            
            # Agregar valores detallados
            msg.values = []
            for comp in components_status:
                msg.values.extend([
                    KeyValue(key=f"{comp['name']}_status", value=comp['status']),
                    KeyValue(key=f"{comp['name']}_level", value=str(comp['level'])),
                    KeyValue(key=f"{comp['name']}_time_since", value=f"{comp['time_since']:.1f}s"),
                    KeyValue(key=f"{comp['name']}_message_count", value=str(comp['message_count']))
                ])
            
            # Agregar timestamp
            msg.values.append(KeyValue(key="timestamp", value=str(time.time())))
            
            # Publicar diagnóstico
            self.diagnostic_pub.publish(msg)
            
            # Loggear cambios de estado (evitar spam)
            self.log_status_changes(components_status, overall_level)
            
        except Exception as e:
            self.get_logger().error(f"🚨 Error en check_components_health: {e}")

    def log_status_changes(self, components_status, overall_level):
        """Loggear solo cuando hay cambios significativos en el estado"""
        if not hasattr(self, 'last_logged_status'):
            self.last_logged_status = {}
            self.last_logged_overall = DiagnosticStatus.OK
        
        # Verificar cambios en componentes individuales
        for comp in components_status:
            last_status = self.last_logged_status.get(comp['name'])
            current_status = f"{comp['level']}:{comp['status']}"
            
            if last_status != current_status:
                icon = "🟢" if comp['level'] == DiagnosticStatus.OK else "⚠️" if comp['level'] == DiagnosticStatus.WARN else "❌"
                self.get_logger().info(f"{icon} {comp['name']}: {comp['status']} ({comp['time_since']:.1f}s)")
                self.last_logged_status[comp['name']] = current_status
        
        # Verificar cambio en estado general
        if self.last_logged_overall != overall_level:
            if overall_level == DiagnosticStatus.OK:
                self.get_logger().info("🎯 Todos los componentes funcionando correctamente")
            elif overall_level == DiagnosticStatus.WARN:
                self.get_logger().warn("⚠️ Algunos componentes con advertencias")
            else:
                self.get_logger().error("🚨 Error crítico en componentes")
            self.last_logged_overall = overall_level

    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        try:
            # Limpiar historiales
            self.encoder_data_history.clear()
            self.imu_data_history.clear()
            self.sharp_front_history.clear()
            self.sharp_left_history.clear()
            self.sharp_right_history.clear()
        except Exception as e:
            self.get_logger().warning(f"Error en cleanup: {e}")
        finally:
            super().destroy_node()

# ----------------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ComponentDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Nodo de diagnóstico apagado por usuario")
    except Exception as e:
        node.get_logger().error(f"🚨 Error fatal: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()