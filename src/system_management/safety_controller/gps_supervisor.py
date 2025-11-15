#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import time

class GPSSupervisor(Node):
    def __init__(self):
        super().__init__('gps_supervisor')

        # Publicador para el estado del GPS
        self.publisher = self.create_publisher(DiagnosticStatus, 'status/gps', 10)

        # Temporizador de supervisión (cada 2 segundos)
        self.timer = self.create_timer(2.0, self.check_gps)

        # Cache para detectar cambios en estados
        self.last_gps_state = {
            'online': False, 
            'topics_found': 0, 
            'level': 2,
            'has_fix': False,
            'quality_level': 'UNKNOWN'
        }

        # Timestamps de última actividad
        self.last_activity_time = None

        # Contador de mensajes recibidos
        self.message_counter = 0

        # Timeout de inactividad (segundos)
        self.ACTIVITY_TIMEOUT = 10.0  # 5 ciclos del supervisor

        # Tópicos esperados del GPS
        self.expected_topics = [
            '/gps/fix',
            '/gps/vel',
            '/gps/vel_with_cov',
            '/gps/course',
            '/gps/raw',
            '/gps/info'
        ]

        # Tópicos críticos para monitorear actividad
        self.critical_topics = [
            '/gps/fix',
            '/gps/vel',
            '/gps/raw'
        ]

        # Suscriptores para monitorear actividad de topics críticos
        self.topic_subscribers = {}
        self.setup_topic_monitors()

        # Variables para almacenar estado del GPS
        self.last_fix_data = None
        self.last_velocity_data = None
        self.fix_quality = 0
        self.satellites_used = 0
        self.satellites_visible = 0
        self.hdop = None
        self.current_speed = 0.0

        # Para parsear información del topic /gps/info
        self.last_info_data = None

        # Umbrales de calidad GPS
        self.QUALITY_THRESHOLDS = {
            'excellent': {'sats_min': 8, 'hdop_max': 1.0},
            'good': {'sats_min': 6, 'hdop_max': 2.0},
            'moderate': {'sats_min': 4, 'hdop_max': 5.0},
            'poor': {'sats_min': 3, 'hdop_max': 10.0}
        }

        #self.get_logger().info('🟢 Supervisor de GPS iniciado')

    def setup_topic_monitors(self):
        """Configura suscriptores para monitorear actividad de topics críticos"""
        from sensor_msgs.msg import NavSatFix
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import String
        
        # Suscriptor para /gps/fix
        try:
            self.fix_subscriber = self.create_subscription(
                NavSatFix,
                '/gps/fix',
                self.fix_callback,
                10
            )
            self.topic_subscribers['/gps/fix'] = self.fix_subscriber
            #self.get_logger().info("🔍 Monitoreando actividad en: /gps/fix")
        except Exception as e:
            #self.get_logger().warning(f"⚠️ No se pudo suscribir a /gps/fix: {e}")
            pass

        # Suscriptor para /gps/vel
        try:
            self.vel_subscriber = self.create_subscription(
                TwistStamped,
                '/gps/vel',
                self.velocity_callback,
                10
            )
            self.topic_subscribers['/gps/vel'] = self.vel_subscriber
            #self.get_logger().info("🔍 Monitoreando actividad en: /gps/vel")
        except Exception as e:
            #self.get_logger().warning(f"⚠️ No se pudo suscribir a /gps/vel: {e}")
            pass

        # Suscriptor para /gps/raw (actividad general)
        try:
            self.raw_subscriber = self.create_subscription(
                String,
                '/gps/raw',
                self.raw_callback,
                10
            )
            self.topic_subscribers['/gps/raw'] = self.raw_subscriber
            #self.get_logger().info("🔍 Monitoreando actividad en: /gps/raw")
        except Exception as e:
            #self.get_logger().warning(f"⚠️ No se pudo suscribir a /gps/raw: {e}")
            pass

        # Suscriptor para /gps/info (información detallada)
        try:
            self.info_subscriber = self.create_subscription(
                String,
                '/gps/info',
                self.info_callback,
                10
            )
            self.topic_subscribers['/gps/info'] = self.info_subscriber
            #self.get_logger().info("🔍 Monitoreando actividad en: /gps/info")
        except Exception as e:
            #self.get_logger().warning(f"⚠️ No se pudo suscribir a /gps/info: {e}")
            pass

    def fix_callback(self, msg):
        """Callback para mensajes de fix del GPS"""
        current_time = time.time()
        self.last_activity_time = current_time
        self.message_counter += 1
        self.last_fix_data = msg

    def velocity_callback(self, msg):
        """Callback para mensajes de velocidad del GPS"""
        current_time = time.time()
        self.last_activity_time = current_time
        self.last_velocity_data = msg
        self.current_speed = abs(msg.twist.linear.x)  # Velocidad en m/s

    def raw_callback(self, msg):
        """Callback para mensajes raw del GPS - indica actividad general"""
        current_time = time.time()
        self.last_activity_time = current_time

    def info_callback(self, msg):
        """Callback mejorado para mensajes de info del GPS - contiene información detallada"""
        current_time = time.time()
        self.last_activity_time = current_time
        self.last_info_data = msg.data
        
        # Resetear valores por si el parsing falla
        self.fix_quality = 0
        self.satellites_used = 0
        self.satellites_visible = 0
        self.hdop = None
        
        try:
            info_str = msg.data
            # Múltiples estrategias de parsing para mayor robustez
            parts = info_str.split()
            for part in parts:
                if part.startswith('fix_q:'):
                    self.fix_quality = int(part.split(':')[1])
                elif part.startswith('sats_usado:'):
                    self.satellites_used = int(part.split(':')[1])
                elif part.startswith('sats_visibles:'):
                    self.satellites_visible = int(part.split(':')[1])
                elif part.startswith('hdop:'):
                    self.hdop = float(part.split(':')[1])
                # Alternativas de nomenclatura para mayor compatibilidad
                elif part.startswith('satellites:'):
                    self.satellites_used = int(part.split(':')[1])
                elif part.startswith('satellites_visible:'):
                    self.satellites_visible = int(part.split(':')[1])
                elif part.startswith('sats:'):
                    self.satellites_used = int(part.split(':')[1])
                    
        except Exception as e:
            #self.get_logger().debug(f"Error parseando info GPS: {e}")
            pass

    def check_activity_timeout(self):
        """Verifica si el GPS ha estado inactivo por más del timeout"""
        current_time = time.time()
        last_activity = self.last_activity_time
        
        if last_activity is None:
            # Nunca ha tenido actividad
            return True
        
        time_since_activity = current_time - last_activity
        return time_since_activity > self.ACTIVITY_TIMEOUT

    def get_topic_names(self):
        """Obtiene lista de topics activos con reintentos."""
        for attempt in range(3):
            try:
                topic_names_and_types = self.get_topic_names_and_types()
                return [name for name, types in topic_names_and_types]
            except Exception as e:
                """if attempt == 2:
                    self.get_logger().error(f"Error obteniendo topics: {e}")"""
                time.sleep(0.1)
        return []

    def has_gps_fix(self):
        """Determina si el GPS tiene fix basado en la información disponible"""
        # Prioridad 1: Usar fix_quality del mensaje /gps/info
        if hasattr(self, 'fix_quality') and self.fix_quality > 0:
            return self.fix_quality >= 1  # fix_quality 1 o mayor indica fix
        
        # Prioridad 2: Usar status del mensaje NavSatFix (estándar ROS2)
        if self.last_fix_data:
            # NavSatStatus.STATUS_FIX = 1, STATUS_SBAS_FIX = 2, etc.
            return self.last_fix_data.status.status >= 1
        
        return False

    def get_fix_status_description(self):
        """Obtiene una descripción legible del estado del fix"""
        if hasattr(self, 'fix_quality'):
            if self.fix_quality == 0:
                return "NO FIX"
            elif self.fix_quality == 1:
                return "GPS FIX"
            elif self.fix_quality == 2:
                return "DGPS FIX"
            else:
                return f"FIX_QUALITY_{self.fix_quality}"
        
        if self.last_fix_data:
            status = self.last_fix_data.status.status
            if status == 0:
                return "NO FIX"
            elif status == 1:
                return "FIX"
            elif status == 2:
                return "SBAS FIX"
            else:
                return f"STATUS_{status}"
        
        return "UNKNOWN"

    def evaluate_gps_quality(self):
        """
        Evalúa la calidad del posicionamiento GPS basado en satélites y HDOP
        Retorna: (nivel_calidad, descripción, recomendación)
        """
        if not self.has_gps_fix():
            return "NO_FIX", "Sin posicionamiento disponible", "Buscar mejor ubicación"
        
        # Evaluar basado en número de satélites y HDOP
        sats = self.satellites_used
        hdop_val = self.hdop if self.hdop is not None else float('inf')

        fix_status_desc = self.get_fix_status_description()
        
        # Determinar nivel de calidad
        if sats >= self.QUALITY_THRESHOLDS['excellent']['sats_min'] and hdop_val <= self.QUALITY_THRESHOLDS['excellent']['hdop_max']:
            return "EXCELLENT", f"Señal excelente - {sats} satélites - HDOP: {hdop_val:.2f} - {fix_status_desc}", "Posicionamiento óptimo"
        elif sats >= self.QUALITY_THRESHOLDS['good']['sats_min'] and hdop_val <= self.QUALITY_THRESHOLDS['good']['hdop_max']:
            return "GOOD", f"Señal buena - {sats} satélites - HDOP: {hdop_val:.2f} - {fix_status_desc}", "Posicionamiento confiable"
        elif sats >= self.QUALITY_THRESHOLDS['moderate']['sats_min'] and hdop_val <= self.QUALITY_THRESHOLDS['moderate']['hdop_max']:
            return "MODERATE", f"Señal moderada - {sats} satélites - HDOP: {hdop_val:.2f} - {fix_status_desc}", "Posicionamiento aceptable"
        elif sats >= self.QUALITY_THRESHOLDS['poor']['sats_min']:
            return "POOR", f"Señal pobre - {sats} satélites - HDOP: {hdop_val:.2f} - {fix_status_desc}", "Buscar mejor recepción"
        else:
            return "MARGINAL", f"Mínima - {sats} satélites - HDOP: {hdop_val:.2f} - {fix_status_desc}", "Revisar visibilidad cielo"

    def check_gps(self):
        """Función principal que verifica el estado del GPS con información de calidad mejorada"""
        try:
            # Obtener lista actual de topics
            topic_list = self.get_topic_names()

            msg = DiagnosticStatus()
            msg.name = "GPS BU-3535N5"
            msg.hardware_id = "USB_GPS_Receiver"

            # Verificar tópicos del GPS
            found_topics = [t for t in self.expected_topics if t in topic_list]
            missing_topics = [t for t in self.expected_topics if t not in topic_list]
            found_topics_count = len(found_topics)

            # ✅ VERIFICACIÓN CRÍTICA: Comprobar timeout de actividad
            activity_timed_out = self.check_activity_timeout()
            
            # Verificar si tiene fix usando nuestra función mejorada
            has_fix = self.has_gps_fix()
            fix_status_desc = self.get_fix_status_description()
            
            # ✅ NUEVO: Evaluar calidad del posicionamiento
            quality_level, quality_desc, quality_advice = self.evaluate_gps_quality()

            # Determinar estado real
            if found_topics_count == 0:
                # No se detectaron topics de GPS
                msg.level = DiagnosticStatus.ERROR
                msg.message = "GPS DESCONECTADO"
                gps_online = False
                
                # Resetear timestamp de actividad
                self.last_activity_time = None
                
            elif activity_timed_out and found_topics_count > 0:
                # ⚠️ GPS ZOMBIE: Tópicos presentes pero sin actividad
                msg.level = DiagnosticStatus.ERROR  
                msg.message = "INACTIVO (TIMEOUT)"
                gps_online = False
                
            else:
                # ✅ GPS activo y comunicándose
                gps_online = True

                # Evaluar nivel según calidad y tópicos detectados
                if quality_level in ["EXCELLENT", "GOOD"] and len(missing_topics) == 0:
                    msg.level = DiagnosticStatus.OK
                    msg.message = f"{quality_desc}"
                elif quality_level in ["EXCELLENT", "GOOD"] and len(missing_topics) <= 2:
                    msg.level = DiagnosticStatus.OK
                    msg.message = f" {quality_desc} - Faltan {len(missing_topics)} tópicos"
                elif quality_level == "MODERATE":
                    msg.level = DiagnosticStatus.WARN
                    msg.message = f" {quality_desc}"
                elif quality_level in ["POOR", "MARGINAL"]:
                    msg.level = DiagnosticStatus.WARN
                    msg.message = f"{quality_desc}"
                elif not has_fix and found_topics_count >= 3:
                    msg.level = DiagnosticStatus.WARN
                    msg.message = f" Conectado - {fix_status_desc} - Esperando señal satélites"
                else:
                    msg.level = DiagnosticStatus.WARN
                    msg.message = f" Conectado - Estado limitado - Faltan {len(missing_topics)} tópicos"

            # Información adicional detallada con métricas de calidad
            current_time = time.time()
            last_activity = self.last_activity_time
            time_since_activity = "Nunca" if last_activity is None else f"{current_time - last_activity:.1f}s"
            
            # Coordenadas si están disponibles
            coordinates = "No disponibles"
            if self.last_fix_data and (has_fix or self.last_fix_data.latitude != 0.0):
                lat = self.last_fix_data.latitude
                lon = self.last_fix_data.longitude
                alt = self.last_fix_data.altitude if self.last_fix_data.altitude else 0.0
                coordinates = f"Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt:.1f}m"

            msg.values = [
                # Información de conectividad
                KeyValue(key="expected_topics", value=str(len(self.expected_topics))),
                KeyValue(key="found_topics", value=str(found_topics_count)),
                KeyValue(key="missing_topics_count", value=str(len(missing_topics))),
                KeyValue(key="missing_topics_list", value=", ".join(missing_topics) if missing_topics else "Ninguno"),
                KeyValue(key="status", value="ONLINE" if gps_online else "OFFLINE"),
                
                # ✅ NUEVO: Información de calidad mejorada
                KeyValue(key="has_fix", value=str(has_fix)),
                KeyValue(key="fix_status", value=fix_status_desc),
                KeyValue(key="fix_quality", value=str(self.fix_quality)),
                KeyValue(key="quality_level", value=quality_level),
                KeyValue(key="quality_description", value=quality_desc),
                KeyValue(key="quality_advice", value=quality_advice),
                KeyValue(key="satellites_used", value=str(self.satellites_used)),
                KeyValue(key="satellites_visible", value=str(self.satellites_visible)),
                KeyValue(key="hdop", value=f"{self.hdop:.2f}" if self.hdop is not None else "N/A"),
                KeyValue(key="hdop_interpretation", value=self.interpret_hdop()),
                
                # Información de posición y movimiento
                KeyValue(key="coordinates", value=coordinates),
                KeyValue(key="current_speed", value=f"{self.current_speed:.2f} m/s"),
                
                # Información de diagnóstico
                KeyValue(key="timestamp", value=str(current_time)),
                KeyValue(key="last_activity", value=time_since_activity),
                KeyValue(key="activity_timeout", value=str(self.ACTIVITY_TIMEOUT)),
                KeyValue(key="activity_timed_out", value=str(activity_timed_out)),
                KeyValue(key="message_count", value=str(self.message_counter)),
                KeyValue(key="critical_topics", value=", ".join(self.critical_topics)),
            ]

            # Control de logs para evitar spam
            current_state = {
                'online': gps_online,
                'topics_found': found_topics_count,
                'level': msg.level,
                'has_fix': has_fix,
                'quality_level': quality_level
            }

            # Solo loggear si hay cambios significativos
            if (self.last_gps_state['online'] != current_state['online'] or
                self.last_gps_state['topics_found'] != current_state['topics_found'] or
                self.last_gps_state['level'] != current_state['level'] or
                self.last_gps_state['has_fix'] != current_state['has_fix'] or
                self.last_gps_state['quality_level'] != current_state['quality_level']):
                self.last_gps_state = current_state

            # Publicar siempre el estado
            self.publisher.publish(msg)

        except Exception as e:
            #self.get_logger().error(f"🚨 Error en check_gps: {e}")
            pass

    def interpret_hdop(self):
        """Proporciona una interpretación legible del valor HDOP"""
        if self.hdop is None:
            return "No disponible"
        elif self.hdop <= 1.0:
            return "Excelente"
        elif self.hdop <= 2.0:
            return "Bueno"
        elif self.hdop <= 5.0:
            return "Moderado"
        elif self.hdop <= 10.0:
            return "Regular"
        else:
            return "Pobre"

    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        try:
            # Limpiar suscriptores
            for subscriber in self.topic_subscribers.values():
                self.destroy_subscription(subscriber)
            self.topic_subscribers.clear()
        except Exception as e:
            #self.get_logger().warning(f"Error en cleanup: {e}")
            pass
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Supervisor de GPS apagado por usuario")
    except Exception as e:
        node.get_logger().error(f"🚨 Error fatal: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown

if __name__ == '__main__':
    main()