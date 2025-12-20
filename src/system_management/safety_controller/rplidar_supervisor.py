#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import LaserScan
import time
import math

class RPLidarSupervisor(Node):
    def __init__(self):
        super().__init__('rplidar_supervisor')

        # Publicador para el estado del RPLIDAR
        self.publisher = self.create_publisher(DiagnosticStatus, 'status/rplidar', 10)

        # Temporizador de supervisión (cada 2 segundos)
        self.timer = self.create_timer(2.0, self.check_rplidar)

        # Cache para detectar cambios en estados
        self.last_lidar_state = {
            'online': False, 
            'has_scan_data': False,
            'level': 2,
            'point_count': 0
        }

        # Timestamps de última actividad
        self.last_scan_time = None
        self.last_activity_time = None

        # Contador de mensajes recibidos
        self.message_counter = 0

        # Timeout de inactividad (segundos)
        self.ACTIVITY_TIMEOUT = 5.0  # Más corto para LIDAR por la alta frecuencia

        # Tópicos esperados del RPLIDAR
        self.expected_topics = [
            '/scan'
        ]

        # Variables para métricas del LIDAR
        self.last_scan_data = None
        self.scan_point_count = 0
        self.average_range = 0.0
        self.min_range = 0.0
        self.max_range = 0.0
        self.valid_points_ratio = 0.0

        # Umbrales para diagnóstico
        self.MIN_VALID_POINTS_RATIO = 0.3  # Al menos 30% de puntos válidos
        self.MIN_POINT_COUNT = 100         # Mínimo número de puntos por scan
        self.MAX_SCAN_GAP = 2.0            # Máximo tiempo entre scans

        # Suscriptores para monitorear actividad
        self.setup_scan_monitor()

        #self.get_logger().info('Supervisor de RPLIDAR iniciado')

    def setup_scan_monitor(self):
        """Configura suscriptor para monitorear datos del LIDAR"""
        try:
            self.scan_subscriber = self.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,
                10
            )
            #self.get_logger().info("Monitoreando actividad en: /scan")
        except Exception as e:
            #self.get_logger().warning(f"No se pudo suscribir a /scan: {e}")
            pass

    def scan_callback(self, msg):
        """Callback para mensajes de scan del LIDAR"""
        current_time = time.time()
        self.last_activity_time = current_time
        self.last_scan_time = current_time
        self.message_counter += 1
        self.last_scan_data = msg

        # Analizar calidad de los datos del scan
        self.analyze_scan_quality(msg)

    def analyze_scan_quality(self, scan_msg):
        """Analiza la calidad de los datos del scan LIDAR"""
        try:
            ranges = scan_msg.ranges
            total_points = len(ranges)
            
            # Contar puntos válidos (no infinitos y dentro de rangos razonables)
            valid_points = 0
            range_sum = 0.0
            min_range = float('inf')
            max_range = 0.0
            
            for r in ranges:
                if not math.isinf(r) and r >= scan_msg.range_min and r <= scan_msg.range_max:
                    valid_points += 1
                    range_sum += r
                    min_range = min(min_range, r)
                    max_range = max(max_range, r)
            
            # Calcular métricas
            self.scan_point_count = total_points
            self.valid_points_ratio = valid_points / total_points if total_points > 0 else 0
            self.average_range = range_sum / valid_points if valid_points > 0 else 0
            self.min_range = min_range if min_range != float('inf') else 0
            self.max_range = max_range
            
        except Exception as e:
            #self.get_logger().warning(f"Error analizando calidad de scan: {e}")
            pass

    def check_activity_timeout(self):
        """Verifica si el LIDAR ha estado inactivo por más del timeout"""
        current_time = time.time()
        last_activity = self.last_activity_time
        
        if last_activity is None:
            # Nunca ha tenido actividad
            return True
        
        time_since_activity = current_time - last_activity
        return time_since_activity > self.ACTIVITY_TIMEOUT

    def check_scan_timeout(self):
        """Verifica si ha pasado demasiado tiempo desde el último scan"""
        current_time = time.time()
        last_scan = self.last_scan_time
        
        if last_scan is None:
            return True
        
        time_since_scan = current_time - last_scan
        return time_since_scan > self.MAX_SCAN_GAP

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

    def has_good_scan_data(self):
        """Determina si los datos del scan son de buena calidad"""
        if self.last_scan_data is None:
            return False
        
        # Verificar número mínimo de puntos
        if self.scan_point_count < self.MIN_POINT_COUNT:
            return False
        
        # Verificar ratio mínimo de puntos válidos
        if self.valid_points_ratio < self.MIN_VALID_POINTS_RATIO:
            return False
            
        return True

    def check_rplidar(self):
        """Función principal que verifica el estado del RPLIDAR"""
        try:
            # Obtener lista actual de topics
            topic_list = self.get_topic_names()

            msg = DiagnosticStatus()
            msg.name = "RPLIDAR A1"
            msg.hardware_id = "Laser_Scanner"

            # Verificar tópicos del LIDAR
            found_topics = [t for t in self.expected_topics if t in topic_list]
            missing_topics = [t for t in self.expected_topics if t not in topic_list]
            found_topics_count = len(found_topics)

            # VERIFICACIONES CRÍTICAS
            activity_timed_out = self.check_activity_timeout()
            scan_timed_out = self.check_scan_timeout()
            has_good_data = self.has_good_scan_data()
            has_scan_data = self.last_scan_data is not None

            # Determinar estado real
            if found_topics_count == 0:
                # No se detectaron topics de LIDAR
                msg.level = DiagnosticStatus.ERROR
                msg.message = "DESCONECTADO"
                lidar_online = False
                self.last_activity_time = None
                
            elif activity_timed_out and found_topics_count > 0:
                # LIDAR ZOMBIE: Tópicos presentes pero sin actividad
                msg.level = DiagnosticStatus.ERROR
                msg.message = "INACTIVO (TIMEOUT)"
                lidar_online = False
                
            elif scan_timed_out and has_scan_data:
                # LIDAR CON PROBLEMAS: Tiempo entre scans muy largo
                msg.level = DiagnosticStatus.WARN
                msg.message = "CON BAJA FRECUENCIA"
                lidar_online = True
                
            elif has_scan_data and not has_good_data:
                # LIDAR CON DATOS DE BAJA CALIDAD
                msg.level = DiagnosticStatus.WARN
                msg.message = "CON DATOS DE BAJA CALIDAD"
                lidar_online = True
                
            else:
                # LIDAR activo y con buena calidad de datos
                lidar_online = True
                msg.level = DiagnosticStatus.OK
                msg.message = f"Conectado - {self.scan_point_count} puntos, {self.valid_points_ratio*100:.1f}% válidos"

            # Información adicional detallada
            current_time = time.time()
            last_activity = self.last_activity_time
            last_scan = self.last_scan_time
            
            time_since_activity = "Nunca" if last_activity is None else f"{current_time - last_activity:.1f}s"
            time_since_scan = "Nunca" if last_scan is None else f"{current_time - last_scan:.1f}s"

            # Información del scan actual
            scan_info = "No disponible"
            range_info = "No disponible"
            if self.last_scan_data and has_scan_data:
                scan_info = f"{self.scan_point_count} puntos"
                range_info = f"min: {self.min_range:.2f}m, max: {self.max_range:.2f}m, avg: {self.average_range:.2f}m"

            msg.values = [
                KeyValue(key="expected_topics", value=str(len(self.expected_topics))),
                KeyValue(key="found_topics", value=str(found_topics_count)),
                KeyValue(key="missing_topics", value=", ".join(missing_topics) if missing_topics else "Ninguno"),
                KeyValue(key="status", value="ONLINE" if lidar_online else "OFFLINE"),
                KeyValue(key="has_scan_data", value=str(has_scan_data)),
                KeyValue(key="scan_quality_good", value=str(has_good_data)),
                KeyValue(key="scan_points", value=scan_info),
                KeyValue(key="valid_points_ratio", value=f"{self.valid_points_ratio*100:.1f}%"),
                KeyValue(key="range_info", value=range_info),
                KeyValue(key="timestamp", value=str(current_time)),
                KeyValue(key="last_activity", value=time_since_activity),
                KeyValue(key="last_scan", value=time_since_scan),
                KeyValue(key="activity_timeout", value=str(self.ACTIVITY_TIMEOUT)),
                KeyValue(key="scan_timeout", value=str(self.MAX_SCAN_GAP)),
                KeyValue(key="activity_timed_out", value=str(activity_timed_out)),
                KeyValue(key="scan_timed_out", value=str(scan_timed_out)),
                KeyValue(key="message_count", value=str(self.message_counter)),
                KeyValue(key="min_valid_points_ratio", value=f"{self.MIN_VALID_POINTS_RATIO*100:.1f}%"),
                KeyValue(key="min_point_count", value=str(self.MIN_POINT_COUNT)),
            ]

            # Control de logs para evitar spam
            current_state = {
                'online': lidar_online,
                'has_scan_data': has_scan_data,
                'level': msg.level,
                'point_count': self.scan_point_count
            }

            # Solo loggear si hay cambios significativos
            """if (self.last_lidar_state['online'] != current_state['online'] or
                self.last_lidar_state['has_scan_data'] != current_state['has_scan_data'] or
                self.last_lidar_state['level'] != current_state['level'] or
                abs(self.last_lidar_state['point_count'] - current_state['point_count']) > 50):  # Cambio significativo en puntos
                
                if msg.level == DiagnosticStatus.OK:
                    icon = "🟢"
                elif msg.level == DiagnosticStatus.WARN:
                    icon = "🟡"
                else:
                    icon = "🔴"
                self.last_lidar_state = current_state"""

            # Publicar siempre el estado
            self.publisher.publish(msg)

        except Exception as e:
            #self.get_logger().error(f"Error en check_rplidar: {e}")
            pass

    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        try:
            if hasattr(self, 'scan_subscriber'):
                self.destroy_subscription(self.scan_subscriber)
        except Exception as e:
            #self.get_logger().warning(f"Error en cleanup: {e}")
            pass
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RPLidarSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info("Supervisor de RPLIDAR apagado por usuario")
        pass
    except Exception as e:
        node.get_logger().error(f"Error fatal: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown

if __name__ == '__main__':
    main()