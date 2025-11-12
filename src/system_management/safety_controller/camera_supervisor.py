#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import time

class CameraSupervisor(Node):
    def __init__(self):
        super().__init__('camera_supervisor')

        # Publicador para el estado de la cámara
        self.publisher = self.create_publisher(DiagnosticStatus, 'status/camera', 10)

        # Temporizador de supervisión (cada 2 segundos)
        self.timer = self.create_timer(2.0, self.check_camera)

        # Cache para detectar cambios en estados
        self.last_camera_state = {
            'online': False, 
            'topics_found': 0, 
            'level': 2
        }

        # Timestamps de última actividad
        self.last_activity_time = None

        # Contador de mensajes recibidos
        self.message_counter = 0

        # Timeout de inactividad (segundos)
        self.ACTIVITY_TIMEOUT = 8.0  # 4 ciclos del supervisor

        # Tópicos esperados de la cámara
        self.expected_topics = [
            '/camera/rgb/image_raw',
            '/camera/depth/image_raw',
            '/camera/rgb/camera_info',
            '/camera/depth/camera_info',
            '/camera/depth_registered/image_raw',
            '/camera/depth_registered/points',
            '/camera/ir/image',
            '/camera/ir/camera_info'
        ]

        # Tópicos críticos para monitorear actividad
        self.critical_topics = [
            '/camera/rgb/image_raw',
            '/camera/depth/image_raw'
        ]

        # Suscriptores para monitorear actividad de topics críticos
        self.topic_subscribers = {}
        self.setup_topic_monitors()

        self.get_logger().info('🟢 Supervisor de Cámara iniciado')

    def setup_topic_monitors(self):
        """Configura suscriptores para monitorear actividad de topics críticos"""
        from sensor_msgs.msg import Image, CameraInfo, PointCloud2
        
        # Mapeo de tipos de mensaje para los topics
        topic_types = {
            '/camera/rgb/image_raw': Image,
            '/camera/depth/image_raw': Image,
            '/camera/rgb/camera_info': CameraInfo,
            '/camera/depth/camera_info': CameraInfo,
            '/camera/depth_registered/image_raw': Image,
            '/camera/depth_registered/points': PointCloud2,
            '/camera/ir/image': Image,
            '/camera/ir/camera_info': CameraInfo
        }
        
        for topic in self.critical_topics:
            try:
                msg_type = topic_types.get(topic, Image)
                # Crear suscriptor específico para el tipo de mensaje
                subscriber = self.create_subscription(
                    msg_type=msg_type,
                    topic=topic,
                    callback=lambda msg, t=topic: self.topic_activity_callback(t),
                    qos_profile=10
                )
                self.topic_subscribers[topic] = subscriber
                self.get_logger().info(f"🔍 Monitoreando actividad en: {topic}")
            except Exception as e:
                self.get_logger().warning(f"⚠️ No se pudo suscribir a {topic}: {e}")

    def topic_activity_callback(self, topic_name):
        """Callback que registra actividad en topics de cámara"""
        current_time = time.time()
        self.last_activity_time = current_time
        self.message_counter += 1
        
        # Debug ocasional (cada 20 mensajes)
        if self.message_counter % 20 == 0:
            self.get_logger().debug(f"📷 Actividad cámara: {topic_name} (msg #{self.message_counter})")

    def check_activity_timeout(self):
        """Verifica si la cámara ha estado inactiva por más del timeout"""
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
                if attempt == 2:
                    self.get_logger().error(f"Error obteniendo topics: {e}")
                time.sleep(0.1)
        return []

    def check_camera(self):
        """Función principal que verifica el estado de la cámara"""
        try:
            # Obtener lista actual de topics
            topic_list = self.get_topic_names()

            msg = DiagnosticStatus()
            msg.name = "Cámara"
            msg.hardware_id = "RGBD_Camera"

            # Verificar tópicos de la cámara
            found_topics = [t for t in self.expected_topics if t in topic_list]
            missing_topics = [t for t in self.expected_topics if t not in topic_list]
            found_topics_count = len(found_topics)

            # ✅ VERIFICACIÓN CRÍTICA: Comprobar timeout de actividad
            activity_timed_out = self.check_activity_timeout()
            
            # Determinar estado real
            if found_topics_count == 0:
                # No se detectaron topics de cámara
                msg.level = DiagnosticStatus.ERROR  # 2 = ERROR
                msg.message = "DESCONECTADA"
                camera_online = False
                
                # Resetear timestamp de actividad
                self.last_activity_time = None
                
            elif activity_timed_out and found_topics_count > 0:
                # ⚠️ CÁMARA ZOMBIE: Tópicos presentes pero sin actividad
                msg.level = DiagnosticStatus.ERROR  # 2 = ERROR  
                msg.message = "INACTIVA (TIMEOUT)"
                camera_online = False
                
            else:
                # ✅ Cámara activa y comunicándose
                camera_online = True

                # Evaluar nivel según tópicos detectados
                if len(missing_topics) == 0:
                    msg.level = DiagnosticStatus.OK
                    msg.message = f"Conectada- {found_topics_count}/{len(self.expected_topics)} tópicos activos"
                elif len(missing_topics) <= 2:  # Permitir faltar algunos tópicos no críticos
                    msg.level = DiagnosticStatus.WARN
                    msg.message = f"CONECTADA - Faltan {len(missing_topics)} tópicos"
                else:
                    msg.level = DiagnosticStatus.WARN
                    msg.message = f"CONECTADA - Faltan {len(missing_topics)} tópicos"

            # Información adicional
            current_time = time.time()
            last_activity = self.last_activity_time
            time_since_activity = "Nunca" if last_activity is None else f"{current_time - last_activity:.1f}s"
            
            msg.values = [
                KeyValue(key="expected_topics", value=str(len(self.expected_topics))),
                KeyValue(key="found_topics", value=str(found_topics_count)),
                KeyValue(key="missing_topics_count", value=str(len(missing_topics))),
                KeyValue(key="missing_topics_list", value=", ".join(missing_topics) if missing_topics else "Ninguno"),
                KeyValue(key="status", value="ONLINE" if camera_online else "OFFLINE"),
                KeyValue(key="timestamp", value=str(current_time)),
                KeyValue(key="last_activity", value=time_since_activity),
                KeyValue(key="activity_timeout", value=str(self.ACTIVITY_TIMEOUT)),
                KeyValue(key="activity_timed_out", value=str(activity_timed_out)),
                KeyValue(key="message_count", value=str(self.message_counter)),
                KeyValue(key="critical_topics", value=", ".join(self.critical_topics)),
            ]

            # Control de logs para evitar spam
            current_state = {
                'online': camera_online,
                'topics_found': found_topics_count,
                'level': msg.level,
                'activity_timed_out': activity_timed_out
            }

            # Solo loggear si hay cambios significativos
            if (self.last_camera_state['online'] != current_state['online'] or
                self.last_camera_state['topics_found'] != current_state['topics_found'] or
                self.last_camera_state['level'] != current_state['level'] or
                self.last_camera_state['activity_timed_out'] != current_state['activity_timed_out']):
                
                if msg.level == DiagnosticStatus.OK:
                    icon = "🟢"
                elif msg.level == DiagnosticStatus.WARN:
                    icon = "🟡"
                else:
                    icon = "🔴"

                # Mensaje detallado según el tipo de problema
                if found_topics_count == 0:
                    status_msg = f"{icon} Cámara: No se detectaron tópicos - posiblemente desconectada"
                elif activity_timed_out:
                    status_msg = f"{icon} Cámara: Sin actividad por {time_since_activity}"
                else:
                    status_msg = f"{icon} Cámara: {msg.message}"
                
                self.get_logger().info(status_msg)
                self.last_camera_state = current_state

            # Publicar siempre el estado
            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"🚨 Error en check_camera: {e}")

    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        try:
            # Limpiar suscriptores
            for subscriber in self.topic_subscribers.values():
                self.destroy_subscription(subscriber)
            self.topic_subscribers.clear()
        except Exception as e:
            self.get_logger().warning(f"Error en cleanup: {e}")
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Supervisor de cámara apagado por usuario")
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