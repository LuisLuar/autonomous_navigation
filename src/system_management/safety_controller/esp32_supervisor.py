#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_msgs.msg import Float32MultiArray, String, Header
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
from sensor_msgs.msg import Range  # Agregar este import

class Esp32DynamicSupervisor(Node):
    def __init__(self):
        super().__init__('supervisor_esp32_dynamic')

        # Publicadores separados para cada ESP32
        self.pub_esp32_1 = self.create_publisher(DiagnosticStatus, 'status/esp32_safety', 10)
        self.pub_esp32_2 = self.create_publisher(DiagnosticStatus, 'status/esp32_control', 10)

        # Temporizador de supervisión (cada 2 segundos)
        self.timer = self.create_timer(2.0, self.check_nodes)

        # Cache para detectar cambios en estados
        self.last_node_states = {
            'Robot_Safety': {'online': False, 'topics_found': 0, 'level': 2},
            'Robot_Control': {'online': False, 'topics_found': 0, 'level': 2}
        }

        # Timestamps de última actividad por nodo
        self.last_activity_time = {
            'Robot_Safety': None,
            'Robot_Control': None
        }

        # Contadores de mensajes recibidos por topic
        self.message_counters = {
            'Robot_Safety': 0,
            'Robot_Control': 0
        }

        # Timeout de inactividad (segundos)
        self.ACTIVITY_TIMEOUT = 8.0  # 4 ciclos del supervisor

        # Nodos y tópicos esperados por cada ESP32
        self.expected_nodes = {
            'Robot_Safety': [
                '/battery/array',
                '/motors/array', 
                '/rele/control'
            ],
            'Robot_Control': [
                '/odom/unfiltered',
                '/imu/unfiltered',
                '/range/front',
                '/range/left', 
                '/range/right',
                '/cmd_vel'
            ]
        }

        # Suscriptores para monitorear actividad de topics críticos
        self.topic_subscribers = {}
        self.setup_topic_monitors()

        #self.get_logger().info('🟢 Supervisor ESP32 dinámico iniciado')

    def setup_topic_monitors(self):
        """Configura suscriptores específicos para monitorear actividad de topics críticos"""
        # Topics críticos que indican actividad real, con sus tipos de mensaje
        critical_topics = {
            'Robot_Safety': [
                ('/battery/array', Float32MultiArray),
                ('/motors/array', Float32MultiArray)
            ],
            'Robot_Control': [
                ('/odom/unfiltered', Odometry),
                ('/imu/unfiltered', Imu),
                ('/range/front',Range),
            ]
        }
        
        for node_name, topics in critical_topics.items():
            for topic, msg_type in topics:
                try:
                    # Crear suscriptor específico para el tipo de mensaje
                    subscriber = self.create_subscription(
                        msg_type=msg_type,
                        topic=topic,
                        callback=lambda msg, n=node_name, t=topic: self.topic_activity_callback(n, t),
                        qos_profile=10
                    )
                    self.topic_subscribers[topic] = subscriber
                    #self.get_logger().info(f"🔍 Monitoreando actividad en: {topic}")
                except Exception as e:
                    self.get_logger().warning(f"⚠️ No se pudo suscribir a {topic}: {e}")

    def topic_activity_callback(self, node_name, topic_name):
        """Callback que registra actividad en topics"""
        current_time = time.time()
        self.last_activity_time[node_name] = current_time
        self.message_counters[node_name] += 1
        
        # Debug ocasional (cada 10 mensajes)
        if self.message_counters[node_name] % 10 == 0:
            self.get_logger().debug(f"📡 Actividad {node_name}: {topic_name} (msg #{self.message_counters[node_name]})")

    def check_activity_timeout(self, node_name):
        """Verifica si el nodo ha estado inactivo por más del timeout"""
        current_time = time.time()
        last_activity = self.last_activity_time.get(node_name)
        
        if last_activity is None:
            # Nunca ha tenido actividad
            return True
        
        time_since_activity = current_time - last_activity
        return time_since_activity > self.ACTIVITY_TIMEOUT

    # ----------------------------------------------------------------------
    # Funciones auxiliares con reintentos
    # ----------------------------------------------------------------------
    def get_node_names(self):
        """Obtiene lista de nodos activos con reintentos."""
        for attempt in range(3):
            try:
                node_names_and_namespaces = self.get_node_names_and_namespaces()
                return [name for name, namespace in node_names_and_namespaces]
            except Exception as e:
                if attempt == 2:
                    self.get_logger().error(f"Error obteniendo nodos: {e}")
                time.sleep(0.1)
        return []

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

    # ----------------------------------------------------------------------
    # Supervisión principal
    # ----------------------------------------------------------------------
    def check_nodes(self):
        try:
            # Obtener listas actuales
            node_list = self.get_node_names()
            topic_list = self.get_topic_names()

            for expected_node, expected_topics in self.expected_nodes.items():
                # Determinar a qué ESP32 pertenece
                if expected_node == 'Robot_Safety':
                    pub = self.pub_esp32_1
                    esp_name = 'ESP32 de Seguridad'
                else:
                    pub = self.pub_esp32_2
                    esp_name = 'ESP32 de Control'

                msg = DiagnosticStatus()
                msg.name = f"{esp_name}"
                msg.hardware_id = esp_name

                # ✅ Corrección: comparación robusta de nombres
                node_detected = any(expected_node.lower() in n.lower() for n in node_list)

                # ✅ VERIFICACIÓN CRÍTICA: Comprobar timeout de actividad
                activity_timed_out = self.check_activity_timeout(expected_node)
                
                # Determinar estado real
                if not node_detected:
                    # Nodo no detectado en el graph
                    msg.level = DiagnosticStatus.ERROR  # 2 = ERROR
                    msg.message = "DESCONECTADO" #f"Nodo {expected_node} desconectado"
                    missing_topics = expected_topics
                    found_topics_count = 0
                    node_online = False
                    
                    # Resetear timestamp de actividad
                    self.last_activity_time[expected_node] = None
                    
                elif activity_timed_out:
                    # ⚠️ NODO ZOMBIE: Está en el graph pero sin actividad
                    msg.level = DiagnosticStatus.ERROR  # 2 = ERROR  
                    msg.message = "INACTIVO (TIMEOUT)" #f"Nodo {expected_node} inactivo (timeout)"
                    missing_topics = expected_topics
                    found_topics_count = 0
                    node_online = False
                    
                else:
                    # ✅ Nodo activo y comunicándose
                    node_online = True
                    
                    # Verificar tópicos del nodo
                    found_topics = [t for t in expected_topics if t in topic_list]
                    missing_topics = [t for t in expected_topics if t not in topic_list]
                    found_topics_count = len(found_topics)

                    # Evaluar nivel según tópicos detectados
                    if len(missing_topics) == 0:
                        msg.level = DiagnosticStatus.OK
                        msg.message = f"Conectado - {found_topics_count}/{len(expected_topics)} tópicos activos"
                    else:
                        msg.level = DiagnosticStatus.WARN
                        msg.message = f"Conectado - Faltan {len(missing_topics)} tópicos"

                # Información adicional
                current_time = time.time()
                last_activity = self.last_activity_time.get(expected_node)
                time_since_activity = "Nunca" if last_activity is None else f"{current_time - last_activity:.1f}s"
                msg_count = self.message_counters.get(expected_node, 0)
                
                msg.values = [
                    KeyValue(key="node_name", value=expected_node),
                    KeyValue(key="expected_topics", value=str(len(expected_topics))),
                    KeyValue(key="found_topics", value=str(found_topics_count)),
                    KeyValue(key="missing_topics", value=", ".join(missing_topics) if missing_topics else "Ninguno"),
                    KeyValue(key="status", value="ONLINE" if node_online else "OFFLINE"),
                    KeyValue(key="timestamp", value=str(current_time)),
                    KeyValue(key="last_activity", value=time_since_activity),
                    KeyValue(key="activity_timeout", value=str(self.ACTIVITY_TIMEOUT)),
                    KeyValue(key="node_detected", value=str(node_detected)),
                    KeyValue(key="activity_timed_out", value=str(activity_timed_out)),
                    KeyValue(key="message_count", value=str(msg_count)),
                ]

                # Control de logs para evitar spam
                current_state = {
                    'online': node_online,
                    'topics_found': found_topics_count,
                    'level': msg.level,
                    'activity_timed_out': activity_timed_out
                }
                last_state = self.last_node_states.get(expected_node, {
                    'online': False, 
                    'topics_found': 0, 
                    'level': 2,
                    'activity_timed_out': False
                })

                # Solo loggear si hay cambios significativos
                if (last_state['online'] != current_state['online'] or
                    last_state['topics_found'] != current_state['topics_found'] or
                    last_state['level'] != current_state['level'] or
                    last_state['activity_timed_out'] != current_state['activity_timed_out']):
                    
                    if msg.level == DiagnosticStatus.OK:
                        icon = "🟢"
                    elif msg.level == DiagnosticStatus.WARN:
                        icon = "⚠️"
                    else:
                        icon = "❌"

                    # Mensaje detallado según el tipo de problema
                    if not node_detected:
                        status_msg = f"{icon} {esp_name}: Nodo desconectado del graph"
                    elif activity_timed_out:
                        status_msg = f"{icon} {esp_name}: Nodo zombie - sin actividad por {time_since_activity}"
                    else:
                        status_msg = f"{icon} {esp_name}: {msg.message}"
                    
                    #self.get_logger().info(status_msg)
                    self.last_node_states[expected_node] = current_state

                # Publicar siempre el estado
                pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"🚨 Error en check_nodes: {e}")

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

# ----------------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Esp32DynamicSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Supervisor apagado por usuario")
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