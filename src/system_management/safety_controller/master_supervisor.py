#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
import threading
from collections import defaultdict
from std_msgs.msg import Bool

class StatusMonitor(Node):
    def __init__(self):
        super().__init__('master_supervisor')
        
        # Parámetros
        self.declare_parameter('status_topic_prefix', '/status/')
        self.declare_parameter('global_status_topic', '/global_status')
        self.declare_parameter('discovery_interval', 2.0)  # segundos
        self.declare_parameter('warning_blink_interval', 1.0)  # segundos para parpadeo
        
        self.status_topic_prefix = self.get_parameter('status_topic_prefix').value
        self.global_status_topic = self.get_parameter('global_status_topic').value
        self.discovery_interval = self.get_parameter('discovery_interval').value
        self.warning_blink_interval = self.get_parameter('warning_blink_interval').value
        
        # Publicador para el estado global
        self.global_status_publisher = self.create_publisher(
            DiagnosticStatus, 
            self.global_status_topic, 
            10
        )

        self.light_safety_pub = self.create_publisher(Bool, '/light/safety', 10)
        
        # Diccionarios para almacenar estados y suscriptores
        self.status_data = {}  # {topic_name: DiagnosticStatus}
        self.status_subscriptions = {}  # {topic_name: subscription}

        # Variables para control del parpadeo
        self.current_global_state = DiagnosticStatus.STALE  # Estado inicial
        self.blink_state = False  # Estado actual del parpadeo (True=ON, False=OFF)
        self.blink_timer = None  # Timer para el parpadeo
        
        # Lock para acceso thread-safe
        self.lock = threading.Lock()
        
        # Timer para descubrir nuevos topicos
        self.discovery_timer = self.create_timer(self.discovery_interval, self.discover_status_topics)
        
        # Descubrir topicos iniciales
        self.discover_status_topics()
        
        #self.get_logger().info('Master Supervisor iniciado')
    
    def discover_status_topics(self):
        """Descubre y se suscribe a nuevos topicos de status"""
        try:
            topic_names_and_types = self.get_topic_names_and_types()
            status_topics = []
            
            for topic_name, topic_types in topic_names_and_types:
                # Buscar topicos que empiecen con el prefijo y sean del tipo correcto
                if (topic_name.startswith(self.status_topic_prefix) and 
                    'diagnostic_msgs/msg/DiagnosticStatus' in topic_types):
                    status_topics.append(topic_name)
            
            with self.lock:
                # Suscribirse a nuevos topicos
                for topic in status_topics:
                    if topic not in self.status_subscriptions:
                        self.create_subscription_for_topic(topic)
                
                # Eliminar suscripciones a topicos que ya no existen
                topics_to_remove = []
                for existing_topic in self.status_subscriptions.keys():
                    if existing_topic not in status_topics:
                        topics_to_remove.append(existing_topic)
                
                for topic in topics_to_remove:
                    self.remove_subscription_for_topic(topic)
                        
        except Exception as e:
            #self.get_logger().error(f'Error descubriendo topicos: {str(e)}')
            pass
    
    def create_subscription_for_topic(self, topic_name):
        """Crea una suscripción para un topico específico"""
        try:
            subscription = self.create_subscription(
                DiagnosticStatus,
                topic_name,
                lambda msg, tn=topic_name: self.status_callback(msg, tn),
                10
            )
            self.status_subscriptions[topic_name] = subscription
            self.status_data[topic_name] = None  # Estado inicial desconocido
            
            #self.get_logger().info(f'Suscrito a: {topic_name}')
            
        except Exception as e:
            #self.get_logger().error(f'Error suscribiéndose a {topic_name}: {str(e)}')
            pass
    
    def remove_subscription_for_topic(self, topic_name):
        """Elimina una suscripción para un topico específico"""
        if topic_name in self.status_subscriptions:
            del self.status_subscriptions[topic_name]
            if topic_name in self.status_data:
                del self.status_data[topic_name]
            
            #self.get_logger().info(f'Desuscrito de: {topic_name}')
    
    def status_callback(self, msg, topic_name):
        """Callback para procesar mensajes de status"""
        with self.lock:
            self.status_data[topic_name] = msg
            self.evaluate_global_status()
    
    def start_blinking(self):
        """Inicia el timer de parpadeo para estado WARNING"""
        if self.blink_timer is None:
            self.blink_timer = self.create_timer(
                self.warning_blink_interval, 
                self.blink_callback
            )
            self.blink_state = False  # Empezar apagado
            #self.get_logger().info("Iniciando parpadeo para estado WARNING")
    
    def stop_blinking(self):
        """Detiene el timer de parpadeo"""
        if self.blink_timer is not None:
            self.blink_timer.destroy()
            self.blink_timer = None
            #self.get_logger().info("Deteniendo parpadeo")
    
    def blink_callback(self):
        """Callback para controlar el parpadeo en estado WARNING"""
        if self.current_global_state == DiagnosticStatus.WARN:
            self.blink_state = not self.blink_state
            safety_relay = Bool()
            safety_relay.data = self.blink_state
            self.light_safety_pub.publish(safety_relay)

    def evaluate_global_status(self):
        """Evalúa el estado global basado en todos los estados individuales"""
        if not self.status_data:
            return
        
        status_counts = defaultdict(int)
        components_with_data = 0
        
        for topic, status_msg in self.status_data.items():
            if status_msg is not None:
                components_with_data += 1
                status_level = status_msg.level
                status_counts[status_level] += 1
        
        # Si no hay datos suficientes, no publicar
        if components_with_data == 0:
            return
        
        # Determinar estado global
        global_status = DiagnosticStatus()
        global_status.name = "Estado GLobal del Sistema"
        
        safety_relay = Bool()
        previous_state = self.current_global_state

        # Lógica de prioridad simple
        if status_counts.get(DiagnosticStatus.ERROR, 0) > 0:
            safety_relay.data = False
            global_status.level = DiagnosticStatus.ERROR
            global_status.message = "ERROR"
            self.current_global_state = DiagnosticStatus.ERROR

        elif status_counts.get(DiagnosticStatus.WARN, 0) > 0:
            global_status.level = DiagnosticStatus.WARN
            global_status.message = "ADVERTENCIA"
            self.current_global_state = DiagnosticStatus.WARN

        elif status_counts.get(DiagnosticStatus.OK, 0) == components_with_data:
            safety_relay.data = True
            global_status.level = DiagnosticStatus.OK
            global_status.message = "OK"
            self.current_global_state = DiagnosticStatus.OK

        else:
            global_status.level = DiagnosticStatus.WARN
            global_status.message = "UNKNOWN"
            self.current_global_state = DiagnosticStatus.WARN
        
        # Publicar estado global (sin values adicionales)
        self.global_status_publisher.publish(global_status)
            
        # Controlar el parpadeo basado en cambios de estado
        if previous_state != self.current_global_state:
            if self.current_global_state == DiagnosticStatus.WARN:
                self.start_blinking()
            else:
                self.stop_blinking()
                # Publicar estado fijo para OK o ERROR
                self.light_safety_pub.publish(safety_relay)
        elif self.current_global_state != DiagnosticStatus.WARN:
            # Para estados no-WARNING, publicar estado fijo
            self.light_safety_pub.publish(safety_relay)

    def destroy_node(self):
        """Override para limpiar recursos"""
        self.stop_blinking()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    status_monitor = StatusMonitor()
    
    try:
        rclpy.spin(status_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            status_monitor.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown

if __name__ == '__main__':
    main()