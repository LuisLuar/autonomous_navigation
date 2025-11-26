import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8
import threading
import time

class ReleCombiner(Node):
    def __init__(self):
        super().__init__('rele_combiner')

        # Inicializar bits de control (6 relés)
        self.rele_bits = [False] * 6
        
        # Flag para controlar si hay cambios pendientes
        self.pending_changes = False

        # Crear suscriptores
        self.subscribers = [
            self.create_subscription(Bool, '/start/motor_left',   self.cb_left_motor, 10),
            self.create_subscription(Bool, '/start/motor_right',  self.cb_right_motor, 10),
            self.create_subscription(Bool, '/light/stop',         self.cb_stop, 10),
            self.create_subscription(Bool, '/light/left',    self.cb_turn_left, 10),
            self.create_subscription(Bool, '/light/right',   self.cb_turn_right, 10),
            self.create_subscription(Bool, '/light/safety',       self.cb_safety, 10)
        ]

        # Publicador para enviar UInt8 a la ESP32
        self.rele_pub = self.create_publisher(UInt8, '/rele_control', 10)

        # Timer para publicación periódica cada 500 ms - SIEMPRE publica
        self.timer = self.create_timer(0.5, self.timer_callback)  # 500 ms = 0.5 segundos
        
        # Lock para thread safety
        self.lock = threading.Lock()

        self.get_logger().info("Nodo 'rele_combiner' iniciado. Envío periódico cada 500 ms.")

    # Callbacks de cada tópico
    def cb_left_motor(self, msg):
        with self.lock:
            self.rele_bits[0] = msg.data
            self.pending_changes = True
        self.publish_combined_state()

    def cb_right_motor(self, msg):
        with self.lock:
            self.rele_bits[1] = msg.data
            self.pending_changes = True
        self.publish_combined_state()

    def cb_stop(self, msg):
        with self.lock:
            self.rele_bits[2] = msg.data
            self.pending_changes = True
        self.publish_combined_state()

    def cb_turn_left(self, msg):
        with self.lock:
            self.rele_bits[3] = msg.data
            self.pending_changes = True
        self.publish_combined_state()

    def cb_turn_right(self, msg):
        with self.lock:
            self.rele_bits[4] = msg.data
            self.pending_changes = True
        self.publish_combined_state()

    def cb_safety(self, msg):
        with self.lock:
            self.rele_bits[5] = msg.data
            self.pending_changes = True
        self.publish_combined_state()

    # Callback del timer - envía CADA 500 ms SIN IMPORTAR si hay cambios
    def timer_callback(self):
        with self.lock:
            self._publish_current_state()  # Siempre publica, sin verificar pending_changes

    # Combinar todos los bits en un entero UInt8 y publicar
    def publish_combined_state(self):
        with self.lock:
            self._publish_current_state()
            self.pending_changes = False

    # Método interno para publicar el estado actual
    def _publish_current_state(self):
        combined_value = 0
        for i, bit in enumerate(self.rele_bits):
            if bit:
                combined_value |= (1 << i)  # activar el bit correspondiente

        msg = UInt8()
        msg.data = combined_value
        self.rele_pub.publish(msg)

        self.get_logger().debug(f"Estado combinado publicado: {bin(combined_value)}")

def main(args=None):
    rclpy.init(args=args)
    node = ReleCombiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown

if __name__ == '__main__':
    main()