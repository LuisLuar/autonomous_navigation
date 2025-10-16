import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8

class ReleCombiner(Node):
    def __init__(self):
        super().__init__('rele_combiner')

        # Inicializar bits de control (6 relés)
        self.rele_bits = [False] * 6

        # Crear suscriptores
        self.subscribers = [
            self.create_subscription(Bool, '/motor_left',   self.cb_left_motor, 10),
            self.create_subscription(Bool, '/motor_right',  self.cb_right_motor, 10),
            self.create_subscription(Bool, '/stop',         self.cb_stop, 10),
            self.create_subscription(Bool, '/turn_left',    self.cb_turn_left, 10),
            self.create_subscription(Bool, '/turn_right',   self.cb_turn_right, 10),
            self.create_subscription(Bool, '/safety',       self.cb_safety, 10)
        ]

        # Publicador para enviar UInt8 a la ESP32
        self.rele_pub = self.create_publisher(UInt8, '/rele/control', 10)

        self.get_logger().info("Nodo 'rele_combiner' iniciado. Escuchando 6 entradas booleanas.")

    # Callbacks de cada tópico
    def cb_left_motor(self, msg):
        self.rele_bits[0] = msg.data
        self.publish_combined_state()

    def cb_right_motor(self, msg):
        self.rele_bits[1] = msg.data
        self.publish_combined_state()

    def cb_stop(self, msg):
        self.rele_bits[2] = msg.data
        self.publish_combined_state()

    def cb_turn_left(self, msg):
        self.rele_bits[3] = msg.data
        self.publish_combined_state()

    def cb_turn_right(self, msg):
        self.rele_bits[4] = msg.data
        self.publish_combined_state()

    def cb_safety(self, msg):
        self.rele_bits[5] = msg.data
        self.publish_combined_state()

    # Combinar todos los bits en un entero UInt8
    def publish_combined_state(self):
        combined_value = 0
        for i, bit in enumerate(self.rele_bits):
            if bit:
                combined_value |= (1 << i)  # activar el bit correspondiente

        msg = UInt8()
        msg.data = combined_value
        self.rele_pub.publish(msg)

        self.get_logger().info(f"Estado combinado publicado: {bin(combined_value)}")

def main(args=None):
    rclpy.init(args=args)
    node = ReleCombiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
