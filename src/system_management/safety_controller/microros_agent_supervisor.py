#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
import subprocess

class MicroROSAgentSupervisor(Node):
    def __init__(self):
        super().__init__('supervisor_microros_agent')

        self.publisher_ = self.create_publisher(
            DiagnosticStatus,
            '/status/microros_agent',
            10
        )

        self.timer = self.create_timer(0.5, self.check_agent)  # 2 Hz
        self.get_logger().info('✅ MicroROS Agent Supervisor iniciado — monitoreando agente...')

    def check_agent(self):
        # Busca el proceso micro_ros_agent UDP4
        result = subprocess.run(
            ['pgrep', '-f', 'micro_ros_agent.*udp4'],
            stdout=subprocess.DEVNULL
        )

        msg = DiagnosticStatus()
        msg.name = "MICRO-ROS agent"

        if result.returncode == 0:
            msg.level = DiagnosticStatus.OK
            msg.message = "conectado"
            #self.get_logger().info("🟢 micro-ROS Agent: ONLINE")
        else:
            msg.level = DiagnosticStatus.ERROR
            msg.message = "DESCONECTADO"
            #self.get_logger().warn("🔴 micro-ROS Agent: OFFLINE")

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MicroROSAgentSupervisor()
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
