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

    def check_agent(self):
        # Verificar los 3 agentes
        udp_active = self.check_agent_process('micro_ros_agent.*udp4')
        control_active = self.check_agent_process('micro_ros_agent.*ttyESP32')
        safety_active = self.check_agent_process('micro_ros_agent.*ttySafety')

        msg = DiagnosticStatus()
        msg.name = "MICRO-ROS agent"

        # Construir mensaje descriptivo
        active_agents = []
        if udp_active:
            active_agents.append("WIFI-8888")
        if control_active:
            active_agents.append("SERIAL-CONTROL")
        if safety_active:
            active_agents.append("SERIAL-SAFETY")

        if active_agents:
            msg.level = DiagnosticStatus.OK
            if len(active_agents) == 1:
                msg.message = f"Conectado Agente: {active_agents[0]}"
            else:
                msg.message = f"Conectado {', '.join(active_agents)}"
        else:
            msg.level = DiagnosticStatus.ERROR
            msg.message = "DESCONECTADO"

        self.publisher_.publish(msg)

    def check_agent_process(self, pattern):
        """Verifica si un agente específico está ejecutándose"""
        result = subprocess.run(
            ['pgrep', '-f', pattern],
            stdout=subprocess.DEVNULL
        )
        return result.returncode == 0
    
    def destroy_node(self):
        super().destroy_node()


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