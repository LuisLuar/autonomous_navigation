#!/usr/bin/env python3
"""
camera_reconnect.py - Nodo ROS2 optimizado para reconoxión de camara ASUSXTION.

Características principales:
 - Reconexión automática del puerto serial si se desconecta la camara
 - Publicación de múltiples tópicos ROS2 con información de la camara completa
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import subprocess
import time
import signal
import psutil
import os

class CameraReconnectNode(Node):
    def __init__(self):
        super().__init__('camera_reconnect_node')
        
        # Parámetros configurables
        self.declare_parameter('timeout_seconds', 2.0)
        self.declare_parameter('monitor_topic', '/camera/rgb/image_raw')
        
        self.timeout = self.get_parameter('timeout_seconds').value
        self.monitor_topic = self.get_parameter('monitor_topic').value
        
        self.last_message_time = time.time()
        self.camera_process = None
        
        # Suscriptor para monitorear mensajes de la cámara
        self.subscription = self.create_subscription(
            Image,
            self.monitor_topic,
            self.image_callback,
            10
        )
        
        # Timer para verificar el estado cada segundo
        self.timer = self.create_timer(1.0, self.check_camera_status)
        
        # Iniciar la cámara por primera vez
        self.start_camera()
    
    # Se ejecuta cada vez que llega una imagen de la cámara
    def image_callback(self, msg):
        self.last_message_time = time.time()
    
    #Verifica si la cámara sigue enviando datos
    def check_camera_status(self):
        current_time = time.time()
        time_since_last_message = current_time - self.last_message_time
        
        if time_since_last_message > self.timeout:
            self.restart_camera()
            # Reset el timer para evitar múltiples reinicios seguidos
            self.last_message_time = time.time()
    
    #Inicia el proceso de la cámara usando ros2 launch
    def start_camera(self):
        try:           
            # Comando para lanzar la cámara
            cmd = [
                'ros2', 'launch', 
                'openni2_camera', 
                'camera_with_cloud.launch.py'
            ]
            
            self.camera_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Para poder matar el grupo de procesos completo
            )
            
            self.last_message_time = time.time()
            
        except Exception as e:
            #self.get_logger().error(f'Failed to start camera: {e}')
            pass
    
    #Reinicia la cámara (detiene y vuelve a iniciar)
    def restart_camera(self):
        self.stop_camera()
        time.sleep(2)  # Esperar a que se liberen los recursos USB
        self.start_camera()
    
    #Detiene el proceso de la cámara de forma segura
    def stop_camera(self):
        if self.camera_process:
            try:                
                # Obtener todos los procesos hijos
                parent = psutil.Process(self.camera_process.pid)
                children = parent.children(recursive=True)
                
                # Terminar primero los hijos
                for child in children:
                    try:
                        child.terminate()
                    except psutil.NoSuchProcess:
                        pass
                
                # Terminar el proceso padre
                self.camera_process.terminate()
                
                # Esperar a que terminen (máximo 5 segundos)
                gone, alive = psutil.wait_procs(children + [parent], timeout=5)
                
                # Si quedan procesos vivos, forzar el cierre
                for p in alive:
                    try:
                        p.kill()
                    except psutil.NoSuchProcess:
                        pass
                
            except Exception as e:
                # Último recurso: matar todo el grupo de procesos
                try:
                    import os
                    os.killpg(os.getpgid(self.camera_process.pid), signal.SIGTERM)
                except:
                    pass
            
            self.camera_process = None

def main(args=None):
    rclpy.init(args=args)
    node = CameraReconnectNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info('Shutting down...')
        pass
    finally:
        try:
            node.stop_camera()
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown

if __name__ == '__main__':
    main()