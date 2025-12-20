#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose
from builtin_interfaces.msg import Time
import numpy as np

class OdometryToPath(Node):
    def __init__(self):
        super().__init__('odometry_to_path_global')
        
        # Parámetros configurables
        self.declare_parameter('max_path_length', 1000)  # Máximo número de poses en el path
        self.declare_parameter('fixed_frame', 'map')     # Frame fijo para el path
        self.declare_parameter('path_topic', '/path/global')    # Tópico de salida para el path
        
        # Obtener parámetros
        self.max_path_length = self.get_parameter('max_path_length').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        path_topic = self.get_parameter('path_topic').value
        
        # Historial de poses
        self.path_history = []
        
        # Subscriptor a odometría
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odom_callback,
            10
        )
        
        # Publicador del path
        self.path_pub = self.create_publisher(
            Path,
            path_topic,
            10
        )
        
        # Timer para publicar el path periódicamente (opcional)
        self.timer = self.create_timer(0.1, self.publish_path)  # 10 Hz
        
        #self.get_logger().info(f'Nodo iniciado. Suscrito a /odometry/global, publicando en {path_topic}')
        #self.get_logger().info(f'Frame fijo: {self.fixed_frame}, longitud máxima del path: {self.max_path_length}')
    
    def odom_callback(self, msg):
        """Callback para procesar mensajes de odometría"""
        # Crear PoseStamped a partir de la odometría
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        # Agregar al historial
        self.path_history.append(pose_stamped)
        
        # Mantener longitud máxima
        if len(self.path_history) > self.max_path_length:
            self.path_history.pop(0)
        
        # Opcional: publicar inmediatamente cuando llega nueva odometría
        # self.publish_path()
    
    def publish_path(self):
        """Publicar el path completo"""
        if len(self.path_history) == 0:
            return
        
        # Crear mensaje Path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.fixed_frame
        
        # Asignar todas las poses del historial
        path_msg.poses = self.path_history
        
        # Publicar
        self.path_pub.publish(path_msg)
        
        # Log opcional (comentar para reducir ruido)
        # self.get_logger().debug(f'Path publicado con {len(self.path_history)} poses', throttle_duration_sec=1.0)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = OdometryToPath()
    
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