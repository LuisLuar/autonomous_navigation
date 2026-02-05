#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        
        # Publicador para el topic de la cámara
        self.publisher = self.create_publisher(Image, '/camera/rgb/image_raw', 1)
        self.bridge = CvBridge()
        
        # Configuración de la cámara
        self.camera_index = 3 # Usualmente 0 es la cámara integrada
        self.frame_width = 640
        self.frame_height = 480
        self.fps = 30  # Frames por segundo
        
        # Iniciar captura de video
        self.cap = cv2.VideoCapture("http://192.168.100.164:8080/video" )#cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            #self.get_logger().error(f"No se pudo abrir la cámara con índice {self.camera_index}")
            #self.get_logger().info("Intentando con índice 1...")
            self.cap = cv2.VideoCapture(1)
            
        if not self.cap.isOpened():
            #self.get_logger().error("No se pudo abrir ninguna cámara")
            raise Exception("No se pudo abrir la cámara")
        
        # Configurar resolución
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        #self.get_logger().info(f"Cámara iniciada: {self.frame_width}x{self.frame_height} @ {self.fps}FPS")
        
        # Timer para publicar frames
        self.timer = self.create_timer(1.0/self.fps, self.publish_frame)
        
        # Contador para debugging
        self.frame_count = 0
        self.start_time = time.time()
    
    def publish_frame(self):
        try:
            # Capturar frame
            ret, frame = self.cap.read()
            
            if not ret:
                #self.get_logger().warn("No se pudo capturar frame de la cámara")
                return
            
            # Verificar y corregir dimensiones si es necesario
            if frame.shape[1] != self.frame_width or frame.shape[0] != self.frame_height:
                frame = cv2.resize(frame, (self.frame_width, self.frame_height))
            
            # Convertir de BGR a RGB (algunas cámaras usan RGB, otras BGR)
            # La mayoría de cámaras OpenCV devuelven BGR
            # Para simular una cámara real, publicamos en BGR como haría una cámara real
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Crear mensaje ROS
            ros_image = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'
            
            # Publicar
            self.publisher.publish(ros_image)
            
            # Log cada 30 frames (aproximadamente 1 segundo a 30fps)
            """self.frame_count += 1
            if self.frame_count % 30 == 0:
                elapsed = time.time() - self.start_time
                actual_fps = self.frame_count / elapsed
                self.get_logger().info(f"Publicando frames a {actual_fps:.1f} FPS")"""
                
                # Opcional: mostrar preview en ventana (útil para debugging)
                # cv2.imshow('Webcam Preview', frame)
                # cv2.waitKey(1)
                
        except Exception as e:
            #self.get_logger().error(f"Error al publicar frame: {e}")
            pass
    
    def destroy_node(self):
        # Liberar recursos
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            if 'node' in locals():
                node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()