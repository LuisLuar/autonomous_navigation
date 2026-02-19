#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage  # CAMBIADO: de Image a CompressedImage
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import os
from datetime import datetime
import threading
import select
import sys
import termios
import tty
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        
        # Configuración - CAMBIA ESTA RUTA SEGÚN TUS NECESIDADES
        self.save_directory = "/home/raynel/Documents/calibrate_camera"  # ← CAMBIA ESTA RUTA
        
        # Crear directorio si no existe
        os.makedirs(self.save_directory, exist_ok=True)
        
        # Inicializar CV Bridge
        self.bridge = CvBridge()
        
        # Variables para almacenar la última imagen
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # QoS Best Effort para la imagen comprimida (como en tu código que funciona)
        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Subscriptores - CAMBIADO: ahora usa CompressedImage
        self.image_sub = self.create_subscription(
            CompressedImage,  # CAMBIADO
            '/image_raw/compressed',  # CAMBIADO
            self.image_callback,
            best_effort_qos  # QoS Best Effort
        )
        
        self.capture_sub = self.create_subscription(
            Bool,
            '/capture',
            self.capture_callback,
            10
        )
        
        # Contador de imágenes
        self.image_count = self.get_next_image_number()
        
        #self.get_logger().info("Nodo de captura de imágenes COMPRIMIDAS inicializado")
        #self.get_logger().info("Esperando imágenes en /image_raw/compressed")
        #self.get_logger().info("Presiona ESPACIO para capturar imagen o envía true al topic /capture")
        
        # Iniciar thread para detección de teclas
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
    
    def get_next_image_number(self):
        """Obtiene el siguiente número disponible para nombrar imágenes"""
        try:
            # Buscar archivos que coincidan con el patrón image_XXXXX.jpg
            existing_files = [f for f in os.listdir(self.save_directory) 
                            if f.startswith('image_') and f.endswith('.jpg')]
            
            if not existing_files:
                return 1
            
            # Extraer números de los nombres de archivo
            numbers = []
            for filename in existing_files:
                try:
                    # Eliminar 'image_' y '.jpg', luego convertir a número
                    num_str = filename[6:-4]  # 'image_00001.jpg' -> '00001'
                    numbers.append(int(num_str))
                except ValueError:
                    continue
            
            if numbers:
                return max(numbers) + 1
            else:
                return 1
                
        except Exception as e:
            return 1
    
    def image_callback(self, msg):
        """Callback para recibir imágenes COMPRIMIDAS de la cámara"""
        try:
            # CAMBIADO: Decodificar imagen comprimida
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            with self.image_lock:
                self.latest_image = image
                
        except Exception as e:
            #self.get_logger().error(f"Error procesando imagen comprimida: {e}")
            pass
    
    def capture_callback(self, msg):
        """Callback para capturar imagen cuando llega mensaje Bool true"""
        if msg.data:
            #self.get_logger().info("Captura solicitada via topic /capture")
            self.capture_image()
    
    def capture_image(self):
        """Captura y guarda la imagen actual"""
        if self.latest_image is None:
            #self.get_logger().warn("No hay imagen disponible para capturar")
            return
        
        try:
            with self.image_lock:
                # Generar nombre único para la imagen
                filename = f"image_{self.image_count:05d}.jpg"
                filepath = os.path.join(self.save_directory, filename)
                
                # Guardar imagen
                cv2.imwrite(filepath, self.latest_image)
                
                # Información adicional
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                image_size = self.latest_image.shape
                
                #self.get_logger().info(f"Imagen guardada: {filename}")
                #self.get_logger().info(f"Ruta: {filepath}")
                #self.get_logger().info(f"Hora: {timestamp}")
                #self.get_logger().info(f"Tamaño: {image_size[1]}x{image_size[0]}")
                
                # Incrementar contador
                self.image_count += 1
                
        except Exception as e:
            #self.get_logger().error(f"Error guardando imagen: {e}")
            pass
    
    def keyboard_listener(self):
        """Escucha las teclas presionadas en segundo plano"""
        try:
            # Verificar si stdin es un terminal válido
            if not sys.stdin.isatty():
                return
            
            # Configurar terminal para lectura de teclas
            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setraw(sys.stdin.fileno())
                
                while rclpy.ok():
                    # Esperar por entrada con timeout
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        
                        if key == ' ':  # Tecla ESPACIO
                            #self.get_logger().info("Captura solicitada via teclado")
                            self.capture_image()
                        
                        elif key == 'q' or key == 'Q':  # Tecla Q para salir
                            #self.get_logger().info("Saliendo por comando de teclado")
                            self.destroy_node()
                            break
                            
            finally:
                # Restaurar configuración del terminal
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                
        except Exception as e:
            pass
    
    def change_save_directory(self, new_directory):
        """Cambia el directorio de guardado"""
        try:
            os.makedirs(new_directory, exist_ok=True)
            self.save_directory = new_directory
            self.image_count = self.get_next_image_number()
            #self.get_logger().info(f"Nuevo directorio de guardado: {new_directory}")
            return True
        except Exception as e:
            #self.get_logger().error(f"Error cambiando directorio: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = ImageCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()