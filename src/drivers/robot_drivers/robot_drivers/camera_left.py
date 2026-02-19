#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np  # ¡IMPORTANTE: Agregar este import!

class CameraLeft(Node):
    def __init__(self):
        super().__init__('camera_left')
        
        # Configuración - SOLO cámara IP
        self.frame_width = 640
        self.frame_height = 480
        self.fps = 30
        self.reconnect_interval = 2.0  # segundos entre intentos
        self.max_reconnect_attempts = 100  # intentos ilimitados prácticamente
        self.heartbeat_interval = 5.0
        
        # SOLO fuente IP camera - especificar backend FFMPEG para streams HTTP
        self.camera_source = "/dev/video2"  #"http://10.42.0.17:8080/video" 
        
        # Variables de estado
        self.reconnect_attempts = 0
        self.last_frame_time = time.time()
        self.is_connected = False
        self.cap = None
        self.consecutive_failures = 0
        self.max_consecutive_failures = 5
        
        # Publicador
        self.publisher = self.create_publisher(Image, '/camera/rgb/left', 10)
        self.bridge = CvBridge()
        
        # Frame placeholder (crearlo una vez para reutilizar)
        self.placeholder_frame = self.create_placeholder_frame()
        
        # Iniciar conexión
        self.connect_camera()
        
        # Timer para publicar frames
        self.timer = self.create_timer(1.0/self.fps, self.publish_frame)
        
        # Timer para verificar conexión
        self.heartbeat_timer = self.create_timer(self.heartbeat_interval, self.check_connection)
        
        #self.get_logger().info("Cámara izquierda inicializada - Solo IP camera")
    
    def create_placeholder_frame(self):
        """Crear frame placeholder una sola vez"""
        placeholder = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
        
        # Agregar texto informativo
        cv2.putText(placeholder, "CAMERA DISCONNECTED", 
                   (50, self.frame_height//2 - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(placeholder, f"IP: 192.168.100.164:8080", 
                   (50, self.frame_height//2),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(placeholder, "Reconnecting...", 
                   (50, self.frame_height//2 + 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return placeholder
    
    def connect_camera(self):
        """Intenta conectar SOLO a la cámara IP"""
        #self.get_logger().info(f"Conectando a IP camera: {self.camera_source}")
        
        try:
            # Liberar cámara anterior si existe
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            
            # Intentar conectar usando backend FFMPEG para streams HTTP
            # Prueba diferentes métodos de conexión
            connection_methods = [
                # Método 1: Usando OpenCV con URL directa
                lambda: cv2.VideoCapture(self.camera_source),
                
                # Método 2: Forzando backend FFMPEG (si está disponible)
                lambda: cv2.VideoCapture(self.camera_source, cv2.CAP_FFMPEG),
                
                # Método 3: Intentar con MJPG stream (común en IP cameras)
                lambda: cv2.VideoCapture(self.camera_source + "?type=mjpg"),
                
                # Método 4: Intentar con parámetros de buffer reducido
                lambda: cv2.VideoCapture(self.camera_source + "?buffer_size=1&timeout=5000"),
            ]
            
            for i, connect_func in enumerate(connection_methods):
                try:
                    self.cap = connect_func()
                    
                    # Configurar timeout y buffers
                    if isinstance(self.camera_source, str) and 'http' in self.camera_source:
                        # Ajustar parámetros para streams HTTP
                        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        # Timeout más corto para detección rápida de fallos
                        
                    # Dar tiempo para estabilizar
                    time.sleep(0.5)
                    
                    # Verificar conexión con intento de lectura
                    if self.cap.isOpened():
                        # Intentar leer varios frames para asegurar conexión estable
                        successful_reads = 0
                        for _ in range(3):
                            ret, frame = self.cap.read()
                            if ret and frame is not None and frame.size > 0:
                                successful_reads += 1
                            time.sleep(0.1)
                        
                        if successful_reads >= 1:
                            # Configurar propiedades
                            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                            
                            self.is_connected = True
                            self.reconnect_attempts = 0
                            self.consecutive_failures = 0
                            self.last_frame_time = time.time()
                            #self.get_logger().info(f"Conexión exitosa (método {i+1})")
                            return True
                        else:
                            self.cap.release()
                            self.cap = None
                            
                except Exception as e:
                    #self.get_logger().debug(f"Método {i+1} falló: {str(e)}")
                    if self.cap is not None:
                        self.cap.release()
                        self.cap = None
            
            # Si llegamos aquí, todos los métodos fallaron
            self.is_connected = False
            self.reconnect_attempts += 1
            
            
            return False
            
        except Exception as e:
            #self.get_logger().error(f"Error en connect_camera: {str(e)}")
            self.is_connected = False
            return False
    
    def check_connection(self):
        """Verifica periódicamente la conexión"""
        if not self.is_connected:
            #self.get_logger().debug("Intentando reconexión a IP camera...")
            self.connect_camera()
        else:
            # Verificar si ha pasado mucho tiempo sin frames válidos
            time_since_last_frame = time.time() - self.last_frame_time
            if time_since_last_frame > (self.heartbeat_interval * 2):
                self.is_connected = False
                self.connect_camera()
    
    def publish_frame(self):
        try:
            if not self.is_connected or self.cap is None:
                # Publicar frame placeholder
                self.publish_placeholder()
                return
            
            # Capturar frame
            ret, frame = self.cap.read()
            
            if not ret or frame is None or frame.size == 0:
                # Frame inválido
                self.consecutive_failures += 1
                
                if self.consecutive_failures >= self.max_consecutive_failures:
                    self.is_connected = False
                    self.connect_camera()
                
                # Publicar placeholder mientras se reconecta
                self.publish_placeholder()
                return
            
            # Resetear contadores de éxito
            self.consecutive_failures = 0
            self.last_frame_time = time.time()
            
            # Verificar y corregir dimensiones
            if frame.shape[1] != self.frame_width or frame.shape[0] != self.frame_height:
                frame = cv2.resize(frame, (self.frame_width, self.frame_height))
            
            # Convertir de BGR a RGB
            try:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            except:
                # Si ya está en RGB o hay error, usar tal cual
                frame_rgb = frame
            
            # Crear mensaje ROS
            ros_image = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_left'
            
            # Publicar
            self.publisher.publish(ros_image)
            
        except cv2.error as e:          
            self.is_connected = False
            self.publish_placeholder()
            
        except Exception as e:
            #self.get_logger().error(f"Error inesperado: {str(e)}", exc_info=True)
            self.publish_placeholder()
    
    def publish_placeholder(self):
        """Publica frame placeholder usando el frame pre-creado"""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(self.placeholder_frame, encoding='rgb8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_left'
            self.publisher.publish(ros_image)
            
        except Exception as e:
            # Error mínimo - no llenar logs con esto
            if hasattr(self, 'last_placeholder_error') and \
               (time.time() - self.last_placeholder_error > 10):
                #self.get_logger().error(f"Error publicando placeholder: {str(e)}")
                self.last_placeholder_error = time.time()
    
    def destroy_node(self):
        """Liberar recursos"""
        #self.get_logger().info("Apagando cámara izquierda...")
        
        if self.cap is not None:
            try:
                self.cap.release()
            except:
                pass
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraLeft()

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