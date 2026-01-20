#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import os
import glob
import sys
import numpy as np
from std_msgs.msg import Header
import struct

class DualImagePublisher(Node):
    def __init__(self):
        super().__init__('dual_image_publisher')
        
        # ===== PARÁMETROS HARCODEADOS (ajusta según necesites) =====
        self.folder_path = "/home/raynel/autonomous_navigation/src/saves/data_logs/ruta1_20260120_093624/perception"
        self.current_idx = 0
        self.frame_rate = 20.0  # Hz
        self.auto_advance = False  # Avanzar automáticamente?
        
        print("\n" + "="*70)
        print("📸 PUBLICADOR DUAL DE IMÁGENES (RGB + PROFUNDIDAD + POINT CLOUD)")
        print("="*70)
        
        # Verificar estructura de carpetas
        self.rgb_folder = os.path.join(self.folder_path, "images")
        self.depth_folder = os.path.join(self.folder_path, "depth")
        
        if not os.path.exists(self.rgb_folder):
            print(f"❌ ERROR: Carpeta RGB no existe: {self.rgb_folder}")
            sys.exit(1)
        
        self.has_depth = os.path.exists(self.depth_folder)
        
        # Cargar imágenes RGB
        self.rgb_images = []
        for ext in ['*.jpg', '*.jpeg', '*.png', '*.bmp']:
            self.rgb_images.extend(glob.glob(os.path.join(self.rgb_folder, ext)))
        
        # Cargar imágenes de profundidad (si existen)
        self.depth_images = []
        self.num_images = len(self.rgb_images)
        
        if self.has_depth:
            for ext in ['*.png', '*.tiff', '*.tif', '*.exr']:  # Agregado .exr para profundidad
                self.depth_images.extend(glob.glob(os.path.join(self.depth_folder, ext)))
            
            # Ordenar por nombre
            def extract_number(filename):
                import re
                nums = re.findall(r'\d+', os.path.basename(filename))
                return int(nums[0]) if nums else 0
            
            self.rgb_images.sort(key=lambda x: extract_number(x))
            self.depth_images.sort(key=lambda x: extract_number(x))
            
            if len(self.depth_images) == 0:
                print("⚠️  ADVERTENCIA: Carpeta depth existe pero no contiene imágenes válidas")
                self.has_depth = False
            else:
                print(f"✅ Imágenes Depth encontradas: {len(self.depth_images)}")
                
                if len(self.rgb_images) != len(self.depth_images):
                    print(f"⚠️  ADVERTENCIA: Número diferente de imágenes RGB ({len(self.rgb_images)}) vs Depth ({len(self.depth_images)})")
                    print("  Se ajustará al mínimo común")
                    self.num_images = min(len(self.rgb_images), len(self.depth_images))
        else:
            print("⚠️  ADVERTENCIA: No se encontró carpeta depth")
            print("  Solo se publicarán imágenes RGB")
        
        # Verificar que tenemos imágenes RGB
        if len(self.rgb_images) == 0:
            print("❌ ERROR: No se encontraron imágenes RGB")
            sys.exit(1)
        
        print(f"📁 Carpeta base: {self.folder_path}")
        print(f"✅ Imágenes RGB encontradas: {len(self.rgb_images)}")
        print(f"✅ Modo con profundidad: {'SI' if self.has_depth else 'NO'}")
        
        # Inicializar
        self.bridge = CvBridge()
        
        # Publicadores para RGB (SIEMPRE se crean)
        self.rgb_image_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)
        
        # Publicadores para Depth (solo si hay imágenes de profundidad)
        self.depth_image_pub = None
        self.depth_info_pub = None
        self.pointcloud_pub = None
        
        if self.has_depth:
            self.depth_image_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
            self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
            self.pointcloud_pub = self.create_publisher(PointCloud2, '/camera/depth_registered/points', 10)
            print("✅ Publicadores de profundidad y point cloud creados")
        else:
            print("ℹ️  Solo se crearán publicadores para imágenes RGB")
        
        # Camera info para RGB (tus parámetros originales)
        self.rgb_camera_info = CameraInfo()
        self.rgb_camera_info.header.frame_id = "camera_rgb_optical_frame"
        fx, fy, cx, cy = 574.1, 574.1, 320.0, 240.0
        self.rgb_camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.rgb_camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.rgb_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.rgb_camera_info.distortion_model = "plumb_bob"
        self.rgb_camera_info.height = 480
        self.rgb_camera_info.width = 640
        
        # Camera info para Depth (solo si hay profundidad)
        self.depth_camera_info = None
        if self.has_depth:
            self.depth_camera_info = CameraInfo()
            self.depth_camera_info.header.frame_id = "camera_depth_optical_frame"
            # Parámetros típicos para cámaras de profundidad
            fx_d, fy_d, cx_d, cy_d = 525.0, 525.0, 319.5, 239.5
            self.depth_camera_info.k = [fx_d, 0.0, cx_d, 0.0, fy_d, cy_d, 0.0, 0.0, 1.0]
            self.depth_camera_info.p = [fx_d, 0.0, cx_d, 0.0, 0.0, fy_d, cy_d, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.depth_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.depth_camera_info.distortion_model = "plumb_bob"
            self.depth_camera_info.height = 480
            self.depth_camera_info.width = 640
        
        # Parámetros para generación de nube de puntos (solo si hay profundidad)
        self.downsample_factor = 2 if self.has_depth else 1
        self.min_depth = 0.3 if self.has_depth else 0.0
        self.max_depth = 10.0 if self.has_depth else 0.0
        
        # Timer para publicación
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Mostrar controles
        self.print_controls()
        
        print(f"🚀 Listo! Publicando imagen {self.current_idx+1}/{self.num_images}")
        print("="*70 + "\n")
    
    def print_controls(self):
        print("\n🎮 CONTROLES (presiona en la ventana de imagen):")
        print("  → (flecha derecha) o 'd': Siguiente imagen")
        print("  ← (flecha izquierda) o 'a': Imagen anterior")
        print("  SPACE: Pausa/continuar auto-avance")
        print("  r: Reiniciar desde imagen 0")
        if self.has_depth:
            print("  p: Activar/desactivar publicación de nube de puntos")
            print("  +/-: Aumentar/disminuir rango de profundidad")
        print("  f/s: Aumentar/disminuir frame rate")
        print("  q o ESC: Salir")
        print("-" * 50)
    
    def timer_callback(self):
        """Publica las imágenes actuales y maneja entrada de teclado"""
        if self.current_idx >= self.num_images:
            self.current_idx = 0
        
        try:
            # Obtener ruta de imagen RGB (SIEMPRE existe)
            rgb_path = self.rgb_images[self.current_idx]
            
            # Leer imagen RGB
            rgb_img = cv2.imread(rgb_path)
            if rgb_img is None:
                self.get_logger().warn(f"No se pudo leer RGB: {rgb_path}")
                return
            
            # Crear timestamp actual
            current_time = self.get_clock().now().to_msg()
            
            # PUBLICAR IMAGEN RGB (SIEMPRE se publica)
            ros_rgb_img = self.bridge.cv2_to_imgmsg(rgb_img, "bgr8")
            ros_rgb_img.header.stamp = current_time
            ros_rgb_img.header.frame_id = "camera_rgb_optical_frame"
            self.rgb_image_pub.publish(ros_rgb_img)
            
            # Publicar camera info RGB
            self.rgb_camera_info.header.stamp = current_time
            self.rgb_info_pub.publish(self.rgb_camera_info)
            
            # Manejar imagen de profundidad (solo si existe)
            depth_img = None
            if self.has_depth and self.current_idx < len(self.depth_images):
                depth_path = self.depth_images[self.current_idx]
                
                # Leer imagen de profundidad
                depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
                if depth_img is not None:
                    # Verificar que depth_img es uint16 (16-bit)
                    if depth_img.dtype != np.uint16:
                        print(f"⚠️  Imagen de profundidad no es 16-bit: {depth_path}")
                        # Convertir si es necesario
                        if depth_img.dtype == np.uint8:
                            depth_img = depth_img.astype(np.uint16) * 256
                        elif depth_img.dtype == np.float32:
                            depth_img = (depth_img * 1000).astype(np.uint16)  # metros a mm
                        elif depth_img.dtype == np.float64:
                            depth_img = (depth_img * 1000).astype(np.uint16)  # metros a mm
                    
                    # PUBLICAR IMAGEN DE PROFUNDIDAD
                    # Para profundidad usamos encoding '16UC1' (16-bit unsigned, 1 channel)
                    ros_depth_img = self.bridge.cv2_to_imgmsg(depth_img, "16UC1")
                    ros_depth_img.header.stamp = current_time
                    ros_depth_img.header.frame_id = "camera_depth_optical_frame"
                    self.depth_image_pub.publish(ros_depth_img)
                    
                    # Publicar camera info Depth
                    self.depth_camera_info.header.stamp = current_time
                    self.depth_info_pub.publish(self.depth_camera_info)
                    
                    # PUBLICAR NUBE DE PUNTOS (si hay profundidad)
                    pointcloud_msg = self.depth_to_pointcloud(depth_img, rgb_img, current_time)
                    if pointcloud_msg is not None and self.pointcloud_pub:
                        self.pointcloud_pub.publish(pointcloud_msg)
                        print(f"📊 Publicada nube de puntos con {pointcloud_msg.width} puntos", end='\r')
                else:
                    self.get_logger().warn(f"No se pudo leer Depth: {depth_path}")
            
            # Mostrar en ventana (con información)
            self.display_images(rgb_img, depth_img, rgb_path, self.current_idx)
            
            # Manejar teclas (desde la ventana OpenCV)
            key = cv2.waitKey(30) & 0xFF  # 30ms timeout
            
            self.handle_keyboard_input(key)
            
            # Auto-avance si está activado
            if self.auto_advance:
                if self.current_idx < self.num_images - 1:
                    self.current_idx += 1
                else:
                    self.current_idx = 0
                    print("🔄 Volviendo al inicio...")
        
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            import traceback
            traceback.print_exc()
    
    def depth_to_pointcloud(self, depth_img, rgb_img, timestamp):
        """
        Convierte imagen de profundidad a nube de puntos PointCloud2
        Solo se llama si hay imágenes de profundidad
        """
        if depth_img is None:
            return None
            
        try:
            # Obtener dimensiones
            height, width = depth_img.shape
            
            # Submuestrear para reducir número de puntos
            step = self.downsample_factor
            height_sub = height // step
            width_sub = width // step
            
            # Crear grids de coordenadas
            u = np.arange(0, width, step).astype(np.float32)
            v = np.arange(0, height, step).astype(np.float32)
            u_grid, v_grid = np.meshgrid(u, v)
            
            # Obtener valores de profundidad submuestreados (convertir mm a metros)
            depth_sub = depth_img[v_grid.astype(int), u_grid.astype(int)].astype(np.float32) / 1000.0
            
            # Crear máscara para valores de profundidad válidos
            valid_mask = (depth_sub > self.min_depth) & (depth_sub < self.max_depth) & (depth_sub != 0)
            
            if not np.any(valid_mask):
                return None
            
            # Aplicar máscara
            u_valid = u_grid[valid_mask]
            v_valid = v_grid[valid_mask]
            z_valid = depth_sub[valid_mask]
            
            # Parámetros intrínsecos de la cámara de profundidad
            fx = self.depth_camera_info.k[0]
            fy = self.depth_camera_info.k[4]
            cx = self.depth_camera_info.k[2]
            cy = self.depth_camera_info.k[5]
            
            # Proyectar a coordenadas 3D
            x_valid = (u_valid - cx) * z_valid / fx
            y_valid = (v_valid - cy) * z_valid / fy
            
            # Obtener colores RGB correspondientes (si existe imagen RGB)
            colors = []
            if rgb_img is not None:
                # Asegurar que las imágenes tengan las mismas dimensiones
                rgb_resized = cv2.resize(rgb_img, (width, height))
                
                # Obtener colores en posiciones válidas
                for u_coord, v_coord in zip(u_valid.astype(int), v_valid.astype(int)):
                    if 0 <= v_coord < height and 0 <= u_coord < width:
                        # OpenCV usa BGR, ROS usa RGB
                        b, g, r = rgb_resized[int(v_coord), int(u_coord)]
                        colors.append([r, g, b])
                    else:
                        colors.append([255, 255, 255])  # Blanco si fuera de límites
                
                colors = np.array(colors, dtype=np.uint8)
            
            # Crear mensaje PointCloud2
            msg = PointCloud2()
            msg.header = Header()
            msg.header.stamp = timestamp
            msg.header.frame_id = "camera_depth_optical_frame"
            
            # Definir campos (x, y, z, rgb)
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
            ]
            
            msg.height = 1  # Nube de puntos no organizada
            msg.width = len(x_valid)
            
            # Calcular tamaño de paso y datos
            msg.point_step = 16  # 4 bytes por float * 3 + 4 bytes para rgb
            msg.row_step = msg.point_step * msg.width
            msg.is_bigendian = False
            msg.is_dense = False  # Puede tener puntos inválidos
            
            # Crear datos estructurados
            data = bytearray(msg.row_step)
            
            for i in range(msg.width):
                # Posición en el buffer
                offset = i * msg.point_step
                
                # Escribir coordenadas x, y, z
                struct.pack_into('fff', data, offset, 
                               float(x_valid[i]), 
                               float(y_valid[i]), 
                               float(z_valid[i]))
                
                # Escribir color RGB (formato float packed)
                if len(colors) > i:
                    r, g, b = colors[i]
                else:
                    r, g, b = 255, 255, 255
                
                # Pack RGB en float32
                rgb_packed = struct.unpack('f', struct.pack('I', 
                    (r << 16) | (g << 8) | b))[0]
                
                struct.pack_into('f', data, offset + 12, rgb_packed)
            
            msg.data = bytes(data)
            
            return msg
            
        except Exception as e:
            self.get_logger().error(f"Error generando nube de puntos: {e}")
            return None
    
    def display_images(self, rgb_img, depth_img, rgb_path, current_idx):
        """Muestra las imágenes en ventanas separadas o combinadas"""
        # Redimensionar para visualización
        display_rgb = cv2.resize(rgb_img, (640, 480))
        
        # Obtener nombre de archivo
        rgb_filename = os.path.basename(rgb_path)
        
        # Añadir texto informativo a RGB
        cv2.putText(display_rgb, f"RGB: {current_idx+1}/{self.num_images}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_rgb, f"Archivo: {rgb_filename}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
        cv2.putText(display_rgb, f"FPS: {self.frame_rate:.1f} | Auto: {'ON' if self.auto_advance else 'OFF'}", 
                   (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(display_rgb, f"Depth: {'HABILITADO' if self.has_depth else 'NO DISPONIBLE'}", 
                   (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                   (0, 255, 0) if self.has_depth else (0, 0, 255), 1)
        cv2.putText(display_rgb, "Flechas: navegar | SPACE: auto | q: salir", 
                   (10, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # Mostrar ventana RGB (siempre)
        cv2.imshow("RGB Image (Topic: /camera/rgb/image_raw)", display_rgb)
        
        # Mostrar ventana de profundidad solo si existe
        if depth_img is not None and self.has_depth:
            depth_vis_resized = self.create_depth_display(depth_img, current_idx)
            cv2.imshow("Depth Image (Topic: /camera/depth/image_raw)", depth_vis_resized)
        elif self.has_depth:
            # Crear ventana negra si hay profundidad pero no se pudo cargar la imagen
            black_img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(black_img, "Depth no disponible", 
                       (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imshow("Depth Image (Topic: /camera/depth/image_raw)", black_img)
    
    def create_depth_display(self, depth_img, current_idx):
        """Crea una visualización en color del mapa de profundidad"""
        # Redimensionar
        depth_vis_resized = cv2.resize(depth_img, (640, 480))
        
        # Crear visualización en color
        depth_colored = self.create_depth_visualization(depth_img)
        depth_colored_resized = cv2.resize(depth_colored, (640, 480))
        
        # Añadir texto informativo
        if current_idx < len(self.depth_images):
            depth_filename = os.path.basename(self.depth_images[current_idx])
            cv2.putText(depth_colored_resized, f"DEPTH: {current_idx+1}/{len(self.depth_images)}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(depth_colored_resized, f"Archivo: {depth_filename}", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
        
        # Calcular estadísticas de profundidad
        depth_valid = depth_img[depth_img > 0]
        if len(depth_valid) > 0:
            # Convertir a metros
            depth_meters = depth_valid.astype(np.float32) / 1000.0
            avg_depth = np.mean(depth_meters)
            min_depth = np.min(depth_meters)
            max_depth = np.max(depth_meters)
            
            cv2.putText(depth_colored_resized, f"Avg: {avg_depth:.2f}m | Min: {min_depth:.2f}m | Max: {max_depth:.2f}m", 
                       (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(depth_colored_resized, f"Puntos validos: {len(depth_valid)}/{depth_img.size}", 
                       (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            cv2.putText(depth_colored_resized, f"Submuestreo: {self.downsample_factor}x", 
                       (10, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 255), 1)
        else:
            cv2.putText(depth_colored_resized, "Sin datos de profundidad", 
                       (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        return depth_colored_resized
    
    def create_depth_visualization(self, depth_img):
        """Crea una visualización en color del mapa de profundidad"""
        # Crear copia para visualización
        depth_vis = depth_img.copy().astype(np.float32)
        
        # Normalizar para visualización
        depth_valid = depth_vis[depth_vis > 0]
        if len(depth_valid) > 0:
            # Escalar a rango 0-255
            if np.max(depth_valid) > 1000:  # Si está en milímetros, convertir a metros para visualización
                depth_vis[depth_vis > 0] = depth_vis[depth_vis > 0] / 1000.0 * 255 / 10.0  # Escalar a 0-10 metros
            else:
                # Si ya está en metros
                depth_vis[depth_vis > 0] = depth_vis[depth_vis > 0] * 255 / 10.0
            
            # Normalizar y convertir a uint8
            depth_normalized = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX)
            depth_normalized = np.nan_to_num(depth_normalized, nan=0.0)
            depth_normalized = depth_normalized.astype(np.uint8)
            
            # Aplicar colormap (viridis es bueno para profundidad)
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_VIRIDIS)
            
            # Marcar píxeles sin datos (valor 0) en negro
            depth_colored[depth_img == 0] = [0, 0, 0]
            
            return depth_colored
        else:
            # Si no hay datos de profundidad válidos, crear imagen negra
            return np.zeros((depth_img.shape[0], depth_img.shape[1], 3), dtype=np.uint8)
    
    def handle_keyboard_input(self, key):
        """Maneja la entrada de teclado"""
        if key == ord('q') or key == 27:  # 'q' o ESC
            print("\n👋 Saliendo...")
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return
        
        elif key == ord('d') or key == 83 or key == 32:  # 'd', flecha derecha o space
            if self.current_idx < self.num_images - 1:
                self.current_idx += 1
                print(f"→ Imagen {self.current_idx+1}/{self.num_images}")
        
        elif key == ord('a') or key == 81:  # 'a' o flecha izquierda
            if self.current_idx > 0:
                self.current_idx -= 1
                print(f"← Imagen {self.current_idx+1}/{self.num_images}")
        
        elif key == ord(' '):  # SPACE - toggle auto advance
            self.auto_advance = not self.auto_advance
            print(f"Auto-avance: {'ACTIVADO' if self.auto_advance else 'DESACTIVADO'}")
        
        elif key == ord('r'):  # 'r' - reiniciar
            self.current_idx = 0
            print(f"🔄 Reiniciado a imagen 1/{self.num_images}")
        
        elif key == ord('f'):  # 'f' - más rápido
            self.frame_rate = min(self.frame_rate + 1.0, 30.0)
            self.timer.cancel()
            self.timer = self.create_timer(1.0/self.frame_rate, self.timer_callback)
            print(f"⚡ Frame rate aumentado a {self.frame_rate:.1f} Hz")
        
        elif key == ord('s'):  # 's' - más lento
            self.frame_rate = max(self.frame_rate - 1.0, 0.5)
            self.timer.cancel()
            self.timer = self.create_timer(1.0/self.frame_rate, self.timer_callback)
            print(f"🐌 Frame rate reducido a {self.frame_rate:.1f} Hz")
        
        elif key == ord('p') and self.has_depth:  # 'p' - cambiar submuestreo de nube de puntos
            if self.downsample_factor == 1:
                self.downsample_factor = 2
            elif self.downsample_factor == 2:
                self.downsample_factor = 4
            elif self.downsample_factor == 4:
                self.downsample_factor = 1
            print(f"📊 Submuestreo de nube de puntos: {self.downsample_factor}x")
        
        elif key == ord('+') and self.has_depth:  # '+' - aumentar rango máximo de profundidad
            self.max_depth = min(self.max_depth + 1.0, 20.0)
            print(f"📏 Rango máximo de profundidad: {self.max_depth:.1f}m")
        
        elif key == ord('-') and self.has_depth:  # '-' - disminuir rango máximo de profundidad
            self.max_depth = max(self.max_depth - 1.0, 3.0)
            print(f"📏 Rango máximo de profundidad: {self.max_depth:.1f}m")
    
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    
    try:
        node = DualImagePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Interrupción por teclado")
    except Exception as e:
        print(f"💥 Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()