#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import math
import json
import os

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        
        # ===== PARÁMETROS =====
        self.declare_parameter("camera_height", 1.2)  # m (altura de la cámara)
        self.declare_parameter("camera_x", -0.32)     # m (posición X respecto al centro del robot)
        self.declare_parameter("camera_y", 0.0)       # m (posición Y respecto al centro del robot)
        self.declare_parameter("chessboard_calibration", "/home/raynel/Documents/calibrate_camera/calibration.json")
        self.declare_parameter("save_path", "/home/raynel/autonomous_navigation/src/perception_stack/params/camera_calibration.json")
        
        self.h = self.get_parameter("camera_height").value
        self.camera_x = self.get_parameter("camera_x").value
        self.camera_y = self.get_parameter("camera_y").value
        self.chessboard_calibration_path = self.get_parameter("chessboard_calibration").value
        self.save_path = self.get_parameter("save_path").value
        
        # ===== VARIABLES DE CALIBRACIÓN =====
        self.calibration_points = []  # Lista de puntos: {'Z': distancia, 'v': fila, 'u': columna}
        self.current_point = None
        self.calibration_complete = False
        
        # ===== INTRINSECOS (se cargarán del JSON) =====
        self.fx = self.fy = self.cx = self.cy = None
        self.distortion_coeffs = None
        self.ready = False
        
        # ===== ROS =====
        self.bridge = CvBridge()
        
        # QoS Best Effort para imagen comprimida (igual que en el visualizador)
        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",  # Tópico de imagen comprimida
            self.image_callback,
            best_effort_qos  # Usar QoS Best Effort
        )
        
        # Cargar calibración del tablero de ajedrez
        self.load_chessboard_calibration()
        
        self.get_logger().info("🎯 NODO DE CALIBRACIÓN VISUAL INICIADO")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"📏 Altura cámara: {self.h}m")
        self.get_logger().info(f"📍 Posición cámara: X={self.camera_x}m, Y={self.camera_y}m")
        self.get_logger().info(f"📷 Calibración tablero: {self.chessboard_calibration_path}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("INSTRUCCIONES:")
        self.get_logger().info("1. Coloca objetos/marcas a distancias CONOCIDAS")
        self.get_logger().info("2. Presiona una tecla numérica (1-9) para distancia")
        self.get_logger().info("3. Haz CLIC en la imagen donde está el objeto")
        self.get_logger().info("4. Presiona 'c' para calcular")
        self.get_logger().info("5. Presiona 's' para guardar")
        self.get_logger().info("6. Presiona 'r' para reiniciar")
        self.get_logger().info("=" * 60)
        
        # Distancias predefinidas (metros)
        self.distance_map = {
            ord('1'): 1.0,
            ord('2'): 2.0,
            ord('3'): 3.0,
            ord('4'): 4.0,
            ord('5'): 5.0,
            ord('6'): 6.0,
            ord('7'): 7.0,
            ord('8'): 8.0,
            ord('9'): 9.0,
        }
        
        self.current_distance = 2.0  # Distancia por defecto
        
        # Para callback de mouse
        self.image_display = None
        cv2.namedWindow("Calibration - Click points")
        cv2.setMouseCallback("Calibration - Click points", self.mouse_callback)

    def load_chessboard_calibration(self):
        """Carga la calibración del tablero de ajedrez"""
        try:
            if not os.path.exists(self.chessboard_calibration_path):
                self.get_logger().error(f"❌ No se encuentra el archivo: {self.chessboard_calibration_path}")
                return
            
            with open(self.chessboard_calibration_path, 'r') as f:
                data = json.load(f)
            
            # Extraer matriz de cámara
            camera_matrix = data['camera_matrix']
            self.fx = camera_matrix[0][0]
            self.fy = camera_matrix[1][1]
            self.cx = camera_matrix[0][2]
            self.cy = camera_matrix[1][2]
            
            # Extraer coeficientes de distorsión
            dist_coeffs = data['distortion_coefficients'][0]
            self.distortion_coeffs = {
                'k1': dist_coeffs[0],
                'k2': dist_coeffs[1],
                'p1': dist_coeffs[2],
                'p2': dist_coeffs[3],
                'k3': dist_coeffs[4]
            }
            
            # Información adicional
            self.chessboard_info = {
                'date': data.get('date', ''),
                'chessboard_size': data.get('chessboard_size', []),
                'square_size_mm': data.get('square_size_mm', 0),
                'frame_size': data.get('frame_size', []),
                'images_used': data.get('images_used', 0),
                'reprojection_error': data.get('reprojection_error', 0)
            }
            
            self.ready = True
            
            self.get_logger().info("✅ Calibración del tablero cargada exitosamente")
            self.get_logger().info(f"   - Error de reproyección: {self.chessboard_info['reprojection_error']:.4f} px")
            self.get_logger().info(f"   - fx={self.fx:.2f}, fy={self.fy:.2f}")
            self.get_logger().info(f"   - cx={self.cx:.2f}, cy={self.cy:.2f}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error cargando calibración: {e}")

    def image_callback(self, msg):
        """Recibe imagen comprimida y la muestra para calibración"""
        try:
            # Decodificar imagen comprimida
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().warn("⚠️ No se pudo decodificar la imagen")
                return
                
            self.image_display = cv_image.copy()
            H, W = cv_image.shape[:2]
            
            # Verificar que los intrínsecos sean consistentes con el tamaño de imagen
            if self.ready and (self.cx != W/2 or self.cy != H/2):
                self.get_logger().debug(f"Centro óptico ajustado: ({self.cx:.1f}, {self.cy:.1f}) para imagen {W}x{H}")
            
            # Dibujar información
            cv2.putText(self.image_display, f"ALTURA CAMARA: {self.h}m", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(self.image_display, f"POSICION: X={self.camera_x}m, Y={self.camera_y}m", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(self.image_display, f"DISTANCIA ACTUAL: {self.current_distance}m (Teclas 1-9)", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(self.image_display, f"PUNTOS MARCADOS: {len(self.calibration_points)}", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Dibujar puntos de calibración
            for i, point in enumerate(self.calibration_points):
                u, v = int(point['u']), int(point['v'])
                Z = point['Z']
                
                # Dibujar círculo
                color = (0, 0, 255) if Z < 2.0 else (0, 255, 0) if Z < 4.0 else (255, 0, 255)
                cv2.circle(self.image_display, (u, v), 8, color, -1)
                cv2.putText(self.image_display, f"{Z}m", (u+10, v-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Dibujar línea horizontal para referencia
                cv2.line(self.image_display, (0, v), (W, v), (100, 100, 100), 1)
            
            # Dibujar línea del horizonte si ya calculamos pitch
            if hasattr(self, 'calculated_pitch'):
                v_horizon = self.cy + self.fy * math.tan(self.calculated_pitch)
                if 0 <= v_horizon < H:
                    cv2.line(self.image_display, (0, int(v_horizon)), (W, int(v_horizon)), 
                            (0, 255, 255), 2)
                    cv2.putText(self.image_display, f"Horizonte (pitch={math.degrees(self.calculated_pitch):.1f}°)", 
                               (10, H-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Instrucciones en pantalla
            y_pos = H - 120
            instructions = [
                "CONTROLES:",
                "Teclas 1-9: Seleccionar distancia (1=1m, 2=1.5m, ...)",
                "CLIC: Marcar punto en la imagen",
                "c: Calcular pitch automáticamente",
                "s: Guardar calibración completa",
                "r: Reiniciar puntos",
                "ESC: Salir"
            ]
            
            for i, text in enumerate(instructions):
                cv2.putText(self.image_display, text, (10, y_pos + i*20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            
            # Mostrar imagen
            cv2.imshow("Calibration - Click points", self.image_display)
            
            # Manejar teclas
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27:  # ESC
                self.get_logger().info("Calibración terminada")
                cv2.destroyAllWindows()
                rclpy.shutdown()
            
            elif key in self.distance_map:
                self.current_distance = self.distance_map[key]
                self.get_logger().info(f"Distancia seleccionada: {self.current_distance}m")
            
            elif key == ord('c'):  # Calcular
                self.calculate_pitch()
            
            elif key == ord('s'):  # Guardar
                self.save_calibration()
            
            elif key == ord('r'):  # Reiniciar
                self.calibration_points = []
                self.get_logger().info("Puntos reiniciados")
            
            elif key == ord('p'):  # Mostrar pitch actual
                if hasattr(self, 'calculated_pitch'):
                    self.get_logger().info(f"Pitch actual: {self.calculated_pitch:.4f} rad ({math.degrees(self.calculated_pitch):.2f}°)")
                    
        except Exception as e:
            self.get_logger().error(f"Error en imagen: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        """Callback para clics del mouse"""
        if event == cv2.EVENT_LBUTTONDOWN and self.image_display is not None:
            # Agregar punto de calibración
            point = {
                'Z': self.current_distance,
                'v': y,  # Fila (coordenada Y en imagen)
                'u': x   # Columna (coordenada X en imagen)
            }
            
            self.calibration_points.append(point)
            self.get_logger().info(f"Punto agregado: ({x}, {y}) a {self.current_distance}m")
            
            # Redibujar
            cv2.circle(self.image_display, (x, y), 8, (0, 255, 0), -1)
            cv2.imshow("Calibration - Click points", self.image_display)

    def calculate_pitch(self):
        """Calcula el pitch óptimo a partir de los puntos marcados"""
        if len(self.calibration_points) < 2:
            self.get_logger().warn("Se necesitan al menos 2 puntos para calcular")
            return
        
        if not self.ready:
            self.get_logger().warn("Esperando intrínsecos de cámara")
            return
        
        pitches = []
        errors = []
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("CALCULANDO PITCH ÓPTIMO")
        self.get_logger().info("=" * 60)
        
        for point in self.calibration_points:
            Z = point['Z']
            v = point['v']
            
            # Fórmula: pitch = -atan(h/Z) + atan((v-cy)/fy)
            pitch = -math.atan2(self.h, Z) + math.atan2((v - self.cy), self.fy)
            pitches.append(pitch)
            
            self.get_logger().info(f"  Punto Z={Z:.1f}m, v={v}px -> pitch={pitch:.4f}rad ({math.degrees(pitch):.2f}°)")
        
        # Calcular pitch promedio
        self.calculated_pitch = np.mean(pitches)
        pitch_std = np.std(pitches)
        
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"PITCH CALCULADO: {self.calculated_pitch:.4f} rad")
        self.get_logger().info(f"PITCH CALCULADO: {math.degrees(self.calculated_pitch):.2f}°")
        self.get_logger().info(f"Desviación estándar: {pitch_std:.4f} rad ({math.degrees(pitch_std):.2f}°)")
        
        # Validación
        if pitch_std > 0.05:  # > ~3°
            self.get_logger().warn("ADVERTENCIA: Alta variación en puntos. Verificar mediciones.")
        
        # Mostrar en pantalla
        if self.image_display is not None:
            H, W = self.image_display.shape[:2]
            cv2.putText(self.image_display, f"PITCH CALCULADO: {math.degrees(self.calculated_pitch):.2f}°", 
                       (W-300, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        self.calibration_complete = True
        self.get_logger().info("✅ Calibración completada")

    def save_calibration(self):
        """Guarda la calibración completa en archivo JSON"""
        if not hasattr(self, 'calculated_pitch'):
            self.get_logger().warn("Primero calcula el pitch (presiona 'c')")
            return
        
        # Crear estructura del JSON final
        calibration_data = {
            'camera_height': self.h,
            'camera_x': self.camera_x,
            'camera_y': self.camera_y,
            'camera_pitch': float(self.calculated_pitch),
            'camera_pitch_degrees': float(math.degrees(self.calculated_pitch)),
            'intrinsics': {
                'fx': float(self.fx),
                'fy': float(self.fy),
                'cx': float(self.cx),
                'cy': float(self.cy)
            },
            'distortion': self.distortion_coeffs,  # Coeficientes del tablero
            'chessboard_calibration_info': self.chessboard_info,  # Info adicional del tablero
            'calibration_points': self.calibration_points,  # Puntos marcados para pitch
            'timestamp': self.get_clock().now().to_msg().sec
        }
        
        try:
            # Crear directorio si no existe
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            
            with open(self.save_path, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            self.get_logger().info(f"✅ Calibración guardada en: {self.save_path}")
            
            # Mostrar resumen completo
            self.get_logger().info("=" * 60)
            self.get_logger().info("RESUMEN DE CALIBRACIÓN COMPLETA:")
            self.get_logger().info("📏 GEOMETRÍA DE CÁMARA:")
            self.get_logger().info(f"  Altura (Z): {self.h} m")
            self.get_logger().info(f"  Posición X: {self.camera_x} m")
            self.get_logger().info(f"  Posición Y: {self.camera_y} m")
            self.get_logger().info(f"  Pitch: {self.calculated_pitch:.4f} rad ({math.degrees(self.calculated_pitch):.2f}°)")
            
            self.get_logger().info("📷 INTRÍNSECOS:")
            self.get_logger().info(f"  fx: {self.fx:.2f} px, fy: {self.fy:.2f} px")
            self.get_logger().info(f"  cx: {self.cx:.2f} px, cy: {self.cy:.2f} px")
            
            self.get_logger().info("🌀 DISTORSIÓN:")
            for k, v in self.distortion_coeffs.items():
                self.get_logger().info(f"  {k}: {v:.6f}")
            
            self.get_logger().info(f"📊 PUNTOS USADOS: {len(self.calibration_points)}")
            self.get_logger().info("=" * 60)
            
        except Exception as e:
            self.get_logger().error(f"Error guardando calibración: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = CameraCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()