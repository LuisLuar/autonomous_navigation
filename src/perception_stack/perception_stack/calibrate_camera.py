#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import math
import json
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        
        # ===== PARÁMETROS =====
        self.declare_parameter("camera_height", 0.38)  # m
        self.declare_parameter("save_path", "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration_left.json")
        
        self.h = self.get_parameter("camera_height").value
        self.save_path = self.get_parameter("save_path").value
        
        # ===== VARIABLES DE CALIBRACIÓN =====
        self.calibration_points = []  # Lista de puntos: {'Z': distancia, 'v': fila, 'u': columna}
        self.current_point = None
        self.calibration_complete = False
        
        # ===== INTRINSECOS =====
        self.fx = self.fy = self.cx = self.cy = None
        self.ready = False
        
        # ===== ROS =====
        self.bridge = CvBridge()
        self.create_subscription(
            Image,
            "/camera/rgb/left",
            self.image_callback,
            10
        )
        
        self.get_logger().info("🎯 NODO DE CALIBRACIÓN VISUAL INICIADO")
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
            ord('2'): 1.5,
            ord('3'): 2.0,
            ord('4'): 2.5,
            ord('5'): 3.0,
            ord('6'): 3.5,
            ord('7'): 4.0,
            ord('8'): 4.5,
            ord('9'): 5.0,
        }
        
        self.current_distance = 2.0  # Distancia por defecto
        
        # Para callback de mouse
        self.image_display = None
        cv2.namedWindow("Calibration - Click points")
        cv2.setMouseCallback("Calibration - Click points", self.mouse_callback)

    def image_callback(self, msg):
        """Recibe imagen y la muestra para calibración"""
        try:
            # Convertir a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_display = cv_image.copy()
            H, W = cv_image.shape[:2]
            
            # Obtener intrínsecos del topic si están disponibles
            if not self.ready:
                # Intentar obtener de mensaje (si vienen en otro topic)
                # Por ahora asumimos valores conocidos
                self.fx = 574.1
                self.fy = 574.1
                self.cx = W / 2
                self.cy = H / 2
                self.ready = True
                self.get_logger().info(f"Imagen: {W}x{H}")
                self.get_logger().info(f"Intrínsecos asumidos: fx={self.fx}, fy={self.fy}")
            
            # Dibujar información
            cv2.putText(self.image_display, f"ALTURA CAMARA: {self.h}m", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(self.image_display, f"DISTANCIA ACTUAL: {self.current_distance}m (Teclas 1-9)", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(self.image_display, f"PUNTOS MARCADOS: {len(self.calibration_points)}", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
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
            y_pos = H - 100
            instructions = [
                "CONTROLES:",
                "Teclas 1-9: Seleccionar distancia (1=1m, 2=1.5m, ...)",
                "CLIC: Marcar punto en la imagen",
                "c: Calcular pitch automáticamente",
                "s: Guardar calibración",
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
        """Guarda la calibración en archivo JSON"""
        if not hasattr(self, 'calculated_pitch'):
            self.get_logger().warn("Primero calcula el pitch (presiona 'c')")
            return
        
        calibration_data = {
            'camera_height': self.h,
            'camera_pitch': float(self.calculated_pitch),
            'camera_pitch_degrees': float(math.degrees(self.calculated_pitch)),
            'intrinsics': {
                'fx': float(self.fx),
                'fy': float(self.fy),
                'cx': float(self.cx),
                'cy': float(self.cy)
            },
            'calibration_points': self.calibration_points,
            'timestamp': self.get_clock().now().to_msg().sec
        }
        
        try:
            with open(self.save_path, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            self.get_logger().info(f"✅ Calibración guardada en: {self.save_path}")
            
            # Mostrar resumen
            self.get_logger().info("=" * 60)
            self.get_logger().info("RESUMEN DE CALIBRACIÓN:")
            self.get_logger().info(f"  Altura: {self.h} m")
            self.get_logger().info(f"  Pitch: {self.calculated_pitch:.4f} rad")
            self.get_logger().info(f"  Pitch: {math.degrees(self.calculated_pitch):.2f}°")
            self.get_logger().info(f"  Puntos usados: {len(self.calibration_points)}")
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