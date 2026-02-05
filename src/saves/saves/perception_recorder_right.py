#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import numpy as np
from sensor_msgs.msg import Image
from custom_interfaces.msg import SegmentationData, ObjectInfoArray, ObjectInfo
from cv_bridge import CvBridge
from pathlib import Path
import csv
import time
import cv2
import json
from datetime import datetime

class PerceptionRightRecorder(Node):
    def __init__(self):
        super().__init__('perception_right_recorder')
        
        # Parámetro de muestreo unificado (sin parámetros de profundidad)
        self.declare_parameter('save_every_n', 1)  # 1 cada 5 frames (6 FPS @ 30Hz)
        self.declare_parameter('require_objects', False)  # Nuevo parámetro
        
        # Estado de logging
        self.is_logging_enabled = False
        self.current_log_path = None
        self.node_name = "perception_right"
        
        # Configuración de muestreo UNIFICADA
        self.save_every_n = self.get_parameter('save_every_n').value
        self.require_objects = self.get_parameter('require_objects').value
        
        # Buffer para datos sincronizados (sin profundidad)
        self.current_frame = None
        self.current_frame_timestamp = None
        self.current_segmentation = None
        self.current_objects = None
        self.frame_ready = False
        
        # Contadores
        self.frame_count = 0
        self.saved_frames = 0
        
        # Puente para imágenes
        self.bridge = CvBridge()
        
        # Subscripciones a temas de percepción de la cámara derecha
        self.create_subscription(
            Image, '/camera/rgb/right',
            self.image_callback, 10)
        
        self.create_subscription(
            SegmentationData, '/segmentation/data_right',
            self.segmentation_callback, 10)
        
        self.create_subscription(
            ObjectInfoArray, '/objects/fused_info_right',
            self.objects_callback, 10)
        
        # Señales del manager
        self.create_subscription(Bool, '/logging_enabled', self.logging_enabled_cb, 10)
        self.create_subscription(String, '/current_log_path', self.log_path_cb, 10)
        
        # Timer para procesar frames completos
        self.process_timer = self.create_timer(0.033, self.process_frame)  # ~30Hz
        
        # Archivos
        self.csv_file = None
        self.csv_writer = None
        self.current_session_dir = None
        
        # Estadísticas
        self.stats_buffer = []
        self.object_classes_summary = {}
        
        #self.get_logger().info(f'PerceptionRightRecorder inicializado')
        #self.get_logger().info(f'Requiere objetos: {self.require_objects}')
        #self.get_logger().info(f'Guardando 1 de cada {self.save_every_n} frames')

    def logging_enabled_cb(self, msg):
        """Callback para habilitar/deshabilitar logging"""
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            
            if self.is_logging_enabled:
                #self.get_logger().info('Logging percepción derecha HABILITADO')
                if self.current_log_path:
                    self._start_logging()
            else:
                #self.get_logger().info('Logging percepción derecha DESHABILITADO')
                self._stop_logging()

    def log_path_cb(self, msg):
        """Callback para recibir la ruta de logging"""
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            #self.get_logger().info(f'Ruta recibida: {self.current_log_path}')
            
            if self.is_logging_enabled and not self.current_session_dir:
                self._start_logging()

    def image_callback(self, msg):
        """Recibe imagen RGB de cámara derecha"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.current_frame_timestamp = time.time()
            self._check_frame_complete()
        except Exception as e:
            pass

    def segmentation_callback(self, msg):
        """Recibe datos de segmentación de cámara derecha"""
        try:
            # Reconstruir máscara
            mask_flat = np.frombuffer(msg.mask_data, dtype=np.uint8)
            mask = mask_flat.reshape((msg.height, msg.width))
            self.current_segmentation = mask
            self._check_frame_complete()
        except Exception as e:
            pass

    def objects_callback(self, msg):
        """Recibe objetos fusionados con información 3D de cámara derecha"""
        try:
            self.current_objects = msg
            self._check_frame_complete()
        except Exception as e:
            pass

    def _check_frame_complete(self):
        """Verifica si tenemos datos suficientes para procesar un frame"""
        # CAMBIO PRINCIPAL: Marcar como ready si tenemos al menos un dato
        # En lugar de requerir todos los datos
        
        # Si require_objects es True y no tenemos objetos, no procesamos
        if self.require_objects and self.current_objects is None:
            self.frame_ready = False
            return
        
        # Marcamos como ready si tenemos al menos un dato disponible
        self.frame_ready = (
            self.current_frame is not None or 
            self.current_segmentation is not None or 
            self.current_objects is not None
        )
        
        # DEBUG: Log para ver qué datos tenemos
        #self.get_logger().info(f'Frame ready: {self.frame_ready} | RGB: {self.current_frame is not None} | Seg: {self.current_segmentation is not None} | Obj: {self.current_objects is not None}')

    def process_frame(self):
        """Procesa frames completos cuando están listos"""
        if not self.is_logging_enabled or not self.frame_ready:
            return
            
        self.frame_count += 1
        
        # Solo guardar según la frecuencia unificada
        if self.frame_count % self.save_every_n == 0:
            self._save_perception_data()
            self.saved_frames += 1
            #self.get_logger().info(f'Frame {self.saved_frames} guardado')
        
        # Resetear para siguiente frame (solo los datos que tenemos)
        self.frame_ready = False
        
        # Solo resetear los datos que efectivamente recibimos en este frame
        # Mantener los que no recibimos para posibles futuros frames
        # Esto ayuda a mantener sincronización
        
        # Note: En el siguiente ciclo, si no recibimos nuevos mensajes,
        # estos datos permanecerán y podrán usarse con nuevos datos que lleguen

    def _start_logging(self):
        """Inicia el logging creando estructura de carpetas"""
        if not self.current_log_path:
            return
            
        try:
            # Crear estructura de carpetas para cámara derecha
            base_dir = Path(self.current_log_path)
            self.current_session_dir = base_dir / "perception_right"
            self.current_session_dir.mkdir(parents=True, exist_ok=True)
            
            # Subcarpetas organizadas
            (self.current_session_dir / "segmentation").mkdir(exist_ok=True)
            (self.current_session_dir / "detections").mkdir(exist_ok=True)
            (self.current_session_dir / "images").mkdir(exist_ok=True)
            
            # Archivo CSV principal
            csv_path = self.current_session_dir / "perception_data.csv"
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # ENCABEZADO CSV (adaptado para cuando no hay objetos)
            header = [
                'timestamp', 'frame_id', 
                # Segmentación
                'seg_filename', 'drivable_percent', 'lane_percent', 'num_lane_contours',
                # Objetos fusionados (pueden estar vacíos)
                'obj_filename', 'num_objects', 'num_static', 'num_moving',
                # Información por objeto
                'object_classes', 'object_confidences', 'object_distances', 'object_lateral_offsets',
                # Imagen RGB
                'image_filename'
            ]
            self.csv_writer.writerow(header)
            self.csv_file.flush()
            
            # Archivo JSON para metadatos de clases
            self.class_metadata_path = self.current_session_dir / "object_classes.json"
            self.object_classes_summary = {}
            
            # Resetear contadores
            self.frame_count = 0
            self.saved_frames = 0
            self.stats_buffer.clear()
            
            #self.get_logger().info(f'Logging percepción derecha iniciado en: {self.current_session_dir}')
            
        except Exception as e:
            pass

    def _save_perception_data(self):
        """Guarda todos los datos de percepción para el frame actual"""
        try:
            timestamp = time.time()
            frame_id = f"frame_{self.frame_count:06d}"
            
            # Inicializar diccionario de datos
            frame_data = {
                'timestamp': timestamp,
                'frame_id': frame_id,
                'seg_filename': '',
                'obj_filename': '',
                'image_filename': '',
                'num_objects': 0,
                'num_static': 0,
                'num_moving': 0,
                'object_classes': '',
                'object_confidences': '',
                'object_distances': '',
                'object_lateral_offsets': ''
            }
            
            # 1. GUARDAR SEGMENTACIÓN (si está disponible)
            if self.current_segmentation is not None:
                seg_stats = self._process_segmentation(frame_id)
                frame_data.update(seg_stats)
            
            # 2. GUARDAR OBJETOS FUSIONADOS (si están disponibles)
            if self.current_objects is not None:
                obj_stats = self._process_objects(frame_id)
                frame_data.update(obj_stats)
            else:
                # Si no hay objetos, crear archivo JSON vacío solo si estamos guardando este frame
                obj_filename = f"{frame_id}_det.json"
                obj_path = self.current_session_dir / "detections" / obj_filename
                with open(obj_path, 'w') as f:
                    json.dump({
                        'timestamp': time.time(),
                        'frame_id': frame_id,
                        'robot_speed': 0.0,
                        'processing_time_ms': 0.0,
                        'num_static_objects': 0,
                        'num_moving_objects': 0,
                        'objects': []
                    }, f, indent=2)
                frame_data['obj_filename'] = obj_filename
            
            # 3. GUARDAR IMAGEN RGB (si está disponible)
            if self.current_frame is not None:
                img_filename = f"{frame_id}_image.jpg"
                img_path = self.current_session_dir / "images" / img_filename
                cv2.imwrite(str(img_path), self.current_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                frame_data['image_filename'] = img_filename
                #self.get_logger().info(f'Imagen guardada: {img_filename}')
            
            # 4. GUARDAR EN CSV (solo si tenemos algún dato)
            # Verificar que al menos tengamos un dato para guardar
            has_data = (
                frame_data['seg_filename'] != '' or 
                frame_data['obj_filename'] != '' or 
                frame_data['image_filename'] != ''
            )
            
            if has_data:
                self._save_frame_to_csv(frame_data)
                
                # 5. Acumular para batch processing
                self.stats_buffer.append(frame_data)
                if len(self.stats_buffer) >= 20:
                    self._flush_stats_buffer()
            else:
                # Si no tenemos ningún dato, no guardamos nada
                #self.get_logger().warn(f'Frame {frame_id} sin datos para guardar')
                pass
            
        except Exception as e:
            #self.get_logger().error(f'Error guardando percepción: {str(e)}')
            pass

    def _process_segmentation(self, frame_id):
        """Procesa y guarda datos de segmentación"""
        stats = {
            'seg_filename': '',
            'drivable_percent': 0.0,
            'lane_percent': 0.0,
            'num_lane_contours': 0
        }
        
        try:
            # Guardar máscara
            seg_filename = f"{frame_id}_seg.png"
            seg_path = self.current_session_dir / "segmentation" / seg_filename
            cv2.imwrite(str(seg_path), self.current_segmentation, 
                       [cv2.IMWRITE_PNG_COMPRESSION, 9])
            stats['seg_filename'] = seg_filename
            
            # Calcular estadísticas
            total_pixels = self.current_segmentation.size
            drivable_pixels = np.sum(self.current_segmentation == 1)
            lane_pixels = np.sum(self.current_segmentation == 2)
            
            if total_pixels > 0:
                stats['drivable_percent'] = (drivable_pixels / total_pixels) * 100
                stats['lane_percent'] = (lane_pixels / total_pixels) * 100
            
            # Contornos de carriles
            lane_mask = (self.current_segmentation == 2).astype(np.uint8) * 255
            if np.any(lane_mask):
                contours, _ = cv2.findContours(lane_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                stats['num_lane_contours'] = len([c for c in contours if cv2.contourArea(c) > 50])
            
        except Exception as e:
            pass
        
        return stats

    def _process_objects(self, frame_id):
        """Procesa y guarda datos de objetos fusionados"""
        stats = {
            'obj_filename': '',
            'num_objects': 0,
            'num_static': 0,
            'num_moving': 0,
            'object_classes': '',
            'object_confidences': '',
            'object_distances': '',
            'object_lateral_offsets': ''
        }
        
        try:
            # Inicializar valores del mensaje si están disponibles
            if hasattr(self.current_objects, 'num_static_objects'):
                stats['num_static'] = self.current_objects.num_static_objects
            if hasattr(self.current_objects, 'num_moving_objects'):
                stats['num_moving'] = self.current_objects.num_moving_objects
            
            # Guardar objetos en JSON
            obj_filename = f"{frame_id}_det.json"
            obj_path = self.current_session_dir / "detections" / obj_filename
            
            objects_data = []
            class_names = []
            confidences = []
            distances = []
            lateral_offsets = []
            
            # Verificar si hay objetos en el mensaje
            if hasattr(self.current_objects, 'objects'):
                for obj in self.current_objects.objects:
                    obj_dict = {
                        'track_id': int(obj.track_id),
                        'class_id': int(obj.class_id),
                        'class_name': str(obj.class_name),
                        'confidence': float(obj.confidence),
                        'bbox': [int(obj.x1), int(obj.y1), int(obj.x2), int(obj.y2)],
                        'center': [float(obj.center_x), float(obj.center_y)],
                        'position_3d': {
                            'distance': float(obj.distance),
                            'lateral_offset': float(obj.lateral_offset),
                            'ground_distance': float(obj.ground_distance),
                            'distance_valid': bool(obj.distance_valid),
                            'distance_source': int(obj.distance_source)
                        }
                    }
                    objects_data.append(obj_dict)
                    
                    # Acumular datos para CSV
                    class_names.append(obj.class_name)
                    confidences.append(f"{obj.confidence:.3f}")
                    distances.append(f"{obj.distance:.2f}")
                    lateral_offsets.append(f"{obj.lateral_offset:.2f}")
            
            stats['num_objects'] = len(objects_data)
            
            # Guardar JSON si hay datos
            json_data = {
                'timestamp': time.time(),
                'frame_id': frame_id,
                'objects': objects_data
            }
            
            # Añadir campos adicionales si existen
            if hasattr(self.current_objects, 'robot_speed'):
                json_data['robot_speed'] = float(self.current_objects.robot_speed)
            if hasattr(self.current_objects, 'processing_time_ms'):
                json_data['processing_time_ms'] = float(self.current_objects.processing_time_ms)
            if hasattr(self.current_objects, 'num_static_objects'):
                json_data['num_static_objects'] = int(self.current_objects.num_static_objects)
            if hasattr(self.current_objects, 'num_moving_objects'):
                json_data['num_moving_objects'] = int(self.current_objects.num_moving_objects)
            
            with open(obj_path, 'w') as f:
                json.dump(json_data, f, indent=2)
            
            stats['obj_filename'] = obj_filename
            stats['object_classes'] = ','.join(class_names) if class_names else ''
            stats['object_confidences'] = ','.join(confidences) if confidences else ''
            stats['object_distances'] = ','.join(distances) if distances else ''
            stats['object_lateral_offsets'] = ','.join(lateral_offsets) if lateral_offsets else ''
            
        except Exception as e:
            pass
        
        return stats

    def _save_frame_to_csv(self, frame_data):
        """Guarda datos del frame en CSV"""
        if not self.csv_writer:
            return
        
        try:
            # Asegurarse de que todos los valores sean strings válidos
            for key in frame_data:
                if frame_data[key] is None:
                    frame_data[key] = ''
            
            row = [
                frame_data['timestamp'],
                frame_data['frame_id'],
                # Segmentación
                frame_data['seg_filename'],
                frame_data['drivable_percent'],
                frame_data['lane_percent'],
                frame_data['num_lane_contours'],
                # Objetos fusionados
                frame_data['obj_filename'],
                frame_data['num_objects'],
                frame_data['num_static'],
                frame_data['num_moving'],
                # Información por objeto
                frame_data['object_classes'],
                frame_data['object_confidences'],
                frame_data['object_distances'],
                frame_data['object_lateral_offsets'],
                # Imagen RGB
                frame_data['image_filename']
            ]
            
            self.csv_writer.writerow(row)
            
            # Flush cada 10 filas
            if len(self.stats_buffer) % 10 == 0:
                self.csv_file.flush()
            
        except Exception as e:
           pass

    def _flush_stats_buffer(self):
        """Actualiza resumen de clases en JSON y limpia buffer"""
        try:
            # Guardar resumen de clases si hay objetos
            if self.object_classes_summary:
                summary_data = {
                    'timestamp': time.time(),
                    'total_frames_processed': self.frame_count,
                    'frames_saved': self.saved_frames,
                    'save_frequency': self.save_every_n,
                    'camera': 'right',
                    'has_depth': False,
                    'require_objects': self.require_objects,
                    'classes_detected': {}
                }
                
                for class_name, stats in self.object_classes_summary.items():
                    avg_conf = stats['total_confidence'] / stats['count'] if stats['count'] > 0 else 0
                    avg_dist = stats['total_distance'] / stats['count'] if stats['count'] > 0 and stats['total_distance'] > 0 else 0
                    summary_data['classes_detected'][class_name] = {
                        'count': stats['count'],
                        'average_confidence': avg_conf,
                        'average_distance': avg_dist
                    }
                
                with open(self.class_metadata_path, 'w') as f:
                    json.dump(summary_data, f, indent=2)
            
            # Limpiar buffer
            self.stats_buffer.clear()
                    
        except Exception as e:
            pass

    def _stop_logging(self):
        """Finaliza logging y genera resumen"""
        # Guardar datos pendientes
        for frame_data in self.stats_buffer:
            self._save_frame_to_csv(frame_data)
        
        # Flush final
        if self.csv_file:
            self.csv_file.flush()
        
        # Actualizar resumen de clases
        self._flush_stats_buffer()
        
        # Cerrar archivo CSV
        if self.csv_file:
            try:
                self.csv_file.close()
                #self.get_logger().info(f'Datos de percepción derecha guardados: {self.saved_frames} frames')
            except Exception as e:
                pass
        
        # Generar resumen de sesión
        if self.current_session_dir:
            self._generate_session_summary()
        
        # Resetear variables
        self.csv_file = None
        self.csv_writer = None
        self.current_session_dir = None
        self.frame_count = 0
        self.saved_frames = 0
        self.stats_buffer.clear()
        self.object_classes_summary = {}
        
        # Resetear datos actuales
        self.current_frame = None
        self.current_segmentation = None
        self.current_objects = None
        self.frame_ready = False

    def _generate_session_summary(self):
        """Genera archivo de resumen de la sesión"""
        try:
            summary_path = self.current_session_dir / "session_summary.txt"
            
            # Contar archivos generados
            seg_dir = self.current_session_dir / "segmentation"
            obj_dir = self.current_session_dir / "detections"
            img_dir = self.current_session_dir / "images"
            
            seg_count = len(list(seg_dir.glob("*.png"))) if seg_dir.exists() else 0
            obj_count = len(list(obj_dir.glob("*.json"))) if obj_dir.exists() else 0
            img_count = len(list(img_dir.glob("*.jpg"))) if img_dir.exists() else 0
            
            with open(summary_path, 'w') as f:
                f.write("=== RESUMEN DE SESIÓN DE PERCEPCIÓN DERECHA ===\n\n")
                f.write(f"Fecha: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Nodo: {self.get_name()}\n")
                f.write(f"Cámara: RIGHT\n")
                f.write(f"Requiere objetos: {self.require_objects}\n")
                f.write(f"Directorio: {self.current_session_dir}\n\n")
                
                f.write("CONFIGURACIÓN:\n")
                f.write(f"  Frecuencia guardado: 1 cada {self.save_every_n} frames\n")
                f.write(f"  Tasa efectiva: {30/self.save_every_n:.1f} FPS\n\n")
                
                f.write("ESTADÍSTICAS:\n")
                f.write(f"  Frames totales procesados: {self.frame_count}\n")
                f.write(f"  Frames guardados: {self.saved_frames}\n\n")
                
                f.write("ARCHIVOS GUARDADOS:\n")
                f.write(f"  Máscaras de segmentación: {seg_count}\n")
                f.write(f"  Archivos de objetos: {obj_count}\n")
                f.write(f"  Imágenes RGB: {img_count}\n")
            
            #self.get_logger().info(f'Resumen generado: {summary_path}')
            
        except Exception as e:
            pass

    def destroy_node(self):
        self._stop_logging()
        super().destroy_node()

def main():
    rclpy.init()
    node = PerceptionRightRecorder()
    
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