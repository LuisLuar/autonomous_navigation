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

class PerceptionRecorder(Node):
    def __init__(self):
        super().__init__('perception_recorder')
        
        # Parámetro de muestreo unificado
        self.declare_parameter('save_every_n', 1)
        self.declare_parameter('save_depth', True)
        self.declare_parameter('depth_scale', 1000.0)
        self.declare_parameter('require_objects', False)  # AÑADIDO: nuevo parámetro
        
        # Estado de logging
        self.is_logging_enabled = False
        self.current_log_path = None
        self.node_name = "perception"
        
        # Configuración de muestreo UNIFICADA
        self.save_every_n = self.get_parameter('save_every_n').value
        self.save_depth = self.get_parameter('save_depth').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.require_objects = self.get_parameter('require_objects').value  # AÑADIDO
        
        # Buffer para datos sincronizados
        self.current_frame = None
        self.current_depth_frame = None
        self.current_frame_timestamp = None
        self.current_segmentation = None
        self.current_objects = None
        self.frame_ready = False
        
        # Contadores
        self.frame_count = 0
        self.saved_frames = 0
        
        # Puente para imágenes
        self.bridge = CvBridge()
        
        # Subscripciones a todos los temas de percepción
        self.create_subscription(
            Image, '/camera/rgb/image_raw',
            self.image_callback, 10)
        
        self.create_subscription(
            Image, '/camera/depth/image_raw',
            self.depth_callback, 10)
        
        self.create_subscription(
            SegmentationData, '/segmentation/data',
            self.segmentation_callback, 10)
        
        self.create_subscription(
            ObjectInfoArray, '/objects/fused_info',
            self.objects_callback, 10)
        
        # Señales del manager
        self.create_subscription(Bool, '/logging_enabled', self.logging_enabled_cb, 10)
        self.create_subscription(String, '/current_log_path', self.log_path_cb, 10)
        
        # Timer para procesar frames completos
        self.process_timer = self.create_timer(0.033, self.process_frame)
        
        # Archivos
        self.csv_file = None
        self.csv_writer = None
        self.current_session_dir = None
        
        # Estadísticas
        self.stats_buffer = []
        self.object_classes_summary = {}
        
        self.get_logger().info('PerceptionRecorder inicializado')
        self.get_logger().info(f'Requiere objetos: {self.require_objects}')

    def logging_enabled_cb(self, msg):
        """Callback para habilitar/deshabilitar logging"""
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            
            if self.is_logging_enabled:
                self.get_logger().info('Logging percepción HABILITADO')
                if self.current_log_path:
                    self._start_logging()
            else:
                self.get_logger().info('Logging percepción DESHABILITADO')
                self._stop_logging()

    def log_path_cb(self, msg):
        """Callback para recibir la ruta de logging"""
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            self.get_logger().info(f'Ruta recibida: {self.current_log_path}')
            
            if self.is_logging_enabled and not self.current_session_dir:
                self._start_logging()

    def image_callback(self, msg):
        """Recibe imagen RGB de cámara"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.current_frame_timestamp = time.time()
            self._check_frame_complete()
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen RGB: {e}')

    def depth_callback(self, msg):
        """Recibe imagen de profundidad"""
        if not self.save_depth:
            return
            
        try:
            if msg.encoding == '16UC1':
                self.current_depth_frame = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            elif msg.encoding == '32FC1':
                depth_float = self.bridge.imgmsg_to_cv2(msg, '32FC1')
                self.current_depth_frame = (depth_float * 1000).astype(np.uint16)
            else:
                self.get_logger().warn(f'Formato de profundidad no soportado: {msg.encoding}')
                return
                
            self._check_frame_complete()
        except Exception as e:
            self.get_logger().error(f'Error procesando profundidad: {e}')

    def segmentation_callback(self, msg):
        """Recibe datos de segmentación"""
        try:
            mask_flat = np.frombuffer(msg.mask_data, dtype=np.uint8)
            mask = mask_flat.reshape((msg.height, msg.width))
            self.current_segmentation = mask
            self._check_frame_complete()
        except Exception as e:
            self.get_logger().error(f'Error procesando segmentación: {e}')

    def objects_callback(self, msg):
        """Recibe objetos fusionados con información 3D"""
        try:
            self.current_objects = msg
            self._check_frame_complete()
        except Exception as e:
            self.get_logger().error(f'Error procesando objetos: {e}')

    def _check_frame_complete(self):
        """Verifica si tenemos datos suficientes para procesar un frame"""
        # CAMBIO CRÍTICO: Si require_objects es True y no tenemos objetos, no procesamos
        if self.require_objects and self.current_objects is None:
            self.frame_ready = False
            return
        
        # Marcamos como ready si tenemos al menos un dato disponible
        self.frame_ready = (
            self.current_frame is not None or 
            self.current_segmentation is not None or 
            self.current_objects is not None or
            (self.save_depth and self.current_depth_frame is not None)
        )
        
        # DEBUG opcional
        # self.get_logger().info(f'Frame ready: {self.frame_ready} | RGB: {self.current_frame is not None} | Seg: {self.current_segmentation is not None} | Obj: {self.current_objects is not None} | Depth: {self.current_depth_frame is not None}')

    def _start_logging(self):
        """Inicia el logging creando estructura de carpetas"""
        if not self.current_log_path:
            return
            
        try:
            # Crear estructura de carpetas
            base_dir = Path(self.current_log_path)
            self.current_session_dir = base_dir / "perception"
            self.current_session_dir.mkdir(parents=True, exist_ok=True)
            
            # Subcarpetas organizadas
            (self.current_session_dir / "segmentation").mkdir(exist_ok=True)
            (self.current_session_dir / "detections").mkdir(exist_ok=True)
            (self.current_session_dir / "images").mkdir(exist_ok=True)
            
            if self.save_depth:
                (self.current_session_dir / "depth").mkdir(exist_ok=True)
                (self.current_session_dir / "depth_vis").mkdir(exist_ok=True)
            
            # Archivo CSV principal - SIMPLIFICADO COMO EN EL SEGUNDO CÓDIGO
            csv_path = self.current_session_dir / "perception_data.csv"
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # ENCABEZADO CSV SIMPLIFICADO (basado en el segundo código que funciona)
            header = [
                'timestamp', 'frame_id', 
                # Segmentación
                'seg_filename', 'drivable_percent', 'lane_percent', 'num_lane_contours',
                # Objetos fusionados
                'obj_filename', 'num_objects', 'num_static', 'num_moving',
                # Información por objeto
                'object_classes', 'object_confidences', 'object_distances', 'object_lateral_offsets',
                # Imagen RGB
                'image_filename',
                # Profundidad (condicional)
                'depth_filename', 'depth_vis_filename', 'avg_depth', 'min_depth', 'max_depth'
            ]
            
            # Añadir campos adicionales solo si save_depth está activado
            if self.save_depth:
                # Ya están incluidos en el header
                pass
                
            self.csv_writer.writerow(header)
            self.csv_file.flush()
            
            # Archivo JSON para metadatos de clases
            self.class_metadata_path = self.current_session_dir / "object_classes.json"
            self.object_classes_summary = {}
            
            # Resetear contadores
            self.frame_count = 0
            self.saved_frames = 0
            self.stats_buffer.clear()
            
            self.get_logger().info(f'Logging percepción iniciado en: {self.current_session_dir}')
            
        except Exception as e:
            self.get_logger().error(f'Error iniciando logging: {e}')

    def process_frame(self):
        """Procesa frames completos cuando están listos"""
        if not self.is_logging_enabled or not self.frame_ready:
            return
                
        self.frame_count += 1
        
        # Solo guardar según la frecuencia unificada
        if self.frame_count % self.save_every_n == 0:
            self._save_perception_data()
            self.saved_frames += 1
        
        # CAMBIO IMPORTANTE: No resetear los datos aquí
        # Solo resetear el flag para esperar nuevos datos
        self.frame_ready = False
        
        # Mantener los datos actuales hasta que lleguen nuevos
        # Esto permite usar datos parciales si no llegan todos

    def _save_perception_data(self):
        """Guarda todos los datos de percepción para el frame actual"""
        try:
            timestamp = time.time()  # Usar tiempo actual si no hay timestamp específico
            if self.current_frame_timestamp:
                timestamp = self.current_frame_timestamp
                
            frame_id = f"frame_{self.frame_count:06d}"
            
            # Inicializar diccionario de datos CON VALORES POR DEFECTO
            frame_data = {
                'timestamp': timestamp,
                'frame_id': frame_id,
                'seg_filename': '',
                'obj_filename': '',
                'image_filename': '',
                'depth_filename': '',
                'depth_vis_filename': '',
                'avg_depth': 0.0,
                'min_depth': 0.0,
                'max_depth': 0.0,
                'num_objects': 0,
                'num_static': 0,
                'num_moving': 0,
                'object_classes': '',
                'object_confidences': '',
                'object_distances': '',
                'object_lateral_offsets': '',
                'drivable_percent': 0.0,
                'lane_percent': 0.0,
                'num_lane_contours': 0
            }
            
            has_any_data = False
            
            # 1. GUARDAR SEGMENTACIÓN (si está disponible)
            if self.current_segmentation is not None:
                seg_stats = self._process_segmentation(frame_id)
                frame_data.update(seg_stats)
                has_any_data = True
            
            # 2. GUARDAR OBJETOS FUSIONADOS (si están disponibles)
            if self.current_objects is not None:
                obj_stats = self._process_objects(frame_id)
                frame_data.update(obj_stats)
                has_any_data = True
            else:
                # Si no hay objetos, crear archivo JSON vacío (como en el segundo código)
                obj_filename = f"{frame_id}_det.json"
                obj_path = self.current_session_dir / "detections" / obj_filename
                with open(obj_path, 'w') as f:
                    json.dump({
                        'timestamp': time.time(),
                        'frame_id': frame_id,
                        'objects': []
                    }, f, indent=2)
                frame_data['obj_filename'] = obj_filename
                has_any_data = True
            
            # 3. GUARDAR IMAGEN RGB (si está disponible)
            if self.current_frame is not None:
                img_filename = f"{frame_id}_image.jpg"
                img_path = self.current_session_dir / "images" / img_filename
                cv2.imwrite(str(img_path), self.current_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                frame_data['image_filename'] = img_filename
                has_any_data = True
            
            # 4. GUARDAR PROFUNDIDAD (si está activada y disponible)
            if self.save_depth and self.current_depth_frame is not None:
                depth_stats = self._process_depth(frame_id)
                frame_data.update(depth_stats)
                has_any_data = True
            
            # 5. GUARDAR EN CSV solo si tenemos algún dato
            if has_any_data:
                self._save_frame_to_csv(frame_data)
                
                # Acumular para batch processing
                self.stats_buffer.append(frame_data)
                if len(self.stats_buffer) >= 20:
                    self._flush_stats_buffer()
                
                # Log cada 50 frames guardados
                if self.saved_frames % 50 == 0:
                    self.get_logger().info(f'Frames guardados: {self.saved_frames}')
            else:
                self.get_logger().warn(f'Frame {frame_id} sin datos para guardar')
            
        except Exception as e:
            self.get_logger().error(f'Error guardando percepción: {e}')

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
            self.get_logger().error(f'Error procesando segmentación: {e}')
        
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
            # Usar valores del mensaje si están disponibles
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
                            'distance_valid': bool(obj.distance_valid)
                        }
                    }
                    objects_data.append(obj_dict)
                    
                    # Actualizar resumen de clases
                    class_name = obj.class_name
                    if class_name not in self.object_classes_summary:
                        self.object_classes_summary[class_name] = {
                            'count': 0,
                            'total_confidence': 0.0,
                            'total_distance': 0.0
                        }
                    self.object_classes_summary[class_name]['count'] += 1
                    self.object_classes_summary[class_name]['total_confidence'] += obj.confidence
                    if obj.distance_valid:
                        self.object_classes_summary[class_name]['total_distance'] += obj.distance
                    
                    # Acumular datos para CSV
                    class_names.append(class_name)
                    confidences.append(f"{obj.confidence:.3f}")
                    distances.append(f"{obj.distance:.2f}")
                    lateral_offsets.append(f"{obj.lateral_offset:.2f}")
            
            stats['num_objects'] = len(objects_data)
            
            # Solo guardar JSON si hay objetos
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
            self.get_logger().error(f'Error procesando objetos: {e}')
        
        return stats

    def _process_depth(self, frame_id):
        """Procesa y guarda datos de profundidad"""
        stats = {
            'depth_filename': '',
            'depth_vis_filename': '',
            'avg_depth': 0.0,
            'min_depth': 0.0,
            'max_depth': 0.0
        }
        
        try:
            # 1. Guardar mapa de profundidad crudo
            depth_filename = f"{frame_id}_depth.png"
            depth_path = self.current_session_dir / "depth" / depth_filename
            cv2.imwrite(str(depth_path), self.current_depth_frame)
            stats['depth_filename'] = depth_filename
            
            # 2. Calcular estadísticas de profundidad
            depth_valid = self.current_depth_frame[self.current_depth_frame > 0]
            if len(depth_valid) > 0:
                if np.max(depth_valid) > 1000:
                    depth_valid_meters = depth_valid.astype(np.float32) / 1000.0
                else:
                    depth_valid_meters = depth_valid.astype(np.float32)
                
                stats['avg_depth'] = np.mean(depth_valid_meters)
                stats['min_depth'] = np.min(depth_valid_meters)
                stats['max_depth'] = np.max(depth_valid_meters)
            
            # 3. Crear y guardar visualización de profundidad
            depth_vis = self._create_depth_visualization(self.current_depth_frame)
            depth_vis_filename = f"{frame_id}_depth_vis.jpg"
            depth_vis_path = self.current_session_dir / "depth_vis" / depth_vis_filename
            cv2.imwrite(str(depth_vis_path), depth_vis, [cv2.IMWRITE_JPEG_QUALITY, 90])
            stats['depth_vis_filename'] = depth_vis_filename
            
        except Exception as e:
            self.get_logger().error(f'Error procesando profundidad: {e}')
        
        return stats

    def _create_depth_visualization(self, depth_frame):
        """Crea una visualización en color del mapa de profundidad"""
        depth_vis = depth_frame.copy().astype(np.float32)
        
        depth_valid = depth_vis[depth_vis > 0]
        if len(depth_valid) > 0:
            if np.max(depth_valid) > 1000:
                depth_vis[depth_vis > 0] = depth_vis[depth_vis > 0] / 1000.0
            
            depth_normalized = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX)
            depth_normalized = np.nan_to_num(depth_normalized, nan=0.0)
            depth_normalized = depth_normalized.astype(np.uint8)
            
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            depth_colored[depth_frame == 0] = [0, 0, 0]
            
            return depth_colored
        else:
            return np.zeros((depth_frame.shape[0], depth_frame.shape[1], 3), dtype=np.uint8)

    def _save_frame_to_csv(self, frame_data):
        """Guarda datos del frame en CSV"""
        if not self.csv_writer:
            return
        
        try:
            # Reemplazar valores None con cadenas vacías o valores por defecto
            for key in frame_data:
                if frame_data[key] is None:
                    if key in ['drivable_percent', 'lane_percent', 'avg_depth', 'min_depth', 'max_depth']:
                        frame_data[key] = 0.0
                    elif key in ['num_lane_contours', 'num_objects', 'num_static', 'num_moving']:
                        frame_data[key] = 0
                    else:
                        frame_data[key] = ''
            
            # Construir fila según el header simplificado
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
            
            # Añadir campos de profundidad si save_depth está activado
            if self.save_depth:
                row.extend([
                    frame_data['depth_filename'],
                    frame_data['depth_vis_filename'],
                    frame_data['avg_depth'],
                    frame_data['min_depth'],
                    frame_data['max_depth']
                ])
            
            self.csv_writer.writerow(row)
            
            # Flush cada 10 filas para equilibrio performance/robustez
            if len(self.stats_buffer) % 10 == 0:
                self.csv_file.flush()
            
            self.get_logger().debug(f'Datos CSV guardados para frame {frame_data["frame_id"]}')
            
        except Exception as e:
            self.get_logger().error(f'Error guardando en CSV: {e}')

    def _flush_stats_buffer(self):
        """Actualiza resumen de clases en JSON y limpia buffer"""
        try:
            # Guardar resumen de clases
            if self.object_classes_summary:
                summary_data = {
                    'timestamp': time.time(),
                    'total_frames_processed': self.frame_count,
                    'frames_saved': self.saved_frames,
                    'save_frequency': self.save_every_n,
                    'save_depth': self.save_depth,
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
            self.get_logger().error(f'Error actualizando resumen: {e}')

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
                self.get_logger().info(f'Datos de percepción guardados: {self.saved_frames} frames')
            except Exception as e:
                self.get_logger().error(f'Error cerrando CSV: {e}')
        
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
        self.current_depth_frame = None
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
            depth_dir = self.current_session_dir / "depth" if self.save_depth else None
            depth_vis_dir = self.current_session_dir / "depth_vis" if self.save_depth else None
            
            seg_count = len(list(seg_dir.glob("*.png"))) if seg_dir.exists() else 0
            obj_count = len(list(obj_dir.glob("*.json"))) if obj_dir.exists() else 0
            img_count = len(list(img_dir.glob("*.jpg"))) if img_dir.exists() else 0
            depth_count = len(list(depth_dir.glob("*.png"))) if depth_dir and depth_dir.exists() else 0
            depth_vis_count = len(list(depth_vis_dir.glob("*.jpg"))) if depth_vis_dir and depth_vis_dir.exists() else 0
            
            with open(summary_path, 'w') as f:
                f.write("=== RESUMEN DE SESIÓN DE PERCEPCIÓN ===\n\n")
                f.write(f"Fecha: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Nodo: {self.get_name()}\n")
                f.write(f"Requiere objetos: {self.require_objects}\n")
                f.write(f"Directorio: {self.current_session_dir}\n\n")
                
                f.write("CONFIGURACIÓN:\n")
                f.write(f"  Frecuencia guardado: 1 cada {self.save_every_n} frames\n")
                f.write(f"  Profundidad guardada: {'Sí' if self.save_depth else 'No'}\n")
                f.write(f"  Tasa efectiva: {30/self.save_every_n:.1f} FPS\n\n")
                
                f.write("ESTADÍSTICAS:\n")
                f.write(f"  Frames totales procesados: {self.frame_count}\n")
                f.write(f"  Frames guardados: {self.saved_frames}\n\n")
                
                f.write("ARCHIVOS GUARDADOS:\n")
                f.write(f"  Máscaras de segmentación: {seg_count}\n")
                f.write(f"  Archivos de objetos: {obj_count}\n")
                f.write(f"  Imágenes RGB: {img_count}\n")
                if self.save_depth:
                    f.write(f"  Mapas de profundidad: {depth_count}\n")
                    f.write(f"  Visualizaciones de profundidad: {depth_vis_count}\n")
            
            self.get_logger().info(f'Resumen generado: {summary_path}')
            
        except Exception as e:
            self.get_logger().error(f'Error generando resumen: {e}')

    def destroy_node(self):
        self._stop_logging()
        super().destroy_node()

def main():
    rclpy.init()
    node = PerceptionRecorder()
    
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