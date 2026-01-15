#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import numpy as np
from sensor_msgs.msg import Image
from custom_interfaces.msg import SegmentationData, ObjectInfoArray, ObjectInfo  # Cambiado
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
        self.declare_parameter('save_every_n', 5)  # 1 cada 5 frames (6 FPS @ 30Hz)
        self.declare_parameter('save_depth', True)  # Guardar imágenes de profundidad
        self.declare_parameter('depth_scale', 1000.0)  # Escala para profundidad (metros a mm)
        
        # Estado de logging
        self.is_logging_enabled = False
        self.current_log_path = None
        self.node_name = "perception"
        
        # Configuración de muestreo UNIFICADA
        self.save_every_n = self.get_parameter('save_every_n').value
        self.save_depth = self.get_parameter('save_depth').value
        self.depth_scale = self.get_parameter('depth_scale').value
        
        # Buffer para datos sincronizados
        self.current_frame = None
        self.current_depth_frame = None
        self.current_frame_timestamp = None
        self.current_segmentation = None
        self.current_objects = None  # Cambiado de current_detections
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
        
        # SUBSCRIPCIÓN ALIMINADA: /detection/results
        # SUBSCRIPCIÓN NUEVA: /objects/fused_info
        self.create_subscription(
            ObjectInfoArray, '/objects/fused_info',  # Cambiado
            self.objects_callback, 10)  # Cambiado
        
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
        self.object_classes_summary = {}  # Cambiado
        
        #self.get_logger().info('  PerceptionRecorder inicializado')
        #self.get_logger().info(f'Configuración: Guardando 1 de cada {self.save_every_n} frames')
        #self.get_logger().info(f'Profundidad: {"Activada" if self.save_depth else "Desactivada"}')

    def logging_enabled_cb(self, msg):
        """Callback para habilitar/deshabilitar logging"""
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            
            if self.is_logging_enabled:
                #self.get_logger().info(' Logging percepción HABILITADO')
                if self.current_log_path:
                    self._start_logging()
            else:
                #self.get_logger().info(' Logging percepción DESHABILITADO')
                self._stop_logging()

    def log_path_cb(self, msg):
        """Callback para recibir la ruta de logging"""
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            #self.get_logger().info(f' Ruta recibida: {self.current_log_path}')
            
            if self.is_logging_enabled and not self.current_session_dir:
                self._start_logging()

    def image_callback(self, msg):
        """Recibe imagen RGB de cámara"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.current_frame_timestamp = time.time()
            self._check_frame_complete()
        except Exception as e:
            #self.get_logger().error(f'Error procesando imagen RGB: {e}')
            pass

    def depth_callback(self, msg):
        """Recibe imagen de profundidad"""
        if not self.save_depth:
            return
            
        try:
            # Convertir mensaje de profundidad a array numpy
            # La profundidad normalmente viene en formato 16UC1 (milímetros) o 32FC1 (metros)
            if msg.encoding == '16UC1':
                # Profundidad en milímetros (uint16)
                self.current_depth_frame = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            elif msg.encoding == '32FC1':
                # Profundidad en metros (float32)
                depth_float = self.bridge.imgmsg_to_cv2(msg, '32FC1')
                # Convertir a milímetros (uint16) para guardar
                self.current_depth_frame = (depth_float * 1000).astype(np.uint16)
            else:
                #self.get_logger().warn(f'Formato de profundidad no soportado: {msg.encoding}')
                return
                
            self._check_frame_complete()
        except Exception as e:
            #self.get_logger().error(f'Error procesando profundidad: {e}')
            pass

    def segmentation_callback(self, msg):
        """Recibe datos de segmentación"""
        try:
            # Reconstruir máscara
            mask_flat = np.frombuffer(msg.mask_data, dtype=np.uint8)
            mask = mask_flat.reshape((msg.height, msg.width))
            self.current_segmentation = mask
            self._check_frame_complete()
        except Exception as e:
            #self.get_logger().error(f'Error procesando segmentación: {e}')
            pass

    def objects_callback(self, msg):  # Cambiado
        """Recibe objetos fusionados con información 3D"""
        try:
            self.current_objects = msg  # Cambiado de current_detections
            self._check_frame_complete()
        except Exception as e:
            #self.get_logger().error(f'Error procesando objetos: {e}')  # Cambiado
            pass

    def _check_frame_complete(self):
        """Verifica si tenemos todos los datos necesarios para un frame"""
        # Requisitos mínimos: RGB + segmentación + objetos fusionados
        # Profundidad es opcional
        has_required_data = (
            self.current_frame is not None and 
            self.current_segmentation is not None and 
            self.current_objects is not None  # Cambiado
        )
        
        # Si tenemos profundidad configurada, esperar también esa
        if self.save_depth:
            has_required_data = has_required_data and (self.current_depth_frame is not None)
        
        self.frame_ready = has_required_data

    def process_frame(self):
        """Procesa frames completos cuando están listos"""
        if not self.is_logging_enabled or not self.frame_ready:
            return
            
        self.frame_count += 1
        
        # Solo guardar según la frecuencia unificada
        if self.frame_count % self.save_every_n == 0:
            self._save_perception_data()
            self.saved_frames += 1
        
        # Resetear para siguiente frame
        self.frame_ready = False
        self.current_frame = None
        self.current_depth_frame = None
        self.current_segmentation = None
        self.current_objects = None  # Cambiado

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
            (self.current_session_dir / "objects").mkdir(exist_ok=True)  # Cambiado
            (self.current_session_dir / "images").mkdir(exist_ok=True)
            
            if self.save_depth:
                (self.current_session_dir / "depth").mkdir(exist_ok=True)
                (self.current_session_dir / "depth_vis").mkdir(exist_ok=True)  # Para visualización
            
            # Archivo CSV principal
            csv_path = self.current_session_dir / "perception_data.csv"
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # NUEVO ENCABEZADO CSV CON INFORMACIÓN FUSIONADA
            header = [
                'timestamp', 'frame_id', 
                # Segmentación
                'seg_filename', 'drivable_percent', 'lane_percent', 'num_lane_contours',
                # Objetos fusionados
                'obj_filename', 'num_objects', 'num_static', 'num_moving',
                # Información por objeto
                'object_classes', 'object_confidences', 'object_distances', 'object_lateral_offsets',
                'object_velocities_x', 'object_velocities_y', 'object_speeds', 'object_relative_speeds',
                'object_is_static', 'object_is_moving_toward', 'object_is_moving_away',
                'object_is_in_path', 'object_ttc', 'object_distance_source',
                # Información combinada
                'objects_in_drivable_area', 'objects_near_lanes', 'has_pedestrian', 'has_vehicle',
                # Imagen RGB
                'image_filename',
                # Profundidad
                'depth_filename', 'depth_vis_filename', 'avg_depth', 'min_depth', 'max_depth',
                # Información adicional del sistema
                'robot_speed', 'processing_time_ms'
            ]
            self.csv_writer.writerow(header)
            self.csv_file.flush()
            
            # Archivo JSON para metadatos de clases
            self.class_metadata_path = self.current_session_dir / "object_classes.json"  # Cambiado
            self.object_classes_summary = {}  # Cambiado
            
            # Resetear contadores
            self.frame_count = 0
            self.saved_frames = 0
            self.stats_buffer.clear()
            
            #self.get_logger().info(f' Logging percepción iniciado en: {self.current_session_dir}')
            
        except Exception as e:
            #self.get_logger().error(f'Error iniciando logging: {e}')
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
                'obj_filename': '',  # Cambiado
                'image_filename': '',
                'depth_filename': '',
                'depth_vis_filename': '',
                'avg_depth': 0.0,
                'min_depth': 0.0,
                'max_depth': 0.0,
                'robot_speed': 0.0,
                'processing_time_ms': 0.0
            }
            
            # 1. GUARDAR SEGMENTACIÓN
            if self.current_segmentation is not None:
                seg_stats = self._process_segmentation(frame_id)
                frame_data.update(seg_stats)
            
            # 2. GUARDAR OBJETOS FUSIONADOS (CAMBIADO)
            if self.current_objects is not None:
                obj_stats = self._process_objects(frame_id)  # Cambiado
                frame_data.update(obj_stats)
            
            # 3. GUARDAR IMAGEN RGB
            if self.current_frame is not None:
                img_filename = f"{frame_id}_image.jpg"
                img_path = self.current_session_dir / "images" / img_filename
                cv2.imwrite(str(img_path), self.current_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                frame_data['image_filename'] = img_filename
            
            # 4. GUARDAR PROFUNDIDAD (si está activada)
            if self.save_depth and self.current_depth_frame is not None:
                depth_stats = self._process_depth(frame_id)
                frame_data.update(depth_stats)
            
            # 5. INFORMACIÓN DEL SISTEMA
            if self.current_objects is not None:
                frame_data['robot_speed'] = self.current_objects.robot_speed
                frame_data['processing_time_ms'] = self.current_objects.processing_time_ms
            
            # 6. GUARDAR EN CSV
            self._save_frame_to_csv(frame_data)
            
            # 7. Acumular para batch processing
            self.stats_buffer.append(frame_data)
            if len(self.stats_buffer) >= 20:
                self._flush_stats_buffer()
            
            # Log cada 50 frames guardados
            #if self.saved_frames % 50 == 0:
                #self.get_logger().info(f' Frames guardados: {self.saved_frames}')
            
        except Exception as e:
            #self.get_logger().error(f'Error guardando percepción: {e}')
            pass

    def _process_segmentation(self, frame_id):
        """Procesa y guarda datos de segmentación (sin cambios)"""
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
            #self.get_logger().error(f'Error procesando segmentación: {e}')
            pass
        
        return stats

    def _process_objects(self, frame_id):  # CAMBIADO COMPLETAMENTE
        """Procesa y guarda datos de objetos fusionados"""
        stats = {
            'obj_filename': '',
            'num_objects': 0,
            'num_static': self.current_objects.num_static_objects,
            'num_moving': self.current_objects.num_moving_objects,
            'object_classes': '',
            'object_confidences': '',
            'object_distances': '',
            'object_lateral_offsets': '',
            'object_velocities_x': '',
            'object_velocities_y': '',
            'object_speeds': '',
            'object_relative_speeds': '',
            'object_is_static': '',
            'object_is_moving_toward': '',
            'object_is_moving_away': '',
            'object_is_in_path': '',
            'object_ttc': '',
            'object_distance_source': '',
            'objects_in_drivable_area': 0,
            'objects_near_lanes': 0,
            'has_pedestrian': 0,
            'has_vehicle': 0
        }
        
        try:
            # Guardar objetos en JSON
            obj_filename = f"{frame_id}_obj.json"
            obj_path = self.current_session_dir / "objects" / obj_filename
            
            objects_data = []
            class_names = []
            confidences = []
            distances = []
            lateral_offsets = []
            velocities_x = []
            velocities_y = []
            speeds = []
            relative_speeds = []
            is_static_list = []
            is_moving_toward_list = []
            is_moving_away_list = []
            is_in_path_list = []
            ttc_list = []
            distance_source_list = []
            
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
                    },
                    'velocity': {
                        'x': float(obj.velocity_x),
                        'y': float(obj.velocity_y),
                        'speed': float(obj.speed),
                        'relative_speed': float(obj.relative_speed)
                    },
                    'state': {
                        'is_static': bool(obj.is_static),
                        'is_moving_toward': bool(obj.is_moving_toward),
                        'is_moving_away': bool(obj.is_moving_away),
                        'is_in_path': bool(obj.is_in_path),
                        'time_to_collision': float(obj.time_to_collision)
                    },
                    'tracking': {
                        'track_age': int(obj.track_age),
                        'is_lost': bool(obj.is_lost),
                        'quality_score': float(obj.quality_score)
                    }
                }
                objects_data.append(obj_dict)
                
                # Actualizar resumen de clases
                class_name = obj.class_name
                if class_name not in self.object_classes_summary:
                    self.object_classes_summary[class_name] = {
                        'count': 0,
                        'total_confidence': 0.0,
                        'total_distance': 0.0,
                        'static_count': 0,
                        'moving_count': 0
                    }
                self.object_classes_summary[class_name]['count'] += 1
                self.object_classes_summary[class_name]['total_confidence'] += obj.confidence
                if obj.distance_valid:
                    self.object_classes_summary[class_name]['total_distance'] += obj.distance
                
                if obj.is_static:
                    self.object_classes_summary[class_name]['static_count'] += 1
                else:
                    self.object_classes_summary[class_name]['moving_count'] += 1
                
                # Clasificar por tipo
                if 'person' in class_name.lower() or 'pedestrian' in class_name.lower():
                    stats['has_pedestrian'] = 1
                elif any(v in class_name.lower() for v in ['car', 'truck', 'bus', 'vehicle']):
                    stats['has_vehicle'] = 1
                
                # Acumular datos para CSV
                class_names.append(class_name)
                confidences.append(f"{obj.confidence:.3f}")
                distances.append(f"{obj.distance:.2f}")
                lateral_offsets.append(f"{obj.lateral_offset:.2f}")
                velocities_x.append(f"{obj.velocity_x:.2f}")
                velocities_y.append(f"{obj.velocity_y:.2f}")
                speeds.append(f"{obj.speed:.2f}")
                relative_speeds.append(f"{obj.relative_speed:.2f}")
                is_static_list.append("1" if obj.is_static else "0")
                is_moving_toward_list.append("1" if obj.is_moving_toward else "0")
                is_moving_away_list.append("1" if obj.is_moving_away else "0")
                is_in_path_list.append("1" if obj.is_in_path else "0")
                ttc_list.append(f"{obj.time_to_collision:.1f}")
                distance_source_list.append(str(obj.distance_source))
            
            # Guardar JSON
            with open(obj_path, 'w') as f:
                json.dump({
                    'timestamp': time.time(),
                    'frame_id': frame_id,
                    'robot_speed': float(self.current_objects.robot_speed),
                    'processing_time_ms': float(self.current_objects.processing_time_ms),
                    'num_static_objects': int(self.current_objects.num_static_objects),
                    'num_moving_objects': int(self.current_objects.num_moving_objects),
                    'objects': objects_data
                }, f, indent=2)
            
            stats['obj_filename'] = obj_filename
            stats['num_objects'] = len(objects_data)
            stats['object_classes'] = ','.join(class_names)
            stats['object_confidences'] = ','.join(confidences)
            stats['object_distances'] = ','.join(distances)
            stats['object_lateral_offsets'] = ','.join(lateral_offsets)
            stats['object_velocities_x'] = ','.join(velocities_x)
            stats['object_velocities_y'] = ','.join(velocities_y)
            stats['object_speeds'] = ','.join(speeds)
            stats['object_relative_speeds'] = ','.join(relative_speeds)
            stats['object_is_static'] = ','.join(is_static_list)
            stats['object_is_moving_toward'] = ','.join(is_moving_toward_list)
            stats['object_is_moving_away'] = ','.join(is_moving_away_list)
            stats['object_is_in_path'] = ','.join(is_in_path_list)
            stats['object_ttc'] = ','.join(ttc_list)
            stats['object_distance_source'] = ','.join(distance_source_list)
            
            # Análisis de relación con segmentación
            if self.current_segmentation is not None:
                stats.update(self._analyze_objects_segmentation_relation(objects_data))
            
        except Exception as e:
            #self.get_logger().error(f'Error procesando objetos: {e}')
            pass
        
        return stats

    def _process_depth(self, frame_id):
        """Procesa y guarda datos de profundidad (sin cambios)"""
        stats = {
            'depth_filename': '',
            'depth_vis_filename': '',
            'avg_depth': 0.0,
            'min_depth': 0.0,
            'max_depth': 0.0
        }
        
        try:
            # 1. Guardar mapa de profundidad crudo (PNG 16-bit)
            depth_filename = f"{frame_id}_depth.png"
            depth_path = self.current_session_dir / "depth" / depth_filename
            cv2.imwrite(str(depth_path), self.current_depth_frame)
            stats['depth_filename'] = depth_filename
            
            # 2. Calcular estadísticas de profundidad
            depth_valid = self.current_depth_frame[self.current_depth_frame > 0]
            if len(depth_valid) > 0:
                # Convertir a metros si está en milímetros
                if np.max(depth_valid) > 1000:  # Asume milímetros si valores > 1000
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
            #self.get_logger().error(f'Error procesando profundidad: {e}')
            pass
        
        return stats

    def _create_depth_visualization(self, depth_frame):
        """Crea una visualización en color del mapa de profundidad (sin cambios)"""
        # Crear copia para visualización
        depth_vis = depth_frame.copy().astype(np.float32)
        
        # Normalizar para visualización
        depth_valid = depth_vis[depth_vis > 0]
        if len(depth_valid) > 0:
            # Escalar a rango 0-255
            if np.max(depth_valid) > 1000:  # Si está en milímetros, convertir a metros
                depth_vis[depth_vis > 0] = depth_vis[depth_vis > 0] / 1000.0
            
            # Aplicar colormap (jet es bueno para profundidad)
            depth_normalized = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX)
            depth_normalized = np.nan_to_num(depth_normalized, nan=0.0)
            depth_normalized = depth_normalized.astype(np.uint8)
            
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # Marcar píxeles sin datos (valor 0) en negro
            depth_colored[depth_frame == 0] = [0, 0, 0]
            
            return depth_colored
        else:
            # Si no hay datos de profundidad válidos, crear imagen negra
            return np.zeros((depth_frame.shape[0], depth_frame.shape[1], 3), dtype=np.uint8)

    def _analyze_objects_segmentation_relation(self, objects_data):
        """Analiza relación entre objetos y segmentación (modificado)"""
        analysis = {
            'objects_in_drivable_area': 0,
            'objects_near_lanes': 0
        }
        
        try:
            height, width = self.current_segmentation.shape
            
            for obj in objects_data:
                center_x, center_y = map(int, obj['center'])
                
                # Verificar si centro está en área transitable
                if (0 <= center_y < height and 0 <= center_x < width):
                    
                    # En área transitable?
                    if self.current_segmentation[center_y, center_x] == 1:
                        analysis['objects_in_drivable_area'] += 1
                    
                    # Cerca de carriles? Buscar en un radio de 20 píxeles
                    radius = 20
                    y_start = max(0, center_y - radius)
                    y_end = min(height, center_y + radius)
                    x_start = max(0, center_x - radius)
                    x_end = min(width, center_x + radius)
                    
                    region = self.current_segmentation[y_start:y_end, x_start:x_end]
                    if np.any(region == 2):
                        analysis['objects_near_lanes'] += 1
                        
        except Exception as e:
            #self.get_logger().debug(f'Error en análisis objetos-segmentación: {e}')
            pass
        
        return analysis

    def _save_frame_to_csv(self, frame_data):
        """Guarda datos del frame en CSV (modificado)"""
        if not self.csv_writer:
            return
        
        try:
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
                frame_data['object_velocities_x'],
                frame_data['object_velocities_y'],
                frame_data['object_speeds'],
                frame_data['object_relative_speeds'],
                frame_data['object_is_static'],
                frame_data['object_is_moving_toward'],
                frame_data['object_is_moving_away'],
                frame_data['object_is_in_path'],
                frame_data['object_ttc'],
                frame_data['object_distance_source'],
                # Información combinada
                frame_data['objects_in_drivable_area'],
                frame_data['objects_near_lanes'],
                frame_data['has_pedestrian'],
                frame_data['has_vehicle'],
                # Imagen RGB
                frame_data['image_filename'],
                # Profundidad
                frame_data['depth_filename'],
                frame_data['depth_vis_filename'],
                frame_data['avg_depth'],
                frame_data['min_depth'],
                frame_data['max_depth'],
                # Información del sistema
                frame_data['robot_speed'],
                frame_data['processing_time_ms']
            ]
            
            self.csv_writer.writerow(row)
            
            # Flush cada 10 filas para equilibrio performance/robustez
            if len(self.stats_buffer) % 10 == 0:
                self.csv_file.flush()
            
        except Exception as e:
            #self.get_logger().error(f'Error guardando en CSV: {e}')
            pass

    def _flush_stats_buffer(self):
        """Actualiza resumen de clases en JSON y limpia buffer (modificado)"""
        try:
            # Guardar resumen de clases
            if self.object_classes_summary:
                summary_data = {
                    'timestamp': time.time(),
                    'total_frames_processed': self.frame_count,
                    'frames_saved': self.saved_frames,
                    'save_frequency': self.save_every_n,
                    'save_depth': self.save_depth,
                    'robot_speed_stats': {
                        'min': float('inf'),
                        'max': 0.0,
                        'avg': 0.0
                    },
                    'classes_detected': {}
                }
                
                # Calcular estadísticas de velocidad del robot
                speeds = [frame.get('robot_speed', 0.0) for frame in self.stats_buffer if 'robot_speed' in frame]
                if speeds:
                    summary_data['robot_speed_stats']['min'] = float(min(speeds))
                    summary_data['robot_speed_stats']['max'] = float(max(speeds))
                    summary_data['robot_speed_stats']['avg'] = float(np.mean(speeds))
                
                for class_name, stats in self.object_classes_summary.items():
                    avg_conf = stats['total_confidence'] / stats['count'] if stats['count'] > 0 else 0
                    avg_dist = stats['total_distance'] / stats['count'] if stats['count'] > 0 and stats['total_distance'] > 0 else 0
                    summary_data['classes_detected'][class_name] = {
                        'count': stats['count'],
                        'average_confidence': avg_conf,
                        'average_distance': avg_dist,
                        'static_count': stats['static_count'],
                        'moving_count': stats['moving_count'],
                        'detection_frequency': (stats['count'] / self.saved_frames) if self.saved_frames > 0 else 0
                    }
                
                with open(self.class_metadata_path, 'w') as f:
                    json.dump(summary_data, f, indent=2)
            
            # Limpiar buffer
            self.stats_buffer.clear()
                    
        except Exception as e:
            #self.get_logger().error(f'Error actualizando resumen: {e}')
            pass

    def _stop_logging(self):
        """Finaliza logging y genera resumen (modificado)"""
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
                #self.get_logger().info(f' Datos de percepción guardados: {self.saved_frames} frames')
            except Exception as e:
                #self.get_logger().error(f'Error cerrando CSV: {e}')
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

    def _generate_session_summary(self):
        """Genera archivo de resumen de la sesión (modificado)"""
        try:
            summary_path = self.current_session_dir / "session_summary.txt"
            
            # Contar archivos generados
            seg_dir = self.current_session_dir / "segmentation"
            obj_dir = self.current_session_dir / "objects"  # Cambiado
            img_dir = self.current_session_dir / "images"
            depth_dir = self.current_session_dir / "depth" if self.save_depth else None
            depth_vis_dir = self.current_session_dir / "depth_vis" if self.save_depth else None
            
            seg_count = len(list(seg_dir.glob("*.png"))) if seg_dir.exists() else 0
            obj_count = len(list(obj_dir.glob("*.json"))) if obj_dir.exists() else 0  # Cambiado
            img_count = len(list(img_dir.glob("*.jpg"))) if img_dir.exists() else 0
            depth_count = len(list(depth_dir.glob("*.png"))) if depth_dir and depth_dir.exists() else 0
            depth_vis_count = len(list(depth_vis_dir.glob("*.jpg"))) if depth_vis_dir and depth_vis_dir.exists() else 0
            
            # Calcular uso de espacio aproximado
            seg_size_mb = seg_count * 0.08  # 80KB por máscara
            obj_size_mb = obj_count * 0.02  # 20KB por JSON de objeto fusionado
            img_size_mb = img_count * 0.15  # 150KB por imagen RGB
            depth_size_mb = depth_count * 0.15  # 150KB por imagen de profundidad (PNG 16-bit)
            depth_vis_size_mb = depth_vis_count * 0.1  # 100KB por visualización de profundidad
            csv_size_mb = 0.002  # ~2KB por 1000 filas (más datos)
            
            total_mb = (seg_size_mb + obj_size_mb + img_size_mb + 
                       depth_size_mb + depth_vis_size_mb + csv_size_mb)
            
            # Leer resumen de clases si existe
            class_summary_path = self.current_session_dir / "object_classes.json"
            class_stats = ""
            if class_summary_path.exists():
                try:
                    with open(class_summary_path, 'r') as f:
                        class_data = json.load(f)
                    
                    for class_name, stats in class_data.get('classes_detected', {}).items():
                        class_stats += f"    {class_name}: {stats['count']} (estáticos: {stats['static_count']}, móviles: {stats['moving_count']})\n"
                except:
                    class_stats = "    Error leyendo estadísticas de clases\n"
            
            with open(summary_path, 'w') as f:
                f.write("=== RESUMEN DE SESIÓN DE PERCEPCIÓN ===\n\n")
                f.write(f"Fecha: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Nodo: {self.get_name()}\n")
                f.write(f"Directorio: {self.current_session_dir}\n\n")
                
                f.write("CONFIGURACIÓN:\n")
                f.write(f"  Frecuencia guardado: 1 cada {self.save_every_n} frames\n")
                f.write(f"  Profundidad guardada: {'Sí' if self.save_depth else 'No'}\n")
                f.write(f"  Tasa efectiva: {30/self.save_every_n:.1f} FPS (de 30 FPS original)\n")
                f.write(f"  Fuente de objetos: /objects/fused_info\n\n")
                
                f.write("ESTADÍSTICAS:\n")
                f.write(f"  Frames totales procesados: {self.frame_count}\n")
                f.write(f"  Frames guardados: {self.saved_frames}\n\n")
                
                f.write("ARCHIVOS GUARDADOS:\n")
                f.write(f"  Máscaras de segmentación: {seg_count}\n")
                f.write(f"  Archivos de objetos fusionados: {obj_count}\n")  # Cambiado
                f.write(f"  Imágenes RGB: {img_count}\n")
                if self.save_depth:
                    f.write(f"  Mapas de profundidad: {depth_count}\n")
                    f.write(f"  Visualizaciones de profundidad: {depth_vis_count}\n")
                f.write("\n")
                
                f.write("USO DE ESPACIO ESTIMADO:\n")
                f.write(f"  Segmentación: {seg_size_mb:.1f} MB\n")
                f.write(f"  Objetos fusionados: {obj_size_mb:.1f} MB\n")  # Cambiado
                f.write(f"  Imágenes RGB: {img_size_mb:.1f} MB\n")
                if self.save_depth:
                    f.write(f"  Profundidad: {depth_size_mb:.1f} MB\n")
                    f.write(f"  Visualizaciones profundidad: {depth_vis_size_mb:.1f} MB\n")
                f.write(f"  CSV/JSON: {csv_size_mb:.1f} MB\n")
                f.write(f"  TOTAL: {total_mb:.1f} MB\n\n")
                
                f.write("ESTADÍSTICAS DE OBJETOS:\n")
                if class_stats:
                    f.write(class_stats)
                else:
                    f.write("    No hay datos disponibles\n")
                f.write("\n")
                
                f.write("ARCHIVOS PRINCIPALES:\n")
                f.write(f"  perception_data.csv - Datos combinados por frame\n")
                f.write(f"  object_classes.json - Resumen de clases detectadas\n")
                f.write(f"  session_summary.txt - Este archivo\n")
                f.write(f"  segmentation/ - Máscaras PNG comprimidas\n")
                f.write(f"  objects/ - JSONs con información 3D fusionada\n")  # Cambiado
                f.write(f"  images/ - Imágenes RGB JPG (85% calidad)\n")
                if self.save_depth:
                    f.write(f"  depth/ - Mapas de profundidad PNG (16-bit)\n")
                    f.write(f"  depth_vis/ - Visualizaciones de profundidad JPG\n")
            
            #self.get_logger().info(f' Resumen generado: {summary_path}')
            #self.get_logger().info(f' Total frames guardados: {self.saved_frames}')
            
        except Exception as e:
            #self.get_logger().error(f'Error generando resumen: {e}')
            pass

    def destroy_node(self):
        self._stop_logging()
        super().destroy_node()

def main():
    rclpy.init()
    node = PerceptionRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info('Interrupción por teclado')
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()