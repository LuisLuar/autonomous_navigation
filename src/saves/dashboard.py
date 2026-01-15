#!/usr/bin/env python3
"""
Dashboard para visualización completa de datos de navegación autónoma.
Versión adaptada a la estructura de saves/data_logs/
"""

import cv2
import numpy as np
import pandas as pd
import json
import glob
import os
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib
matplotlib.use('Agg')  # Para usar matplotlib sin GUI

class NavigationDashboard:
    def __init__(self, session_path):
        """
        Inicializa el dashboard con la ruta de la sesión.
        
        Args:
            session_path: Ruta a la carpeta de la sesión 
                         (ej: '~/autonomous_navigation/src/saves/data_logs/ruta6_20260106_134920')
        """

        self.session_path = Path(session_path)
        self.current_frame = 0
        self.paused = False
        self.play_speed = 1.0  # Velocidad de reproducción
        self.speed_steps = [0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0]
        self.show_detections = True
        self.show_segmentation = True
        self.show_trajectory = True
        
        # Inicializar atributos de datos (aunque se cargarán después)
        self.frames_list = []
        self.perception_df = None
        self.raw_sensors = None
        self.global_path = None
        self.ekf_local = None
        self.ekf_global = None
        self.control_data = None
        self.metadata = {}

        self.debug_use_video = True
        self.video_tail_cut = 300
        self.video_cap = None
        self.video_frame_idx = 0

        # Cargar todos los datos disponibles
        self.load_data()
        
        # Cargar todos los datos disponibles
        self.load_data()
        
        # Configurar ventana
        self.window_name = "Navigation Dashboard"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1920, 1080)
        
        # Configuración de colores
        self.colors = {
            'lane': (0, 255, 255),      # Amarillo para carriles
            'drivable': (0, 200, 0),    # Verde para área transitable
            'person': (0, 0, 255),      # Rojo para personas
            'car': (255, 0, 0),         # Azul para coches
            'truck': (0, 165, 255),     # Naranja para camiones
            'bus': (255, 128, 0),       # Naranja oscuro para buses
            'path': (255, 255, 0),      # Amarillo para trayectoria
            'current': (0, 255, 0),     # Verde para posición actual
            'grid': (50, 50, 50),       # Gris para grid
            'text': (255, 255, 255),    # Blanco para texto
            'warning': (0, 200, 255),   # Naranja para warnings
            'error': (0, 0, 255),       # Rojo para errores
            'background': (20, 20, 20)  # Gris oscuro para fondo
        }
        
        print(f" Dashboard inicializado para sesión: {session_path}")
        print(f" Total frames disponibles: {len(self.frames_list)}")
        print("\n Controles:")
        print("  [SPACE] - Play/Pause")
        print("  [→] - Frame siguiente")
        print("  [←] - Frame anterior")
        print("  [↑] - Aumentar velocidad")
        print("  [↓] - Disminuir velocidad")
        print("  [1] - Toggle detecciones")
        print("  [2] - Toggle segmentación")
        print("  [3] - Toggle trayectoria")
        print("  [R] - Reiniciar")
        print("  [S] - Guardar screenshot")
        print("  [F] - Pantalla completa")
        print("  [Q/ESC] - Salir")
    
    def get_debug_video_frame(self, frame_idx, total_frames):
        """
        Devuelve un frame de video aproximado al frame lógico actual.
        NO sincroniza timestamps.
        SOLO visual.
        """
        if self.video_cap is None or self.video_total_frames == 0:
            return None

        # Clamp defensivo
        frame_idx = max(0, min(frame_idx, total_frames - 1))

        video_idx = int(
            (frame_idx / max(1, total_frames - 1)) * (self.video_effective_frames - 1)
        )

        video_idx = max(0, min(video_idx, self.video_effective_frames - 1))
        self.video_cap.set(cv2.CAP_PROP_POS_FRAMES, video_idx)
        ret, frame = self.video_cap.read()

        return frame if ret else None




    def load_data(self):
        """Carga todos los datos de la sesión."""
        print(" Cargando datos...")
        
        # 1. Cargar lista de frames disponibles desde perception/
        perception_path = self.session_path / "perception"
        self.frames_list = []
        
        # Buscar imágenes RGB
        images_dir = perception_path / "images"
        if images_dir.exists():
            image_files = sorted(glob.glob(str(images_dir / "*.jpg")))
            print(f"  Encontradas {len(image_files)} imágenes en {images_dir}")
            
            for img_file in image_files:
                try:
                    frame_id = Path(img_file).stem.replace("_image", "")
                    
                    # Construir rutas
                    seg_file = str(perception_path / "segmentation" / f"{frame_id}_seg.png")
                    det_file = str(perception_path / "detections" / f"{frame_id}_det.json")
                    
                    # Verificar que existan los archivos
                    if not os.path.exists(seg_file):
                        seg_file = None
                    if not os.path.exists(det_file):
                        det_file = None
                    
                    self.frames_list.append({
                        'id': frame_id,
                        'image': img_file,
                        'segmentation': seg_file,
                        'detections': det_file
                    })
                    self.total_frames = len(self.frames_list)

                except Exception as e:
                    print(f"    Error procesando archivo {img_file}: {e}")
            
            print(f"   Imágenes cargadas: {len(self.frames_list)} frames")
        else:
            print(f"   No se encontró el directorio: {images_dir}")
        
        # 2. Cargar CSV de percepción (¡ESTO ES IMPORTANTE!)
        self.perception_df = None
        csv_path = perception_path / "perception_data.csv"
        if csv_path.exists():
            self.perception_df = pd.read_csv(csv_path)
            print(f"   CSV percepción cargado: {len(self.perception_df)} registros")
            
            # Verificar que tenga timestamp
            if 'timestamp' in self.perception_df.columns:
                print(f"   CSV percepción tiene timestamps")
                print(f"   Rango de timestamps: {self.perception_df['timestamp'].min():.2f} a {self.perception_df['timestamp'].max():.2f}")
        else:
            print(f"   No se encontró CSV de percepción: {csv_path}")
        
        # 3. Cargar datos de sensores crudos
        self.raw_sensors = None
        raw_sensors_path = self.session_path / "raw_sensors.csv"
        if raw_sensors_path.exists():
            self.raw_sensors = pd.read_csv(raw_sensors_path)
            print(f"   Sensores crudos cargados: {len(self.raw_sensors)} registros")
        
        # 4. Cargar path global
        self.global_path = None
        # Buscar archivos de path global
        path_files = list(self.session_path.glob("global_path*.csv"))
        if path_files:
            # Preferir el archivo final si existe
            final_path = self.session_path / "global_path_final.csv"
            if final_path.exists():
                self.global_path = pd.read_csv(final_path)
                print(f"   Path global final cargado: {len(self.global_path)} puntos")
            else:
                self.global_path = pd.read_csv(path_files[0])
                print(f"   Path global cargado: {len(self.global_path)} puntos")
        
        # 5. Cargar datos EKF
        self.ekf_local = None
        self.ekf_global = None
        
        ekf_local_path = self.session_path / "ekf_local.csv"
        if ekf_local_path.exists():
            self.ekf_local = pd.read_csv(ekf_local_path)
            print(f"   EKF local cargado: {len(self.ekf_local)} registros")
        
        ekf_global_path = self.session_path / "ekf_global.csv"
        if ekf_global_path.exists():
            self.ekf_global = pd.read_csv(ekf_global_path)
            print(f"  EKF global cargado: {len(self.ekf_global)} registros")
        
        # 6. Cargar datos de control
        self.control_data = None
        control_path = self.session_path / "control_signals.csv"
        if control_path.exists():
            self.control_data = pd.read_csv(control_path)
            print(f"   Datos de control cargados: {len(self.control_data)} registros")
            
            # Verificar si tiene timestamp
            if 'timestamp' in self.control_data.columns:
                print(f"   Control data tiene timestamps")
                print(f"   Rango de timestamps control: {self.control_data['timestamp'].min():.2f} a {self.control_data['timestamp'].max():.2f}")
        
        # 7. Cargar metadatos
        self.metadata = {}
        metadata_path = self.session_path / "metadata.txt"
        if metadata_path.exists():
            with open(metadata_path, 'r') as f:
                for line in f:
                    if ':' in line:
                        key, value = line.strip().split(':', 1)
                        self.metadata[key.strip()] = value.strip()
            print(f"   Metadatos cargados")
        
        # 8. Carga de video para debug
        video_path = os.path.join(self.session_path, "robot.mp4")

        if os.path.exists(video_path):
            self.video_cap = cv2.VideoCapture(video_path)
            self.video_total_frames = int(self.video_cap.get(cv2.CAP_PROP_FRAME_COUNT))
            self.video_effective_frames = max(1,self.video_total_frames - self.video_tail_cut)

            print(f" Video debug cargado ({self.video_total_frames} frames) ({self.video_effective_frames} frames efectivos)")
        else:
            self.video_cap = None

        print(f" Carga de datos completada")
        
        # 8. Analizar sincronización
        self.analyze_synchronization()
    
    def get_frame_data(self, frame_idx):
        """Obtiene todos los datos para un frame específico usando timestamps."""
        # Asegurar que frame_idx sea entero
        frame_idx = int(frame_idx)
        
        if frame_idx < 0 or frame_idx >= len(self.frames_list):
            return None
        
        frame_info = self.frames_list[frame_idx]
        data = {'frame_id': frame_info['id'], 'frame_idx': frame_idx}
        
        # DEBUG: Mostrar información del frame
        # print(f" Frame {frame_idx}: {frame_info['id']}")
        
        # 1. Cargar imagen RGB
        if frame_info['image'] and os.path.exists(frame_info['image']):
            data['image'] = cv2.imread(frame_info['image'])
        
        # 2. Cargar segmentación (si existe)
        if frame_info['segmentation'] and os.path.exists(frame_info['segmentation']):
            seg_mask = cv2.imread(frame_info['segmentation'], cv2.IMREAD_GRAYSCALE)
            if seg_mask is not None:
                data['segmentation'] = seg_mask
        
        # En load_data(), modificar la carga de detecciones:

        # 3. Cargar detecciones (ambos formatos)
        if frame_info['detections'] and os.path.exists(frame_info['detections']):
            try:
                with open(frame_info['detections'], 'r') as f:
                    det_data = json.load(f)
                    
                    # Compatibilidad con ambos formatos
                    if 'objects' in det_data:  # Nuevo formato fusionado
                        data['detections'] = det_data
                        data['detection_type'] = 'fused_3d'
                    elif 'detections' in det_data:  # Formato antiguo
                        data['detections'] = det_data
                        data['detection_type'] = 'yolo_detections'
                    else:
                        # Formato desconocido, tratar como lista directa
                        data['detections'] = {'detections': det_data}
                        data['detection_type'] = 'unknown'
                        
            except Exception as e:
                print(f"  Error cargando detecciones: {e}")
        
        # 4. Obtener datos del CSV de percepción (incluyendo timestamp)
        if self.perception_df is not None:
            # Buscar por frame_id
            frame_row = self.perception_df[self.perception_df['frame_id'] == frame_info['id']]
            if not frame_row.empty:
                data['perception_stats'] = frame_row.iloc[0].to_dict()
                
                # Guardar el timestamp de la percepción
                if 'timestamp' in data['perception_stats']:
                    data['perception_timestamp'] = data['perception_stats']['timestamp']
                    # print(f"   Timestamp percepción: {data['perception_timestamp']}")
        
        # 5. Obtener datos de control correspondientes (¡SINCRONIZACIÓN POR TIMESTAMP!)
        if self.control_data is not None and 'timestamp' in self.control_data.columns:
            if 'perception_timestamp' in data:
                # Tenemos timestamp de la percepción, buscar el control más cercano
                perception_time = data['perception_timestamp']
                
                # Buscar el índice del control con timestamp más cercano
                time_diffs = abs(self.control_data['timestamp'] - perception_time)
                closest_idx = time_diffs.idxmin()
                min_diff = time_diffs.min()
                
                # Sólo usar si la diferencia es razonable (ej. < 0.1 segundos)
                if min_diff < 0.1:  # 100 ms de tolerancia
                    control_idx = closest_idx
                    data['control_stats'] = self.control_data.iloc[control_idx].to_dict()
                    data['time_diff'] = min_diff
                    
                    # DEBUG: Mostrar sincronización
                    # print(f"   Sincronizado con control idx {control_idx}")
                    # print(f"   Diferencia temporal: {min_diff:.3f} s")
                else:
                    # Si la diferencia es muy grande, usar índice proporcional
                    print(f"  Gran diferencia temporal: {min_diff:.3f} s")
                    control_idx = self.get_proportional_control_idx(frame_idx)
                    data['control_stats'] = self.control_data.iloc[control_idx].to_dict()
            else:
                # No hay timestamp de percepción, usar índice proporcional
                control_idx = self.get_proportional_control_idx(frame_idx)
                data['control_stats'] = self.control_data.iloc[control_idx].to_dict()
        else:
            # No hay timestamps en control data, usar índice simple
            if self.control_data is not None and frame_idx < len(self.control_data):
                data['control_stats'] = self.control_data.iloc[frame_idx].to_dict()
        
        # 6. Obtener datos EKF sincronizados (¡AGREGAR ESTO!)
        if self.ekf_global is not None and 'timestamp' in self.ekf_global.columns:
            if 'perception_timestamp' in data:
                perception_time = data['perception_timestamp']
                
                # Buscar el punto EKF más cercano en tiempo
                time_diffs = abs(self.ekf_global['timestamp'] - perception_time)
                closest_idx = time_diffs.idxmin()
                min_diff = time_diffs.min()
                
                if min_diff < 0.2:  # 200 ms de tolerancia
                    data['ekf_data'] = self.ekf_global.iloc[closest_idx].to_dict()
                    data['ekf_time_diff'] = min_diff
                    data['ekf_idx'] = closest_idx
        
        # DEBUG: Mostrar velocidad si está disponible
        if 'control_stats' in data and 'cmd_vel_linear_x' in data['control_stats']:
            speed = data['control_stats']['cmd_vel_linear_x']
            # print(f"   Velocidad: {speed:.3f} m/s")
        
        return data

    def get_proportional_control_idx(self, frame_idx):
        """Obtiene índice proporcional en datos de control basado en relación de conteos."""
        if self.control_data is None:
            return 0
        
        frames_count = len(self.frames_list)
        control_count = len(self.control_data)
        
        if frames_count == 0 or control_count == 0:
            return 0
        
        # Calcular índice proporcional
        proportion = frame_idx / max(1, frames_count - 1)
        control_idx = int(proportion * (control_count - 1))
        
        # Asegurar que esté dentro de los límites
        control_idx = max(0, min(control_idx, control_count - 1))
        
        return control_idx

    def analyze_synchronization(self):
        """Analiza la sincronización entre imágenes, percepción y control."""
        print(f"\n ANÁLISIS DE SINCRONIZACIÓN:")
        
        # Contar elementos
        frames_count = len(self.frames_list)
        perception_count = len(self.perception_df) if self.perception_df is not None else 0
        control_count = len(self.control_data) if self.control_data is not None else 0
        
        print(f"  • Imágenes: {frames_count}")
        print(f"  • Registros percepción: {perception_count}")
        print(f"  • Registros control: {control_count}")
        
        # Verificar timestamps
        has_perception_timestamps = self.perception_df is not None and 'timestamp' in self.perception_df.columns
        has_control_timestamps = self.control_data is not None and 'timestamp' in self.control_data.columns
        
        print(f"  • Timestamps percepción: {'SÍ' if has_perception_timestamps else 'NO'}")
        print(f"  • Timestamps control: {'SÍ' if has_control_timestamps else 'NO'}")
        
        # Mostrar rangos de tiempo si existen
        if has_perception_timestamps:
            perception_times = self.perception_df['timestamp']
            print(f"  • Rango percepción: {perception_times.min():.2f} a {perception_times.max():.2f}")
            print(f"  • Duración percepción: {perception_times.max() - perception_times.min():.2f} s")
        
        if has_control_timestamps:
            control_times = self.control_data['timestamp']
            print(f"  • Rango control: {control_times.min():.2f} a {control_times.max():.2f}")
            print(f"  • Duración control: {control_times.max() - control_times.min():.2f} s")
        
        # Verificar sincronización para algunos frames de ejemplo
        if frames_count > 0 and perception_count > 0 and control_count > 0:
            print(f"\n EJEMPLOS DE SINCRONIZACIÓN (primeros 5 frames):")
            print(f"{'Frame':<6} {'Img ID':<12} {'Percepción':<12} {'Control':<10} {'Speed':<8} {'TimeDiff':<8}")
            print("-" * 60)
            
            for i in range(min(5, frames_count)):
                # Obtener datos del frame
                frame_data = self.get_frame_data_sync_analysis(i)
                
                frame_id = frame_data.get('frame_id', 'N/A')
                percep_time = frame_data.get('perception_timestamp', 'N/A')
                control_idx = frame_data.get('control_idx', 'N/A')
                speed = frame_data.get('speed', 'N/A')
                time_diff = frame_data.get('time_diff', 'N/A')
                
                print(f"{i:<6} {frame_id:<12} {str(percep_time)[:10]:<12} {str(control_idx)[:8]:<10} {str(speed)[:6]:<8} {str(time_diff)[:6]:<8}")

    def get_frame_data_sync_analysis(self, frame_idx):
        """Versión simplificada para análisis de sincronización."""
        if frame_idx < 0 or frame_idx >= len(self.frames_list):
            return {}
        
        frame_info = self.frames_list[frame_idx]
        result = {'frame_id': frame_info['id'], 'frame_idx': frame_idx}
        
        # Obtener timestamp de percepción
        if self.perception_df is not None:
            frame_row = self.perception_df[self.perception_df['frame_id'] == frame_info['id']]
            if not frame_row.empty and 'timestamp' in frame_row.columns:
                result['perception_timestamp'] = frame_row.iloc[0]['timestamp']
        
        # Obtener datos de control sincronizados
        if self.control_data is not None and 'timestamp' in self.control_data.columns:
            if 'perception_timestamp' in result:
                perception_time = result['perception_timestamp']
                
                # Buscar el control más cercano
                time_diffs = abs(self.control_data['timestamp'] - perception_time)
                closest_idx = time_diffs.idxmin()
                min_diff = time_diffs.min()
                
                if min_diff < 0.1:
                    result['control_idx'] = closest_idx
                    result['time_diff'] = min_diff
                    
                    # Obtener velocidad
                    control_row = self.control_data.iloc[closest_idx]
                    if 'cmd_vel_linear_x' in control_row:
                        result['speed'] = control_row['cmd_vel_linear_x']
        
        return result
    
    def get_ekf_synchronized_data(self, perception_timestamp):
        """Obtiene datos EKF sincronizados por timestamp."""
        ekf_data = None
        
        # Usar EKF global si está disponible, sino local
        if self.ekf_global is not None and 'timestamp' in self.ekf_global.columns:
            ekf_data = self.ekf_global
        elif self.ekf_local is not None and 'timestamp' in self.ekf_local.columns:
            ekf_data = self.ekf_local
        else:
            return None
        
        # Buscar el punto EKF más cercano en tiempo
        time_diffs = abs(ekf_data['timestamp'] - perception_timestamp)
        closest_idx = time_diffs.idxmin()
        min_diff = time_diffs.min()
        
        # Si la diferencia es razonable (ej. < 0.2 segundos), usar esos datos
        if min_diff < 0.2:  # 200 ms de tolerancia
            return {
                'idx': closest_idx,
                'data': ekf_data.iloc[closest_idx],
                'time_diff': min_diff
            }
        
        return None
    
    def create_detection_overlay(self, image, detections_data, frame_stats=None):
        """Crea overlay de detecciones con información 3D, tracking y velocidades calculadas."""
        if image is None or not detections_data:
            return image
        
        overlay = image.copy()
        height, width = image.shape[:2]
        
        # Obtener velocidad del robot si está disponible
        robot_speed = 0.0
        if frame_stats and 'control_stats' in frame_stats:
            robot_speed = frame_stats['control_stats'].get('cmd_vel_linear_x', 0.0)
        
        # Verificar estructura de datos
        if isinstance(detections_data, dict):
            if 'objects' in detections_data:  # Nuevo formato fusionado
                detections_list = detections_data['objects']
                source_text = f"FUSED 3D"
                robot_speed_data = detections_data.get('robot_speed', robot_speed)
                if robot_speed_data > 0:
                    robot_speed = robot_speed_data
            elif 'detections' in detections_data:  # Formato antiguo
                detections_list = detections_data['detections']
                source_text = f"DETECTIONS"
            else:
                detections_list = []
                source_text = ""
        elif isinstance(detections_data, list):
            detections_list = detections_data
            source_text = ""
        else:
            return image
        
        if not detections_list:
            return image
        
        # Contadores y estadísticas
        type_counts = {
            'person': 0,
            'vehicle': 0,
            'other': 0
        }
        
        # Estadísticas de tracking
        tracking_stats = {
            'total_objects': 0,
            'static_objects': 0,
            'moving_toward': 0,
            'moving_away': 0,
            'in_path': 0,
            'danger_objects': 0  # TTC bajo
        }
        
        # Primera pasada: procesar todos los objetos
        processed_objects = []
        for det in detections_list:
            if not isinstance(det, dict):
                continue
            
            # Extraer bounding box
            bbox = det.get('bbox', [])
            if len(bbox) != 4:
                # Intentar extraer de campos separados
                bbox = [
                    det.get('x1', det.get('bbox_x1', 0)),
                    det.get('y1', det.get('bbox_y1', 0)),
                    det.get('x2', det.get('bbox_x2', width)),
                    det.get('y2', det.get('bbox_y2', height))
                ]
            
            if len(bbox) != 4:
                continue
            
            # Información básica
            class_name = det.get('class_name', 'unknown')
            confidence = det.get('confidence', 0.0)
            track_id = det.get('track_id', 0)
            
            # Información 3D y tracking
            distance = float(det.get('distance', 0.0))
            lateral_offset = float(det.get('lateral_offset', 0.0))
            speed = float(det.get('speed', 0.0))  # Velocidad absoluta del objeto
            velocity_x = float(det.get('velocity_x', 0.0))  # Velocidad en X (hacia/desde robot)
            velocity_y = float(det.get('velocity_y', 0.0))  # Velocidad lateral
            relative_speed = float(det.get('relative_speed', 0.0))  # Velocidad relativa YA CALCULADA
            ground_distance = float(det.get('ground_distance', 0.0))
            
            # Estados y flags
            is_static = bool(det.get('is_static', False))
            is_moving_toward = bool(det.get('is_moving_toward', False))
            is_moving_away = bool(det.get('is_moving_away', False))
            is_in_path = bool(det.get('is_in_path', False))
            ttc = float(det.get('time_to_collision', 999.0))
            
            # Calidad de track
            track_age = int(det.get('track_age', 0))
            quality_score = float(det.get('quality_score', 0.0))
            distance_valid = bool(det.get('distance_valid', False))
            distance_source = int(det.get('distance_source', 0))  # 0=none, 1=depth, 2=ipm, 3=size
            
            # Determinar tipo
            class_lower = class_name.lower()
            if 'person' in class_lower or 'pedestrian' in class_lower:
                obj_type = 'person'
                base_color = self.colors['person']
            elif any(v in class_lower for v in ['car', 'truck', 'bus', 'vehicle']):
                obj_type = 'vehicle'
                base_color = self.colors['car']
            else:
                obj_type = 'other'
                base_color = (128, 128, 128)  # Gris
            
            # Contar tipos
            type_counts[obj_type] += 1
            tracking_stats['total_objects'] += 1
            
            # Contar estados
            if is_static:
                tracking_stats['static_objects'] += 1
            if is_moving_toward:
                tracking_stats['moving_toward'] += 1
            if is_moving_away:
                tracking_stats['moving_away'] += 1
            if is_in_path:
                tracking_stats['in_path'] += 1
            if ttc < 5.0:  # Objetos peligrosos (TTC < 5 segundos)
                tracking_stats['danger_objects'] += 1
            
            # CALCULAR VELOCIDAD RELATIVA REAL (si no está ya calculada)
            # La velocidad relativa ya debería venir calculada en relative_speed,
            # pero por si acaso verificamos:
            if relative_speed == 0 and velocity_x != 0:
                # relative_speed = velocidad_objeto - velocidad_robot (proyectada en dirección al robot)
                # Si velocity_x ya incluye la compensación, usarla directamente
                relative_speed = velocity_x
            
            # Calcular velocidad de aproximación real
            # Positive: alejándose, Negative: acercándose
            approach_speed = relative_speed
            
            # Determinar color de caja según estado y peligro
            if ttc < 2.0:  # CRÍTICO - Rojo parpadeante
                box_color = (0, 0, 255)  # Rojo sólido
                thickness = 3
            elif ttc < 5.0:  # PELIGRO - Naranja
                box_color = (0, 165, 255)  # Naranja
                thickness = 2
            elif is_in_path:  # En trayectoria - Amarillo
                box_color = (0, 255, 255)  # Amarillo
                thickness = 2
            elif is_static:  # Estático - Verde
                box_color = (0, 200, 0)  # Verde
                thickness = 1
            elif approach_speed < -0.3:  # Acercándose rápido - Rojo
                box_color = (0, 0, 200)  # Rojo oscuro
                thickness = 2
            elif approach_speed > 0.3:  # Alejándose rápido - Azul
                box_color = (255, 100, 0)  # Azul (BGR)
                thickness = 2
            else:  # Normal - Color base del tipo
                box_color = base_color
                thickness = 1
            
            # Almacenar objeto procesado
            processed_objects.append({
                'bbox': bbox,
                'class_name': class_name,
                'confidence': confidence,
                'track_id': track_id,
                'distance': distance,
                'lateral_offset': lateral_offset,
                'speed': speed,
                'relative_speed': relative_speed,
                'approach_speed': approach_speed,
                'velocity_x': velocity_x,
                'velocity_y': velocity_y,
                'is_static': is_static,
                'is_moving_toward': is_moving_toward,
                'is_moving_away': is_moving_away,
                'is_in_path': is_in_path,
                'ttc': ttc,
                'track_age': track_age,
                'quality_score': quality_score,
                'distance_valid': distance_valid,
                'distance_source': distance_source,
                'obj_type': obj_type,
                'box_color': box_color,
                'thickness': thickness,
                'ground_distance': ground_distance
            })
        
        # Segunda pasada: dibujar objetos (ordenar por distancia para evitar solapamiento)
        # Ordenar por distancia (más cercanos primero para que se vean mejor)
        processed_objects.sort(key=lambda x: x['distance'] if x['distance'] > 0 else float('inf'))
        
        for obj in processed_objects:
            x1, y1, x2, y2 = map(int, obj['bbox'])
            
            # Dibujar bounding box
            cv2.rectangle(overlay, (x1, y1), (x2, y2), obj['box_color'], obj['thickness'])
            
            # Dibujar esquina de tracking si tiene ID
            if obj['track_id'] > 0:
                # Punto de tracking (esquina superior izquierda)
                cv2.circle(overlay, (x1 + 8, y1 + 8), 6, (255, 255, 255), -1)
                cv2.putText(overlay, f"#{obj['track_id']}", (x1 + 12, y1 + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
                
                # Indicador de edad del track (barra vertical)
                if obj['track_age'] > 0:
                    age_bar_height = min(20, obj['track_age'] * 2)
                    cv2.rectangle(overlay, (x1 - 5, y1), 
                                (x1 - 2, y1 + age_bar_height), 
                                (200, 200, 0), -1)
            
            # ETIQUETA PRINCIPAL (arriba del bbox)
            # Formato: [Tipo] [Conf] | Distancia | V_rel
            main_label = f"{obj['class_name']} {obj['confidence']:.2f}"
            
            # Añadir distancia si es válida
            if obj['distance_valid'] and obj['distance'] > 0:
                # Mostrar fuente de distancia con símbolo
                source_symbol = ""
                if obj['distance_source'] == 1:
                    source_symbol = "📐"  # Depth
                elif obj['distance_source'] == 2:
                    source_symbol = "📏"  # IPM
                elif obj['distance_source'] == 3:
                    source_symbol = "📏"  # Size estimation
                
                main_label += f" | {obj['distance']:.1f}m{source_symbol}"
            
            # Añadir velocidad relativa SI ES SIGNIFICATIVA (> 0.1 m/s)
            if abs(obj['relative_speed']) > 0.1:
                direction_symbol = "←" if obj['relative_speed'] < 0 else "→"
                speed_value = abs(obj['relative_speed'])
                main_label += f" | {direction_symbol}{speed_value:.1f}m/s"
            
            # Dibujar etiqueta principal
            label_size = cv2.getTextSize(main_label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
            
            # Fondo para etiqueta
            cv2.rectangle(overlay, (x1, y1 - label_size[1] - 5),
                        (x1 + label_size[0], y1), obj['box_color'], -1)
            
            # Texto principal
            cv2.putText(overlay, main_label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # ETIQUETA SECUNDARIA (debajo del bbox si hay espacio)
            if y2 + 40 < height:
                secondary_lines = []
                
                # Línea 1: Información de posición y trayectoria
                pos_info = []
                if abs(obj['lateral_offset']) > 0.1:
                    side = "DER" if obj['lateral_offset'] > 0 else "IZQ"
                    pos_info.append(f"{side}:{abs(obj['lateral_offset']):.1f}m")
                
                if obj['ground_distance'] > 0 and obj['distance'] > 0:
                    pos_info.append(f"Total:{obj['ground_distance']:.1f}m")
                
                if pos_info:
                    secondary_lines.append(" ".join(pos_info))
                
                # Línea 2: Información de movimiento y estado
                move_info = []
                
                # Velocidad absoluta si es significativa
                if abs(obj['speed']) > 0.1:
                    move_info.append(f"V:{obj['speed']:.1f}m/s")
                
                # Estados especiales
                if obj['is_static']:
                    move_info.append("ESTÁTICO")
                elif obj['is_moving_toward']:
                    move_info.append("ACERCÁNDOSE")
                elif obj['is_moving_away']:
                    move_info.append("ALEJÁNDOSE")
                
                # En trayectoria
                if obj['is_in_path']:
                    move_info.append("EN TRAYECTORIA")
                
                if move_info:
                    secondary_lines.append(" ".join(move_info))
                
                # Línea 3: Tiempo para colisión y calidad
                safety_info = []
                if obj['ttc'] < 999.0:
                    if obj['ttc'] < 2.0:
                        safety_info.append(f"⚠️TTC:{obj['ttc']:.1f}s")
                    elif obj['ttc'] < 5.0:
                        safety_info.append(f"TTC:{obj['ttc']:.1f}s")
                
                # Calidad de track
                if obj['track_age'] > 10:
                    safety_info.append(f"Track:{obj['track_age']}")
                
                if obj['quality_score'] > 0:
                    safety_info.append(f"Q:{obj['quality_score']:.1f}")
                
                if safety_info:
                    secondary_lines.append(" ".join(safety_info))
                
                # Dibujar líneas secundarias
                line_height = 15
                for i, line in enumerate(secondary_lines):
                    line_y = y2 + 10 + (i * line_height)
                    if line_y + 10 < height:
                        # Fondo semi-transparente para mejor legibilidad
                        text_size = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)[0]
                        overlay_copy = overlay.copy()
                        cv2.rectangle(overlay_copy, (x1, line_y - text_size[1] - 2),
                                    (x1 + text_size[0], line_y + 2), (0, 0, 0), -1)
                        # Mezclar con transparencia
                        overlay = cv2.addWeighted(overlay_copy, 0.6, overlay, 0.4, 0)
                        
                        cv2.putText(overlay, line, (x1, line_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 200), 1)
            
            # LÍNEA DE POSICIÓN 3D (opcional, solo si distancia válida)
            if obj['distance_valid'] and obj['distance'] > 0 and obj['lateral_offset'] != 0:
                center_x = (x1 + x2) // 2
                center_y = y2
                
                # Calcular posición lateral proyectada
                # Asumiendo FOV de ~60 grados y ancho de imagen
                fov_rad = 1.047  # 60 grados en radianes
                focal_px = width / (2 * np.tan(fov_rad / 2))
                
                lateral_px = int((obj['lateral_offset'] / obj['distance']) * focal_px)
                projected_x = center_x + lateral_px
                
                # Dibujar línea de posición 3D
                line_color = (255, 200, 0) if obj['is_in_path'] else (200, 200, 200)
                cv2.line(overlay, (center_x, center_y), (projected_x, center_y), 
                        line_color, 1, cv2.LINE_AA)
                cv2.circle(overlay, (projected_x, center_y), 3, line_color, -1)
                
                # Flecha de dirección del movimiento lateral
                if abs(obj['velocity_y']) > 0.1:
                    arrow_length = int(obj['velocity_y'] * 20)  # Escalar para visualización
                    arrow_end = projected_x + arrow_length
                    arrow_color = (0, 255, 0) if obj['velocity_y'] > 0 else (0, 0, 255)
                    cv2.arrowedLine(overlay, (projected_x, center_y - 10),
                                (arrow_end, center_y - 10), arrow_color, 1, tipLength=0.3)
        
        # PANEL DE RESUMEN (esquina superior derecha)
        summary_x = width - 220
        summary_y = 30
        
        # Fondo del panel
        panel_height = 130
        cv2.rectangle(overlay, (summary_x - 10, summary_y - 25),
                    (width - 10, summary_y + panel_height), (0, 0, 0, 128), -1)
        overlay_copy = overlay.copy()
        cv2.rectangle(overlay_copy, (summary_x - 10, summary_y - 25),
                    (width - 10, summary_y + panel_height), (0, 0, 0), -1)
        overlay = cv2.addWeighted(overlay_copy, 0.5, overlay, 0.5, 0)
        
        # Título del panel
        panel_title = "ESTADÍSTICAS DETECCIÓN"
        if source_text:
            panel_title += f" ({source_text})"
        
        cv2.putText(overlay, panel_title, (summary_x, summary_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        summary_y += 20
        
        # Velocidad del robot
        if robot_speed != 0:
            robot_color = (0, 255, 0) if robot_speed > 0 else (255, 100, 0)
            cv2.putText(overlay, f"Robot: {robot_speed:.2f} m/s", 
                    (summary_x, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, robot_color, 1)
            summary_y += 18
        
        # Contadores por tipo
        if type_counts['person'] > 0:
            cv2.putText(overlay, f"👤 Personas: {type_counts['person']}", 
                    (summary_x, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 
                    self.colors['person'], 1)
            summary_y += 16
        
        if type_counts['vehicle'] > 0:
            cv2.putText(overlay, f"🚗 Vehículos: {type_counts['vehicle']}", 
                    (summary_x, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 
                    self.colors['car'], 1)
            summary_y += 16
        
        if type_counts['other'] > 0:
            cv2.putText(overlay, f"📦 Otros: {type_counts['other']}", 
                    (summary_x, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 
                    (200, 200, 200), 1)
            summary_y += 16
        
        # Estadísticas de tracking
        if tracking_stats['static_objects'] > 0:
            cv2.putText(overlay, f"🛑 Estáticos: {tracking_stats['static_objects']}", 
                    (summary_x, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 
                    (0, 200, 0), 1)
            summary_y += 16
        
        if tracking_stats['in_path'] > 0:
            warning_color = (0, 200, 255)  # Naranja
            cv2.putText(overlay, f"⚠️ En trayectoria: {tracking_stats['in_path']}", 
                    (summary_x, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 
                    warning_color, 1)
            summary_y += 16
        
        if tracking_stats['danger_objects'] > 0:
            danger_color = (0, 0, 255)  # Rojo
            cv2.putText(overlay, f"🚨 Peligro (TTC<5s): {tracking_stats['danger_objects']}", 
                    (summary_x, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 
                    danger_color, 1)
        
        return overlay
    
    def create_segmentation_overlay(self, image, seg_mask):
        """Crea overlay de segmentación."""
        if image is None or seg_mask is None:
            return image
        
        # Crear una imagen de debug para ver la segmentación cruda
        debug_seg = np.zeros((seg_mask.shape[0], seg_mask.shape[1], 3), dtype=np.uint8)
        
        # Colorear la segmentación
        # Fondo: negro (0)
        # Área transitable: verde (1)
        # Carriles: amarillo (2)
        debug_seg[seg_mask == 1] = self.colors['drivable']  # Verde
        debug_seg[seg_mask == 2] = self.colors['lane']      # Amarillo
        
        # Redimensionar para que coincida con la imagen
        if debug_seg.shape != image.shape:
            debug_seg = cv2.resize(debug_seg, (image.shape[1], image.shape[0]))
        
        # Aplicar overlay semi-transparente
        overlay = image.copy()
        alpha = 0.5
        
        # Solo aplicar overlay donde hay segmentación (valores 1 o 2)
        mask = (seg_mask > 0).astype(np.uint8)
        mask = cv2.resize(mask, (image.shape[1], image.shape[0]))
        mask_3ch = np.stack([mask, mask, mask], axis=2)
        
        # Aplicar overlay solo en las áreas segmentadas
        overlay = np.where(mask_3ch > 0, 
                        cv2.addWeighted(debug_seg, alpha, overlay, 1 - alpha, 0), 
                        overlay)
        
        # Añadir contornos de carriles
        lane_mask = (seg_mask == 2).astype(np.uint8) * 255
        if np.any(lane_mask):
            lane_mask = cv2.resize(lane_mask, (image.shape[1], image.shape[0]))
            contours, _ = cv2.findContours(lane_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) > 100:
                    cv2.drawContours(overlay, [contour], -1, (0, 255, 255), 2)
        
        return overlay
    
    def create_lane_error_visualization(self, seg_mask, frame_stats):
        """Crea visualización del error de carril usando algoritmo predictivo robusto."""
        if seg_mask is None:
            # Crear imagen vacía si no hay datos
            empty_img = np.zeros((400, 600, 3), dtype=np.uint8)
            cv2.putText(empty_img, "No segmentation data", (50, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            return empty_img
        
        h, w = seg_mask.shape
        
        # Crear imagen de debug
        debug_img = np.full((h, w, 3), 0, dtype=np.uint8)
        
        # Colorear máscara de segmentación
        lane_pixels = (seg_mask == 2)
        road_pixels = (seg_mask == 1)
        debug_img[lane_pixels] = [0, 100, 200]  # Azul-naranja para carriles
        debug_img[road_pixels] = [50, 50, 50]   # Gris para carretera
        
        # ==================== ALGORITMO PREDICTIVO ROBUSTO ====================
        
        # PARÁMETROS (igual que en tiempo real)
        num_horizons = 5
        ref_ratio = 0.8  # Línea de referencia (80% = lado derecho)
        horizon_weights = [0.001, 0.004, 0.05, 0.004, 0.001]  # pesos [cercano → lejano]
        min_points_per_horizon = 5
        outlier_percentile_low = 5
        outlier_percentile_high = 90
        
        horizon_data = []
        horizon_errors = []
        
        for i in range(num_horizons):
            # Distribución no lineal: más resolución cerca del robot
            # i=0 → bottom (más cercano, más peso)
            # i=N-1 → top (más lejano, menos peso)
            ratio = (num_horizons - i - 1) / num_horizons
            y_center = int(h * (0.7 + ratio * 0.2))  # Entre 40% y 90%
            
            # Franja vertical alrededor del horizonte
            slice_height = h // (num_horizons * 10)
            y1 = max(0, y_center - slice_height)
            y2 = min(h, y_center + slice_height)
            
            # SOLO lado derecho (donde está el carril)
            slice_mask = seg_mask[y1:y2, w//2:w]
            
            # EXTRACCIÓN ROBUSTA: usar tanto lane_mask (2) como road_mask (1)
            lane_ys, lane_xs = np.where(slice_mask == 2)
            road_ys, road_xs = np.where(slice_mask == 1)
            
            # Estrategia inteligente: ¿cuál está más a la derecha?
            all_xs = []
            all_ys = []
            
            lane_has_enough = lane_xs is not None and len(lane_xs) >= min_points_per_horizon
            road_has_enough = road_xs is not None and len(road_xs) >= min_points_per_horizon
            
            if lane_has_enough and road_has_enough:
                # Usamos la que esté MÁS A LA DERECHA
                lane_rightmost = np.percentile(lane_xs, 85) if len(lane_xs) > 0 else -1
                road_rightmost = np.percentile(road_xs, 85) if len(road_xs) > 0 else -1
                
                if lane_rightmost > road_rightmost:
                    all_xs.extend(lane_xs)
                    all_ys.extend(lane_ys)
                else:
                    all_xs.extend(road_xs)
                    all_ys.extend(road_ys)
            elif lane_has_enough:
                all_xs.extend(lane_xs)
                all_ys.extend(lane_ys)
            elif road_has_enough:
                all_xs.extend(road_xs)
                all_ys.extend(road_ys)
            else:
                # Combinar ambas aunque sean escasas
                if lane_xs is not None:
                    all_xs.extend(lane_xs)
                    all_ys.extend(lane_ys)
                if road_xs is not None:
                    all_xs.extend(road_xs)
                    all_ys.extend(road_ys)
            
            # Si no hay suficientes puntos, continuar
            if len(all_xs) < min_points_per_horizon:
                continue
            
            all_xs = np.array(all_xs)
            all_ys = np.array(all_ys)
            
            # FILTRADO DE OUTLIERS
            x_low = np.percentile(all_xs, outlier_percentile_low)
            x_high = np.percentile(all_xs, outlier_percentile_high)
            
            # Filtrar puntos dentro del rango
            valid_mask = (all_xs >= x_low) & (all_xs <= x_high)
            filtered_xs = all_xs[valid_mask]
            filtered_ys = all_ys[valid_mask]
            
            if len(filtered_xs) < min_points_per_horizon:
                continue
            
            # El "borde" es el percentil alto (lado derecho)
            edge_x = int(np.percentile(filtered_xs, 85))
            
            # Ajustar a coordenadas globales
            global_x = edge_x + w // 2
            
            # Calcular confianza
            total_pixels = slice_mask.shape[0] * slice_mask.shape[1]
            point_ratio = len(filtered_xs) / total_pixels
            confidence = min(1.0, point_ratio * 10)
            
            horizon_data.append({
                'horizon_idx': i,
                'y': y_center,
                'x': global_x,
                'confidence': confidence,
                'slice_bounds': (y1, y2)
            })
        
        # ==================== CÁLCULO DEL ERROR ====================
        
        # LÍNEA DE REFERENCIA (¡IMPORTANTE!)
        # La referencia está a la DERECHA (80% del ancho)
        # CONVENCIÓN DE SIGNOS:
        # - Error POSITIVO (+) → Robot debe ALEJARSE del carril
        # - Error NEGATIVO (-) → Robot debe ACERCARSE al carril
        
        ref_x = int(w * ref_ratio)
        weighted_error = 0.0
        total_weight = 0.0
        
        if horizon_data:
            # DETECTAR SI ES CURVA
            is_curve = False
            if len(horizon_data) >= 3:
                x_positions = [h['x'] for h in horizon_data]
                x_range = np.max(x_positions) - np.min(x_positions)
                normalized_variation = x_range / w
                is_curve = normalized_variation > 0.15  # umbral
            
            # AJUSTE EN CURVAS
            if is_curve:
                curve_weight_boost = 1.5
                adjusted_weights = []
                
                for i, w_orig in enumerate(horizon_weights[:len(horizon_data)]):
                    if i < 2:  # Horizontes cercanos
                        adjusted_weights.append(w_orig * curve_weight_boost)
                    else:  # Horizontes lejanos
                        adjusted_weights.append(w_orig / curve_weight_boost)
                
                # Normalizar para que sumen 1
                total = sum(adjusted_weights)
                weights = [w / total for w in adjusted_weights]
            else:
                weights = horizon_weights[:len(horizon_data)]
            
            # CÁLCULO DE ERROR PONDERADO
            for h_data, weight in zip(horizon_data, weights):
                detected_x = h_data['x']
                
                # ¡¡¡FÓRMULA IMPORTANTE!!!
                # Error = (ref_x - detected_x) / (w/2.0)
                # Si detected_x < ref_x → error > 0 → alejarse
                # Si detected_x > ref_x → error < 0 → acercarse
                error = (ref_x - detected_x) / (w / 2.0)
                horizon_errors.append(error)
                
                # Ponderar por confianza y peso
                effective_weight = weight * h_data['confidence']
                weighted_error += error * effective_weight
                total_weight += effective_weight
            
            if total_weight > 0:
                weighted_error = weighted_error / total_weight
                weighted_error = np.clip(weighted_error, -1.0, 1.0)
            else:
                weighted_error = 0.0
        
        # ==================== VISUALIZACIÓN ====================
        
        # Dibujar horizontes y puntos
        colors = [(0, 255, 255), (0, 255, 128), (0, 255, 0), (128, 255, 0), (255, 128, 0)]
        
        for i, point in enumerate(horizon_data):
            color = colors[min(i, len(colors)-1)]
            y = point['y']
            x = point['x']
            
            # Línea horizontal del horizonte
            cv2.line(debug_img, (w//2, y), (w, y), (100, 100, 100), 1)
            
            # Punto central detectado
            cv2.circle(debug_img, (x, y), 8, color, -1)
            cv2.circle(debug_img, (x, y), 10, (255, 255, 255), 2)
            
            # Etiqueta con error
            if i < len(horizon_errors):
                cv2.putText(debug_img, f"H{i}: {horizon_errors[i]:+.2f}", (x + 10, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # ==================== LÍNEA DE REFERENCIA ====================
        ref_x = w // 2 + int((w // 2) * 0.6)  # 80% desde el centro (equivale a 80% total)
        cv2.line(debug_img, (ref_x, 0), (ref_x, h), (255, 0, 0), 3)
        cv2.putText(debug_img, "REF (80%)", (ref_x + 10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        # ==================== BARRA DE ERROR ====================
        
        # Calcular error promedio si hay puntos
        if horizon_data:
            # Error ya calculado como weighted_error
            
            # Barra de error visual
            bar_width = 300
            bar_height = 40
            bar_y = 50
            
            # Dibujar barra base
            cv2.rectangle(debug_img, (50, bar_y), (50 + bar_width, bar_y + bar_height), 
                        (100, 100, 100), -1)
            
            # Centro de la barra (error = 0)
            center_x = 50 + bar_width // 2
            cv2.line(debug_img, (center_x, bar_y), (center_x, bar_y + bar_height), 
                    (255, 255, 255), 2)
            
            # Posición del error
            # Mapear error de [-1, 1] a posición en barra
            error_pos = int(center_x + (weighted_error * bar_width / 2))
            error_pos = np.clip(error_pos, 50, 50 + bar_width)
            
            # Color según magnitud del error
            if abs(weighted_error) < 0.2:
                bar_color = (0, 255, 0)  # Verde: OK
            elif abs(weighted_error) < 0.5:
                bar_color = (0, 200, 255)  # Naranja: Moderado
            else:
                bar_color = (0, 0, 255)  # Rojo: Grande
            
            # Indicador de error
            cv2.rectangle(debug_img, (error_pos - 5, bar_y + 5), 
                        (error_pos + 5, bar_y + bar_height - 5), bar_color, -1)
            
            # Texto de error con interpretación
            if weighted_error > 0.05:
                direction = "← ALEJARSE (Giro ANTIHORARIO)"
                dir_color = self.colors['warning']
            elif weighted_error < -0.05:
                direction = "ACERCARSE → (Giro HORARIO)"
                dir_color = self.colors['warning']
            else:
                direction = "OK (Recto)"
                dir_color = (0, 255, 0)
            
            cv2.putText(debug_img, f"Error: {weighted_error:+.3f}", 
                    (50, bar_y + bar_height + 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, dir_color, 2)
            cv2.putText(debug_img, direction, 
                    (50, bar_y + bar_height + 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, dir_color, 1)
            
            # Mostrar si es curva
            if horizon_data and len(horizon_data) >= 3:
                x_positions = [h['x'] for h in horizon_data]
                x_range = np.max(x_positions) - np.min(x_positions)
                normalized_variation = x_range / w
                
                is_curve = normalized_variation > 0.15
                curve_text = f"Curva: {'SÍ' if is_curve else 'NO'} (var: {normalized_variation:.2f})"
                curve_color = (255, 200, 0) if is_curve else (100, 100, 100)
                
                cv2.putText(debug_img, curve_text, 
                        (50, bar_y + bar_height + 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, curve_color, 1)
        
        # ==================== LEYENDA ====================
        legend_y = h - 20
        cv2.putText(debug_img, "CONVENCION: (+) Alejarse / (-) Acercarse", 
                (10, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # ==================== ESTADÍSTICAS ====================
        stats_y = h - 60
        if frame_stats and 'drivable_percent' in frame_stats:
            cv2.putText(debug_img, f"Area transitable: {frame_stats['drivable_percent']:.1f}%", 
                    (20, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.colors['drivable'], 2)
            stats_y -= 30
        
        if frame_stats and 'lane_percent' in frame_stats:
            cv2.putText(debug_img, f"Carriles: {frame_stats['lane_percent']:.1f}%", 
                    (20, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.colors['lane'], 2)
            stats_y -= 30
        
        if frame_stats and 'num_lane_contours' in frame_stats:
            cv2.putText(debug_img, f"Contornos carril: {frame_stats['num_lane_contours']}", 
                    (20, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        
        return debug_img
    
    def create_trajectory_plot(self, frame_data):
        """Crea gráfica de trayectorias sincronizada con EKF Global y Local."""
        fig, ax = plt.subplots(figsize=(8, 6), dpi=100)

        fig.subplots_adjust(
            left=0.06,
            right=0.98,
            top=0.92,
            bottom=0.08
        )

        # 1. Path global (si existe)
        if self.global_path is not None and 'x' in self.global_path.columns:
            x_data = self.global_path['x'].to_numpy()
            y_data = self.global_path['y'].to_numpy()
            
            ax.plot(x_data, y_data, 'b-', linewidth=2, alpha=0.7, label='Path Global Planificado')
            
            if len(self.global_path) > 0:
                start_point = self.global_path.iloc[0]
                end_point = self.global_path.iloc[-1]
                
                ax.scatter(start_point['x'], start_point['y'], 
                        c='green', s=100, marker='s', label='Inicio', zorder=5)
                ax.scatter(end_point['x'], end_point['y'], 
                        c='red', s=100, marker='*', label='Meta', zorder=5)
        
        # 2. Trayectoria EKF GLOBAL
        if self.ekf_global is not None and 'position_x' in self.ekf_global.columns:
            # USAR ÍNDICE PROPORCIONAL EN VEZ DE TIMESTAMP
            frame_idx = frame_data.get('frame_idx', 0)
            frames_count = len(self.frames_list)
            ekf_global_count = len(self.ekf_global)
            
            if frames_count > 0 and ekf_global_count > 0:
                # Calcular índice proporcional para EKF Global
                proportion = frame_idx / frames_count
                ekf_global_idx = int(proportion * ekf_global_count)
                ekf_global_idx = max(0, min(ekf_global_idx, ekf_global_count - 1))
                
                # Mostrar trayectoria EKF Global hasta el índice actual
                ekf_global_until_now = self.ekf_global.iloc[:ekf_global_idx + 1]
                
                if len(ekf_global_until_now) > 0:
                    pos_x_global = ekf_global_until_now['position_x'].to_numpy()
                    pos_y_global = ekf_global_until_now['position_y'].to_numpy()
                    
                    ax.plot(pos_x_global, pos_y_global, 'g-', linewidth=3, alpha=0.9, label='EKF Global')
                    
                    # Punto actual EKF Global
                    last_point_global = ekf_global_until_now.iloc[-1]
                    ax.scatter(last_point_global['position_x'], last_point_global['position_y'], 
                            c='red', s=150, marker='o', edgecolors='white', 
                            linewidth=2, label='Pos Actual Global', zorder=10)
        
        # 3. Trayectoria EKF LOCAL
        if self.ekf_local is not None and 'position_x' in self.ekf_local.columns:
            # USAR ÍNDICE PROPORCIONAL EN VEZ DE TIMESTAMP
            frame_idx = frame_data.get('frame_idx', 0)
            frames_count = len(self.frames_list)
            ekf_local_count = len(self.ekf_local)
            
            if frames_count > 0 and ekf_local_count > 0:
                # Calcular índice proporcional para EKF Local
                proportion = frame_idx / frames_count
                ekf_local_idx = int(proportion * ekf_local_count)
                ekf_local_idx = max(0, min(ekf_local_idx, ekf_local_count - 1))
                
                # Mostrar trayectoria EKF Local hasta el índice actual
                ekf_local_until_now = self.ekf_local.iloc[:ekf_local_idx + 1]
                
                if len(ekf_local_until_now) > 0:
                    pos_x_local = ekf_local_until_now['position_x'].to_numpy()
                    pos_y_local = ekf_local_until_now['position_y'].to_numpy()
                    
                    ax.plot(pos_x_local, pos_y_local, 'm-', linewidth=2, alpha=0.7, label='EKF Local')
                    
                    # Punto actual EKF Local
                    last_point_local = ekf_local_until_now.iloc[-1]
                    ax.scatter(last_point_local['position_x'], last_point_local['position_y'], 
                            c='orange', s=100, marker='s', edgecolors='white', 
                            linewidth=2, label='Pos Actual Local', zorder=9)
        
        # 4. Configurar gráfica
        ax.set_xlabel('X [m]', fontsize=12, fontweight='bold')
        ax.set_ylabel('Y [m]', fontsize=12, fontweight='bold')
        ax.set_title('Trayectoria del Robot', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')

        ax.relim()
        ax.autoscale_view()
        ax.set_aspect('equal', adjustable='datalim')

        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(
                loc='upper right',
                fontsize=9,
                framealpha=0.85,
                borderpad=0.3
            )

        # 7. Convertir figura a imagen
        canvas = FigureCanvasAgg(fig)
        canvas.draw()
        trajectory_img = np.array(canvas.renderer.buffer_rgba())
        trajectory_img = cv2.cvtColor(trajectory_img, cv2.COLOR_RGBA2BGR)
        plt.close(fig)
        
        return trajectory_img
    
    def create_stats_panel(self, frame_data):
        """Panel de estadísticas que aprovecha TODO el alto disponible."""

        # ================= DIMENSIONES =================
        PANEL_HEIGHT = 900
        PANEL_WIDTH = 600
        stats_panel = np.zeros((PANEL_HEIGHT, PANEL_WIDTH, 3), dtype=np.uint8)
        stats_panel[:] = self.colors['background']

        def px_y(p): return int(PANEL_HEIGHT * p / 100)
        def px_x(p): return int(PANEL_WIDTH * p / 100)

        # ================= ZONAS DEL PANEL =================
        HEADER_H = px_y(28)     # Parte superior (textos)
        FOOTER_H = px_y(10)     # Parte inferior
        CONTENT_TOP = HEADER_H
        CONTENT_BOTTOM = PANEL_HEIGHT - FOOTER_H
        CONTENT_H = CONTENT_BOTTOM - CONTENT_TOP

        # ================= HEADER =================
        y = px_y(4)

        cv2.putText(
            stats_panel, "ESTADISTICAS DE CONTROL",
            (px_x(4), y),
            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3
        )
        y += px_y(6)

        if 'control_stats' in frame_data and self.control_data is not None:
            control = frame_data['control_stats']

            # ---- VELOCIDADES ----
            cv2.putText(
                stats_panel, "VELOCIDADES",
                (px_x(4), y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 200, 100), 2
            )
            y += px_y(4)

            # LINEAL
            speed = float(control.get('cmd_vel_linear_x', 0.0) or 0.0)
            if abs(speed) > 0.3:
                c, lbl = (0,255,0), "ALTA"
            elif abs(speed) > 0.1:
                c, lbl = (0,200,255), "MEDIA"
            elif abs(speed) > 0.01:
                c, lbl = (255,255,0), "BAJA"
            else:
                c, lbl = (180,180,180), "MUY BAJA"

            cv2.putText(stats_panel, "LINEAL:", (px_x(6), y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255), 2)
            cv2.putText(stats_panel, f"{speed:.3f} m/s ({lbl})",
                        (px_x(30), y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, c, 2)
            y += px_y(4.5)

            # ANGULAR
            angular = float(control.get('cmd_vel_angular_z', 0.0) or 0.0)
            if abs(angular) > 0.5:
                c, lbl = (0,255,0), "ALTA"
            elif abs(angular) > 0.2:
                c, lbl = (0,200,255), "MEDIA"
            elif abs(angular) > 0.05:
                c, lbl = (255,255,0), "BAJA"
            else:
                c, lbl = (180,180,180), "MUY BAJA"

            cv2.putText(stats_panel, "ANGULAR:", (px_x(6), y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255), 2)
            cv2.putText(stats_panel, f"{angular:.3f} rad/s ({lbl})",
                        (px_x(30), y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, c, 2)

        # Línea separadora HEADER → GRAFICAS
        cv2.line(
            stats_panel,
            (px_x(4), HEADER_H),
            (px_x(96), HEADER_H),
            (100,100,100), 2
        )

        # ================= ZONA DE GRÁFICAS (CLAVE) =================
        graph_gap = 8
        graph_h = (CONTENT_H - graph_gap) // 2
        graph_w = px_x(92)
        graph_x = px_x(4)

        # Gráfica LINEAL
        linear_graph = self.create_velocity_graph(
            graph_w, graph_h,
            'cmd_vel_linear_x',
            "VELOCIDAD LINEAL",
            (0,200,255),
            speed
        )
        if linear_graph is not None:
            stats_panel[
                CONTENT_TOP : CONTENT_TOP + graph_h,
                graph_x : graph_x + graph_w
            ] = linear_graph

        # Gráfica ANGULAR
        angular_graph = self.create_velocity_graph(
            graph_w, graph_h,
            'cmd_vel_angular_z',
            "VELOCIDAD ANGULAR",
            (255,100,0),
            angular
        )
        if angular_graph is not None:
            stats_panel[
                CONTENT_TOP + graph_h + graph_gap :
                CONTENT_TOP + graph_h*2 + graph_gap,
                graph_x : graph_x + graph_w
            ] = angular_graph

        # ================= FOOTER =================
        footer_y = PANEL_HEIGHT - px_y(4)

        cv2.putText(
            stats_panel,
            f"Frame: {self.current_frame+1}/{len(self.frames_list)}",
            (px_x(4), footer_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2
        )

        status = "PAUSADO" if self.paused else f"REPRODUCIENDO ({self.play_speed}x)"
        status_color = (0,200,255) if self.paused else (0,255,0)

        cv2.putText(
            stats_panel, status,
            (px_x(52), footer_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2
        )

        return stats_panel


    def create_velocity_graph(self, width, height, column_name, title, base_color, current_value=None):
        """Crea una gráfica de velocidad para una columna específica."""
        if self.control_data is None or column_name not in self.control_data.columns:
            return None
        
        # Crear fondo de gráfica
        graph = np.zeros((height, width, 3), dtype=np.uint8)
        graph[:] = (30, 30, 30)
        
        # Dibujar grid
        grid_spacing = width // 10
        for i in range(1, 10):
            x_pos = i * grid_spacing
            cv2.line(graph, (x_pos, 0), (x_pos, height), (50, 50, 50), 1)
        
        # Línea central (cero)
        mid_y = height // 2
        cv2.line(graph, (0, mid_y), (width, mid_y), (80, 80, 80), 2)
        
        # Obtener datos (últimos 100 frames o menos)
        start_idx = max(0, self.current_frame - 100)
        end_idx = min(len(self.control_data), self.current_frame + 1)
        
        # Variable para almacenar los datos procesados
        data = np.array([])
        
        if end_idx > start_idx:
            # Obtener datos históricos
            data = self.control_data[column_name].iloc[start_idx:end_idx].to_numpy()
            
            # MANEJO DE NaN: reemplazar NaN por 0
            data = np.nan_to_num(data, nan=0.0)
            
            # Si tenemos valor actual sincronizado, reemplazar el último valor
            if current_value is not None and len(data) > 0:
                data[-1] = current_value
            elif current_value is not None and len(data) == 0:
                # Si no hay datos históricos pero sí valor actual
                data = np.array([current_value])
            
            if len(data) > 1:
                # Calcular máximo absoluto para escalar
                max_val = np.nanmax(np.abs(data))
                if max_val < 0.001:  # Si todos los valores son casi cero
                    max_val = 1.0  # Valor por defecto para evitar división por cero
                
                # Normalizar para visualización (-1 a 1 mapeado a altura completa)
                normalized = (data / max_val) * 0.9  # Usar 90% de la altura
                
                # Convertir a coordenadas de gráfica
                points = []
                for i, value in enumerate(normalized):
                    x = int((i / max(1, len(normalized)-1)) * (width - 10)) + 5
                    y = int(mid_y - (value * (height // 2)))
                    points.append((x, y))
                
                # Dibujar línea
                for i in range(len(points)-1):
                    # Color más intenso para valores mayores
                    intensity = min(255, int(150 + abs(data[i]) * 100))
                    
                    if "LINEAL" in title:
                        # Azul para positivo, naranja para negativo
                        if data[i] >= 0:
                            color = (intensity, intensity//2, 0)  # Naranja
                        else:
                            color = (0, intensity//2, intensity)  # Azul
                    else:
                        # Rojo para positivo, verde para negativo (angular)
                        if data[i] >= 0:
                            color = (0, intensity//2, intensity)  # Rojo (BGR)
                        else:
                            color = (0, intensity, intensity//2)  # Verde (BGR)
                    
                    cv2.line(graph, points[i], points[i+1], color, 2)
                
                # Dibujar punto actual
                if points:
                    last_point = points[-1]
                    # Usar current_value si está disponible, sino el último de data
                    display_value = current_value if current_value is not None else data[-1]
                    
                    # Color del punto actual
                    if abs(display_value) > 0.3:
                        point_color = (0, 255, 0)  # Verde para valores altos
                    elif abs(display_value) > 0.1:
                        point_color = (0, 200, 255)  # Naranja para valores medios
                    elif abs(display_value) > 0.01:
                        point_color = (255, 255, 0)  # Amarillo para valores bajos
                    else:
                        point_color = (200, 200, 200)  # Gris para valores muy bajos
                    
                    cv2.circle(graph, last_point, 6, point_color, -1)
                    cv2.circle(graph, last_point, 8, (255, 255, 255), 1)
                    
                    # Etiqueta con valor actual
                    value_text = f"{display_value:.3f}"
                    if "LINEAL" in title:
                        value_text += " m/s"
                    else:
                        value_text += " rad/s"
                    
                    # Posicionar etiqueta
                    text_x = last_point[0] + 10 if last_point[0] < width - 80 else last_point[0] - 80
                    text_y = last_point[1] - 10 if last_point[1] > 20 else last_point[1] + 20
                    
                    # Fondo para etiqueta
                    text_size = cv2.getTextSize(value_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
                    cv2.rectangle(graph, 
                                (text_x - 3, text_y - text_size[1] - 3),
                                (text_x + text_size[0] + 3, text_y + 3),
                                (40, 40, 40), -1)
                    
                    cv2.putText(graph, value_text, 
                            (text_x, text_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, point_color, 1)
        
        # Título de la gráfica
        cv2.putText(graph, title, 
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Escala - Solo si tenemos datos
        if len(data) > 0:
            max_val = np.nanmax(np.abs(data))
            scale_text = f"Max: {max_val:.2f}"
            if "LINEAL" in title:
                scale_text += " m/s"
            else:
                scale_text += " rad/s"
            
            cv2.putText(graph, scale_text, 
                    (width - 100, height - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        
        return graph

    def compose_dashboard(self, frame_data):
        """Compone el dashboard completo."""
        # Crear canvas principal
        dashboard = np.zeros((1080, 1920, 3), dtype=np.uint8)
        dashboard[:] = self.colors['background']
        
        # ==================== DEFINIR TAMAÑOS Y POSICIONES ====================
        # Configuración uniforme
        component_width = 640
        component_height = 480
        title_height = 40
        
        # Margenes
        margin_x = 20
        margin_y = 20
        
        # Espacio entre componentes
        gap_x = 20
        gap_y = 20
        
        # Calcular posiciones de forma sistemática
        # COLUMNA 1 (izquierda)
        col1_x = margin_x
        
        # COLUMNA 2 (centro)
        col2_x = col1_x + component_width + gap_x
        
        # COLUMNA 3 (derecha)
        col3_x = col2_x + component_width + gap_x
        col3_width = 1920 - col3_x - margin_x  # El ancho restante
        
        # FILA 1 (superior)
        row1_y = margin_y
        
        # FILA 2 (inferior)
        row2_y = row1_y + component_height + title_height + gap_y
        
        # Tamaño total con título
        total_height = title_height + component_height
        
        # ==================== 1. IMAGEN ORIGINAL / VIDEO DEBUG (COL1, ROW1) ====================

        base_frame = frame_data.get('image')
        source_label = "IMAGEN ORIGINAL"

        # Video SOLO para debug visual
        if self.debug_use_video:
            video_frame = self.get_debug_video_frame(
                self.current_frame,
                self.total_frames
            )
            if video_frame is not None:
                base_frame = video_frame
                source_label = "VIDEO DEBUG (MP4)"

        if base_frame is not None:
            base_resized = cv2.resize(base_frame, (component_width, component_height))

            title_bg = np.zeros((title_height, component_width, 3), dtype=np.uint8)
            title_bg[:] = (40, 40, 40)
            cv2.putText(
                title_bg,
                source_label,
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            combined = np.vstack([title_bg, base_resized])
            dashboard[row1_y:row1_y + combined.shape[0],
                    col1_x:col1_x + combined.shape[1]] = combined
        else:
            placeholder = np.zeros((component_height, component_width, 3), dtype=np.uint8)
            cv2.putText(
                placeholder,
                "NO HAY IMAGEN BASE",
                (150, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2
            )

            title_bg = np.zeros((title_height, component_width, 3), dtype=np.uint8)
            title_bg[:] = (40, 40, 40)
            cv2.putText(
                title_bg,
                "IMAGEN BASE NO DISPONIBLE",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            combined = np.vstack([title_bg, placeholder])
            dashboard[row1_y:row1_y + combined.shape[0],
                    col1_x:col1_x + combined.shape[1]] = combined



                
        # ==================== 2. DETECCIONES (COL2, ROW1) ====================
        if self.show_detections and 'image' in frame_data and 'detections' in frame_data:
            # Pasar frame_stats para obtener velocidad del robot
            frame_stats_for_det = frame_data.get('perception_stats', {})
            if 'control_stats' in frame_data:
                frame_stats_for_det['control_stats'] = frame_data['control_stats']
            
            det_img = self.create_detection_overlay(
                frame_data['image'].copy(), 
                frame_data['detections'],
                frame_stats_for_det  # <-- Pasar stats para velocidad del robot
            )
            if det_img is not None:
                det_resized = cv2.resize(det_img, (component_width, component_height))
                
                # Título
                title_bg = np.zeros((title_height, component_width, 3), dtype=np.uint8)
                title_bg[:] = (40, 40, 40)
                
                # Contar detecciones
                num_detections = 0
                if 'detections' in frame_data:
                    num_detections = len(frame_data['detections'].get('detections', []))
                
                cv2.putText(title_bg, f"DETECCIONES ({num_detections} objetos)", 
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                
                combined = np.vstack([title_bg, det_resized])
                dashboard[row1_y:row1_y+combined.shape[0], col2_x:col2_x+combined.shape[1]] = combined
        else:
            # Placeholder para detecciones
            placeholder = np.zeros((component_height, component_width, 3), dtype=np.uint8)
            if not self.show_detections:
                cv2.putText(placeholder, "DETECCIONES DESACTIVADAS", (150, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            else:
                cv2.putText(placeholder, "NO HAY DETECCIONES", (180, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            
            title_bg = np.zeros((title_height, component_width, 3), dtype=np.uint8)
            title_bg[:] = (40, 40, 40)
            status_text = "DETECCIONES" if self.show_detections else "DETECCIONES (DESACTIVADAS)"
            cv2.putText(title_bg, status_text, 
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            combined = np.vstack([title_bg, placeholder])
            dashboard[row1_y:row1_y+combined.shape[0], col2_x:col2_x+combined.shape[1]] = combined
        
        # ==================== 3. SEGMENTACION (COL1, ROW2) ====================
        if self.show_segmentation and 'image' in frame_data and 'segmentation' in frame_data:
            seg_img = self.create_segmentation_overlay(frame_data['image'].copy(), frame_data['segmentation'])
            if seg_img is not None:
                seg_resized = cv2.resize(seg_img, (component_width, component_height))
                
                # Título
                title_bg = np.zeros((title_height, component_width, 3), dtype=np.uint8)
                title_bg[:] = (40, 40, 40)
                
                # Estadísticas
                stats_text = ""
                if 'perception_stats' in frame_data:
                    stats = frame_data['perception_stats']
                    if 'drivable_percent' in stats:
                        stats_text = f" | Area: {stats['drivable_percent']:.1f}%"
                
                status_text = "SEGMENTACION" if self.show_segmentation else "SEGMENTACION (DESACTIVADA)"
                cv2.putText(title_bg, f"{status_text}{stats_text}", 
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                
                combined = np.vstack([title_bg, seg_resized])
                dashboard[row2_y:row2_y+combined.shape[0], col1_x:col1_x+combined.shape[1]] = combined
        else:
            # Placeholder para segmentación
            placeholder = np.zeros((component_height, component_width, 3), dtype=np.uint8)
            if not self.show_segmentation:
                cv2.putText(placeholder, "SEGMENTACION DESACTIVADA", (140, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            else:
                cv2.putText(placeholder, "NO HAY SEGMENTACION", (160, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            
            title_bg = np.zeros((title_height, component_width, 3), dtype=np.uint8)
            title_bg[:] = (40, 40, 40)
            status_text = "SEGMENTACION" if self.show_segmentation else "SEGMENTACION (DESACTIVADA)"
            cv2.putText(title_bg, status_text, 
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            combined = np.vstack([title_bg, placeholder])
            dashboard[row2_y:row2_y+combined.shape[0], col1_x:col1_x+combined.shape[1]] = combined
        
        # ==================== 4. ERROR DE CARRIL (COL2, ROW2) ====================
        if 'segmentation' in frame_data:
            lane_img = self.create_lane_error_visualization(
                frame_data['segmentation'], 
                frame_data.get('perception_stats', {})
            )
            if lane_img is not None:
                lane_resized = cv2.resize(lane_img, (component_width, component_height))
                
                # Título
                title_bg = np.zeros((title_height, component_width, 3), dtype=np.uint8)
                title_bg[:] = (40, 40, 40)
                cv2.putText(title_bg, "ERROR DE CARRIL", 
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                
                combined = np.vstack([title_bg, lane_resized])
                dashboard[row2_y:row2_y+combined.shape[0], col2_x:col2_x+combined.shape[1]] = combined
        else:
            # Placeholder para error de carril
            placeholder = np.zeros((component_height, component_width, 3), dtype=np.uint8)
            cv2.putText(placeholder, "NO HAY DATOS DE CARRIL", (160, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            
            title_bg = np.zeros((title_height, component_width, 3), dtype=np.uint8)
            title_bg[:] = (40, 40, 40)
            cv2.putText(title_bg, "ERROR DE CARRIL (NO DISPONIBLE)", 
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            combined = np.vstack([title_bg, placeholder])
            dashboard[row2_y:row2_y+combined.shape[0], col2_x:col2_x+combined.shape[1]] = combined
        
        # ==================== 5. TRAYECTORIA (COL3, ROW1) ====================
        # Ajustar tamaño de la columna derecha
        traj_width = min(600, col3_width)
        traj_height = 450
        
        if self.show_trajectory:
            trajectory_img = self.create_trajectory_plot(frame_data) 
            if trajectory_img is not None:
                traj_resized = cv2.resize(trajectory_img, (traj_width, traj_height))
                # Centrar horizontalmente en la columna 3
                traj_x = col3_x + (col3_width - traj_width) // 2
                dashboard[row1_y:row1_y+traj_resized.shape[0], traj_x:traj_x+traj_resized.shape[1]] = traj_resized
        else:
            # Placeholder para trayectoria
            placeholder = np.zeros((traj_height, traj_width, 3), dtype=np.uint8)
            cv2.putText(placeholder, "TRAYECTORIA DESACTIVADA", (130, 225),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            traj_x = col3_x + (col3_width - traj_width) // 2
            dashboard[row1_y:row1_y+placeholder.shape[0], traj_x:traj_x+placeholder.shape[1]] = placeholder
        
        # ==================== 6. ESTADÍSTICAS (COL3, ROW2) ====================
        stats_panel = self.create_stats_panel(frame_data)
        if stats_panel is not None:
            # Ajustar tamaño del panel de estadísticas
            stats_width = min(600, col3_width)
            stats_height = 500
            stats_resized = cv2.resize(stats_panel, (stats_width, stats_height))
            
            # Calcular posición Y para estadísticas
            stats_y = row2_y
            
            # Centrar horizontalmente
            stats_x = col3_x + (col3_width - stats_width) // 2
            
            dashboard[stats_y:stats_y+stats_resized.shape[0], stats_x:stats_x+stats_resized.shape[1]] = stats_resized
        
        # ==================== LÍNEAS DIVISORIAS ====================
        # Línea vertical entre columna 1 y 2
        line1_x = col1_x + component_width + gap_x // 2
        cv2.line(dashboard, (line1_x, 0), (line1_x, 1080), (60, 60, 60), 2)
        
        # Línea vertical entre columna 2 y 3
        line2_x = col2_x + component_width + gap_x // 2
        cv2.line(dashboard, (line2_x, 0), (line2_x, 1080), (60, 60, 60), 2)
        
        # Línea horizontal entre fila 1 y 2
        line_y = row1_y + component_height + title_height + gap_y // 2
        cv2.line(dashboard, (0, line_y), (1920, line_y), (60, 60, 60), 2)
        
        # ==================== TÍTULO PRINCIPAL ====================
        title = "DASHBOARD DE NAVEGACION AUTONOMA"
        if self.metadata and 'Ruta' in self.metadata:
            title += f" - {self.metadata['Ruta']}"
        
        # Centrar título
        title_size = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
        title_x = (1920 - title_size[0]) // 2
        cv2.putText(dashboard, title, (title_x, 15), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        
        return dashboard
    
    def run(self):
        """Ejecuta el bucle principal del dashboard."""
        print("\n Iniciando dashboard...")
        
        # Inicializar acumulador fraccionario
        fractional_accumulator = 0.0
        
        while True:
            # Asegurar que current_frame sea entero (importante!)
            self.current_frame = int(self.current_frame)
            
            # Obtener datos del frame actual
            frame_data = self.get_frame_data(self.current_frame)
            
            if frame_data is None:
                print(" No hay datos para el frame actual")
                break
            
            # Componer dashboard
            dashboard = self.compose_dashboard(frame_data)
            
            # Mostrar dashboard
            cv2.imshow(self.window_name, dashboard)
            
            # Manejar entrada de teclado
            key = cv2.waitKey(30 if not self.paused else 0) & 0xFF
            
            if key == ord('q') or key == 27:  # Q o ESC
                break
            elif key == ord(' '):  # SPACE - Play/Pause
                self.paused = not self.paused
            elif key == ord('r'):  # R - Reiniciar
                self.current_frame = 0
            elif key == ord('f'):  # F - Pantalla completa
                cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, 
                                    cv2.WINDOW_FULLSCREEN if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN) == 0 else 0)
            elif key == ord('s'):  # S - Guardar screenshot
                timestamp = pd.Timestamp.now().strftime("%Y%m%d_%H%M%S")
                save_path = f"dashboard_{self.session_path.name}_{self.current_frame:06d}_{timestamp}.png"
                cv2.imwrite(save_path, dashboard)
                print(f" Screenshot guardado: {save_path}")
            elif key == ord('1'):  # 1 - Toggle detecciones
                self.show_detections = not self.show_detections
            elif key == ord('2'):  # 2 - Toggle segmentacion
                self.show_segmentation = not self.show_segmentation
            elif key == ord('3'):  # 3 - Toggle trayectoria
                self.show_trajectory = not self.show_trajectory
            elif key == 82:  # Flecha arriba - Aumentar velocidad
                self.play_speed = min(10.0, self.play_speed + 0.25)
                print(f"⚡ Velocidad: {self.play_speed:.2f}x")
            elif key == 84:  # Flecha abajo - Disminuir velocidad
                self.play_speed = max(0.25, self.play_speed - 0.25)
                print(f" Velocidad: {self.play_speed:.2f}x")
            elif key == 81:  # Flecha izquierda - Frame anterior
                self.current_frame = max(0, self.current_frame - 1)
                print(f" Frame: {self.current_frame + 1}/{len(self.frames_list)}")
            elif key == 83:  # Flecha derecha - Frame siguiente
                self.current_frame = min(len(self.frames_list) - 1, self.current_frame + 1)
                print(f" Frame: {self.current_frame + 1}/{len(self.frames_list)}")
            
            # Avanzar automáticamente si no está en pausa
            if not self.paused:
                # Acumular velocidad fraccionaria
                fractional_accumulator += self.play_speed
                
                # Si tenemos al menos 1 frame acumulado
                if fractional_accumulator >= 1.0:
                    frames_to_advance = int(fractional_accumulator)
                    fractional_accumulator -= frames_to_advance
                    
                    self.current_frame += frames_to_advance
                    
                    if self.current_frame >= len(self.frames_list):
                        self.current_frame = 0  # Loop al llegar al final
                        fractional_accumulator = 0.0
                        print(" Reiniciando desde el frame 0")
        
        cv2.destroyAllWindows()
        print("\n Dashboard cerrado")

def main():
    """Función principal."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Dashboard de visualización de navegación autónoma')
    parser.add_argument('session_path', type=str, nargs='?', default=None,
                       help='Ruta a la carpeta de la sesión (ej: data_logs/ruta6_20260106_134920)')
    parser.add_argument('--frame', type=int, default=0,
                       help='Frame inicial (default: 0)')
    
    args = parser.parse_args()
    
    # Si no se proporciona ruta, mostrar las disponibles
    if args.session_path is None:
        # Buscar sesiones disponibles
        base_path = Path("~/autonomous_navigation/src/saves/data_logs").expanduser()
        if base_path.exists():
            sessions = sorted([d for d in base_path.iterdir() if d.is_dir()])
            if sessions:
                print(" Sesiones disponibles:")
                for i, session in enumerate(sessions):
                    print(f"  [{i}] {session.name}")
                
                try:
                    choice = int(input("\nSeleccione el número de la sesión: "))
                    if 0 <= choice < len(sessions):
                        args.session_path = str(sessions[choice])
                    else:
                        print(" Selección inválida")
                        return
                except ValueError:
                    print(" Entrada inválida")
                    return
            else:
                print(" No se encontraron sesiones")
                return
        else:
            print(" No se encontró el directorio de sesiones")
            return
    
    # Verificar que la ruta existe
    session_path = Path(args.session_path)
    if not session_path.exists():
        # Intentar con la ruta completa
        full_path = Path("~/autonomous_navigation/src/saves/data_logs") / args.session_path
        full_path = full_path.expanduser()
        if full_path.exists():
            session_path = full_path
        else:
            print(f" Error: La ruta {args.session_path} no existe")
            print(f"   Buscada en: {session_path}")
            print(f"   Buscada en: {full_path}")
            return
    
    # Crear y ejecutar dashboard
    dashboard = NavigationDashboard(str(session_path))
    dashboard.current_frame = args.frame
    dashboard.run()

if __name__ == "__main__":
    main()