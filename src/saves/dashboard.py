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
import math

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

        self.show_ipm_visualization = False  # Por defecto mostrar segmentación
        self.show_ipm_bev = False  # Nueva opción para visualización BEV

        # Parámetros de calibración IPM
        self.camera_params = None
        self.load_calibration()  # Cargar calibración al inicializar

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
    
    def load_calibration(self):
        """Carga los parámetros de calibración de la cámara."""
        calibration_path = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration.json"
        
        try:
            with open(calibration_path, 'r') as f:
                calib = json.load(f)
            
            self.camera_params = {
                'height': float(calib['camera_height']),
                'pitch': math.radians(float(calib['camera_pitch'])),
                'fx': float(calib['intrinsics']['fx']),
                'fy': float(calib['intrinsics']['fy']),
                'cx': float(calib['intrinsics']['cx']),
                'cy': float(calib['intrinsics']['cy'])
            }
            
            print(" Parámetros de calibración cargados:")
            print(f"  • Altura: {self.camera_params['height']:.3f} m")
            print(f"  • Pitch: {math.degrees(self.camera_params['pitch']):.2f}°")
            print(f"  • Focales: fx={self.camera_params['fx']:.1f}, fy={self.camera_params['fy']:.1f}")
            print(f"  • Centro: cx={self.camera_params['cx']:.1f}, cy={self.camera_params['cy']:.1f}")
            
        except Exception as e:
            print(f" Error cargando calibración: {e}")
            print(" Usando parámetros por defecto...")
            
            # Valores por defecto (típicos para VGA)
            self.camera_params = {
                'height': 0.38,
                'pitch': math.radians(7.52),
                'fx': 574.1,
                'fy': 574.1,
                'cx': 320.0,
                'cy': 240.0
            }

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

        progress = self.get_frame_progress(frame_idx)

        video_idx = int(progress * (self.video_effective_frames - 1))
        video_idx = max(0, min(video_idx, self.video_effective_frames - 1))

        self.video_cap.set(cv2.CAP_PROP_POS_FRAMES, video_idx)
        ret, frame = self.video_cap.read()


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
        
        # ================== PROGRESO NORMALIZADO DEL ROBOT ==================
        self.control_progress = None

        if self.control_data is not None and \
        'timestamp' in self.control_data.columns and \
        'cmd_vel_linear_x' in self.control_data.columns:

            timestamps = self.control_data['timestamp'].to_numpy()
            speeds = self.control_data['cmd_vel_linear_x'].to_numpy()

            # Seguridad
            speeds = np.nan_to_num(speeds, nan=0.0)
            speeds = np.clip(speeds, 0.0, 1.0)

            progress = np.zeros(len(speeds))
            total_progress = 0.0

            for i in range(1, len(speeds)):
                dt = timestamps[i] - timestamps[i - 1]
                if dt < 0 or dt > 1.0:
                    dt = 0.0
                total_progress += speeds[i] * dt
                progress[i] = total_progress

            if total_progress > 0:
                self.control_progress = progress / total_progress
            else:
                self.control_progress = progress

            print(" Progreso normalizado del robot calculado")


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
    
    def get_frame_progress(self, frame_idx):
        """
        Devuelve el progreso físico real del robot (0–1)
        usando control_signals y cantidad de frames.
        """
        if self.control_progress is None or self.control_data is None:
            return frame_idx / max(1, len(self.frames_list) - 1)

        control_idx = self.get_proportional_control_idx(frame_idx)
        return float(self.control_progress[control_idx])


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

    def extract_lane_edge_adaptive(self, seg_mask):
        """
        Función común para extraer el borde del carril usando algoritmo adaptativo.
        Retorna: (coeffs, points_meters, horizon_data, angle_info)
        """
        if seg_mask is None:
            return None, np.array([]), [], None
        
        h, w = seg_mask.shape
        
        # ==================== PARÁMETROS ADAPTATIVOS ====================
        roi_ratio = 0.45
        base_ratio = 0.65
        range_ratio = 0.2
        
        # Calcular ROI absoluto
        roi_start = int(h * roi_ratio)
        roi_line_y = roi_start
        
        # 1. CALCULAR BASE_RATIO adaptativo
        right_half = seg_mask[roi_start:, w//2:w]
        lane_rows, _ = np.where(right_half == 2)
        road_rows, _ = np.where(right_half == 1)
        
        min_lane_row = np.min(lane_rows) if len(lane_rows) > 0 else h
        min_road_row = np.min(road_rows) if len(road_rows) > 0 else h
        min_row = min(min_lane_row, min_road_row)
        
        if min_row < h - roi_start:
            new_base_ratio = (roi_start + min_row) / h
            base_ratio = max(roi_ratio + 0.05, min(0.8, new_base_ratio))
        else:
            base_ratio = roi_ratio + 0.1
        
        # 2. CALCULAR RANGE_RATIO adaptativo
        problem_row_ratio = 0.9
        for row in range(roi_start, h, 10):
            row_slice = seg_mask[row:min(row+10, h), w//2:w]
            road_pixels = np.sum(row_slice == 1)
            total_pixels = row_slice.size
            density = road_pixels / total_pixels if total_pixels > 0 else 0
            
            if density > 0.7:
                problem_row_ratio = row / h
                break
        
        margin = 0.08
        max_allowed_range = max(0.05, problem_row_ratio - base_ratio - margin)
        range_ratio = min(0.25, max(0.08, max_allowed_range))
        
        # ==================== DETECCIÓN DE PUNTOS ====================
        num_horizons = 5
        min_points_per_horizon = 5
        horizon_data = []
        
        for i in range(num_horizons):
            ratio = i / max(1, num_horizons - 1)
            y_center = int(h * (base_ratio + ratio * range_ratio))
            
            if y_center < roi_start:
                continue
            
            slice_height = int(h * 0.06)
            y1 = max(roi_start, y_center - slice_height // 2)
            y2 = min(h, y_center + slice_height // 2)
            
            slice_mask = seg_mask[y1:y2, w//2:w]
            
            # Extraer borde robusto
            valid_pixels = (slice_mask == 1) | (slice_mask == 2)
            ys, xs = np.where(valid_pixels)
            
            if len(xs) < min_points_per_horizon:
                continue
            
            # Filtrar outliers
            x_low = np.percentile(xs, 5)
            x_high = np.percentile(xs, 95)
            valid_mask = (xs >= x_low) & (xs <= x_high)
            filtered_xs = xs[valid_mask]
            
            if len(filtered_xs) < min_points_per_horizon:
                continue
            
            # Percentil alto para borde derecho
            if len(filtered_xs) > 100:
                percentile = 97
            elif len(filtered_xs) > 50:
                percentile = 95
            else:
                percentile = 93
            
            edge_x = int(np.percentile(filtered_xs, percentile))
            
            # Confianza
            total_pixels = slice_mask.shape[0] * slice_mask.shape[1]
            density = len(filtered_xs) / total_pixels
            confidence = min(1.0, density * 12)
            
            global_x = edge_x + w // 2
            
            horizon_data.append({
                'horizon_idx': i,
                'y': y_center,
                'x': global_x,
                'confidence': confidence,
                'slice_bounds': (y1, y2),
                'distance_ratio': ratio,
            })
        
        # ==================== TRANSFORMACIÓN IPM ====================
        if self.camera_params is None:
            self.load_calibration()
        
        # Pre-calcular matriz de rotación
        cp = math.cos(self.camera_params['pitch'])
        sp = math.sin(self.camera_params['pitch'])
        R = np.array([
            [1,  0,   0],
            [0,  cp,  sp],
            [0, -sp,  cp]
        ])
        
        points_meters = []
        valid_horizon_data = []
        
        for h_data in horizon_data:
            u = h_data['x']
            v = h_data['y']
            
            # Transformación de píxeles a coordenadas cámara
            x = (u - self.camera_params['cx']) / self.camera_params['fx']
            y = -(v - self.camera_params['cy']) / self.camera_params['fy']
            ray = np.array([x, y, 1.0])
            
            # Rotar según pitch
            ray_w = R @ ray
            
            # Verificar que apunte al suelo
            if ray_w[1] >= 0:
                continue
            
            # Distancia al suelo
            t = self.camera_params['height'] / -ray_w[1]
            X_camera = ray_w[0] * t
            Z_camera = ray_w[2] * t
            
            # Filtrar por distancia (0.5m a 12m)
            if 0.5 <= Z_camera <= 12.0:
                points_meters.append((Z_camera, X_camera))
                valid_horizon_data.append(h_data)
        
        points_meters = np.array(points_meters)
        
        # ==================== AJUSTE POLINÓMICO Y ÁNGULO ====================
        coeffs = None
        angle_info = None
        
        if len(points_meters) >= 3:
            try:
                Z = points_meters[:, 0]
                X = points_meters[:, 1]
                coeffs = np.polyfit(Z, X, 2)
                
                # Calcular información del ángulo
                lookahead_distance = 3.0
                max_angle_error = 3.0
                max_angle_range = 170.0
                
                a, b, c = coeffs[0], coeffs[1], coeffs[2]
                slope = 2 * a * lookahead_distance + b
                lane_angle_rad = -math.atan(slope)
                lane_angle_deg = math.degrees(lane_angle_rad)
                
                # Calcular error de ángulo
                angle_error = 0.0
                if abs(lane_angle_deg) > max_angle_error:
                    angle_error = -lane_angle_deg
                
                angle_error_normalized = np.clip(angle_error / max_angle_range, -1.0, 1.0)
                
                angle_info = {
                    'angle_rad': lane_angle_rad,
                    'angle_deg': lane_angle_deg,
                    'error': angle_error_normalized,
                    'lookahead': lookahead_distance,
                    'max_error': max_angle_error
                }
                
            except Exception as e:
                print(f" Error ajustando polinomio: {e}")
                coeffs = None
        
        return coeffs, points_meters, valid_horizon_data, angle_info

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
        """Crea overlay de detecciones usando los datos YA GUARDADOS en tiempo real."""
        if image is None or not detections_data:
            return image
        
        overlay = image.copy()
        height, width = image.shape[:2]
        
        # Obtener velocidad del robot si está disponible
        robot_speed = 0.0
        if frame_stats and 'control_stats' in frame_stats:
            robot_speed = frame_stats['control_stats'].get('cmd_vel_linear_x', 0.0)
        
        # Verificar estructura de datos (según tu JSON)
        if isinstance(detections_data, dict) and 'objects' in detections_data:
            objects_list = detections_data['objects']
            source_text = "FUSED 3D"
            robot_speed_data = detections_data.get('robot_speed', robot_speed)
            if robot_speed_data > 0:
                robot_speed = robot_speed_data
        else:
            # Formato incompatible
            objects_list = []
            source_text = ""
        
        if not objects_list:
            return image
        
        # ==================== HISTORIAL DE OBJETOS (para trayectorias) ====================
        frame_idx = getattr(self, 'current_frame', 0)
        
        # Inicializar historial si no existe
        if not hasattr(self, 'object_history'):
            self.object_history = {}  # track_id -> [(frame_idx, center_x, center_y)]
        
        # ==================== PROCESAR OBJETOS ====================
        processed_objects = []
        
        for obj in objects_list:
            # Extraer bbox directamente (ya está en el formato correcto)
            bbox = obj.get('bbox', [])
            if len(bbox) != 4:
                continue
            
            x1, y1, x2, y2 = map(int, bbox)
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            # ==================== INFORMACIÓN DEL JSON ====================
            # Información básica
            class_name = obj.get('class_name', 'unknown')
            confidence = float(obj.get('confidence', 0.0))
            track_id = int(obj.get('track_id', 0))
            
            # Posición 3D (del JSON)
            position_3d = obj.get('position_3d', {})
            distance = float(position_3d.get('distance', 0.0))
            lateral_offset = float(position_3d.get('lateral_offset', 0.0))
            ground_distance = float(position_3d.get('ground_distance', 0.0))
            distance_valid = bool(position_3d.get('distance_valid', False))
            distance_source = int(position_3d.get('distance_source', 0))  # 1=depth, 2=IPM, 3=size
            
            # Velocidades (del JSON - ¡CUIDADO! Hay un bug en los datos)
            velocity_data = obj.get('velocity', {})
            speed = float(velocity_data.get('speed', 0.0))  # ¡Posiblemente incorrecto!
            velocity_x = float(velocity_data.get('x', 0.0))  # ¡Posiblemente incorrecto!
            velocity_y = float(velocity_data.get('y', 0.0))  # ¡Posiblemente incorrecto!
            relative_speed = float(velocity_data.get('relative_speed', 0.0))  # ¡Posiblemente incorrecto!
            
            # Estados (del JSON)
            state_data = obj.get('state', {})
            is_static = bool(state_data.get('is_static', False))
            is_moving_toward = bool(state_data.get('is_moving_toward', False))
            is_moving_away = bool(state_data.get('is_moving_away', False))
            is_in_path = bool(state_data.get('is_in_path', False))
            ttc = float(state_data.get('time_to_collision', 999.0))
            
            # Tracking (del JSON)
            tracking_data = obj.get('tracking', {})
            track_age = int(tracking_data.get('track_age', 0))
            quality_score = float(tracking_data.get('quality_score', 0.0))
            
            # ==================== CORREGIR DATOS IMPOSIBLES ====================
            # ¡Las velocidades en tu JSON son imposibles (239 m/s = 860 km/h)!
            # Esto indica un bug en el cálculo en tiempo real
            
            # Si la velocidad es imposible (> 50 m/s), mostrar como error
            speed_error = False
            if abs(speed) > 50.0:  # Más de 180 km/h es imposible
                speed_error = True
                speed = 0.0  # Resetear a 0
                velocity_x = 0.0
                velocity_y = 0.0
                relative_speed = 0.0
            
            # ==================== DETERMINAR COLOR Y TIPO ====================
            class_lower = class_name.lower()
            if 'person' in class_lower or 'pedestrian' in class_lower:
                obj_type = 'person'
                base_color = self.colors['person']
            elif any(v in class_lower for v in ['car', 'truck', 'bus', 'vehicle']):
                obj_type = 'vehicle'
                base_color = self.colors['car']
            else:
                obj_type = 'other'
                base_color = (128, 128, 128)
            
            # Color de caja según estado
            if ttc < 2.0:  # Crítico
                box_color = (0, 0, 255)  # Rojo
                thickness = 3
            elif ttc < 5.0:  # Peligro
                box_color = (0, 165, 255)  # Naranja
                thickness = 2
            elif is_in_path:  # En trayectoria
                box_color = (0, 255, 255)  # Amarillo
                thickness = 2
            elif is_static:  # Estático
                box_color = (0, 200, 0)  # Verde
                thickness = 1
            elif speed_error:  # Error en velocidad
                box_color = (255, 0, 255)  # Magenta (indica error)
                thickness = 2
            elif relative_speed < -0.3:  # Acercándose rápido
                box_color = (0, 0, 200)  # Rojo oscuro
                thickness = 2
            elif relative_speed > 0.3:  # Alejándose rápido
                box_color = (255, 100, 0)  # Azul
                thickness = 2
            else:
                box_color = base_color
                thickness = 1
            
            # ==================== GUARDAR EN HISTORIAL PARA TRAYECTORIAS ====================
            if track_id > 0:
                if track_id not in self.object_history:
                    self.object_history[track_id] = []
                
                self.object_history[track_id].append({
                    'frame_idx': frame_idx,
                    'center_x': center_x,
                    'center_y': center_y,
                    'distance': distance,
                    'class_name': class_name
                })
                
                # Mantener solo últimos 30 frames
                if len(self.object_history[track_id]) > 30:
                    self.object_history[track_id] = self.object_history[track_id][-30:]
            
            # ==================== ALMACENAR PARA PROCESAMIENTO ====================
            processed_objects.append({
                'bbox': bbox,
                'center_x': center_x,
                'center_y': center_y,
                'class_name': class_name,
                'confidence': confidence,
                'track_id': track_id,
                'distance': distance,
                'lateral_offset': lateral_offset,
                'ground_distance': ground_distance,
                'distance_valid': distance_valid,
                'distance_source': distance_source,
                'speed': speed,
                'velocity_x': velocity_x,
                'velocity_y': velocity_y,
                'relative_speed': relative_speed,
                'is_static': is_static,
                'is_moving_toward': is_moving_toward,
                'is_moving_away': is_moving_away,
                'is_in_path': is_in_path,
                'ttc': ttc,
                'track_age': track_age,
                'quality_score': quality_score,
                'obj_type': obj_type,
                'box_color': box_color,
                'thickness': thickness,
                'base_color': base_color,
                'speed_error': speed_error
            })
        
        # ==================== DIBUJAR TRAYECTORIAS ====================
        for track_id, history in self.object_history.items():
            if len(history) >= 2:
                points = [(h['center_x'], h['center_y']) for h in history]
                
                # Buscar color para este track
                track_color = (150, 150, 150)  # Gris por defecto
                for obj in processed_objects:
                    if obj['track_id'] == track_id:
                        track_color = obj['base_color']
                        break
                
                # Dibujar línea suavizada
                for i in range(len(points) - 1):
                    alpha = max(0.3, 1.0 - (i / len(points) * 0.7))
                    line_color = (
                        int(track_color[0] * alpha),
                        int(track_color[1] * alpha),
                        int(track_color[2] * alpha)
                    )
                    
                    cv2.line(overlay, points[i], points[i+1], 
                            line_color, 1, cv2.LINE_AA)
        
        # ==================== DIBUJAR OBJETOS ====================
        # Ordenar por distancia (más cercanos primero)
        processed_objects.sort(key=lambda x: x['distance'] if x['distance'] > 0 else float('inf'))
        
        for obj in processed_objects:
            x1, y1, x2, y2 = map(int, obj['bbox'])
            
            # 1. Bounding box
            cv2.rectangle(overlay, (x1, y1), (x2, y2), obj['box_color'], obj['thickness'])
            
            # 2. ID de track (esquina superior izquierda)
            if obj['track_id'] > 0:
                cv2.circle(overlay, (x1 + 8, y1 + 8), 6, (255, 255, 255), -1)
                cv2.putText(overlay, f"#{obj['track_id']}", (x1 + 12, y1 + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
            
            # 3. DISTANCIA GRANDE Y CLARA (esquina inferior derecha)
            if obj['distance_valid'] and obj['distance'] > 0:
                # Texto de distancia
                dist_text = f"{obj['distance']:.1f}m"
                
                # Color según distancia
                if obj['distance'] < 3.0:
                    dist_color = (0, 0, 255)  # Rojo: muy cerca
                elif obj['distance'] < 6.0:
                    dist_color = (0, 165, 255)  # Naranja: cerca
                elif obj['distance'] < 12.0:
                    dist_color = (0, 255, 255)  # Amarillo: media
                else:
                    dist_color = (0, 255, 0)  # Verde: lejos
                
                # Tamaño del texto
                font_scale = 0.6 if obj['distance'] < 10 else 0.5
                text_size = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)[0]
                
                # Fondo negro para mejor legibilidad
                cv2.rectangle(overlay, 
                            (x2 - text_size[0] - 10, y2 - text_size[1] - 10),
                            (x2 - 5, y2 - 5), (0, 0, 0), -1)
                
                # Texto de distancia
                cv2.putText(overlay, dist_text,
                        (x2 - text_size[0] - 5, y2 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, dist_color, 2)
            
            # 4. ETIQUETA SUPERIOR
            # Símbolo de fuente de distancia
            source_symbol = ""
            if obj['distance_source'] == 1:
                source_symbol = "D"  # Depth
            elif obj['distance_source'] == 2:
                source_symbol = "I"  # IPM
            elif obj['distance_source'] == 3:
                source_symbol = "N"  # Size
            
            main_label = f"{obj['class_name']} {obj['confidence']:.2f}"
            
            # Añadir distancia si es válida
            if obj['distance_valid'] and obj['distance'] > 0:
                main_label += f" | {obj['distance']:.1f}m{source_symbol}"
            
            # Añadir velocidad relativa (si no es error)
            if not obj['speed_error'] and abs(obj['relative_speed']) > 0.1:
                direction = "-" if obj['relative_speed'] < 0 else "+"
                main_label += f" | {direction}{abs(obj['relative_speed']):.1f}m/s"
            elif obj['speed_error']:
                main_label += " | VEL_ERROR"
            
            # Dibujar etiqueta
            label_size = cv2.getTextSize(main_label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
            cv2.rectangle(overlay, (x1, y1 - label_size[1] - 5),
                        (x1 + label_size[0], y1), obj['box_color'], -1)
            cv2.putText(overlay, main_label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # 5. INFORMACIÓN ADICIONAL (si hay espacio)
            if y2 + 40 < height:
                info_lines = []
                
                # Línea 1: Posición lateral
                if abs(obj['lateral_offset']) > 0.1:
                    side = "DER" if obj['lateral_offset'] > 0 else "IZQ"
                    info_lines.append(f"{side}:{abs(obj['lateral_offset']):.1f}m")
                
                # Línea 2: Estado de movimiento
                if obj['is_static']:
                    info_lines.append("ESTÁTICO")
                elif obj['is_moving_toward']:
                    info_lines.append("ACERCANDOSE")
                elif obj['is_moving_away']:
                    info_lines.append("ALEJANDOSE")
                
                # Línea 3: TTC si es relevante
                if obj['ttc'] < 10.0:
                    if obj['ttc'] < 2.0:
                        info_lines.append(f"TTC:{obj['ttc']:.1f}s")
                    elif obj['ttc'] < 5.0:
                        info_lines.append(f"TTC:{obj['ttc']:.1f}s")
                
                # Dibujar líneas
                line_height = 14
                for i, line in enumerate(info_lines[:3]):  # Máximo 3 líneas
                    line_y = y2 + 10 + (i * line_height)
                    if line_y + 10 < height:
                        # Fondo semi-transparente
                        text_size = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)[0]
                        overlay_copy = overlay.copy()
                        cv2.rectangle(overlay_copy, (x1, line_y - text_size[1] - 2),
                                    (x1 + text_size[0], line_y + 2), (0, 0, 0), -1)
                        overlay = cv2.addWeighted(overlay_copy, 0.6, overlay, 0.4, 0)
                        
                        cv2.putText(overlay, line, (x1, line_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 200), 1)
        
        # ==================== PANEL DE RESUMEN ====================
        summary_x = width - 220
        summary_y = 30
        
        # Fondo del panel
        cv2.rectangle(overlay, (summary_x - 10, summary_y - 25),
                    (width - 10, summary_y + 120), (0, 0, 0, 128), -1)
        overlay_copy = overlay.copy()
        cv2.rectangle(overlay_copy, (summary_x - 10, summary_y - 25),
                    (width - 10, summary_y + 120), (0, 0, 0), -1)
        overlay = cv2.addWeighted(overlay_copy, 0.5, overlay, 0.5, 0)
        
        # Título
        cv2.putText(overlay, f"ESTADISTICAS ({source_text})", 
                (summary_x, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        summary_y += 20
        
        # Contadores por tipo
        type_counts = {'person': 0, 'vehicle': 0, 'other': 0}
        for obj in processed_objects:
            type_counts[obj['obj_type']] += 1
        
        if type_counts['person'] > 0:
            cv2.putText(overlay, f" Personas: {type_counts['person']}", 
                    (summary_x, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 
                    self.colors['person'], 1)
            summary_y += 16
        
        if type_counts['vehicle'] > 0:
            cv2.putText(overlay, f" Vehiculos: {type_counts['vehicle']}", 
                    (summary_x, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 
                    self.colors['car'], 1)
            summary_y += 16
        
        # Contar objetos peligrosos
        """danger_count = sum(1 for obj in processed_objects if obj['ttc'] < 5.0)
        if danger_count > 0:
            cv2.putText(overlay, f" Peligrosos: {danger_count}", 
                    (summary_x, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 
                    (0, 0, 255), 1)"""
        
        return overlay
    
    def create_lane_edge_ipm_visualization(self, seg_mask, frame_data, original_image=None):
        """Crea visualización del algoritmo IPM para detección de borde del carril."""
        if seg_mask is None:
            # Crear imagen vacía si no hay datos
            empty_img = np.zeros((400, 600, 3), dtype=np.uint8)
            cv2.putText(empty_img, "No segmentation data", (50, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            return empty_img
        
        h, w = seg_mask.shape
        
        # Crear imagen de debug CON LA IMAGEN ORIGINAL COMO FONDO
        if original_image is not None:
            # Redimensionar la imagen original para que coincida con la máscara de segmentación
            if original_image.shape[:2] != (h, w):
                original_resized = cv2.resize(original_image, (w, h))
            else:
                original_resized = original_image
            
            # Usar la imagen original como fondo
            img = original_resized.copy()
            
            # Convertir a BGR si es necesario
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            # Si no hay imagen original, usar fondo negro (comportamiento original)
            img = np.full((h, w, 3), 0, dtype=np.uint8)
        
        # Colorear máscara de segmentación
        lane_pixels = (seg_mask == 2)
        road_pixels = (seg_mask == 1)
        img[lane_pixels] = [0, 100, 200]  # Azul para lane
        img[road_pixels] = [50, 50, 50]   # Gris para road
        
        # Usar la función común para extraer el borde (método adaptativo - lane+road)
        coeffs_adaptive, points_meters_adaptive, horizon_data_adaptive, angle_info_adaptive = self.extract_lane_edge_adaptive(seg_mask)
        
        # ==================== CONFIGURACIÓN ADAPTATIVA (LANE+ROAD) ====================
        # Parámetros ADAPTATIVOS (lane+road)
        roi_ratio_adaptive = 0.45
        base_ratio_adaptive = 0.65
        range_ratio_adaptive = 0.2
        
        # Calcular parámetros para visualización ADAPTATIVA
        roi_start_adaptive = int(h * roi_ratio_adaptive)
        right_half_adaptive = seg_mask[roi_start_adaptive:, w//2:w]
        
        lane_rows_adaptive, _ = np.where(right_half_adaptive == 2)
        road_rows_adaptive, _ = np.where(right_half_adaptive == 1)
        
        min_lane_row_adaptive = np.min(lane_rows_adaptive) if len(lane_rows_adaptive) > 0 else h
        min_road_row_adaptive = np.min(road_rows_adaptive) if len(road_rows_adaptive) > 0 else h
        min_row_adaptive = min(min_lane_row_adaptive, min_road_row_adaptive)
        
        if min_row_adaptive < h - roi_start_adaptive:
            new_base_ratio_adaptive = (roi_start_adaptive + min_row_adaptive) / h
            base_ratio_adaptive = max(roi_ratio_adaptive + 0.05, min(0.8, new_base_ratio_adaptive))
        else:
            base_ratio_adaptive = roi_ratio_adaptive + 0.1
        
        base_line_y_adaptive = int(h * base_ratio_adaptive)
        top_line_y_adaptive = int(h * (base_ratio_adaptive + range_ratio_adaptive))
        
        # ==================== CONFIGURACIÓN LANE-ONLY ====================
        # Parámetros LANE-ONLY (solo lane mask)
        roi_ratio_laneonly = 0.6
        base_ratio_laneonly = 0.7
        range_ratio_laneonly = 0.2
        
        # Calcular parámetros para visualización LANE-ONLY
        roi_start_laneonly = int(h * roi_ratio_laneonly)
        
        # Extraer puntos LANE-ONLY (similar al código de referencia)
        horizon_data_laneonly = []
        points_meters_laneonly = []
        
        for i in range(5):  # 5 horizontes como en el código de referencia
            ratio = i / 4.0  # 0 a 1
            
            # Usar base_ratio_laneonly y range_ratio_laneonly
            y_center = int(h * (base_ratio_laneonly + ratio * range_ratio_laneonly))
            
            # Verificar límites
            if y_center < roi_start_laneonly or y_center >= h - 10:
                continue
            
            slice_h = max(6, int(h * 0.02))  # 2% de altura
            y1 = max(roi_start_laneonly, y_center - slice_h // 2)
            y2 = min(h - 10, y_center + slice_h // 2)
            
            # Solo lane mask
            slice_mask = seg_mask[y1:y2, w//2:w]
            ys, xs = np.where(slice_mask == 2)
            
            if len(xs) < 3:
                continue
            
            # Borde MÁS A LA IZQUIERDA dentro de la mitad derecha
            edge_x_local = int(np.median(xs))
            edge_x = edge_x_local + w // 2
            
            # Transformar a metros
            point_3d = self._pixel_to_meters(edge_x, y_center)
            if point_3d is not None:
                X, Z = point_3d
                if 0.5 <= Z <= 12.0:
                    points_meters_laneonly.append((Z, X))
                    horizon_data_laneonly.append({
                        'x': edge_x,
                        'y': y_center,
                        'confidence': min(1.0, len(xs) / (slice_mask.shape[0] * slice_mask.shape[1]) * 10)
                    })
        
        points_meters_laneonly = np.array(points_meters_laneonly)
        
        # Ajustar polinomio para lane-only
        coeffs_laneonly = None
        angle_info_laneonly = None
        
        if len(points_meters_laneonly) >= 3:
            try:
                Z = points_meters_laneonly[:, 0]
                X = points_meters_laneonly[:, 1]
                coeffs_laneonly = np.polyfit(Z, X, 2)
                
                # Calcular información del ángulo para lane-only
                lookahead_distance = 3.0
                max_angle_error = 3.0
                max_angle_range = 170.0
                
                a, b, c = coeffs_laneonly[0], coeffs_laneonly[1], coeffs_laneonly[2]
                slope = 2 * a * lookahead_distance + b
                lane_angle_rad = -math.atan(slope)
                lane_angle_deg = math.degrees(lane_angle_rad)
                
                # Calcular error de ángulo
                angle_error = 0.0
                if abs(lane_angle_deg) > max_angle_error:
                    angle_error = -lane_angle_deg
                
                angle_error_normalized = np.clip(angle_error / max_angle_range, -1.0, 1.0)
                
                angle_info_laneonly = {
                    'angle_rad': lane_angle_rad,
                    'angle_deg': lane_angle_deg,
                    'error': angle_error_normalized,
                    'lookahead': lookahead_distance,
                    'max_error': max_angle_error
                }
                
            except Exception as e:
                coeffs_laneonly = None
        
        # ==================== DIBUJAR ROI Y LÍNEAS PARA AMBAS CONFIGURACIONES ====================
        
        # ROI ADAPTATIVA (línea amarilla)
        roi_line_y_adaptive = roi_start_adaptive
        cv2.line(img, (0, roi_line_y_adaptive), (w, roi_line_y_adaptive), 
                (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"ROI-ADAPT", (10, roi_line_y_adaptive - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # ROI LANE-ONLY (línea magenta)
        roi_line_y_laneonly = roi_start_laneonly
        cv2.line(img, (0, roi_line_y_laneonly), (w, roi_line_y_laneonly), 
                (255, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"ROI-LANE", (10, roi_line_y_laneonly - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        # LÍNEA DE BASE ADAPTATIVA (calculada adaptativamente)
        cv2.line(img, (0, base_line_y_adaptive), (w, base_line_y_adaptive),
                (255, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f"Base-Adapt", (w - 80, base_line_y_adaptive - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
        
        # ==================== DIBUJAR PUNTOS DETECTADOS ADAPTATIVOS ====================
        colors_adaptive = [(0, 255, 255), (0, 255, 128), (0, 255, 0), 
                (128, 255, 0), (255, 128, 0)]
        
        for i, h_data in enumerate(horizon_data_adaptive):
            color = colors_adaptive[min(i, len(colors_adaptive)-1)]
            confidence = h_data.get('confidence', 1.0)
            y1, y2 = h_data.get('slice_bounds', (0, 0))
            
            # Dibujar franja de búsqueda (solo contorno)
            cv2.rectangle(img, (w//2, y1), (w, y2), 
                        (color[0]//3, color[1]//3, color[2]//3), 1)
            
            # Punto detectado
            radius = int(4 + confidence * 4)
            cv2.circle(img, (h_data['x'], h_data['y']), radius, color, -1)
            cv2.circle(img, (h_data['x'], h_data['y']), radius+2, (255, 255, 255), 1)
            
            # Label
            text = f"H{i}"
            cv2.putText(img, text, (h_data['x'] + 10, h_data['y'] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
            
            # Mostrar distancias en metros
            if i < len(points_meters_adaptive):
                Z, X = points_meters_adaptive[i]
                dist_text = f"Z={Z:.1f}m, X={X:.2f}m"
                cv2.putText(img, dist_text, (w - 200, h_data['y']),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # ==================== DIBUJAR PUNTOS LANE-ONLY ====================
        for i, h_data in enumerate(horizon_data_laneonly):
            y = h_data['y']
            confidence = h_data.get('confidence', 1.0)
            x, y_pt = h_data['x'], h_data['y']
            
            # Punto detectado Lane-Only (diferente marcador - X magenta)
            radius = int(4 + confidence * 4)
            
            # Dibujar como X
            cv2.line(img, (x-radius, y_pt-radius), (x+radius, y_pt+radius), (255, 0, 255), 2)
            cv2.line(img, (x-radius, y_pt+radius), (x+radius, y_pt-radius), (255, 0, 255), 2)
            
            # Label
            text = f"L{i}"
            cv2.putText(img, text, (x + 10, y_pt - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 255), 1)
        
        # ==================== INFORMACIÓN TEXTUAL (IGUAL AL CÓDIGO DE REFERENCIA) ====================
        info_y = 30
        spacing = 22
        
        # Título
        cv2.putText(img, "ADAPTIVE LANE DETECTION - DUAL PARAMS", (20, info_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        info_y += spacing
        
        # Parámetros ADAPTATIVOS (lane+road)
        adaptive_text = f"ADAPTIVE: Base={base_ratio_adaptive:.3f} Range={range_ratio_adaptive:.3f}"
        cv2.putText(img, adaptive_text, (20, info_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)  # Amarillo
        info_y += spacing
        
        # Parámetros LANE-ONLY
        laneonly_text = f"LANE-ONLY: Base={base_ratio_laneonly:.3f} Range={range_ratio_laneonly:.3f}"
        cv2.putText(img, laneonly_text, (20, info_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)  # Magenta
        info_y += spacing
        
        # ROI adaptativo
        roi_adaptive_text = f"ROI-ADAPT: {roi_ratio_adaptive:.3f}"
        cv2.putText(img, roi_adaptive_text, (20, info_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        info_y += spacing
        
        # ROI lane-only
        roi_laneonly_text = f"ROI-LANE: {roi_ratio_laneonly:.3f}"
        cv2.putText(img, roi_laneonly_text, (20, info_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        info_y += spacing
        
        # ==================== MOSTRAR ÁNGULOS Y ERRORES PARA AMBAS CONFIGURACIONES ====================
        
        # Ángulo ADAPTATIVO
        if angle_info_adaptive is not None:
            angle_deg_adaptive = angle_info_adaptive['angle_deg']
            angle_error_adaptive = angle_info_adaptive['error']
            
            angle_text_adaptive = f"Angle-Adapt: {angle_deg_adaptive:+.1f}° @ {angle_info_adaptive['lookahead']}m"
            angle_color_adaptive = (0, 255, 0) if abs(angle_deg_adaptive) < angle_info_adaptive['max_error'] else (0, 165, 255)
            cv2.putText(img, angle_text_adaptive, (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, angle_color_adaptive, 1)
            info_y += spacing
            
            # Error de ángulo ADAPTATIVO
            angle_error_text_adaptive = f"Error-Adapt: {angle_error_adaptive:+.3f}"
            cv2.putText(img, angle_error_text_adaptive, (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 1)
            info_y += spacing
        
        # Ángulo LANE-ONLY
        if angle_info_laneonly is not None:
            angle_deg_laneonly = angle_info_laneonly['angle_deg']
            angle_error_laneonly = angle_info_laneonly['error']
            
            angle_text_laneonly = f"Angle-Lane: {angle_deg_laneonly:+.1f}° @ {angle_info_laneonly['lookahead']}m"
            angle_color_laneonly = (0, 255, 0) if abs(angle_deg_laneonly) < angle_info_laneonly['max_error'] else (0, 165, 255)
            cv2.putText(img, angle_text_laneonly, (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, angle_color_laneonly, 1)
            info_y += spacing
            
            # Error de ángulo LANE-ONLY
            angle_error_text_laneonly = f"Error-Lane: {angle_error_laneonly:+.3f}"
            cv2.putText(img, angle_error_text_laneonly, (20, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 1)
            info_y += spacing
        
        # ==================== INDICADOR DE DIRECCIÓN (USAR ADAPTATIVO COMO DEFAULT) ====================
        direction_text = "STRAIGHT"
        direction_color = (0, 255, 0)
        
        if angle_info_adaptive is not None:
            if abs(angle_info_adaptive['angle_deg']) > angle_info_adaptive['max_error']:
                if angle_info_adaptive['angle_deg'] > 0:
                    direction_text = "CORRECT LEFT"
                    direction_color = (0, 165, 255)
                else:
                    direction_text = "CORRECT RIGHT"
                    direction_color = (0, 100, 255)
        
        cv2.putText(img, direction_text, (20, info_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, direction_color, 2)
        info_y += spacing
        
        # ==================== FLECHA DE DIRECCIÓN ====================
        if angle_info_adaptive is not None:
            center_x = w // 2
            bottom_y = h - 20
            line_length = 100
            
            end_x = center_x - int(line_length * math.sin(angle_info_adaptive['angle_rad']))
            end_y = bottom_y - int(line_length * math.cos(angle_info_adaptive['angle_rad']))
            
            cv2.arrowedLine(img, (center_x, bottom_y), (end_x, end_y),
                        (0, 255, 0), 3, cv2.LINE_AA, tipLength=0.3)
            
            arrow_text = f"{abs(angle_info_adaptive['angle_deg']):.1f}°"
            text_size = cv2.getTextSize(arrow_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            cv2.putText(img, arrow_text, (end_x - text_size[0]//2, end_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # ==================== PANEL LATERAL CON ESTADÍSTICAS ====================
        panel_width = 250
        panel_x = w - panel_width - 10
        panel_y = 20
        panel_height = 140
        
        # Fondo del panel
        cv2.rectangle(img, (panel_x, panel_y), (panel_x + panel_width, panel_y + panel_height),
                    (40, 40, 40), -1)
        cv2.rectangle(img, (panel_x, panel_y), (panel_x + panel_width, panel_y + panel_height),
                    (100, 100, 100), 1)
        
        # Título del panel
        cv2.putText(img, "IPM DETECTION STATS", (panel_x + 10, panel_y + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Calcular estadísticas de la máscara
        total_pixels = h * w
        lane_pixel_count = np.sum(seg_mask == 2)
        road_pixel_count = np.sum(seg_mask == 1)
        
        lane_percent = (lane_pixel_count / total_pixels) * 100
        road_percent = (road_pixel_count / total_pixels) * 100
        
        # Mostrar estadísticas
        stats_y = panel_y + 40
        cv2.putText(img, f"Lane mask: {lane_pixel_count} px", 
                (panel_x + 10, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        stats_y += 18
        
        cv2.putText(img, f"Lane area: {lane_percent:.1f}%", 
                (panel_x + 10, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        stats_y += 18
        
        cv2.putText(img, f"Road mask: {road_pixel_count} px", 
                (panel_x + 10, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        stats_y += 18
        
        cv2.putText(img, f"Road area: {road_percent:.1f}%", 
                (panel_x + 10, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        stats_y += 18
        
        # Información de control si está disponible
        if frame_data and 'control_stats' in frame_data:
            speed = frame_data['control_stats'].get('cmd_vel_linear_x', 0.0)
            cv2.putText(img, f"Robot speed: {speed:.2f} m/s", 
                    (panel_x + 10, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 100), 1)
        
        return img
    
    # ==================== FUNCIÓN AUXILIAR PARA TRANSFORMACIÓN PÍXELES A METROS ====================
    def _pixel_to_meters(self, u, v):
        """Transforma un punto (u, v) de píxeles a coordenadas métricas."""
        if self.camera_params is None:
            return None
        
        # Rayo en coordenadas cámara
        x = (u - self.camera_params['cx']) / self.camera_params['fx']
        y = -(v - self.camera_params['cy']) / self.camera_params['fy']
        ray = np.array([x, y, 1.0])
        
        # Matriz de rotación para pitch
        cp = math.cos(self.camera_params['pitch'])
        sp = math.sin(self.camera_params['pitch'])
        
        R = np.array([
            [1,  0,   0],
            [0,  cp,  sp],
            [0, -sp,  cp]
        ])
        
        # Rotar según pitch
        ray_w = R @ ray
        
        # Condición: rayo debe apuntar hacia el suelo
        if ray_w[1] >= 0:
            return None
        
        # Distancia al suelo
        t = self.camera_params['height'] / -ray_w[1]
        X_camera = ray_w[0] * t
        Z_camera = ray_w[2] * t
        
        return X_camera, Z_camera

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
    
    def create_lane_error_visualization(self, seg_mask, frame_stats, original_image=None):
        """Crea visualización del error de carril usando algoritmo predictivo robusto con 2 configuraciones."""
        if seg_mask is None:
            # Crear imagen vacía si no hay datos
            empty_img = np.zeros((400, 600, 3), dtype=np.uint8)
            cv2.putText(empty_img, "No segmentation data", (50, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            return empty_img
        
        h, w = seg_mask.shape
        
        # Crear imagen de debug CON LA IMAGEN ORIGINAL COMO FONDO
        if original_image is not None:
            # Redimensionar la imagen original para que coincida con la máscara de segmentación
            if original_image.shape[:2] != (h, w):
                original_resized = cv2.resize(original_image, (w, h))
            else:
                original_resized = original_image
            
            # Usar la imagen original como fondo
            debug_img = original_resized.copy()
            
            # Convertir a BGR si es necesario
            if len(debug_img.shape) == 2:
                debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)
        else:
            # Si no hay imagen original, usar fondo negro (comportamiento original)
            debug_img = np.full((h, w, 3), 0, dtype=np.uint8)
        
        # Colorear máscara de segmentación SEMI-TRANSPARENTE sobre el fondo
        lane_pixels = (seg_mask == 2)
        road_pixels = (seg_mask == 1)
        
        # Crear máscaras de color semi-transparentes
        lane_overlay = np.zeros_like(debug_img, dtype=np.uint8)
        road_overlay = np.zeros_like(debug_img, dtype=np.uint8)
        
        # Aplicar colores con transparencia
        alpha_lane = 0.95  # 5% de transparencia para carriles
        alpha_road = 0.7   # 30% de transparencia para carretera
        
        lane_overlay[lane_pixels] = [0, 100, 200]  # Azul-naranja para carriles
        road_overlay[road_pixels] = self.colors['drivable']   # Gris para carretera
        
        # Aplicar overlay con transparencia
        debug_img = cv2.addWeighted(lane_overlay, alpha_lane, debug_img, 1, 0)
        debug_img = cv2.addWeighted(road_overlay, alpha_road, debug_img, 1, 0)
        
        # ==================== CONFIGURACIÓN 1 (ORIGINAL) ====================
        num_horizons_1 = 5
        ref_ratio_1 = 0.77  # 77% del ancho
        horizon_weights_1 = [0.001, 0.002, 0.04, 0.002, 0.001]
        y_range_1_start = 0.71
        y_range_1_end = 0.77
        
        horizon_data_1 = []
        horizon_errors_1 = []
        
        # Extraer puntos para Config 1 (igual al original)
        for i in range(num_horizons_1):
            ratio = (num_horizons_1 - i - 1) / num_horizons_1
            y_center = int(h * (y_range_1_start + ratio * (y_range_1_end - y_range_1_start)))
            
            slice_height = h // (num_horizons_1 * 15)
            y1 = max(0, y_center - slice_height)
            y2 = min(h, y_center + slice_height)
            
            # SOLO lado derecho
            slice_mask = seg_mask[y1:y2, w//2:w]
            
            # EXTRACCIÓN ROBUSTA
            lane_ys, lane_xs = np.where(slice_mask == 2)
            road_ys, road_xs = np.where(slice_mask == 1)
            
            all_xs = []
            all_ys = []
            
            lane_has_enough = lane_xs is not None and len(lane_xs) >= 5
            road_has_enough = road_xs is not None and len(road_xs) >= 5
            
            if lane_has_enough and road_has_enough:
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
                if lane_xs is not None:
                    all_xs.extend(lane_xs)
                    all_ys.extend(lane_ys)
                if road_xs is not None:
                    all_xs.extend(road_xs)
                    all_ys.extend(road_ys)
            
            if len(all_xs) < 5:
                continue
            
            all_xs = np.array(all_xs)
            all_ys = np.array(all_ys)
            
            # Filtrar outliers
            x_low = np.percentile(all_xs, 5)
            x_high = np.percentile(all_xs, 90)
            
            valid_mask = (all_xs >= x_low) & (all_xs <= x_high)
            filtered_xs = all_xs[valid_mask]
            filtered_ys = all_ys[valid_mask]
            
            if len(filtered_xs) < 5:
                continue
            
            # El "borde" es el percentil alto (lado derecho)
            edge_x = int(np.percentile(filtered_xs, 85))
            global_x = edge_x + w // 2
            
            # Calcular confianza
            total_pixels = slice_mask.shape[0] * slice_mask.shape[1]
            point_ratio = len(filtered_xs) / total_pixels
            confidence = min(1.0, point_ratio * 10)
            
            # Bonus por acuerdo
            agreement_bonus = 0.0
            if lane_xs is not None and road_xs is not None:
                if len(lane_xs) > 0 and len(road_xs) > 0:
                    lane_median = np.median(lane_xs)
                    road_median = np.median(road_xs)
                    diff = abs(lane_median - road_median)
                    if diff < 20:
                        agreement_bonus = 0.2
            
            confidence = min(1.0, point_ratio * 10 + agreement_bonus)
            
            horizon_data_1.append({
                'horizon_idx': i,
                'y': y_center,
                'x': global_x,
                'confidence': confidence,
                'slice_bounds': (y1, y2)
            })
        
        # ==================== CONFIGURACIÓN 2 (NUEVA) ====================
        num_horizons_2 = 5
        ref_ratio_2 = 0.75  # 75% del ancho
        horizon_weights_2 = [0.001, 0.002, 0.04, 0.002, 0.001]
        y_range_2_start = 0.75
        y_range_2_end = 0.95
        
        horizon_data_2 = []
        horizon_errors_2 = []
        
        # Extraer puntos para Config 2 (solo lane_mask, prioriza izquierda)
        for i in range(num_horizons_2):
            ratio = (num_horizons_2 - i - 1) / num_horizons_2
            y_center = int(h * (y_range_2_start + ratio * (y_range_2_end - y_range_2_start)))
            
            slice_height = h // (num_horizons_2 * 15)
            y1 = max(0, y_center - slice_height)
            y2 = min(h, y_center + slice_height)
            
            # SOLO lado derecho
            slice_mask = seg_mask[y1:y2, w//2:w]
            
            # Solo lane_mask
            lane_ys, lane_xs = np.where(slice_mask == 2)
            
            if len(lane_xs) < 5:
                continue
            
            # Priorizar puntos a la izquierda (percentil bajo)
            edge_x = int(np.percentile(lane_xs, 15))  # 15% = izquierda
            global_x = edge_x + w // 2
            
            # Confianza simple
            total_pixels = slice_mask.shape[0] * slice_mask.shape[1]
            point_ratio = len(lane_xs) / total_pixels
            confidence = min(1.0, point_ratio * 10)
            
            horizon_data_2.append({
                'horizon_idx': i,
                'y': y_center,
                'x': global_x,
                'confidence': confidence,
                'slice_bounds': (y1, y2),
                'config_num': 2
            })
        
        # ==================== CÁLCULO DE ERRORES ====================
        # Función para calcular error
        def calculate_error(horizon_data, ref_ratio, horizon_weights, w):
            if not horizon_data:
                return 0.0, []
            
            ref_x = int(w * ref_ratio)
            errors = []
            weighted_error = 0.0
            total_weight = 0.0
            
            # Detectar curva
            is_curve = False
            if len(horizon_data) >= 3:
                x_positions = [h['x'] for h in horizon_data]
                x_range = np.max(x_positions) - np.min(x_positions)
                normalized_variation = x_range / w
                is_curve = normalized_variation > 0.15
            
            # Ajuste en curvas
            if is_curve:
                curve_weight_boost = 1.5
                adjusted_weights = []
                
                for i, w_orig in enumerate(horizon_weights[:len(horizon_data)]):
                    if i < 2:  # Horizontes cercanos
                        adjusted_weights.append(w_orig * curve_weight_boost)
                    else:  # Horizontes lejanos
                        adjusted_weights.append(w_orig / curve_weight_boost)
                
                total = sum(adjusted_weights)
                weights = [w / total for w in adjusted_weights]
            else:
                weights = horizon_weights[:len(horizon_data)]
            
            # Calcular error ponderado
            for h_data, weight in zip(horizon_data, weights):
                detected_x = h_data['x']
                error = (ref_x - detected_x) / (w / 2.0)
                errors.append(error)
                
                effective_weight = weight * h_data['confidence']
                weighted_error += error * effective_weight
                total_weight += effective_weight
            
            if total_weight > 0:
                weighted_error = weighted_error / total_weight
                weighted_error = np.clip(weighted_error, -1.0, 1.0)
            else:
                weighted_error = 0.0
            
            return weighted_error, errors
        
        # Calcular errores para ambas configuraciones
        error_1, horizon_errors_1 = calculate_error(horizon_data_1, ref_ratio_1, horizon_weights_1, w)
        error_2, horizon_errors_2 = calculate_error(horizon_data_2, ref_ratio_2, horizon_weights_2, w)
        
        # Detectar curvas para ambas configuraciones
        is_curve_1 = False
        if len(horizon_data_1) >= 3:
            x_positions = [h['x'] for h in horizon_data_1]
            x_range = np.max(x_positions) - np.min(x_positions)
            normalized_variation = x_range / w
            is_curve_1 = normalized_variation > 0.15
        
        is_curve_2 = False
        if len(horizon_data_2) >= 3:
            x_positions = [h['x'] for h in horizon_data_2]
            x_range = np.max(x_positions) - np.min(x_positions)
            normalized_variation = x_range / w
            is_curve_2 = normalized_variation > 0.15
        
        # ==================== VISUALIZACIÓN ====================
        
        # Colores para Config 1 (igual al original)
        colors_config_1 = [
            (0, 255, 255),    # Cian (cercano)
            (0, 255, 128),    # Verde-amarillo
            (0, 255, 0),      # Verde
            (128, 255, 0),    # Verde-azul
            (255, 128, 0)     # Azul (lejano)
        ]
        
        # Color para Config 2
        color_config_2 = (255, 0, 255)  # Magenta
        
        # === DIBUJAR HORIZONTES CONFIG 1 ===
        for h_data in horizon_data_1:
            idx = h_data['horizon_idx']
            color = colors_config_1[min(idx, len(colors_config_1)-1)]
            confidence = h_data.get('confidence', 1.0)
            y = h_data['y']
            x = h_data['x']
            
            # Línea horizontal del horizonte
            alpha = int(80 * confidence)
            cv2.line(debug_img, (w//2, y), (w, y), (alpha, alpha, alpha), 1)
            
            # Punto detectado
            radius = int(4 + confidence * 4)
            cv2.circle(debug_img, (x, y), radius, color, -1)
            cv2.circle(debug_img, (x, y), radius+2, (255, 255, 255), 1)
            
            # Label
            text = f"H{idx}"
            if len(horizon_errors_1) > idx:
                text += f" ({horizon_errors_1[idx]:+.2f})"
            cv2.putText(debug_img, text, (x + 10, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
        
        # === DIBUJAR HORIZONTES CONFIG 2 ===
        for h_data in horizon_data_2:
            idx = h_data['horizon_idx']
            y = h_data['y']
            confidence = h_data.get('confidence', 1.0)
            x, y_pt = h_data['x'], h_data['y']
            
            # Punto detectado Config 2 (diferente marcador)
            radius = int(4 + confidence * 4)
            
            # Dibujar como X
            cv2.line(debug_img, (x-radius, y_pt-radius), (x+radius, y_pt+radius), color_config_2, 2)
            cv2.line(debug_img, (x-radius, y_pt+radius), (x+radius, y_pt-radius), color_config_2, 2)
            
            # Label
            text = f"C2-H{idx}"
            if len(horizon_errors_2) > idx:
                text += f" ({horizon_errors_2[idx]:+.2f})"
            cv2.putText(debug_img, text, (x + 10, y_pt - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color_config_2, 1)
        
        # === LÍNEAS DE REFERENCIA PARA AMBAS CONFIGURACIONES ===
        ref_x_1 = int(w * ref_ratio_1)
        ref_x_2 = int(w * ref_ratio_2)
        
        # Línea de referencia Config 1 (azul)
        cv2.line(debug_img, (ref_x_1, 0), (ref_x_1, h), (255, 0, 0), 2)
        cv2.putText(debug_img, f"REF_C1 ({int(ref_ratio_1*100)}%)", (ref_x_1 + 5, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Línea de referencia Config 2 (magenta)
        cv2.line(debug_img, (ref_x_2, 0), (ref_x_2, h), color_config_2, 2)
        cv2.putText(debug_img, f"REF_C2 ({int(ref_ratio_2*100)}%)", (ref_x_2 + 5, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_config_2, 2)
        
        # === INFORMACIÓN TEXTUAL ===
        y_text = 70
        spacing = 25
        
        # Error Config 1
        error_color_1 = (0, 255, 0) if abs(error_1) < 0.3 else (0, 200, 255) if abs(error_1) < 0.6 else (0, 0, 255)
        
        if error_1 > 0.05:
            dir_text_1 = "ALEJARSE"
        elif error_1 < -0.05:
            dir_text_1 = "ACERCARSE"
        else:
            dir_text_1 = "OK"
        
        cv2.putText(debug_img, f"Config 1 [ORIGINAL]: {error_1:+.3f} {dir_text_1}", 
                (20, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.6, error_color_1, 2)
        y_text += spacing
        
        # Error Config 2
        error_color_2 = (0, 255, 0) if abs(error_2) < 0.3 else (0, 200, 255) if abs(error_2) < 0.6 else (0, 0, 255)
        
        if error_2 > 0.05:
            dir_text_2 = "ALEJARSE"
        elif error_2 < -0.05:
            dir_text_2 = "ACERCARSE"
        else:
            dir_text_2 = "OK"
        
        cv2.putText(debug_img, f"Config 2 [NEW]: {error_2:+.3f} {dir_text_2}", 
                (20, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.6, error_color_2, 2)
        y_text += spacing
        
        # Curvas
        cv2.putText(debug_img, f"Curve: C1={'YES' if is_curve_1 else 'NO'} | C2={'YES' if is_curve_2 else 'NO'}", 
                (20, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        y_text += spacing
        
        # Puntos
        cv2.putText(debug_img, f"Points: C1={len(horizon_data_1)} | C2={len(horizon_data_2)}", 
                (20, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 255), 2)
        y_text += spacing
        
        # === BARRA DE ERROR COMBINADA ===
        # Decidir qué error mostrar (usar Config 1 como default)
        active_config = 1  # Puedes cambiar esto según lo necesites
        active_error = error_1 if active_config == 1 else error_2
        active_color = error_color_1 if active_config == 1 else error_color_2
        
        center_x = w // 2
        bar_x = int(center_x + active_error * w / 4)
        
        # Dibujar centro
        cv2.rectangle(debug_img, (center_x - 2, h-20), (center_x + 2, h), (255, 255, 255), -1)
        
        # Barra coloreada
        if abs(active_error) < 0.2:
            bar_color = (0, 255, 0)  # Verde: OK
        elif abs(active_error) < 0.5:
            bar_color = (0, 200, 255)  # Naranja: Moderado
        else:
            bar_color = (0, 0, 255)  # Rojo: Grande
        
        cv2.rectangle(debug_img, (bar_x - 5, h-35), (bar_x + 5, h-10), bar_color, -1)
        
        # Flecha indicando dirección
        if active_error > 0.05:  # Alejarse
            cv2.arrowedLine(debug_img, (bar_x, h-45), (bar_x + 15, h-45), (255, 255, 0), 2)
        elif active_error < -0.05:  # Acercarse
            cv2.arrowedLine(debug_img, (bar_x, h-45), (bar_x - 15, h-45), (255, 255, 0), 2)
        
        # === LEYENDA ===
        legend_y = h - 60
        cv2.putText(debug_img, "SIGN: (+)=Away (-)=Closer", (w - 250, legend_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        
        # Mostrar configuración activa
        cv2.putText(debug_img, f"Active: Config {active_config}", (w - 250, legend_y + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        
        # ==================== ESTADÍSTICAS ====================
        stats_y = h - 90
        if frame_stats and 'drivable_percent' in frame_stats:
            cv2.putText(debug_img, f"Drivable: {frame_stats['drivable_percent']:.1f}%", 
                    (w - 200, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['drivable'], 1)
            stats_y -= 20
        
        if frame_stats and 'lane_percent' in frame_stats:
            cv2.putText(debug_img, f"Lane: {frame_stats['lane_percent']:.1f}%", 
                    (w - 200, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 200), 1)
            stats_y -= 20
        
        if frame_stats and 'num_lane_contours' in frame_stats:
            cv2.putText(debug_img, f"Contours: {frame_stats['num_lane_contours']}", 
                    (w - 200, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
        
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


    def create_ipm_bev_visualization(self, seg_mask, frame_data):
        """Crea visualización BEV IDÉNTICA al LaneDebugNode en tiempo real."""
        if seg_mask is None:
            empty_img = np.zeros((400, 600, 3), dtype=np.uint8)
            cv2.putText(empty_img, "No segmentation data", (50, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            return empty_img
        
        # ==================== CRÍTICO: CALCULAR EL BEV EXACTO COMO EL NODO IPM ====================
        # Los parámetros DEBEN ser idénticos al nodo IPM
        scale = 40.0  # px/m (MISMO QUE LaneDebugNode)
        max_distance = 12.0  # m
        bev_w = 400  # Ancho (MISMO)   
        bev_h = int(max_distance * scale)  # Alto (MISMO)
        
        # Crear BEV en ESCALA DE GRISES (igual que el nodo IPM)
        bev = np.zeros((bev_h, bev_w), dtype=np.uint8)
        
        if self.camera_params is None:
            self.load_calibration()
        
        h, w = seg_mask.shape
        roi_start = int(h * 0.45)  # ROI 45% (MISMO que en nodo IPM)
        
        # ¡¡¡ESTOS SON LOS PARÁMETROS EXACTOS DEL NODO IPM!!!
        pitch = self.camera_params['pitch']
        h_cam = self.camera_params['height']
        fx = self.camera_params['fx']
        fy = self.camera_params['fy']
        cx = self.camera_params['cx']
        cy = self.camera_params['cy']
        
        # IMPORTANTE: El nodo IPM usa esta matriz EXACTA
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        R = np.array([
            [1,  0,   0],
            [0,  cp,  sp],  # sp sin signo negativo
            [0, -sp,  cp]   # -sp aquí
        ])
        v0 = roi_start
        # Procesar EXACTAMENTE igual que el nodo IPM
        for v in range(v0, h, 2):  # CADA 2 PÍXELES VERTICAL
            for u in range(0, w, 2):      # CADA 2 PÍXELES HORIZONTAL
                cls = seg_mask[v, u]
                if cls == 0:
                    continue
                
                # Transformación IDÉNTICA al nodo IPM
                x = (u - cx) / fx
                y = -(v - cy) / fy  # Y NEGATIVO (¡IMPORTANTE!)
                ray = np.array([x, y, 1.0])
                
                # Rotación
                ray_w = R @ ray
                
                # Condición: rayo debe apuntar hacia el suelo (Y negativo)
                if ray_w[1] >= 0:
                    continue
                
                # Distancia al suelo
                t = h_cam / -ray_w[1]
                X = ray_w[0] * t      # Lateral
                Z = ray_w[2] * t      # Adelante
                
                # Filtrar por distancia
                if Z <= 0 or Z > max_distance:
                    continue
                
                # Mapear a BEV (FÓRMULA ESTÁNDAR DEL SISTEMA)
                bx = int(bev_w / 2 + X * scale)
                by = int(bev_h - Z * scale)
                
                if 0 <= bx < bev_w and 0 <= by < bev_h:
                    # ¡VALORES DE GRIS IDÉNTICOS AL NODO IPM!
                    if cls == 2:      # Lane/carril
                        cv2.circle(bev, (bx, by), 3, int(255 if cls==2 else 180), -1)   # Blanco
                    elif cls == 1:    # Road/área transitable
                        bev[by, bx] = 180    # Gris medio
                    else:             # Otros
                        bev[by, bx] = 100    # Gris oscuro
        
        # ==================== AHORA AGREGAR LAS LÍNEAS COMO LaneDebugNode ====================
        # Convertir a color
        kernel = np.ones((3,3), np.uint8)
        bev = cv2.morphologyEx(bev, cv2.MORPH_CLOSE, kernel)
        bev_color = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)


        
        # Extraer información del borde (si existe)
        coeffs, points_meters, _, _ = self.extract_lane_edge_adaptive(seg_mask)
        
        # ==================== DIBUJAR LÍNEA DEL BORDE (si existe) ====================
        if coeffs is not None:
            # Crear path del borde (igual que en tiempo real)
            edge_path_points = []
            max_z = min(8.0, max_distance)
            
            for z in np.arange(0.5, max_z, 0.1):
                x_edge = np.polyval(coeffs, z)
                
                # FÓRMULA DE LaneDebugNode: px = W/2 - y * scale, py = H - x * scale
                # Donde y = -x_edge (porque Y positivo es izquierda)
                # Y x = z (adelante)
                px = int(bev_w / 2 + x_edge * scale)  # y negativo porque x_edge es lateral derecho
                py = int(bev_h - z * scale)
                
                if 0 <= px < bev_w and 0 <= py < bev_h:
                    edge_path_points.append((px, py))
            
            # Dibujar línea del borde (AMARILLO, igual que LaneDebugNode)
            if len(edge_path_points) > 1:
                pts_array = np.array(edge_path_points, dtype=np.int32)
                cv2.polylines(bev_color, [pts_array], False, (0, 255, 255), 3)
                
                # Etiqueta "RIGHT" (igual que LaneDebugNode)
                if edge_path_points:
                    cv2.putText(bev_color, "RIGHT", 
                            (edge_path_points[0][0] + 5, edge_path_points[0][1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # También dibujar puntos de ajuste
            for i, (Z, X) in enumerate(points_meters):
                px = int(bev_w / 2 - (-X) * scale)  # y negativo
                py = int(bev_h - Z * scale)
                
                if 0 <= px < bev_w and 0 <= py < bev_h:
                    cv2.circle(bev_color, (px, py), 4, (255, 0, 0), -1)  # Azul
        
        # ==================== ELEMENTOS VISUALES DE LaneDebugNode ====================
        # Eje central (robot) - línea verde
        cv2.line(bev_color, (bev_w//2, bev_h), (bev_w//2, 0), (0, 255, 0), 1)
        
        # Marcas métricas (cada metro)
        max_dist = int(bev_h / scale)
        for z in range(1, int(max_distance) + 1):
            y = int(bev_h - z * scale)
            if y > 0:
                cv2.line(bev_color, (0, y), (bev_w, y), (50, 50, 50), 1)
                if z % 2 == 0:
                    cv2.putText(bev_color, f"{z}m", (5, y-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # Robot (coche) en la parte inferior
        cv2.rectangle(bev_color, (bev_w//2-15, bev_h-10), (bev_w//2+15, bev_h), (0, 255, 255), -1)
        
        # ==================== INFORMACIÓN DEL SISTEMA ====================
        cv2.putText(bev_color, f"BEV: {bev_w}x{bev_h} | Scale: {scale} px/m", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Información de calibración
        if self.camera_params:
            pitch_deg = math.degrees(self.camera_params['pitch'])
            calib_text = f"Calib: Pitch={pitch_deg:.1f}°, H={self.camera_params['height']:.3f}m"
            cv2.putText(bev_color, calib_text, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 100), 1)
        
        # Información del polinomio
        if coeffs is not None:
            coeff_text = f"Poly: {coeffs[0]:.4f}z² + {coeffs[1]:.4f}z + {coeffs[2]:.4f}"
            cv2.putText(bev_color, coeff_text, (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 0), 1)
        
        # Leyenda (igual que LaneDebugNode)
        legend_y = bev_h - 80
        legend_items = [
            ("RIGHT", (0, 255, 255), "Borde de la calzada"),
            ("DET_POINTS", (255, 0, 0), "Puntos detección"),
            ("LANE", (255, 255, 255), "Carril (255)"),
            ("ROAD", (180, 180, 180), "Área transitable (180)"),
        ]
        
        for label, color, desc in legend_items:
            # Dibujar muestra de color
            if label in ["LANE", "ROAD"]:
                cv2.putText(bev_color, f"{label}: {desc}", (10, legend_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            else:
                cv2.rectangle(bev_color, (10, legend_y-10), (25, legend_y+5), color, -1)
                cv2.putText(bev_color, f"{label}: {desc}", (30, legend_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            legend_y += 20
        
        return bev_color


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
            if self.show_ipm_visualization:
                # Mostrar visualización IPM
                seg_img = self.create_lane_edge_ipm_visualization(
                    frame_data['segmentation'], 
                    frame_data,
                    frame_data.get('image')
                )
                title_text = "LANE EDGE IPM DETECTION"
            else:
                # Mostrar segmentación normal
                seg_img = self.create_segmentation_overlay(frame_data['image'].copy(), frame_data['segmentation'])
                title_text = "SEGMENTACION"
            
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
                
                status_text = title_text if self.show_segmentation else f"{title_text} (DESACTIVADA)"
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
                frame_data.get('perception_stats', {}),
                frame_data.get('image') 
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
         # Definir posiciones para la columna derecha ANTES de usarlas
        traj_width = min(600, col3_width)
        traj_height = 450
        traj_x = col3_x + (col3_width - traj_width) // 2
        
        # Asegurar que row1_y esté definida para esta sección
        if self.show_ipm_bev and 'segmentation' in frame_data:
            # Mostrar visualización BEV
            bev_img = self.create_ipm_bev_visualization(frame_data['segmentation'], frame_data)
            if bev_img is not None:
                bev_resized = cv2.resize(bev_img, (traj_width, traj_height))
                
                # Verificar que las dimensiones sean válidas
                if (row1_y >= 0 and row1_y + bev_resized.shape[0] <= dashboard.shape[0] and
                    traj_x >= 0 and traj_x + bev_resized.shape[1] <= dashboard.shape[1]):
                    
                    dashboard[row1_y:row1_y+bev_resized.shape[0], traj_x:traj_x+bev_resized.shape[1]] = bev_resized
                    
                    # Título para BEV - VERIFICAR QUE EL SLICE NO SEA VACÍO
                    if row1_y - title_height >= 0 and row1_y - title_height < row1_y:
                        title_bg = np.zeros((title_height, traj_width, 3), dtype=np.uint8)
                        title_bg[:] = (40, 40, 40)
                        cv2.putText(title_bg, "IPM BIRD'S EYE VIEW", 
                                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                        
                        # Solo asignar si el slice es válido
                        if title_bg.shape[0] > 0 and title_bg.shape[1] > 0:
                            dashboard[row1_y-title_height:row1_y, traj_x:traj_x+traj_width] = title_bg
                    else:
                        # Si el cálculo da negativo o cero, ajustar
                        adjusted_title_y = max(0, row1_y - title_height)
                        if adjusted_title_y < row1_y:
                            title_bg = np.zeros((row1_y - adjusted_title_y, traj_width, 3), dtype=np.uint8)
                            title_bg[:] = (40, 40, 40)
                            cv2.putText(title_bg, "IPM BIRD'S EYE VIEW", 
                                    (10, title_bg.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                            dashboard[adjusted_title_y:row1_y, traj_x:traj_x+traj_width] = title_bg
        
        elif self.show_trajectory:
            # Mostrar trayectoria normal
            trajectory_img = self.create_trajectory_plot(frame_data) 
            if trajectory_img is not None:
                traj_resized = cv2.resize(trajectory_img, (traj_width, traj_height))
                
                # Verificar que las dimensiones sean válidas
                if (row1_y >= 0 and row1_y + traj_resized.shape[0] <= dashboard.shape[0] and
                    traj_x >= 0 and traj_x + traj_resized.shape[1] <= dashboard.shape[1]):
                    
                    dashboard[row1_y:row1_y+traj_resized.shape[0], traj_x:traj_x+traj_resized.shape[1]] = traj_resized
                    
                    # Título para trayectoria - CON VERIFICACIÓN
                    if row1_y - title_height >= 0 and row1_y - title_height < row1_y:
                        title_bg = np.zeros((title_height, traj_width, 3), dtype=np.uint8)
                        title_bg[:] = (40, 40, 40)
                        cv2.putText(title_bg, "TRAYECTORIA", 
                                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                        
                        if title_bg.shape[0] > 0 and title_bg.shape[1] > 0:
                            dashboard[row1_y-title_height:row1_y, traj_x:traj_x+traj_width] = title_bg
        
        else:
            # Placeholder
            placeholder = np.zeros((traj_height, traj_width, 3), dtype=np.uint8)
            if self.show_ipm_bev:
                cv2.putText(placeholder, "BEV NO DISPONIBLE", (130, 225),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            else:
                cv2.putText(placeholder, "TRAYECTORIA DESACTIVADA", (130, 225),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # Verificar dimensiones
            if (row1_y >= 0 and row1_y + placeholder.shape[0] <= dashboard.shape[0] and
                traj_x >= 0 and traj_x + placeholder.shape[1] <= dashboard.shape[1]):
                
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
            elif key == ord('4'):  # 4 - Toggle visualización IPM
                self.show_ipm_visualization = not self.show_ipm_visualization
                print(f" Visualización IPM: {'ACTIVADA' if self.show_ipm_visualization else 'DESACTIVADA'}")

            elif key == ord('5'):  # 5 - Toggle visualización BEV
                self.show_ipm_bev = not self.show_ipm_bev
                print(f" Visualización BEV: {'ACTIVADA' if self.show_ipm_bev else 'DESACTIVADA'}")

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