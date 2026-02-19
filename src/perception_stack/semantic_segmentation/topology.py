#!/usr/bin/env python3
"""
TopologyAssigner - Nodo 2 de la arquitectura dividida
======================================================
- Se suscribe al tópico /lane/detected_lines (mensaje personalizado)
- Transforma líneas de base_footprint a odom
- Realiza calibración y asignación topológica
- Tracking y detección de líneas faltantes con detección de cambio de carril
- Publica líneas etiquetadas en odom
"""

import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque, defaultdict
import time
import math
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sklearn.cluster import DBSCAN

# Importar mensajes personalizados
from custom_interfaces.msg import LaneLines, LaneLine

class SystemState(Enum):
    """Estados del sistema"""
    CALIBRATING = 'CALIBRATING'
    TRACKING = 'TRACKING'
    LANE_CHANGE_TRANSITION = 'LANE_CHANGE_TRANSITION'

@dataclass
class LineModel:
    """Estructura para representar una línea de carril"""
    points: deque
    persistent_y: float  # Posición lateral en base_footprint
    confidence: float
    last_update: float
    angle: float
    length: float
    source_frames: int
    min_xy_distance: float = float('inf')
    relative_position: str = "UNKNOWN"  # LEFT, RIGHT, CENTER

class TopologyAssigner(Node):
    def __init__(self):
        super().__init__('topology_assigner')
        
        # =================== PARÁMETROS AJUSTADOS ===================
        self.declare_parameters('', [
            ('memory_size', 5000),
            ('calibration_frames', 50),
            ('min_calibration_samples', 30),
            
            ('curve_detection_threshold', 40.0),
            ('publish_rate', 10.0),
            ('debug', True),
            
            # PARÁMETROS DE AGRUPAMIENTO
            ('y_tolerance', 0.95),
            ('min_group_samples', 8),
            ('min_groups_for_calibration', 2),
            
            # PARÁMETROS PARA DETECCIÓN DE LÍNEAS FALTANTES
            ('missing_line_search_tolerance', 1.0),
            ('missing_line_validation_frames', 10),
            ('lane_width_estimate', 2.8),
            
            # PARÁMETROS PARA TRACKING CON DISTANCIA EN X,Y
            ('xy_distance_weight', 0.6),
            ('y_distance_weight', 0.4),
            ('max_xy_distance', 5.0),
            ('max_y_distance', 0.5),
            
            # NUEVOS PARÁMETROS PARA DETECCIÓN DE CAMBIO DE CARRIL
            ('lane_change_detection_threshold', 0.8),  # Umbral para detectar cambio
            ('lane_change_history_size', 5),        # Tamaño del historial
            ('min_confidence_for_lane_change', 0.7), # Confianza mínima
            
            # PARÁMETROS PARA SUAVIZADO DE PERSISTENT_Y
            ('persistent_y_smoothing_factor', 0.8),  # Factor de suavizado para persistent_y
            
            # PARÁMETROS PARA TRANSICIÓN DE CAMBIO DE CARRIL
            ('transition_max_y_distance', 1.0),       # Distancia Y máxima durante transición
            ('transition_xy_weight', 0.8),            # Ponderación XY durante transición
            ('transition_y_weight', 0.2),            # Ponderación Y durante transición
            ('transition_frames', 5),                # Frames en estado de transición
            
            # PARÁMETROS PARA LIMPIEZA DE MEMORIA
            ('memory_retention_factor', 0.1),        # Factor de retención de memoria (0.0 = limpiar todo, 1.0 = mantener todo)
            ('confidence_reduction_factor', 0.6),     # Factor de reducción de confianza al cambiar de carril
        ])
        
        # Modelo de carretera
        self.road_model: Dict[str, LineModel] = {}
        self.pubs = {}
        
        # Estados del sistema
        self.system_state = SystemState.CALIBRATING
        self.calibration_data = {
            'lines_buffer': deque(maxlen=self.get_parameter('calibration_frames').value),
            'robot_positions': deque(maxlen=20),
            'start_time': None,
            'frames_processed': 0,
            'y_positions_history': []
        }
        
        # Pose del robot
        self.robot_pose = {'x': 0., 'y': 0., 'yaw': 0.}
        self.robot_history = deque(maxlen=20)
        self.is_in_curve = False
        self.odom_ready = False
        
        # Detección de curvas
        self.curve_history = deque(maxlen=10)
        self.curve_confidence = 0.0
        
        # =================== SISTEMA DE DETECCIÓN DE LÍNEAS FALTANTES ===================
        self.expected_lines = {
            'INTERMEDIO': ['left_border', 'lane_dividing', 'right_lane', 'right_border'],
            'DERECHO': ['left_border', 'lane_dividing', 'right_lane'],
            'IZQUIERDO': ['lane_dividing', 'right_lane', 'right_border']
        }
        
        self.robot_lane_position = None
        self.missing_lines = []
        self.candidate_lines = {}
        self.missing_line_candidates_history = defaultdict(lambda: deque(maxlen=5))
        
        # ================
        # --- Estabilidad de yaw para calibración ---
        self.yaw_history = deque(maxlen=20)
        self.last_yaw_check_time = None

        
        # =================== SISTEMA DE DETECCIÓN DE CAMBIO DE CARRIL ===================
        self.robot_relative_position_history = deque(maxlen=self.get_parameter('lane_change_history_size').value)
        self.last_robot_relative_position = None
        self.current_robot_relative_position = None
        self.lane_change_detected = False
        self.topology_update_pending = False
        
        # Contador de frames en transición
        self.transition_frame_count = 0
        
        # =================== ROS ===================
        # Suscriptores y publishers
        self.create_subscription(
            LaneLines, 
            '/lane/detected_lines',
            self.cb_detected_lines, 
            10
        )
        
        self.create_subscription(
            Odometry, 
            '/odometry/local', 
            self.cb_odom, 
            10
        )
        
        self.main_publishers = {}
        main_labels = ['left_border', 'lane_dividing', 'right_lane', 'right_border']
        for label in main_labels:
            self.main_publishers[label] = self.create_publisher(
                PointCloud2, 
                f'/memory_local/{label}', 
                10
            )
        
        self.pub_calibration = self.create_publisher(
            Bool, 
            '/lane_topology/calibration_ready', 
            10
        )
        
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self.publish_memories
        )
        
        self.get_logger().info(" TopologyAssigner inicializado - Modo CALIBRATING")
        self.get_logger().info(" Suscrito a /lane/detected_lines")
        self.get_logger().info(" Esperando odometría y líneas detectadas...")
    
    # =================== ODOMETRÍA ===================
    def cb_odom(self, msg):
        self.odom_ready = True
        self.robot_pose['x'] = msg.pose.pose.position.x
        self.robot_pose['y'] = msg.pose.pose.position.y
        self.robot_pose['yaw'] = self.q2yaw(msg.pose.pose.orientation)
        self.robot_history.append(self.robot_pose.copy())

        self.robot_history.append(self.robot_pose.copy())
        self.yaw_history.append(self.robot_pose['yaw'])

    def is_yaw_stable(self,
                    max_std=0.087,        # rad ≈ 1.1°
                    min_samples=15):
        """
        Retorna True si el yaw dejó de moverse (magnetómetro estable)
        """
        if len(self.yaw_history) < min_samples:
            return False

        yaws = np.unwrap(np.array(self.yaw_history))
        yaw_std = np.std(yaws)

        if self.get_parameter('debug').value:
            self.get_logger().debug(f"Yaw std: {yaw_std:.4f} rad")

        return yaw_std < max_std

    
    # =================== PROCESAMIENTO DE LÍNEAS DETECTADAS ===================
    def cb_detected_lines(self, msg):
        if not self.odom_ready:
            self.get_logger().warn(" Odometría no disponible")
            return
        
        #  NO calibrar si el yaw aún se está moviendo
        if not self.is_yaw_stable():
            if self.get_parameter('debug').value:
                self.get_logger().debug(" Esperando estabilización del yaw...")
            return
        
        self.is_in_curve = msg.is_in_curve
        lines_base = self.convert_msg_to_lines(msg.lines)
        
        if not lines_base:
            return
        
        if not self.is_in_curve and lines_base:
            self.is_in_curve = self.detect_curve_from_msg(msg)
        
        if self.get_parameter('debug').value and len(lines_base) > 0:
            self.get_logger().debug(
                f"Frame {msg.frame_id}: "
                f"{len(lines_base)} líneas recibidas, "
                f"Curva: {self.is_in_curve}, "
                f"Estado: {self.system_state.value}"
            )
        
        if self.system_state == SystemState.CALIBRATING:
            self.calibration_step(lines_base)
        elif self.system_state in [SystemState.TRACKING, SystemState.LANE_CHANGE_TRANSITION]:
            lines_odom = self.transform_lines_to_odom(lines_base)
            self.track_and_assign_with_missing_detection(lines_odom)
            if lines_odom:
                self.detect_lane_change()
    
    def convert_msg_to_lines(self, lane_lines_msg):
        lines_base = []
        
        for i, lane_line in enumerate(lane_lines_msg):
            points = []
            for point in lane_line.points:
                points.append([point.x, point.y, point.z])
            
            line = {
                'id': i,
                'points': points,
                'mean_rel_y': lane_line.mean_rel_y,
                'length': lane_line.length,
                'angle': lane_line.angle,
                'perp_std': lane_line.perp_std,
                'direction': self.calculate_direction(lane_line.angle),
                'quality': lane_line.quality,
                'n_points': lane_line.n_points,
                'x_mean': lane_line.x_mean,
            }
            
            lines_base.append(line)
        
        return lines_base
    
    def calculate_direction(self, angle):
        return [math.cos(angle), math.sin(angle)]
    
    def detect_curve_from_msg(self, msg):
        if not msg.lines or len(msg.lines) < 2:
            current_detection = False
        else:
            angles = []
            for lane_line in msg.lines:
                angle = lane_line.angle
                if angle > math.pi/2:
                    angle -= math.pi
                elif angle < -math.pi/2:
                    angle += math.pi
                angles.append(angle)
            
            if len(angles) >= 2:
                angle_std = np.std(angles)
                curve_threshold = math.radians(15)
                curved_lines = sum(1 for a in angles if abs(a) > math.radians(10))
                total_lines = len(angles)
                current_detection = (
                    angle_std > curve_threshold or 
                    curved_lines / total_lines > 0.5
                )
            else:
                current_detection = False
        
        self.curve_history.append(current_detection)
        
        if len(self.curve_history) >= 5:
            recent = list(self.curve_history)[-5:]
            self.curve_confidence = sum(1 for h in recent if h) / 5.0
        
        final_detection = (current_detection and self.curve_confidence > 0.6)
        
        return final_detection
    
    # =================== CALIBRACIÓN ===================
    def calibration_step(self, lines_base):
        if not lines_base:
            return
        
        if self.calibration_data['start_time'] is None:
            self.calibration_data['start_time'] = time.time()
            self.get_logger().info(" Iniciando calibración con líneas detectadas...")
        
        self.calibration_data['lines_buffer'].append(lines_base)
        self.calibration_data['frames_processed'] += 1
        
        for line in lines_base:
            self.calibration_data['y_positions_history'].append(line['mean_rel_y'])
        
        min_frames = 30
        if len(self.calibration_data['lines_buffer']) < min_frames:
            if self.get_parameter('debug').value:
                self.get_logger().debug(
                    f"Acumulando frames: {len(self.calibration_data['lines_buffer'])}/{min_frames}"
                )
            return
        
        all_lines = []
        for frame_lines in self.calibration_data['lines_buffer']:
            all_lines.extend(frame_lines)
        
        min_samples = self.get_parameter('min_calibration_samples').value
        if len(all_lines) < min_samples:
            if self.get_parameter('debug').value:
                self.get_logger().debug(
                    f"Esperando más muestras: {len(all_lines)}/{min_samples}"
                )
            return
        
        self.get_logger().info(
            f" Calibrando con {len(all_lines)} líneas de "
            f"{len(self.calibration_data['lines_buffer'])} frames"
        )
        
        line_groups = self.cluster_lines_by_lateral_position_debug(all_lines)
        self.get_logger().info(f" Grupos encontrados: {len(line_groups)}")
        
        min_groups = self.get_parameter('min_groups_for_calibration').value
        if len(line_groups) < min_groups:
            self.get_logger().warn(
                f" Se necesitan al menos {min_groups} líneas, "
                f"solo se encontraron {len(line_groups)}"
            )
            return
        
        line_groups.sort(key=lambda g: g['mean_y'], reverse=True)
        topology = self.determine_lane_topology_flexible(line_groups)
        
        if topology is None:
            self.get_logger().warn(" No se pudo determinar la topología del carril")
            return
        
        self.build_road_model(line_groups, topology)
        
        self.system_state = SystemState.TRACKING
        self.get_logger().info(" CALIBRACIÓN COMPLETADA - Modo TRACKING activado")
        
        msg = Bool()
        msg.data = True
        self.pub_calibration.publish(msg)
    
    def cluster_lines_by_lateral_position_debug(self, all_lines):
        if not all_lines:
            return []
        
        y_positions = np.array([line['mean_rel_y'] for line in all_lines])
        qualities = np.array([line.get('quality', 0.5) for line in all_lines])
        n_points = np.array([line.get('n_points', 0) for line in all_lines])
        
        if len(y_positions) < 20:
            groups = []
            sorted_indices = np.argsort(y_positions)[::-1]
            
            for idx in sorted_indices:
                y = y_positions[idx]
                placed = False
                
                for group in groups:
                    if abs(y - group['mean_y']) < self.get_parameter('y_tolerance').value * 1.5:
                        group['y_list'].append(y)
                        group['indices'].append(idx)
                        group['mean_y'] = np.median(group['y_list'])
                        placed = True
                        break
                
                if not placed:
                    groups.append({
                        'y_list': [y],
                        'indices': [idx],
                        'mean_y': y
                    })
            
            result_groups = []
            for group in groups:
                if len(group['indices']) >= 3:
                    indices = group['indices']
                    result_groups.append({
                        'mean_y': float(group['mean_y']),
                        'quality': float(np.mean(qualities[indices])),
                        'n_lines': len(indices),
                        'total_points': int(np.sum(n_points[indices])),
                        'best_line': all_lines[indices[np.argmax(qualities[indices])]],
                        'std_y': float(np.std([all_lines[i]['mean_rel_y'] for i in indices])),
                        'lines': [all_lines[i] for i in indices]
                    })
            
            return result_groups
        
        clustering = DBSCAN(
            eps=self.get_parameter('y_tolerance').value,
            min_samples=1
        ).fit(y_positions.reshape(-1, 1))
        
        labels = clustering.labels_
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        
        groups = []
        for cluster_id in range(n_clusters):
            mask = labels == cluster_id
            n_samples = np.sum(mask)
            
            if n_samples < 3:
                continue
            
            cluster_y = y_positions[mask]
            cluster_qualities = qualities[mask]
            cluster_n_points = n_points[mask]
            cluster_lines = [all_lines[i] for i in range(len(all_lines)) if mask[i]]
            
            mean_y = float(np.median(cluster_y))
            group_quality = float(np.mean(cluster_qualities))
            total_points = int(np.sum(cluster_n_points))
            best_line_idx = np.argmax(cluster_qualities)
            best_line = cluster_lines[best_line_idx]
            
            groups.append({
                'mean_y': mean_y,
                'quality': group_quality,
                'n_lines': n_samples,
                'total_points': total_points,
                'best_line': best_line,
                'std_y': float(np.std(cluster_y)),
                'lines': cluster_lines
            })
        
        filtered_groups = []
        for group in sorted(groups, key=lambda g: g['mean_y'], reverse=True):
            too_close = False
            for existing in filtered_groups:
                if abs(group['mean_y'] - existing['mean_y']) < 0.35:
                    too_close = True
                    if group['quality'] > existing['quality']:
                        filtered_groups.remove(existing)
                        filtered_groups.append(group)
                    break
            
            if not too_close:
                filtered_groups.append(group)
        
        return filtered_groups
    
    def determine_lane_topology_flexible(self, line_groups):
        n_lines = len(line_groups)
        self.get_logger().info(f" Determinando topología con {n_lines} líneas")
        
        line_groups.sort(key=lambda g: g['mean_y'], reverse=True)
        y_positions = [g['mean_y'] for g in line_groups]
        
        left_count = sum(1 for y in y_positions if y > 0.2)
        right_count = sum(1 for y in y_positions if y < -0.2)
        
        self.get_logger().info(
            f" Líneas izquierdas (Y>0.2): {left_count}, "
            f"derechas (Y<-0.2): {right_count}"
        )
        
        robot_position = None
        
        if left_count == 2 and right_count == 2:
            robot_position = 'INTERMEDIO'
        elif left_count == 2 and right_count == 1:
            robot_position = 'INTERMEDIO'
        elif left_count == 1 and right_count == 2:
            robot_position = 'INTERMEDIO'
        elif left_count == 3 and right_count == 1:
            robot_position = 'DERECHO'
        elif left_count == 3 and right_count == 0:
            robot_position = 'DERECHO'
        elif left_count == 1 and right_count == 3:
            robot_position = 'IZQUIERDO'
        elif left_count == 0 and right_count == 3:
            robot_position = 'IZQUIERDO'
        elif left_count == 1 and right_count == 1:
            robot_position = 'INTERMEDIO'
        elif left_count == 2 and right_count == 0:
            robot_position = 'INTERMEDIO'

        elif left_count == 1 and right_count == 0:
            robot_position = 'INTERMEDIO'



        self.get_logger().info(f" Robot en carril: {robot_position}")
        self.robot_lane_position = robot_position
        
        topology = None
        
        if robot_position == 'INTERMEDIO':
            if n_lines == 4:
                topology = {
                    'left_border': line_groups[0],
                    'lane_dividing': line_groups[1],
                    'right_lane': line_groups[2],
                    'right_border': line_groups[3]
                }
            elif n_lines == 3:
                if left_count == 2 and right_count == 1:
                    topology = self.analyze_3_lines_intermediate_left2_right1(line_groups)
                elif left_count == 1 and right_count == 2:
                    topology = self.analyze_3_lines_intermediate_left1_right2(line_groups)
                else:
                    topology = {
                        'left_border': line_groups[0],
                        'lane_dividing': line_groups[1],
                        'right_lane': line_groups[2]
                    }
            elif n_lines == 2 and left_count == 2:
                topology = {
                    'left_border': line_groups[0],
                    'lane_dividing': line_groups[1],
                    
                }
            elif n_lines == 2:
                topology = {
                    'lane_dividing': line_groups[0],
                    'right_lane': line_groups[1]
                }
            elif n_lines == 1:
                topology = {
                    'lane_dividing': line_groups[0]
                }
        
        elif robot_position == 'DERECHO':
            if n_lines == 4:
                topology = {
                    'left_border': line_groups[0],
                    'lane_dividing': line_groups[1],
                    'right_lane': line_groups[2],
                    'right_border': line_groups[3]
                }
            elif n_lines == 3:
                topology = {
                    'left_border': line_groups[0],
                    'lane_dividing': line_groups[1],
                    'right_lane': line_groups[2]
                }
            elif n_lines == 2:
                topology = {
                    'lane_dividing': line_groups[0],
                    'right_lane': line_groups[1]
                }
            elif n_lines == 1:
                topology = {
                    'lane_dividing': line_groups[0]
                }
        
        elif robot_position == 'IZQUIERDO':
            if n_lines == 4:
                topology = {
                    'left_border': line_groups[0],
                    'lane_dividing': line_groups[1],
                    'right_lane': line_groups[2],
                    'right_border': line_groups[3]
                }
            elif n_lines == 3:
                topology = {
                    'lane_dividing': line_groups[0],
                    'right_lane': line_groups[1],
                    'right_border': line_groups[2]
                }
            elif n_lines == 2:
                topology = {
                    'right_lane': line_groups[0],
                    'right_border': line_groups[1]
                }
            elif n_lines == 1:
                topology = {
                    'right_lane': line_groups[0]
                }
        
        if topology is not None and robot_position in self.expected_lines:
            expected = self.expected_lines[robot_position]
            current = list(topology.keys())
            self.missing_lines = [line for line in expected if line not in current]
            
            if self.missing_lines and self.get_parameter('debug').value:
                self.get_logger().info(
                    f" Líneas faltantes detectadas: {self.missing_lines}"
                )
        
        return topology
    
    def analyze_3_lines_intermediate_left2_right1(self, line_groups):
        y0 = line_groups[0]['mean_y']
        y1 = line_groups[1]['mean_y']
        y2 = line_groups[2]['mean_y']
        
        d_left = abs(y0 - y1)
        d_right = abs(y1 - y2)
        lane_width_est = d_left
        tol = 1.0
        
        self.get_logger().debug(
            f"[3L L2-R1] d_left={d_left:.2f}, d_right={d_right:.2f}"
        )
        
        if abs(d_right - lane_width_est) < tol * lane_width_est:
            return {
                'left_border': line_groups[0],
                'lane_dividing': line_groups[1],
                'right_lane': line_groups[2]
            }
        else:
            return {
                'left_border': line_groups[0],
                'lane_dividing': line_groups[1],
                'right_border': line_groups[2]
            }
    
    def analyze_3_lines_intermediate_left1_right2(self, line_groups):
        y0 = line_groups[0]['mean_y']
        y1 = line_groups[1]['mean_y']
        y2 = line_groups[2]['mean_y']
        
        d_right = abs(y1 - y2)
        d_left = abs(y0 - y1)
        lane_width_est = d_right
        tol = 0.35
        
        self.get_logger().debug(
            f"[3L L1-R2] d_left={d_left:.2f}, d_right={d_right:.2f}"
        )
        
        if abs(d_left - lane_width_est) < tol * lane_width_est:
            return {
                'lane_dividing': line_groups[0],
                'right_lane': line_groups[1],
                'right_border': line_groups[2]
            }
        else:
            return {
                'left_border': line_groups[0],
                'right_lane': line_groups[1],
                'right_border': line_groups[2]
            }
    
    def build_road_model(self, line_groups, topology):
        self.road_model.clear()
        
        for label, group_info in topology.items():
            best_line = group_info['best_line']
            
            pts_odom = [
                self.transform_point_to_odom(x, y, z)
                for x, y, z in best_line['points']
            ]
            
            min_xy_dist = self.min_distance_robot_to_line(pts_odom)
            
            # Determinar posición relativa inicial
            relative_pos = self.determine_relative_position(best_line['mean_rel_y'])
            
            line_model = LineModel(
                points=deque(pts_odom, maxlen=self.get_parameter('memory_size').value),
                persistent_y=group_info['mean_y'],
                confidence=group_info['quality'],
                last_update=time.time(),
                angle=best_line['angle'],
                length=best_line['length'],
                source_frames=group_info['n_lines'],
                min_xy_distance=min_xy_dist,
                relative_position=relative_pos
            )
            
            self.road_model[label] = line_model
        
        self.expected_road_labels = ['left_border', 'lane_dividing', 'right_lane', 'right_border']
        self.missing_lines = [
            label for label in self.expected_road_labels
            if label not in self.road_model
        ]
        
        if self.missing_lines:
            self.get_logger().info(
                f" Estructura vial incompleta. Líneas faltantes: {self.missing_lines}"
            )
    
    # =================== TRANSFORMACIÓN A ODOM ===================
    def transform_lines_to_odom(self, lines_base):
        if not lines_base:
            return []
        
        lines_odom = []
        
        for line in lines_base:
            pts_odom = [
                self.transform_point_to_odom(x, y, z)
                for x, y, z in line['points']
            ]
            
            lines_odom.append({
                'points': pts_odom,
                'mean_rel_y': line['mean_rel_y'],
                'quality': line.get('quality', 0.5),
                'angle': line['angle'],
                'length': line['length']
            })
        
        return lines_odom
    
    def transform_point_to_odom(self, x, y, z):
        xr = self.robot_pose['x']
        yr = self.robot_pose['y']
        yaw = self.robot_pose['yaw']
        
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        
        xo = xr + x * cy - y * sy
        yo = yr + x * sy + y * cy
        
        return (xo, yo, z)
    
    def min_distance_robot_to_line(self, line_points):
        if not line_points:
            return float('inf')
        
        robot_x = self.robot_pose['x']
        robot_y = self.robot_pose['y']
        
        min_dist = float('inf')
        for point in line_points:
            dist = math.sqrt((robot_x - point[0])**2 + (robot_y - point[1])**2)
            if dist < min_dist:
                min_dist = dist
        
        return min_dist
    
    # =================== NUEVO: DETECCIÓN DE CAMBIO DE CARRIL ===================
    def determine_relative_position(self, line_y):
        """Determina la posición relativa de una línea al robot"""
        if line_y > 0.5:
            return "LEFT"
        elif line_y < -0.5:
            return "RIGHT"
        else:
            return "CENTER"
    
    def calculate_robot_relative_position(self, lines_data):
        """
        Calcula la posición relativa del robot basado en las líneas detectadas.
        lines_data puede ser una lista de diccionarios (lines_odom) o una lista de LineModel.
        """
        if not lines_data:
            return self.last_robot_relative_position
        
        # Extraer mean_rel_y de cada línea, ya sea de diccionario o LineModel
        line_y_positions = []
        for line in lines_data:
            if isinstance(line, dict):
                y = line['mean_rel_y']
            else:  # LineModel
                y = line.persistent_y
            line_y_positions.append(y)
        
        # Ordenar líneas por posición Y
        sorted_lines = sorted(zip(line_y_positions, lines_data), key=lambda x: x[0], reverse=True)
        
        left_lines = [l[0] for l in sorted_lines if l[0] > 0.3]
        right_lines = [l[0] for l in sorted_lines if l[0] < -0.3]
        
        n_left = len(left_lines)
        n_right = len(right_lines)
        
        # Determinar posición relativa del robot
        if n_left >= 2 and n_right >= 2:
            return "CENTER"
        elif n_left >= 2 and n_right <= 1:
            return "RIGHT"
        elif n_left <= 1 and n_right >= 2:
            return "LEFT"
        else:
            # Si no hay suficientes líneas, usar la última posición conocida
            return self.last_robot_relative_position
    
    def detect_lane_change(self):
        """Detecta cambios de carril basados en la posición relativa del robot"""
        if not self.road_model:
            return
        
        # Preparar datos para calcular posición relativa
        lines_data = list(self.road_model.values())
        
        # Calcular posición relativa actual
        self.current_robot_relative_position = self.calculate_robot_relative_position(lines_data)
        
        # Si hay una posición anterior, comparar
        if self.last_robot_relative_position is not None:
            # Detectar cambio si la posición ha cambiado
            if self.current_robot_relative_position != self.last_robot_relative_position:
                self.get_logger().info(
                    f" Posición relativa cambiada: {self.last_robot_relative_position} -> {self.current_robot_relative_position}"
                )
                
                # Si no estamos en transición, iniciar transición
                if self.system_state == SystemState.TRACKING:
                    self.system_state = SystemState.LANE_CHANGE_TRANSITION
                    self.transition_frame_count = 0
                    self.get_logger().info(" Iniciando transición de cambio de carril")
        
        # Actualizar posición anterior
        self.last_robot_relative_position = self.current_robot_relative_position
        self.robot_relative_position_history.append(self.current_robot_relative_position)
        
        # Manejar estado de transición
        if self.system_state == SystemState.LANE_CHANGE_TRANSITION:
            self.transition_frame_count += 1
            
            # Si la transición ha durado suficiente, confirmar el cambio
            if self.transition_frame_count >= self.get_parameter('transition_frames').value:
                self.handle_lane_change()
                self.system_state = SystemState.TRACKING
                self.transition_frame_count = 0
                self.get_logger().info(" Transición completada, volviendo a TRACKING")
    
    def handle_lane_change(self):
        """Maneja el cambio de carril actualizando la topología"""
        self.get_logger().info(f" Cambio de carril detectado: {self.last_robot_relative_position} -> {self.current_robot_relative_position}")
        
        # Actualizar posición del robot
        if self.current_robot_relative_position == "LEFT":
            self.robot_lane_position = "IZQUIERDO"
        elif self.current_robot_relative_position == "RIGHT":
            self.robot_lane_position = "DERECHO"
        else:
            self.robot_lane_position = "INTERMEDIO"
        
        self.get_logger().info(f" Robot en carril: {self.robot_lane_position}")
        
        # Limpiar memorías antes de reetiquetar
        self.clean_memories_for_lane_change()
        
        # Actualizar topología basada en la nueva posición
        self.update_topology_for_new_position()
        
        # Marcar que hay una actualización pendiente
        self.topology_update_pending = True
        
        self.get_logger().info(f" Topología actualizada. Líneas faltantes: {self.missing_lines}")
    
    def clean_memories_for_lane_change(self):
        """Limpia las memorías al cambiar de carril para evitar fantasmas geométricos"""
        retention_factor = self.get_parameter('memory_retention_factor').value
        confidence_reduction = self.get_parameter('confidence_reduction_factor').value
        
        for name, model in self.road_model.items():
            # Retener solo una fracción de los puntos más recientes
            if retention_factor < 1.0:
                n_to_keep = int(len(model.points) * retention_factor)
                if n_to_keep > 0:
                    model.points = deque(list(model.points)[-n_to_keep:], maxlen=model.points.maxlen)
                else:
                    model.points = deque(maxlen=model.points.maxlen)
            
            # Reducir confianza
            model.confidence *= confidence_reduction
            
            # Resetear contador de frames fuente
            model.source_frames = 1
            
            self.get_logger().debug(f" Memoria {name} limpiada: {len(model.points)} puntos, confianza: {model.confidence:.2f}")
    
    def update_topology_for_new_position(self):
        """Actualiza la topología basada en la nueva posición del robot"""
        if not self.road_model:
            return
        
        # Ordenar memorias por posición Y persistente
        sorted_memories = sorted(
            self.road_model.items(), 
            key=lambda item: item[1].persistent_y, 
            reverse=True
        )
        
        # Actualizar etiquetas basadas en la nueva posición del robot
        if self.robot_lane_position == 'INTERMEDIO':
            if len(sorted_memories) >= 4:
                labels = ['left_border', 'lane_dividing', 'right_lane', 'right_border']
                for i, (name, model) in enumerate(sorted_memories[:4]):
                    if i < len(labels):
                        new_label = labels[i]
                        if new_label != name:
                            self.road_model[new_label] = self.road_model.pop(name)
                            if name in self.missing_lines:
                                self.missing_lines.remove(name)
                                self.missing_lines.append(new_label)
            elif len(sorted_memories) >= 3:
                # Asignar basado en conteo de líneas
                left_count = sum(1 for name, model in sorted_memories[:3] if model.persistent_y > 0.2)
                right_count = sum(1 for name, model in sorted_memories[:3] if model.persistent_y < -0.2)
                
                if left_count >= 2 and right_count <= 1:
                    labels = ['left_border', 'lane_dividing', 'right_lane']
                else:
                    labels = ['lane_dividing', 'right_lane', 'right_border']
                
                for i, (name, model) in enumerate(sorted_memories[:3]):
                    if i < len(labels):
                        new_label = labels[i]
                        if new_label != name:
                            self.road_model[new_label] = self.road_model.pop(name)
                            if name in self.missing_lines:
                                self.missing_lines.remove(name)
                                self.missing_lines.append(new_label)
        
        elif self.robot_lane_position == 'DERECHO':
            if len(sorted_memories) >= 3:
                labels = ['left_border', 'lane_dividing', 'right_lane']
                for i, (name, model) in enumerate(sorted_memories[:3]):
                    if i < len(labels):
                        new_label = labels[i]
                        if new_label != name:
                            self.road_model[new_label] = self.road_model.pop(name)
                            if name in self.missing_lines:
                                self.missing_lines.remove(name)
                                self.missing_lines.append(new_label)
            elif len(sorted_memories) >= 2:
                labels = ['lane_dividing', 'right_lane']
                for i, (name, model) in enumerate(sorted_memories[:2]):
                    if i < len(labels):
                        new_label = labels[i]
                        if new_label != name:
                            self.road_model[new_label] = self.road_model.pop(name)
                            if name in self.missing_lines:
                                self.missing_lines.remove(name)
                                self.missing_lines.append(new_label)
        
        elif self.robot_lane_position == 'IZQUIERDO':
            if len(sorted_memories) >= 3:
                labels = ['lane_dividing', 'right_lane', 'right_border']
                for i, (name, model) in enumerate(sorted_memories[:3]):
                    if i < len(labels):
                        new_label = labels[i]
                        if new_label != name:
                            self.road_model[new_label] = self.road_model.pop(name)
                            if name in self.missing_lines:
                                self.missing_lines.remove(name)
                                self.missing_lines.append(new_label)
            elif len(sorted_memories) >= 2:
                labels = ['right_lane', 'right_border']
                for i, (name, model) in enumerate(sorted_memories[:2]):
                    if i < len(labels):
                        new_label = labels[i]
                        if new_label != name:
                            self.road_model[new_label] = self.road_model.pop(name)
                            if name in self.missing_lines:
                                self.missing_lines.remove(name)
                                self.missing_lines.append(new_label)
        
        # Actualizar líneas faltantes
        if self.robot_lane_position in self.expected_lines:
            expected = self.expected_lines[self.robot_lane_position]
            current = list(self.road_model.keys())
            self.missing_lines = [line for line in expected if line not in current]
    
    # =================== TRACKING Y ASIGNACIÓN ===================
    def track_and_assign_with_missing_detection(self, lines_odom):
        if not lines_odom:
            return
        
        current_time = time.time()
        assigned_lines = []
        
        # Calcular distancias del robot a cada memoria
        for name, model in self.road_model.items():
            if name in self.missing_lines:
                continue
            model.min_xy_distance = self.min_distance_robot_to_line(list(model.points))
        
        # Determinar parámetros según estado actual
        if self.system_state == SystemState.LANE_CHANGE_TRANSITION:
            # Usar parámetros más permisivos durante transición
            max_y_dist = self.get_parameter('transition_max_y_distance').value
            xy_weight = self.get_parameter('transition_xy_weight').value
            y_weight = self.get_parameter('transition_y_weight').value
        else:
            # Usar parámetros normales
            max_y_dist = self.get_parameter('max_y_distance').value
            xy_weight = self.get_parameter('xy_distance_weight').value
            y_weight = self.get_parameter('y_distance_weight').value
        
        # Asignar líneas detectadas a modelos existentes
        for line in lines_odom:
            best_match = None
            min_combined_distance = float('inf')
            
            # Calcular distancia del robot a esta línea nueva
            line_xy_distance = self.min_distance_robot_to_line(line['points'])
            
            for name, model in self.road_model.items():
                if name in self.missing_lines:
                    continue
                
                # Distancia basada en posición Y
                y_distance = abs(line['mean_rel_y'] - model.persistent_y)
                
                if y_distance > max_y_dist:
                    continue
                
                # Distancia en X,Y entre el robot y la línea almacenada
                memory_xy_distance = model.min_xy_distance
                
                # Combinar distancias con pesos
                normalized_xy = min(memory_xy_distance / self.get_parameter('max_xy_distance').value, 1.0)
                normalized_y = min(y_distance / max_y_dist, 1.0)
                
                combined_distance = xy_weight * normalized_xy + y_weight * normalized_y
                
                if self.is_in_curve:
                    angle_diff = abs(line['angle'] - model.angle)
                    angle_diff = min(angle_diff, 2*math.pi - angle_diff)
                    combined_distance += angle_diff * 0.1
                
                if combined_distance < min_combined_distance:
                    min_combined_distance = combined_distance
                    best_match = name
            
            if best_match:
                model = self.road_model[best_match]
                model.points.extend(line['points'])
                model.last_update = current_time
                
                # SUAVIZADO DE PERSISTENT_Y - CORRECCIÓN PARA PROBLEMA 1
                smoothing_factor = self.get_parameter('persistent_y_smoothing_factor').value
                model.persistent_y = (1 - smoothing_factor) * model.persistent_y + smoothing_factor * line['mean_rel_y']
                
                model.confidence = min(1.0, model.confidence * 0.95 + line['quality'] * 0.05)
                model.source_frames += 1
                assigned_lines.append(line)
                
                if self.get_parameter('debug').value:
                    self.get_logger().debug(
                        f"Línea asignada a {best_match}: "
                        f"Distancia XY={line_xy_distance:.2f}m, "
                        f"Distancia Y={abs(line['mean_rel_y'] - model.persistent_y):.2f}m, "
                        f"Persistent Y actualizado: {model.persistent_y:.3f}"
                    )
        
        # Buscar líneas faltantes solo si no estamos en transición
        if self.system_state == SystemState.TRACKING and self.missing_lines:
            unassigned_lines = [line for line in lines_odom if line not in assigned_lines]
            
            for missing_line in self.missing_lines:
                expected_y = self.predict_missing_line_position(missing_line)
                
                if expected_y is None:
                    continue
                
                for line in unassigned_lines:
                    if any(
                        abs(line['mean_rel_y'] - self.road_model[l].persistent_y) < 0.4
                        for l in self.road_model
                        if l not in self.missing_lines
                    ):
                        continue
                    
                    distance = abs(line['mean_rel_y'] - expected_y)
                    
                    if distance < self.get_parameter('missing_line_search_tolerance').value:
                        if self.validate_missing_line_candidate(missing_line, line):
                            self.add_missing_line(missing_line, line, current_time)
                            if missing_line in self.missing_lines:
                                self.missing_lines.remove(missing_line)
                            break
    
    def validate_missing_line_candidate(self, label, candidate):
        if label not in self.missing_line_candidates_history:
            self.missing_line_candidates_history[label] = deque(maxlen=5)
        
        self.missing_line_candidates_history[label].append({
            'y': candidate['mean_rel_y'],
            'quality': candidate['quality'],
            'angle': candidate['angle'],
            'time': time.time()
        })
        
        history = self.missing_line_candidates_history[label]
        if len(history) >= 3:
            y_values = [c['y'] for c in history]
            y_std = np.std(y_values)
            avg_quality = np.mean([c['quality'] for c in history])
            
            if self.road_model:
                ref_line = None
                if 'lane_dividing' in self.road_model:
                    ref_line = self.road_model['lane_dividing']
                elif len(self.road_model) > 0:
                    ref_line = list(self.road_model.values())[0]
                
                if ref_line:
                    angle_diffs = [abs(c['angle'] - ref_line.angle) for c in history]
                    avg_angle_diff = np.mean(angle_diffs)
                    max_angle_diff = max(angle_diffs)
                else:
                    avg_angle_diff = 0
                    max_angle_diff = 0
            else:
                avg_angle_diff = 0
                max_angle_diff = 0
            
            is_consistent = (y_std < 0.3 and
                            avg_quality > 0.4 and
                            max_angle_diff < math.radians(20))
            
            return is_consistent
        
        return False
    
    def add_missing_line(self, label, line_data, current_time):
        line_model = LineModel(
            points=deque(line_data['points'], maxlen=self.get_parameter('memory_size').value),
            persistent_y=line_data['mean_rel_y'],
            confidence=line_data['quality'],
            last_update=current_time,
            angle=line_data['angle'],
            length=line_data['length'],
            source_frames=1,
            min_xy_distance=self.min_distance_robot_to_line(line_data['points']),
            relative_position=self.determine_relative_position(line_data['mean_rel_y'])
        )
        
        self.road_model[label] = line_model
        
        if label not in self.main_publishers and label in ['left_border', 'lane_dividing', 'right_lane', 'right_border']:
            self.main_publishers[label] = self.create_publisher(
                PointCloud2, f'/memory_local/{label}', 10
            )
        
        if label in self.missing_line_candidates_history:
            del self.missing_line_candidates_history[label]
        
        self.get_logger().info(f" Línea {label} agregada al modelo")
    
    def predict_missing_line_position(self, missing_line):
        if not self.road_model:
            return None
        
        existing_positions = {}
        for name, model in self.road_model.items():
            existing_positions[name] = model.persistent_y
        
        lane_width = self.get_parameter('lane_width_estimate').value
        
        if missing_line == 'right_border':
            if 'right_lane' in existing_positions:
                return existing_positions['right_lane'] - lane_width
            elif 'lane_dividing' in existing_positions:
                return existing_positions['lane_dividing'] - lane_width * 1.5
        
        elif missing_line == 'left_border':
            if 'lane_dividing' in existing_positions:
                return existing_positions['lane_dividing'] + lane_width
        
        elif missing_line == 'right_lane':
            if 'lane_dividing' in existing_positions:
                return existing_positions['lane_dividing'] - lane_width / 2
        
        elif missing_line == 'lane_dividing':
            if 'left_border' in existing_positions and 'right_lane' in existing_positions:
                return (existing_positions['left_border'] + existing_positions['right_lane']) / 2
        
        return None
    
    # =================== PUBLICACIÓN ===================
    def publish_memories(self):
        if not self.odom_ready or not self.road_model:
            return
        
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'odom'
        
        for name, model in self.road_model.items():
            if not model.points:
                continue
            
            cloud = point_cloud2.create_cloud_xyz32(h, list(model.points))
            
            if name in self.main_publishers:
                self.main_publishers[name].publish(cloud)
            else:
                if name not in self.pubs:
                    self.pubs[name] = self.create_publisher(
                        PointCloud2, f'/memory/{name}', 10
                    )
                self.pubs[name].publish(cloud)
    
    # =================== UTILIDADES ===================
    @staticmethod
    def q2yaw(q):
        return math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y**2 + q.z**2)
        )
    
    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = TopologyAssigner()
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