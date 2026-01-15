#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
import os
import xml.etree.ElementTree as ET
from collections import deque
import networkx as nx
from shapely.geometry import LineString, Point

from custom_interfaces.msg import NearbyOSMElements, OSMElement
from std_msgs.msg import Bool

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from std_msgs.msg import Float32MultiArray, Header, ColorRGBA
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import Buffer, TransformListener, TransformException
from pyproj import Transformer
from visualization_msgs.msg import Marker, MarkerArray
import warnings
warnings.filterwarnings('ignore')

# ---------------- PARAMETERS ----------------
LANE_WIDTH = 3.5
MAX_CANDIDATES = 50
LOOKAHEAD_DISTANCE = 100.0
SIGMA_POSITION = 2.0
SIGMA_HEADING = math.radians(45.0)
SIGMA_CAMERA = 0.5
SIGMA_OSM = 1.0
HISTORY_LENGTH = 3
CANDIDATE_RADIUS = 200.0
OSM_ASSOCIATION_THRESHOLD = 5.0
MAX_CURVATURE_ANGLE = math.radians(45.0)

transformer_ll_to_xy = Transformer.from_crs("EPSG:4326", "EPSG:32717", always_xy=True)


# ---------------- Helper functions ----------------
def latlon_to_xy(lat, lon):
    x, y = transformer_ll_to_xy.transform(lon, lat)
    return (x, y)

def gaussian_pdf(x, sigma):
    """Gaussian probability density function"""
    if sigma <= 0:
        return 1.0
    return (1.0 / (sigma * math.sqrt(2 * math.pi))) * math.exp(-0.5 * (x**2) / (sigma**2))

def point_to_line_distance(px, py, x1, y1, x2, y2):
    """Distance from point to line segment"""
    A = px - x1
    B = py - y1
    C = x2 - x1
    D = y2 - y1

    dot = A * C + B * D
    len_sq = C * C + D * D
    
    if len_sq == 0:
        return math.hypot(px - x1, py - y1), (x1, y1), 0.0
        
    param = dot / len_sq
    
    if param < 0:
        xx, yy = x1, y1
    elif param > 1:
        xx, yy = x2, y2
    else:
        xx = x1 + param * C
        yy = y1 + param * D
        
    return math.hypot(px - xx, py - yy), (xx, yy), param

def interpolate_path(points, max_distance=1.0):
    """Interpolate path to have points at most max_distance apart"""
    if len(points) < 2:
        return points
        
    result = [points[0]]
    
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist > max_distance:
            n_segments = int(math.ceil(dist / max_distance))
            for j in range(1, n_segments):
                t = j / n_segments
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                result.append((x, y))
        else:
            result.append(points[i + 1])
            
    return result

# ---------------- Main Node ----------------
class LaneEgoGenerator(Node):
    def __init__(self):
        super().__init__('lane_ego_generator')
        
        # State variables
        self.global_path_points = []
        self.raw_global_path_points = []
        self.right_lane_points = []
        self.robot_pose = None
        self.robot_covariance = {
            'x': SIGMA_POSITION**2,
            'y': SIGMA_POSITION**2,
            'yaw': SIGMA_HEADING**2
        }
        self.robot_speed = 0.0
        self.robot_ekf_yaw = 0.0
        self.utm_origin = None
        self.osm_lanes = []
        self.osm_lane_dict = {}
        
        # HMM State
        self.candidates = []
        self.beliefs = None
        self.last_best_offset = 0.0
        self.last_best_lane_id = None
        self.offset_history = deque(maxlen=10)
        self.last_update_time = None
        self.lane_change_counter = 0
        self.lane_change_cooldown = 0
        self.current_osm_lane = None  # Nuevo: carril OSM actual
        self.current_osm_lane_points = []  # Nuevo: puntos del carril actual
        self.osm_features = []
        
        # Variable para controlar el estado
        self.goal_reached = False
        self.goal_threshold = 3.0  # metros de distancia para considerar que se llegó a la meta

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Parameters - MEJORADOS
        self.declare_parameter('lane_width', LANE_WIDTH)
        self.declare_parameter('max_candidates', MAX_CANDIDATES)
        self.declare_parameter('lookahead_distance', LOOKAHEAD_DISTANCE)
        self.declare_parameter('sigma_position', SIGMA_POSITION)
        self.declare_parameter('sigma_heading', SIGMA_HEADING)
        self.declare_parameter('sigma_camera', 0.03)
        self.declare_parameter('sigma_osm', SIGMA_OSM)
        self.declare_parameter('candidate_radius', CANDIDATE_RADIUS)
        self.declare_parameter('camera_weight', 50.0)
        self.declare_parameter('osm_weight', 2.0)
        self.declare_parameter('use_lane_level', True)
        self.declare_parameter('yaw_fusion_alpha', 0.7)
        self.declare_parameter('path_smoothing_alpha', 0.25)
        self.declare_parameter('path_smoothing_iterations', 3)
        self.declare_parameter('max_curvature_angle', math.degrees(MAX_CURVATURE_ANGLE))
        self.declare_parameter('build_from_osm', True)  # NUEVO: construir desde OSM
        self.declare_parameter('hybrid_mode', True)  # NUEVO: modo híbrido
        self.declare_parameter('osm_quality_threshold', 0.8)  # NUEVO: calidad mínima para usar OSM
        self.declare_parameter('max_osm_gap', 5.0)  # NUEVO: gap máximo en OSM
        
        # Subscribers
        self.create_subscription(Path, '/global_path', self.global_path_callback, 10)
        self.create_subscription(Path, '/lane/right_tracked', self.right_lane_callback, 10)
        self.create_subscription(Odometry, '/odometry/global', self.odometry_callback, 10)
        self.create_subscription(PointStamped, '/utm_map_origin', self.origin_callback, 1)
        
        # Publishers
        self.lane_ego_pub = self.create_publisher(Path, '/lane_ego', 10)
        self.belief_pub = self.create_publisher(Float32MultiArray, '/lane_ego/beliefs', 10)
        self.lane_center_pub = self.create_publisher(Path, '/map_matching/lane_center', 10)
        self.lane_left_pub = self.create_publisher(Path, '/map_matching/lane_left', 10)
        self.lane_right_pub = self.create_publisher(Path, '/map_matching/lane_right', 10)
        self.candidates_pub = self.create_publisher(MarkerArray, '/lane_ego/candidates', 10)
        self.lane_info_pub = self.create_publisher(MarkerArray, '/lane_ego/lane_info', 10)
        self.smooth_path_pub = self.create_publisher(Path, '/lane_ego/smooth_path', 10)
        self.osm_lane_debug_pub = self.create_publisher(Path, '/lane_ego/osm_lane_debug', 10)  # NUEVO
        self.pub_osm_context = self.create_publisher(NearbyOSMElements,"/map_matching/nearby_osm_elements",5)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)

        # Timer
        self.create_timer(0.1, self.update)
        
        #self.get_logger().info('Lane EGO Generator node initialized (Full Lane-Level)')
        
    def load_osm_data(self):
        """Load OSM data - MEJORADO con más información de geometría"""
        osm_file = self.find_osm_file()
        if not osm_file:
            #self.get_logger().warn('No OSM file found')
            return
            
        try:
            tree = ET.parse(osm_file)
            root = tree.getroot()
            
            nodes_dict = {}
            for node in root.findall('node'):
                node_id = node.attrib['id']
                try:
                    lat = float(node.attrib['lat'])
                    lon = float(node.attrib['lon'])
                    xy = latlon_to_xy(lat, lon)
                    if self.utm_origin:
                        x = xy[0] - self.utm_origin[0]
                        y = xy[1] - self.utm_origin[1]
                    else:
                        x, y = xy
                    nodes_dict[node_id] = (x, y)

                    # EXTRAER ELEMENTOS OSM 
                    # Procesar nodos que tienen features especiales
                    tags = {t.attrib["k"]: t.attrib["v"] for t in node.findall("tag")}
                    
                    feature_type = None
                    sub_type = ""
                    
                    # Identificar tipos de features
                    if tags.get("traffic_calming"):
                        feature_type = OSMElement.FEATURE_TRAFFIC_CALMING
                        sub_type = tags.get("traffic_calming", "")
                        
                    elif tags.get("highway") == "crossing":
                        feature_type = OSMElement.FEATURE_CROSSING
                        sub_type = tags.get("crossing", "unknown")
                        
                    elif tags.get("highway") == "stop":
                        feature_type = OSMElement.FEATURE_STOP
                        sub_type = "stop"
                        
                    elif tags.get("highway") == "give_way":
                        feature_type = OSMElement.FEATURE_GIVE_WAY
                        sub_type = "give_way"
                    
                    # Si encontramos un feature, guardarlo
                    if feature_type is not None:
                        self.osm_features.append({
                            "id": node_id,
                            "type": feature_type,
                            "sub_type": sub_type,
                            "pos": (x, y),
                            "tags": tags,
                            "lat": lat,
                            "lon": lon
                        })
                except:
                    continue
            
            self.osm_lanes = []
            self.osm_lane_dict = {}
            
            for way in root.findall('way'):
                tags = {t.attrib['k']: t.attrib['v'] for t in way.findall('tag')}
                
                if 'highway' not in tags:
                    continue
                
                way_id = way.attrib['id']
                oneway = tags.get('oneway', 'no') == 'yes'
                lanes = int(tags.get('lanes', 1))
                lanes_forward = int(tags.get('lanes:forward', lanes if oneway else lanes//2))
                lanes_backward = int(tags.get('lanes:backward', 0 if oneway else lanes//2))
                width = float(tags.get('width', lanes * LANE_WIDTH))
                
                if width < 6.5:
                    lanes_forward = 1
                    lanes_backward = 0
                
                way_nodes = []
                for nd in way.findall('nd'):
                    node_id = nd.attrib['ref']
                    if node_id in nodes_dict:
                        way_nodes.append(nodes_dict[node_id])
                
                if len(way_nodes) < 2:
                    continue
                
                line = LineString(way_nodes)
                interp_distance = 2.0  # Más denso para mejor calidad
                interpolated_points = []
                
                distance = 0.0
                while distance <= line.length:
                    point = line.interpolate(distance)
                    interpolated_points.append((point.x, point.y))
                    distance += interp_distance
                
                if line.length > 0:
                    end_point = line.interpolate(line.length)
                    if (end_point.x, end_point.y) != interpolated_points[-1]:
                        interpolated_points.append((end_point.x, end_point.y))
                
                num_lanes = lanes_forward
                lane_width = width / max(1, lanes)
                
                for lane_idx in range(num_lanes):
                    lane_points = []
                    lane_yaws = []
                    
                    for i in range(len(interpolated_points)):
                        point = interpolated_points[i]
                        
                        if i == 0:
                            p1, p2 = interpolated_points[0], interpolated_points[1]
                        elif i == len(interpolated_points) - 1:
                            p1, p2 = interpolated_points[-2], interpolated_points[-1]
                        else:
                            p1, p2 = interpolated_points[i-1], interpolated_points[i+1]
                        
                        dx = p2[0] - p1[0]
                        dy = p2[1] - p1[1]
                        length = math.hypot(dx, dy)
                        
                        if length > 0:
                            nx = -dy / length
                            ny = dx / length
                            yaw = math.atan2(dy, dx)
                        else:
                            nx, ny = 0, 0
                            yaw = 0.0
                        
                        offset_from_center = (lane_idx - (num_lanes - 1) / 2.0) * lane_width
                        
                        lane_point = (
                            point[0] + nx * offset_from_center,
                            point[1] + ny * offset_from_center
                        )
                        lane_points.append(lane_point)
                        lane_yaws.append(yaw)
                    
                    lane_id = f"{way_id}_{lane_idx}"
                    
                    # Calcular calidad de la geometría
                    lane_length = self.calculate_lane_length(lane_points)
                    curvature = self.calculate_lane_curvature(lane_points)
                    
                    lane_data = {
                        'id': lane_id,
                        'points': lane_points,
                        'yaws': lane_yaws,
                        'direction': 'forward',
                        'lane_idx': lane_idx,
                        'way_id': way_id,
                        'lane_width': lane_width,
                        'num_lanes': num_lanes,
                        'is_forward': True,
                        'is_oneway': oneway,
                        'length': lane_length,
                        'curvature': curvature,
                        'quality_score': self.calculate_lane_quality(lane_points)  # NUEVO
                    }
                    
                    self.osm_lanes.append(lane_data)
                    self.osm_lane_dict[lane_id] = lane_data
            
            #self.get_logger().info(f'Loaded {len(self.osm_lanes)} lanes from OSM with quality scores')
            
        except Exception as e:
            #self.get_logger().error(f'Error loading OSM: {str(e)}')
            pass
    
    def calculate_lane_length(self, points):
        """Calcular longitud de un carril"""
        if len(points) < 2:
            return 0.0
        length = 0.0
        for i in range(1, len(points)):
            x1, y1 = points[i-1]
            x2, y2 = points[i]
            length += math.hypot(x2 - x1, y2 - y1)
        return length
    
    def calculate_lane_curvature(self, points):
        """Calcular curvatura promedio de un carril"""
        if len(points) < 3:
            return 0.0
        
        total_curvature = 0.0
        count = 0
        
        for i in range(1, len(points) - 1):
            x0, y0 = points[i-1]
            x1, y1 = points[i]
            x2, y2 = points[i+1]
            
            v1 = (x1 - x0, y1 - y0)
            v2 = (x2 - x1, y2 - y1)
            
            dot = v1[0]*v2[0] + v1[1]*v2[1]
            norm1 = math.hypot(v1[0], v1[1])
            norm2 = math.hypot(v2[0], v2[1])
            
            if norm1 > 0 and norm2 > 0:
                cos_angle = dot / (norm1 * norm2)
                cos_angle = max(-1.0, min(1.0, cos_angle))
                angle = math.acos(cos_angle)
                total_curvature += angle
                count += 1
        
        return total_curvature / max(1, count)
    
    def calculate_lane_quality(self, points):
        """Calcular calidad de geometría del carril (0.0-1.0)"""
        if len(points) < 3:
            return 0.5
        
        # 1. Densidad de puntos
        length = self.calculate_lane_length(points)
        point_density = len(points) / max(1.0, length)
        density_score = min(1.0, point_density / 2.0)  # 2 puntos por metro es bueno
        
        # 2. Suavidad (cambios angulares pequeños)
        curvature = self.calculate_lane_curvature(points)
        smoothness_score = 1.0 - min(1.0, curvature / math.radians(30.0))
        
        # 3. Consistencia de espaciado
        spacings = []
        for i in range(1, len(points)):
            x1, y1 = points[i-1]
            x2, y2 = points[i]
            spacings.append(math.hypot(x2 - x1, y2 - y1))
        
        if spacings:
            avg_spacing = np.mean(spacings)
            std_spacing = np.std(spacings)
            consistency_score = 1.0 - min(1.0, std_spacing / max(0.1, avg_spacing))
        else:
            consistency_score = 0.5
        
        # Ponderar scores
        quality = 0.4 * density_score + 0.3 * smoothness_score + 0.3 * consistency_score
        return quality
            
    def find_osm_file(self):
        possible_paths = [
            os.path.join(os.path.dirname(__file__), "maps", "casa.osm"),
            os.path.join(os.path.dirname(__file__), "casa.osm"),
            "/home/raynel/autonomous_navigation/src/navigation_system/maps/casa.osm",
            "casa.osm",
        ]
        for p in possible_paths:
            if os.path.exists(p):
                #self.get_logger().info(f'Found OSM file: {p}')
                return p
        return None
    
    def smooth_polyline(self, points, alpha=0.25, iterations=3):
        """Moving average smoothing preserving endpoints"""
        if len(points) < 3:
            return points

        pts = points[:]
        for _ in range(iterations):
            new_pts = [pts[0]]
            
            for i in range(1, len(pts)-1):
                x = (1-alpha)*pts[i][0] + alpha*(pts[i-1][0] + pts[i+1][0])/2
                y = (1-alpha)*pts[i][1] + alpha*(pts[i-1][1] + pts[i+1][1])/2
                new_pts.append((x, y))
            
            new_pts.append(pts[-1])
            pts = new_pts
        
        return pts
        
    def global_path_callback(self, msg):
        """Process global path"""
        # Resetear estado de goal cuando llega un nuevo path
        self.goal_reached = False
        self.publish_goal_status(False)

        self.raw_global_path_points = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.raw_global_path_points.append((x, y))
        
        interp_points = interpolate_path(self.raw_global_path_points, max_distance=1.0)
        
        alpha = float(self.get_parameter('path_smoothing_alpha').value)
        iterations = int(self.get_parameter('path_smoothing_iterations').value)
        smooth_points = self.smooth_polyline(interp_points, alpha=alpha, iterations=iterations)
        
        self.global_path_points = smooth_points
        self.publish_smooth_path(smooth_points)
        
        #self.get_logger().info(f'Global path received: {len(self.raw_global_path_points)} raw -> {len(smooth_points)} smooth points')
        
    def publish_smooth_path(self, points):
        """Publicar path suavizado para debug"""
        msg = Path()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        
        self.smooth_path_pub.publish(msg)
        
    def right_lane_callback(self, msg):
        """Process right lane detection from camera"""
        if len(msg.poses) == 0:
            self.right_lane_points = []
            return

        if not msg.header.frame_id:
            #self.get_logger().warn("right_lane message has empty frame_id")
            self.right_lane_points = []
            return

        try:
            target = 'map'
            source = msg.header.frame_id
            timeout = rclpy.duration.Duration(seconds=0.2)
            if not self.tf_buffer.can_transform(target, source, rclpy.time.Time(), timeout):
                #self.get_logger().warn(f"Cannot transform from {source} to {target} yet")
                self.right_lane_points = []
                return

            transform = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time(), timeout)
            self.right_lane_points = []
            
            q = transform.transform.rotation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y

            for pose in msg.poses:
                x_bf = pose.pose.position.x
                y_bf = pose.pose.position.y
                x_map = tx + x_bf * math.cos(yaw) - y_bf * math.sin(yaw)
                y_map = ty + x_bf * math.sin(yaw) + y_bf * math.cos(yaw)
                self.right_lane_points.append((x_map, y_map))

        except Exception as e:
            #self.get_logger().warn(f'TF error for camera points: {e}')
            self.right_lane_points = []
            
    def odometry_callback(self, msg):
        """Process robot pose from EKF con fusión mejorada"""
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            q = msg.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            self.robot_ekf_yaw = yaw
            
            alpha = float(self.get_parameter('yaw_fusion_alpha').value)
            
            if self.last_best_lane_id and self.last_best_lane_id in self.osm_lane_dict:
                lane = self.osm_lane_dict[self.last_best_lane_id]
                
                min_dist = float('inf')
                lane_yaw = yaw
                closest_point = None
                
                for i, (lx, ly) in enumerate(lane['points']):
                    dist = math.hypot(lx - x, ly - y)
                    if dist < min_dist:
                        min_dist = dist
                        closest_point = (lx, ly)
                        if i < len(lane['yaws']):
                            lane_yaw = lane['yaws'][i]
                
                # MEJORA: Solo fusionar si estamos cerca del carril
                if min_dist < 3.0:  # 3 metros máximo
                    fused_yaw = lane_yaw * alpha + yaw * (1 - alpha)
                    fused_yaw = math.atan2(math.sin(fused_yaw), math.cos(fused_yaw))
                    self.robot_pose = (x, y, fused_yaw)
                else:
                    self.robot_pose = (x, y, yaw)
            else:
                self.robot_pose = (x, y, yaw)
            
            self.robot_speed = msg.twist.twist.linear.x
            
            cov_list = msg.pose.covariance
            
            if cov_list is not None and len(cov_list) >= 36:
                cov_x = float(cov_list[0])
                cov_y = float(cov_list[7])
                cov_yaw = float(cov_list[35])
                
                if cov_x <= 0:
                    cov_x = SIGMA_POSITION**2
                if cov_y <= 0:
                    cov_y = SIGMA_POSITION**2
                if cov_yaw <= 0:
                    cov_yaw = SIGMA_HEADING**2
                    
                self.robot_covariance = {
                    'x': cov_x,
                    'y': cov_y,
                    'yaw': cov_yaw
                }
                
        except Exception as e:
            #self.get_logger().warn(f'Error processing odometry: {str(e)}')
            pass
            
    def origin_callback(self, msg):
        if self.utm_origin is None:
            self.utm_origin = (msg.point.x, msg.point.y)
            #self.get_logger().info(f'UTM origin set: {self.utm_origin}')
            self.load_osm_data()
            
    def find_closest_osm_lane(self, x, y, max_distance=OSM_ASSOCIATION_THRESHOLD):
        """Encuentra el carril OSM más cercano a un punto"""
        closest_lane = None
        min_distance = float('inf')
        closest_point_idx = 0
        closest_param = 0.0
        
        for lane in self.osm_lanes:
            points = lane['points']
            
            for i in range(len(points) - 1):
                x1, y1 = points[i]
                x2, y2 = points[i + 1]
                
                dist, proj_point, param = point_to_line_distance(x, y, x1, y1, x2, y2)
                
                if dist < min_distance and dist <= max_distance:
                    min_distance = dist
                    closest_lane = lane
                    closest_point_idx = i
                    closest_param = param
        
        return closest_lane, min_distance, (closest_point_idx, closest_param)
    
    def build_lane_from_osm_lane(self, lane_data, start_point, lookahead):
        """CONSTRUIR DESDE OSM - NUEVA FUNCIÓN"""
        if not lane_data or 'points' not in lane_data:
            return None
        
        lane_points = lane_data['points']
        lane_yaws = lane_data['yaws']
        
        if len(lane_points) < 2:
            return None
        
        # Encontrar punto de inicio más cercano
        start_idx = 0
        min_start_dist = float('inf')
        
        for i, (lx, ly) in enumerate(lane_points):
            dist = math.hypot(lx - start_point[0], ly - start_point[1])
            if dist < min_start_dist:
                min_start_dist = dist
                start_idx = i
        
        # Construir camino con lookahead
        result_points = []
        accumulated_dist = 0.0
        
        for i in range(start_idx, len(lane_points)):
            if i >= len(lane_points):
                break
                
            x, y = lane_points[i]
            
            # Obtener yaw
            if i < len(lane_yaws):
                yaw = lane_yaws[i]
            else:
                # Calcular yaw basado en puntos adyacentes
                if i > 0 and i < len(lane_points) - 1:
                    x_prev, y_prev = lane_points[i-1]
                    x_next, y_next = lane_points[i+1]
                    yaw = math.atan2(y_next - y_prev, x_next - x_prev)
                elif i > 0:
                    x_prev, y_prev = lane_points[i-1]
                    yaw = math.atan2(y - y_prev, x - x_prev)
                else:
                    yaw = 0.0
            
            result_points.append((x, y, yaw))
            
            # Calcular distancia acumulada
            if len(result_points) > 1:
                x1, y1, _ = result_points[-2]
                x2, y2, _ = result_points[-1]
                accumulated_dist += math.hypot(x2 - x1, y2 - y1)
            
            if accumulated_dist >= lookahead:
                break
        
        # Si no alcanzamos el lookahead, intentar seguir en el siguiente way OSM conectado
        if accumulated_dist < lookahead * 0.8:  # Solo si falta mucho
            # Aquí podrías implementar lógica para seguir en el siguiente way conectado
            pass
        
        return result_points
    
    def build_lane_from_candidate(self, candidate, lookahead):
        """Construir el lane desde el candidato ganador (backup)"""
        lane_points = []
        
        seg_idx = candidate['segment_idx']
        offset = candidate['offset']
        max_curvature_angle = float(self.get_parameter('max_curvature_angle').value)
        max_curvature_rad = math.radians(max_curvature_angle)
        
        for i in range(seg_idx, len(self.global_path_points) - 1):
            if i > seg_idx:
                if i >= 2 and i < len(self.global_path_points) - 1:
                    x0, y0 = self.global_path_points[i-2]
                    x1, y1 = self.global_path_points[i-1]
                    x2, y2 = self.global_path_points[i]
                    x3, y3 = self.global_path_points[i+1]
                    
                    yaw1 = math.atan2(y1 - y0, x1 - x0)
                    yaw2 = math.atan2(y2 - y1, x2 - x1)
                    yaw3 = math.atan2(y3 - y2, x3 - x2)
                    
                    dyaw1 = abs(math.atan2(math.sin(yaw2 - yaw1), math.cos(yaw2 - yaw1)))
                    dyaw2 = abs(math.atan2(math.sin(yaw3 - yaw2), math.cos(yaw3 - yaw2)))
                    
                    if dyaw1 > max_curvature_rad or dyaw2 > max_curvature_rad:
                        #self.get_logger().debug(f'Curvature break detected at segment {i}')
                        break
            
            if len(lane_points) > 0 and self.calculate_path_length(lane_points) >= lookahead:
                break
                
            x1, y1 = self.global_path_points[i]
            x2, y2 = self.global_path_points[i+1]
            
            dx = x2 - x1
            dy = y2 - y1
            seg_len = math.hypot(dx, dy)
            if seg_len == 0:
                continue
            
            if i > 0 and i < len(self.global_path_points) - 2:
                px0, py0 = self.global_path_points[i-1]
                px2, py2 = self.global_path_points[i+2]
                dx_smooth = (px2 - px0) / 2.0
                dy_smooth = (py2 - py0) / 2.0
                norm_smooth = math.hypot(dx_smooth, dy_smooth)
                
                if norm_smooth > 0:
                    yaw = math.atan2(dy_smooth, dx_smooth)
                    nx = -dy_smooth / norm_smooth
                    ny = dx_smooth / norm_smooth
                else:
                    yaw = math.atan2(dy, dx)
                    nx = -dy / seg_len
                    ny = dx / seg_len
            else:
                yaw = math.atan2(dy, dx)
                nx = -dy / seg_len
                ny = dx / seg_len
            
            n_pts = max(2, int(seg_len / 0.5))
            for j in range(n_pts):
                t = j / (n_pts - 1) if n_pts > 1 else 0.5
                px = x1 + t * dx + nx * offset
                py = y1 + t * dy + ny * offset
                lane_points.append((px, py, yaw))
                
                if len(lane_points) > 1:
                    d = math.hypot(
                        px - lane_points[-2][0],
                        py - lane_points[-2][1]
                    )
                    if self.calculate_path_length(lane_points) >= lookahead:
                        break
        
        return lane_points
    
    def calculate_path_length(self, points):
        """Calcular longitud total de un path"""
        if len(points) < 2:
            return 0.0
            
        total = 0.0
        for i in range(1, len(points)):
            x1, y1, _ = points[i-1]
            x2, y2, _ = points[i]
            total += math.hypot(x2 - x1, y2 - y1)
        return total
            
    def smooth_path_chaikin(self, points, iterations=2):
        """Suavizar curvas y entradas con algoritmo de Chaikin"""
        if len(points) < 3:
            return points
            
        current_points = points
        
        for _ in range(iterations):
            new_points = []
            
            new_points.append(current_points[0])
            
            for i in range(len(current_points) - 1):
                p0 = current_points[i]
                p1 = current_points[i + 1]
                
                x0, y0, yaw0 = p0
                x1, y1, yaw1 = p1
                
                Qx = 0.75 * x0 + 0.25 * x1
                Qy = 0.75 * y0 + 0.25 * y1
                Qyaw = yaw0 * 0.75 + yaw1 * 0.25
                
                Rx = 0.25 * x0 + 0.75 * x1
                Ry = 0.25 * y0 + 0.75 * y1
                Ryaw = yaw0 * 0.25 + yaw1 * 0.75
                
                new_points.append((Qx, Qy, Qyaw))
                new_points.append((Rx, Ry, Ryaw))
            
            new_points.append(current_points[-1])
            current_points = new_points
        
        return current_points
    
    def generate_candidates(self):
        """Generate candidate lane centers - MEJORADO con calidad OSM"""
        if not self.global_path_points or len(self.global_path_points) < 2:
            return []
            
        candidates = []
        lane_width = float(self.get_parameter('lane_width').value)
        candidate_radius = float(self.get_parameter('candidate_radius').value)
        use_lane_level = bool(self.get_parameter('use_lane_level').value)
        
        if self.robot_pose:
            robot_x, robot_y = self.robot_pose[0], self.robot_pose[1]
        else:
            robot_x, robot_y = 0.0, 0.0
        
        closest_idx = 0
        min_dist = float('inf')
        for i in range(len(self.global_path_points) - 1):
            x1, y1 = self.global_path_points[i]
            x2, y2 = self.global_path_points[i + 1]
            
            dist, _, _ = point_to_line_distance(robot_x, robot_y, x1, y1, x2, y2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        start_idx = max(0, closest_idx - 1)
        end_idx = min(len(self.global_path_points) - 1, closest_idx + 3)
        
        for i in range(start_idx, end_idx):
            if i >= len(self.global_path_points) - 1:
                break
                
            x1, y1 = self.global_path_points[i]
            x2, y2 = self.global_path_points[i + 1]
            
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length == 0:
                continue
                
            if i > 0 and i < len(self.global_path_points) - 2:
                px0, py0 = self.global_path_points[i-1]
                px2, py2 = self.global_path_points[i+2]
                dx_smooth = (px2 - px0) / 2.0
                dy_smooth = (py2 - py0) / 2.0
                norm_smooth = math.hypot(dx_smooth, dy_smooth)
                
                if norm_smooth > 0:
                    nx = -dy_smooth / norm_smooth
                    ny = dx_smooth / norm_smooth
                    segment_yaw = math.atan2(dy_smooth, dx_smooth)
                else:
                    nx = -dy / length
                    ny = dx / length
                    segment_yaw = math.atan2(dy, dx)
            else:
                nx = -dy / length
                ny = dx / length
                segment_yaw = math.atan2(dy, dx)
            
            n_points_along = 3
            for j in range(n_points_along):
                t = j / (n_points_along - 1) if n_points_along > 1 else 0.5
                seg_x = x1 + t * dx
                seg_y = y1 + t * dy
                
                n_offsets = 9
                offsets = np.linspace(-lane_width, lane_width, n_offsets)
                
                for offset in offsets:
                    cand_x = seg_x + nx * offset
                    cand_y = seg_y + ny * offset
                    
                    dist_to_robot = math.hypot(cand_x - robot_x, cand_y - robot_y)
                    if dist_to_robot > candidate_radius:
                        continue
                    
                    # MEJORA: Evaluación de calidad OSM
                    osm_lane = None
                    osm_lane_id = None
                    lane_idx = -1
                    osm_quality = 0.0
                    
                    if use_lane_level and self.osm_lanes:
                        closest_lane, lane_dist, _ = self.find_closest_osm_lane(cand_x, cand_y)
                        if closest_lane and lane_dist < OSM_ASSOCIATION_THRESHOLD:
                            osm_lane = closest_lane
                            osm_lane_id = closest_lane['id']
                            lane_idx = closest_lane['lane_idx']
                            osm_quality = closest_lane.get('quality_score', 0.5)
                    
                    candidates.append({
                        'x': cand_x,
                        'y': cand_y,
                        'yaw': segment_yaw,
                        'offset': offset,
                        'segment_idx': i,
                        't_param': t,
                        'osm_lane': osm_lane,
                        'osm_lane_id': osm_lane_id,
                        'lane_idx': lane_idx,
                        'distance_to_robot': dist_to_robot,
                        'is_lane_level': (osm_lane is not None),
                        'osm_quality': osm_quality  # NUEVO: calidad del carril
                    })
        
        max_candidates = int(self.get_parameter('max_candidates').value)
        if len(candidates) > max_candidates:
            candidates.sort(key=lambda c: c['distance_to_robot'])
            candidates = candidates[:max_candidates]
            
        return candidates
        
    def compute_candidate_likelihood(self, candidate, pred_dx=0.0, pred_dy=0.0):
        """Compute likelihood of a candidate - MEJORADO con calidad OSM"""
        likelihood = 1.0

        if self.robot_pose:
            rx, ry, ryaw = self.robot_pose
            dist = math.hypot(candidate['x'] - rx, candidate['y'] - ry)
            sigma_pos = math.sqrt(max(self.robot_covariance.get('x', SIGMA_POSITION**2), 0.01))
            likelihood *= gaussian_pdf(dist, sigma_pos)

            heading_diff = abs(math.atan2(math.sin(candidate['yaw'] - ryaw), math.cos(candidate['yaw'] - ryaw)))
            sigma_yaw = math.sqrt(max(self.robot_covariance.get('yaw', SIGMA_HEADING**2), 1e-4))
            likelihood *= gaussian_pdf(heading_diff, sigma_yaw)
            
            if pred_dx != 0.0 or pred_dy != 0.0:
                pred_x = rx + pred_dx
                pred_y = ry + pred_dy
                motion_error = math.hypot(candidate['x'] - pred_x, candidate['y'] - pred_y)
                likelihood *= gaussian_pdf(motion_error, 1.5)

        expected_dist = float(self.get_parameter('lane_width').value) / 2.0
        if self.right_lane_points:
            min_dist_to_right = float('inf')
            closest_right_point = None
            for cam_x, cam_y in self.right_lane_points:
                dist = math.hypot(cam_x - candidate['x'], cam_y - candidate['y'])
                if dist < min_dist_to_right:
                    min_dist_to_right = dist
                    closest_right_point = (cam_x, cam_y)

            camera_error = abs(min_dist_to_right - expected_dist)

            if closest_right_point:
                dx = closest_right_point[0] - candidate['x']
                dy = closest_right_point[1] - candidate['y']
                nx = math.sin(candidate['yaw'])
                ny = -math.cos(candidate['yaw'])
                dot = dx * nx + dy * ny
                if dot < 0:
                    camera_error *= 10.0

            sigma_cam = float(self.get_parameter('sigma_camera').value)
            camera_weight = float(self.get_parameter('camera_weight').value)
            camera_likelihood = gaussian_pdf(camera_error, sigma_cam)
            likelihood *= (camera_likelihood ** max(1.0, camera_weight))

        # MEJORA: Observación OSM con calidad
        if candidate.get('osm_lane') is not None:
            osm_weight = float(self.get_parameter('osm_weight').value)
            lane = candidate['osm_lane']
            osm_quality = candidate.get('osm_quality', 0.5)
            
            min_lane_dist = float('inf')
            for lx, ly in lane['points']:
                dist = math.hypot(lx - candidate['x'], ly - candidate['y'])
                if dist < min_lane_dist:
                    min_lane_dist = dist
            
            # Factor de calidad: carriles de mejor calidad tienen más peso
            quality_factor = 0.5 + osm_quality * 1.5  # 0.5 a 2.0
            
            if candidate.get('lane_idx') >= 0:
                lane_idx = candidate['lane_idx']
                lane_idx_factor = gaussian_pdf(lane_idx, 0.5)
                likelihood *= (1.0 + lane_idx_factor * osm_weight * quality_factor)
            
            sigma_osm = float(self.get_parameter('sigma_osm').value)
            osm_likelihood = gaussian_pdf(min_lane_dist, sigma_osm)
            likelihood *= (osm_likelihood ** (osm_weight * quality_factor))
            
            if candidate.get('is_lane_level', False):
                likelihood *= 1.8  # Más bonus para lane-level

        if self.last_best_offset is not None:
            offset_diff = abs(candidate['offset'] - self.last_best_offset)
            trans_sigma = max(expected_dist / 3.0, 0.1)
            likelihood *= gaussian_pdf(offset_diff, trans_sigma)
            
            if self.lane_change_cooldown > 0:
                likelihood *= 0.5

        return max(likelihood, 1e-12)
        
    def update_hmm(self, alpha=0.8):
        """Update HMM beliefs"""
        candidates = self.generate_candidates()
        if not candidates:
            return None, None

        dt = 0.1
        pred_dx = 0.0
        pred_dy = 0.0
        
        if self.robot_pose and self.robot_speed:
            pred_dx = self.robot_speed * math.cos(self.robot_pose[2]) * dt
            pred_dy = self.robot_speed * math.sin(self.robot_pose[2]) * dt

        likelihoods = np.array([self.compute_candidate_likelihood(c, pred_dx, pred_dy) for c in candidates], dtype=float)
        if np.sum(likelihoods) <= 0:
            likelihoods = np.ones(len(candidates))

        prior = np.ones(len(candidates)) / len(candidates)
        if self.candidates and self.beliefs is not None:
            prev = self.candidates
            prev_mids = [ (p['x'], p['y']) for p in prev ]
            cur_mids = [ (c['x'], c['y']) for c in candidates ]
            
            mapped_prior = np.zeros(len(candidates))
            for j, cm in enumerate(cur_mids):
                dists = [math.hypot(cm[0]-pm[0], cm[1]-pm[1]) for pm in prev_mids]
                if len(dists)==0:
                    mapped_prior[j] = 1.0/len(candidates)
                else:
                    idx = int(np.argmin(dists))
                    mapped_prior[j] = self.beliefs[idx] * math.exp(-dists[idx]/5.0)
            
            if np.sum(mapped_prior) > 0:
                mapped_prior = mapped_prior / np.sum(mapped_prior)
                prior = alpha * mapped_prior + (1.0 - alpha) * (np.ones(len(candidates)) / len(candidates))

        posterior = prior * likelihoods
        if np.sum(posterior) <= 0:
            posterior = np.ones(len(candidates))
        posterior = posterior / np.sum(posterior)

        self.candidates = candidates
        self.beliefs = posterior

        return candidates, posterior
        
    def generate_lane_ego_path(self, candidates, beliefs):
        """Generate lane ego path - MODIFICADO: usa OSM cuando es bueno"""
        if not candidates or beliefs is None:
            return None, None, None, None, 0.0, None
            
        best_idx = np.argmax(beliefs)
        best_candidate = candidates[best_idx]
        best_belief = beliefs[best_idx]
        
        current_lane_id = best_candidate.get('osm_lane_id')
        if (current_lane_id and self.last_best_lane_id and 
            current_lane_id != self.last_best_lane_id):
            self.lane_change_counter += 1
            self.lane_change_cooldown = 5
            #self.get_logger().info(f'Lane change detected: {self.last_best_lane_id} -> {current_lane_id}')
        
        self.last_best_offset = best_candidate['offset']
        self.last_best_lane_id = current_lane_id
        self.offset_history.append(self.last_best_offset)
        
        if self.lane_change_cooldown > 0:
            self.lane_change_cooldown -= 1
        
        lookahead = float(self.get_parameter('lookahead_distance').value)
        build_from_osm = bool(self.get_parameter('build_from_osm').value)
        hybrid_mode = bool(self.get_parameter('hybrid_mode').value)
        osm_quality_threshold = float(self.get_parameter('osm_quality_threshold').value)
        
        lane_center_calzada_points = None
        
        # Decidir de dónde construir el camino
        if build_from_osm and best_candidate.get('osm_lane') is not None:
            lane = best_candidate['osm_lane']
            osm_quality = best_candidate.get('osm_quality', 0.5)
            
            if hybrid_mode:
                # MODO HÍBRIDO: usar OSM solo si es de buena calidad
                if osm_quality >= osm_quality_threshold:
                    #self.get_logger().debug(f'Using OSM lane (quality: {osm_quality:.2f})')
                    lane_center_calzada_points = self.build_lane_from_osm_lane(
                        lane, 
                        (best_candidate['x'], best_candidate['y']), 
                        lookahead
                    )
                    self.current_osm_lane = lane
                    self.current_osm_lane_points = lane_center_calzada_points if lane_center_calzada_points else []
                    self.publish_osm_lane_debug(lane)
                else:
                    #self.get_logger().debug(f'OSM quality too low ({osm_quality:.2f} < {osm_quality_threshold}), using path')
                    lane_center_calzada_points = self.build_lane_from_candidate(best_candidate, lookahead)
                    self.current_osm_lane = None
                    self.current_osm_lane_points = []
            else:
                # MODO OSM PURO
                lane_center_calzada_points = self.build_lane_from_osm_lane(
                    lane, 
                    (best_candidate['x'], best_candidate['y']), 
                    lookahead
                )
                self.current_osm_lane = lane
                self.current_osm_lane_points = lane_center_calzada_points if lane_center_calzada_points else []
                self.publish_osm_lane_debug(lane)
        else:
            # FALLBACK: construir desde path
            lane_center_calzada_points = self.build_lane_from_candidate(best_candidate, lookahead)
            self.current_osm_lane = None
            self.current_osm_lane_points = []
        
        if not lane_center_calzada_points:
            return None, None, None, None, 0.0, None
        
        lane_width = float(self.get_parameter('lane_width').value)
        lane_left_points = []
        lane_right_points = []
        lane_ego_points = []
        
        for x, y, yaw in lane_center_calzada_points:
            nx = -math.sin(yaw)
            ny = math.cos(yaw)
            
            left_x = x + nx * (lane_width / 2)
            left_y = y + ny * (lane_width / 2)
            right_x = x - nx * (lane_width / 2)
            right_y = y - ny * (lane_width / 2)
            
            lane_ego_x = (x + right_x) / 2.0
            lane_ego_y = (y + right_y) / 2.0
            
            lane_left_points.append((left_x, left_y))
            lane_right_points.append((right_x, right_y))
            lane_ego_points.append((lane_ego_x, lane_ego_y, yaw))
        
        if lane_ego_points and len(lane_ego_points) > 2:
            lane_ego_points = self.smooth_path_chaikin(lane_ego_points, iterations=2)
        
        lane_info = {
            'lane_id': current_lane_id,
            'lane_idx': best_candidate.get('lane_idx', -1),
            'is_lane_level': best_candidate.get('is_lane_level', False),
            'lane_change_count': self.lane_change_counter,
            'belief': best_belief,
            'offset': self.last_best_offset,
            'osm_quality': best_candidate.get('osm_quality', 0.0),
            'build_source': 'OSM' if self.current_osm_lane else 'PATH'
        }
        
        return lane_ego_points, lane_center_calzada_points, lane_left_points, lane_right_points, best_belief, lane_info
    
    def publish_osm_lane_debug(self, lane):
        """Publicar carril OSM para debug"""
        msg = Path()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in lane['points']:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.1  # Un poco más alto para verlo
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        
        self.osm_lane_debug_pub.publish(msg)

    def publish_nearby_osm_elements(self):
        """Publica elementos OSM cercanos con contexto relativo al lane ego"""
        if self.robot_pose is None:
            return

        x, y, yaw = self.robot_pose

        msg = NearbyOSMElements()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Añadir información contextual del carril
        if self.last_best_lane_id:
            msg.current_lane_id = self.last_best_lane_id
            msg.lane_level_localization = True
        else:
            msg.lane_level_localization = False
        
        msg.lane_width = float(self.get_parameter('lane_width').value)
        
        # Calcular posición longitudinal del robot en el carril
        if self.current_osm_lane_points:
            # Encontrar punto más cercano en el carril
            min_dist = float('inf')
            closest_idx = 0
            accumulated_dist = 0.0
            
            for i, (lx, ly, lyaw) in enumerate(self.current_osm_lane_points):
                dist = math.hypot(lx - x, ly - y)
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
            
            # Calcular distancia acumulada hasta el punto más cercano
            for i in range(closest_idx):
                if i < len(self.current_osm_lane_points) - 1:
                    x1, y1, _ = self.current_osm_lane_points[i]
                    x2, y2, _ = self.current_osm_lane_points[i+1]
                    accumulated_dist += math.hypot(x2 - x1, y2 - y1)
            
            msg.robot_longitudinal_position = accumulated_dist
        
        # Procesar elementos OSM
        for f in self.osm_features:
            fx, fy = f["pos"]
            dist = math.hypot(fx - x, fy - y)

            # Radio de interés ajustado por tipo de elemento
            max_distance = 40.0
            if f["type"] == OSMElement.FEATURE_STOP:
                max_distance = 30.0
            elif f["type"] == OSMElement.FEATURE_CROSSING:
                max_distance = 25.0
            elif f["type"] == OSMElement.FEATURE_TRAFFIC_CALMING:
                max_distance = 15.0
                
            if dist > max_distance:
                continue

            el = OSMElement()
            el.header = msg.header
            el.feature_type = f["type"]
            el.distance = dist
            el.latitude = f.get("lat", 0.0)
            el.longitude = f.get("lon", 0.0)
            el.osm_id = f["id"]
            el.tags = [f"{k}={v}" for k, v in f["tags"].items()]
            el.sub_type = f["sub_type"]
            el.feature_name = f["sub_type"]
            el.confidence = 1.0  # Máxima confianza para elementos OSM estáticos
            
            # --- Añadir información relativa al carril ---
            if self.current_osm_lane_points:
                # Transformar al sistema del lane ego
                lane_longitudinal, lane_lateral = self.transform_to_lane_ego_frame(fx, fy)
                
                if lane_longitudinal is not None:
                    el.longitudinal_distance = lane_longitudinal
                    el.lateral_offset = lane_lateral
                    el.is_ahead = lane_longitudinal > 0
                    
                    # Calcular ángulo relativo
                    dx = fx - x
                    dy = fy - y
                    element_yaw = math.atan2(dy, dx)
                    relative_yaw = element_yaw - yaw
                    
                    # Normalizar ángulo a [-π, π]
                    while relative_yaw > math.pi:
                        relative_yaw -= 2 * math.pi
                    while relative_yaw < -math.pi:
                        relative_yaw += 2 * math.pi
                    
                    el.relative_angle = relative_yaw
                    
                    # Asignar índice de carril
                    if self.current_osm_lane:
                        el.lane_index = self.current_osm_lane.get('lane_idx', -1)
            
            msg.nearby_elements.append(el)

        self.pub_osm_context.publish(msg)

    


    def publish_lane_reconstruction(self, lane_ego_points, lane_center_points, left_points, right_points, lane_info=None):
        """Publish lane reconstruction"""
        if not lane_ego_points or not lane_center_points:
            return
        
        center_msg = Path()
        center_msg.header = Header()
        center_msg.header.frame_id = 'map'
        center_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y, yaw in lane_center_points:
            pose = PoseStamped()
            pose.header = center_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            center_msg.poses.append(pose)
        
        self.lane_center_pub.publish(center_msg)
        
        left_msg = Path()
        left_msg.header = center_msg.header
        for x, y in left_points:
            pose = PoseStamped()
            pose.header = left_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            left_msg.poses.append(pose)
        
        self.lane_left_pub.publish(left_msg)
        
        right_msg = Path()
        right_msg.header = center_msg.header
        for x, y in right_points:
            pose = PoseStamped()
            pose.header = right_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            right_msg.poses.append(pose)
        
        self.lane_right_pub.publish(right_msg)
        
        lane_ego_msg = Path()
        lane_ego_msg.header = center_msg.header
        
        for x, y, yaw in lane_ego_points:
            pose = PoseStamped()
            pose.header = lane_ego_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            lane_ego_msg.poses.append(pose)
        
        self.lane_ego_pub.publish(lane_ego_msg)
        
        if lane_info:
            self.publish_lane_info_markers(lane_info)
        
    def publish_lane_info_markers(self, lane_info):
        """Publicar marcadores con información del carril actual"""
        if not self.robot_pose:
            return
            
        marker_array = MarkerArray()
        
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lane_info'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.robot_pose[0]
        marker.pose.position.y = self.robot_pose[1]
        marker.pose.position.z = 2.0
        
        marker.color = ColorRGBA()
        if lane_info.get('is_lane_level', False):
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            lane_mode = "LANE-LEVEL"
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            lane_mode = "PATH-RELATIVE"
        
        marker.color.a = 1.0
        marker.scale.z = 0.5
        
        lane_id = lane_info.get('lane_id', 'N/A')
        lane_idx = lane_info.get('lane_idx', -1)
        changes = lane_info.get('lane_change_count', 0)
        belief = lane_info.get('belief', 0.0)
        offset = lane_info.get('offset', 0.0)
        osm_quality = lane_info.get('osm_quality', 0.0)
        build_source = lane_info.get('build_source', 'UNKNOWN')
        
        marker.text = (f"Mode: {lane_mode}\n"
                      f"Lane: {lane_id}\n"
                      f"Idx: {lane_idx}\n"
                      f"Source: {build_source}\n"
                      f"Quality: {osm_quality:.2f}\n"
                      f"Changes: {changes}\n"
                      f"Belief: {belief:.3f}\n"
                      f"Offset: {offset:.2f}m")
        
        marker_array.markers.append(marker)
        
        self.lane_info_pub.publish(marker_array)
        
    def publish_candidates_visualization(self, candidates, beliefs):
        """Visualize candidates with colors based on belief"""
        if not candidates:
            return
            
        marker_array = MarkerArray()
        
        for i, (candidate, belief) in enumerate(zip(candidates, beliefs)):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'candidates'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = candidate['x']
            marker.pose.position.y = candidate['y']
            marker.pose.position.z = 0.0
            
            marker.color = ColorRGBA()
            
            if candidate.get('is_lane_level', False):
                marker.color.r = 0.0
                marker.color.g = 0.5 + belief * 0.5
                marker.color.b = 1.0 - belief * 0.5
            else:
                marker.color.r = 1.0 - belief
                marker.color.g = belief
                marker.color.b = 0.0
            
            marker.color.a = 0.7
            marker.scale.x = 0.2 + belief * 0.3
            marker.scale.y = 0.2 + belief * 0.3
            marker.scale.z = 0.2 + belief * 0.3
            
            marker_array.markers.append(marker)
        
        self.candidates_pub.publish(marker_array)
        
    def publish_beliefs(self, beliefs):
        """Publish belief probabilities for debugging"""
        msg = Float32MultiArray()
        msg.data = beliefs.tolist()
        self.belief_pub.publish(msg)
    
    def transform_to_lane_ego_frame(self, global_x, global_y):
        """Transforma coordenadas globales a sistema de referencia del lane ego"""
        if not self.current_osm_lane_points:
            return None, None
        
        # Encontrar el punto más cercano en el lane ego
        min_dist = float('inf')
        closest_idx = 0
        for i, (x_ego, y_ego, yaw_ego) in enumerate(self.current_osm_lane_points):
            dist = math.hypot(global_x - x_ego, global_y - y_ego)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        if closest_idx < len(self.current_osm_lane_points):
            x_ref, y_ref, yaw_ref = self.current_osm_lane_points[closest_idx]
            
            # Transformar a coordenadas del lane ego
            dx = global_x - x_ref
            dy = global_y - y_ref
            
            # Rotar al sistema de referencia del lane ego
            cos_yaw = math.cos(yaw_ref)
            sin_yaw = math.sin(yaw_ref)
            
            lane_x = dx * cos_yaw + dy * sin_yaw  # Longitudinal
            lane_y = -dx * sin_yaw + dy * cos_yaw  # Lateral
            
            # Calcular distancia longitudinal acumulada
            longitudinal_dist = 0.0
            if lane_x > 0:  # Adelante del punto de referencia
                # Sumar desde closest_idx hacia adelante
                for i in range(closest_idx, len(self.current_osm_lane_points) - 1):
                    x1, y1, _ = self.current_osm_lane_points[i]
                    x2, y2, _ = self.current_osm_lane_points[i+1]
                    segment_length = math.hypot(x2 - x1, y2 - y1)
                    if i == closest_idx:
                        # Porción del segmento actual
                        portion = math.hypot(x_ref + lane_x * cos_yaw - lane_y * sin_yaw - x1,
                                            y_ref + lane_x * sin_yaw + lane_y * cos_yaw - y1)
                        longitudinal_dist += max(0, portion)
                    else:
                        longitudinal_dist += segment_length
            else:  # Atrás del punto de referencia
                # Sumar desde closest_idx hacia atrás
                for i in range(closest_idx - 1, -1, -1):
                    if i >= 0 and i < len(self.current_osm_lane_points) - 1:
                        x1, y1, _ = self.current_osm_lane_points[i]
                        x2, y2, _ = self.current_osm_lane_points[i+1]
                        segment_length = math.hypot(x2 - x1, y2 - y1)
                        if i == closest_idx - 1:
                            # Porción del segmento actual
                            portion = segment_length - math.hypot(
                                x_ref + lane_x * cos_yaw - lane_y * sin_yaw - x1,
                                y_ref + lane_x * sin_yaw + lane_y * cos_yaw - y1
                            )
                            longitudinal_dist += max(0, portion)
                        else:
                            longitudinal_dist += segment_length
            
            return longitudinal_dist, lane_y
        
        return None, None
    
    def check_goal_reached(self):
        """Verifica si el robot ha llegado a la meta del global_path"""
        if not self.robot_pose or not self.global_path_points:
            return False
        
        # Obtener la posición final del global_path
        if len(self.global_path_points) > 0:
            goal_x, goal_y = self.global_path_points[-1]
            
            # Calcular distancia entre robot y meta
            robot_x, robot_y, _ = self.robot_pose
            distance_to_goal = math.hypot(robot_x - goal_x, robot_y - goal_y)
            
            # Verificar si está dentro del umbral
            if distance_to_goal <= self.goal_threshold:
                return True
        
        return False
    
    def publish_goal_status(self, reached):
        """Publica el estado de llegada a la meta"""
        msg = Bool()
        msg.data = reached
        self.goal_reached_pub.publish(msg)
            
    def update(self):
        """Main update loop"""
        if not self.global_path_points or not self.robot_pose:
            self.publish_goal_status(False)
            return
        
        # Verificar si se alcanzó la meta
        goal_reached = self.check_goal_reached()
        # Solo actualizar si cambia el estado
        if goal_reached != self.goal_reached:
            self.goal_reached = goal_reached
            self.publish_goal_status(goal_reached)

        candidates, beliefs = self.update_hmm()
        
        if candidates is None or beliefs is None:
            return
            
        result = self.generate_lane_ego_path(candidates, beliefs)
        if result[0] is None:
            return
            
        lane_ego_points, center_points, left_points, right_points, best_belief, lane_info = result
        
        self.publish_lane_reconstruction(lane_ego_points, center_points, left_points, right_points, lane_info)
        self.publish_candidates_visualization(candidates, beliefs)
        self.publish_beliefs(beliefs)
        self.publish_nearby_osm_elements()

        
        if lane_info:
            lane_status = f"{lane_info.get('build_source', 'UNKNOWN')}"
            if lane_info.get('is_lane_level', False):
                lane_status += f" (Lane-Level: {lane_info.get('lane_id', 'N/A')})"
            
            if self.right_lane_points and center_points:
                center_x, center_y, _ = center_points[0]
                
                min_dist = float('inf')
                for cam_x, cam_y in self.right_lane_points:
                    dist = math.hypot(cam_x - center_x, cam_y - center_y)
                    if dist < min_dist:
                        min_dist = dist
                
                expected_dist = float(self.get_parameter('lane_width').value) / 2.0
                error = abs(min_dist - expected_dist)
                
                """self.get_logger().info(
                    f'Lane EGO: {len(center_points)} points, '
                    f'Source: {lane_status}, '
                    f'Quality: {lane_info.get("osm_quality", 0.0):.2f}, '
                    f'Belief: {best_belief:.3f}, '
                    f'Offset: {self.last_best_offset:.2f}m, '
                    f'Right Lane Error: {error:.2f}m'
                )"""

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaneEgoGenerator()
    
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