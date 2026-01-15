#!/usr/bin/env python3
"""
Nodo de Información OSM con lane_ego - Versión corregida
========================================================
Procesa elementos OSM y los proyecta sobre el lane_ego
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import math
import os
import warnings
warnings.filterwarnings('ignore')

# Mensajes ROS2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, Bool

# Mensaje personalizado
try:
    from custom_interfaces.msg import NearbyOSMElements, OSMElement
    CUSTOM_INTERFACE_AVAILABLE = True
except ImportError as e:
    print(f"ADVERTENCIA: No se pudo importar custom_interfaces: {e}")
    CUSTOM_INTERFACE_AVAILABLE = False

# GIS - importar con manejo de errores
GIS_AVAILABLE = False
try:
    from pyproj import Transformer
    GIS_AVAILABLE = True
except ImportError as e:
    print(f"ADVERTENCIA: No se pudo importar pyproj: {e}")

try:
    import osmium
    OSM_AVAILABLE = True
except ImportError as e:
    print(f"ADVERTENCIA: No se pudo importar osmium: {e}")
    OSM_AVAILABLE = False

try:
    import shapely.geometry as geom
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False

from collections import deque


class OSMFeature:
    """Clase para características OSM"""
    def __init__(self, feature_type, lat, lon, tags=None, osm_id=None, node_id=None):
        self.type = feature_type  # 'traffic_calming', 'crossing', 'intersection'
        self.lat = lat
        self.lon = lon
        self.tags = tags or {}
        self.osm_id = osm_id
        self.node_id = node_id
        self.distance_to_lane = float('inf')
        self.progress_along_lane = 0.0
        self.lane_offset = 0.0
        self.distance_to_robot = float('inf')
        
    def is_traffic_calming(self):
        return self.type == 'traffic_calming'
    
    def is_crossing(self):
        return self.type == 'crossing'
    
    def is_intersection(self):
        return self.type == 'intersection'
    
    def distance_to_point(self, lat, lon):
        """Distancia haversine en metros"""
        R = 6371000.0
        
        lat1 = math.radians(self.lat)
        lon1 = math.radians(self.lon)
        lat2 = math.radians(lat)
        lon2 = math.radians(lon)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c


class SimpleOSMHandler(osmium.SimpleHandler):
    """Manejador simple para OSM"""
    def __init__(self):
        super().__init__()
        self.features = []
        self.nodes = {}
        
    def node(self, n):
        # Guardar todos los nodos
        self.nodes[n.id] = (n.location.lat, n.location.lon)
        
        if n.tags:
            tags = dict(n.tags)
            
            # 1. REDUCTORES DE VELOCIDAD
            if tags.get('traffic_calming') == 'bump':
                feature = OSMFeature(
                    'traffic_calming',
                    n.location.lat,
                    n.location.lon,
                    tags,
                    f"n{n.id}",
                    n.id
                )
                self.features.append(feature)
                
            # 2. PASOS PEATONALES
            elif tags.get('highway') == 'crossing':
                feature = OSMFeature(
                    'crossing',
                    n.location.lat,
                    n.location.lon,
                    tags,
                    f"n{n.id}",
                    n.id
                )
                self.features.append(feature)


class OSMInformationNode(Node):
    def __init__(self):
        super().__init__('osm_information_node')
        
        # Parámetros
        self.declare_parameter('osm_file', '')
        self.declare_parameter('search_radius', 5.0)
        self.declare_parameter('lookahead_distance', 10.0)
        self.declare_parameter('goal_distance_threshold', 2.0)
        self.declare_parameter('utm_zone', 17)
        self.declare_parameter('enable_osm', True)
        
        self.osm_file = self.get_parameter('osm_file').get_parameter_value().string_value
        self.search_radius = self.get_parameter('search_radius').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.goal_threshold = self.get_parameter('goal_distance_threshold').value
        self.utm_zone = self.get_parameter('utm_zone').value
        self.enable_osm = self.get_parameter('enable_osm').value
        
        # Variables de estado
        self.utm_origin = None
        self.transformer_utm_to_wgs = None
        self.transformer_wgs_to_utm = None
        self.osm_features = []
        self.origin_received = False
        self.osm_loaded = False
        
        # Estado del sistema
        self.robot_pose_map = None
        self.global_path = None
        self.lane_ego_path = None
        self.current_lane_ego_idx = 0
        
        # Historial
        self.feature_history = deque(maxlen=10)
        self.goal_reached_history = deque(maxlen=5)
        
        # QoS
        qos_system = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        qos_path = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE 
        )
        
        # Suscriptores
        self.origin_sub = self.create_subscription(
            PointStamped,
            '/utm_map_origin',
            self.origin_callback,
            qos_system
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odom_callback,
            qos_system
        )
        
        self.global_path_sub = self.create_subscription(
            Path,
            '/global_path',
            self.global_path_callback,
            qos_path
        )
        
        self.lane_ego_sub = self.create_subscription(
            Path,
            '/path/global',
            self.lane_ego_callback,
            qos_system
        )
        
        # Publicadores
        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/osm/visualization_markers',
            10
        )
        
        if CUSTOM_INTERFACE_AVAILABLE:
            self.nearby_pub = self.create_publisher(
                NearbyOSMElements,
                '/osm/nearby_features',
                10
            )
        #else:
            #self.get_logger().warn('custom_interfaces no disponible')
        
        self.goal_reached_pub = self.create_publisher(
            Bool,
            '/goal_reached',
            10
        )
        
        # Verificar si podemos procesar OSM
        self.gis_available = GIS_AVAILABLE and OSM_AVAILABLE
        
        if self.gis_available:
            self.init_transformers()
        else:
            #self.get_logger().warn('Librerías GIS no disponibles, OSM deshabilitado')
            self.enable_osm = False
        
        #self.get_logger().info(f'Nodo OSM Information iniciado (OSM: {self.enable_osm})')
        
        # Timer
        self.create_timer(0.5, self.process_information)
    
    def init_transformers(self):
        """Inicializar transformadores de coordenadas"""
        try:
            # UTM → WGS84 (lat/lon) - zona 17S
            self.transformer_utm_to_wgs = Transformer.from_crs(
                "EPSG:32717",  # Zona 17S fija
                "EPSG:4326",   # WGS84
                always_xy=True
            )
            
            # WGS84 (lat/lon) → UTM - zona 17S
            self.transformer_wgs_to_utm = Transformer.from_crs(
                "EPSG:4326",   # WGS84
                "EPSG:32717",  # Zona 17S fija
                always_xy=True
            )
            
            #self.get_logger().info('Transformadores UTM↔WGS84 inicializados (zona 17S)')
            
        except Exception as e:
            #self.get_logger().error(f'Error inicializando transformadores: {e}')
            self.gis_available = False
    
    def load_osm_file(self):
        """Cargar archivo OSM"""
        if not self.gis_available or not self.enable_osm:
            return False
        
        # Si no se especificó archivo, buscar automáticamente
        if not self.osm_file:
            possible_paths = [
                os.path.join(os.path.dirname(__file__), "maps", "casa.osm"),
                os.path.join(os.path.dirname(__file__), "casa.osm"),
                "/home/raynel/autonomous_navigation/src/navigation_system/maps/casa.osm",
                "/home/raynel/autonomous_navigation/install/navigation_system/share/navigation_system/maps/casa.osm",
                "casa.osm",
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    self.osm_file = path
                    #self.get_logger().info(f'Archivo OSM encontrado: {path}')
                    break
        
        if not self.osm_file or not os.path.exists(self.osm_file):
            #self.get_logger().error(f'Archivo OSM no encontrado: {self.osm_file}')
            return False
        
        try:
            handler = SimpleOSMHandler()
            parser = osmium.io.Reader(self.osm_file)
            osmium.apply(parser, handler)
            parser.close()
            
            self.osm_features = handler.features
            self.osm_loaded = True
            
            # Estadísticas
            traffic_count = sum(1 for f in self.osm_features if f.is_traffic_calming())
            crossing_count = sum(1 for f in self.osm_features if f.is_crossing())
            
            #self.get_logger().info(f'OSM cargado: {len(self.osm_features)} características')
            #self.get_logger().info(f'  - Reductores: {traffic_count}')
            #self.get_logger().info(f'  - Pasos peatonales: {crossing_count}')
            
            return True
            
        except Exception as e:
            #self.get_logger().error(f'Error cargando OSM: {e}')
            return False
    
    def origin_callback(self, msg):
        """Callback para origen UTM"""
        if not self.origin_received:
            self.utm_origin = (msg.point.x, msg.point.y)
            self.origin_received = True
            #self.get_logger().info(f'Origen UTM recibido: ({msg.point.x:.1f}, {msg.point.y:.1f})')
            
            # Cargar OSM una vez que tenemos el origen
            if self.enable_osm and not self.osm_loaded:
                self.load_osm_file()
    
    def odom_callback(self, msg):
        """Callback para odometría"""
        self.robot_pose_map = msg.pose.pose
    
    def global_path_callback(self, msg):
        """Callback para global path"""
        #self.get_logger().debug(f'Global path recibido con {len(msg.poses)} poses') 
        self.global_path = msg
    
    def lane_ego_callback(self, msg):
        """Callback para lane_ego"""
        self.lane_ego_path = msg
        if self.robot_pose_map and self.lane_ego_path and len(self.lane_ego_path.poses) > 0:
            self.update_current_lane_ego_index()
    
    def update_current_lane_ego_index(self):
        """Actualizar índice en lane_ego"""
        if not self.robot_pose_map or not self.lane_ego_path:
            return
        
        robot_x = self.robot_pose_map.position.x
        robot_y = self.robot_pose_map.position.y
        
        min_dist = float('inf')
        best_idx = 0
        
        # Buscar en todo el path
        for i in range(len(self.lane_ego_path.poses)):
            pose = self.lane_ego_path.poses[i].pose
            dist = math.hypot(robot_x - pose.position.x, robot_y - pose.position.y)
            
            if dist < min_dist:
                min_dist = dist
                best_idx = i
        
        self.current_lane_ego_idx = best_idx
        
        # Solo debug ocasional
        # if self.get_clock().now().nanoseconds % 10 == 0:
        #     #self.get_logger().debug(f'Índice lane_ego actualizado: {best_idx}')
    
    def process_information(self):
        """Procesamiento principal"""
        # Verificar condiciones mínimas
        if not self.origin_received or not self.robot_pose_map:
            # #self.get_logger().debug('Esperando origen UTM o pose del robot')
            return
        
        # 1. Verificar meta (siempre hacer esto)
        goal_reached = self.check_goal_reached()
        self.publish_goal_reached(goal_reached)
        
        # 2. Procesar OSM si está habilitado
        if not self.enable_osm or not self.osm_loaded or not self.lane_ego_path:
            return
        
        # 3. Convertir posición del robot a lat/lon
        lat, lon = self.map_to_latlon(
            self.robot_pose_map.position.x,
            self.robot_pose_map.position.y
        )
        
        if lat is None or lon is None:
            return
        
        # 4. Buscar elementos OSM cercanos
        nearby_features = []
        for feature in self.osm_features:
            distance = feature.distance_to_point(lat, lon)
            if distance <= self.search_radius:
                feature.distance_to_robot = distance
                nearby_features.append(feature)
        
        if not nearby_features:
            # #self.get_logger().debug(f'No hay elementos OSM dentro de {self.search_radius}m')
            return
        
        # 5. Proyectar sobre lane_ego
        projected_features = self.project_features_to_lane_ego(nearby_features)
        
        if not projected_features:
            # #self.get_logger().debug('No hay elementos proyectados sobre lane_ego')
            return
        
        # 6. Publicar información
        if CUSTOM_INTERFACE_AVAILABLE:
            self.publish_nearby_elements(projected_features)
        
        # 7. Publicar markers para visualización
        self.publish_visualization_markers(projected_features)
        
        # Log informativo
        #self.get_logger().info(f'Encontrados {len(projected_features)} elementos OSM cercanos')
    
    def map_to_latlon(self, map_x, map_y):
        """Convertir coordenadas mapa a lat/lon"""
        try:
            if not self.utm_origin or not self.transformer_utm_to_wgs:
                return None, None
            
            # Map → UTM absoluto
            utm_x = self.utm_origin[0] + map_x
            utm_y = self.utm_origin[1] + map_y
            
            # UTM → lat/lon
            lon, lat = self.transformer_utm_to_wgs.transform(utm_x, utm_y)
            
            # Debug ocasional
            # if self.get_clock().now().nanoseconds % 100 == 0:
            #     #self.get_logger().debug(f'Map ({map_x:.1f}, {map_y:.1f}) -> LatLon ({lat:.6f}, {lon:.6f})')
            
            return lat, lon
            
        except Exception as e:
            #self.get_logger().warn(f'Error map→latlon: {e}')
            return None, None
    
    def latlon_to_map(self, lat, lon):
        """Convertir lat/lon a coordenadas mapa"""
        try:
            if not self.utm_origin or not self.transformer_wgs_to_utm:
                return None, None
            
            # lat/lon → UTM
            utm_x, utm_y = self.transformer_wgs_to_utm.transform(lon, lat)
            
            # UTM → Map
            map_x = utm_x - self.utm_origin[0]
            map_y = utm_y - self.utm_origin[1]
            
            return map_x, map_y
            
        except Exception as e:
            #self.get_logger().warn(f'Error latlon→map: {e}')
            return None, None
    
    def project_features_to_lane_ego(self, features):
        """Proyectar características sobre lane_ego"""
        if not self.lane_ego_path or len(self.lane_ego_path.poses) < 2:
            return []
        
        projected = []
        
        for feature in features:
            # Convertir feature a coordenadas mapa
            map_x, map_y = self.latlon_to_map(feature.lat, feature.lon)
            
            if map_x is None or map_y is None:
                continue
            
            # Buscar punto más cercano en lane_ego
            closest_idx, closest_dist = self.find_closest_point_on_lane_ego(map_x, map_y)
            
            if closest_idx >= 0 and closest_dist <= self.lookahead_distance:
                feature.distance_to_lane = closest_dist
                feature.progress_along_lane = closest_idx / max(len(self.lane_ego_path.poses), 1) * 100.0
                
                # Calcular offset lateral (simplificado)
                if closest_idx < len(self.lane_ego_path.poses):
                    lane_pose = self.lane_ego_path.poses[closest_idx].pose
                    
                    # Vector del punto lane_ego al feature
                    dx = map_x - lane_pose.position.x
                    dy = map_y - lane_pose.position.y
                    
                    # Aproximación simple del offset lateral
                    feature.lane_offset = math.hypot(dx, dy)
                
                projected.append(feature)
        
        # Ordenar por progreso
        projected.sort(key=lambda f: f.progress_along_lane)
        return projected
    
    def find_closest_point_on_lane_ego(self, x, y):
        """Encontrar punto más cercano en lane_ego"""
        if not self.lane_ego_path:
            return -1, float('inf')
        
        min_dist = float('inf')
        best_idx = -1
        
        # Buscar en todo el lane_ego (podría limitarse)
        for i in range(len(self.lane_ego_path.poses)):
            pose = self.lane_ego_path.poses[i].pose
            dist = math.hypot(x - pose.position.x, y - pose.position.y)
            
            if dist < min_dist:
                min_dist = dist
                best_idx = i
        
        return best_idx, min_dist
    
    def publish_nearby_elements(self, features):
        """Publicar elementos cercanos usando mensaje personalizado"""
        try:
            msg = NearbyOSMElements()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            
            for feature in features[:5]:  # Limitar a 5 elementos
                element = OSMElement()
                element.header.stamp = self.get_clock().now().to_msg()
                element.header.frame_id = "map"
                
                # Tipo de característica
                if feature.is_traffic_calming():
                    element.feature_type = OSMElement.FEATURE_TRAFFIC_CALMING
                    element.feature_name = "Reductor de velocidad"
                    element.sub_type = feature.tags.get('traffic_calming', 'bump')
                    #self.get_logger().info(f'🚧 Reductor a {feature.distance_to_lane:.1f}m')
                    
                elif feature.is_crossing():
                    element.feature_type = OSMElement.FEATURE_CROSSING
                    element.feature_name = "Paso peatonal"
                    element.sub_type = feature.tags.get('crossing', 'uncontrolled')
                    #self.get_logger().info(f'🚶 Paso peatonal a {feature.distance_to_lane:.1f}m')
                
                else:
                    element.feature_type = OSMElement.FEATURE_UNKNOWN
                    element.feature_name = "Elemento OSM"
                    element.sub_type = "unknown"
                
                # Información básica
                element.distance = float(feature.distance_to_robot)
                element.latitude = feature.lat
                element.longitude = feature.lon
                element.osm_id = feature.osm_id
                element.longitudinal_distance = float(feature.distance_to_lane)
                element.lateral_offset = float(feature.lane_offset)
                element.progress_along_path = float(feature.progress_along_lane)
                element.is_ahead = True  # Siempre adelante por filtro
                
                # Tags como lista de strings
                element.tags = [f"{k}={v}" for k, v in feature.tags.items()]
                
                msg.nearby_elements.append(element)
            
            self.nearby_pub.publish(msg)
            
        except Exception as e:
            #self.get_logger().error(f'Error publicando elementos OSM: {e}')
            pass
    
    def publish_visualization_markers(self, features):
        """Publicar markers para RViz"""
        try:
            marker_array = MarkerArray()
            
            # Marker para robot
            if self.robot_pose_map:
                robot_marker = Marker()
                robot_marker.header.frame_id = "map"
                robot_marker.header.stamp = self.get_clock().now().to_msg()
                robot_marker.ns = "robot"
                robot_marker.id = 0
                robot_marker.type = Marker.SPHERE
                robot_marker.action = Marker.ADD
                robot_marker.pose.position = self.robot_pose_map.position
                robot_marker.pose.position.z = 0.3
                robot_marker.scale.x = 0.5
                robot_marker.scale.y = 0.5
                robot_marker.scale.z = 0.5
                robot_marker.color.r = 0.0
                robot_marker.color.g = 1.0
                robot_marker.color.b = 0.0
                robot_marker.color.a = 0.8
                marker_array.markers.append(robot_marker)
            
            # Circle de búsqueda
            if self.robot_pose_map:
                circle_marker = Marker()
                circle_marker.header.frame_id = "map"
                circle_marker.header.stamp = self.get_clock().now().to_msg()
                circle_marker.ns = "search_radius"
                circle_marker.id = 1
                circle_marker.type = Marker.CYLINDER
                circle_marker.action = Marker.ADD
                circle_marker.pose.position = self.robot_pose_map.position
                circle_marker.pose.position.z = 0.01
                circle_marker.scale.x = self.search_radius * 2
                circle_marker.scale.y = self.search_radius * 2
                circle_marker.scale.z = 0.02
                circle_marker.color.r = 0.3
                circle_marker.color.g = 0.3
                circle_marker.color.b = 0.8
                circle_marker.color.a = 0.1
                marker_array.markers.append(circle_marker)
            
            # Markers para características OSM
            for i, feature in enumerate(features[:15]):  # Limitar a 15 markers
                map_x, map_y = self.latlon_to_map(feature.lat, feature.lon)
                if map_x is None:
                    continue
                
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "osm_features"
                marker.id = i + 100
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = map_x
                marker.pose.position.y = map_y
                marker.pose.position.z = 0.5
                
                # Color según tipo
                if feature.is_traffic_calming():
                    marker.color.r = 1.0  # Rojo
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif feature.is_crossing():
                    marker.color.r = 0.0  # Azul
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                else:
                    marker.color.r = 1.0  # Amarillo
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                
                # Tamaño según distancia
                size = max(0.2, 0.5 - feature.distance_to_lane / self.lookahead_distance)
                marker.scale.x = size
                marker.scale.y = size
                marker.scale.z = size
                
                marker.color.a = 0.8
                marker.text = f"{feature.distance_to_lane:.1f}m"
                
                marker_array.markers.append(marker)
            
            self.markers_pub.publish(marker_array)
            
        except Exception as e:
            #self.get_logger().error(f'Error publicando markers: {e}')
            pass
    
    def check_goal_reached(self):
        """Verificar meta usando lane_ego en lugar de global_path"""
        if not self.robot_pose_map:
            return False
        
        # USAR LANE_EGO EN LUGAR DE GLOBAL_PATH
        if not self.lane_ego_path or len(self.lane_ego_path.poses) == 0:
            return False
        
        # 1. Obtener posición del robot
        robot_x = self.robot_pose_map.position.x
        robot_y = self.robot_pose_map.position.y
        
        # 2. Obtener la ÚLTIMA POSE del lane_ego (meta real del robot)
        goal_pose = self.lane_ego_path.poses[-1].pose
        goal = goal_pose.position
        
        # 3. Calcular distancia al FINAL del lane_ego
        distance = math.hypot(goal.x - robot_x, goal.y - robot_y)
        
        # 4. DEBUG
        """self.get_logger().info(
            f'🔍 Goal Check (lane_ego):\n'
            f'   Robot: ({robot_x:.2f}, {robot_y:.2f})\n'
            f'   LaneEgo Goal: ({goal.x:.2f}, {goal.y:.2f})\n'
            f'   Distancia: {distance:.2f}m\n'
            f'   Umbral: {self.goal_threshold}m'
        )"""
        
        # 5. Verificar si estamos cerca del FINAL del lane_ego
        is_near = distance < self.goal_threshold
        
        """if is_near:
            self.get_logger().info(f'🎯 ¡LLEGÓ AL FINAL DEL LANE_EGO! Distancia: {distance:.2f}m')
            return True"""
        
        return False
        
    def publish_goal_reached(self, reached):
        """Publicar estado de goal reached"""
        msg = Bool()
        msg.data = bool(reached)
        self.goal_reached_pub.publish(msg)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OSMInformationNode()
    
    try:
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo OSM Information detenido')
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()