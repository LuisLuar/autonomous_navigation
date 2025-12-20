#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Mensajes ROS2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray, Marker

# Tu mensaje personalizado
from custom_interfaces.msg import NearbyOSMElements, OSMElement
from std_msgs.msg import Header

# GIS
from pyproj import Transformer
import osmium
import rtree
import shapely.geometry as geom

class OSMFeature:
    """Clase para características OSM"""
    def __init__(self, feature_type, lat, lon, tags=None, osm_id=None):
        self.type = feature_type  # 'traffic_calming', 'crossing', 'other'
        self.lat = lat
        self.lon = lon
        self.tags = tags or {}
        self.osm_id = osm_id
        self.geometry = geom.Point(lon, lat)
        
    def is_traffic_calming(self):
        return self.type == 'traffic_calming'
    
    def is_crossing(self):
        return self.type == 'crossing'
    
    def get_traffic_type(self):
        return self.tags.get('traffic_calming', 'unknown')
    
    def distance_to(self, other_lat, other_lon):
        """Distancia en metros usando fórmula haversine"""
        from math import radians, sin, cos, sqrt, atan2
        
        R = 6371000.0  # Radio de la Tierra en metros
        
        lat1 = radians(self.lat)
        lon1 = radians(self.lon)
        lat2 = radians(other_lat)
        lon2 = radians(other_lon)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        
        return R * c

class OSMParserHandler(osmium.SimpleHandler):
    """Manejador para parsear archivos OSM"""
    def __init__(self):
        super().__init__()
        self.features = []
        
    def node(self, n):
        if n.tags:
            tags = dict(n.tags)
            
            # Solo procesar elementos de interés
            if tags.get('traffic_calming') == 'bump':
                feature = OSMFeature(
                    'traffic_calming',
                    n.location.lat,
                    n.location.lon,
                    tags,
                    n.id
                )
                self.features.append(feature)
                
            elif tags.get('highway') == 'crossing':
                feature = OSMFeature(
                    'crossing',
                    n.location.lat,
                    n.location.lon,
                    tags,
                    n.id
                )
                self.features.append(feature)

class OSMPlannerNode(Node):
    def __init__(self):
        super().__init__('information_osm')
        
        # Parámetros
        self.declare_parameter('osm_file', '/home/raynel/autonomous_navigation/src/navigation_system/maps/espe_actualizadov2.osm')
        self.declare_parameter('search_radius', 5.0)  # 5 metros como solicitaste
        self.declare_parameter('utm_zone', 17)
        
        self.osm_file = self.get_parameter('osm_file').get_parameter_value().string_value
        self.search_radius = self.get_parameter('search_radius').value
        self.utm_zone = self.get_parameter('utm_zone').value
        
        # Variables de estado
        self.utm_origin = None
        self.transformer = None
        self.osm_features = []
        self.rtree_idx = None
        self.origin_received = False
        self.osm_loaded = False
        
        # Última posición
        self.last_map_pos = (0.0, 0.0)
        self.last_geo_pos = (0.0, 0.0)
        
        # QoS
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # SUSCRIPTORES
        self.origin_sub = self.create_subscription(
            PointStamped,
            '/utm_map_origin',
            self.origin_callback,
            qos_profile
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odom_callback,
            qos_profile
        )
        
        # PUBLICADORES
        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/osm/visualization_markers',
            10
        )
        
        # ¡AHORA CON TU MENSAJE PERSONALIZADO!
        self.nearby_pub = self.create_publisher(
            NearbyOSMElements,
            '/osm/nearby_features',
            10
        )
        
        # Inicializar
        self.init_transformers()
        
        # Cargar OSM
        if self.osm_file:
            #self.get_logger().info(f'Cargando OSM desde: {self.osm_file}')
            self.load_osm()
        
        #self.get_logger().info('Nodo OSM Planner con mensaje personalizado iniciado')
    
    def init_transformers(self):
        """Inicializar transformadores de coordenadas"""
        try:
            self.transformer = Transformer.from_crs(
                "EPSG:32717",  # UTM zona 17S
                "EPSG:4326",   # WGS84 (lat/lon)
                always_xy=True
            )
            #self.get_logger().info('Transformador UTM→LatLon inicializado')
        except Exception as e:
            #self.get_logger().error(f'Error inicializando transformador: {e}')
            pass
    
    def load_osm(self):
        """Cargar archivo OSM"""
        try:
            handler = OSMParserHandler()
            
            parser = osmium.io.Reader(self.osm_file)
            osmium.apply(parser, handler)
            parser.close()
            
            self.osm_features = handler.features
            self.build_rtree_index()
            self.osm_loaded = True
            
            # Estadísticas
            traffic_count = sum(1 for f in self.osm_features if f.is_traffic_calming())
            crossing_count = sum(1 for f in self.osm_features if f.is_crossing())
            
            """self.get_logger().info(f'OSM cargado: {len(self.osm_features)} características')
            self.get_logger().info(f'   - Reductores: {traffic_count}')
            self.get_logger().info(f'   - Pasos peatonales: {crossing_count}')"""
            
        except Exception as e:
            #self.get_logger().error(f' Error cargando OSM: {e}')
            pass
    
    def build_rtree_index(self):
        """Construir índice R-tree para búsquedas rápidas"""
        self.rtree_idx = rtree.index.Index()
        
        for i, feature in enumerate(self.osm_features):
            self.rtree_idx.insert(i, (feature.lon, feature.lat, feature.lon, feature.lat))
    
    def origin_callback(self, msg):
        """Callback para recibir origen UTM"""
        if not self.origin_received:
            self.utm_origin = (msg.point.x, msg.point.y)
            self.origin_received = True
            
            #self.get_logger().info(f'Origen UTM recibido: ({msg.point.x:.2f}, {msg.point.y:.2f})')
    
    def odom_callback(self, msg):
        """Callback para odometría global"""
        map_x = msg.pose.pose.position.x
        map_y = msg.pose.pose.position.y
        
        self.last_map_pos = (map_x, map_y)
        
        # Convertir a lat/lon si tenemos origen
        if self.origin_received and self.transformer and self.osm_loaded:
            self.convert_and_find_nearby(map_x, map_y)
    
    def convert_and_find_nearby(self, map_x, map_y):
        """Transformar posición y buscar elementos cercanos"""
        try:
            # 1. Map → UTM absoluto
            utm_x = self.utm_origin[0] + map_x
            utm_y = self.utm_origin[1] + map_y
            
            # 2. UTM → lat/lon
            lon, lat = self.transformer.transform(utm_x, utm_y)
            
            self.last_geo_pos = (lat, lon)
            
            # 3. Buscar características cercanas
            nearby_features = self.find_nearby_features(lat, lon)
            
            # 4. Publicar información
            if nearby_features:
                self.publish_nearby_elements(nearby_features, lat, lon)
                self.publish_visualization_markers(nearby_features)
            
        except Exception as e:
            #self.get_logger().warn(f'  Error en transformación: {e}')
            pass
    
    def find_nearby_features(self, lat, lon):
        """Buscar características OSM cercanas"""
        radius_deg = self.search_radius / 111000.0
        
        bbox = (lon - radius_deg, lat - radius_deg, 
                lon + radius_deg, lat + radius_deg)
        
        nearby_indices = list(self.rtree_idx.intersection(bbox))
        
        nearby_features = []
        for idx in nearby_indices:
            feature = self.osm_features[idx]
            distance = feature.distance_to(lat, lon)
            
            if distance <= self.search_radius:
                feature.distance = distance
                nearby_features.append(feature)
        
        # Ordenar por distancia
        nearby_features.sort(key=lambda f: f.distance)
        
        return nearby_features
    
    def publish_nearby_elements(self, features, lat, lon):
        """Publicar información de elementos cercanos usando mensaje personalizado"""
        msg = NearbyOSMElements()
        
        # Encabezado
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Información de posición actual
        #self.get_logger().info(f' Posición actual: ({lat:.6f}, {lon:.6f})')
        
        # Convertir cada feature a OSMElement
        for feature in features:
            element = OSMElement()
            element.header = Header()
            element.header.stamp = self.get_clock().now().to_msg()
            element.header.frame_id = "map"
            
            # Tipo de característica
            if feature.is_traffic_calming():
                element.feature_type = NearbyOSMElements.FEATURE_TRAFFIC_CALMING
                feature_name = "Reductor de velocidad"
                sub_type = feature.get_traffic_type()
            elif feature.is_crossing():
                element.feature_type = NearbyOSMElements.FEATURE_CROSSING
                feature_name = "Paso peatonal"
                sub_type = "zebra_crossing"
            else:
                element.feature_type = NearbyOSMElements.FEATURE_UNKNOWN
                feature_name = "Desconocido"
                sub_type = "unknown"
            
            # Información detallada
            element.distance = float(feature.distance)
            element.latitude = feature.lat
            element.longitude = feature.lon
            element.osm_id = str(feature.osm_id)
            element.feature_name = feature_name
            element.sub_type = sub_type
            
            # Convertir tags a lista de strings
            element.tags = [f"{key}={value}" for key, value in feature.tags.items()]
            
            msg.nearby_elements.append(element)
            
            # Log informativo
            self.get_logger().info(
                f'   🚦 {feature_name} ({sub_type}) '
                f'a {element.distance:.2f}m - ID: {element.osm_id}'
            )
        
        # Publicar mensaje
        self.nearby_pub.publish(msg)
    
    def publish_visualization_markers(self, features):
        """Publicar markers para RViz"""
        marker_array = MarkerArray()
        
        # Marker para robot
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "robot"
        robot_marker.id = 0
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = self.last_map_pos[0]
        robot_marker.pose.position.y = self.last_map_pos[1]
        robot_marker.pose.position.z = 0.3
        robot_marker.scale.x = 0.5
        robot_marker.scale.y = 0.5
        robot_marker.scale.z = 0.5
        robot_marker.color.r = 0.0
        robot_marker.color.g = 1.0
        robot_marker.color.b = 0.0
        robot_marker.color.a = 0.8
        marker_array.markers.append(robot_marker)
        
        # Radio de búsqueda
        radius_marker = Marker()
        radius_marker.header.frame_id = "map"
        radius_marker.header.stamp = self.get_clock().now().to_msg()
        radius_marker.ns = "radius"
        radius_marker.id = 1
        radius_marker.type = Marker.CYLINDER
        radius_marker.action = Marker.ADD
        radius_marker.pose.position.x = self.last_map_pos[0]
        radius_marker.pose.position.y = self.last_map_pos[1]
        radius_marker.pose.position.z = 0.01
        radius_marker.scale.x = self.search_radius * 2
        radius_marker.scale.y = self.search_radius * 2
        radius_marker.scale.z = 0.02
        radius_marker.color.r = 0.3
        radius_marker.color.g = 0.3
        radius_marker.color.b = 0.8
        radius_marker.color.a = 0.2
        marker_array.markers.append(radius_marker)
        
        # Markers para características
        for i, feature in enumerate(features[:10]):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "osm"
            marker.id = i + 100
            marker.type = Marker.CUBE
            
            # Posición estimada en mapa
            marker.pose.position.x = self.last_map_pos[0] + (feature.lon - self.last_geo_pos[1]) * 100000
            marker.pose.position.y = self.last_map_pos[1] + (feature.lat - self.last_geo_pos[0]) * 100000
            marker.pose.position.z = 0.5
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Color según tipo
            if feature.is_traffic_calming():
                marker.color.r = 1.0  # Rojo
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker_type = "🚧"
            else:
                marker.color.r = 0.0  # Azul
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker_type = "🚶"
            
            marker.color.a = 0.8
            
            # Texto con información
            marker.text = f"{marker_type} {feature.distance:.1f}m"
            
            marker_array.markers.append(marker)
        
        self.markers_pub.publish(marker_array)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = OSMPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info('Nodo OSM Planner detenido')
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()