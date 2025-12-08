#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
import math

# AÑADIR ESTOS IMPORT:
import osmnx as ox
import networkx as nx
from pyproj import Proj, transform

# ==================== CLASES AUXILIARES ====================

class GlobalPlanner:
    def __init__(self, osm_file):
        """Inicializa el planificador con archivo OSM"""
        self.get_logger().info(f'Cargando OSM desde: {osm_file}')
        self.graph = ox.graph_from_xml(osm_file)  # Tu espe_fixed.osm
        self.G_proj = ox.project_graph(self.graph)
        self.get_logger().info(f'Grafo cargado: {len(self.graph.nodes)} nodos, {len(self.graph.edges)} aristas')
    
    def plan_route(self, start_latlon, goal_latlon):
        """Planifica ruta entre dos puntos lat/lon"""
        # 1. Encontrar nodos más cercanos
        orig_node = ox.nearest_nodes(self.G_proj, start_latlon[1], start_latlon[0])
        dest_node = ox.nearest_nodes(self.G_proj, goal_latlon[1], goal_latlon[0])
        
        self.get_logger().info(f'Nodo inicio: {orig_node}, Nodo fin: {dest_node}')
        
        # 2. Planificar con A*
        route = nx.shortest_path(self.G_proj, orig_node, dest_node, weight='length')
        
        # 3. Extraer waypoints (lat/lon)
        route_coords = []
        for node in route:
            node_data = self.G_proj.nodes[node]
            lat, lon = node_data['y'], node_data['x']
            route_coords.append((lat, lon))
        
        self.get_logger().info(f'Ruta calculada: {len(route_coords)} waypoints')
        return route_coords


class CoordinateTransformer:
    def __init__(self, utm_zone=17, hemisphere='S'):  # Ecuador zona 17S
        """Inicializa transformador de coordenadas"""
        self.wgs84 = Proj(init='epsg:4326')
        self.utm = Proj(init=f'epsg:327{utm_zone}')  # 32717 para Ecuador Sur
        
    def latlon_to_utm(self, lat, lon):
        """Convierte lat/lon a UTM (metros)"""
        return transform(self.wgs84, self.utm, lon, lat)
    
    def utm_to_latlon(self, easting, northing):
        """Convierte UTM a lat/lon"""
        return transform(self.utm, self.wgs84, easting, northing)


# ==================== NODO PRINCIPAL ====================

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner')
        
        # Declarar parámetros
        self.declare_parameter('osm_file', '/home/raynel/Documents/offline_title/espe_fixed.osm')
        self.declare_parameter('utm_zone', 17)
        self.declare_parameter('frame_id', 'map')
        
        # Obtener parámetros
        osm_file = self.get_parameter('osm_file').value
        utm_zone = self.get_parameter('utm_zone').value
        frame_id = self.get_parameter('frame_id').value
        
        # Suscriptores
        self.create_subscription(GeoPoint, '/goal_latlon', self.goal_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.position_callback, 10)
        
        # Publicador
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)
        
        # Inicializar componentes
        self.planner = GlobalPlanner(osm_file)
        self.transformer = CoordinateTransformer(utm_zone=utm_zone)
        self.frame_id = frame_id
        
        self.current_position = None
        
        self.get_logger().info(f' GlobalPlannerNode iniciado')
        self.get_logger().info(f'   - OSM file: {osm_file}')
        self.get_logger().info(f'   - Frame ID: {frame_id}')
        self.get_logger().info(f'   - Esperando destino en /goal_latlon')
    
    def position_callback(self, msg):
        """Guardar posición actual desde GPS"""
        if msg.status.status >= 0:  # GPS válido
            self.current_position = GeoPoint()
            self.current_position.latitude = msg.latitude
            self.current_position.longitude = msg.longitude
            self.current_position.altitude = msg.altitude
    
    def goal_callback(self, msg):
        """Planificar cuando llega nuevo destino"""
        if not self.current_position:
            self.get_logger().warn('No hay posición GPS para planificar')
            return
        
        self.get_logger().info(f'Destino recibido: {msg.latitude:.6f}, {msg.longitude:.6f}')
        self.get_logger().info(f'Posición actual: {self.current_position.latitude:.6f}, {self.current_position.longitude:.6f}')
        
        start = (self.current_position.latitude, self.current_position.longitude)
        goal = (msg.latitude, msg.longitude)
        
        try:
            # Calcular ruta
            route_latlon = self.planner.plan_route(start, goal)
            
            if not route_latlon:
                self.get_logger().error('Ruta vacía calculada')
                return
            
            # Convertir a Path ROS
            path_msg = self.create_path_message(route_latlon)
            
            # Publicar
            self.path_pub.publish(path_msg)
            self.get_logger().info(f'Ruta publicada en /global_plan con {len(route_latlon)} waypoints')
            
        except Exception as e:
            self.get_logger().error(f'Error en planificación: {str(e)}')
    
    def create_path_message(self, route_latlon):
        """Convertir lista de lat/lon a mensaje Path"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        
        for i, (lat, lon) in enumerate(route_latlon):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = path.header.stamp
            
            # Convertir a UTM
            easting, northing = self.transformer.latlon_to_utm(lat, lon)
            
            pose.pose.position.x = easting
            pose.pose.position.y = northing
            pose.pose.position.z = 0.0
            
            # Orientación (hacia siguiente waypoint)
            if i < len(route_latlon) - 1:
                next_lat, next_lon = route_latlon[i + 1]
                next_easting, next_northing = self.transformer.latlon_to_utm(next_lat, next_lon)
                
                dx = next_easting - easting
                dy = next_northing - northing
                yaw = math.atan2(dy, dx)
                
                pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.orientation.w = math.cos(yaw / 2)
            else:
                pose.pose.orientation.w = 1.0
            
            path.poses.append(pose)
        
        return path


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()