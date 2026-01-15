#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os

import xml.etree.ElementTree as ET
import numpy as np
import networkx as nx
import math
from collections import defaultdict

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from pyproj import Transformer
from shapely.geometry import LineString

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
# CONFIG
INTERP_DIST = 3.0
LANE_WIDTH = 1.5
CONNECT_DIST = 2.0

# Penalizaciones
COST_FACTORS = {
    "road": 1.0,
    "intersection": 1.2,
    "lane_change": 1.5,
    "u_turn": 5.0,
    "bridge": 10.0,
}

# Transformers
transformer_ll_to_xy = Transformer.from_crs("EPSG:4326", "EPSG:32717", always_xy=True)

# HELPERS
def latlon_to_xy(lat, lon):
    """Convierte lat/lon a UTM"""
    try:
        x, y = transformer_ll_to_xy.transform(lon, lat)
        return (x, y)
    except Exception as e:
        #self.get_logger()(f"Error convirtiendo lat/lon a UTM: {e}")
        return (0, 0)

def interpolate_line(coords, step):
    clean = []
    for c in coords:
        if not clean or c != clean[-1]:
            clean.append(c)
    if len(clean) < 2:
        return []
    line = LineString(clean)
    if line.length < 0.01:
        return clean

    pts = []
    d = 0.0
    while d <= line.length:
        p = line.interpolate(d)
        pts.append((p.x, p.y))
        d += step

    end = line.interpolate(line.length)
    if pts[-1] != (end.x, end.y):
        pts.append((end.x, end.y))
    return pts

def unit_normal(p1, p2):
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    L = math.hypot(dx, dy)
    if L == 0:
        return (0, 0)
    return (-dy / L, dx / L)

def nearest_node(G, p):
    return min(G.nodes, key=lambda n: np.hypot(n[0] - p[0], n[1] - p[1]))

def cost(u, v, data):
    base = data.get("weight", 0)
    edge_type = data.get("type", "road")
    
    if edge_type == "lane_change":
        penalty = 5.0
    elif edge_type == "intersection":
        penalty = 3.0
    elif edge_type == "u_turn":
        penalty = 20.0
    elif edge_type == "bridge":
        penalty = 50.0
    else:
        penalty = 0
    
    return base + penalty

def find_osm_file():
    possible_paths = [
        os.path.join(os.path.dirname(__file__), "maps", "casa.osm"), #espe_actualizadov2
        os.path.join(os.path.dirname(__file__), "casa.osm"),
        "/home/raynel/autonomous_navigation/src/navigation_system/maps/casa.osm",
        "casa.osm",
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            return path
    
    return None

# ROS NODE
class OfflineGlobalPlanner(Node):

    def __init__(self):
        super().__init__("offline_global_planner")

        # Buscar archivo OSM
        self.osm_file = find_osm_file()
        if self.osm_file is None:
            raise FileNotFoundError("No se encontró el archivo OSM")
        
        #self.G = self.build_complete_graph()
        
        # Variables de estado
        self.current_position_utm = None  # Posición actual en UTM
        self.goal_utm = None  # Goal en UTM
        
        # Usar GPS para posición actual (no odometría)
        self.sub_gps = self.create_subscription(
            NavSatFix, "/gps/filtered", self.gps_cb, 10
        )
        
        # Usar goal en lat/lon
        self.sub_goal = self.create_subscription(
            GeoPoint, "/goal_latlon", self.goal_cb, 10
        )

        self.utm_origin = None

        self.sub_origin = self.create_subscription(
            PointStamped,
            "/utm_map_origin",
            self.origin_cb,
            1
        )

        self.sub_clear = self.create_subscription(
            Bool,
            "/clear_path",
            self.clear_cb,
            10
        )


        self.pub_path = self.create_publisher(Path, "/global_path", 10)

    def clear_cb(self, msg):
        if msg.data:
            #self.get_logger().info("Clear path signal received.")
            self.goal_utm = None
            self.current_position_map = None
            empty_path = Path()
            empty_path.header.frame_id = "map"
            empty_path.header.stamp = self.get_clock().now().to_msg()
            self.pub_path.publish(empty_path)

    def origin_cb(self, msg):
        if self.utm_origin is not None:
            return

        self.utm_origin = (msg.point.x, msg.point.y)

        #self.get_logger().info(f"Planner received UTM origin: {self.utm_origin}")

        self.G = self.build_complete_graph()
        #self.get_logger().info(f"OSM graph built: {len(self.G.nodes)} nodes, {len(self.G.edges)} edges")


    def utm_to_map(self, utm):
        return (
            utm[0] - self.utm_origin[0],
            utm[1] - self.utm_origin[1]
        )


    # CALLBACKS
    def gps_cb(self, msg):
        if msg.status.status < 0:
            return

        if self.utm_origin is None:
            #self.get_logger().warn("GPS received but UTM origin not set yet")
            return

        utm_x, utm_y = latlon_to_xy(msg.latitude, msg.longitude)
        self.current_position_map = self.utm_to_map((utm_x, utm_y))


    def goal_cb(self, msg):

        if self.utm_origin is None:
            #self.get_logger().warn("Goal received but UTM origin not set yet")
            return

        self.goal_utm = latlon_to_xy(msg.latitude, msg.longitude)
        self.goal_map = self.utm_to_map(self.goal_utm)
        self.compute_path()


    # CONSTRUCCIÓN DEL GRAFO
    def build_complete_graph(self):
        G = nx.DiGraph()
        
        try:
            tree = ET.parse(self.osm_file)
        except ET.ParseError as e:
            return G
        
        root = tree.getroot()

        # Extraer nodos OSM
        u_turn_nodes = set()

        for n in root.findall("node"):
            tags = {t.attrib["k"]: t.attrib["v"] for t in n.findall("tag")}
            if tags.get("u_turn") == "yes":
                u_turn_nodes.add(n.attrib["id"])
        
        osm_nodes = {}
        for n in root.findall("node"):
            try:
                lat = float(n.attrib["lat"])
                lon = float(n.attrib["lon"])
                osm_nodes[n.attrib["id"]] = self.utm_to_map(latlon_to_xy(lat, lon))
            except (KeyError, ValueError) as e:
                continue

        # Estructuras para conexiones
        way_info = {}
        osm_node_to_ways = defaultdict(list)
        
        # 1. PROCESAR WAYS (CARRILES BÁSICOS)
        way_count = 0
        for way in root.findall("way"):
            tags = {t.attrib["k"]: t.attrib["v"] for t in way.findall("tag")}
            
            # Filtrar por tipo de vía
            highway_type = tags.get("highway")
            if highway_type not in ["service", "residential", "unclassified"]:
                continue
            
            way_id = way.attrib["id"]
            oneway = tags.get("oneway", "no")
            
            # Guardar información del way
            node_refs = [nd.attrib["ref"] for nd in way.findall("nd")]
            way_info[way_id] = {
                'oneway': oneway,
                'node_refs': node_refs
            }
            
            # Registrar nodos OSM usados por este way
            for node_ref in node_refs:
                if node_ref in osm_nodes:
                    osm_node_to_ways[node_ref].append(way_id)
            
            # Obtener coordenadas interpoladas
            coords = [osm_nodes[r] for r in node_refs if r in osm_nodes]
            interp = interpolate_line(coords, INTERP_DIST)
            if len(interp) < 2:
                continue

            # Configurar carriles según dirección
            lanes = int(tags.get("lanes", 1))
            lanes_fwd = int(tags.get("lanes:forward", lanes))
            lanes_bwd = int(tags.get("lanes:backward", 0))

            lane_configs = []

            for i in range(lanes_fwd):
                lane_configs.append(("forward", i))

            for i in range(lanes_bwd):
                lane_configs.append(("backward", i))

            road_width = float(tags.get("width", lanes * 3.0))
            lane_width = road_width / lanes

            # Generar cada carril
            for direction, lane_idx in lane_configs:
                # Calcular offset según dirección y carril
                if direction == 'forward':
                    sign = 1
                    lane_offset = (0.75 - lane_idx) * lane_width
                else:  # backward
                    sign = -1
                    lane_offset = (lane_idx - 0.75) * lane_width
                
                # Generar puntos del carril
                lane_points = []
                for i in range(len(interp)):
                    # Calcular normal en este punto
                    if i == 0:
                        p1, p2 = interp[0], interp[1]
                    elif i == len(interp) - 1:
                        p1, p2 = interp[-2], interp[-1]
                    else:
                        p1, p2 = interp[i-1], interp[i+1]
                    
                    nx_, ny_ = unit_normal(p1, p2)
                    point = interp[i]
                    offset_point = (point[0] + nx_ * lane_offset * sign, 
                                  point[1] + ny_ * lane_offset * sign)
                    lane_points.append(offset_point)
                
                # Agregar aristas al grafo
                for i in range(len(lane_points) - 1):
                    p1 = lane_points[i]
                    p2 = lane_points[i + 1]
                    dist = np.hypot(p2[0] - p1[0], p2[1] - p1[1])
                    
                    # Agregar nodos
                    G.add_node(p1)
                    G.add_node(p2)
                    
                    # Determinar dirección de la arista
                    if direction == 'forward':
                        G.add_edge(p1, p2, weight=dist, way_id=way_id, 
                                  direction=direction, lane_index=lane_idx, type="road")
                    else:  # backward
                        G.add_edge(p2, p1, weight=dist, way_id=way_id,
                                  direction=direction, lane_index=lane_idx, type="road")

            way_count += 1

        # 2. CONECTAR INTERSECCIONES
        connection_count = self.connect_intersections(G, osm_nodes, osm_node_to_ways)

        # 3. CONECTAR CARRILES DEL MISMO SENTIDO
        lane_change_count = self.connect_lane_changes(G)
        # 4. CONEXIONES PARA GIROS EN U
        u_turn_count = self.connect_u_turns(G, osm_nodes, way_info, u_turn_nodes)

        return G

    def connect_intersections(self, G, osm_nodes, osm_node_to_ways):
        connection_count = 0
        
        intersection_osm_nodes = {node: ways for node, ways in osm_node_to_ways.items() if len(ways) >= 2}
        
        for osm_node_id, way_ids in intersection_osm_nodes.items():
            if len(way_ids) < 2:
                continue
            
            int_point = osm_nodes[osm_node_id]
            
            nearby_points = []
            for way_id in way_ids:
                for u, v, data in G.edges(data=True):
                    if data.get('way_id') == way_id:
                        dist_u = np.hypot(u[0] - int_point[0], u[1] - int_point[1])
                        dist_v = np.hypot(v[0] - int_point[0], v[1] - int_point[1])
                        
                        if dist_u < 5:
                            nearby_points.append((u, way_id))
                        if dist_v < 5:
                            nearby_points.append((v, way_id))
            
            for i in range(len(nearby_points)):
                for j in range(i+1, len(nearby_points)):
                    p1, way1 = nearby_points[i]
                    p2, way2 = nearby_points[j]
                    
                    if way1 == way2:
                        continue
                    
                    dist = np.hypot(p1[0] - p2[0], p1[1] - p2[1])
                    
                    if dist < 10:
                        if not G.has_edge(p1, p2):
                            G.add_edge(p1, p2, weight=dist, type='intersection')
                            connection_count += 1
                        if not G.has_edge(p2, p1):
                            G.add_edge(p2, p1, weight=dist, type='intersection')
                            connection_count += 1
        
        return connection_count

    def connect_lane_changes(self, G):
        lane_change_count = 0
        
        points_by_way_dir = defaultdict(list)
        for node in G.nodes():
            for _, _, data in G.out_edges(node, data=True):
                if 'way_id' in data and 'direction' in data:
                    key = (data['way_id'], data['direction'])
                    points_by_way_dir[key].append(node)
        
        for (way_id, direction), points in points_by_way_dir.items():
            unique_points = list(set(points))
            
            for i in range(len(unique_points)):
                for j in range(i+1, len(unique_points)):
                    p1 = unique_points[i]
                    p2 = unique_points[j]
                    dist = np.hypot(p1[0] - p2[0], p1[1] - p2[1])
                    
                    if dist < 3.0:
                        if not G.has_edge(p1, p2):
                            G.add_edge(p1, p2, weight=dist, type='lane_change')
                            lane_change_count += 1
                        if not G.has_edge(p2, p1):
                            G.add_edge(p2, p1, weight=dist, type='lane_change')
                            lane_change_count += 1
        
        return lane_change_count

    def connect_u_turns(self, G, osm_nodes, way_info, u_turn_nodes):
        u_turn_count = 0

        for node_id in u_turn_nodes:
            if node_id not in osm_nodes:
                continue

            center = osm_nodes[node_id]

            nearby = []
            for n in G.nodes():
                if np.hypot(n[0] - center[0], n[1] - center[1]) < 6.0:
                    nearby.append(n)

            for i in range(len(nearby)):
                for j in range(i+1, len(nearby)):
                    p1 = nearby[i]
                    p2 = nearby[j]

                    dist = np.hypot(p1[0] - p2[0], p1[1] - p2[1])
                    if dist < 12.0:
                        if not G.has_edge(p1, p2):
                            G.add_edge(p1, p2, weight=dist, type="u_turn")
                            u_turn_count += 1
                        if not G.has_edge(p2, p1):
                            G.add_edge(p2, p1, weight=dist, type="u_turn")
                            u_turn_count += 1

        return u_turn_count


    # CÁLCULO DE RUTA
    def compute_path(self):
        if self.utm_origin is None:
            #self.get_logger().warn("UTM origin not set yet")
            return

        if not hasattr(self, "G") or self.G is None or len(self.G.nodes) == 0:
            #self.get_logger().warn("Graph not ready")
            return

        if self.current_position_map is None:
            #self.get_logger().warn("Current position not available")
            return

        if self.goal_map is None:
            #self.get_logger().warn("Goal not available")
            return


        # Encontrar nodos más cercanos
        start = nearest_node(self.G, self.current_position_map)
        goal  = nearest_node(self.G, self.goal_map)


        try:
            path_nodes = nx.astar_path(
                self.G,
                start,
                goal,
                heuristic=lambda a, b: np.hypot(a[0]-b[0], a[1]-b[1]),
                weight=cost
            )
            
        except nx.NetworkXNoPath:
            return
        
        # Crear mensaje Path
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Analizar tipos de segmentos
        segment_types = []
        for i in range(len(path_nodes) - 1):
            u, v = path_nodes[i], path_nodes[i+1]
            if self.G.has_edge(u, v):
                edge_data = self.G.get_edge_data(u, v)
                segment_types.append(edge_data.get('type', 'road'))
            
            # Añadir punto al path
            p = PoseStamped()
            p.header = path_msg.header
            p.pose.position.x = u[0]
            p.pose.position.y = u[1]
            p.pose.orientation.w = 1.0
            path_msg.poses.append(p)
        
        # Añadir último punto
        last_point = PoseStamped()
        last_point.header = path_msg.header
        last_point.pose.position.x = path_nodes[-1][0]
        last_point.pose.position.y = path_nodes[-1][1]
        last_point.pose.orientation.w = 1.0
        path_msg.poses.append(last_point)
        
        # Estadísticas
        from collections import Counter
        type_counts = Counter(segment_types)
        stats_str = ", ".join([f"{k}: {v}" for k, v in type_counts.items()])

        # Calcular distancia total
        total_distance = 0
        for i in range(len(path_nodes) - 1):
            u, v = path_nodes[i], path_nodes[i+1]
            total_distance += np.hypot(v[0] - u[0], v[1] - u[1])
        
        direct_distance = np.hypot(goal[0] - start[0], goal[1] - start[1])
        detour_factor = total_distance / direct_distance if direct_distance > 0 else 1
        
        # Publicar rutaf
        self.pub_path.publish(path_msg)
    
    def destroy_node(self):
        super().destroy_node()


# ======================
def main():
    rclpy.init()
    node = OfflineGlobalPlanner()
    
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


if __name__ == "__main__":
    main()