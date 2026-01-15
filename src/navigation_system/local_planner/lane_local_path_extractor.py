#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class LaneLocalExtractor(Node):
    def __init__(self):
        super().__init__("lane_local_extractor")
        
        # Variables de estado
        self.lane_ego = None
        self.pose = None
        
        # Suscriptores
        self.create_subscription(Path, "/lane_ego", self.cb_lane, 10)
        self.create_subscription(Odometry, "/odometry/global", self.cb_odom, 10)
        
        # Publicador
        self.pub_local = self.create_publisher(Path, "/lane_local", 10)
        
        # Parámetros configurables
        self.declare_parameter('lookahead_distance', 8.0)  # 5-10 m
        self.declare_parameter('resolution', 0.5)          # 0.3-0.5 m
        self.declare_parameter('publish_rate', 20.0)       # 20 Hz
        
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.resolution = self.get_parameter('resolution').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Timer para publicación periódica
        self.create_timer(1.0/publish_rate, self.update)
        
        #self.get_logger().info(f"Lane Local Extractor iniciado")
        #self.get_logger().info(f"Lookahead: {self.lookahead}m, Resolución: {self.resolution}m")
    
    def cb_lane(self, msg):
        """Callback para la trayectoria global del carril"""
        self.lane_ego = msg
    
    def cb_odom(self, msg):
        """Callback para la odometría global"""
        self.pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
    
    def find_closest_point_index(self, points, robot_position):
        """Encuentra el índice del punto más cercano al robot"""
        x, y, _ = robot_position
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (px, py) in enumerate(points):
            dist = math.hypot(px - x, py - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        return closest_idx
    
    def smooth_and_resample(self, points, start_idx, total_distance):
        """Suaviza y re-muestrea los puntos"""
        if len(points) <= start_idx + 1:
            return points[start_idx:] if start_idx < len(points) else []
        
        smoothed_points = [points[start_idx]]
        accumulated_distance = 0.0
        current_idx = start_idx
        
        while accumulated_distance < total_distance and current_idx < len(points) - 1:
            # Calcular distancia entre puntos consecutivos
            dx = points[current_idx + 1][0] - points[current_idx][0]
            dy = points[current_idx + 1][1] - points[current_idx][1]
            segment_distance = math.hypot(dx, dy)
            
            if segment_distance <= self.resolution:
                # Si el segmento es más corto que la resolución, usar el punto directamente
                smoothed_points.append(points[current_idx + 1])
                accumulated_distance += segment_distance
                current_idx += 1
            else:
                # Re-muestrear el segmento
                num_samples = int(segment_distance / self.resolution)
                for j in range(1, num_samples + 1):
                    ratio = j * self.resolution / segment_distance
                    if ratio > 1.0:
                        ratio = 1.0
                    
                    x = points[current_idx][0] + dx * ratio
                    y = points[current_idx][1] + dy * ratio
                    smoothed_points.append((x, y))
                    
                    accumulated_distance += self.resolution
                    
                    if accumulated_distance >= total_distance:
                        break
                
                # Si no llegamos al final, añadir el punto final del segmento
                if accumulated_distance < total_distance and j == num_samples:
                    smoothed_points.append(points[current_idx + 1])
                    accumulated_distance += segment_distance - (num_samples * self.resolution)
                
                current_idx += 1
            
            if accumulated_distance >= total_distance:
                break
        
        return smoothed_points
    
    def update(self):
        """Método principal de actualización y publicación"""
        if self.lane_ego is None or self.pose is None:
            return
        
        # Extraer puntos del path
        points = []
        for pose_stamped in self.lane_ego.poses:
            pose = pose_stamped.pose
            points.append((pose.position.x, pose.position.y))
        
        if len(points) < 2:
            return
        
        # 1. Encontrar punto más cercano al robot
        closest_idx = self.find_closest_point_index(points, self.pose)
        
        # 2. Extraer tramo corto con lookahead
        lookahead_points = self.extract_lookahead_path(points, closest_idx, self.lookahead)
        
        if len(lookahead_points) < 2:
            return
        
        # 3. Suavizar y re-muestrear
        smoothed_points = self.smooth_and_resample(lookahead_points, 0, self.lookahead)
        
        # 4. Publicar path local
        self.publish_local_path(smoothed_points)
    
    def extract_lookahead_path(self, points, start_idx, lookahead_distance):
        """Extrae una sección del path con la distancia de lookahead especificada"""
        extracted_points = [points[start_idx]]
        accumulated_distance = 0.0
        
        for i in range(start_idx, len(points) - 1):
            dx = points[i + 1][0] - points[i][0]
            dy = points[i + 1][1] - points[i][1]
            segment_distance = math.hypot(dx, dy)
            
            if accumulated_distance + segment_distance <= lookahead_distance:
                extracted_points.append(points[i + 1])
                accumulated_distance += segment_distance
            else:
                # Interpolar el último punto
                remaining_distance = lookahead_distance - accumulated_distance
                if remaining_distance > 0:
                    ratio = remaining_distance / segment_distance
                    x = points[i][0] + dx * ratio
                    y = points[i][1] + dy * ratio
                    extracted_points.append((x, y))
                break
        
        return extracted_points
    
    def publish_local_path(self, points):
        """Publica el path local en el tópico correspondiente"""
        path_msg = Path()
        path_msg.header.frame_id = "map"  # o el frame_id apropiado
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in points:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        
        self.pub_local.publish(path_msg)
        
        # Log opcional para debugging
        # self.get_logger().info(f"Publicado path local con {len(points)} puntos", throttle_duration_sec=1.0)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = LaneLocalExtractor()
    
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