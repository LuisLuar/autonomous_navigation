#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class GlobalPathMapToOdom(Node):

    def __init__(self):
        super().__init__('global_path_map_to_odom')

        self.sub_path = self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            10
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odometry/local',
            self.odom_callback,
            10
        )

        self.pub_lane_ego = self.create_publisher(
            Path,
            '/global_path_odom',
            10
        )

        self.pub_marker = self.create_publisher(
            Marker,
            '/lane_osm',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.robot_pose = None
        self.robot_yaw = None
        self.last_lane_ego = None
        self.global_path_odom_base = None

        # Parámetros configurables
        self.lookahead_distance = 10.0  # metros de vista hacia adelante
        self.path_end_threshold = 2.0   # metros para considerar que llegó al final
        self.target_frame = 'odom'
        
        # Variables de estado
        self.has_path = False
        self.path_received_time = None
        
        self.get_logger().info('Nodo Path map → lane_ego + lane_osm iniciado')

    # =========================
    # ODOM CALLBACK
    # =========================
    def odom_callback(self, msg: Odometry):
        self.robot_pose = msg.pose.pose.position
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.has_path:
            self.publish_lane_marker()

    # =========================
    # PATH CALLBACK
    # =========================
    def path_callback(self, msg: Path):
        if self.robot_pose is None:
            return

        if len(msg.poses) < 2:
            self.has_path = False
            # Publicar marcador vacío para limpiar
            self.clear_marker()
            return

        # =========================
        # 1️⃣ TRANSFORMAR PATH A ODOM
        # =========================
        path_odom = []

        for pose in msg.poses:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time()
                )
                pose_odom = do_transform_pose_stamped(pose, transform)
                path_odom.append(pose_odom)
            except Exception as e:
                self.get_logger().warn(f'Transform error: {e}')
                continue

        if len(path_odom) < 2:
            self.has_path = False
            self.clear_marker()
            return

        # =========================
        # 2️⃣ YAW DEL PATH
        # =========================
        p0 = path_odom[0].pose.position
        p1 = path_odom[1].pose.position

        yaw_path = math.atan2(p1.y - p0.y, p1.x - p0.x)
        yaw_correction = self.robot_yaw - yaw_path

        cos_y = math.cos(yaw_correction)
        sin_y = math.sin(yaw_correction)

        # =========================
        # 3️⃣ LANE_EGO
        # =========================
        lane_ego = Path()
        lane_ego.header.frame_id = 'odom'
        lane_ego.header.stamp = self.get_clock().now().to_msg()

        for pose in path_odom:
            x = pose.pose.position.x - p0.x
            y = pose.pose.position.y - p0.y

            x_r = cos_y * x - sin_y * y
            y_r = sin_y * x + cos_y * y

            ego_pose = PoseStamped()
            ego_pose.header.frame_id = 'odom'
            ego_pose.header.stamp = lane_ego.header.stamp
            ego_pose.pose.position.x = self.robot_pose.x + x_r
            ego_pose.pose.position.y = self.robot_pose.y + y_r
            ego_pose.pose.position.z = 0.0
            ego_pose.pose.orientation.w = 1.0

            lane_ego.poses.append(ego_pose)

        self.last_lane_ego = lane_ego
        self.global_path_odom_base = lane_ego
        self.has_path = True
        self.path_received_time = self.get_clock().now()
        
        self.pub_lane_ego.publish(lane_ego)

    def get_aligned_path(self):
        if self.global_path_odom_base is None or self.robot_pose is None:
            return None

        path = self.global_path_odom_base.poses

        # Verificar si el path está vacío
        if len(path) == 0:
            return None

        # 1️⃣ índice más cercano
        min_dist = float('inf')
        idx = 0
        for i, p in enumerate(path):
            dx = p.pose.position.x - self.robot_pose.x
            dy = p.pose.position.y - self.robot_pose.y
            d = math.hypot(dx, dy)
            if d < min_dist:
                min_dist = d
                idx = i

        # Verificar si estamos cerca del final del path
        last_pose = path[-1].pose.position
        dist_to_end = math.hypot(last_pose.x - self.robot_pose.x,
                                 last_pose.y - self.robot_pose.y)
        
        # Si estamos muy cerca del final, no mostrar marcador
        if dist_to_end < self.path_end_threshold:
            return None

        # 2️⃣ yaw local del path
        if idx + 1 >= len(path):
            return None

        p0 = path[idx].pose.position
        p1 = path[idx + 1].pose.position
        yaw_path = math.atan2(p1.y - p0.y, p1.x - p0.x)

        delta_yaw = self.robot_yaw - yaw_path
        cos_y = math.cos(delta_yaw)
        sin_y = math.sin(delta_yaw)

        aligned = []

        for pose in path[idx:]:
            x = pose.pose.position.x - p0.x
            y = pose.pose.position.y - p0.y

            x_r = cos_y * x - sin_y * y
            y_r = sin_y * x + cos_y * y

            pt = PoseStamped()
            pt.pose.position.x = self.robot_pose.x + x_r
            pt.pose.position.y = self.robot_pose.y + y_r
            pt.pose.position.z = 0.0
            aligned.append(pt)

        return aligned

    # =========================
    # MARKER (VENTANA DESLIZANTE)
    # =========================
    def publish_lane_marker(self):
        poses = self.get_aligned_path()
        
        # Si no hay poses o estamos en el final, publicar marcador vacío
        if poses is None or len(poses) < 2:
            self.clear_marker()
            return

        # 🔑 punto más cercano al robot
        min_dist = float('inf')
        start_idx = 0

        for i, p in enumerate(poses):
            dx = p.pose.position.x - self.robot_pose.x
            dy = p.pose.position.y - self.robot_pose.y
            d = math.hypot(dx, dy)
            if d < min_dist:
                min_dist = d
                start_idx = i

        # Asegurarse de que start_idx no sea el último punto
        if start_idx >= len(poses) - 1:
            self.clear_marker()
            return

        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lane_osm'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15

        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        marker.color.a = 1.0

        dist_acc = 0.0
        points_added = False

        for i in range(start_idx, len(poses) - 1):
            p = poses[i].pose.position
            p_next = poses[i + 1].pose.position

            marker.points.append(Point(x=p.x, y=p.y, z=0.05))
            points_added = True

            step = math.hypot(p_next.x - p.x, p_next.y - p.y)
            dist_acc += step

            if dist_acc >= self.lookahead_distance:
                # Añadir el último punto para completar la línea
                if i + 1 < len(poses):
                    p_last = poses[i + 1].pose.position
                    marker.points.append(Point(x=p_last.x, y=p_last.y, z=0.05))
                break

        # Si no se añadió ningún punto o hay muy pocos puntos, limpiar
        if not points_added or len(marker.points) < 2:
            self.clear_marker()
            return

        self.pub_marker.publish(marker)

    def clear_marker(self):
        """Publica un marcador vacío para limpiar la visualización"""
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lane_osm'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETEALL  # Esto elimina el marcador
        
        # Alternativa: publicar marcador sin puntos
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        marker.color.a = 0.0  # Transparente
        
        self.pub_marker.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathMapToOdom()

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