#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
from collections import deque

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

from custom_interfaces.msg import SegmentationData

class LaneMemorySegmenterNode(Node):
    def __init__(self):
        super().__init__('lane_memory_segmenter')

        # ==================== PARÁMETROS ====================
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('max_age', 8.0)
        self.declare_parameter('voxel_size', 0.15)
        self.declare_parameter('slice_length', 0.15)
        self.declare_parameter('lookahead', 2.0)
        self.declare_parameter('lane_half_width', 1.5)
        self.declare_parameter('smoothing_factor', 0.7)
        self.declare_parameter('max_prediction_curvature', 10.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.max_age = self.get_parameter('max_age').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.slice_length = self.get_parameter('slice_length').value
        self.lookahead = self.get_parameter('lookahead').value
        self.lane_half_width = self.get_parameter('lane_half_width').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.max_prediction_curvature = self.get_parameter('max_prediction_curvature').value

        # ==================== MEMORIA ====================
        self.lane_memory = []          # edge points [x, y, t]
        self.lane_mask_memory = []     # mask points [x, y, t]
        self.lane_width_est = 3.0
        self.lane_width_alpha = 0.15

        self.last_good_target = None
        self.last_good_prediction = None
        self.last_good_curvature = None
        self.last_good_time = None
        self.nominal_lookahead = self.lookahead
        self.freeze_timeout = 3.0

        self.target_history = deque(maxlen=5)

        self.robot_pose = None
        self.centerline = None
        self.edge_centerline = None
        self.mask_centerline = None

        # ==================== SUB / PUB ====================
        self.create_subscription(PointCloud2, '/lane/edge_points', self.edge_callback, 10)
        self.create_subscription(SegmentationData, '/segmentation/data', self.mask_callback, 10)
        self.create_subscription(Odometry, '/odometry/local', self.odom_callback, 10)

        self.pub_prediction = self.create_publisher(Marker, '/lane/prediction', 10)
        self.pub_target = self.create_publisher(Marker, '/lane/target_point', 10)
        self.pub_target_vis = self.create_publisher(Marker, '/lane/target_vis', 10)
        self.pub_curvature = self.create_publisher(Float32, '/lane/curvature', 10)
        self.pub_e_lat = self.create_publisher(Float32, '/lane/error_lateral', 10)
        self.pub_e_head = self.create_publisher(Float32, '/lane/error_heading', 10)

        # ==================== TIMER ====================
        self.create_timer(0.1, self.timer_callback)

    # ================== CALLBACKS =====================
    def edge_callback(self, msg: PointCloud2):
        now = self.get_clock().now().nanoseconds * 1e-9
        for p in point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True):
            self.lane_memory.append([p[0], p[1], now])

    def mask_callback(self, msg: SegmentationData):
        now = self.get_clock().now().nanoseconds * 1e-9
        mask_array = np.frombuffer(msg.mask_data, dtype=np.uint8).reshape(msg.height, msg.width)
        ys, xs = np.where(mask_array == 2)  # solo carril
        for x, y in zip(xs, ys):
            self.lane_mask_memory.append([float(x), float(y), now])

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(q)
        self.robot_pose = {'x': p.x, 'y': p.y, 'yaw': yaw}

    # ================== TIMER ==========================
    def timer_callback(self):
        if self.robot_pose is None or (len(self.lane_memory) + len(self.lane_mask_memory)) < 5:
            return

        self.forget_old()
        self.forget_old_mask()

        self.edge_centerline = self.process_pointcloud(self.lane_memory)
        self.mask_centerline = self.process_pointcloud(self.lane_mask_memory)

        # Calcular ancho estimado si existen ambos
        if self.edge_centerline is not None and self.mask_centerline is not None:
            dists = [np.min(np.linalg.norm(self.edge_centerline - pm, axis=1)) for pm in self.mask_centerline]
            if len(dists) > 0:
                width_inst = 2.0 * np.mean(dists)
                self.lane_width_est = (self.lane_width_alpha * width_inst +
                                       (1 - self.lane_width_alpha) * self.lane_width_est)

        # Escoger centerline a usar
        if self.mask_centerline is not None:
            self.centerline = self.mask_centerline
        else:
            self.centerline = self.edge_centerline

        use_freeze = self.centerline is None or len(self.centerline) < 5
        prediction, target, curvature = None, None, 0.0

        if not use_freeze:
            prediction, target, curvature = self.predict_lane(self.centerline)
            if target is None and self.last_good_target is not None:
                target = self.last_good_target
            elif target is not None:
                self.last_good_target = target.copy()

            self.last_good_prediction = prediction.copy() if prediction is not None else None
            self.last_good_curvature = curvature
            self.last_good_time = time.time()

        else:
            if self.last_good_time is None:
                return
            age = time.time() - self.last_good_time
            if age > self.freeze_timeout:
                return
            prediction = self.last_good_prediction
            target = self.last_good_target
            curvature = self.last_good_curvature

        # Suavizado de target
        if target is not None:
            self.target_history.append(target)
            target_smoothed = np.mean(np.array(self.target_history), axis=0)
        else:
            target_smoothed = None

        e_lat, e_heading, closest = self.compute_errors(prediction, target_smoothed) if target_smoothed is not None else (0.0, 0.0, None)

        self.publish_errors(e_lat, e_heading, closest)
        self.publish_prediction_marker(prediction)
        if target_smoothed is not None:
            self.publish_target_marker(target_smoothed)
        self.publish_curvature(curvature)

    # ================== UTILIDADES =====================
    def forget_old(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.lane_memory = [p for p in self.lane_memory if now - p[2] < self.max_age]

    def forget_old_mask(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.lane_mask_memory = [p for p in self.lane_mask_memory if now - p[2] < self.max_age]

    def process_pointcloud(self, memory):
        if len(memory) < 5:
            return None
        voxels = {}
        for x, y, _ in memory:
            if np.isnan(x) or np.isnan(y):
                continue
            key = (int(x / self.voxel_size), int(y / self.voxel_size))
            voxels.setdefault(key, []).append((x, y))
        cloud = [np.mean(np.array(v), axis=0) for v in voxels.values() if len(v) > 0]
        if len(cloud) < 5:
            return None
        cloud = np.array(cloud)
        cloud = cloud[cloud[:, 0].argsort()]
        centers = []
        x_min, x_max = cloud[0, 0], cloud[-1, 0]
        x = x_min
        while x < x_max:
            pts = cloud[(cloud[:, 0] >= x) & (cloud[:, 0] < x + self.slice_length)]
            if len(pts) > 2:
                centers.append([pts[:, 0].mean(), pts[:, 1].mean()])
            x += self.slice_length
        return np.array(centers) if len(centers) >= 2 else None

    def euler_from_quaternion(self, q):
        import math
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # Roll, Pitch no usados
        return 0.0, 0.0, yaw

    # ================== PREDICCIÓN Y TARGET =====================
    def predict_lane(self, centerline):
        if len(centerline) < 5 or self.robot_pose is None:
            return None, None, 0.0
        rx, ry, yaw = self.robot_pose['x'], self.robot_pose['y'], self.robot_pose['yaw']
        robot_pos = np.array([rx, ry])
        forward = np.array([np.cos(yaw), np.sin(yaw)])

        x = centerline[:, 0]
        y = centerline[:, 1]
        coeffs = np.polyfit(x, y, 2)
        poly = np.poly1d(coeffs)

        xs = np.linspace(x.min(), x.max(), 200)
        ys = poly(xs)
        curve = np.column_stack((xs, ys))
        vecs = curve - robot_pos
        ahead_mask = np.dot(vecs, forward) > 0.2
        curve_ahead = curve[ahead_mask]
        if len(curve_ahead) < 5:
            return None, None, 0.0

        dists = np.linalg.norm(np.diff(curve_ahead, axis=0), axis=1)
        s = np.insert(np.cumsum(dists), 0, 0.0)
        idx = np.argmin(np.abs(s - self.lookahead))
        base_pt = curve_ahead[idx]
        dy_dx = 2 * coeffs[0] * base_pt[0] + coeffs[1]
        normal = np.array([-dy_dx, 1.0])
        normal /= np.linalg.norm(normal)
        offset = self.lane_half_width if self.mask_centerline is not None else self.lane_width_est / 2.0
        target = base_pt + normal * offset
        pred_base = curve_ahead[:idx+10]
        prediction = pred_base + normal * offset
        # Curvature
        k_now = abs(2*coeffs[0]) / (1 + dy_dx**2)**1.5
        idx_preview = np.argmin(np.abs(s - (self.lookahead + self.max_prediction_curvature)))
        idx_preview = min(idx_preview, len(curve_ahead)-1)
        preview_pt = curve_ahead[idx_preview]
        dy_dx_future = 2 * coeffs[0] * preview_pt[0] + coeffs[1]
        k_future = abs(2*coeffs[0]) / (1 + dy_dx_future**2)**1.5
        curvature = 0.4 * k_now + 0.6 * k_future
        return prediction, target, curvature

    # ================== ERRORES =====================
    def compute_errors(self, prediction, target_point):
        if prediction is None or target_point is None or self.robot_pose is None:
            return 0.0, 0.0, None
        pred_array = np.array(prediction)
        target_array = np.array(target_point)
        distances = np.linalg.norm(pred_array - target_array, axis=1)
        closest_idx = np.argmin(distances)
        P0 = pred_array[closest_idx]
        P1 = pred_array[min(closest_idx+1, len(pred_array)-1)]
        robot_pos = np.array([self.robot_pose['x'], self.robot_pose['y']])
        v = P1 - P0
        w = robot_pos - P0
        v_norm = np.linalg.norm(v)
        if v_norm < 1e-3:
            return 0.0, 0.0, None
        e_lat = (-v[0]*w[1] + v[1]*w[0]) / v_norm
        dx, dy = target_point[0] - robot_pos[0], target_point[1] - robot_pos[1]
        target_heading = np.arctan2(dy, dx)
        e_heading = np.arctan2(np.sin(target_heading - self.robot_pose['yaw']),
                               np.cos(target_heading - self.robot_pose['yaw']))
        t = np.dot(w, v) / np.dot(v, v)
        closest = P0 + np.clip(t, 0.0, 1.0)*v
        return e_lat, e_heading, closest

    # ================== PUBLISH =====================
    def publish_prediction_marker(self, prediction):
        if prediction is None or len(prediction) == 0:
            return
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "prediction"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.05
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 0.7, 1.0, 1.0)
        for x, y in prediction:
            marker.points.append(Point(x=float(x), y=float(y), z=0.0))
        self.pub_prediction.publish(marker)

    def publish_target_marker(self, target):
        if target is None or self.robot_pose is None:
            return
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "lane_target"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.scale.x = 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = (0.0, 1.0, 0.0, 1.0)
        p0 = Point(x=self.robot_pose['x'], y=self.robot_pose['y'], z=0.1)
        p1 = Point(x=target[0], y=target[1], z=0.1)
        m.points = [p0, p1]
        self.pub_target_vis.publish(m)

    def publish_curvature(self, curvature):
        msg = Float32()
        msg.data = curvature
        self.pub_curvature.publish(msg)

    def publish_errors(self, e_lat, e_heading, closest):
        msg = Float32()
        msg.data = e_lat
        self.pub_e_lat.publish(msg)
        msg.data = e_heading
        self.pub_e_head.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneMemorySegmenterNode()
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
