#!/usr/bin/env python3
"""
LaneMemoryNode v4  (SLAM-lite + predicción + referencia de control)
------------------------------------------------------------------
EXTIENDE v3 con:
4) Predicción corta del carril (lookahead)
5) Punto objetivo para control (Pure Pursuit / Stanley)
6) Visualización clara en RViz

NO controla el robot todavía: SOLO genera referencias geométricas.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from std_msgs.msg import Float32

from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

from sensor_msgs.msg import Imu
from collections import deque


class LaneMemoryNode(Node):
    def __init__(self):
        super().__init__('lane_memory_node_v4')

        # ==================== PARÁMETROS ====================
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('max_age', 8.0)
        self.declare_parameter('voxel_size', 0.15)
        # ==================== PARÁMETROS OPTIMIZADOS ====================
        self.declare_parameter('slice_length', 0.15)  # Más suave que 0.1
        self.declare_parameter('lookahead', 2.0)      # 2m para mejor estabilidad
        self.declare_parameter('lane_half_width', 1.5) # Ajustar según carril real
        self.declare_parameter('smoothing_factor', 0.7) # Factor de suavizado
        self.declare_parameter('max_prediction_curvature', 10.0) # Máxima distancia de predicción

        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.lane_half_width = self.get_parameter('lane_half_width').value
        self.frame_id = self.get_parameter('frame_id').value
        self.max_age = self.get_parameter('max_age').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.slice_length = self.get_parameter('slice_length').value
        self.lookahead = self.get_parameter('lookahead').value


        # Historiales para suavizado
        self.target_history = deque(maxlen=5)
        self.prediction_history = deque(maxlen=5)
        self.coeffs_history = deque(maxlen=5)


        # ==================== ESTADO ====================
        self.lane_memory = []   # [x, y, t]
        self.centerline = np.empty((0, 2))
        self.edge_centerline = None
        self.mask_centerline = None


        # ===== FREEZE GEOMÉTRICO =====
        self.last_good_centerline = None
        self.last_good_prediction = None
        self.last_good_target = None
        self.last_good_curvature = None
        self.last_good_time = None
        self.nominal_lookahead = self.lookahead

        self.freeze_timeout = 3.0  # segundos

        # ================= ESTADO LANE MASK =================
        self.lane_mask_memory = []   # [x, y, t]

        self.lane_width_est = 3.0    # ancho estimado del carril del parqueadero (m)
        self.lane_width_alpha = 0.15 # EMA suave


        # ==================== SUB / PUB ====================
        self.create_subscription(PointCloud2, '/lane/edge_points', self.cloud_callback, 10)#/lane/edge_points
        self.create_subscription(PointCloud2,'/lane/mask_points',self.lane_mask_callback,10)#/lane/mask_points


        self.pub_cloud = self.create_publisher(Marker, '/lane/memory_cloud', 10)
        self.pub_center = self.create_publisher(Marker, '/lane/centerline', 10)
        self.pub_prediction = self.create_publisher(Marker, '/lane/prediction', 10)
        self.pub_target = self.create_publisher(Marker, '/lane/target_point', 10)
        self.pub_curvature = self.create_publisher(Float32,'/lane/curvature',10)

        # ==================== CONTROL ====================
        self.pub_e_lat = self.create_publisher(Float32, '/lane/error_lateral', 10)
        self.pub_e_head = self.create_publisher(Float32, '/lane/error_heading', 10)
        
        self.pub_target_vis = self.create_publisher(
            Marker, '/lane/target_vis', 10)

        self.pub_lateral_error_vis = self.create_publisher(
            Marker, '/lane/lateral_error_vis', 10)

        
        self.robot_pose = None

        self.create_subscription(Odometry,'/odometry/local',self.odom_callback,10)

        self.create_subscription(Imu, '/imu/unfiltered', self.imu_callback, 10)

        self.last_pitch = None
        self.pitch_event = False
        self.last_good_target = None

        self.create_timer(0.1, self.timer_callback)

        #self.get_logger().info('LaneMemoryNode v4 iniciado (predicción + control ref)')


    # ================= IMU (PITCH) =====================
    def imu_callback(self, msg):
        self.pitch_event = False

        q = msg.orientation
        _, pitch, _ = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])

        if self.last_pitch is not None:
            dp = pitch - self.last_pitch
            if abs(dp) > np.deg2rad(4):  # umbral típico
                self.pitch_event = True
        else:
            self.pitch_event = False

        self.last_pitch = pitch

        if self.pitch_event:
            self.lookahead = 0.8
            self.freeze_timeout = 2.0
        else:
            self.lookahead = self.nominal_lookahead
            self.freeze_timeout = 1.0

    # =====================================================
    def cloud_callback(self, msg: PointCloud2):
        now = self.get_clock().now().nanoseconds * 1e-9
        for p in point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True):
            self.lane_memory.append([p[0], p[1], now])
    
    def lane_mask_callback(self, msg: PointCloud2):
        now = self.get_clock().now().nanoseconds * 1e-9
        for p in point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True):
            self.lane_mask_memory.append([p[0], p[1], now])


    # ================= Odometry ========================
    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        _, _, yaw = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])

        self.robot_pose = {
            'x': p.x,
            'y': p.y,
            'yaw': yaw
        }
        #self.get_logger().info(f"Robot pose updated: x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.2f} rad")


    # =====================================================
    def timer_callback(self):
        if len(self.lane_memory) < 10:
            return

        self.forget_old()
        self.forget_old_mask()

        self.edge_centerline = self.process_pointcloud(self.lane_memory)
        self.mask_centerline = self.process_pointcloud(self.lane_mask_memory)

        if self.edge_centerline is not None and self.mask_centerline is not None:
            dists = []
            for pm in self.mask_centerline:
                d = np.min(np.linalg.norm(self.edge_centerline - pm, axis=1))
                dists.append(d)

            if len(dists) > 3:
                width_inst = 2.0 * np.mean(dists)
                self.lane_width_est = (
                    self.lane_width_alpha * width_inst +
                    (1 - self.lane_width_alpha) * self.lane_width_est
                )



        if self.mask_centerline is not None:
            self.centerline = self.mask_centerline
        else:
            self.centerline = self.edge_centerline


        use_freeze = False
        prediction = None
        target = None
        curvature = 0.0

        if self.centerline is None:
            use_freeze = True
        elif len(self.centerline) < 5:
            use_freeze = True

        if not use_freeze:
            # Obtener predicción y target
            prediction, target, curvature = self.predict_lane(self.centerline)
            if target is None:
                if self.last_good_target is None:
                    return  # no hay nada seguro aún
                target = self.last_good_target
            else:
                self.last_good_target = target.copy()

            # ===== guardar último válido =====
            self.last_good_centerline = self.centerline.copy()
            self.last_good_prediction = prediction.copy()
            self.last_good_target = target.copy()
            self.last_good_curvature = curvature
            self.last_good_time = time.time()

        else:
            if self.last_good_time is None:
                return

            age = time.time() - self.last_good_time
            if age > self.freeze_timeout:
                return  # ya no es confiable

            prediction = self.last_good_prediction
            target = self.last_good_target
            curvature = self.last_good_curvature

        # ======== SUAVIZADO FINAL (SOLO TARGET) ========
        if prediction is not None and target is not None and self.robot_pose is not None:
            self.target_history.append(target)

            if len(self.target_history) > 1:
                target_smoothed = np.mean(
                    np.array(self.target_history), axis=0
                )
            else:
                target_smoothed = target

            # 👉 OJO: usamos prediction DIRECTA (NO suavizada)
            e_lat, e_heading, closest = self.compute_errors(
                prediction, target_smoothed
            )

            """dx = target_smoothed[0] - self.robot_pose['x']
            self.get_logger().info(
                f"Target: {dx:+.2f}m adelante | "
                f"Error lateral: {e_lat:+.2f}m | "
                f"Puntos usados: {len(self.centerline) if self.centerline is not None else 0}"
            )"""
        else:
            e_lat, e_heading, closest = 0.0, 0.0, None

        # ======== Publicar errores CALCULADOS RESPECTO AL TARGET ========
        self.publish_errors(e_lat, e_heading, closest)
        
        # Publicar visualizaciones
        if self.centerline is not None:
            self.publish_cloud(self.centerline)

        if self.centerline is not None:
            self.publish_centerline(self.centerline)

        self.publish_prediction(prediction)
        self.publish_target(target)
        self.publish_curvature(curvature)
        if target is not None and self.robot_pose is not None:
            self.publish_target_marker(target)

        if closest is not None and self.robot_pose is not None:
            self.publish_lateral_error_marker(closest)


    # =====================================================
    def process_pointcloud(self, memory):
        """
        Pipeline común:
        memory -> voxel -> centerline
        """
        if len(memory) < 10:
            return None

        # voxel
        voxels = {}
        for x, y, _ in memory:
            if np.isnan(x) or np.isnan(y):
                continue
            key = (int(x / self.voxel_size), int(y / self.voxel_size))
            voxels.setdefault(key, []).append((x, y))

        cloud = []
        for v in voxels.values():
            if len(v) > 0:
                cloud.append(np.mean(np.array(v), axis=0))

        if len(cloud) < 5:
            return None

        cloud = np.array(cloud)
        cloud = cloud[cloud[:, 0].argsort()]

        # slicing
        centers = []
        x_min, x_max = cloud[0, 0], cloud[-1, 0]
        x = x_min

        while x < x_max:
            pts = cloud[(cloud[:, 0] >= x) & (cloud[:, 0] < x + self.slice_length)]
            if len(pts) > 2:
                centers.append([pts[:, 0].mean(), pts[:, 1].mean()])
            x += self.slice_length

        if len(centers) < 5:
            return None

        return np.array(centers)



    # ================== MEMORIA ==========================
    def forget_old(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.lane_memory = [p for p in self.lane_memory if now - p[2] < self.max_age]
    
    def forget_old_mask(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.lane_mask_memory = [
            p for p in self.lane_mask_memory if now - p[2] < self.max_age
        ]


    # ================== DENSIDAD =========================
    def voxel_filter(self):
        voxels = {}
        for x, y, _ in self.lane_memory:
            # Verifica que x,y sean números válidos
            if np.isnan(x) or np.isnan(y):
                continue
                
            key = (int(x / self.voxel_size), int(y / self.voxel_size))
            voxels.setdefault(key, []).append((float(x), float(y)))
        
        # Filtra voxels que tengan al menos un punto
        filtered_points = []
        for v in voxels.values():
            if len(v) > 0:
                # Calcula el promedio de forma segura
                points_array = np.array(v)
                mean_point = np.mean(points_array, axis=0)
                filtered_points.append(mean_point)
        
        if len(filtered_points) > 0:
            return np.array(filtered_points)
        else:
            return np.empty((0, 2))  # Array vacío de forma segura

    # ================== CENTERLINE =======================
    def extract_centerline(self, cloud):
        cloud = cloud[cloud[:, 0].argsort()]
        x_min, x_max = cloud[0, 0], cloud[-1, 0]

        centers = []
        x = x_min
        while x < x_max:
            pts = cloud[(cloud[:, 0] >= x) & (cloud[:, 0] < x + self.slice_length)]
            if len(pts) > 2:
                centers.append([pts[:, 0].mean(), pts[:, 1].mean()])
            x += self.slice_length

        return np.array(centers)

    # ================== PREDICCIÓN =======================
    def predict_lane(self, centerline):
        """
        Genera:
        - predicción del carril desplazado
        - target a lookahead REAL delante del robot
        - curvatura local
        TODO en frame odom
        """
        if len(centerline) < 5 or self.robot_pose is None:
            return np.empty((0, 2)), None, 0.0

        rx = self.robot_pose['x']
        ry = self.robot_pose['y']
        yaw = self.robot_pose['yaw']

        robot_pos = np.array([rx, ry])
        forward = np.array([np.cos(yaw), np.sin(yaw)])

        # ================= 1. AJUSTE POLINÓMICO =================
        x = centerline[:, 0]
        y = centerline[:, 1]

        if np.std(x) < 0.2:
            return np.empty((0,2)), None, 0.0

        coeffs = np.polyfit(x, y, 2)
        poly = np.poly1d(coeffs)

        # ================= 2. MUESTREO SOBRE LA CURVA =================
        xs = np.linspace(x.min(), x.max(), 200)
        ys = poly(xs)
        curve = np.column_stack((xs, ys))

        # ================= 3. FILTRAR SOLO PUNTOS DELANTE =================
        vecs = curve - robot_pos
        ahead_mask = np.dot(vecs, forward) > 0.2  # 20 cm mínimo delante
        curve_ahead = curve[ahead_mask]

        if len(curve_ahead) < 5:
            return np.empty((0, 2)), None, 0.0

        # ================= 4. BUSCAR LOOKAHEAD REAL =================
        dists = np.linalg.norm(np.diff(curve_ahead, axis=0), axis=1)
        s = np.insert(np.cumsum(dists), 0, 0.0)

        idx = np.argmin(np.abs(s - self.lookahead))
        base_pt = curve_ahead[idx]

        # ================= 5. NORMAL AL CARRIL =================
        dy_dx = 2 * coeffs[0] * base_pt[0] + coeffs[1]

        if not np.isfinite(dy_dx):
            return np.empty((0,2)), None, 0.0

        normal = np.array([-dy_dx, 1.0])
        normal /= np.linalg.norm(normal)

        # ================= 6. TARGET FINAL =================
        offset = (
            self.lane_half_width
            if self.mask_centerline is not None
            else self.lane_width_est / 2.0
        )

        target = base_pt + normal * offset

        # ================= 7. PREDICCIÓN =================
        pred_base = curve_ahead[:idx+10]
        prediction = pred_base + normal * offset


        # ================= 8. CURVATURA ANTICIPADA =================
        # Curvatura actual (en base_pt)
        x_now = base_pt[0]
        dy_dx_now = 2 * coeffs[0] * x_now + coeffs[1]
        k_now = abs(2 * coeffs[0]) / (1 + dy_dx_now**2)**1.5

        preview_distance = self.get_parameter('max_prediction_curvature').value # metros adelante (ajustable)

        idx_preview = np.argmin(np.abs(s - (self.lookahead + preview_distance)))
        idx_preview = min(idx_preview, len(curve_ahead) - 1)

        preview_pt = curve_ahead[idx_preview]

        x_future = preview_pt[0]
        dy_dx_future = 2 * coeffs[0] * x_future + coeffs[1]
        k_future = abs(2 * coeffs[0]) / (1 + dy_dx_future**2)**1.5


        curvature = 0.4 * k_now + 0.6 * k_future

        return prediction, target, curvature



    
    def publish_curvature(self, curvature):
        msg = Float32()
        msg.data = float(curvature)
        self.pub_curvature.publish(msg)

    # ==================== CENTRO DEL CARRIL =====================
    def generate_offset_centerline(self, poly, x_vals, offset):
        """
        Genera una línea paralela desplazada 'offset' metros
        usando la normal de la curva.
        """
        center = []

        for x in x_vals:
            y = poly(x)

            # Derivada dy/dx
            dy_dx = np.polyder(poly)(x)

            # Vector normal unitario
            norm = np.array([-dy_dx, 1.0])
            norm = norm / np.linalg.norm(norm)

            # Offset hacia el centro del carril
            x_c = x + norm[0] * offset
            y_c = y + norm[1] * offset

            center.append([x_c, y_c])

        return np.array(center)


    # ================ CALCULO ERRORES ==================
    def compute_errors(self, prediction, target_point):
        """
        Calcula errores respecto al SEGMENTO que contiene el TARGET.
        """
        if len(prediction) < 2 or self.robot_pose is None or target_point is None:
            return 0.0, 0.0, None

        # ======== CAMBIO CRÍTICO: Encontrar segmento del TARGET ========
        # Convertir a arrays numpy para operaciones vectoriales
        pred_array = np.array(prediction)
        target_array = np.array(target_point)
        
        # Encontrar el índice del punto más cercano al target en la predicción
        distances = np.linalg.norm(pred_array - target_array, axis=1)
        closest_idx = np.argmin(distances)
        
        # Obtener el segmento que contiene/está cerca del target
        if closest_idx == 0:
            # Si el target está en el primer punto, usar primeros dos puntos
            P0 = pred_array[0]
            P1 = pred_array[1]
        elif closest_idx == len(pred_array) - 1:
            # Si está en el último, usar últimos dos puntos
            P0 = pred_array[-2]
            P1 = pred_array[-1]
        else:
            # Usar el punto más cercano y el siguiente
            P0 = pred_array[closest_idx]
            P1 = pred_array[closest_idx + 1]
        
        # ======== Cálculo de errores (igual fórmula, pero con P0-P1 del target) ========
        robot_pos = np.array([
            self.robot_pose['x'],
            self.robot_pose['y']
        ])

        v = P1 - P0  # Vector del segmento del target
        w = robot_pos - P0  # Vector del robot al inicio del segmento

        v_norm = np.linalg.norm(v)
        if v_norm < 1e-3:
            return 0.0, 0.0, None

        # Error lateral con signo (distancia perpendicular al segmento del target)
        e_lat = (-v[0]*w[1] + v[1]*w[0]) / v_norm

        # ===== HEADING CORRECTO: robot -> target =====
        dx = target_point[0] - self.robot_pose['x']
        dy = target_point[1] - self.robot_pose['y']

        target_heading = np.arctan2(dy, dx)

        e_heading = self.normalize_angle(
            target_heading - self.robot_pose['yaw']
        )


        # Punto más cercano en el segmento (para visualización)
        t = np.dot(w, v) / np.dot(v, v)
        t_clamped = np.clip(t, 0.0, 1.0)  # Asegurar que está en el segmento
        closest = P0 + t_clamped * v

        return e_lat, e_heading, closest

    def normalize_angle(self, a):
        return np.arctan2(np.sin(a), np.cos(a))



    def publish_errors(self, e_lat, e_heading, closest):
        msg = Float32()
        
        msg.data = float(e_lat)
        self.pub_e_lat.publish(msg)

        msg.data = float(e_heading)
        self.pub_e_head.publish(msg)


    # ================== RVIZ =============================
    def publish_cloud(self, cloud):
        self._publish_points(cloud, '/lane/memory_cloud', 0, (1.0, 0.5, 0.0))

    def publish_centerline(self, centerline):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'centerline'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.06
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.1, 1.0, 0.1, 1.0)
        for x, y in centerline:
            marker.points.append(Point(x=float(x), y=float(y), z=0.0))
        self.pub_center.publish(marker)

    def publish_prediction(self, prediction):
        if prediction is None or len(prediction) == 0:
            return

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'prediction'
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.05
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 0.7, 1.0, 1.0)
        for x, y in prediction:
            marker.points.append(Point(x=float(x), y=float(y), z=0.0))
        self.pub_prediction.publish(marker)

    def publish_target(self, target):
        if target is None:
            return

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target'
        marker.id = 3
        marker.type = Marker.SPHERE
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.0, 0.0, 1.0)
        marker.pose.position.x = float(target[0])
        marker.pose.position.y = float(target[1])
        self.pub_target.publish(marker)

    def _publish_points(self, cloud, ns, mid, color):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = mid
        marker.type = Marker.POINTS
        marker.scale.x = marker.scale.y = 0.07
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 1.0)
        for x, y in cloud:
            marker.points.append(Point(x=float(x), y=float(y), z=0.0))
        self.pub_cloud.publish(marker)

    def publish_target_marker(self, target):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "lane_target"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD

        m.scale.x = 0.05
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0

        p0 = Point()
        p0.x = self.robot_pose['x']
        p0.y = self.robot_pose['y']
        p0.z = 0.1

        p1 = Point()
        p1.x = target[0]
        p1.y = target[1]
        p1.z = 0.1

        m.points = [p0, p1]
        self.pub_target_vis.publish(m)
    
    def publish_lateral_error_marker(self, proj_pt):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "lane_lateral_error"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD

        m.scale.x = 0.03
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0

        p0 = Point()
        p0.x = self.robot_pose['x']
        p0.y = self.robot_pose['y']
        p0.z = 0.05

        p1 = Point()
        p1.x = proj_pt[0]
        p1.y = proj_pt[1]
        p1.z = 0.05

        m.points = [p0, p1]
        self.pub_lateral_error_vis.publish(m)



    def destroy_node(self):
        super().destroy_node()

# =========================================================

def main(args=None):
    rclpy.init(args=args)
    node = LaneMemoryNode()
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