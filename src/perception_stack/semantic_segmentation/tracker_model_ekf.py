#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.linalg import block_diag

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class MultiLaneEKFTracker(Node):
    def __init__(self):
        super().__init__('multi_lane_ekf_tracker')
        
        # === SUBS/PUBS ===
        #self.create_subscription(Path, "/lane/center_path", self.ego_callback, 1)
        #self.create_subscription(Path, "/lane/left_border", self.left_callback, 1)
        self.create_subscription(Path, "/lane/right_border", self.right_callback, 1)
        
        #self.ego_tracked_pub = self.create_publisher(Path, "/lane/ego_tracked", 1)
        #self.left_tracked_pub = self.create_publisher(Path, "/lane/left_tracked", 1)
        self.right_tracked_pub = self.create_publisher(Path, "/lane/right_tracked", 1)
        
        # === PARÁMETROS EKF ===
        self.declare_parameter("process_noise", 0.01)
        self.declare_parameter("measurement_noise", 0.1)
        self.declare_parameter("max_lost_time", 2.0)
        self.declare_parameter("poly_degree", 2)
        self.declare_parameter("min_points", 5)  # Mismo que el central
        
        self.Q_scale = self.get_parameter("process_noise").value
        self.R_scale = self.get_parameter("measurement_noise").value
        self.max_lost_time = self.get_parameter("max_lost_time").value
        self.poly_degree = self.get_parameter("poly_degree").value
        self.min_points = self.get_parameter("min_points").value
        
        # === 3 EKFs IDÉNTICOS al central ===
        self.ekfs = {
            'ego': self.create_ekf(),
            'left': self.create_ekf(),
            'right': self.create_ekf()
        }
        
        # === ESTADO DE TRACKING ===
        self.tracking_states = {
            'ego': {
                'is_tracking': False, 
                'last_update': None,
                'last_measurement': None,
                'consecutive_failures': 0,
                'track_id': 0
            },
            'left': {
                'is_tracking': False, 
                'last_update': None,
                'last_measurement': None,
                'consecutive_failures': 0,
                'track_id': 0
            },
            'right': {
                'is_tracking': False, 
                'last_update': None,
                'last_measurement': None,
                'consecutive_failures': 0,
                'track_id': 0
            }
        }
        
        #self.get_logger().info(f" Multi-Lane EKF Tracker (igual al central)")

    def create_ekf(self):
        """Crear EKF IDÉNTICO al central"""
        self.state_dim = 2 * (self.poly_degree + 1)
        return {
            'x': np.zeros((self.state_dim, 1)),
            'P': np.eye(self.state_dim) * 10.0,
            'Q': self.build_process_noise(),
            'R': self.build_measurement_noise()
        }

    def build_process_noise(self):
        """MATRIZ IDÉNTICA al central"""
        Q_coeff = np.eye(self.poly_degree + 1) * self.Q_scale
        Q_deriv = np.eye(self.poly_degree + 1) * (self.Q_scale * 10.0)  # 10.0 como el central
        return block_diag(Q_coeff, Q_deriv)

    def build_measurement_noise(self):
        """MATRIZ IDÉNTICA al central"""
        return np.eye(self.poly_degree + 1) * self.R_scale

    # ========== CALLBACKS ==========
    """def ego_callback(self, msg: Path):
        self.process_lane(msg, 'ego', self.ego_tracked_pub)

    def left_callback(self, msg: Path):
        self.process_lane(msg, 'left', self.left_tracked_pub)"""

    def right_callback(self, msg: Path):
        self.process_lane(msg, 'right', self.right_tracked_pub)

    def process_lane(self, msg: Path, lane_type: str, publisher):
        """PROCESO IDÉNTICO al central"""
        current_time = self.get_clock().now()
        state = self.tracking_states[lane_type]
        
        # 1. Verificar puntos mínimos
        if len(msg.poses) < self.min_points:
            #self.get_logger().warn(f" {lane_type}: Solo {len(msg.poses)} puntos")
            self.handle_tracking_loss(lane_type, current_time)
            return
        
        # 2. Extraer y validar puntos (IGUAL al central)
        xs, ys, valid = self.extract_and_validate_points(msg.poses, lane_type)
        if not valid:
            self.handle_tracking_loss(lane_type, current_time)
            return
        
        # 3. Ajustar polinomio con ponderación (IGUAL al central)
        z, success = self.fit_polynomial(xs, ys)
        if not success:
            self.handle_tracking_loss(lane_type, current_time)
            return
        
        # 4. Calcular dt
        dt = self.calculate_dt(state['last_update'], current_time)
        
        # 5. Ejecutar EKF
        ekf = self.ekfs[lane_type]
        
        if state['is_tracking']:
            self.predict(ekf, dt, lane_type)
            self.update(ekf, z, lane_type)
            state['consecutive_failures'] = 0
        else:
            self.initialize_ekf(ekf, z, lane_type)
            state['is_tracking'] = True
            state['track_id'] += 1
            #self.get_logger().info(f" Track {lane_type} #{state['track_id']} iniciado")
        
        # 6. Actualizar tiempos
        state['last_update'] = current_time
        state['last_measurement'] = current_time
        
        # 7. Reconstruir path
        tracked_path = self.reconstruct_path(ekf, xs, msg.header, lane_type)
        
        # 8. Publicar
        publisher.publish(tracked_path)

    # ========== FUNCIONES IDÉNTICAS al central ==========
    
    def extract_and_validate_points(self, poses, lane_type):
        """IGUAL al central con filtrado por distancia"""
        xs, ys = [], []
        
        for pose in poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            
            # Mismas validaciones que el central
            if not (np.isfinite(x) and np.isfinite(y)):
                continue
            
            # MISMO filtro de distancia
            if x < -5.0 or x > 50.0:
                continue
            
            # Validación adicional por tipo de línea
            if lane_type == 'ego' and y > 0.5:  # Ego debe estar a la derecha
                continue
            elif lane_type == 'right' and y > -1.0:  # Right debe ser más negativo
                continue
            elif lane_type == 'left' and y < 1.0:  # Left debe ser positivo
                continue
            
            xs.append(x)
            ys.append(y)
        
        if len(xs) < self.min_points:
            return [], [], False
        
        return np.array(xs), np.array(ys), True

    def fit_polynomial(self, xs, ys):
        """IGUAL al central con ponderación"""
        try:
            # MISMA ponderación
            weights = 1.0 / (xs + 1.0)
            coeffs = np.polyfit(xs, ys, self.poly_degree, w=weights)
            
            if not all(np.isfinite(coeffs)):
                return None, False
            
            return coeffs.reshape((-1, 1)), True
            
        except Exception as e:
            #self.get_logger().warn(f"Error en ajuste polinómico: {e}")
            return None, False

    def calculate_dt(self, last_time, current_time):
        """IGUAL al central"""
        if last_time is None:
            return 0.1
        
        dt_nanos = current_time.nanoseconds - last_time.nanoseconds
        dt = dt_nanos / 1e9
        
        # MISMO límite
        return min(max(dt, 0.01), 0.5)

    def initialize_ekf(self, ekf, z, lane_type):
        """IGUAL al central"""
        ekf['x'][:self.poly_degree + 1] = z
        ekf['x'][self.poly_degree + 1:] = 0.0
        ekf['P'] = np.eye(self.state_dim) * 0.1

    def predict(self, ekf, dt, lane_type):
        """IGUAL al central"""
        F = np.eye(self.state_dim)
        
        for i in range(self.poly_degree + 1):
            F[i, self.poly_degree + 1 + i] = dt
        
        ekf['x'] = F @ ekf['x']
        ekf['P'] = F @ ekf['P'] @ F.T + ekf['Q']
        
        # MISMO decaimiento
        decay = 0.95
        ekf['x'][self.poly_degree + 1:] *= decay
        ekf['P'][self.poly_degree + 1:, self.poly_degree + 1:] *= decay**2

    def update(self, ekf, z, lane_type):
        """IGUAL al central"""
        H = np.zeros((self.poly_degree + 1, self.state_dim))
        H[:, :self.poly_degree + 1] = np.eye(self.poly_degree + 1)
        
        y = z - H @ ekf['x']
        S = H @ ekf['P'] @ H.T + ekf['R']
        
        try:
            K = ekf['P'] @ H.T @ np.linalg.inv(S)
        except:
            #self.get_logger().warn(" Error en inversión de S")
            return
        
        ekf['x'] = ekf['x'] + K @ y
        ekf['P'] = (np.eye(self.state_dim) - K @ H) @ ekf['P']
        
        # MISMO límite de curvatura
        max_a = 0.5
        if self.poly_degree >= 2:
            ekf['x'][0] = np.clip(ekf['x'][0], -max_a, max_a)

    def handle_tracking_loss(self, lane_type, current_time):
        """IGUAL al central"""
        state = self.tracking_states[lane_type]
        state['consecutive_failures'] += 1
        
        if state['last_measurement'] is not None:
            lost_time = (current_time.nanoseconds - state['last_measurement'].nanoseconds) / 1e9
            
            if lost_time > self.max_lost_time:
                state['is_tracking'] = False
                #self.get_logger().warn(f" Track {lane_type} perdido ({lost_time:.1f}s)")

    def reconstruct_path(self, ekf, xs, header, lane_type):
        """IGUAL al central"""
        path = Path()
        path.header = header
        
        if not self.tracking_states[lane_type]['is_tracking']:
            return path
        
        xs_sorted = np.sort(xs)
        
        # Para grado 2 (igual que el central)
        if self.poly_degree == 2:
            a, b, c = ekf['x'][:3].flatten()
            
            for x in xs_sorted:
                y = a * x**2 + b * x + c
                
                pose = PoseStamped()
                pose.header = header
                pose.pose.position.x = x
                pose.pose.position.y = float(y)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        
        return path

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiLaneEKFTracker()
    
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