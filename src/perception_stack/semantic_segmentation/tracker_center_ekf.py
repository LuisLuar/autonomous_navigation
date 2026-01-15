#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.linalg import block_diag


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class RobustLaneEKF(Node):
    def __init__(self):
        super().__init__('lane_ekf_tracker')
        
        # === SUBS/PUBS ===
        self.create_subscription(
            Path,
            "/lane/offset_path",
            self.path_cb,
            1
        )
        
        self.tracked_pub = self.create_publisher(
            Path,
            "/lane/tracked_path",
            1
        )
        
        # === PARÁMETROS EKF ===
        self.declare_parameter("process_noise", 0.01)
        self.declare_parameter("measurement_noise", 0.1)
        self.declare_parameter("max_lost_time", 2.0)      # segundos
        self.declare_parameter("poly_degree", 2)          # grado polinomio
        self.declare_parameter("min_points", 5)           # mínimo puntos
        
        self.Q_scale = self.get_parameter("process_noise").value
        self.R_scale = self.get_parameter("measurement_noise").value
        self.max_lost_time = self.get_parameter("max_lost_time").value
        self.poly_degree = self.get_parameter("poly_degree").value
        self.min_points = self.get_parameter("min_points").value
        
        # === ESTADO EKF (6D: [a, b, c, a_dot, b_dot, c_dot]) ===
        self.state_dim = 2 * (self.poly_degree + 1)  # Coefs + derivadas
        self.x = np.zeros((self.state_dim, 1))      # Estado
        self.P = np.eye(self.state_dim) * 10.0      # Covarianza inicial alta
        
        # Matrices de ruido
        self.Q = self.build_process_noise()
        self.R = self.build_measurement_noise()
        
        # === CONTROL TEMPORAL ===
        self.last_update_time = None
        self.last_measurement_time = None
        self.is_tracking = False
        self.track_id = 0
        self.consecutive_failures = 0
        
        
        #self.get_logger().info(f"Lane EKF Tracker iniciado (grado {self.poly_degree})")

    def build_process_noise(self):
        """Matriz de ruido del proceso"""
        # Ruido pequeño en coeficientes, mayor en derivadas
        Q_coeff = np.eye(self.poly_degree + 1) * self.Q_scale
        Q_deriv = np.eye(self.poly_degree + 1) * (self.Q_scale * 10.0)
        return block_diag(Q_coeff, Q_deriv)

    def build_measurement_noise(self):
        """Matriz de ruido de medición"""
        # Solo medimos coeficientes, no derivadas
        R = np.eye(self.poly_degree + 1) * self.R_scale
        return R

    def path_cb(self, msg: Path):
        current_time = self.get_clock().now()
        
        # 1. Verificar si tenemos suficientes puntos
        if len(msg.poses) < self.min_points:
            #self.get_logger().warn(f"Solo {len(msg.poses)} puntos (mínimo {self.min_points})")
            self.handle_tracking_loss(current_time)
            return
        
        # 2. Extraer y preprocesar puntos
        xs, ys, valid = self.extract_and_validate_points(msg.poses)
        if not valid:
            self.handle_tracking_loss(current_time)
            return
        
        # 3. Ajustar polinomio (medición)
        z, success = self.fit_polynomial(xs, ys)
        if not success:
            self.handle_tracking_loss(current_time)
            return
        
        # 4. Calcular dt desde última actualización
        dt = self.calculate_dt(current_time)
        
        # 5. Ejecutar EKF
        if self.is_tracking:
            self.predict(dt)
            self.update(z)
            self.consecutive_failures = 0
        else:
            # Inicialización
            self.initialize_state(z)
            self.is_tracking = True
            self.track_id += 1
            #self.get_logger().info(f"Track {self.track_id} iniciado")
        
        self.last_update_time = current_time
        self.last_measurement_time = current_time
        
        # 6. Reconstruir y publicar path
        tracked_path = self.reconstruct_path(xs, msg.header)
        self.tracked_pub.publish(tracked_path)

    def extract_and_validate_points(self, poses):
        """Extraer y validar puntos del path"""
        xs, ys = [], []
        
        for pose in poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            
            # Filtrar valores inválidos
            if not (np.isfinite(x) and np.isfinite(y)):
                continue
            
            # Filtrar puntos muy atrás o muy adelante
            if x < -5.0 or x > 50.0:
                continue
            
            xs.append(x)
            ys.append(y)
        
        if len(xs) < self.min_points:
            return [], [], False
        
        return np.array(xs), np.array(ys), True

    def fit_polynomial(self, xs, ys):
        """Ajustar polinomio a los puntos"""
        try:
            # Ponderar por distancia (más peso a puntos cercanos)
            weights = 1.0 / (xs + 1.0)  # Más peso a x pequeños (cerca)
            
            # Ajuste ponderado
            coeffs = np.polyfit(xs, ys, self.poly_degree, w=weights)
            
            # Validar coeficientes
            if not all(np.isfinite(coeffs)):
                return None, False
            
            return coeffs.reshape((-1, 1)), True
            
        except Exception as e:
            #self.get_logger().warn(f" Error en ajuste polinómico: {e}")
            return None, False

    def calculate_dt(self, current_time):
        """Calcular delta tiempo desde última actualización"""
        if self.last_update_time is None:
            return 0.1  # Valor por defecto
        
        dt_nanos = current_time.nanoseconds - self.last_update_time.nanoseconds
        dt = dt_nanos / 1e9  # Convertir a segundos
        
        # Limitar dt razonable
        return min(max(dt, 0.01), 0.5)  # Entre 10ms y 500ms

    def initialize_state(self, z):
        """Inicializar estado EKF"""
        # Coeficientes de la medición
        self.x[:self.poly_degree + 1] = z
        
        # Derivadas iniciales en cero
        self.x[self.poly_degree + 1:] = 0.0
        
        # Covarianza inicial
        self.P = np.eye(self.state_dim) * 0.1
        
        self.is_tracking = True

        self.last_measurement_time = self.get_clock().now()


    def predict(self, dt):
        """Predicción EKF"""
        # Matriz de transición F
        F = np.eye(self.state_dim)
        
        # Actualizar posición con derivadas: x = x + x_dot * dt
        for i in range(self.poly_degree + 1):
            F[i, self.poly_degree + 1 + i] = dt
        
        # Predicción
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        
        # Suavizar derivadas (modelo de decaimiento)
        decay = 0.95
        self.x[self.poly_degree + 1:] *= decay
        self.P[self.poly_degree + 1:, self.poly_degree + 1:] *= decay**2

    def update(self, z):
        """Actualización EKF"""
        # Jacobiano H (medimos solo coeficientes)
        H = np.zeros((self.poly_degree + 1, self.state_dim))
        H[:, :self.poly_degree + 1] = np.eye(self.poly_degree + 1)
        
        # Innovación
        y = z - H @ self.x
        
        # Covarianza de innovación
        S = H @ self.P @ H.T + self.R
        
        # Ganancia de Kalman
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except:
            #self.get_logger().warn(" Error en inversión de S")
            return
        
        # Actualización
        self.x = self.x + K @ y
        self.P = (np.eye(self.state_dim) - K @ H) @ self.P

        # Limitar curvatura (seguridad física)
        max_a = 0.5   # m⁻¹ aprox (curva fuerte urbana)
        self.x[0] = np.clip(self.x[0], -max_a, max_a)


    def handle_tracking_loss(self, current_time):
        """Manejar pérdida de track"""
        self.consecutive_failures += 1
        
        # Verificar si hemos perdido el track
        if self.last_measurement_time is not None:
            lost_time = (current_time.nanoseconds - self.last_measurement_time.nanoseconds) / 1e9
            
            if lost_time > self.max_lost_time:
                self.is_tracking = False
                #self.get_logger().warn(f" Track perdido después de {lost_time:.1f}s")


    def reconstruct_path(self, xs, header):
        """Reconstruir path desde estado filtrado"""
        path = Path()
        path.header = header
        
        # Ordenar xs
        xs_sorted = np.sort(xs)
        
        # Evaluar polinomio filtrado
        a, b, c = self.x[:3].flatten()  # Para grado 2
        
        for x in xs_sorted:
            y = a * x**2 + b * x + c
            
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = x
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        return path

    def create_debug_path(self, xs, header):
        """Crear path de debug (polinomio puro)"""
        path = Path()
        path.header = header
        
        if not self.is_tracking:
            return path
        
        xs_range = np.linspace(min(xs), max(xs), 30)
        a, b, c = self.x[:3].flatten()
        
        for x in xs_range:
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
    node = RobustLaneEKF()
    
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