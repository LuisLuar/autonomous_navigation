#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import casadi as ca
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from custom_interfaces.msg import LaneModel

from diagnostic_msgs.msg import DiagnosticStatus


class LaneControllerNMPC(Node):

    def __init__(self):
        super().__init__('lane_controller_nmpc')

        # Declarar y obtener parámetros del YAML
        # Si no existen en el YAML, tomarán el valor por defecto (segundo argumento)
        self.N = self.declare_parameter('N', 10).value
        self.dt = self.declare_parameter('dt', 0.1).value
        self.v_ref = self.declare_parameter('v_ref', 1.5).value
        self.k_curve = self.declare_parameter('k_curve', 2.5).value

        self.Q_ey = self.declare_parameter('Q_ey', 10.0).value
        self.Q_epsi = self.declare_parameter('Q_epsi', 12.5).value
        self.R_accel = self.declare_parameter('R_accel', 1.0).value
        self.R_omega = self.declare_parameter('R_omega', 2.5).value
        self.S_accel = self.declare_parameter('S_accel', 2.0).value
        self.S_omega = self.declare_parameter('S_omega', 3.0).value
        self.W_v_follow = self.declare_parameter('W_v_follow', 5.0).value
        self.Q_terminal = self.declare_parameter('Q_terminal', 15.0).value

        # Límites físicos 
        self.declare_parameter('max_lin_accel', 0.05)
        self.declare_parameter('max_lin_decel', 0.5)
        self.declare_parameter('v_max', 1.7)
        self.declare_parameter('w_max', 0.6)
        self.declare_parameter('min_v_start', 0.1)
        self.declare_parameter('max_v_start', 0.2)

        # Subs/Pubs
        self.create_subscription(LaneModel, '/lane/model_filtered', self.lane_cb, 10)
        self.create_subscription(Odometry, '/odometry/local', self.odom_cb, 10)
        self.create_subscription(Bool, '/emergency', self.emergency_cb, 10)
        self.create_subscription(Bool, '/safe_stop', self.safe_stop_cb, 10)
        self.create_subscription(Float32, '/alpha/vision', self.vision_alpha_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/active/vision', self.active_vision_cb, 10)
        self.create_subscription(Bool, '/manual', self.manual_mode_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_manual', self.manual_twist_cb, 10)
        # 2. Suscripción al estado global
        self.create_subscription(
            DiagnosticStatus, 
            '/global_status', 
            self.global_status_cb, 
            10)

        self.is_manual = False
        self.manual_twist = Twist()
        
        # Variable para almacenar el estado (0=OK, 1=WARN, 2=ERROR según estándar)
        self.global_level = 0

        self.alpha_vision = 1.0
        self.active_vision = False

        # Estado [ey, epsi, v]
        self.state = np.zeros(3)
        self.kappa = 0.0
        self.emergency = False
        self.safe_stop = False

        # Warm start memory
        self.u_prev = np.zeros(2)

        self.setup_nmpc()
        self.create_timer(0.033, self.control_loop)
    
    # ============ CONTROL DE VELOCIDAD LINEAL POR DETECCION DE OBJETOS =====================
    def vision_alpha_cb(self, msg):
        self.alpha_vision = msg.data

    # ============================================================
    # SETUP NMPC
    # ============================================================

    def setup_nmpc(self):
        nx = 3
        nu = 2

        x = ca.SX.sym('x', nx)
        u = ca.SX.sym('u', nu)
        kappa_p = ca.SX.sym('kappa')

        ey_dot = x[2] * ca.sin(x[1])
        denom = 1 - kappa_p * x[0]
        denom_safe = denom + 1e-6
        epsi_dot = u[1] - (x[2] * kappa_p * ca.cos(x[1])) / denom_safe
        v_dot = u[0]

        f = ca.Function('f', [x, u, kappa_p], [ca.vertcat(ey_dot, epsi_dot, v_dot)])

        U = ca.SX.sym('U', nu, self.N)
        X = ca.SX.sym('X', nx, self.N + 1)

        # P = [ey0, epsi0, v0, kappa, u_prev_accel, u_prev_omega, velocidad_lineal]
        P = ca.SX.sym('P', nx + 1 + nu + 1)

        cost = 0
        X[:, 0] = P[0:3]

        for k in range(self.N):
            st = X[:, k]
            con = U[:, k]

            # 1. Error lateral y angular
            cost += self.Q_ey * st[0]**2
            cost += self.Q_epsi * st[1]**2

            # 2. Velocidad adaptativa
            v_ref_dinamica = self.v_ref * P[6] 
            v_target = v_ref_dinamica / (1.0 + self.k_curve * ca.fabs(P[3]))
            cost += self.W_v_follow * (st[2] - v_target)**2

            # 3. Magnitud de control
            cost += self.R_accel * con[0]**2
            cost += self.R_omega * con[1]**2

            # 4. Penalización de jerk
            if k == 0:
                delta_u = con - P[4:6]
            else:
                delta_u = con - U[:, k-1]

            cost += self.S_accel * delta_u[0]**2
            cost += self.S_omega * delta_u[1]**2

            # 5. Integración RK4
            k1 = f(st, con, P[3])
            k2 = f(st + self.dt/2*k1, con, P[3])
            k3 = f(st + self.dt/2*k2, con, P[3])
            k4 = f(st + self.dt*k3, con, P[3])

            X[:, k+1] = st + self.dt/6*(k1 + 2*k2 + 2*k3 + k4)

        # Costo terminal
        st_terminal = X[:, self.N]
        cost += self.Q_terminal * (st_terminal[0]**2 + st_terminal[1]**2)

        variables = ca.reshape(U, -1, 1)
        nlp = {'x': variables, 'f': cost, 'g': ca.vertcat(), 'p': P}

        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 50
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

        # Guardamos último control aplicado
        self.u_prev = np.zeros(2)

    # ============================================================
    # CONTROL LOOP
    # ============================================================
    def control_loop(self):
        if self.emergency or not self.active_vision:
            self.publish_cmd(0.0, 0.0)
            return
        
        # 2. Prioridad: Control Manual
        if self.is_manual:
            # Dejamos pasar la señal del joystick directamente
            self.publish_cmd(self.manual_twist.linear.x, self.manual_twist.angular.z)
            return
        
        # 3. PARADA SEGURA: Forzamos alpha_vision a 0 para que el NMPC frene suavemente
        current_alpha = self.alpha_vision
        if self.safe_stop:
            current_alpha = 0.0

        # Parámetros del solver
        p = np.concatenate([
            self.state,
            [self.kappa],
            self.u_prev,
            [current_alpha] #para velocidad lineal
        ])

        # Warm start simple
        u0 = np.tile(self.u_prev, self.N)

        # Límites físicos
        accel_max = self.get_parameter('max_lin_accel').value      # Aceleración suave
        decel_max = self.get_parameter('max_lin_decel').value       # Frenado fuerte
        w_max = self.get_parameter('w_max').value

        lbx = np.array([-decel_max, -w_max] * self.N)
        ubx = np.array([ accel_max,  w_max] * self.N)

        try:
            res = self.solver(x0=u0, p=p, lbx=lbx, ubx=ubx)
            u_opt = res['x'].full().flatten()

            accel_cmd = float(u_opt[0])
            omega_cmd = float(u_opt[1])

            # Guardamos control aplicado
            self.u_prev = np.array([accel_cmd, omega_cmd])

        except:
            accel_cmd = 0.0
            omega_cmd = self.u_prev[1]  # mantener giro anterior

        v_final = self.state[2] + accel_cmd * self.dt

        self.publish_cmd(v_final, omega_cmd)

    # ============================================================

    def lane_cb(self, msg):
        self.state[0] = -msg.d_lat
        self.state[1] = -msg.yaw
        self.kappa = msg.curvature

    def odom_cb(self, msg):
        self.state[2] = msg.twist.twist.linear.x

    def emergency_cb(self, msg):
        self.emergency = msg.data
    
    def safe_stop_cb(self, msg):
        self.safe_stop = msg.data

    def active_vision_cb(self, msg):
        self.active_vision = msg.data
    
    def global_status_cb(self, msg):
        # msg.level es un byte: 0=OK, 1=WARN, 2=ERROR
        self.global_level = ord(msg.level) if isinstance(msg.level, bytes) else msg.level
        
        if self.global_level == 1: # WARNING
            # Si el diagnóstico pide precaución (0.5), pero la visión detecta algo más crítico (ej. 0.2)
            # nos quedamos con el valor más bajo (más seguro).
            self.alpha_vision = min(self.alpha_vision, 0.5)
            
        elif self.global_level == 2: # ERROR
            # Frenado total inmediato por diagnóstico
            self.alpha_vision = 0.0
    
    def manual_mode_cb(self, msg):
        previous_manual = self.is_manual
        self.is_manual = msg.data
        
        # Si acabamos de soltar el control manual (pasamos de True a False)
        if previous_manual and not self.is_manual:
            #self.get_logger().info("Retomando control NMPC...")
            # Limpiamos la memoria del solver para un arranque suave
            self.u_prev = np.array([0.0, self.manual_twist.angular.z])

    def manual_twist_cb(self, msg):
        self.manual_twist = msg

    def publish_cmd(self, v, w):
        if not rclpy.ok(): 
            return
    
        v_min = self.get_parameter('min_v_start').value
        v_max = self.get_parameter('v_max').value
        v_threshold = self.get_parameter('max_v_start').value

        if 0.001 < v < v_min and self.alpha_vision != 0.0:
            v = v_min

        if v <= v_threshold:
            scale = max(0.3, v / v_threshold)
            w *= scale

        if self.state[2] == 0.0:
            w = 0.0

        msg = Twist()
        msg.linear.x = float(np.clip(v, 0.0, v_max))
        msg.angular.z = float(w)

        self.pub_cmd.publish(msg)

# ============================================================
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaneControllerNMPC()
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