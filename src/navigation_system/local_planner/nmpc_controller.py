#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import casadi as ca
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from custom_interfaces.msg import LaneModel

class LaneControllerNMPC(Node):

    def __init__(self):
        super().__init__('lane_controller_nmpc')

        # ================= NMPC =================
        # N: si reacciona tarde en curvas -> aumentar
        # dt: si vibra o es inestable -> bajar ligeramente
        # v_ref: velocidad crucero en rectas

        self.N = 10
        self.dt = 0.1
        self.v_ref = 1.5

        # ================= PESOS =================
        # Q_ey: subir si se abre en curvas / bajar si oscila lateralmente
        self.Q_ey = 10.0

        # Q_epsi: subir si entra torcido a curvas / bajar si gira demasiado agresivo
        self.Q_epsi = 12.5

        # R_accel: subir si acelera/frena brusco
        self.R_accel = 1.0

        # R_omega: subir si satura mucho ω / bajar si no gira suficiente
        self.R_omega = 2.5

        # S_accel: subir si hay tirones entre pasos (jerk longitudinal)
        self.S_accel = 2.0

        # S_omega: subir si vibra el giro / bajar si responde lento
        self.S_omega = 3.0

        # W_v_follow: subir si entra muy rápido en curvas
        self.W_v_follow = 5.0

        # Q_terminal: subir si corrige muy tarde dentro del horizonte
        self.Q_terminal = 15.0

        # Límites físicos
        self.declare_parameter('max_lin_accel', 0.05)
        self.declare_parameter('max_lin_decel', 0.5)
        self.declare_parameter('v_max', 1.7)
        self.declare_parameter('w_max', 0.6)
        self.declare_parameter('min_v_start', 0.1) #para evitar que el robot se quede atascado al iniciar desde parado
        self.declare_parameter('max_v_start', 0.2) #para evitar que el robot gire bruscamente al iniciar desde parado

        # Subs/Pubs
        self.create_subscription(LaneModel, '/lane/model_filtered', self.lane_cb, 10)
        self.create_subscription(Odometry, '/odometry/local', self.odom_cb, 10)
        self.create_subscription(Bool, '/emergency', self.emergency_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Estado [ey, epsi, v]
        self.state = np.zeros(3)
        self.kappa = 0.0
        self.emergency = False

        # Warm start memory
        self.u_prev = np.zeros(2)

        self.setup_nmpc()
        self.create_timer(0.033, self.control_loop)

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

        # P = [ey0, epsi0, v0, kappa, u_prev_accel, u_prev_omega]
        P = ca.SX.sym('P', nx + 1 + nu)

        cost = 0
        X[:, 0] = P[0:3]

        for k in range(self.N):
            st = X[:, k]
            con = U[:, k]

            # 1. Error lateral y angular
            cost += self.Q_ey * st[0]**2
            cost += self.Q_epsi * st[1]**2

            # 2. Velocidad adaptativa
            v_target = self.v_ref / (1.0 + 2.5 * ca.fabs(P[3]))
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
        if self.emergency:
            self.publish_cmd(0.0, 0.0)
            return

        # Parámetros del solver
        p = np.concatenate([
            self.state,
            [self.kappa],
            self.u_prev
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

    def publish_cmd(self, v, w):

        v_min = self.get_parameter('min_v_start').value
        v_max = self.get_parameter('v_max').value
        v_threshold = self.get_parameter('max_v_start').value

        if 0.001 < v < v_min:
            v = v_min

        if v <= v_threshold:
            scale = max(0.3, v / v_threshold)
            w *= scale

        msg = Twist()
        msg.linear.x = float(np.clip(v, 0.0, v_max))
        msg.angular.z = float(w)

        self.pub_cmd.publish(msg)

# ============================================================

def main():
    rclpy.init()
    node = LaneControllerNMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()