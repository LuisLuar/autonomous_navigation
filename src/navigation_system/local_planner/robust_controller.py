#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry

from std_msgs.msg import Int32


class LaneControllerStep6(Node):

    def __init__(self):
        super().__init__('lane_controller_step6')

        # ===================== SUBS =====================
        self.create_subscription(Float32, '/omega/lane', self.lane_cb, 10)
        self.create_subscription(Float32, '/omega/lane_orientation', self.lane_orientation_cb, 10)
        self.create_subscription(Bool, '/active/lane', self.lane_active_cb, 10)

        self.create_subscription(Float32, '/lane/error_lateral', self.e_lat_cb, 10)
        self.create_subscription(Float32, '/lane/error_heading', self.e_head_cb, 10)
        self.create_subscription(Float32, '/lane/curvature', self.curvature_cb, 10)

        self.create_subscription(Odometry, '/odometry/local', self.odom_cb, 10)

        self.create_subscription(Bool, '/emergency', self.emergency_cb, 10)
        self.create_subscription(Bool, '/manual', self.manual_cb, 10)

        self.create_subscription(Bool, '/active/planner', self.planner_active_cb, 10)
        self.create_subscription(Bool, '/goal_reached', self.goal_reached_cb, 10)
        self.planner_active = False
        self.goal_reached = False

        # ===================== PUB ======================
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_mode = self.create_publisher(Int32, '/lane/controller_mode', 10)


        # =================== PARAMS =====================
        self.declare_parameter('v_max', 0.7)
        self.declare_parameter('w_max', 0.2)
        self.declare_parameter('max_lin_accel', 0.01)

        # FRAME (control estable)
        self.declare_parameter('Kp_lane', 0.5)
        self.declare_parameter('Kd_lane', 0.0012)
        self.declare_parameter('Kp_lane_orientation', 0.03)
        self.declare_parameter('curvature_gain', 10.0)

        # LANE (control geométrico)
        self.declare_parameter('lookahead', 2.0)

        self.declare_parameter('k_lat', 0.1) #Qué tan rápido vuelves al carril cuando estás desplazado
        self.declare_parameter('k_head', 0.08) #Qué tan agresivo alineas el robot con la dirección del carril
        self.declare_parameter('k_curv', 0.1) #Anticipación de curva
        self.declare_parameter('beta', 2.0) #Cuánto reduces velocidad lineal en curva

        self.declare_parameter('transition_time', 10.0)
        self.declare_parameter('base_time', 60.0)   # ⬅️ OBLIGATORIO

        # =================== STATE ======================
        self.mode = 'BASE'          # BASE → TRANSITION → LANE
        self.alpha = 0.0

        self.start_time = time.time()

        self.e_lat = None
        self.e_head = None
        self.curvature = 0.0

        self.omega_lane = 0.0
        self.omega_lane_orientation = 0.0
        self.active_lane = False

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.v_cmd = 0.0
        self.w_cmd = 0.0

        self.e_lane_prev = 0.0
        self.e_lane_dot = 0.0
        self.deriv_filter = 0.85

        self.emergency = False
        self.manual = False

        self.last_time = time.time()
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('LaneController Step6 FINAL READY')

    # ================= CALLBACKS =================
    def lane_cb(self, msg): self.omega_lane = msg.data
    def lane_orientation_cb(self, msg): self.omega_lane_orientation = msg.data
    def lane_active_cb(self, msg): self.active_lane = msg.data

    def e_lat_cb(self, msg): self.e_lat = msg.data
    def e_head_cb(self, msg): self.e_head = msg.data
    def curvature_cb(self, msg): self.curvature = msg.data

    def manual_cb(self, msg): self.manual = msg.data
    def emergency_cb(self, msg): self.emergency = msg.data

    def odom_cb(self, msg):
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

    def planner_active_cb(self, msg): self.planner_active = msg.data
    def goal_reached_cb(self, msg): self.goal_reached = msg.data
    
    def publish_mode(self):
        msg = Int32()
        if self.mode == 'BASE':
            msg.data = 1
        elif self.mode == 'TRANSITION':
            msg.data = 2
        elif self.mode == 'LANE':
            msg.data = 3
        else:
            msg.data = 0
        self.pub_mode.publish(msg)


    # ================= CONTROL LOOP =================
    def control_loop(self):
        now = time.time()
        dt = max(1e-3, now - self.last_time)
        self.last_time = now

        # ---------- GOAL REACHED ----------
        """if self.goal_reached or not self.planner_active:
            self.reset_controller()
            self.publish_zero()
            return """

        # ---------- MANUAL ----------
        if self.manual:
            return

        # ---------- EMERGENCY ----------
        if self.emergency:
            self.reset_controller()
            self.publish_zero()
            return

        # ---------- BASE MODE (OBLIGATORIO) ----------
        base_time = self.get_parameter('base_time').value
        if (now - self.start_time) < base_time:
            #self.get_logger().info(f"⏳ BASE MODE ACTIVE: {now - self.start_time:.1f}s")
            self.mode = 'BASE'
            v, w = self.frame_control(dt)
            self.publish_cmd(v, w)
            self.publish_mode()
            return

        # ---------- STATE MACHINE ----------
        if self.mode == 'BASE':
            v, w = self.frame_control(dt)

            # VALIDACIÓN REAL
            """if (
                self.active_lane and
                self.e_lat is not None and
                abs(self.e_lat) < 0.5 and
                self.e_head is not None and
                abs(self.e_head) < math.radians(15) and
                abs(w) < 0.1
            ):
                self.mode = 'TRANSITION'
                self.alpha = 0.0

        elif self.mode == 'TRANSITION':
            T = self.get_parameter('transition_time').value
            self.alpha = min(1.0, self.alpha + dt / T)

            v_f, w_f = self.frame_control(dt)
            v_l, w_l = self.lane_control(dt)

            v = (1 - self.alpha) * v_f + self.alpha * v_l
            w = (1 - self.alpha) * w_f + self.alpha * w_l

            # SEGURIDAD
            if abs(w) >= self.get_parameter('w_max').value:
                self.mode = 'BASE'
                self.alpha = 0.0

            if self.alpha >= 1.0:
                self.mode = 'LANE'

        else:  # LANE
            v, w = self.lane_control(dt)"""

        self.publish_cmd(v, w)
        self.publish_mode()


    # ================= FRAME CONTROL (ESTABLE) =================
    def frame_control(self, dt):
        # creep si no hay carril
        #if not self.active_lane:
        #    return v, 0.0

        Kp0 = self.get_parameter('Kp_lane').value
        Kd0 = self.get_parameter('Kd_lane').value
        Ko0 = self.get_parameter('Kp_lane_orientation').value

        e = self.omega_lane
        e_dot_raw = (e - self.e_lane_prev) / dt
        self.e_lane_dot = (
            self.deriv_filter * self.e_lane_dot +
            (1 - self.deriv_filter) * e_dot_raw
        )
        self.e_lane_prev = e

        Kp = Kp0 / (1.0 + 0.65*self.linear_velocity) 
        Kd = Kd0 * 0.15*self.linear_velocity
        Ko = Ko0 / (1.0 + 0.65*self.linear_velocity)

        w = Kp * e + Kd * self.e_lane_dot + Ko * self.omega_lane_orientation
        w = np.clip(w, -self.get_parameter('w_max').value,
                        self.get_parameter('w_max').value)

        v_des = self.get_parameter('v_max').value
        beta = self.get_parameter('curvature_gain').value
        v_des /= (1 + beta * abs(w))

        v = self.limit_accel(v_des, dt)
        return v, w

    # ================= LANE CONTROL (GEOMÉTRICO) =================
    def lane_control(self, dt):
        if self.e_lat is None or self.e_head is None:
            return self.limit_accel(0.2, dt), 0.0

        # Parámetros
        k_lat  = self.get_parameter('k_lat').value
        k_head = self.get_parameter('k_head').value
        k_curv = self.get_parameter('k_curv').value
        L      = self.get_parameter('lookahead').value

        # Velocidad de referencia (NO la medida)
        v_ref = max(self.v_cmd, 0.05)

        # --- Términos del control ---
        w_heading = k_head * v_ref * self.e_head / L

        w_lat = k_lat * v_ref * self.e_lat / (L*L + self.e_lat*self.e_lat)

        w_curv = k_curv * v_ref * self.curvature

        w = w_heading + w_lat + w_curv

        # Saturación física
        w = np.clip(
            w,
            -self.get_parameter('w_max').value,
            self.get_parameter('w_max').value
        )

        # Velocidad lineal adaptativa a curvatura
        v_max = self.get_parameter('v_max').value
        beta  = self.get_parameter('beta').value
        v_des = v_max / (1 + beta * abs(self.curvature))

        v = self.limit_accel(v_des, dt)
        return v, w


    # ================= UTIL =================
    def limit_accel(self, v_des, dt):
        a = self.get_parameter('max_lin_accel').value
        dv = np.clip(v_des - self.v_cmd, -a * dt, a * dt)
        self.v_cmd += dv
        return self.v_cmd

    def publish_cmd(self, v, w):
        self.v_cmd = v
        self.w_cmd = w
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

    def publish_zero(self):
        self.pub_cmd.publish(Twist())

    def reset_controller(self):
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.alpha = 0.0
        self.mode = 'BASE'
        self.start_time = time.time()
        self.e_lane_prev = 0.0
        self.e_lane_dot = 0.0

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = LaneControllerStep6()
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
