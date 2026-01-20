#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import numpy as np
import math
import time
from collections import deque
from nav_msgs.msg import Odometry
from collections import deque
import statistics

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')

        # ================= SUBSCRIBERS =================
        self.create_subscription(Bool, '/emergency', self.emergency_cb, 10)

        self.create_subscription(Float32, '/omega/lane', self.lane_cb, 10) #Red de segmentacion
        self.create_subscription(Float32, '/omega/lane_orientation', self.lane_orientation_cb, 10) #Red de segmentacion
        self.create_subscription(Bool, '/active/lane', self.lane_active_cb, 10)

        self.create_subscription(Float32,'/omega/lidar',self.lidar_omega_cb,10)
        self.create_subscription(Bool,'/active/lidar_lateral',self.lidar_lateral_active_cb,10)

        self.create_subscription(Float32, '/alpha/lidar', self.lidar_alpha_cb, 10)
        self.create_subscription(Bool, '/active/lidar_front', self.lidar_front_active_cb, 10)
        
        self.create_subscription(Float32, '/alpha/vision', self.vision_alpha_cb, 10)
        self.create_subscription(Bool, '/active/vision', self.vision_active_cb, 10)

        self.create_subscription(Float32,'/alpha/osm_obstacles',self.osm_obstacles_alpha_cb,10)
        self.create_subscription(Bool,'/active/osm_obstacles',self.osm_obstacles_active_cb,10)

        self.create_subscription(Float32,'/alpha/osm',self.osm_alpha_cb,10)
        self.create_subscription(Bool,'/active/osm',self.osm_active_cb,10)
        self.create_subscription(Bool, '/goal_reached', self.goal_reached_cb, 10)

        self.create_subscription(Bool, '/manual', self.manual_cb, 10)

        self.create_subscription(
            Odometry, '/odometry/local', self.odom_cb, 10
        )

        # ================= PUBLISHER =================
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)


        # ================= PARAMETERS =================
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 0.20)

        self.declare_parameter('max_linear_accel', 0.01)
        self.declare_parameter('max_angular_accel', 0.35)

        self.declare_parameter('Kp_lane', 0.5) #0.35
        self.declare_parameter('Kd_lane', 0.0012)
        self.declare_parameter('Kp_lane_orientation', 0.045) #0.35
        self.declare_parameter('Kp_planner', 0.01)
        self.declare_parameter('Kp_lidar_lateral', 0.1)
        self.declare_parameter('control_rate', 20.0)

        self.declare_parameter('curvature_gain', 10.0) #5


        self.declare_parameter('max_wait_time',2)

        # ================= STATE =================
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.safety_time = 0.0

        self.last_time = time.time()

        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self.control_loop)
        self.pp_active = False

        # ===== ORIENTACION =====
        self.yaw = 0.0
        self.psi_ref = None      # referencia interna
        self.last_yaw_time = None


        # ====== INICIALIZACION COMPLETA ======
        self.emergency = False

        self.goal_reached = False

        self.omega_lane = 0.0
        self.omega_lane_orientation = 0.0
        self.active_lane = False

        self.omega_lidar = 0.0
        self.active_lidar_lateral = False

        self.alpha_vision = 0.0
        self.active_vision = False

        self.alpha_lidar = 0.0
        self.active_lidar_front = False

        self.alpha_osm = 0.0
        self.active_osm = False

        self.alpha_osm_obstacles = 0.0
        self.active_osm_obstacles = False

        self.e_lane_prev = 0.0
        self.e_lane_dot = 0.0
        self.deriv_filter = 0.7

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.manual = False
        self.e_history = deque(maxlen=200)  # ~10s a 20Hz

        # Crear buffer para 100 números
        self.angular_trend = deque(maxlen=250) #5s a 20Hz

        #self.get_logger().info("Velocity Controller READY (with emergency stop)")


    # =============== ODOMETRÍA ====================    
    def odom_cb(self, msg: Odometry):
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z
        # Aquí puedes usar las velocidades lineales y angulares según sea necesario
        # Por ejemplo, podrías almacenarlas en variables de estado si es necesario
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.angular_trend.append(self.yaw)
        pass
    # =====================================================
    def emergency_cb(self, msg):
        if msg.data and not self.emergency:
            self.v_cmd = 0.0
            self.w_cmd = 0.0
            self.e_lane_prev = 0.0
            self.e_lane_dot = 0.0
            self.safety_time = 0.0
            self.last_time = time.time()
        self.emergency = msg.data


    # ===============GOAL REACHED =================
    def goal_reached_cb(self, msg):
        self.goal_reached = msg.data

    # ============ CONTROL DE SEGMENTACION ==============    
    def lane_cb(self, msg):
        self.omega_lane = msg.data

    def lane_orientation_cb(self, msg):
        self.omega_lane_orientation = msg.data

    def lane_active_cb(self, msg):
        self.active_lane = msg.data

    # ============ CONTROL DE LIDAR LATERAL ==============
    def lidar_omega_cb(self, msg):
        self.omega_lidar = msg.data

    def lidar_lateral_active_cb(self, msg):
        self.active_lidar_lateral = msg.data

    # ============ CONTROL DE LIDAR FRONTAL ==============
    def lidar_alpha_cb(self, msg):
        self.alpha_lidar = msg.data

    def lidar_front_active_cb(self, msg):
        self.active_lidar_front = msg.data

    # ============ CONTROL DE VISION =====================
    def vision_alpha_cb(self, msg):
        self.alpha_vision = msg.data

    def vision_active_cb(self, msg):
        self.active_vision = msg.data

    # =========== CONTROL DE OSM REACHED========================
    def osm_alpha_cb(self, msg):
        self.alpha_osm = msg.data   
    
    def osm_active_cb(self, msg):
        self.active_osm = msg.data

    # =========== CONTROL DE OSM OBSTACLES========================
    def osm_obstacles_alpha_cb(self, msg):
        self.alpha_osm_obstacles = msg.data   
    
    def osm_obstacles_active_cb(self, msg):
        self.active_osm_obstacles = msg.data

    # =============== HABILITACION MODO MANUAL ====================
    def manual_cb(self, msg):
        self.manual = msg.data

    # =====================================================
    def control_loop(self):

        # ================= MODO MANUAL =====================
        if self.manual:
            return
        # ================= EMERGENCY STOP =================
        if self.emergency:
            self.v_cmd = 0.0
            self.w_cmd = 0.0

            cmd = Twist()  # todo en cero
            self.pub_cmd.publish(cmd)
            return

        now = time.time()
        dt = max(0.001, now - self.last_time)
        self.last_time = now

        # ================= GOAL REACHED =================
        if self.goal_reached:
            self.v_cmd = 0.0
            self.w_cmd = 0.0

            cmd = Twist()
            self.pub_cmd.publish(cmd)
            return
        
        # ===== inicializar referencia =====
        # Convertir a lista para cálculos
        angular = list(self.angular_trend)
        if len(angular) == 0:
            prom_angular = 0.0
        else:
            prom_angular = sum(angular) / len(angular)
        # 1. Media/Promedio
              
        # 3. Desviación estándar
        dev_angular = statistics.stdev(angular) if len(angular) > 1 else 0
        #self.get_logger().info(f"Angular Velocity Trend - Mean: {prom_angular:.4f}, Std Dev: {dev_angular:.4f}")
        error_angular = prom_angular - self.yaw 

        if dev_angular == 0.0:
            dev_angular = 100000000000.0
        
        
        # ================= Linear Velocity =================
        v_max = self.get_parameter('max_linear_speed').value
        v_des = v_max 
        """if self.active_osm:
            v_max = self.get_parameter('max_linear_speed').value
            v_des = v_max * self.alpha_osm
        else:
            self.v_cmd = 0.0
            self.w_cmd = 0.0
        
            cmd = Twist()
            self.pub_cmd.publish(cmd)
            return"""

        
        """if not self.active_lidar_front and not self.active_vision:
            v_des *= 0.5"""
        
        """if self.active_lidar_front and self.alpha_lidar == 0.0:
            self.v_cmd = 0.0
            self.w_cmd = 0.0

            cmd = Twist()
            self.pub_cmd.publish(cmd)
            return
        elif self.active_lidar_front:
            v_des *= self.alpha_lidar"""

        """if self.active_vision and self.alpha_vision == 0.0:
            self.v_cmd = 0.0
            self.w_cmd = 0.0

            cmd = Twist()
            self.pub_cmd.publish(cmd)
            return
        elif self.active_vision:    
            v_des *= self.alpha_vision"""

        if self.active_osm_obstacles and self.alpha_osm_obstacles == 0.0:
            self.v_cmd = 0.0
            self.w_cmd = 0.0

            cmd = Twist()
            self.pub_cmd.publish(cmd)
            return
        elif self.active_osm_obstacles:    
            v_des *= self.alpha_osm_obstacles
        
        condition = not self.active_lane and not self.active_lidar_lateral# and not self.active_droidcam_lane
        if condition:
            v_des = v_max * 0.5
            self.w_cmd = 0.0
            self.safety_time += dt

            if (self.safety_time > self.get_parameter('max_wait_time').value):
                self.v_cmd = 0.0
                self.w_cmd = 0.0

                cmd = Twist()
                self.pub_cmd.publish(cmd)
                return
        else:
            self.safety_time = 0

        # ================= Angular Velocity =================
        w_max = self.get_parameter('max_angular_speed').value
        v = max(abs(self.linear_velocity), 0.1)

        # --- Derivada ---
        e = self.omega_lane
        e_dot_raw = (e - self.e_lane_prev) / dt
        self.e_lane_dot = (
            self.deriv_filter * self.e_lane_dot +
            (1.0 - self.deriv_filter) * e_dot_raw
        )
        self.e_lane_prev = e

        # --- PD lateral ---
        w_lane = 0.0
        if self.active_lane:
           
            Kp0 = self.get_parameter('Kp_lane').value         
            Kd0 = self.get_parameter('Kd_lane').value
            Kp0_orientation = self.get_parameter('Kp_lane_orientation').value  
            
            Kp = Kp0 / (1.0 + 0.65*v) 
            Kd = Kd0 * 0.15*v

            Kp_orientation = Kp0_orientation / (1.0 + 0.65*v)

            w_lane = Kp * e + Kd * self.e_lane_dot + Kp0_orientation * self.omega_lane_orientation

            if self.v_cmd > 0.001:
                self.e_history.append(self.omega_lane)

                rms = self.compute_rms()
                #self.get_logger().info(
                #    f"v={self.v_cmd:.2f} m/s | RMS(e)={rms:.3f}"
                #)
      
        w_lidar_lateral = 0.0
        if self.active_lidar_lateral:
            Kp_lidar = self.get_parameter('Kp_lidar_lateral').value
            w_lidar_lateral = Kp_lidar * self.omega_lidar
        
        w_des = 0.0
        if not condition:
            w_des = w_lane #+ w_lidar_lateral #+ error_angular * (1/dev_angular) * 0.01
            w_des = np.clip(w_des, -w_max, w_max)
            self.w_cmd = w_des        

        # ==================== FINAL VELOCITY COMMAND =================
        beta = self.get_parameter('curvature_gain').value
        if(self.linear_velocity > 0.6):
            v_des = v_des / (1 + beta*abs(self.w_cmd))
            

        # ================= ACCELERATION LIMIT LINEAR =================
        a_v = self.get_parameter('max_linear_accel').value
        self.v_cmd += np.clip(v_des - self.v_cmd, -a_v * dt, a_v * dt)

        


        # ================= PUBLISH =================
        cmd = Twist()
        cmd.linear.x = self.v_cmd
        cmd.angular.z = self.w_cmd
        self.pub_cmd.publish(cmd)

    def compute_rms(self):
        if len(self.e_history) == 0:
            return 0.0
        return math.sqrt(np.mean(np.square(self.e_history)))

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = VelocityController()
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