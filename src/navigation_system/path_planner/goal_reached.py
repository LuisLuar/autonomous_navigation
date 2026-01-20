#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool
import math
from collections import deque


class GoalMonitor(Node):
    def __init__(self):
        super().__init__('goal_monitor')

        # ------------------------
        # Subscripciones
        # ------------------------
        self.sub_path = self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            10
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odom_callback,
            20
        )

        # ------------------------
        # Publicadores
        # ------------------------
        self.pub_active_planner = self.create_publisher(
            Bool,
            '/active/planner',
            10
        )

        self.pub_goal_reached = self.create_publisher(
            Bool,
            '/goal_reached',
            10
        )

        # ------------------------
        # Estado interno
        # ------------------------
        self.goal_point = None
        self.path_available = False
        self.goal_reached = False

        


        self.distance_history = deque(maxlen=20)

        # ------------------------
        # Parámetros
        # ------------------------
        self.goal_tolerance = 2.0       # metros
        self.goal_min_radius = 3.0      # radio para habilitar mínimo local
        self.min_samples = 20            # historial mínimo para detectar aumento
        self.increase_count_required = 4

        self.publish_timer = self.create_timer(
            0.5,  # 500 ms
            self.timer_publish_states
        )


        self.get_logger().info('Goal Monitor iniciado')

    # ---------------------------------------------------
    # Callbacks
    # ---------------------------------------------------

    def path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            return

        last_pose = msg.poses[-1].pose.position
        self.goal_point = (last_pose.x, last_pose.y)

        self.path_available = True
        self.goal_reached = False
        self.distance_history.clear()

        self.publish_active_planner(True)
        self.publish_goal_reached(False)

        self.get_logger().info(
            f'Nuevo path recibido. Goal: x={last_pose.x:.2f}, y={last_pose.y:.2f}'
        )

    def odom_callback(self, msg: Odometry):
        if not self.path_available or self.goal_point is None:
            return

        if self.goal_reached:
            return

        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y

        gx, gy = self.goal_point
        dist = math.hypot(rx - gx, ry - gy)

        self.distance_history.append(dist)

        # ------------------------
        # Criterio 1: tolerancia directa
        # ------------------------
        if dist < self.goal_tolerance:
            self.set_goal_reached('Tolerancia alcanzada')
            return

        # ------------------------
        # Criterio 2: mínimo local ROBUSTO
        # ------------------------
        if len(self.distance_history) < self.min_samples:
            return

        min_dist = min(self.distance_history)

        # ❗ solo habilitar mínimo si ya estuvo cerca del goal
        if min_dist > self.goal_min_radius:
            return

        # verificar aumento sostenido
        increases = 0
        hist = list(self.distance_history)

        for i in range(len(hist) - 1):
            if hist[i + 1] > hist[i]:
                increases += 1
            else:
                increases = 0

            if increases >= self.increase_count_required:
                self.set_goal_reached('Mínimo local válido (alejándose del goal)')
                return


    # ---------------------------------------------------
    # Métodos auxiliares
    # ---------------------------------------------------

    def set_goal_reached(self, reason: str):
        self.goal_reached = True
        self.path_available = False

        self.publish_goal_reached(True)
        self.publish_active_planner(False)

        self.get_logger().info(f'GOAL ALCANZADO → {reason}')

    def publish_active_planner(self, state: bool):
        msg = Bool()
        msg.data = state
        self.pub_active_planner.publish(msg)

    def publish_goal_reached(self, state: bool):
        msg = Bool()
        msg.data = state
        self.pub_goal_reached.publish(msg)
    
    def timer_publish_states(self):
        # active planner = existe path activo y no se ha llegado
        active = self.path_available and not self.goal_reached

        msg_active = Bool()
        msg_active.data = active
        self.pub_active_planner.publish(msg_active)

        msg_goal = Bool()
        msg_goal.data = self.goal_reached
        self.pub_goal_reached.publish(msg_goal)


    def destroy_node(self):
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = GoalMonitor()
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
