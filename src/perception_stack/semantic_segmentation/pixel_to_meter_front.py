#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import json
import math

from custom_interfaces.msg import PixelPoint
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener

class PixelToMeterTransform(Node):

    def __init__(self):
        super().__init__('pixel_to_meter_transform_front')

        # =================== CARGA DE CALIBRACIÓN ===================
        calibration_path = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration_front.json"
        try:
            with open(calibration_path, 'r') as f:
                calib = json.load(f)

            self.h = float(calib['camera_height'])
            self.pitch = float(calib['camera_pitch'])
            self.fx = float(calib['intrinsics']['fx'])
            self.fy = float(calib['intrinsics']['fy'])
            self.cx = float(calib['intrinsics']['cx'])
            self.cy = float(calib['intrinsics']['cy'])

        except Exception as e:
            rclpy.shutdown()
            return

        # =================== PARÁMETROS ===================
        self.declare_parameter("min_distance", 0.5)
        self.declare_parameter("max_distance", 12.0)

        self.declare_parameter("camera_offset_x", 0.31)
        self.declare_parameter("camera_offset_y", 0.0)

        self.min_distance = self.get_parameter("min_distance").value
        self.max_distance = self.get_parameter("max_distance").value
        self.cam_x = self.get_parameter("camera_offset_x").value
        self.cam_y = self.get_parameter("camera_offset_y").value

        # ========== TRANSFORMADA A ODOM =============================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # =================== PUBLICADORES ===================
        self.pub_lane_candidates   = self.create_publisher(PointCloud2, '/lane/meter_candidates_front', 10)
       
        # =================== SUBSCRIPCIONES ===================
        self.create_subscription(
            PixelPoint, '/lane/pixel_candidates',
            lambda msg: self.process_and_publish(msg, self.pub_lane_candidates), 10)

    # =================== IPM EXACTA ===================
    def pixel_to_meters(self, u, v):
        # Pixel → rayo en cámara
        x = (u - self.cx) / self.fx
        y = -(v - self.cy) / self.fy
        ray = np.array([x, y, 1.0])

        # Rotación por pitch (MISMA que calibrador)
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)

        R = np.array([
            [1,  0,   0],
            [0,  cp,  sp],
            [0, -sp,  cp]
        ])

        ray_w = R @ ray

        # Intersección con suelo Y = 0
        if ray_w[1] >= 0:
            return None

        t = self.h / -ray_w[1]

        Xc = ray_w[0] * t   # lateral (derecha +)
        Zc = ray_w[2] * t   # adelante

        return Xc, Zc

    # =================== PIPELINE ===================
    def process_and_publish(self, msg: PixelPoint, publisher):

        if not msg.is_valid or len(msg.x_coordinates) == 0:
            return

        points = []

        for u, v in zip(msg.x_coordinates, msg.y_coordinates):
            res = self.pixel_to_meters(u, v)
            if res is None:
                continue

            Xc, Zc = res                

            if not (self.min_distance <= Zc <= self.max_distance):
                continue

            # Cámara → base_footprint
            x_base = Zc + self.cam_x
            y_base = -Xc + self.cam_y

            points.append((x_base, y_base, 0.0))

        if not points:
            return

        # === Publicar en base_footprint ===
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_footprint"

        cloud_base = point_cloud2.create_cloud_xyz32(header, points)
        publisher.publish(cloud_base)

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = PixelToMeterTransform()
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
