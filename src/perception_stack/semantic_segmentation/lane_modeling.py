#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from sklearn.linear_model import RANSACRegressor


class RightLaneFromBEV(Node):

    def __init__(self):
        super().__init__('right_lane_from_bev')

        self.bridge = CvBridge()

        # =========================
        # PARÁMETROS
        # =========================
        self.scale = 40.0        # px / m (MISMO que IPM)
        self.min_z = 1.5         # m
        self.max_z = 12.0        # m

        self.kernel_close = cv2.getStructuringElement(
            cv2.MORPH_RECT, (7, 21)
        )

        # =========================
        # ROS
        # =========================
        self.create_subscription(
            Image,
            '/ipm/bev',
            self.cb,
            1
        )

        self.pub = self.create_publisher(
            Path,
            '/lane/right_border',
            1
        )

        #self.get_logger().info("✅ Right lane extractor (RANSAC) iniciado")

    # ======================================================
    def cb(self, msg: Image):

        bev = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        H, W = bev.shape

        # --------------------------------------------------
        # 1. BINARIZAR SOLO CARRIL
        # --------------------------------------------------
        _, bin_img = cv2.threshold(
            bev, 200, 255, cv2.THRESH_BINARY
        )

        # --------------------------------------------------
        # 2. MORFOLOGÍA (cerrar huecos)
        # --------------------------------------------------
        bin_img = cv2.morphologyEx(
            bin_img, cv2.MORPH_CLOSE, self.kernel_close
        )

        # --------------------------------------------------
        # 3. MITAD DERECHA
        # --------------------------------------------------
        bin_img[:, :W // 2] = 0

        # --------------------------------------------------
        # 4. CONNECTED COMPONENTS
        # --------------------------------------------------
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            bin_img, connectivity=8
        )

        if num_labels < 2:
            return

        # Ignorar fondo (0), elegir blob más grande
        largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])

        if stats[largest, cv2.CC_STAT_AREA] < 200:
            return

        mask = (labels == largest).astype(np.uint8) * 255

        # --------------------------------------------------
        # 5. EXTRAER PUNTOS (BEV → metros)
        # --------------------------------------------------
        ys, xs = np.where(mask > 0)

        if len(xs) < 50:
            return

        z = (H - ys) / self.scale
        y = -(xs - W / 2) / self.scale

        valid = (z > self.min_z) & (z < self.max_z)
        z = z[valid]
        y = y[valid]

        if len(z) < 20:
            return

        # --------------------------------------------------
        # 6. RANSAC LINE (y = a*z + b)
        # --------------------------------------------------
        ransac = RANSACRegressor(
            min_samples=10,
            residual_threshold=0.25,
            random_state=0
        )

        try:
            ransac.fit(z.reshape(-1, 1), y)
        except:
            return

        a = ransac.estimator_.coef_[0]
        b = ransac.estimator_.intercept_

        # --------------------------------------------------
        # 7. GENERAR PATH
        # --------------------------------------------------
        path = Path()
        path.header = msg.header
        path.header.frame_id = 'base_footprint'

        z_vals = np.linspace(self.min_z, self.max_z, 20)

        for zi in z_vals:
            yi = a * zi + b

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = zi
            pose.pose.position.y = yi
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.pub.publish(path)

        """self.get_logger().info(
            f"✅ Right lane OK | pts={len(z)} | a={a:.3f}, b={b:.3f}",
            throttle_duration_sec=1.0
        )"""
    
    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = RightLaneFromBEV()
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
