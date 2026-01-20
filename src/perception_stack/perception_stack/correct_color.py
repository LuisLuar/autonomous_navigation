#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

class ImageRectifier(Node):
    def __init__(self):
        super().__init__('image_rectifier_node')

        self.declare_parameter("gain_l", 0.5)
        self.declare_parameter("gain_v", 1.1)
        self.declare_parameter("gain_s", 0.5)
        self.declare_parameter("clahe_clip", 10.2)
        self.declare_parameter("blur_k", 3)

        self.bridge = CvBridge()

        # Suscripción a la imagen original
        self.sub_img = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        # Publicador de imagen corregida
        self.pub_img = self.create_publisher(
            Image,
            '/camera/rgb/image_corrected',
            10
        )

        self.get_logger().info("Nodo ImageRectifier iniciado.")

    def image_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        # Recuperar parámetros
        gain_l = self.get_parameter("gain_l").value
        gain_v = self.get_parameter("gain_v").value
        gain_s = self.get_parameter("gain_s").value
        clahe_clip = self.get_parameter("clahe_clip").value
        blur_k = self.get_parameter("blur_k").value

        # ====================
        # CORRECCIÓN LAB-L
        # ====================
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)

        # CLAHE suave
        clahe = cv2.createCLAHE(
            clipLimit=clahe_clip,
            tileGridSize=(8,8)
        )
        l_corr = clahe.apply(l)
        l_corr = np.clip(l_corr.astype(np.float32) * gain_l, 0, 255).astype(np.uint8)

        lab_corr = cv2.merge([l_corr, a, b])
        img_lab = cv2.cvtColor(lab_corr, cv2.COLOR_LAB2BGR)

        # ====================
        # CORRECCIÓN HSV
        # ====================
        hsv = cv2.cvtColor(img_lab, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        v_corr = np.clip(v.astype(np.float32) * gain_v, 0, 255).astype(np.uint8)
        s_corr = np.clip(s.astype(np.float32) * gain_s, 0, 255).astype(np.uint8)

        hsv_corr = cv2.merge([h, s_corr, v_corr])
        img_corr = cv2.cvtColor(hsv_corr, cv2.COLOR_HSV2BGR)

        # ====================
        # BLUR SUAVE SI ES NECESARIO
        # ====================
        if blur_k >= 3 and blur_k % 2 == 1:
            img_corr = cv2.GaussianBlur(img_corr, (blur_k, blur_k), 0)

        # ====================
        # PUBLICAR
        # ====================
        try:
            msg_out = self.bridge.cv2_to_imgmsg(img_corr, encoding="bgr8")
            msg_out.header = msg.header
            self.pub_img.publish(msg_out)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge publish error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageRectifier()
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
