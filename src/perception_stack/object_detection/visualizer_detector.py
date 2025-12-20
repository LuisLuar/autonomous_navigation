#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_interfaces.msg import DetectionArray

import cv2
import numpy as np


class YoloVisualizerNode(Node):
    def __init__(self):
        super().__init__('yolo_visualizer')

        self.bridge = CvBridge()

        self.last_detections = None

        self.sub_image = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.callback_image, 1
        )

        self.sub_detections = self.create_subscription(
            DetectionArray, '/detection/results', self.callback_detections, 1
        )

        self.pub_annotated = self.create_publisher(
            Image, '/detection/annotated_image', 1
        )

    def callback_detections(self, msg: DetectionArray):
        self.last_detections = msg

    def callback_image(self, msg: Image):
        if self.last_detections is None:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception:
            return

        for det in self.last_detections.detections:
            color = self.class_color_hash(det.class_name)

            cv2.rectangle(
                img, (det.x1, det.y1), (det.x2, det.y2), color, 2
            )

            label = f"{det.class_name} {det.confidence:.2f}"
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )

            cv2.rectangle(
                img,
                (det.x1, det.y1 - th - 6),
                (det.x1 + tw + 6, det.y1),
                color,
                -1
            )

            cv2.putText(
                img,
                label,
                (det.x1 + 3, det.y1 - 3),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )

        out = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
        out.header = msg.header
        self.pub_annotated.publish(out)

    @staticmethod
    def class_color_hash(name: str):
        h = 0
        for ch in name:
            h = (h * 31 + ord(ch)) & 0xFFFFFFFF
        return (
            int((h >> 16) & 255),
            int((h >> 8) & 255),
            int(h & 255),
        )
    
    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisualizerNode()
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
