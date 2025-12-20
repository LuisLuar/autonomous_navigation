#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_interfaces.msg import DetectionArray, Detection

import numpy as np
import time
import os

from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # ------------ Config TensorRT -------------
        self.model_path = "/home/raynel/autonomous_navigation/src/perception_stack/object_detection/models/yolo11m.engine"

        self.conf_thres = float(os.getenv("YOLO_CONF", 0.25))
        self.iou_thres = float(os.getenv("YOLO_IOU", 0.45))
        self.device = "0"
        self.imgsz = 640

        # ------------ Load model -------------
        try:
            self.model = YOLO(self.model_path, task='detect')
            self.backend = "TensorRT"
        except Exception:
            onnx_path = self.model_path.replace('.engine', '.onnx')
            self.model = YOLO(onnx_path, task='detect')
            self.device = "cuda"
            self.backend = "ONNX"

        self.bridge = CvBridge()

        # ROS I/O
        self.sub_image = self.create_subscription(Image, '/camera/rgb/image_raw', self.callback_image, 1)

        self.pub_results = self.create_publisher(DetectionArray, '/detection/results', 1)

        self.class_names = self.model.names if hasattr(self.model, 'names') else {}

    def callback_image(self, msg: Image):
        t0 = time.time()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception:
            return

        # -------- INFERENCE --------
        try:
            results = self.model(
                cv_image,
                imgsz=self.imgsz,
                conf=self.conf_thres,
                iou=self.iou_thres,
                device=self.device,
                verbose=False
            )
        except Exception:
            return

        inference_time = (time.time() - t0) * 1000.0

        detection_array = DetectionArray()
        detection_array.header = msg.header
        detection_array.inference_time_ms = inference_time
        detection_array.backend = self.backend

        if len(results) > 0 and results[0].boxes is not None:
            for box in results[0].boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])

                det = Detection()
                det.class_id = cls
                det.class_name = self.class_names.get(cls, str(cls))
                det.confidence = conf
                det.x1, det.y1, det.x2, det.y2 = map(int, xyxy)
                det.center_x = (det.x1 + det.x2) / 2.0
                det.center_y = (det.y1 + det.y2) / 2.0

                detection_array.detections.append(det)

        self.pub_results.publish(detection_array)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
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


"""
  0: person --------------------------------------
  1: bicycle --------------------------------------
  2: car --------------------------------------
  3: motorcycle --------------------------------------
  4: airplane
  5: bus --------------------------------------
  6: train --------------------------------------
  7: truck --------------------------------------
  8: boat
  9: traffic light
  10: fire hydrant
  11: stop sign --------------------------------------?
  12: parking meter
  13: bench
  14: bird
  15: cat
  16: dog
  17: horse
  18: sheep
  19: cow
  20: elephant
  21: bear
  22: zebra
  23: giraffe
  24: backpack
  25: umbrella
  26: handbag
  27: tie
  28: suitcase
  29: frisbee
  30: skis
  31: snowboard
  32: sports ball
  33: kite
  34: baseball bat
  35: baseball glove
  36: skateboard
  37: surfboard
  38: tennis racket
  39: bottle
  40: wine glass
  41: cup
  42: fork
  43: knife
  44: spoon
  45: bowl
  46: banana
  47: apple
  48: sandwich
  49: orange
  50: broccoli
  51: carrot
  52: hot dog
  53: pizza
  54: donut
  55: cake
  56: chair
  57: couch
  58: potted plant
  59: bed
  60: dining table
  61: toilet
  62: tv
  63: laptop
  64: mouse
  65: remote
  66: keyboard
  67: cell phone
  68: microwave
  69: oven
  70: toaster
  71: sink
  72: refrigerator
  73: book
  74: clock
  75: vase
  76: scissors
  77: teddy bear
  78: hair drier
  79: toothbrush
"""