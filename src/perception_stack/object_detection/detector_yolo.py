#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from custom_interfaces.msg import DetectionArray, Detection
import numpy as np
import time
import os
from ultralytics import YOLO
import torch
import cv2


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # ------------ Config Modelo -------------
        self.model_path = "/home/raynel/autonomous_navigation/src/perception_stack/object_detection/models/yolo11s.pt"

        self.conf_thres = float(os.getenv("YOLO_CONF", 0.25))
        self.iou_thres = float(os.getenv("YOLO_IOU", 0.45))
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.imgsz = 640

        self.process_every_n_frames = self.declare_parameter(
            'process_every_n_frames', 1
        ).value

        self.get_logger().info(f"Dispositivo: {self.device}")
        self.get_logger().info(f"Procesando cada {self.process_every_n_frames} frames")

        # ------------ Tracking -------------
        self.use_tracking = True
        self.tracker_type = "bytetrack.yaml"
        self.next_id = 1

        # ------------ Load model -------------
        self.model = YOLO(self.model_path)
        self.class_names = self.model.names

        self.frame_count = 0

        # QoS Best Effort (IMPORTANTE para compressed)
        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.sub_image = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            best_effort_qos
        )

        self.pub_results = self.create_publisher(
            DetectionArray,
            '/detection/results',
            1
        )

    # =====================================================
    # CALLBACK DE IMAGEN (CompressedImage)
    # =====================================================
    def image_callback(self, msg: CompressedImage):

        self.frame_count += 1

        if self.frame_count % self.process_every_n_frames != 0:
            return

        t0 = time.time()

        # ---------- DESCOMPRESIÓN ----------
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().warn("Imagen decodificada es None")
                return

        except Exception as e:
            self.get_logger().error(f"Error decodificando imagen: {e}")
            return

        # ---------- INFERENCIA ----------
        try:
            if self.use_tracking:
                results = self.model.track(
                    source=cv_image,
                    imgsz=self.imgsz,
                    conf=self.conf_thres,
                    iou=self.iou_thres,
                    device=self.device,
                    persist=True,
                    tracker=self.tracker_type,
                    verbose=False,
                    half=self.device == "cuda"
                )
            else:
                results = self.model.predict(
                    source=cv_image,
                    imgsz=self.imgsz,
                    conf=self.conf_thres,
                    iou=self.iou_thres,
                    device=self.device,
                    verbose=False,
                    half=self.device == "cuda"
                )

        except Exception as e:
            self.get_logger().error(f"Error en inferencia: {e}")
            return

        inference_time = (time.time() - t0) * 1000.0

        detection_array = DetectionArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = "camera_link"
        detection_array.inference_time_ms = inference_time
        detection_array.backend = "PyTorch+Tracking"

        # ---------- PROCESAR RESULTADOS ----------
        if results and len(results) > 0:
            r = results[0]

            if r.boxes is not None and len(r.boxes) > 0:

                boxes = r.boxes
                has_ids = hasattr(boxes, "id") and boxes.id is not None

                for i in range(len(boxes)):

                    xyxy = boxes.xyxy[i].cpu().numpy()
                    det = Detection()

                    det.class_id = int(boxes.cls[i])
                    det.class_name = self.class_names.get(det.class_id, "unknown")
                    det.confidence = float(boxes.conf[i])

                    det.x1, det.y1, det.x2, det.y2 = map(int, xyxy)
                    det.center_x = float((xyxy[0] + xyxy[2]) / 2)
                    det.center_y = float((xyxy[1] + xyxy[3]) / 2)

                    if has_ids:
                        det.track_id = int(boxes.id[i])
                    else:
                        det.track_id = self.next_id
                        self.next_id += 1

                    detection_array.detections.append(det)

        self.pub_results.publish(detection_array)


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